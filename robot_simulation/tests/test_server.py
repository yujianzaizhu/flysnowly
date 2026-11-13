import base64
import json
import os
import shutil
import sqlite3
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import test_bag_player as bag_fixtures  # noqa: E402

BACKEND_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "backend"
)
sys.path.insert(0, BACKEND_DIR)

import map_editor  # noqa: E402
import server  # noqa: E402
import world  # noqa: E402


def _find_real_map_dir():
    """真实地图目录：优先仓库 data/map，其次旧的 home 路径。"""
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    candidates = (
        os.path.normpath(os.path.join(here, os.pardir, "data", "map", "map_1")),
        os.path.normpath(os.path.join(here, os.pardir, "data", "map")),
        "/home/fibocom/heyufei/data/map/map",
    )
    for path in candidates:
        if os.path.isdir(path) and (
            os.path.isfile(os.path.join(path, "default.pgm"))
            or os.path.isfile(os.path.join(path, "grid_map.pgm"))
        ):
            return path
    return None


REAL_MAP_DIR = _find_real_map_dir()


def square(cx, cy, half):
    return [
        (cx - half, cy - half),
        (cx + half, cy - half),
        (cx + half, cy + half),
        (cx - half, cy + half),
        (cx - half, cy - half),
    ]


class ServerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.httpd = server.create_server("127.0.0.1", 0)
        cls.port = cls.httpd.server_address[1]
        cls.thread = threading.Thread(target=cls.httpd.serve_forever, daemon=True)
        cls.thread.start()
        cls.base = "http://127.0.0.1:{}".format(cls.port)

    @classmethod
    def tearDownClass(cls):
        cls.httpd.shutdown()
        cls.httpd.simulator.stop()
        cls.httpd.server_close()
        cls.thread.join(2.0)

    def setUp(self):
        self.post("/api/map", {"map": 1})
        self.post("/api/reset", {})
        self.post("/api/speed", {"speed": 1})
        with self.httpd.simulator.lock:
            manager = self.httpd.simulator.manager
            manager.grid.build_free_area((manager.robot.pose[0], manager.robot.pose[1]))
            manager.grid.partition_zones()

    def get(self, path):
        with urllib.request.urlopen(self.base + path, timeout=3) as response:
            return response.status, response.read().decode("utf-8")

    def post(self, path, payload):
        request = urllib.request.Request(
            self.base + path,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=5) as response:
            return response.status, json.loads(response.read().decode("utf-8"))

    def test_state_shape(self):
        status, body = self.get("/api/state")
        self.assertEqual(status, 200)
        state = json.loads(body)
        for key in ("pose", "velocity", "battery", "vision", "task", "zones", "map", "ray_scan"):
            self.assertIn(key, state)
        self.assertEqual(
            len(state["map"]["occupancy"]),
            state["map"]["cols"] * state["map"]["rows"],
        )
        self.assertGreater(state["map"]["cols"] * state["map"]["rows"], 5000)
        self.assertEqual(len(state["ray_scan"]), 41)
        self.assertIn("speed", state)
        self.assertIn("config", state)
        for key in ("robot_radius", "vision_range", "lane_spacing", "ring_margin"):
            self.assertIn(key, state["config"])
        self.assertEqual(state["mowing_mode"], "A")
        for key in ("overall_coverage", "distance", "duration"):
            self.assertIn(key, state["task"])

    def test_map_scan_imports_all_maps_under_directory(self):
        """自动导入：扫描指定目录下所有含 .pgm 的地图并注册，编号复用不重复占号。"""
        root = tempfile.mkdtemp(prefix="mapscan-")
        try:
            for name in ("scan_a", "scan_b"):
                folder = os.path.join(root, name)
                os.makedirs(folder)
                body = bytearray()
                body += b"P5\n40 40\n255\n"
                for row in range(40):
                    for col in range(40):
                        edge = row in (0, 39) or col in (0, 39)
                        body.append(0 if edge else 254)
                with open(os.path.join(folder, "default.pgm"), "wb") as handle:
                    handle.write(bytes(body))
            status, data = self.post(
                "/api/map/scan", {"path": root, "select_first": False}
            )
            self.assertEqual(status, 200)
            self.assertEqual(len(data["imported"]), 2)
            self.assertEqual(data["failures"], [])
            names = [item["name"] for item in data["imported"]]
            self.assertEqual(len(names), len(set(names)), "重名地图应自动加目录后缀")
            ids = [item["id"] for item in data["imported"]]
            # 再扫一次：同一来源复用编号，不会重复占号
            _, again = self.post(
                "/api/map/scan", {"path": root, "select_first": False}
            )
            self.assertEqual([item["id"] for item in again["imported"]], ids)
            self.assertEqual([item["name"] for item in again["imported"]], names)
        finally:
            shutil.rmtree(root, ignore_errors=True)

    def test_internal_error_returns_json_instead_of_dropping(self):
        # 内部异常必须回一条 JSON（500），不能让浏览器看到"点了没反应"
        with self.httpd.simulator.lock:
            manager = self.httpd.manager
            original = manager.start_mowing

            def boom(*args, **kwargs):
                raise RuntimeError("模拟内部错误")

            manager.start_mowing = boom
        try:
            request = urllib.request.Request(
                self.base + "/api/task",
                data=json.dumps({"task": "mow", "zones": [1]}).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with self.assertRaises(urllib.error.HTTPError) as context:
                urllib.request.urlopen(request, timeout=5)
            self.assertEqual(context.exception.code, 500)
            body = json.loads(context.exception.read().decode("utf-8"))
            self.assertIn("模拟内部错误", body["error"])
        finally:
            with self.httpd.simulator.lock:
                manager.start_mowing = original

    def test_reports_endpoint_lists_folders(self):
        import mow_report

        previous = mow_report.OUTPUT_DIR
        root = tempfile.mkdtemp(prefix="mow_reports_api_")
        mow_report.OUTPUT_DIR = root
        try:
            folder = os.path.join(root, "2026-11-09_16-31-12")
            os.makedirs(folder)
            with open(os.path.join(folder, "report.html"), "w", encoding="utf-8") as handle:
                handle.write("<html><body>割草任务报告</body></html>")
            with open(os.path.join(folder, "coverage.png"), "wb") as handle:
                handle.write(b"\x89PNG\r\n\x1a\n")
            _, body = self.get("/api/reports")
            data = json.loads(body)
            self.assertEqual(data["directory"], root)
            self.assertEqual(len(data["reports"]), 1)
            entry = data["reports"][0]
            self.assertEqual(entry["folder"], "2026-11-09_16-31-12")
            self.assertEqual(data["latest"], "/reports/2026-11-09_16-31-12/report.html")
            status, html = self.get(data["latest"])
            self.assertEqual(status, 200)
            self.assertIn("割草任务报告", html)
            with urllib.request.urlopen(
                self.base + "/reports/2026-11-09_16-31-12/coverage.png", timeout=3
            ) as response:
                self.assertEqual(response.status, 200)
                self.assertEqual(response.headers.get("Content-Type"), "image/png")
                self.assertTrue(response.read().startswith(b"\x89PNG"))
        finally:
            mow_report.OUTPUT_DIR = previous
            shutil.rmtree(root, ignore_errors=True)

    def test_reports_latest_without_report_is_404(self):
        import mow_report

        previous = mow_report.OUTPUT_DIR
        empty = tempfile.mkdtemp(prefix="mow_report_empty_")
        mow_report.OUTPUT_DIR = empty
        try:
            with self.assertRaises(urllib.error.HTTPError) as context:
                self.get("/reports/latest")
            self.assertEqual(context.exception.code, 404)
            _, body = self.get("/api/reports")
            data = json.loads(body)
            self.assertEqual(data["reports"], [])
            self.assertIsNone(data["latest"])
        finally:
            mow_report.OUTPUT_DIR = previous
            shutil.rmtree(empty, ignore_errors=True)

    def test_start_mapping(self):
        _, state = self.post("/api/task", {"task": "map"})
        self.assertEqual(state["task"]["type"], "map")
        self.assertTrue(state["task"]["phase"].startswith("map"))

    def test_start_mowing_with_order(self):
        # 分区只按连通区域算，内置图都只有一块，所以这里用"两个区域 + 单行
        # 通道"的地图来验证多区域顺序。
        draft = map_editor.Draft("双区服务测试")
        draft.add_region(square(7.0, 7.0, 2.6))
        draft.add_region(square(22.0, 7.0, 2.6))
        draft.add_dock(5.2, 7.0, 0.45)
        draft.add_passage([(5.0, 7.0), (23.0, 7.0)])
        map_id = world.register_map(draft.to_definition("双区服务测试"))
        try:
            self.post("/api/map", {"map": map_id})
            with self.httpd.simulator.lock:
                manager = self.httpd.manager
                manager.grid.build_free_area(
                    (manager.robot.pose[0], manager.robot.pose[1])
                )
                zones = manager.grid.partition_zones()
            self.assertGreaterEqual(len(zones), 2)
            order = [zones[1]["id"], zones[0]["id"]]
            _, state = self.post(
                "/api/task", {"task": "mow", "zones": order, "mode": "B"}
            )
            self.assertEqual(state["task"]["type"], "mow")
            self.assertEqual(state["task"]["zone_order"], order)
            self.assertEqual(state["task"]["current_zone"], order[0])
            self.assertEqual(state["task"]["mode"], "B")
            self.assertEqual(state["mowing_mode"], "B")
        finally:
            self.post("/api/stop", {})
            if map_id in world.MAPS:
                world.delete_map(map_id)
            self.post("/api/map", {"map": 1})

    def test_mowing_without_map_is_rejected(self):
        self.post("/api/map", {"map": 2})
        request = urllib.request.Request(
            self.base + "/api/task",
            data=json.dumps({"task": "mow", "zones": [1]}).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with self.assertRaises(urllib.error.HTTPError) as context:
            urllib.request.urlopen(request, timeout=3)
        self.assertEqual(context.exception.code, 400)

    def test_manual_command_cancels_task(self):
        self.post("/api/task", {"task": "mow", "zones": [1]})
        _, state = self.post("/api/command", {"command": "forward"})
        self.assertIsNone(state["task"]["type"])
        time.sleep(0.4)
        _, body = self.get("/api/state")
        state = json.loads(body)
        self.assertGreater(state["velocity"]["linear"], 0.0)

    def test_speed_endpoint(self):
        for speed in (8, 16, 32):
            _, state = self.post("/api/speed", {"speed": speed})
            self.assertEqual(state["speed"], speed)
            _, body = self.get("/api/state")
            self.assertEqual(json.loads(body)["speed"], speed)
        self.post("/api/speed", {"speed": 1})

    def test_charge_and_resume_endpoints(self):
        self.post("/api/task", {"task": "mow", "zones": [1]})
        _, state = self.post("/api/charge", {})
        self.assertEqual(state["task"]["charge_reason"], "manual")
        self.assertIn(state["task"]["phase"], ("charge_go", "charging", "charge_return"))

    def test_map_edit_endpoint(self):
        status, state = self.post(
            "/api/map/edit", {"action": "add_static", "x": 5.0, "y": 4.0, "r": 0.4}
        )
        self.assertEqual(status, 200)
        edits = state["world"]["edits"]
        self.assertEqual(len(edits["static"]), 1)
        shape = state["world"]["obstacle_shapes"][-1]
        self.assertEqual(shape.get("edit"), "static")

        status, state = self.post(
            "/api/map/edit", {"action": "add_dynamic", "x": 4.0, "y": 4.0, "r": 0.3, "span": 1.8}
        )
        self.assertEqual(len(state["world"]["edits"]["dynamic"]), 1)
        self.assertEqual(len(state["world"]["dynamic"]), 1)

        status, state = self.post(
            "/api/map/edit",
            {"action": "add_no_go", "x0": 3.0, "y0": 2.5, "x1": 5.0, "y1": 4.0},
        )
        self.assertEqual(len(state["world"]["edits"]["no_go"]), 1)
        zone = state["world"]["edits"]["no_go"][0]
        self.assertEqual(zone["x1"], 5.0)
        grid = self.httpd.simulator.manager.grid
        col, row = grid.cell_of(4.0, 3.2)
        self.assertTrue(grid.cell_blocked_mow(col, row))
        outside_col, outside_row = grid.cell_of(6.5, 3.2)
        self.assertFalse(grid.cell_blocked_mow(outside_col, outside_row))

        static_id = state["world"]["edits"]["static"][0]["id"]
        status, state = self.post(
            "/api/map/edit", {"action": "delete", "kind": "static", "id": static_id}
        )
        self.assertEqual(state["world"]["edits"]["static"], [])

        status, state = self.post("/api/map/edit", {"action": "clear"})
        for kind in ("static", "dynamic", "no_go"):
            self.assertEqual(state["world"]["edits"][kind], [])
        self.assertEqual(state["world"]["dynamic"], [])

    def test_map_edit_requires_zones_and_idle_task(self):
        # 未建图时不允许编辑
        with self.httpd.simulator.lock:
            manager = self.httpd.simulator.manager
            manager.grid.zones = []
        request = urllib.request.Request(
            self.base + "/api/map/edit",
            data=json.dumps(
                {"action": "add_static", "x": 5.0, "y": 4.0, "r": 0.4}
            ).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with self.assertRaises(urllib.error.HTTPError) as ctx:
            urllib.request.urlopen(request, timeout=5)
        self.assertEqual(ctx.exception.code, 400)

        # 建图完成后可以编辑
        with self.httpd.simulator.lock:
            manager.grid.build_free_area((manager.robot.pose[0], manager.robot.pose[1]))
            manager.grid.partition_zones()
        status, state = self.post(
            "/api/map/edit", {"action": "add_static", "x": 5.0, "y": 4.0, "r": 0.4}
        )
        self.assertEqual(status, 200)
        self.assertEqual(len(state["world"]["edits"]["static"]), 1)

        # 任务进行中不允许编辑
        with self.httpd.simulator.lock:
            manager = self.httpd.simulator.manager
            manager.task = {"type": "mow", "zones": [1], "mode": "A"}
            manager.phase = "zone_stripes"
        request = urllib.request.Request(
            self.base + "/api/map/edit",
            data=json.dumps(
                {"action": "add_no_go", "x0": 3.0, "y0": 2.5, "x1": 5.0, "y1": 4.0}
            ).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with self.assertRaises(urllib.error.HTTPError) as ctx:
            urllib.request.urlopen(request, timeout=5)
        self.assertEqual(ctx.exception.code, 400)
        with self.httpd.simulator.lock:
            manager.task = None
            manager.phase = "idle"

    def test_dynamic_path_can_be_edited(self):
        status, state = self.post(
            "/api/map/edit", {"action": "add_dynamic", "x": 4.0, "y": 4.0, "r": 0.3, "span": 1.5}
        )
        item = state["world"]["edits"]["dynamic"][0]
        status, state = self.post(
            "/api/map/edit",
            {
                "action": "update_dynamic",
                "id": item["id"],
                "ax": 3.0,
                "ay": 3.0,
                "bx": 6.0,
                "by": 3.0,
            },
        )
        self.assertEqual(status, 200)
        updated = state["world"]["edits"]["dynamic"][0]
        self.assertEqual(updated["ax"], 3.0)
        self.assertEqual(updated["bx"], 6.0)
        self.post("/api/map/edit", {"action": "clear"})

    def test_map_switch(self):
        _, state = self.post("/api/map", {"map": 3})
        self.assertEqual(state["map_id"], 3)
        self.assertAlmostEqual(state["pose"]["x"], world.START["x"], places=2)
        self.assertAlmostEqual(state["pose"]["y"], world.START["y"], places=2)
        self.assertEqual(
            len(state["map"]["occupancy"]),
            state["map"]["cols"] * state["map"]["rows"],
        )

    @unittest.skipUnless(REAL_MAP_DIR, "真实地图目录不存在")
    def test_import_real_map_endpoint(self):
        _, state = self.post(
            "/api/map/import",
            {"path": REAL_MAP_DIR},
        )
        self.assertTrue(state["world"]["imported"])
        self.assertGreater(state["world"]["import_meta"]["free_area_m2"], 10.0)
        self.assertIn("实景", state["map_name"])
        self.assertEqual(
            len(state["map"]["occupancy"]),
            state["map"]["cols"] * state["map"]["rows"],
        )

    def test_upload_map_endpoint(self):
        width, height = 40, 30
        pixels = bytearray(width * height)
        for row in range(4, height - 4):
            for col in range(4, width - 4):
                pixels[row * width + col] = 255
        pgm = b"P5\n40 30\n255\n" + bytes(pixels)
        files = [
            {
                "path": "map/grid_map.pgm",
                "data": base64.b64encode(pgm).decode("ascii"),
            },
            {
                "path": "map/path/default.json",
                "data": base64.b64encode(b'{"resolution":"0.1"}').decode("ascii"),
            },
        ]
        _, state = self.post("/api/map/upload", {"files": files})
        self.assertTrue(state["world"]["imported"])
        self.assertAlmostEqual(state["world"]["import_meta"]["resolution"], 0.1)
        self.assertAlmostEqual(state["world"]["width"], 4.4, places=1)
        self.assertGreater(state["world"]["import_meta"]["free_area_m2"], 1.0)
        imported_id = state["map_id"]
        _, state = self.post("/api/map/delete", {"map": imported_id})
        self.assertEqual(state["map_id"], 1)
        self.assertNotIn(
            imported_id,
            [item["id"] for item in state["world"]["maps"]],
        )

    def test_invalid_payloads(self):
        for path, payload in (
            ("/api/command", {"command": "fly"}),
            ("/api/task", {"task": "unknown"}),
            ("/api/speed", {"speed": 3}),
            ("/api/map", {"map": 99}),
            ("/api/map/delete", {"map": 1}),
            ("/api/task", {"task": "mow", "zones": [1], "mode": "C"}),
            ("/api/config", {"robot_radius": 5.0}),
        ):
            request = urllib.request.Request(
                self.base + path,
                data=json.dumps(payload).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with self.assertRaises(urllib.error.HTTPError) as context:
                urllib.request.urlopen(request, timeout=3)
            self.assertEqual(context.exception.code, 400)

    def test_config_endpoint(self):
        _, state = self.post(
            "/api/config",
            {
                "robot_radius": 0.20,
                "vision_range": 1.2,
                "lane_spacing": 0.28,
                "mowing_mode": "B",
            },
        )
        self.assertAlmostEqual(state["config"]["robot_radius"], 0.20)
        self.assertAlmostEqual(state["config"]["vision_range"], 1.2)
        self.assertEqual(state["config"]["mowing_mode"], "B")
        self.assertAlmostEqual(world.ROBOT_RADIUS, 0.20)
        _, state = self.post("/api/config/reset", {})
        self.assertAlmostEqual(state["config"]["robot_radius"], 0.18)
        self.assertAlmostEqual(world.ROBOT_RADIUS, 0.18)

    def test_stop_and_resume_endpoints(self):
        self.post("/api/task", {"task": "mow", "zones": [1], "mode": "A"})
        time.sleep(0.3)
        _, state = self.post("/api/stop", {})
        self.assertEqual(state["task"]["phase"], "stopped")
        self.assertIsNotNone(state["task"]["type"])
        _, state = self.post("/api/resume", {})
        self.assertNotEqual(state["task"]["phase"], "stopped")

    def test_reset(self):
        self.post("/api/task", {"task": "map"})
        _, state = self.post("/api/reset", {})
        self.assertEqual(state["task"]["phase"], "idle")
        self.assertEqual(state["task"]["map_progress"], 0)
        self.assertAlmostEqual(state["pose"]["x"], world.START["x"], places=2)
        self.assertAlmostEqual(state["pose"]["y"], world.START["y"], places=2)

    def test_index_is_served(self):
        status, body = self.get("/")
        self.assertEqual(status, 200)
        self.assertIn("控制台", body)

    def test_bag_dir_import_and_image_frame(self):
        tmp = tempfile.mkdtemp(prefix="robot-bagdir-")
        group = os.path.join(tmp, "连续play")
        os.makedirs(group)
        bag_fixtures.build_bag(os.path.join(group, "a.bag"), with_images=True)
        try:
            _status, body = self.post("/api/bag/dir", {"path": tmp})
            keys = {item["key"] for item in body["bags"]}
            key = os.path.join("连续play", "a.bag")
            self.assertIn(key, keys)
            self.assertIn(tmp, body["roots"])

            _status, data = self.post("/api/bag/load", {"name": key})
            self.assertTrue(data["state"]["loaded"])
            self.assertGreater(data["state"]["image_frames"], 0)
            self.assertEqual(data["state"]["image_topic"], "perception_debug_image")
            self.assertIn("perception_debug_image", [t["name"] for t in data["topics"]])

            _status, frame = self.get("/api/bag/frame?t=0")
            image = json.loads(frame)["image"]
            self.assertTrue(image["image"].startswith("data:image/jpeg;base64,"))
            self.assertEqual(image["count"], 2)
        finally:
            self.post("/api/bag/control", {"command": "stop"})
            self.httpd.bag_queue = None
            shutil.rmtree(tmp, ignore_errors=True)

    def test_bag_playlist_plays_multiple_bags(self):
        tmp = tempfile.mkdtemp(prefix="robot-bagqueue-")
        group = os.path.join(tmp, "队列")
        os.makedirs(group)
        bag_fixtures.build_bag(os.path.join(group, "one.bag"))
        bag_fixtures.build_bag(os.path.join(group, "two.bag"))
        names = [os.path.join("队列", "one.bag"), os.path.join("队列", "two.bag")]
        try:
            self.post("/api/bag/dir", {"path": tmp})
            _status, body = self.post("/api/bag/playlist", {"names": names})
            self.assertTrue(body["queue"]["active"])
            self.assertEqual(body["queue"]["total"], 2)
            self.assertEqual(body["queue"]["names"], names)
            deadline = time.time() + 8.0
            state = {}
            while time.time() < deadline:
                state = json.loads(self.get("/api/bag")[1])["state"]
                if state.get("loaded") and state.get("playing"):
                    break
                time.sleep(0.2)
            self.assertTrue(state.get("loaded"))
            self.assertTrue(state.get("playing"))
            self.assertEqual(state.get("name"), "one.bag")
        finally:
            self.post("/api/bag/control", {"command": "stop"})
            self.httpd.bag_queue = None
            shutil.rmtree(tmp, ignore_errors=True)

    def test_bag_without_map_gets_open_field(self):
        tmp = tempfile.mkdtemp(prefix="robot-bagnomap-")
        path = os.path.join(tmp, "nomap.bag")
        bag_fixtures.build_bag(path)
        try:
            self.post("/api/bag/dir", {"path": tmp})
            _status, data = self.post("/api/bag/load", {"name": "nomap.bag"})
            self.assertTrue(data["state"]["loaded"])
            _status, body = self.get("/api/state")
            state = json.loads(body)
            definition = state["world"]
            self.assertIn("无地图", definition["name"])
            self.assertGreaterEqual(definition["width"], 24.0)
            self.assertEqual(state.get("obstacles") or [], [])
            self.assertAlmostEqual(
                state["pose"]["x"], definition["width"] / 2.0, delta=3.0
            )
        finally:
            self.post("/api/bag/control", {"command": "stop"})
            self.post("/api/map", {"map": 1})
            shutil.rmtree(tmp, ignore_errors=True)

    def test_continuous_play_keeps_position_across_bags(self):
        """同一批"无地图"日志连续播放：换包时机器人不该跳到别处。

        这批日志是分段录的，前后段的位姿首尾相接，但它们没有 whole_map，回放只能
        把轨迹居中摆到开阔场地里。以前每包各自居中 → 换包就跳一段；现在沿用上一包
        的换算，绝对位置保持连续。
        """
        tmp = tempfile.mkdtemp(prefix="robot-bagqueue-")
        first = os.path.join(tmp, "zmqbag_1_2026-08-29-06-20-18.bag")
        second = os.path.join(tmp, "zmqbag_2_2026-08-29-06-27-04.bag")
        bag_fixtures.build_bag(first, samples=20, origin=(0.0, 0.0))
        bag_fixtures.build_bag(second, samples=20, origin=(5.0, 3.0))
        queue = {
            "names": [first, second],
            "index": 0,
            "active": True,
            "use_log_map": True,
            "loading": False,
            "failed": [],
        }
        self.httpd.bag_queue = queue
        try:
            manager, _info, _path = server.load_bag(self.httpd, first, True)
            first_offset = manager.replay.offset
            tail = manager.replay.track[-1]
            manager, _info, _path = server.load_bag(self.httpd, second, True)
            self.assertAlmostEqual(manager.replay.offset[0], first_offset[0], places=4)
            self.assertAlmostEqual(manager.replay.offset[1], first_offset[1], places=4)
            head = manager.replay.track[0]
            # 位姿按日志坐标系平移：两包在日志里差 (5, 3)，仿真里也应差 (5, 3)
            self.assertAlmostEqual(head[1] - tail[1], 5.0, delta=0.6)
            self.assertAlmostEqual(head[2] - tail[2], 3.0, delta=0.6)
        finally:
            self.httpd.bag_queue = None
            self.post("/api/bag/control", {"command": "stop"})
            self.post("/api/map", {"map": 1})
            shutil.rmtree(tmp, ignore_errors=True)

    def test_continuous_play_clears_map_when_next_bag_has_no_map(self):
        """连播时下一包没有地图（机器删图重建）：清掉前一张地图，切空白场地。"""
        tmp = tempfile.mkdtemp(prefix="robot-bagclearmap-")
        with_map = os.path.join(tmp, "zmqbag_1_2026-09-15-07-04-14.bag")
        no_map = os.path.join(tmp, "zmqbag_2_2026-09-15-07-05-14.bag")
        bag_fixtures.build_map_frame_bag(with_map, samples=20)
        bag_fixtures.build_bag(no_map, samples=20)
        self.httpd.bag_queue = {
            "names": [with_map, no_map],
            "index": 0,
            "active": True,
            "use_log_map": True,
            "loading": False,
            "failed": [],
        }
        try:
            server.load_bag(self.httpd, with_map, True)
            epoch = world.MAP_EPOCH
            self.assertNotIn("无地图", world.MAP_NAME)
            manager, _info, _path = server.load_bag(self.httpd, no_map, True)
            # 前一张地图作废：空白场地 + 纪元 +1 + 历史点云清空
            self.assertIn("无地图", world.MAP_NAME)
            self.assertEqual(world.MAP_EPOCH, epoch + 1)
            self.assertEqual(getattr(self.httpd, "bag_cloud_history", []), [])
            # 机器人摆在空白场地中心附近
            first = manager.replay.track[0]
            self.assertAlmostEqual(first[1], world.WIDTH / 2.0, delta=4.0)
            self.assertAlmostEqual(first[2], world.HEIGHT / 2.0, delta=4.0)
        finally:
            self.httpd.bag_queue = None
            self.post("/api/bag/control", {"command": "stop"})
            self.post("/api/map", {"map": 1})
            shutil.rmtree(tmp, ignore_errors=True)

    def test_continuous_play_keeps_epoch_for_same_map(self):
        """同一张地图的相邻两包换算只差几毫米：不该判成"换了场地"（不能清轨迹/点云）。"""
        tmp = tempfile.mkdtemp(prefix="robot-bagsameepoch-")
        first = os.path.join(tmp, "zmqbag_1_2026-09-15-07-04-14.bag")
        second = os.path.join(tmp, "zmqbag_2_2026-09-15-07-05-14.bag")
        bag_fixtures.build_map_frame_bag(first, samples=20)
        bag_fixtures.build_map_frame_bag(second, samples=20, wall_offset_ns=10 ** 9)
        self.httpd.bag_queue = {
            "names": [first, second],
            "index": 0,
            "active": True,
            "use_log_map": True,
            "loading": False,
            "failed": [],
        }
        try:
            server.load_bag(self.httpd, first, True)
            epoch = world.MAP_EPOCH
            self.httpd.bag_cloud_history = [[1.0, 2.0]]
            server.load_bag(self.httpd, second, True)
            self.assertEqual(world.MAP_EPOCH, epoch)
            self.assertEqual(self.httpd.bag_cloud_history, [[1.0, 2.0]])
        finally:
            self.httpd.bag_queue = None
            self.httpd.bag_cloud_history = []
            self.post("/api/bag/control", {"command": "stop"})
            self.post("/api/map", {"map": 1})
            shutil.rmtree(tmp, ignore_errors=True)

    def test_continuous_play_imports_map_when_bag_has_map(self):
        """连播时后面那包带地图（建图完成）：把它的地图重新导进来。"""
        tmp = tempfile.mkdtemp(prefix="robot-bagmapback-")
        no_map = os.path.join(tmp, "zmqbag_1_2026-09-15-07-04-14.bag")
        with_map = os.path.join(tmp, "zmqbag_2_2026-09-15-07-05-14.bag")
        bag_fixtures.build_bag(no_map, samples=20)
        bag_fixtures.build_map_frame_bag(with_map, samples=20)
        self.httpd.bag_queue = {
            "names": [no_map, with_map],
            "index": 0,
            "active": True,
            "use_log_map": True,
            "loading": False,
            "failed": [],
        }
        try:
            server.load_bag(self.httpd, no_map, True)
            self.assertIn("无地图", world.MAP_NAME)
            server.load_bag(self.httpd, with_map, True)
            self.assertNotIn("无地图", world.MAP_NAME)
            self.assertGreater(len(world.POLYGON), 3)
        finally:
            self.httpd.bag_queue = None
            self.post("/api/bag/control", {"command": "stop"})
            self.post("/api/map", {"map": 1})
            shutil.rmtree(tmp, ignore_errors=True)

    def test_single_bag_load_recenters_track(self):
        """单包载入（不在连续播放里）仍把轨迹居中摆到场地：改动只针对连续播放。"""
        tmp = tempfile.mkdtemp(prefix="robot-bagsingle-")
        path = os.path.join(tmp, "nomap.bag")
        bag_fixtures.build_bag(path, samples=20, origin=(40.0, 40.0))
        try:
            server.load_bag(self.httpd, path, True)
            _status, body = self.get("/api/state")
            state = json.loads(body)
            definition = state["world"]
            self.assertAlmostEqual(
                state["pose"]["x"], definition["width"] / 2.0, delta=4.0
            )
            self.assertAlmostEqual(
                state["pose"]["y"], definition["height"] / 2.0, delta=4.0
            )
        finally:
            self.post("/api/bag/control", {"command": "stop"})
            self.post("/api/map", {"map": 1})
            shutil.rmtree(tmp, ignore_errors=True)

    def test_replay_hides_dock_when_bag_has_no_dock_pose(self):
        """日志里没有明确的桩位 → 回放时地图上不放桩；有桩位话题 → 按日志摆放并显示。"""
        tmp = tempfile.mkdtemp(prefix="robot-bagdock-")
        plain = os.path.join(tmp, "plain.bag")
        withdock = os.path.join(tmp, "withdock.bag")
        bag_fixtures.build_bag(plain)
        bag_fixtures.build_dock_bag(withdock, dock=(3.25, 2.5))
        try:
            self.post("/api/bag/dir", {"path": tmp})
            _status, data = self.post("/api/bag/load", {"name": "plain.bag"})
            self.assertFalse(data["state"]["dock_visible"])
            _status, raw = self.get("/api/state")
            self.assertFalse(json.loads(raw)["world"]["dock_visible"])

            _status, data = self.post("/api/bag/load", {"name": "withdock.bag"})
            self.assertTrue(data["state"]["dock_visible"])
            _status, raw = self.get("/api/state")
            state = json.loads(raw)
            self.assertTrue(state["world"]["dock_visible"])
            dock = state["world"]["dock"]
            # 桩位按日志坐标换算到仿真坐标：和日志里的 (3.25, 2.5) 对得上（+回放平移）
            replay = self.httpd.manager.replay
            scale = replay.scale if replay.transform else 1.0
            dx, dy = replay.map_offset if replay.transform else replay.offset
            self.assertAlmostEqual(dock["x"], 3.25 * scale + dx, places=2)
            self.assertAlmostEqual(dock["y"], 2.5 * scale + dy, places=2)

            # 复位后桩照常显示（回到仿真自己的场地）
            self.post("/api/bag/control", {"command": "stop"})
            self.post("/api/reset", {})
            _status, raw = self.get("/api/state")
            self.assertTrue(json.loads(raw)["world"]["dock_visible"])
        finally:
            self.post("/api/bag/control", {"command": "stop"})
            self.post("/api/reset", {})
            shutil.rmtree(tmp, ignore_errors=True)

    def test_bag_topics_expose_layers(self):
        tmp = tempfile.mkdtemp(prefix="robot-baglayer-")
        bag_fixtures.build_map_bag(os.path.join(tmp, "layer.bag"))
        try:
            self.post("/api/bag/dir", {"path": tmp})
            _status, data = self.post("/api/bag/load", {"name": "layer.bag"})
            layers = {topic["name"]: topic.get("layer") for topic in data["topics"]}
            self.assertEqual(layers.get("whole_map"), "map")
            self.assertEqual(layers.get("grid_costmap"), "costmap")
            self.assertEqual(layers.get("base_pose"), "pose")
            self.assertIsNone(layers.get("imu_data"))
            self.assertTrue(data["state"]["battery_locked"])
            self.assertIsNone(data["state"]["battery_topic"])
        finally:
            self.post("/api/bag/control", {"command": "stop"})
            self.post("/api/map", {"map": 1})
            shutil.rmtree(tmp, ignore_errors=True)

    def test_bag_playlist_requires_names(self):
        with self.assertRaises(urllib.error.HTTPError) as ctx:
            self.post("/api/bag/playlist", {"names": []})
        self.assertEqual(ctx.exception.code, 400)

    def _wait_queue(self, timeout=30.0):
        """等到连续播放队列跑完（active=False 且 index 走到队尾）。"""
        deadline = time.time() + timeout
        queue = {}
        while time.time() < deadline:
            queue = json.loads(self.get("/api/state")[1]).get("queue") or {}
            total = queue.get("total") or 0
            if total and not queue.get("active") and queue.get("index") >= total:
                return queue
            time.sleep(0.15)
        return queue

    def test_bag_playlist_skips_broken_bag(self):
        """一条日志读不出来只跳过它，整条连续播放要能跑完。"""
        tmp = tempfile.mkdtemp(prefix="robot-bagskip-")
        group = os.path.join(tmp, "跳过")
        os.makedirs(group)
        bag_fixtures.build_bag(os.path.join(group, "ok1.bag"), samples=8, dt=0.1)
        broken = os.path.join(group, "broken.bag")
        with open(broken, "wb") as handle:
            handle.write(b"not a sqlite database")
        bag_fixtures.build_bag(os.path.join(group, "ok2.bag"), samples=8, dt=0.1)
        names = [
            os.path.join("跳过", "ok1.bag"),
            os.path.join("跳过", "broken.bag"),
            os.path.join("跳过", "ok2.bag"),
        ]
        try:
            self.post("/api/bag/dir", {"path": tmp})
            self.post("/api/speed", {"speed": 32})
            _status, body = self.post("/api/bag/playlist", {"names": names})
            self.assertEqual(body["queue"]["total"], 3)
            queue = self._wait_queue()
            self.assertFalse(queue.get("active"))
            self.assertEqual(queue.get("index"), queue.get("total"))
            self.assertFalse(queue.get("loading"))
            failed = [os.path.basename(item) for item in queue.get("failed") or []]
            self.assertEqual(failed, ["broken.bag"])
        finally:
            self.post("/api/bag/control", {"command": "stop"})
            self.httpd.bag_queue = None
            self.post("/api/speed", {"speed": 1})
            shutil.rmtree(tmp, ignore_errors=True)

    def test_continuous_playback_keeps_cloud_history(self):
        """连续播放换包不清历史感知点云，「清理地图」才清。"""
        tmp = tempfile.mkdtemp(prefix="robot-bagcloud-")
        group = os.path.join(tmp, "点云")
        os.makedirs(group)
        bag_fixtures.build_map_frame_bag(
            os.path.join(group, "cloud_a.bag"), samples=120, dt=0.1, clouds=True
        )
        bag_fixtures.build_map_frame_bag(
            os.path.join(group, "cloud_b.bag"), samples=120, dt=0.1, clouds=True
        )
        names = [os.path.join("点云", "cloud_a.bag"), os.path.join("点云", "cloud_b.bag")]
        try:
            self.post("/api/bag/dir", {"path": tmp})
            self.post("/api/speed", {"speed": 32})
            self.post("/api/bag/playlist", {"names": names})
            self._wait_queue()
            cloud = json.loads(self.get("/api/bag/cloud?t=0")[1])["cloud"]
            self.assertIsNotNone(cloud)
            # 上一段日志的点云被留下来（换包不清）
            self.assertGreaterEqual(cloud.get("bags") or 0, 1)
            self.assertGreater(cloud.get("history") or 0, 0)
            self.assertGreater(cloud.get("count") or 0, 0)
            # 每个点带年龄（前端据它淡出），并下发淡出窗口
            self.assertEqual(len(cloud.get("ages") or []), cloud.get("count"))
            self.assertGreater(cloud.get("fade") or 0, 0)

            _status, cleared = self.post("/api/bag/cloud/clear", {})
            self.assertTrue(cleared["ok"])
            self.assertGreater(cleared["cleared"], 0)
            cloud = json.loads(self.get("/api/bag/cloud?t=0")[1])["cloud"]
            self.assertTrue(cloud is None or (cloud.get("history") or 0) == 0)
        finally:
            self.post("/api/bag/control", {"command": "stop"})
            self.httpd.bag_queue = None
            self.post("/api/map", {"map": 1})
            self.post("/api/speed", {"speed": 1})
            shutil.rmtree(tmp, ignore_errors=True)

    def test_history_cloud_expires_by_wall_clock(self):
        """跨包历史点云也受 60 s 约束：两段日志录制时间差太多 → 换包后不再显示。"""
        tmp = tempfile.mkdtemp(prefix="robot-bagcloudage-")
        group = os.path.join(tmp, "点云老化")
        os.makedirs(group)
        bag_fixtures.build_map_frame_bag(
            os.path.join(group, "cloud_a.bag"), samples=120, dt=0.1, clouds=True
        )
        # 第二段日志的墙钟比第一段晚 2 分钟（真实日志也是一包接一包、间隔几分钟）
        bag_fixtures.build_map_frame_bag(
            os.path.join(group, "cloud_b.bag"), samples=120, dt=0.1, clouds=True,
            wall_offset_ns=120 * 10 ** 9,
        )
        names = [os.path.join("点云老化", "cloud_a.bag"), os.path.join("点云老化", "cloud_b.bag")]
        try:
            self.post("/api/bag/dir", {"path": tmp})
            self.post("/api/speed", {"speed": 32})
            self.post("/api/bag/playlist", {"names": names})
            self._wait_queue()
            cloud = json.loads(self.get("/api/bag/cloud?t=0")[1])["cloud"]
            # 上一包的点云已经超过留存窗口：不再出现在返回里
            if cloud is not None:
                self.assertEqual(cloud.get("history") or 0, 0)
                self.assertEqual(cloud.get("bags") or 0, 0)
                for age in cloud.get("ages") or []:
                    self.assertLess(age, server.bag_player.CLOUD_FADE_SECONDS)
        finally:
            self.post("/api/bag/control", {"command": "stop"})
            self.httpd.bag_queue = None
            self.post("/api/map", {"map": 1})
            self.post("/api/speed", {"speed": 1})
            shutil.rmtree(tmp, ignore_errors=True)

    def test_manual_load_clears_cloud_history(self):
        """手动载入日志要把历史点云清掉（重新起笔）。"""
        tmp = tempfile.mkdtemp(prefix="robot-bagcloudload-")
        try:
            bag_fixtures.build_map_frame_bag(
                os.path.join(tmp, "one.bag"), samples=120, dt=0.1, clouds=True
            )
            bag_fixtures.build_map_frame_bag(
                os.path.join(tmp, "two.bag"), samples=120, dt=0.1, clouds=True
            )
            self.post("/api/bag/dir", {"path": tmp})
            self.post("/api/bag/load", {"name": "one.bag"})
            # 直接灌一段历史，模拟连续播放换包后的累积
            archived = self.httpd.manager.replay.cloud_at(float("inf"), max_points=6000)
            server.cloud_history(self.httpd).append(list(archived["points"]))
            self.assertGreater(len(server.cloud_history(self.httpd)), 0)
            self.post("/api/bag/load", {"name": "two.bag"})
            self.assertEqual(server.cloud_history(self.httpd), [])
        finally:
            self.post("/api/bag/control", {"command": "stop"})
            self.post("/api/map", {"map": 1})
            shutil.rmtree(tmp, ignore_errors=True)

    def test_static_traversal_is_blocked(self):
        with self.assertRaises(urllib.error.HTTPError) as context:
            self.get("/../backend/server.py")
        self.assertIn(context.exception.code, (403, 404))

    def test_stop_task_stops_replay_and_unlocks_editor(self):
        """按“停止任务”要真的把日志回放停下来，回放停了地图编辑就得能用。"""
        tmp = tempfile.mkdtemp(prefix="robot-stopplay-")
        try:
            path = os.path.join(tmp, "clip.bag")
            bag_fixtures.build_bag(path)
            self.post("/api/bag/load", {"name": path})
            self.post("/api/bag/control", {"command": "play"})
            time.sleep(0.5)
            _status, body = self.post("/api/stop", {})
            state = body.get("state") or {}
            self.assertIsNone((state.get("task") or {}).get("type"))
            self.assertFalse((state.get("bag") or {}).get("active"))
            # 回放真的停了：编辑地图不再被拦
            _status, draft = self.post("/api/editor/blank", {})
            self.assertTrue(draft["draft"]["active"])
            self.post("/api/editor/discard", {})
        finally:
            self.post("/api/bag/control", {"command": "stop"})
            self.post("/api/map", {"map": 1})
            shutil.rmtree(tmp, ignore_errors=True)

    def test_editor_is_blocked_while_replay_is_active(self):
        tmp = tempfile.mkdtemp(prefix="robot-editlock-")
        try:
            path = os.path.join(tmp, "clip.bag")
            bag_fixtures.build_bag(path)
            self.post("/api/bag/load", {"name": path})
            self.post("/api/bag/control", {"command": "play"})
            time.sleep(0.4)
            with self.assertRaises(urllib.error.HTTPError) as ctx:
                self.post("/api/editor/blank", {})
            self.assertEqual(ctx.exception.code, 400)
            self.post("/api/bag/control", {"command": "stop"})
            _status, draft = self.post("/api/editor/blank", {})
            self.assertTrue(draft["draft"]["active"])
            self.post("/api/editor/discard", {})
        finally:
            self.post("/api/bag/control", {"command": "stop"})
            self.post("/api/map", {"map": 1})
            shutil.rmtree(tmp, ignore_errors=True)

    def test_bag_load_keeps_dock_fixed(self):
        """载入日志只把车体放到日志第一帧，不能顺手把充电桩也搬过去。

        以前这里把 DOCK 覆盖成回放起点，连续播放每换一个包桩就跳一次。
        """
        tmp = tempfile.mkdtemp(prefix="robot-bagdock-")
        try:
            first = os.path.join(tmp, "one.bag")
            bag_fixtures.build_bag(first, samples=40)
            self.post("/api/bag/load", {"name": first})
            dock_after_first = dict(world.DOCK)
            # 第二段日志起点不同（车在别的角落），桩位不能跟着跑
            second = os.path.join(tmp, "two.bag")
            con = sqlite3.connect(second)
            con.execute(
                "create table Zmqmsg (TOPIC BLOB NOT NULL, TI REAL NOT NULL, DA BLOB NOT NULL)"
            )
            for index in range(40):
                stamp = 2000.0 + index * 0.1
                con.execute(
                    "insert into Zmqmsg values (?, ?, ?)",
                    (b"base_pose", stamp,
                     bag_fixtures._pose_message(stamp, 6.0 + 0.02 * index, 4.0, 0.0)),
                )
            con.commit()
            con.close()
            _status, body = self.post("/api/bag/load", {"name": second})
            self.assertIn("state", body)
            self.assertEqual(dict(world.DOCK), dock_after_first)
        finally:
            self.post("/api/bag/control", {"command": "stop"})
            self.post("/api/map", {"map": 1})
            shutil.rmtree(tmp, ignore_errors=True)

    def test_log_map_dock_is_stable_across_bags(self):
        """同一场地（同分辨率/尺寸/原点）的日志要复用同一个充电桩位置。

        每段日志的 whole_map 内容略有差别，按"自由区里最靠中心"重新选桩会跳到
        别的角落，连续播放时界面上充电桩一直在换地方。
        """
        log_map = {
            "resolution": 0.05,
            "width": 40,
            "height": 40,
            "origin": (-1.0, -1.0),
            "data": bytearray([0] * (40 * 40)),
        }
        transform = {
            "origin": [-1.0, -1.0],
            "scale": 1.0,
            "padding": 0.2,
            "cell": 0.1,
            "offset": [0.25, 0.25],
        }
        server.BAG_DOCK_CACHE.clear()
        first = {"dock": {"x": 1.05, "y": 1.05, "r": 0.45}, "replay_transform": transform}
        server.apply_stable_dock(first, log_map)
        second = {"dock": {"x": 2.05, "y": 1.55, "r": 0.45}, "replay_transform": transform}
        server.apply_stable_dock(second, log_map)
        self.assertEqual(second["dock"], {"x": 1.05, "y": 1.05, "r": 0.45})
        # 缓存位置已经撞到障碍时才重新选
        blocked = dict(log_map)
        blocked["data"] = bytearray([100] * (40 * 40))
        server.apply_stable_dock({"dock": {"x": 0.05, "y": 0.05, "r": 0.45}, "replay_transform": transform}, blocked)
        # 缓存位置在新区块里被占了：不强行复用，保留这次自己选的位置
        third = {"dock": {"x": 3.05, "y": 3.05, "r": 0.45}, "replay_transform": transform}
        server.apply_stable_dock(third, blocked)
        self.assertEqual(third["dock"], {"x": 3.05, "y": 3.05, "r": 0.45})
        server.BAG_DOCK_CACHE.clear()

    def test_editor_map_persists_and_reloads(self):
        tmp = tempfile.mkdtemp()
        original = server.editor_map_root
        server.editor_map_root = lambda: tmp
        registered = None
        try:
            definition = {
                "name": "持久化/测试图",
                "width": 12.0,
                "height": 9.0,
                "dock": {"x": 2.0, "y": 2.0, "r": 0.4},
                "start": {"x": 3.0, "y": 2.0, "theta": 0.0},
                "id": 987654,
            }
            path = server.persist_editor_map(definition)
            self.assertTrue(os.path.isfile(path))
            with open(path, encoding="utf-8") as handle:
                saved = json.load(handle)
            self.assertNotIn("id", saved)  # 编号只在运行时分配
            self.assertEqual(saved["name"], "持久化/测试图")

            self.assertEqual(server.load_persisted_editor_maps(), 1)
            loaded = [
                item for item in world.map_list() if item["name"] == "持久化/测试图"
            ]
            self.assertEqual(len(loaded), 1)
            self.assertTrue(loaded[0]["editor"])
            registered = loaded[0]["id"]
            # 重复加载同一来源要覆盖同一编号，不能越读越多
            self.assertEqual(server.load_persisted_editor_maps(), 1)
            self.assertEqual(
                len([m for m in world.map_list() if m["name"] == "持久化/测试图"]), 1
            )
        finally:
            server.editor_map_root = original
            if registered is not None:
                try:
                    world.delete_map(registered)
                except ValueError:
                    pass
            shutil.rmtree(tmp, ignore_errors=True)



if __name__ == "__main__":
    unittest.main()
