import base64
import json
import os
import shutil
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

    def test_nav_snapshot_endpoint(self):
        status, body = self.get("/api/nav")
        self.assertEqual(status, 200)
        nav = json.loads(body)
        names = {node["name"] for node in nav["nodes"]}
        self.assertEqual(names, {"planner", "dock", "region_manager", "trajectory_map"})
        ports = {service["name"]: service["port"] for service in nav["services"]}
        self.assertEqual(ports["nav_task_action"], 25009)
        self.assertEqual(ports["autodock_action"], 25001)
        self.assertEqual(ports["map_load"], 20020)
        self.assertEqual(ports["record_path"], 20015)
        self.assertEqual(ports["get_grid_map"], 20017)
        self.assertIn("static_config", nav)
        topics = {topic["name"] for topic in nav["topics"]}
        self.assertIn("cmd_vel", topics)
        self.assertIn("nav_status", topics)

    def test_nav_mode_toggle_endpoint(self):
        _, state = self.post("/api/nav/mode", {"enabled": False})
        self.assertFalse(state["nav"]["enabled"])
        _, state = self.post("/api/nav/mode", {"enabled": True})
        self.assertTrue(state["nav"]["enabled"])

    def test_nav_action_endpoint_drives_sim(self):
        self.post("/api/map", {"map": 3})
        with self.httpd.simulator.lock:
            manager = self.httpd.manager
            manager.grid.build_free_area(
                (manager.robot.pose[0], manager.robot.pose[1])
            )
            zones = manager.grid.partition_zones()
        order = [zones[0]["id"]]
        _, state = self.post(
            "/api/nav/action",
            {
                "node": "planner",
                "action": "nav_task_action",
                "cmd": 1,
                "goal": {"mode": 30, "path_name": ",".join(str(z) for z in order)},
            },
        )
        self.assertEqual(state["task"]["type"], "mow")
        self.assertEqual(state["task"]["zone_order"], order)
        planner = [n for n in state["nav"]["nodes"] if n["name"] == "planner"][0]
        self.assertEqual(planner["actions"][0]["state"], "running")
        nav_topics = {topic["name"] for topic in state["nav"]["topics"]}
        self.assertIn("nav_task_path", nav_topics)

    def test_nav_action_autodock_triggers_charge(self):
        _, state = self.post(
            "/api/nav/action",
            {
                "node": "dock",
                "action": "autodock_action",
                "cmd": 1,
                "goal": {"docking_waypoint": "dock"},
            },
        )
        self.assertIn(state["task"]["phase"], ("charge_go", "to_dock"))
        self.assertEqual(state["task"]["charge_reason"], "manual")
        nav_topics = {topic["name"] for topic in state["nav"]["topics"]}
        self.assertIn("recharge_start_pose", nav_topics)
        self.assertIn("global_plan", nav_topics)

    def test_start_mapping(self):
        _, state = self.post("/api/task", {"task": "map"})
        self.assertEqual(state["task"]["type"], "map")
        self.assertTrue(state["task"]["phase"].startswith("map"))

    def test_start_mowing_with_order(self):
        self.post("/api/map", {"map": 3})
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

    def test_static_traversal_is_blocked(self):
        with self.assertRaises(urllib.error.HTTPError) as context:
            self.get("/../backend/server.py")
        self.assertIn(context.exception.code, (403, 404))


if __name__ == "__main__":
    unittest.main()
