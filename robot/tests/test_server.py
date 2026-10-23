import base64
import json
import os
import sys
import threading
import time
import unittest
import urllib.error
import urllib.request

BACKEND_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "backend"
)
sys.path.insert(0, BACKEND_DIR)

import server  # noqa: E402
import world  # noqa: E402


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

    def test_map_switch(self):
        _, state = self.post("/api/map", {"map": 3})
        self.assertEqual(state["map_id"], 3)
        self.assertAlmostEqual(state["pose"]["x"], world.START["x"], places=2)
        self.assertAlmostEqual(state["pose"]["y"], world.START["y"], places=2)
        self.assertEqual(
            len(state["map"]["occupancy"]),
            state["map"]["cols"] * state["map"]["rows"],
        )

    @unittest.skipUnless(
        os.path.isdir("/home/fibocom/heyufei/data/map/map"), "真实地图目录不存在"
    )
    def test_import_real_map_endpoint(self):
        _, state = self.post(
            "/api/map/import",
            {"path": "/home/fibocom/heyufei/data/map/map"},
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
        self.assertIn("割草机器人控制台", body)

    def test_static_traversal_is_blocked(self):
        with self.assertRaises(urllib.error.HTTPError) as context:
            self.get("/../backend/server.py")
        self.assertIn(context.exception.code, (403, 404))


if __name__ == "__main__":
    unittest.main()
