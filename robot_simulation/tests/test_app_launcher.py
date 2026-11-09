"""统一入口测试：验证应用切换及两套页面/API 的数据隔离。"""

import http.client
import json
import os
import sys
import threading
import time
import unittest
from unittest import mock
import urllib.error
import urllib.request

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
BACKEND_DIR = os.path.join(os.path.dirname(TEST_DIR), "backend")
sys.path.insert(0, BACKEND_DIR)

import app_launcher  # noqa: E402
from nav_process import server as nav_server, tea_bridge  # noqa: E402


class ApplicationSwitchTest(unittest.TestCase):
    def setUp(self):
        self.patch = mock.patch.object(
            nav_server.tea_bridge,
            "create",
            side_effect=lambda opts, log_fn=None: tea_bridge.DisabledBridge("测试桥"),
        )
        self.patch.start()
        self.controller = app_launcher.ApplicationController("127.0.0.1", 0)
        self.thread = threading.Thread(target=self.controller.run, daemon=True)
        self.thread.start()
        self.assertTrue(self.controller.ready.wait(5))
        self.base = "http://127.0.0.1:{}".format(self.controller.port)

    def tearDown(self):
        self.controller.stop()
        self.thread.join(timeout=5)
        self.patch.stop()

    def _get(self, path, expect_error=None):
        try:
            with urllib.request.urlopen(self.base + path, timeout=2) as response:
                return response.status, response.read().decode("utf-8")
        except urllib.error.HTTPError as exc:
            body = exc.read().decode("utf-8")
            self.assertEqual(exc.code, expect_error)
            return exc.code, body

    def _post(self, path, payload, expect_error=None):
        request = urllib.request.Request(
            self.base + path,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=3) as response:
                return response.status, json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            body = json.loads(exc.read().decode("utf-8"))
            self.assertEqual(exc.code, expect_error)
            return exc.code, body

    def _wait_mode(self, target):
        deadline = time.time() + 10
        while time.time() < deadline:
            try:
                _status, body = self._get("/api/mode")
                state = json.loads(body)
                if state["mode"] == target and not state["switching"]:
                    return
            except (OSError, urllib.error.URLError):
                pass
            time.sleep(0.05)
        self.fail("模式切换超时: {}".format(target))

    def test_switch_reloads_two_independent_applications(self):
        _status, page = self._get("/")
        self.assertIn('data-sim-mode="standalone"', page)
        self.assertIn('data-dock-tab="bag"', page)
        self.assertNotIn('data-drive="arc_left"', page)

        _status, result = self._post("/api/sim_mode", {"mode": "nav_process"})
        self.assertTrue(result["switching"])
        self._wait_mode("nav_process")
        _status, page = self._get("/")
        self.assertIn('data-sim-mode="nav_process"', page)
        self.assertNotIn("日志分析", page)
        self.assertNotIn('data-dock-tab="bag"', page)
        _status, raw_state = self._get("/api/nav/state")
        state = json.loads(raw_state)
        for key in ("world", "task", "bag", "map", "zones"):
            self.assertNotIn(key, state)
        self._post("/api/bag/control", {"command": "play"}, expect_error=404)
        self._post("/api/task", {"task": "map"}, expect_error=404)

        self._post("/api/sim_mode", {"mode": "standalone"})
        self._wait_mode("standalone")
        _status, page = self._get("/")
        self.assertIn('data-sim-mode="standalone"', page)
        self.assertIn('data-dock-tab="bag"', page)

    def test_switch_closes_keep_alive_connections(self):
        connections = [
            http.client.HTTPConnection("127.0.0.1", self.controller.port, timeout=2)
            for _ in range(4)
        ]
        try:
            for connection in connections:
                connection.request("GET", "/")
                response = connection.getresponse()
                self.assertIn(
                    'data-sim-mode="standalone"', response.read().decode("utf-8")
                )

            self._post("/api/sim_mode", {"mode": "nav_process"})
            self._wait_mode("nav_process")

            closed = 0
            for connection in connections:
                # 旧长连接必须已被关闭；即使内核允许再次写入，也不能拿到旧应用页面。
                try:
                    connection.request("GET", "/")
                    response = connection.getresponse()
                    page = response.read().decode("utf-8")
                except (OSError, http.client.HTTPException):
                    closed += 1
                    continue
                self.assertNotIn('data-sim-mode="standalone"', page)
                self.assertIn('data-sim-mode="nav_process"', page)
            self.assertEqual(closed, len(connections))
        finally:
            for connection in connections:
                connection.close()

    def test_building_session_blocks_mode_switch(self):
        self._post("/api/sim_mode", {"mode": "nav_process"})
        self._wait_mode("nav_process")

        class BuildingTea(object):
            def is_building(self):
                return True

            def stop(self):
                pass

        self.controller.httpd.tea = BuildingTea()
        _status, body = self._post(
            "/api/sim_mode", {"mode": "standalone"}, expect_error=409
        )
        self.assertIn("工作区构建中", body["error"])


class LauncherArgumentTest(unittest.TestCase):
    @mock.patch.object(app_launcher, "ApplicationController")
    def test_tea_bridge_keeps_standalone_as_default(self, controller_class):
        app_launcher.main(["--tea-bridge"])
        args = controller_class.call_args.args
        self.assertEqual(args[2], "standalone")
        self.assertTrue(controller_class.call_args.kwargs["tea_opts"]["enabled"])
        controller_class.return_value.run.assert_called_once_with()

    @mock.patch.object(app_launcher, "ApplicationController")
    def test_navigation_mode_must_be_explicit(self, controller_class):
        app_launcher.main(["--tea-bridge", "--mode", "nav_process"])
        self.assertEqual(controller_class.call_args.args[2], "nav_process")
        controller_class.return_value.run.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
