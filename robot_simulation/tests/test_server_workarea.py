"""独立导航进程应用测试：注入 FakeTeaBridge，不依赖真实 tea。"""

import json
import os
import sys
import threading
import unittest
import urllib.error
import urllib.request

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
BACKEND_DIR = os.path.join(os.path.dirname(TEST_DIR), "backend")
sys.path.insert(0, BACKEND_DIR)

import config as standalone_config  # noqa: E402
from nav_process import config, server, tea_bridge  # noqa: E402


class FakeTeaBridge:
    def __init__(self):
        self.calls = []
        self.record_script = {}
        self.map_resp = {"ok": True, "result": 0, "map_version": 3}
        self.map_snapshot_data = {
            "version": 3,
            "map": {"res": 0.05, "sub_maps": [{"type": 1, "poses": [[0, 0], [1, 0]]}]},
        }
        self.building = False
        self.session_type = None
        # 镜像"在线桥"的真实 opts: create_server/_workarea_connect 启用的桥 enabled=True,
        # connect 幂等判断要求两侧一致(否则会被误判为参数变化而重建)
        self.opts = dict(tea_bridge.DEFAULT_OPTS)
        self.opts["enabled"] = True

    def start(self):
        self.calls.append(("start",))

    def stop(self):
        self.calls.append(("stop",))

    def tick(self, robot):
        pass

    def record_path(self, cmd, type_=None, id_=None, deleted_ids=None, path_name=None):
        self.calls.append(("record", cmd, type_, deleted_ids))
        return dict(self.record_script.get(cmd, {"ok": True, "result": 0, "info": "ok"}))

    def get_map_info(self, cmd=tea_bridge.MAP_CMD_LOAD, wait=True):
        self.calls.append(("get_map", cmd))
        return dict(self.map_resp)

    def probe(self):
        self.calls.append(("probe",))
        return {"ok": True, "connected": True, "result": 0}

    def nav_action(self, cmd, goal=None):
        return {"ok": True, "status": "accepted"}

    def begin_session(self, type_):
        self.calls.append(("begin", type_))
        self.building = True
        self.session_type = type_

    def end_session(self):
        self.calls.append(("end",))
        self.building = False
        self.session_type = None

    def mark_stale(self):
        pass

    def is_building(self):
        return self.building

    def map_snapshot(self, version=None):
        snap = dict(self.map_snapshot_data)
        if version == snap["version"]:
            return {"version": version, "unchanged": True}
        return snap

    def latest_cmd_vel(self):
        return None

    def latest_nav_result(self):
        return None

    def summary(self):
        return {
            "enabled": True,
            "connected": True,
            "host": "127.0.0.1",
            "record_port": 20015,
            "map_port": 20018,
            "session": {
                "state": "building" if self.building else "idle",
                "type": self.session_type,
                "stale": False,
            },
            "trail": [[0, 0], [1, 1]] if self.building else [],
            "trail_len": 1.4,
            "map_version": 3,
            "map_checked": True,
            "pub_count": 0,
        }


class WorkareaRouteTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.httpd = server.create_server("127.0.0.1", 0, {"enabled": False})
        cls.port = cls.httpd.server_address[1]
        cls.thread = threading.Thread(target=cls.httpd.serve_forever, daemon=True)
        cls.thread.start()
        cls.base = "http://127.0.0.1:{}".format(cls.port)

    @classmethod
    def tearDownClass(cls):
        cls.httpd.shutdown()
        cls.httpd.stop_application()
        cls.httpd.server_close()
        cls.thread.join(timeout=2)

    def setUp(self):
        self.tea = FakeTeaBridge()
        self.httpd.tea = self.tea
        self.httpd.simulator.tea = self.tea
        self.httpd.activity = "work"
        self.httpd.nav.update({"active": False, "paused": False, "goal": None, "since": 0.0})
        self.httpd.robot.set_nav_mode(False)
        self.httpd.robot.stop()
        config.reset()

    def _get(self, path, expect_error=None):
        try:
            with urllib.request.urlopen(self.base + path, timeout=3) as response:
                return response.status, response.read().decode("utf-8")
        except urllib.error.HTTPError as exc:
            self.assertEqual(exc.code, expect_error)
            return exc.code, exc.read().decode("utf-8")

    def _get_json(self, path):
        _status, body = self._get(path)
        return json.loads(body)

    def _post(self, path, payload, expect_error=None):
        request = urllib.request.Request(
            self.base + path,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=3) as response:
                body = json.loads(response.read().decode("utf-8"))
                if isinstance(body, dict) and "workarea_result" in body:
                    body = body["workarea_result"]
                return response.status, body
        except urllib.error.HTTPError as exc:
            body = json.loads(exc.read().decode("utf-8"))
            if expect_error is None:
                raise AssertionError("意外的 {} : {}".format(exc.code, body))
            self.assertEqual(exc.code, expect_error)
            return exc.code, body

    def _logs(self):
        return [entry["message"] for entry in list(self.httpd.robot.events)]

    def test_state_and_page_are_navigation_only(self):
        state = self._get_json("/api/nav/state")
        self.assertEqual(state["sim_mode"], "nav_process")
        self.assertIn("workarea", state)
        for key in ("world", "task", "bag", "map", "zones"):
            self.assertNotIn(key, state)
        _status, page = self._get("/")
        self.assertIn('data-sim-mode="nav_process"', page)
        self.assertIn("功能区", page)
        self.assertNotIn("日志分析", page)
        self.assertNotIn('data-dock-tab="bag"', page)
        self._get("/analysis.html", expect_error=404)

    def test_standalone_routes_do_not_exist(self):
        for path, payload in (
            ("/api/task", {"task": "map"}),
            ("/api/map/edit", {"action": "add_static"}),
            ("/api/bag/control", {"command": "play"}),
        ):
            self._post(path, payload, expect_error=404)
        self._get("/api/bag", expect_error=404)

    def test_drive_sets_velocity(self):
        self._post("/api/nav/drive", {"v": 0.3, "w": -0.15})
        self.assertAlmostEqual(self.httpd.robot.target_v, 0.3)
        self.assertAlmostEqual(self.httpd.robot.target_w, -0.15)
        self._post("/api/nav/stop", {})
        self.assertEqual(self.httpd.robot.target_v, 0.0)

    def test_config_is_not_applied_to_standalone(self):
        original = standalone_config.settings.nav_speed
        _status, state = self._post("/api/nav/config", {"values": {"arc_speed": 0.5}})
        self.assertEqual(state["config"]["arc_speed"], 0.5)
        self.assertEqual(standalone_config.settings.nav_speed, original)
        self.assertFalse(hasattr(standalone_config.settings, "arc_speed"))

    def test_start_build_flow_and_log(self):
        status, body = self._post("/api/nav/workarea/start", {"type": 1})
        self.assertEqual(status, 200)
        self.assertTrue(body["ok"])
        self.assertEqual(body["session"]["state"], "building")
        self.assertIn(("begin", 1), self.tea.calls)
        self.assertTrue(any("START_MAP" in message and "成功" in message for message in self._logs()))

    def test_start_invalid_type_rejected(self):
        self._post("/api/nav/workarea/start", {"type": 4}, expect_error=400)

    def test_start_check_failure_does_not_start(self):
        self.tea.record_script[tea_bridge.CMD_ALLOW_START] = {"ok": False, "result": 2, "info": "无工作区"}
        _status, body = self._post("/api/nav/workarea/start", {"type": 3, "check": True})
        self.assertFalse(body["ok"])
        kinds = [call[:2] for call in self.tea.calls]
        self.assertNotIn(("record", tea_bridge.CMD_START), kinds)

    def test_save_refreshes_map_and_ends_session(self):
        self._post("/api/nav/workarea/start", {"type": 1})
        self.tea.record_script[tea_bridge.CMD_SAVE] = {"ok": True, "result": 0, "point_count": 5}
        _status, body = self._post("/api/nav/workarea/save", {"check": True})
        self.assertTrue(body["ok"])
        self.assertEqual(body["map_version"], 3)
        self.assertEqual(body["session"]["state"], "idle")
        self.assertIn(("get_map", tea_bridge.MAP_CMD_LOAD), self.tea.calls)

    def test_save_failure_keeps_session(self):
        self._post("/api/nav/workarea/start", {"type": 2})
        self.tea.record_script[tea_bridge.CMD_SAVE] = {"ok": False, "result": 6, "info": "轨迹太短"}
        _status, body = self._post("/api/nav/workarea/save", {})
        self.assertFalse(body["ok"])
        self.assertEqual(body["session"]["state"], "building")
        self.assertNotIn(("end",), self.tea.calls)

    def test_cancel_also_refreshes_map(self):
        self._post("/api/nav/workarea/start", {"type": 1})
        _status, body = self._post("/api/nav/workarea/cancel", {})
        self.assertTrue(body["ok"])
        self.assertIn(("get_map", tea_bridge.MAP_CMD_LOAD), self.tea.calls)
        self.assertEqual(body["session"]["state"], "idle")

    def test_manual_refresh_map_while_building(self):
        self._post("/api/nav/workarea/start", {"type": 1})
        _status, body = self._post("/api/nav/workarea/refresh_map", {})
        self.assertTrue(body["ok"])
        self.assertEqual(body["map_version"], 3)
        self.assertIn(("get_map", tea_bridge.MAP_CMD_LOAD), self.tea.calls)
        # 纯读命令, 不打断构建会话。
        self.assertTrue(self.tea.building)
        self.assertTrue(any("手动刷新" in message for message in self._logs()))

    def test_clear_load_and_raw(self):
        _status, body = self._post("/api/nav/workarea/clear", {"ids": [2, 5]})
        self.assertTrue(body["ok"])
        records = [call for call in self.tea.calls if call[0] == "record"]
        self.assertEqual(records[-1][3], [2, 5])
        self._post("/api/nav/workarea/load", {})
        self._post("/api/nav/workarea/raw", {"service": "record_path", "cmd": 6})
        kinds = [call[:2] for call in self.tea.calls]
        self.assertIn(("record", tea_bridge.CMD_LOAD), kinds)
        self.assertIn(("record", tea_bridge.CMD_POINT), kinds)

    def test_clear_without_ids_is_whole_map(self):
        """页面"删除当前图"按钮不带 id: deleted_ids 为空 = 老协议整图清空, 成功后刷新地图。"""

        _status, body = self._post("/api/nav/workarea/clear", {})
        self.assertTrue(body["ok"])
        records = [call for call in self.tea.calls if call[0] == "record"]
        self.assertEqual(records[-1][1], tea_bridge.CMD_CLEAR)
        self.assertIsNone(records[-1][3])
        self.assertIn(("get_map", tea_bridge.MAP_CMD_LOAD), self.tea.calls)
        self.assertTrue(any("整图清空" in message and "成功" in message for message in self._logs()))

    def test_initial_map_fetch_empty_reports_empty(self):
        """接入自动拉图: tea 确认读图成功但无子图 -> 日志给出"地图为空"。"""

        self.tea.map_snapshot_data = {"version": 1, "map": {"res": 0.05, "sub_maps": []}}
        resp = server._fetch_initial_map(self.tea, self.httpd.robot.log)
        self.assertTrue(resp["ok"])
        self.assertIn(("get_map", tea_bridge.MAP_CMD_LOAD), self.tea.calls)
        self.assertTrue(any("工作区地图为空" in message for message in self._logs()))

    def test_initial_map_fetch_reports_submaps(self):
        resp = server._fetch_initial_map(self.tea, self.httpd.robot.log)
        self.assertTrue(resp["ok"])
        self.assertTrue(any("1 个子图" in message for message in self._logs()))

    def test_initial_map_fetch_retries_then_warns(self):
        """tea 未就绪时按重试次数重拉, 仍失败记 warn 并放弃(可由页面刷新补拉)。"""

        original = server.AUTO_MAP_FETCH_INTERVAL
        server.AUTO_MAP_FETCH_INTERVAL = 0
        try:
            self.tea.map_resp = {"ok": False, "result": None, "info": "服务无响应"}
            resp = server._fetch_initial_map(self.tea, self.httpd.robot.log)
        finally:
            server.AUTO_MAP_FETCH_INTERVAL = original
        self.assertFalse(resp["ok"])
        gets = [call for call in self.tea.calls if call[0] == "get_map"]
        self.assertEqual(len(gets), server.AUTO_MAP_FETCH_RETRIES)
        self.assertTrue(any("自动获取工作区地图失败" in message for message in self._logs()))

    def test_auto_fetch_map_runs_in_background(self):
        thread = server._auto_fetch_map(self.tea, self.httpd.robot)
        thread.join(timeout=3)
        self.assertFalse(thread.is_alive())
        self.assertIn(("get_map", tea_bridge.MAP_CMD_LOAD), self.tea.calls)

    def test_map_endpoint_version_short_circuit(self):
        snap = self._get_json("/api/nav/workarea/map")
        self.assertEqual(snap["version"], 3)
        snap = self._get_json("/api/nav/workarea/map?v=3")
        self.assertEqual(snap, {"version": 3, "unchanged": True})

    def test_disabled_bridge_rejected_without_standalone_fallback(self):
        disabled = tea_bridge.DisabledBridge("测试未启用")
        self.httpd.tea = disabled
        self.httpd.simulator.tea = disabled
        self._post("/api/nav/workarea/start", {"type": 1}, expect_error=400)
        self._post("/api/nav/activity", {"activity": "workarea"}, expect_error=400)
        self._post("/api/task", {"task": "map"}, expect_error=404)

    def test_connect_with_same_opts_is_idempotent(self):
        """同参数 connect 不重建桥: 重建会拆掉已绑定的内嵌 proxy(run.sh 开机 connect 实测踩坑)。"""

        _status, summary = self._post("/api/nav/workarea/connect", {})
        self.assertTrue(summary["enabled"])
        self.assertIs(self.httpd.tea, self.tea)
        self.assertNotIn(("stop",), self.tea.calls)
        self.assertFalse(any(call[0] == "get_map" for call in self.tea.calls))

    def test_connect_with_new_opts_recreates_bridge(self):
        # proxy/话题端口用空闲端口, 避免测试期 connect 到真实 10001/10002(共享机串线)
        import socket as _socket

        def _free_port():
            with _socket.socket() as sock:
                sock.bind(("127.0.0.1", 0))
                return sock.getsockname()[1]

        try:
            self._post(
                "/api/nav/workarea/connect",
                {
                    "record_port": 21515,
                    "run_proxy": False,
                    "proxy_pub_port": _free_port(),
                    "proxy_sub_port": _free_port(),
                },
            )
            self.assertIn(("stop",), self.tea.calls)
            self.assertIsNot(self.httpd.tea, self.tea)
            if hasattr(self.httpd.tea, "opts"):
                self.assertEqual(self.httpd.tea.opts["record_port"], 21515)
        finally:
            # 必须停掉重建出的真实桥: 泄漏的 SUB 线程会钉住其 zmq context,
            # 之后任意测试触发 GC 时 context.__del__/term() 会永久阻塞
            self.httpd.tea.stop()
            self.httpd.tea = self.tea
            self.httpd.simulator.tea = self.tea

    def test_activity_and_building_guard(self):
        _status, state = self._post("/api/nav/activity", {"activity": "workarea"})
        self.assertEqual(state["activity"], "workarea")
        self._post("/api/nav/workarea/start", {"type": 1})
        self._post("/api/nav/activity", {"activity": "work"}, expect_error=409)
        self._post("/api/nav/workarea/cancel", {})
        _status, state = self._post("/api/nav/activity", {"activity": "work"})
        self.assertEqual(state["activity"], "work")


if __name__ == "__main__":
    unittest.main()
