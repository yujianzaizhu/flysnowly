"""点到点导航接入测试：注入 FakeTeaBridge，不依赖真实 tea。

覆盖: /api/nav/goal* 接口、nav_task_action 参数、手动/导航互斥(409)、
cmd_vel 速度闭环与超时清零、nav_task_action_result 终态自动退出。
"""

import json
import math
import os
import sys
import threading
import time
import unittest
import urllib.error
import urllib.request

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
BACKEND_DIR = os.path.join(os.path.dirname(TEST_DIR), "backend")
sys.path.insert(0, BACKEND_DIR)

from nav_process import config, server, tea_bridge  # noqa: E402
from nav_process.robot import NavRobot  # noqa: E402


class FakeTeaBridge:
    """最小接口 Fake: 只实现点到点路径上会被调用的方法。"""

    def __init__(self):
        self.calls = []
        self.nav_script = {"ok": True, "status": 0, "msgs": "ok"}
        self.cmd_vel = None  # 伪造 latest_cmd_vel 返回: {"linear", "angular", "age"}
        self.nav_result = None  # 伪造 latest_nav_result 返回: {"outcome", "text", "at"}
        self.building = False

    def start(self):
        pass

    def stop(self):
        pass

    def tick(self, robot):
        pass

    def record_path(self, cmd, type_=None, id_=None, deleted_ids=None, path_name=None):
        return {"ok": True, "result": 0, "info": "ok"}

    def get_map_info(self, cmd=tea_bridge.MAP_CMD_LOAD, wait=True):
        return {"ok": True, "result": 0, "map_version": 1}

    def probe(self):
        return {"ok": True, "connected": True, "result": 0}

    def nav_action(self, cmd, goal=None):
        self.calls.append(("nav", cmd, goal))
        return dict(self.nav_script)

    def begin_session(self, type_):
        self.building = True

    def end_session(self):
        self.building = False

    def mark_stale(self):
        pass

    def is_building(self):
        return self.building

    def map_snapshot(self, version=None):
        return {"version": 1, "map": None}

    def latest_cmd_vel(self):
        return self.cmd_vel

    def latest_nav_result(self):
        return self.nav_result

    def summary(self):
        return {
            "enabled": True,
            "connected": True,
            "host": "127.0.0.1",
            "record_port": 20015,
            "map_port": 20018,
            "nav_port": 25009,
            "session": {
                "state": "building" if self.building else "idle",
                "type": None,
                "stale": False,
                "since": None,
            },
            "trail": [],
            "trail_len": 0.0,
            "map_version": 1,
            "pub_count": 0,
        }


class NavP2PRouteTest(unittest.TestCase):
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
        with self.httpd.robot.lock:
            # 仿真线程在后台积分位姿, 用例间回到原点保证断言确定
            self.httpd.robot.pose = [0.0, 0.0, 0.0]
            self.httpd.robot.v = 0.0
            self.httpd.robot.w = 0.0
        config.reset()

    def _get_json(self, path):
        with urllib.request.urlopen(self.base + path, timeout=3) as response:
            return json.loads(response.read().decode("utf-8"))

    def _post(self, path, payload, expect_error=None):
        request = urllib.request.Request(
            self.base + path,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=3) as response:
                self.assertIsNone(expect_error)
                return response.status, response.read().decode("utf-8")
        except urllib.error.HTTPError as exc:
            self.assertEqual(exc.code, expect_error)
            return exc.code, exc.read().decode("utf-8")

    def _post_json(self, path, payload):
        _status, body = self._post(path, payload)
        return json.loads(body)

    def _wait_until(self, predicate, timeout=2.0):
        deadline = time.time() + timeout
        state = self._get_json("/api/nav/state")
        while time.time() < deadline:
            if predicate(state):
                return state
            time.sleep(0.05)
            state = self._get_json("/api/nav/state")
        return state

    def test_goal_start_activates_and_locks_manual(self):
        data = self._post_json("/api/nav/goal", {"x": 2.0, "y": 1.0})
        self.assertTrue(data["nav"]["active"])
        self.assertFalse(data["nav"]["paused"])
        self.assertEqual(data["nav"]["goal"]["x"], 2.0)
        self.assertEqual(data["nav"]["goal"]["y"], 1.0)
        self.assertTrue(data["nav_action_result"]["ok"])
        # nav_task_action 参数: START + NAVIGATION + 目标坐标 + 默认朝向(指向目标)
        nav_calls = [call for call in self.tea.calls if call[0] == "nav"]
        self.assertEqual(len(nav_calls), 1)
        _tag, cmd, goal = nav_calls[0]
        self.assertEqual(cmd, tea_bridge.ACTION_START)
        self.assertEqual(goal["mode"], tea_bridge.TASK_MODE_NAVIGATION)
        self.assertAlmostEqual(goal["target_pose"]["x"], 2.0)
        self.assertAlmostEqual(goal["target_pose"]["y"], 1.0)
        self.assertAlmostEqual(goal["target_pose"]["yaw"], math.atan2(1.0, 2.0), places=3)
        # 导航中手动遥控与急停都被锁定
        status, _body = self._post("/api/nav/drive", {"v": 0.3, "w": 0.0}, expect_error=409)
        self.assertEqual(status, 409)
        status, _body = self._post("/api/nav/stop", {}, expect_error=409)
        self.assertEqual(status, 409)

    def test_goal_conflict_when_building(self):
        self.tea.building = True
        status, _body = self._post("/api/nav/goal", {"x": 1.0, "y": 0.0}, expect_error=409)
        self.assertEqual(status, 409)

    def test_goal_conflict_when_already_active(self):
        self._post("/api/nav/goal", {"x": 1.0, "y": 0.0})
        status, _body = self._post("/api/nav/goal", {"x": 2.0, "y": 0.0}, expect_error=409)
        self.assertEqual(status, 409)

    def test_goal_rejected_by_tea_status(self):
        self.tea.nav_script = {"ok": True, "status": 1, "msgs": "task cmd unknown"}
        data = self._post_json("/api/nav/goal", {"x": 1.0, "y": 0.0})
        self.assertFalse(data["nav"]["active"])
        self.assertFalse(data["nav_action_result"]["ok"])
        self.assertIn("task cmd unknown", data["nav_action_result"]["info"])

    def test_goal_bridge_failure(self):
        self.tea.nav_script = {"ok": False, "info": "桥未启动", "error": "桥未启动"}
        data = self._post_json("/api/nav/goal", {"x": 1.0, "y": 0.0})
        self.assertFalse(data["nav"]["active"])
        self.assertFalse(data["nav_action_result"]["ok"])

    def test_pause_continue_cancel_flow(self):
        self._post("/api/nav/goal", {"x": 1.0, "y": 0.0})
        data = self._post_json("/api/nav/goal/pause", {})
        self.assertTrue(data["nav"]["active"])
        self.assertTrue(data["nav"]["paused"])
        data = self._post_json("/api/nav/goal/continue", {})
        self.assertTrue(data["nav"]["active"])
        self.assertFalse(data["nav"]["paused"])
        data = self._post_json("/api/nav/goal/cancel", {})
        self.assertFalse(data["nav"]["active"])
        cmds = [call[1] for call in self.tea.calls if call[0] == "nav"]
        self.assertEqual(
            cmds,
            [
                tea_bridge.ACTION_START,
                tea_bridge.ACTION_PAUSE,
                tea_bridge.ACTION_CONTINUE,
                tea_bridge.ACTION_CANCEL,
            ],
        )

    def test_cancel_without_active_conflict(self):
        status, _body = self._post("/api/nav/goal/cancel", {}, expect_error=409)
        self.assertEqual(status, 409)

    def test_enter_workarea_conflict_while_navigating(self):
        self._post("/api/nav/goal", {"x": 1.0, "y": 0.0})
        status, _body = self._post("/api/nav/activity", {"activity": "workarea"}, expect_error=409)
        self.assertEqual(status, 409)

    def test_terminal_result_deactivates(self):
        self._post("/api/nav/goal", {"x": 1.0, "y": 0.0})
        # 任务下发之后收到的 nav_task_action_result 判为本次任务终态
        self.tea.nav_result = {"outcome": 1, "text": "succeeded", "at": time.monotonic()}
        state = self._wait_until(lambda s: not s["nav"]["active"])
        self.assertFalse(state["nav"]["active"])
        self.assertFalse(self.httpd.robot.nav_mode)
        self.assertEqual(state["nav"]["last_result"]["outcome"], 1)

    def test_stale_result_does_not_deactivate(self):
        # 旧任务的结果(at 早于本次下发)不能误杀当前任务
        self.tea.nav_result = {"outcome": 2, "text": "old", "at": time.monotonic() - 100.0}
        self._post("/api/nav/goal", {"x": 1.0, "y": 0.0})
        time.sleep(0.2)
        state = self._get_json("/api/nav/state")
        self.assertTrue(state["nav"]["active"])

    def test_cmd_vel_drives_robot_and_timeout(self):
        self._post("/api/nav/goal", {"x": 1.0, "y": 0.0})
        self.tea.cmd_vel = {"linear": 0.3, "angular": 0.0, "age": 0.05}
        state = self._wait_until(lambda s: s["velocity"]["linear"] > 0.2)
        self.assertGreater(state["velocity"]["linear"], 0.2)
        # cmd_vel 超时(age 超阈值) -> 导航速度源清零, 机器人减速停车
        self.tea.cmd_vel = {"linear": 0.3, "angular": 0.0, "age": 10.0}
        state = self._wait_until(lambda s: abs(s["velocity"]["linear"]) < 0.05)
        self.assertLess(abs(state["velocity"]["linear"]), 0.05)


class NavRobotNavModeTest(unittest.TestCase):
    def test_nav_mode_mutex(self):
        robot = NavRobot()
        robot.drive(0.4, 0.0)
        for _ in range(30):
            robot.step(0.02)
        self.assertGreater(robot.v, 0.2)
        robot.set_nav_mode(True)  # 进入导航: 清手动目标
        robot.drive(0.4, 0.0)  # 导航中手动输入被忽略(兜底, server 侧另有 409)
        robot.drive_nav(0.3, 0.0)
        for _ in range(60):
            robot.step(0.02)
        self.assertAlmostEqual(robot.v, 0.3, places=2)
        # 导航速度源限幅与手动一致
        robot.drive_nav(2.0, 0.0)
        for _ in range(60):
            robot.step(0.02)
        self.assertAlmostEqual(robot.v, 0.6, places=2)
        robot.set_nav_mode(False)  # 退出导航: 清导航目标并停车
        for _ in range(60):
            robot.step(0.02)
        self.assertAlmostEqual(robot.v, 0.0, places=2)

    def test_drive_nav_ignored_outside_nav_mode(self):
        robot = NavRobot()
        robot.drive_nav(0.5, 0.0)
        for _ in range(30):
            robot.step(0.02)
        self.assertAlmostEqual(robot.v, 0.0, places=3)


if __name__ == "__main__":
    unittest.main()
