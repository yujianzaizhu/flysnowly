"""工作区会话与辅助逻辑测试。

纯逻辑部分(错误码映射/四元数/配置参数)不依赖 pyzmq;
会话镜像与轨迹抽稀部分依赖 pyzmq, 缺失时跳过。
"""

import math
import os
import sys
import unittest

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
BACKEND_DIR = os.path.join(os.path.dirname(TEST_DIR), "backend")
sys.path.insert(0, BACKEND_DIR)

from nav_process import config, tea_bridge  # noqa: E402

try:
    import zmq  # noqa: F401

    _ZMQ_OK = True
except Exception:
    _ZMQ_OK = False


class WorkareaLogicTest(unittest.TestCase):
    """无 pyzmq 环境也运行。"""

    def test_error_text_mapping(self):
        self.assertIn("NO_WORKAREA_BEFORE_CREATE_FORBIDDEN_ZONE", tea_bridge.error_text(2))
        self.assertIn("TOO_FEW_POINTS_BEFORE_SAVE_MAP", tea_bridge.error_text(6))
        self.assertIn("Unknow_Build_Map_Error", tea_bridge.error_text(26))
        # 未知数值原样显示
        self.assertEqual(tea_bridge.error_text(99), "99")

    def test_yaw_quat_roundtrip(self):
        for yaw in (0.0, 0.5, -2.7, math.pi, -math.pi):
            x, y, z, w = tea_bridge.yaw_to_quat(yaw)
            back = tea_bridge.quat_to_yaw(x, y, z, w)
            delta = abs(back - yaw)
            delta = min(delta, abs(delta - 2 * math.pi))
            self.assertLess(delta, 1e-6)

    def test_element_names_cover_three_types(self):
        self.assertEqual(tea_bridge.ELEMENT_NAMES[1], "区域")
        self.assertEqual(tea_bridge.ELEMENT_NAMES[2], "通道")
        self.assertEqual(tea_bridge.ELEMENT_NAMES[3], "禁区")

    def test_config_workarea_params(self):
        settings = config.settings
        for key in ("arc_speed", "back_speed", "turn_speed", "arc_radius"):
            self.assertTrue(hasattr(settings, key), key)
        # 圆弧角速度 = 线速度 / 半径 (R=1m)
        self.assertAlmostEqual(settings.arc_speed / settings.arc_radius, 0.4, places=2)
        with self.assertRaises(ValueError):
            config.configure(arc_speed=9.9)
        with self.assertRaises(ValueError):
            config.configure(arc_radius=0.1)
        config.reset()

    def test_cmd_names_complete(self):
        for cmd in range(0, 9):
            self.assertIn(cmd, tea_bridge.CMD_NAMES)


@unittest.skipUnless(_ZMQ_OK, "pyzmq 不可用")
class WorkareaSessionTest(unittest.TestCase):
    """TeaBridge 会话镜像与轨迹抽稀(不连网络, 直接驱动内部状态)。"""

    def _bridge(self):
        # run_proxy=False 且不 start(): 仅用状态容器, 不建 socket
        bridge = tea_bridge.TeaBridge({"enabled": True, "run_proxy": False})
        return bridge

    def test_session_transitions(self):
        bridge = self._bridge()
        self.assertEqual(bridge.summary()["session"]["state"], "idle")
        bridge.begin_session(3)
        summary = bridge.summary()
        self.assertEqual(summary["session"]["state"], "building")
        self.assertEqual(summary["session"]["type"], 3)
        self.assertTrue(bridge.is_building())
        bridge.mark_stale()
        self.assertTrue(bridge.summary()["session"]["stale"])
        bridge.end_session()
        summary = bridge.summary()
        self.assertEqual(summary["session"]["state"], "idle")
        self.assertFalse(bridge.is_building())
        self.assertEqual(summary["trail"], [])

    def test_trail_thinning_and_cap(self):
        bridge = self._bridge()
        bridge.begin_session(1)
        # 直线 0.07m 步进: 0.05m 门限全部保留
        for i in range(100):
            bridge._append_trail(i * 0.07, 0.0)
        self.assertEqual(len(bridge._trail), 100)
        # 原地抖动不记录
        before = len(bridge._trail)
        for _ in range(10):
            bridge._append_trail(6.93 + 0.005, 0.01)
        self.assertEqual(len(bridge._trail), before)
        # 超上限隔点抽稀
        bridge._trail = []
        for i in range(tea_bridge.TRAIL_MAX_POINTS + 500):
            bridge._append_trail(i * 0.06, 0.0)
        self.assertLessEqual(len(bridge._trail), tea_bridge.TRAIL_MAX_POINTS + 1)

    def test_map_snapshot_without_cache(self):
        bridge = self._bridge()
        self.assertEqual(bridge.map_snapshot(), {"version": 0, "map": None})


if __name__ == "__main__":
    unittest.main()
