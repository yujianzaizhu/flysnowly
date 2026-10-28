"""static_config 解析与导入测试。"""

import os
import sys
import tempfile
import unittest

BACKEND_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "backend"
)
sys.path.insert(0, BACKEND_DIR)

import config  # noqa: E402
import static_config  # noqa: E402


class YamlSubsetTest(unittest.TestCase):
    def test_parses_nested_map_and_list(self):
        text = """
# comment
topic:
  topic_a:
    - name_a
    - 10001
  topic_b:
    - name_b
    - 10002
Behavior:
  RotateBehavior:
    max_vel_theta: 0.50
    rot_distance: 1.5
prohibided_angles_to_rotate:
  - 170
  - 180
flag: true
"""
        doc = static_config.parse_yaml(text)
        self.assertEqual(doc["topic"]["topic_a"], ["name_a", 10001])
        self.assertAlmostEqual(doc["Behavior"]["RotateBehavior"]["rot_distance"], 1.5)
        self.assertEqual(doc["prohibided_angles_to_rotate"], [170, 180])
        self.assertTrue(doc["flag"])

    def test_collect_extracts_interfaces_and_params(self):
        parsed = static_config.collect()
        if not parsed["files"]:
            self.skipTest("static_config 目录不存在")
        self.assertGreater(len(parsed["topics"]), 50)
        self.assertEqual(parsed["topics"]["topic_robot_message_cmd_vel"]["name"], "cmd_vel")
        self.assertEqual(parsed["services"]["service_nav_map_load"]["name"], "map_load")
        self.assertEqual(parsed["services"]["service_nav_map_load"]["port"], 20020)
        self.assertEqual(parsed["services"]["service_nav_record_path"]["port"], 20015)
        self.assertEqual(parsed["services"]["service_nav_get_grid_map"]["port"], 20017)
        self.assertEqual(parsed["actions"]["action_nav_task"]["port"], 25009)
        params = parsed["params"]
        for key in (
            "nav_speed",
            "mow_speed",
            "edge_speed",
            "record_interval",
            "xy_goal_tolerance",
            "region_dilate_size",
            "robot_long",
        ):
            self.assertIn(key, params)
        self.assertGreater(params["nav_speed"], 0.0)
        self.assertGreater(params["record_interval"], 0.0)


class ApplyConfigTest(unittest.TestCase):
    def setUp(self):
        config.reset()
        static_config.NAV_RUNTIME.update(
            {"loaded": False, "params": {}, "config_values": {}}
        )

    def tearDown(self):
        config.reset()
        static_config.NAV_RUNTIME.update(
            {"loaded": False, "params": {}, "config_values": {}}
        )

    def test_apply_maps_navigation_speeds_into_settings(self):
        parsed, runtime = static_config.load_and_apply()
        if not parsed["files"]:
            self.skipTest("static_config 目录不存在")
        self.assertTrue(runtime["loaded"])
        self.assertAlmostEqual(config.settings.nav_speed, parsed["params"]["nav_speed"])
        self.assertAlmostEqual(config.settings.mow_speed, parsed["params"]["mow_speed"])
        self.assertAlmostEqual(config.settings.edge_speed, parsed["params"]["edge_speed"])
        summary = static_config.summary()
        self.assertTrue(summary["loaded"])
        self.assertGreater(summary["topic_count"], 50)
        self.assertIn("record_interval", summary["params"])

    def test_apply_rejects_missing_directory(self):
        empty = tempfile.mkdtemp(prefix="empty_static_")
        with self.assertRaises(ValueError):
            static_config.load_and_apply(empty)


if __name__ == "__main__":
    unittest.main()
