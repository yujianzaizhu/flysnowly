"""导航节点（planner / dock）与仿真桥接的接口测试。"""

import os
import sys
import unittest

BACKEND_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "backend"
)
sys.path.insert(0, BACKEND_DIR)

import config  # noqa: E402
import map_import  # noqa: E402
import nav_bus  # noqa: E402
import nav_bridge  # noqa: E402
import static_config  # noqa: E402
import world  # noqa: E402
from grid_map import GridMap  # noqa: E402
from robot_car import RobotCar  # noqa: E402
from tasks import TaskManager  # noqa: E402


def make_system(map_id=3):
    world.select_map(map_id)
    robot = RobotCar()
    grid = GridMap()
    nav = nav_bridge.NavBridge(robot, grid)
    manager = TaskManager(robot, grid, nav=nav)
    return robot, grid, nav, manager


def prepare_zones(robot, grid):
    grid.build_free_area((robot.pose[0], robot.pose[1]))
    return grid.partition_zones()


class ActionSemanticsTest(unittest.TestCase):
    def setUp(self):
        self.robot, self.grid, self.nav, self.manager = make_system()
        prepare_zones(self.robot, self.grid)

    def call(self, name, cmd, goal=None):
        return self.nav.bus.call(name, {"cmd": cmd, "goal": goal or {}}, client="test")

    def test_start_rejects_second_task_and_supports_pause_continue(self):
        start = self.call("nav_task_action", nav_bus.ACTION_START, {"mode": nav_bus.MODE_NAVIGATION, "target_pose": {"x": 3.0, "y": 3.0}})
        self.assertEqual(start["status"], nav_bus.RESULT_SUCCEED)
        again = self.call("nav_task_action", nav_bus.ACTION_START, {"mode": nav_bus.MODE_NAVIGATION, "target_pose": {"x": 3.0, "y": 3.0}})
        self.assertEqual(again["status"], nav_bus.RESULT_REJECTED)
        pause = self.call("nav_task_action", nav_bus.ACTION_PAUSE)
        self.assertEqual(pause["status"], nav_bus.RESULT_SUCCEED)
        self.assertEqual(self.nav.planner.main_state, nav_bus.NAV_PAUSE)
        cont = self.call("nav_task_action", nav_bus.ACTION_CONTINUE)
        self.assertEqual(cont["status"], nav_bus.RESULT_SUCCEED)
        self.assertEqual(self.nav.planner.main_state, nav_bus.NAV_RUNNING)
        finish = self.call("nav_task_action", nav_bus.ACTION_FINISHED)
        self.assertEqual(finish["status"], nav_bus.RESULT_SUCCEED)
        result = self.nav.bus.last("nav_task_action_result")
        self.assertEqual(result["status"], nav_bus.GOAL_SUCCEEDED)

    def test_cancel_publishes_preempted_and_unknown_cmd_unsupported(self):
        self.call("nav_task_action", nav_bus.ACTION_START, {"mode": nav_bus.MODE_NAVIGATION, "target_pose": {"x": 3.0, "y": 3.0}})
        cancel = self.call("nav_task_action", nav_bus.ACTION_CANCEL)
        self.assertEqual(cancel["status"], nav_bus.RESULT_SUCCEED)
        self.assertEqual(self.nav.bus.last("nav_task_action_result")["status"], nav_bus.GOAL_PREEMPTED)
        unknown = self.call("nav_task_action", 99)
        self.assertEqual(unknown["status"], nav_bus.RESULT_UNSUPPORTED)

    def test_navigation_publishes_task_plan_path_and_nav_status(self):
        self.call("nav_task_action", nav_bus.ACTION_START, {"mode": nav_bus.MODE_NAVIGATION, "target_pose": {"x": 4.0, "y": 4.0}})
        plan = self.nav.bus.last("task_plan_path")
        self.assertEqual(plan["type"], "nav_msgs/Path")
        self.assertGreater(len(plan["poses"]), 0)
        status = self.nav.bus.last("nav_status")
        self.assertEqual(status["main_state"], nav_bus.NAV_RUNNING)
        self.assertAlmostEqual(plan["poses"][-1]["x"], 4.0, delta=0.15)

    def test_coverage_action_publishes_nav_task_path(self):
        zone = self.grid.zones[0]
        response = self.call(
            "nav_task_action",
            nav_bus.ACTION_START,
            {"mode": nav_bus.MODE_COVERAGE, "zone_id": zone["id"], "bow_type": nav_bus.BOW_HORIZONTAL},
        )
        self.assertEqual(response["status"], nav_bus.RESULT_SUCCEED)
        message = self.nav.bus.last("nav_task_path")
        self.assertEqual(message["type"], "nav_msgs/PathWithIndex")
        self.assertGreater(len(message["poses"]), 1)
        segments = self.nav.plan_coverage(zone, "h")
        self.assertTrue(segments)
        for a, b in segments:
            self.assertNotEqual(a, b)

    def test_unknown_zone_is_rejected_and_reports_aborted(self):
        response = self.call("nav_task_action", nav_bus.ACTION_START, {"mode": nav_bus.MODE_COVERAGE, "zone_id": 999})
        self.assertEqual(response["status"], nav_bus.RESULT_FAILED)
        self.assertEqual(self.nav.bus.last("nav_task_action_result")["status"], nav_bus.GOAL_ABORTED)


class DockNodeTest(unittest.TestCase):
    def setUp(self):
        self.robot, self.grid, self.nav, self.manager = make_system()
        prepare_zones(self.robot, self.grid)

    def test_visual_egway_publishes_egway_poses(self):
        zone = self.grid.zones[0]
        response = self.nav.call_action(
            "dock", "visual_egway_action", {"cmd": nav_bus.ACTION_START, "goal": {"docking_waypoint": zone["id"]}}
        )
        self.assertEqual(response["status"], nav_bus.RESULT_SUCCEED)
        message = self.nav.bus.last("egway_poses")
        self.assertEqual(message["type"], "nav_msgs/Path")
        self.assertGreater(len(message["poses"]), 10)
        edge = self.nav.plan_visual_egway(zone)
        self.assertGreater(len(edge), 10)

    def test_autodock_publishes_plan_and_result(self):
        self.assertTrue(self.nav.begin_autodock())
        self.assertEqual(self.nav.dock_kind, "autodock")
        plan = self.nav.bus.last("global_plan")
        start_pose = self.nav.bus.last("recharge_start_pose")
        self.assertGreater(len(plan["poses"]), 0)
        self.assertEqual(start_pose["type"], "nav_msgs/Path")
        self.nav.finish_autodock(True, "已停靠")
        result = self.nav.bus.last("autodock_action_result")
        self.assertEqual(result["status"], nav_bus.GOAL_SUCCEEDED)
        self.assertTrue(result["result"]["is_success"])
        self.assertIsNone(self.nav.dock_kind)

    def test_leave_dock_publishes_exit_plan(self):
        self.assertTrue(self.nav.begin_leave_dock())
        plan = self.nav.bus.last("global_plan")
        self.assertGreaterEqual(len(plan["poses"]), 1)
        self.nav.finish_leave_dock(True)
        result = self.nav.bus.last("leave_dock_action_result")
        self.assertEqual(result["status"], nav_bus.GOAL_SUCCEEDED)


class BridgeIoTest(unittest.TestCase):
    def setUp(self):
        self.robot, self.grid, self.nav, self.manager = make_system()
        prepare_zones(self.robot, self.grid)

    def test_tick_publishes_documented_inputs_and_cmd_vel(self):
        self.manager.tick(0.02)
        for topic, kind in (
            ("fuse_pose_3d", "usr_define_msgs/Odom"),
            ("combined", "geometry_msgs/PoseWithCovarianceStamped"),
            ("imu_data", "usr_define_msgs/Imu"),
            ("tf_dynamic", "tf2_msgs/TFMessage"),
            ("odom_hardware_signal", "usr_define_msgs/HardwareSignal"),
            ("is_edgeway", "std_msgs/Int32"),
            ("region_area", "std_msgs/Float32"),
            ("cmd_vel", "geometry_msgs/Twist"),
        ):
            message = self.nav.bus.last(topic)
            self.assertIsNotNone(message, topic)
            self.assertEqual(message["type"], kind, topic)

    def test_heartbeat_and_nav_status_are_published(self):
        self.manager.tick(0.02)
        heartbeat = self.nav.bus.last("process_heart_beat")
        self.assertEqual(heartbeat["type"], "usr_define_health/ProcessHeartBeat")
        self.assertIn(heartbeat["heartbeat_id"], (800012, 800020, 800021, 800022))
        status = self.nav.bus.last("nav_status")
        self.assertIn("main_state_name", status)

    def test_snapshot_lists_nodes_topics_and_service_ports(self):
        snapshot = self.nav.snapshot()
        names = {node["name"] for node in snapshot["nodes"]}
        self.assertEqual(names, {"planner", "dock", "region_manager", "trajectory_map"})
        ports = {service["name"]: service["port"] for service in snapshot["services"]}
        self.assertEqual(ports["nav_task_action"], 25009)
        self.assertEqual(ports["autodock_action"], 25001)
        self.assertEqual(ports["visual_egway_action"], 25005)
        self.assertEqual(ports["egway_coverage_action"], 25006)
        topics = {topic["name"] for topic in snapshot["topics"]}
        self.assertIn("cmd_vel", topics)
        self.assertIn("nav_status", topics)


class RegionTrajectoryNodeTest(unittest.TestCase):
    def setUp(self):
        self.robot, self.grid, self.nav, self.manager = make_system()

    def test_region_manager_builds_map_and_serves_zones(self):
        response = self.nav.load_map(4)
        self.assertTrue(response["result"])
        self.assertTrue(response["map_id_array"])
        zones = self.grid.zones
        self.assertTrue(zones)
        zone = zones[0]
        grid_map = self.nav.zone_map(zone)
        self.assertEqual(grid_map["type"], "nav_msgs/OccupancyGrid")
        info = grid_map["info"]
        self.assertEqual(len(grid_map["data"]), info["width"] * info["height"])
        self.assertIn(-1, grid_map["data"])
        self.assertIn(0, grid_map["data"])
        map_info = self.nav.map_info()
        self.assertEqual(len(map_info["sub_maps"]), len(zones))
        self.assertEqual(map_info["station_pose_x"], world.DOCK["x"])
        self.assertIsNotNone(self.nav.bus.last("map_info"))
        self.assertIsNotNone(self.nav.bus.last("charge_pose"))

    def test_planner_coverage_consumes_region_manager_map(self):
        self.nav.load_map(4)
        zone = self.grid.zones[0]
        calls = self.nav.bus.service("get_grid_map")["calls"]
        segments = self.nav.plan_coverage(zone, "h")
        self.assertTrue(segments)
        self.assertGreater(self.nav.bus.service("get_grid_map")["calls"], calls)
        self.assertEqual(self.nav.planner.plan["map_source"], "region_manager/get_grid_map")
        self.assertIsNotNone(self.nav.planner.plan["map_area"])

    def test_trajectory_map_records_and_saves(self):
        self.assertTrue(self.nav.begin_mapping())
        self.assertTrue(self.nav.trajectory.mapping)
        recorded = 0
        for step in range(10):
            x = self.robot.pose[0] + 0.1 * (step + 1)
            if self.nav.record_mapping_point((x, self.robot.pose[1], 0.0)):
                recorded += 1
        self.assertGreater(recorded, 0)
        zones = self.nav.finish_mapping()
        self.assertFalse(self.nav.trajectory.mapping)
        self.assertTrue(zones)
        self.assertIsNotNone(self.nav.bus.last("mapping_path_info"))
        self.assertTrue(self.nav.bus.last("trajectory_map_change")["data"])
        self.assertGreaterEqual(self.nav.trajectory.save_count, 1)

    def test_follow_plan_returns_recorded_path(self):
        self.nav.bus.call(
            "record_path", {"command": 1, "path_name": "auto_map", "pose": {"x": 1.0, "y": 1.0}}
        )
        self.nav.bus.call("record_path", {"command": 6, "pose": {"x": 1.5, "y": 1.0}})
        self.nav.bus.call("record_path", {"command": 3, "path_name": "auto_map"})
        response = self.nav.bus.call("follow_plan", {"follow_path_name": "auto_map"})
        self.assertGreaterEqual(len(response["plan"]["poses"]), 1)

    def test_snapshot_lists_four_nodes(self):
        names = {node["name"] for node in self.nav.snapshot()["nodes"]}
        self.assertEqual(names, {"planner", "dock", "region_manager", "trajectory_map"})


class StaticConfigFlowTest(unittest.TestCase):
    """导入 static_config 参数 + 真实地图后，节点主导完成建图/分区/割草。"""

    def setUp(self):
        config.reset()
        self.robot, self.grid, self.nav, self.manager = make_system(1)

    def tearDown(self):
        config.reset()

    def test_imported_params_drive_nodes(self):
        parsed, runtime = static_config.load_and_apply()
        if not parsed["files"]:
            self.skipTest("static_config 目录不存在")
        self.manager.nav.record_interval = runtime["params"]["record_interval"]
        self.nav.trajectory.set_record_interval(runtime["params"]["record_interval"])
        self.assertAlmostEqual(config.settings.nav_speed, runtime["params"]["nav_speed"])
        self.assertAlmostEqual(config.settings.edge_speed, runtime["params"]["edge_speed"])
        summary = self.nav.snapshot()["static_config"]
        self.assertTrue(summary["loaded"])
        self.assertGreater(summary["topic_count"], 50)

    def test_full_flow_with_imported_map_and_nodes(self):
        parsed, _runtime = static_config.load_and_apply()
        if not parsed["files"]:
            self.skipTest("static_config 目录不存在")
        map_dir = os.path.join(
            os.path.dirname(BACKEND_DIR), os.pardir, "data", "map", "map_1"
        )
        if not os.path.isdir(map_dir):
            self.skipTest("真实地图目录不存在")
        definition = map_import.build_map(map_dir, "node_flow", max_extent=16.0)
        world.register_map(definition)
        world.select_map(definition["id"])
        robot = RobotCar()
        grid = GridMap()
        nav = nav_bridge.NavBridge(robot, grid)
        nav.apply_static_config()
        manager = TaskManager(robot, grid, nav=nav)
        self.addCleanup(world.delete_map, definition["id"])
        self.addCleanup(world.select_map, 3)

        # 建图记录 + 地图装载全部走节点
        nav.begin_mapping()
        nav.record_mapping_point((robot.pose[0], robot.pose[1], robot.pose[2]))
        zones = nav.finish_mapping()
        self.assertTrue(zones)
        self.assertGreaterEqual(len(zones), 1)

        manager.start_mowing(None, mode="B")
        self.assertEqual(manager.task["type"], "mow")
        self.assertEqual(manager.phase, "to_zone")

        planned = False
        for _ in range(4000):
            manager.tick(0.02)
            if manager.phase in ("zone_stripes", "zone_plan"):
                planned = True
            if manager.phase in ("done", "to_dock"):
                break
        self.assertTrue(planned, "节点应完成区域弓字规划")
        self.assertIsNotNone(nav.bus.last("nav_task_path"))
        self.assertIsNotNone(nav.bus.last("task_plan_path"))
        self.assertIsNotNone(nav.bus.last("cmd_vel"))


class NodeModeSwitchTest(unittest.TestCase):
    def setUp(self):
        self.robot, self.grid, self.nav, self.manager = make_system()
        prepare_zones(self.robot, self.grid)

    def test_node_mode_is_enabled_by_default(self):
        self.assertTrue(self.nav.enabled)
        self.assertTrue(self.manager._node_mode())

    def test_disabled_mode_uses_system_algorithms(self):
        self.nav.set_enabled(False)
        self.assertFalse(self.manager._node_mode())
        calls = self.nav.bus.service("nav_task_action")["calls"]
        path, endpoint, deviation = self.manager._find_route(
            (self.robot.pose[0] + 1.0, self.robot.pose[1])
        )
        self.assertTrue(path)
        self.assertEqual(self.nav.bus.service("nav_task_action")["calls"], calls)
        zone = self.grid.zones[0]
        lanes = self.manager._node_lanes(zone, "COVERAGE")
        self.assertTrue(lanes)
        self.assertEqual(self.nav.bus.service("nav_task_action")["calls"], calls)
        self.assertIsNone(self.nav.command("nav", (0.0, 0.0, 0.0), (1.0, 0.0), 0.5))

    def test_enabled_mode_uses_node_services_and_commands(self):
        self.nav.set_enabled(True)
        calls = self.nav.bus.service("nav_task_action")["calls"]
        self.manager._find_route((self.robot.pose[0] + 1.0, self.robot.pose[1]))
        self.assertGreater(self.nav.bus.service("nav_task_action")["calls"], calls)
        command = self.nav.command(
            "nav", (self.robot.pose[0], self.robot.pose[1], 0.0),
            (self.robot.pose[0] + 1.0, self.robot.pose[1]), 0.5,
        )
        self.assertIsNotNone(command)
        self.assertIn(command["node"], ("planner", "dock"))
        self.assertGreater(command["v"], 0.0)
        self.assertEqual(self.nav.bus.last("cmd_vel")["type"], "geometry_msgs/Twist")

    def test_cmd_vel_published_by_node_when_enabled(self):
        self.nav.set_enabled(True)
        self.manager.tick(0.02)
        self.nav.command("stripe", (2.0, 2.0, 0.0), (3.0, 2.0), 0.5)
        message = self.nav.bus.last("cmd_vel")
        publishers = [
            topic["publishers"]
            for topic in self.nav.snapshot()["topics"]
            if topic["name"] == "cmd_vel"
        ][0]
        self.assertIn("planner", publishers)
        self.assertGreater(message["linear"]["x"], 0.0)

    def test_disabled_cmd_vel_comes_from_sim(self):
        self.nav.set_enabled(False)
        self.manager.tick(0.02)
        publishers = [
            topic["publishers"]
            for topic in self.nav.snapshot()["topics"]
            if topic["name"] == "cmd_vel"
        ][0]
        self.assertIn("sim", publishers)


class TaskManagerNodeIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.robot, self.grid, self.nav, self.manager = make_system()
        prepare_zones(self.robot, self.grid)

    def test_route_planning_goes_through_planner_node(self):
        calls_before = self.nav.bus.service("nav_task_action")["calls"]
        path, endpoint, deviation = self.manager._find_route(
            (self.robot.pose[0] + 1.0, self.robot.pose[1])
        )
        self.assertTrue(path)
        self.assertGreater(self.nav.bus.service("nav_task_action")["calls"], calls_before)
        self.assertIsNotNone(self.nav.bus.last("task_plan_path"))
        self.assertAlmostEqual(deviation, 0.0, places=6)

    def test_zone_edge_uses_dock_visual_egway(self):
        zone = self.grid.zones[0]
        calls_before = self.nav.bus.service("visual_egway_action")["calls"]
        points = self.manager._zone_edge_points(zone)
        self.assertGreater(len(points), 10)
        self.assertGreater(self.nav.bus.service("visual_egway_action")["calls"], calls_before)
        published = self.nav.bus.last("egway_poses")["poses"]
        self.assertGreater(len(published), 10)

    def test_stripes_use_planner_coverage(self):
        zone = self.grid.zones[0]
        calls_before = self.nav.bus.service("nav_task_action")["calls"]
        lanes = self.manager._node_lanes(zone, "COVERAGE")
        self.assertTrue(lanes)
        self.assertGreater(self.nav.bus.service("nav_task_action")["calls"], calls_before)

    def test_state_exposes_nav_snapshot(self):
        state = self.manager.get_state()
        self.assertIn("nav", state)
        self.assertEqual(len(state["nav"]["nodes"]), 4)


if __name__ == "__main__":
    unittest.main()
