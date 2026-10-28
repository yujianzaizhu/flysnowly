import math
import os
import sys
import unittest

BACKEND_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "backend"
)
sys.path.insert(0, BACKEND_DIR)

import config  # noqa: E402
import world  # noqa: E402
from grid_map import GridMap  # noqa: E402
from robot_car import RobotCar  # noqa: E402
from tasks import TaskManager  # noqa: E402

DT = 0.02


def make_system(map_id=1):
    world.select_map(map_id)
    robot = RobotCar()
    grid = GridMap()
    manager = TaskManager(robot, grid)
    return robot, grid, manager


def advance(manager, seconds, stop=("done",), phases=None, poses=None, keep_battery=False):
    for index in range(int(round(seconds / DT))):
        if keep_battery and index % 50 == 0:
            manager.robot.battery = 100.0
        manager.tick(DT)
        if phases is not None:
            phases.add(manager.phase)
        if poses is not None:
            poses.append((manager.robot.pose[0], manager.robot.pose[1]))
        if manager.phase in stop:
            return


def map_field(robot, grid, manager):
    manager.start_mapping()
    advance(manager, 250.0)


class MappingTaskTest(unittest.TestCase):
    def test_map_loop_only_escapes_when_it_encloses_occupied_cell(self):
        robot, grid, manager = make_system()
        segment = [(5.0, 5.0), (5.0, 7.0), (7.0, 7.0), (7.0, 5.0)]
        col, row = grid.cell_of(6.0, 6.0)
        grid.data[grid.index(col, row)] = 1
        self.assertFalse(manager._loop_encloses_obstacle(segment))
        grid.data[grid.index(col, row)] = 2
        self.assertTrue(manager._loop_encloses_obstacle(segment))

    def test_mapping_is_counter_clockwise_and_returns_to_dock(self):
        robot, grid, manager = make_system()
        poses = []
        manager.start_mapping()
        advance(manager, 250.0, poses=poses)
        self.assertEqual(manager.phase, "done")
        self.assertGreaterEqual(grid.boundary_discovery(), 0.95)
        self.assertEqual(robot.collisions, 0)
        dock_distance = math.hypot(
            robot.pose[0] - world.DOCK["x"], robot.pose[1] - world.DOCK["y"]
        )
        self.assertLess(dock_distance, 0.5)

        area = 0.0
        for index in range(len(poses)):
            x1, y1 = poses[index]
            x2, y2 = poses[(index + 1) % len(poses)]
            area += x1 * y2 - x2 * y1
        self.assertGreater(area, 0.0, "巡边方向应为逆时针")

    def test_mapping_low_battery_returns_to_breakpoint(self):
        robot, grid, manager = make_system()
        robot.battery = 35.0
        phases = set()
        manager.start_mapping()
        advance(manager, 400.0, phases=phases)
        self.assertIn("charging", phases)
        self.assertIn("charge_return", phases)
        self.assertEqual(manager.phase, "done")
        self.assertGreaterEqual(grid.boundary_discovery(), 0.95)
        messages = [entry["message"] for entry in robot.events]
        self.assertTrue(any("电量低于 30%" in message for message in messages))
        self.assertTrue(any("返回断点" in message for message in messages))
        self.assertEqual(robot.collisions, 0)


    def test_mapping_clears_map_edits(self):
        robot, grid, manager = make_system()
        world.add_static_obstacle(5.0, 4.5, 0.4)
        world.add_no_go_zone(3.0, 3.0, 5.0, 4.0)
        self.assertTrue(world.edit_dict()["static"])
        manager.start_mapping()
        self.assertEqual(world.edit_dict()["static"], [])
        self.assertEqual(world.edit_dict()["no_go"], [])
        self.assertEqual(world.NO_GO, [])

    def test_no_go_ring_bypass_generates_counter_clockwise_route(self):
        robot, grid, manager = make_system()
        manager.task = {"type": "mow", "zones": [1], "mode": "A"}
        manager.zone_id = 1
        manager.mowing_mode = "A"
        manager.phase = "zone_edge"
        robot.pose = [2.0, 4.0, 0.0]
        manager.edge_points = [(2.0, 4.0), (3.0, 4.2), (4.0, 4.4), (6.5, 4.0), (7.5, 4.0)]
        manager.edge_index = 0
        world.add_no_go_zone(3.0, 3.0, 5.0, 5.0)
        try:
            zone = manager._no_go_at(3.6, 3.6, 0.0)
            self.assertIsNotNone(zone)
            self.assertTrue(manager._begin_no_go_ring(zone, "zone_edge"))
            self.assertEqual(manager.phase, "no_go_ring")
            points = manager.no_go_ring_points
            self.assertGreaterEqual(len(points), 4)
            # 路径在禁区外，且绕行方向为逆时针（正面积）
            for point in points:
                self.assertFalse(
                    world.in_no_go(point[0], point[1]),
                    "绕行路径不应落在禁区内",
                )
            area = 0.0
            for index in range(len(points)):
                x1, y1 = points[index]
                x2, y2 = points[(index + 1) % len(points)]
                area += x1 * y2 - x2 * y1
            self.assertGreater(area, 0.0)
            manager.edges = None
            manager.phase = "no_go_ring"
            manager.no_go_ring_resume = "zone_edge"
            manager._finish_no_go_ring()
            self.assertEqual(manager.edge_index, 3)
            self.assertEqual(manager.phase, "zone_edge")
        finally:
            world.clear_edits()

    def test_no_go_guard_blocks_forward_motion(self):
        robot, grid, manager = make_system()
        manager.task = {"type": "mow", "zones": [1], "mode": "A"}
        manager.phase = "zone_stripes"
        x, y, theta = manager._pose()
        world.add_no_go_zone(
            x + 0.18, y - 0.6, x + 1.6, y + 0.6
        )
        try:
            v, w = manager._guard_no_go((0.6, 0.2))
            self.assertAlmostEqual(v, 0.0)
            self.assertNotEqual(w, 0.0)
            # 朝远离禁区的方向行驶不受影响
            v2, w2 = manager._guard_no_go((-0.6, 0.2))
            self.assertAlmostEqual(v2, -0.6)
            self.assertAlmostEqual(w2, 0.2)
        finally:
            world.clear_edits()

    def test_finished_mapping_never_restarts_from_stale_breakpoint(self):
        robot, grid, manager = make_system()
        map_field(robot, grid, manager)
        self.assertEqual(manager.phase, "done")
        self.assertTrue(manager.mapping_done)
        manager.breakpoint = {
            "pose": manager._pose(),
            "phase": "map_follow",
            "zone_id": None,
            "zone_index": 0,
            "map_path": [],
            "map_path_dist": 0.0,
        }
        manager._restore_breakpoint()
        self.assertNotIn(manager.phase, ("map_follow", "map_seek", "map_escape"))
        self.assertIsNone(manager.breakpoint)
        advance(manager, 90.0)
        self.assertEqual(manager.phase, "done")
        messages = [entry["message"] for entry in robot.events]
        self.assertEqual(messages.count("开始自动建图：从当前位置逆时针巡边"), 1)

    def test_control_redirects_mapping_phase_after_finish(self):
        robot, grid, manager = make_system()
        map_field(robot, grid, manager)
        manager.phase = "map_follow"
        manager.map_path = [(robot.pose[0], robot.pose[1])]
        manager._control(DT, 0.0)
        self.assertNotEqual(manager.phase, "map_follow")
        advance(manager, 60.0)
        self.assertEqual(manager.phase, "done")


class MowingTaskTest(unittest.TestCase):
    def setUp(self):
        config.configure(
            max_linear=1.5,
            nav_speed=1.0,
            map_speed=1.0,
            mow_speed=1.3,
            edge_speed=0.85,
        )

    def tearDown(self):
        config.reset()

    def test_single_zone_full_coverage(self):
        robot, grid, manager = make_system(3)
        map_field(robot, grid, manager)
        manager.start_mowing([1])
        advance(manager, 900.0)
        self.assertEqual(manager.phase, "done")
        self.assertGreaterEqual(grid.zone_coverage(1), 0.85)
        # 本测试使用加速参数缩短仿真时间；碰撞安全由默认速度测试覆盖。
        self.assertLessEqual(robot.collisions, 30)
        messages = [entry["message"] for entry in robot.events]
        self.assertTrue(any("边缘已完成" in message for message in messages))
        self.assertTrue(any("规划弓字线段" in message for message in messages))
        self.assertTrue(any("规划弓字线段" in message for message in messages))

    def test_b_mode_uses_larger_no_go_margin(self):
        robot, grid, manager = make_system()
        map_field(robot, grid, manager)
        world.add_no_go_zone(4.0, 4.0, 5.0, 5.0)
        try:
            manager.start_mowing([1], mode="B")
            near_col, near_row = grid.cell_of(3.70, 4.5)
            self.assertTrue(grid.cell_blocked_mow(near_col, near_row))
            self.assertFalse(world.in_no_go(3.70, 4.5, 0.0), "0.3 m 外应仍在禁区之外")
        finally:
            world.clear_edits()

    def test_mow_start_restores_obstacle_baseline(self):
        robot, grid, manager = make_system(1)
        map_field(robot, grid, manager)
        zone = max(grid.zones, key=lambda item: item["free"])
        cx, cy = zone["cx"], zone["cy"]
        col, row = grid.cell_of(cx, cy)
        outline_before = [tuple(point) for point in zone["outline"]]
        # 上次割草涂抹出来的假障碍（建图完成时并不存在）
        smear = grid.index(col + 8, row + 8)
        grid.data[smear] = 2
        grid.mow_blocked[smear] = 1
        manager.start_mowing([zone["id"]])
        self.assertEqual(grid.data[smear], 0)
        self.assertFalse(grid.mow_blocked[smear])
        self.assertEqual([tuple(point) for point in grid.zones[0]["outline"]], outline_before)
        messages = [entry["message"] for entry in robot.events]
        self.assertTrue(any("重置区域内障碍涂抹" in message for message in messages))

    def test_auto_recharge_resumes_mowing(self):
        robot, grid, manager = make_system(3)
        map_field(robot, grid, manager)
        robot.battery = 31.0
        phases = set()
        manager.start_mowing([1])
        advance(manager, 900.0, phases=phases)
        self.assertIn("charge_go", phases)
        self.assertTrue("charging" in phases or "charge_return" in phases)
        self.assertIn("charge_return", phases)
        self.assertEqual(manager.phase, "done")
        self.assertGreaterEqual(grid.zone_coverage(1), 0.85)

    def test_manual_recharge_waits_for_resume(self):
        robot, grid, manager = make_system(3)
        map_field(robot, grid, manager)
        manager.start_mowing([1])
        advance(manager, 10.0, stop=("done",))
        coverage_before = grid.zone_coverage(1)
        manager.request_charge()
        advance(manager, 40.0, stop=("done",))
        self.assertEqual(manager.phase, "charging")
        self.assertIsNotNone(manager.breakpoint)
        self.assertGreater(robot.battery, 90.0)
        self.assertGreaterEqual(coverage_before, 0.0)

        manager.request_resume()
        advance(manager, 900.0)
        self.assertEqual(manager.phase, "done")
        self.assertGreaterEqual(grid.zone_coverage(1), 0.85)

    def test_zone_order_and_multiple_zones(self):
        robot, grid, manager = make_system(3)
        map_field(robot, grid, manager)
        robot.battery = 100.0
        self.assertGreaterEqual(len(grid.zones), 2)
        smallest = sorted(grid.zones, key=lambda zone: zone["free"])[:2]
        order = [smallest[1]["id"], smallest[0]["id"]]
        manager.start_mowing(order)
        self.assertEqual(manager.zone_order, order)
        advance(manager, 1400.0, stop=("done", "to_dock"), keep_battery=True)
        self.assertIn(manager.phase, ("to_dock", "done"))
        for zone_id in order:
            self.assertGreaterEqual(grid.zone_coverage(zone_id), 0.83)
        messages = [entry["message"] for entry in robot.events]
        positions = [
            next(
                index
                for index, message in enumerate(messages)
                if "区域 {} 割草完成".format(zone_id) in message
            )
            for zone_id in order
        ]
        self.assertLess(positions[0], positions[1])

    def test_stop_task_returns_to_idle(self):
        robot, grid, manager = make_system(3)
        manager.start_mapping()
        advance(manager, 3.0, stop=())
        manager.stop_task("测试停止", resumable=False)
        self.assertIsNone(manager.task)
        self.assertEqual(manager.phase, "idle")

    def test_stopped_task_can_continue(self):
        robot, grid, manager = make_system(3)
        map_field(robot, grid, manager)
        robot.battery = 100.0
        manager.start_mowing([1])
        advance(manager, 25.0, stop=())
        manager.stop_task("控制台暂停")
        self.assertEqual(manager.phase, "stopped")
        self.assertIsNotNone(manager.breakpoint)
        coverage = grid.zone_coverage(1)
        advance(manager, 8.0, stop=())
        self.assertAlmostEqual(grid.zone_coverage(1), coverage, places=4)
        manager.request_resume()
        self.assertIn(manager.phase, ("zone_edge", "zone_plan", "to_zone"))
        advance(manager, 900.0)
        self.assertEqual(manager.phase, "done")
        self.assertGreaterEqual(grid.zone_coverage(1), 0.80)


if __name__ == "__main__":
    unittest.main()
