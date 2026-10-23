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
