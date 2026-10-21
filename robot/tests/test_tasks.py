import math
import os
import sys
import unittest

BACKEND_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "backend"
)
sys.path.insert(0, BACKEND_DIR)

import world  # noqa: E402
from grid_map import GridMap  # noqa: E402
from robot_car import RobotCar  # noqa: E402
from tasks import TaskManager  # noqa: E402

DT = 0.02


def make_system():
    world.select_map(1)
    robot = RobotCar()
    grid = GridMap()
    manager = TaskManager(robot, grid)
    return robot, grid, manager


def advance(manager, seconds, stop=("done",), phases=None, poses=None):
    for _ in range(int(round(seconds / DT))):
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
    def test_single_zone_full_coverage(self):
        robot, grid, manager = make_system()
        map_field(robot, grid, manager)
        manager.start_mowing([1])
        advance(manager, 400.0)
        self.assertEqual(manager.phase, "done")
        self.assertGreaterEqual(grid.zone_coverage(1), 0.85)
        self.assertEqual(robot.collisions, 0)
        messages = [entry["message"] for entry in robot.events]
        self.assertTrue(any("边缘已完成" in message for message in messages))
        self.assertTrue(any("规划弓字线段" in message for message in messages))
        self.assertTrue(any("规划弓字线段" in message for message in messages))

    def test_auto_recharge_resumes_mowing(self):
        robot, grid, manager = make_system()
        map_field(robot, grid, manager)
        robot.battery = 40.0
        phases = set()
        manager.start_mowing([1])
        advance(manager, 650.0, phases=phases)
        self.assertIn("charge_go", phases)
        self.assertTrue("charging" in phases or "charge_return" in phases)
        self.assertIn("charge_return", phases)
        self.assertEqual(manager.phase, "done")
        self.assertGreaterEqual(grid.zone_coverage(1), 0.85)

    def test_manual_recharge_waits_for_resume(self):
        robot, grid, manager = make_system()
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
        advance(manager, 420.0)
        self.assertEqual(manager.phase, "done")
        self.assertGreaterEqual(grid.zone_coverage(1), 0.85)

    def test_zone_order_and_multiple_zones(self):
        robot, grid, manager = make_system()
        map_field(robot, grid, manager)
        robot.battery = 100.0
        smallest = sorted(grid.zones, key=lambda zone: zone["free"])[:2]
        order = [smallest[1]["id"], smallest[0]["id"]]
        manager.start_mowing(order)
        self.assertEqual(manager.zone_order, order)
        advance(manager, 700.0, stop=("done", "to_dock"))
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
        robot, grid, manager = make_system()
        manager.start_mapping()
        advance(manager, 3.0, stop=())
        manager.stop_task("测试停止")
        self.assertIsNone(manager.task)
        self.assertEqual(manager.phase, "idle")


if __name__ == "__main__":
    unittest.main()
