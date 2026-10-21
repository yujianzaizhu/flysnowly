import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import params  # noqa: E402

BACKEND_DIR = params.BACKEND_DIR
if BACKEND_DIR not in sys.path:
    sys.path.insert(0, BACKEND_DIR)

import planner  # noqa: E402
import world  # noqa: E402
from grid_map import GridMap  # noqa: E402
from robot_car import RobotCar  # noqa: E402
from tasks import TaskManager  # noqa: E402

DT = 0.02


def make_system():
    world.select_map(1)
    robot = RobotCar()
    grid = GridMap()
    return robot, grid, TaskManager(robot, grid)


def advance(manager, seconds, phases=None, stop=("done",)):
    for _ in range(int(round(seconds / DT))):
        manager.tick(DT)
        if phases is not None:
            phases.append(manager.phase)
        if manager.phase in stop:
            return


def map_field(robot, grid, manager):
    manager.start_mapping()
    advance(manager, 250.0)


class ModeFlowTest(unittest.TestCase):
    def setUp(self):
        params.apply_params()

    def tearDown(self):
        params.reset_params()

    def test_a_mode_edge_before_stripes(self):
        robot, grid, manager = make_system()
        map_field(robot, grid, manager)
        phases = []
        manager.start_mowing([3], mode="A")
        advance(manager, 400.0, phases=phases)
        self.assertEqual(manager.phase, "done")
        self.assertLess(phases.index("zone_edge"), phases.index("zone_stripes"))
        self.assertGreaterEqual(grid.zone_coverage(3), 0.80)

    def test_b_mode_stripes_then_final_edge(self):
        robot, grid, manager = make_system()
        map_field(robot, grid, manager)
        phases = []
        manager.start_mowing([3], mode="B")
        advance(manager, 700.0, phases=phases)
        self.assertEqual(manager.phase, "done")
        self.assertLess(phases.index("zone_plan"), phases.index("zone_edge"))
        self.assertGreaterEqual(phases.count("zone_edge"), 1)
        messages = [entry["message"] for entry in robot.events]
        self.assertTrue(any("B 模式" in message for message in messages))
        self.assertTrue(any("弓字完成，开始沿边割草" in message for message in messages))
        self.assertGreaterEqual(grid.zone_coverage(3), 0.78)

    def test_b_mode_large_obstacle_replans_without_ring(self):
        robot, grid, manager = make_system()
        for row in range(30, 51):
            for col in range(40, 61):
                x, y = grid.center_of(col, row)
                if math.hypot(x - 4.0, y - 4.0) <= 0.55:
                    grid.mark_occupied(col, row)
        robot.pose = [3.1, 4.0, 0.0]
        manager.task = {"type": "mow", "zones": [1], "mode": "B"}
        manager.zone_id = 1
        manager.mowing_mode = "B"
        manager.phase = "zone_stripes"
        manager.segment = ((3.2, 4.0), (4.5, 4.0))
        manager._begin_detour("stripes")
        self.assertEqual(manager.phase, "zone_plan")
        self.assertIsNone(manager.ring_circle)
        messages = [entry["message"] for entry in robot.events]
        self.assertTrue(any("大型障碍物" in message for message in messages))

    def test_ring_stops_at_zone_boundary(self):
        grid = GridMap()
        zone = {"x0": 1.0, "y0": 1.0, "x1": 5.0, "y1": 5.0, "id": 1}
        circle = {"x": 1.7, "y": 3.0, "r": 0.40, "kind": "small"}
        route = planner.ring_route(circle, (2.4, 3.0), grid=grid, zone=zone)
        self.assertTrue(route)
        self.assertLess(len(route), planner.RING_STEPS)
        for x, y in route:
            self.assertGreaterEqual(x, zone["x0"] + 0.22 - 1e-6)
            self.assertLessEqual(x, zone["x1"] - 0.22 + 1e-6)
            self.assertGreaterEqual(y, zone["y0"] + 0.22 - 1e-6)
            self.assertLessEqual(y, zone["y1"] - 0.22 + 1e-6)

    def test_small_obstacle_triggers_petal_scan_when_lost(self):
        robot, grid, manager = make_system()
        robot.pose = [5.8, 5.0, 0.0]
        robot.vision = [(0.0, 1.0)] * 21
        manager.ring_circle = {"x": 5.0, "y": 5.0, "r": 0.30, "kind": "small"}
        manager.ring_points = [(5.8, 5.0), (5.75, 5.15)]
        manager.ring_started = True
        manager.phase = "ring"
        for _ in range(40):
            manager._ring_step(0.02)
        self.assertGreater(manager.ring_scan_time, 0.0)
        messages = [entry["message"] for entry in robot.events]
        self.assertTrue(any("花瓣" in message for message in messages))

    def test_duration_freezes_after_task_done(self):
        robot, grid, manager = make_system()
        manager.task = {"type": "mow", "zones": [1], "mode": "A"}
        manager.task_start_sim = 10.0
        manager.task_end_sim = 25.0
        robot.sim_time = 100.0
        state = manager.get_state()
        self.assertAlmostEqual(state["task"]["duration"], 15.0, places=1)

    def test_state_exposes_overall_metrics(self):
        robot, grid, manager = make_system()
        map_field(robot, grid, manager)
        manager.start_mowing([3], mode="B")
        advance(manager, 20.0, stop=())
        state = manager.get_state()
        self.assertEqual(state["mowing_mode"], "B")
        self.assertEqual(state["task"]["mode"], "B")
        for key in ("overall_coverage", "coverage", "distance", "duration"):
            self.assertIn(key, state["task"])
        self.assertGreaterEqual(state["task"]["overall_coverage"], 0.0)


if __name__ == "__main__":
    unittest.main()
