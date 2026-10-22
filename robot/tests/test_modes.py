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
        advance(manager, 650.0, phases=phases)
        self.assertEqual(manager.phase, "done")
        self.assertLess(phases.index("zone_edge"), phases.index("zone_stripes"))
        self.assertGreaterEqual(grid.zone_coverage(3), 0.80)

    def test_b_mode_stripes_then_final_edge(self):
        robot, grid, manager = make_system()
        map_field(robot, grid, manager)
        phases = []
        manager.start_mowing([3], mode="B")
        advance(manager, 1150.0, phases=phases)
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

    def test_a_mode_edge_actively_detours_known_obstacle(self):
        robot, grid, manager = make_system()
        for row in range(45, 56):
            for col in range(50, 61):
                x, y = grid.center_of(col, row)
                if math.hypot(x - 5.5, y - 5.0) <= 0.35:
                    grid.mark_occupied(col, row)
        robot.pose = [5.0, 5.0, 0.0]
        manager.task = {"type": "mow", "zones": [1], "mode": "A"}
        manager.zone_id = 1
        manager.mowing_mode = "A"
        manager.edge_points = [(5.9, 5.0)]
        manager.edge_index = 0
        manager.phase = "zone_edge"
        manager._edge_step()
        self.assertIn(manager.phase, ("ring", "detour"))


    def test_edge_lookahead_stays_on_current_long_segment(self):
        robot, grid, manager = make_system()
        manager.edge_points = [(0.0, 0.0), (8.0, 0.0), (8.0, 6.0), (0.0, 6.0), (0.0, 0.0)]
        manager.edge_index = 1
        robot.pose = [0.25, 0.02, 0.0]
        target = manager._edge_lookahead_target(0.25, 0.02)
        self.assertAlmostEqual(target[1], 0.0, places=6)
        self.assertGreater(target[0], 0.45)
        self.assertLess(target[0], 0.65)

    def test_pause_during_edge_keeps_edge_after_long_pause(self):
        robot, grid, manager = make_system()
        manager.task = {"type": "mow", "zones": [1], "mode": "A"}
        manager.zone_id = 1
        manager.zone_stage = "edge"
        manager.mowing_mode = "A"
        manager.edge_points = [(4.85, 5.0), (4.70, 4.85), (4.60, 4.70)]
        manager.edge_index = 0
        manager.edge_start_sim = robot.sim_time - 89.0
        robot.pose = [5.0, 5.0, 0.0]
        manager.phase = "zone_edge"

        manager.stop_task("测试暂停")
        advance(manager, 12.0, stop=())
        manager.request_resume()
        self.assertEqual(manager.phase, "zone_edge")
        self.assertLessEqual(robot.sim_time - manager.edge_start_sim, 90.0)
        manager.tick(DT)
        self.assertEqual(manager.phase, "zone_edge")

    def test_finish_ring_resumes_edge_instead_of_planning_stripes(self):
        robot, grid, manager = make_system()
        manager.task = {"type": "mow", "zones": [1], "mode": "A"}
        manager.zone_id = 1
        manager.zone_stage = "edge"
        manager.mowing_mode = "A"
        manager.edge_points = [(2.0, 2.0), (2.5, 2.0), (3.0, 2.0)]
        manager.edge_index = 0
        robot.pose = [1.0, 2.0, 0.0]
        manager.ring_after = "detour"
        manager.detour_source = "edge"
        manager.ring_points = [(1.2, 2.2)]
        manager.ring_circle = {"x": 1.5, "y": 2.0, "r": 0.25, "kind": "small"}
        manager.phase = "ring"

        manager._finish_ring()
        self.assertIn(manager.phase, ("zone_edge", "to_segment"))
        self.assertTrue(manager.edge_points)

    def test_front_hits_zone_ignores_wall_and_detects_obstacle(self):
        robot, grid, manager = make_system()
        grid.zones = [
            {"id": 1, "name": "T", "x0": 0.0, "y0": 0.0, "x1": 10.0, "y1": 10.0, "free": 1000}
        ]
        manager.zone_id = 1
        robot.pose = [5.0, 5.0, 0.0]
        robot.vision = [(0.0, 0.30)]
        self.assertFalse(manager._front_hits_zone())
        col, row = grid.cell_of(5.3, 5.0)
        grid.mark_occupied(col, row)
        self.assertTrue(manager._front_hits_zone())

    def test_ring_route_follows_observed_obstacle_outline(self):
        robot, grid, manager = make_system()
        for col in range(40, 65):
            for row in range(40, 65):
                x, y = grid.center_of(col, row)
                if ((x - 5.2) / 0.8) ** 2 + ((y - 5.0) / 0.45) ** 2 <= 1.0:
                    grid.mark_occupied(col, row)
        circle = {"x": 5.2, "y": 5.0, "r": 0.60, "kind": "large"}
        route = planner.ring_route(circle, (5.2, 3.8), grid=grid)
        self.assertGreaterEqual(len(route), 4)
        radii = {
            round(math.hypot(point[0] - circle["x"], point[1] - circle["y"]), 2)
            for point in route
        }
        self.assertGreater(len(radii), 3)

    def test_world_obstacle_outline_is_exposed(self):
        data = world.world_dict()
        shapes = data["obstacle_shapes"]
        self.assertTrue(shapes)
        self.assertEqual(len(shapes[0]["outline"]), 48)
        for x, y in shapes[0]["outline"]:
            self.assertTrue(0.0 <= x <= data["width"])
            self.assertTrue(0.0 <= y <= data["height"])


    def test_small_patch_cleanup_generates_segments(self):
        robot, grid, manager = make_system()
        zone = {"id": 1, "name": "P", "x0": 2.0, "y0": 2.0, "x1": 5.0, "y1": 5.0, "free": 900}
        grid.zones = [zone]
        grid.mask_ready = True
        for col in range(20, 51):
            for row in range(20, 51):
                index = grid.index(col, row)
                grid.data[index] = 1
                grid.free_mask[index] = 1
                grid.zone_id_map[index] = 1
                grid.mowed[index] = 1
        for col in range(30, 35):
            for row in range(30, 35):
                grid.mowed[grid.index(col, row)] = 0
        manager.task = {"type": "mow", "zones": [1], "mode": "A"}
        manager.zone_id = 1
        manager.mowing_mode = "A"
        manager.zone_stage = "stripes"
        manager.patch_cleanup_done = False
        robot.pose = [2.0, 3.0, 0.0]
        self.assertTrue(manager._plan_patch_cleanup())
        self.assertEqual(manager.zone_stage, "patch")
        self.assertTrue(manager.segments)
        self.assertIn(manager.phase, ("zone_stripes", "to_segment"))

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
