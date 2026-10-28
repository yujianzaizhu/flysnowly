import math
import os
import sys
import unittest
from collections import deque

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


def advance(manager, seconds, phases=None, stop=("done",), keep_battery=False):
    for index in range(int(round(seconds / DT))):
        if keep_battery and index % 50 == 0:
            manager.robot.battery = 100.0
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
        params.apply_params(
            max_linear=1.5,
            nav_speed=1.0,
            map_speed=1.0,
            mow_speed=1.3,
            edge_speed=0.85,
        )

    def tearDown(self):
        params.reset_params()

    def test_a_mode_edge_before_stripes(self):
        robot, grid, manager = make_system()
        map_field(robot, grid, manager)
        phases = []
        zone_id = grid.zones[0]["id"]
        manager.start_mowing([zone_id], mode="A")
        advance(manager, 1850.0, phases=phases, keep_battery=True)
        self.assertEqual(manager.phase, "done")
        self.assertLess(phases.index("zone_edge"), phases.index("zone_stripes"))
        self.assertGreaterEqual(grid.zone_coverage(zone_id), 0.80)

    def test_b_mode_stripes_then_final_edge(self):
        robot, grid, manager = make_system()
        map_field(robot, grid, manager)
        phases = []
        zone_id = grid.zones[0]["id"]
        manager.start_mowing([zone_id], mode="B")
        advance(manager, 1850.0, phases=phases, keep_battery=True)
        self.assertEqual(manager.phase, "done")
        self.assertLess(phases.index("zone_plan"), phases.index("zone_edge"))
        self.assertGreaterEqual(phases.count("zone_edge"), 1)
        messages = [entry["message"] for entry in robot.events]
        self.assertTrue(any("B 模式" in message for message in messages))
        self.assertTrue(any("弓字完成，开始沿边割草" in message for message in messages))
        self.assertGreaterEqual(grid.zone_coverage(zone_id), 0.78)

    def test_b_mode_large_obstacle_replans_without_ring(self):
        # 大型障碍物按面积判定（>= 5 m²），这里构造半径 1.4 m 的障碍（≈6.2 m²）
        robot, grid, manager = make_system()
        for row in range(10, 71):
            for col in range(10, 71):
                x, y = grid.center_of(col, row)
                if math.hypot(x - 4.0, y - 4.0) <= 1.40:
                    grid.mark_occupied(col, row)
        robot.pose = [2.1, 4.0, 0.0]
        manager.task = {"type": "mow", "zones": [1], "mode": "B"}
        manager.zone_id = 1
        manager.mowing_mode = "B"
        manager.phase = "zone_stripes"
        manager.segment = ((2.2, 4.0), (3.0, 4.0))
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

    def test_charge_during_edge_restores_unfinished_edge(self):
        robot, grid, manager = make_system()
        manager.task = {"type": "mow", "zones": [1], "mode": "A"}
        manager.zone_id = 1
        manager.zone_stage = "edge"
        manager.edge_points = [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0)]
        manager.edge_index = 2
        manager.edge_start_sim = robot.sim_time - 30.0
        manager.phase = "zone_edge"
        manager.request_charge()
        self.assertIsNotNone(manager.breakpoint)
        manager._resume_from_charge()
        manager._arrive_breakpoint()
        self.assertEqual(manager.phase, "zone_edge")
        self.assertEqual(manager.edge_index, 2)
        self.assertEqual(len(manager.edge_points), 4)

    def test_edge_timeout_scales_with_path_length(self):
        robot, grid, manager = make_system()
        manager.edge_points = [(float(index), 0.0) for index in range(60)]
        self.assertGreater(manager._edge_time_limit(), 120.0)

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
        self.assertGreaterEqual(len(radii), 3)

    def test_world_obstacle_outline_is_exposed(self):
        data = world.world_dict()
        shapes = data["obstacle_shapes"]
        self.assertTrue(shapes)
        self.assertEqual(len(shapes[0]["outline"]), 48)
        for x, y in shapes[0]["outline"]:
            self.assertTrue(0.0 <= x <= data["width"])
            self.assertTrue(0.0 <= y <= data["height"])




    def test_outer_corners_flow_without_forward_reverse(self):
        robot, grid, manager = make_system()
        manager.task = {"type": "mow", "zones": [1], "mode": "A"}
        manager.zone_id = 1
        manager.edge_points = [
            (0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0), (0.0, 0.0)
        ]
        manager.edge_index = 0
        manager._prepare_edge_corners()
        self.assertEqual(manager.edge_corners, [])

    def test_inner_corner_advances_past_vertex_before_reverse(self):
        robot, grid, manager = make_system()
        manager.task = {"type": "mow", "zones": [1], "mode": "A"}
        manager.zone_id = 1
        manager.edge_points = [
            (0.0, 0.0), (4.0, 0.0), (4.0, 1.5), (3.0, 1.5),
            (3.0, 2.5), (4.0, 2.5), (4.0, 4.0), (0.0, 4.0), (0.0, 0.0),
        ]
        manager.edge_index = 0
        manager._prepare_edge_corners()
        self.assertGreaterEqual(len(manager.edge_corners), 2)
        self.assertTrue(all(corner[4] < 0.0 for corner in manager.edge_corners))
        corner = min(
            manager.edge_corners,
            key=lambda item: math.hypot(item[1] - 3.0, item[2] - 1.5),
        )
        manager.edge_index = corner[0]
        robot.pose = [corner[1], corner[2], corner[3]]
        manager.phase = "zone_edge"
        self.assertTrue(manager._maybe_start_corner())
        self.assertEqual(manager.phase, "corner_clean")
        self.assertEqual(manager.corner_stage, "forward")
        self.assertGreater(
            math.hypot(
                manager.corner_vertex[0] - corner[1],
                manager.corner_vertex[1] - corner[2],
            ),
            0.10,
        )
        robot.pose = [
            manager.corner_vertex[0],
            manager.corner_vertex[1],
            corner[3],
        ]
        manager._corner_step(DT, 0.0)
        self.assertEqual(manager.corner_stage, "reverse")
        for _ in range(400):
            manager._corner_step(DT, 0.002)
            if manager.phase == "zone_edge":
                break
        self.assertEqual(manager.phase, "zone_edge")
        self.assertIn(corner[0], manager.edge_corner_done)

    def test_unreachable_stripe_entry_uses_opposite_end(self):
        robot, grid, manager = make_system()
        manager.segments = [((1.0, 2.0), (4.0, 2.0))]
        manager.segment_index = 0
        manager._segment_entry = lambda a, b: (3.9, 2.0) if a == (4.0, 2.0) else None
        manager._set_route = lambda goal, kind: True
        manager._goto_segment(0)
        self.assertTrue(manager.segment_reversed)
        self.assertEqual(manager.segment_end, (1.0, 2.0))
        self.assertEqual(manager.phase, "to_segment")
        self.assertEqual(manager.failed_segments, set())

    def test_unreachable_edge_still_plans_reachable_stripes(self):
        robot, grid, manager = make_system()
        self.assertTrue(grid.build_free_area((world.START["x"], world.START["y"])))
        zone = grid.partition_zones()[0]
        manager.task = {"type": "mow", "zones": [zone["id"]], "mode": "A"}
        manager.zone_id = zone["id"]
        manager.phase = "to_zone"
        manager._find_route = lambda goal: ([], goal, float("inf"))
        manager._plan_to_zone_corner()
        self.assertEqual(manager.route_kind, "to_zone_stripes")
        self.assertTrue(manager.route)
        manager._on_route_done()
        self.assertEqual(manager.zone_stage, "stripes")
        self.assertEqual(manager.phase, "zone_plan")
        self.assertTrue(manager.edge_done)

    def test_mapping_stuck_can_escape_along_history(self):
        robot, grid, manager = make_system()
        robot.pose = [5.0, 5.0, 0.0]
        manager.history = deque(
            [(4.4, 5.0), (4.6, 5.0), (4.8, 5.0), (4.95, 5.0)]
        )
        manager.phase = "map_follow"
        self.assertTrue(manager._begin_escape("map_follow"))
        self.assertEqual(manager.phase, "escape")
        manager._resume_after_escape()
        self.assertEqual(manager.phase, "map_seek")

    def test_goto_segment_uses_reachable_entry_when_start_is_blocked(self):
        robot, grid, manager = make_system()
        self.assertTrue(grid.build_free_area((world.START["x"], world.START["y"])))
        grid.partition_zones()
        manager.task = {"type": "mow", "zones": [1], "mode": "A"}
        manager.zone_id = 1
        start = (world.START["x"], world.START["y"])
        end = (start[0] + 1.2, start[1])
        grid.stamp_obstacle(start[0], start[1], 0.35)
        manager.segments = [(start, end)]
        manager._goto_segment(0)
        self.assertIsNotNone(manager.segment)
        entry = manager.segment[0]
        self.assertFalse(grid.is_blocked_world(entry[0], entry[1]))

    def test_stuck_escape_reverses_along_history(self):
        robot, grid, manager = make_system()
        manager.task = {"type": "mow", "zones": [1], "mode": "A"}
        manager.zone_id = 1
        manager.phase = "zone_edge"
        robot.pose = [5.0, 5.0, 0.0]
        manager.history = deque([(4.4, 5.0), (4.6, 5.0), (4.8, 5.0), (4.95, 5.0)])
        self.assertTrue(manager._begin_escape("zone_edge"))
        self.assertEqual(manager.phase, "escape")
        start_x = robot.pose[0]
        for _ in range(30):
            manager.tick(DT)
        self.assertLess(robot.pose[0], start_x - 0.05)
        for _ in range(500):
            manager.tick(DT)
            if manager.phase != "escape":
                break
        self.assertEqual(manager.phase, "zone_edge")

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
