import math
import os
import sys
import unittest

BACKEND_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "backend"
)
sys.path.insert(0, BACKEND_DIR)

import planner  # noqa: E402
import world  # noqa: E402
from grid_map import GridMap, OCCUPIED  # noqa: E402


def zone_cell(grid, zone):
    for row in range(grid.rows):
        for col in range(grid.cols):
            if not grid.free_mask[grid.index(col, row)]:
                continue
            x, y = grid.center_of(col, row)
            found = grid.zone_at(x, y)
            if found is not None and found["id"] == zone["id"]:
                return x, y
    return None


class MappingTest(unittest.TestCase):
    def setUp(self):
        world.select_map(1)

    def test_scan_marks_boundary(self):
        grid = GridMap()
        readings = world.vision_scan(7.6, 2.0, 0.0)
        grid.update_from_scan((7.6, 2.0, 0.0), readings, world.VISION_RANGE)
        self.assertGreater(grid.boundary_discovery(), 0.0)
        wall_col, wall_row = grid.cell_of(8.45, 2.0)
        self.assertEqual(grid.data[grid.index(wall_col, wall_row)], OCCUPIED)

    def test_auto_partition_covers_irregular_field(self):
        grid = GridMap()
        self.assertTrue(grid.build_free_area((4.9, 6.2)))
        zones = grid.partition_zones()
        self.assertGreaterEqual(len(zones), 2)
        self.assertLessEqual(len(zones), 9)
        free_total = sum(1 for value in grid.free_mask if value)
        covered = sum(zone["free"] for zone in zones)
        self.assertGreater(covered, free_total * 0.80)
        for zone in zones:
            area = (zone["x1"] - zone["x0"]) * (zone["y1"] - zone["y0"])
            self.assertGreater(zone["free"] / max(area, 1e-6), 0.30)

    def test_mowing_counts_zone_coverage(self):
        grid = GridMap()
        self.assertTrue(grid.build_free_area((4.9, 6.2)))
        zones = grid.partition_zones()
        zone = zones[0]
        point = zone_cell(grid, zone)
        self.assertIsNotNone(point)
        before = grid.zone_coverage(zone["id"])
        grid.mark_mowed(point[0], point[1])
        self.assertGreater(grid.zone_coverage(zone["id"]), before)


    def test_world_defines_large_and_small_irregular_obstacles(self):
        for map_id in range(1, 6):
            world.select_map(map_id)
            kinds = {shape["kind"] for shape in world.OBSTACLE_SHAPES}
            self.assertIn("large", kinds)
            self.assertIn("small", kinds)
            self.assertTrue(
                any(len(shape["parts"]) >= 2 for shape in world.OBSTACLE_SHAPES)
            )
        world.select_map(1)

    def test_obstacle_kind_is_estimated_from_visual_cells(self):
        grid = GridMap()
        for row in range(30, 40):
            for col in range(30, 40):
                grid.mark_occupied(col, row)
        for row in range(60, 63):
            for col in range(60, 63):
                grid.mark_occupied(col, row)
        circles = grid.obstacle_circles(None)
        kinds = {circle["kind"] for circle in circles}
        self.assertIn("large", kinds)
        self.assertIn("small", kinds)

    def test_stamp_obstacle_blocks_navigation(self):
        grid = GridMap()
        grid.stamp_obstacle(10.0, 5.6, 0.5)
        center = grid.cell_of(10.0, 5.6)
        self.assertEqual(grid.data[grid.index(*center)], OCCUPIED)
        self.assertTrue(grid.is_blocked_world(10.7, 5.6))
        self.assertFalse(grid.is_blocked_world(11.2, 5.6))

    def test_obstacle_cluster_detection(self):
        grid = GridMap()
        for index in range(40):
            angle = 2 * math.pi * index / 40.0
            col, row = grid.cell_of(
                5.0 + 0.32 * math.cos(angle), 4.5 + 0.32 * math.sin(angle)
            )
            grid.mark_occupied(col, row)
        circles = grid.obstacle_circles(None)
        self.assertTrue(circles)
        circle = circles[0]
        self.assertAlmostEqual(circle["x"], 5.0, delta=0.15)
        self.assertAlmostEqual(circle["y"], 4.5, delta=0.15)
        self.assertGreaterEqual(circle["r"], 0.30)


class PlannerTest(unittest.TestCase):
    def setUp(self):
        world.select_map(1)

    def test_astar_finds_path(self):
        grid = GridMap()
        path = grid.plan((1.0, 1.0), (4.0, 1.0))
        self.assertGreaterEqual(len(path), 2)
        self.assertLess(math.hypot(path[-1][0] - 4.0, path[-1][1] - 1.0), 0.35)

    def test_astar_avoids_known_obstacle(self):
        grid = GridMap()
        for index in range(40):
            angle = 2 * math.pi * index / 40.0
            col, row = grid.cell_of(
                3.0 + 0.35 * math.cos(angle), 4.0 + 0.35 * math.sin(angle)
            )
            grid.mark_occupied(col, row)
        path = grid.plan((2.0, 4.0), (4.0, 4.0))
        self.assertTrue(path)
        for x, y in path:
            self.assertFalse(grid.is_blocked_world(x, y))

    def test_lane_segments_inside_zone(self):
        grid = GridMap()
        self.assertTrue(grid.build_free_area((4.9, 6.2)))
        zone = max(grid.partition_zones(), key=lambda item: item["free"])
        segments = planner.lane_segments(grid, zone)
        self.assertTrue(segments)
        for (ax, ay), (bx, by) in segments:
            self.assertGreaterEqual(min(ax, bx), zone["x0"] - 1e-6)
            self.assertLessEqual(max(ax, bx), zone["x1"] + 1e-6)
            self.assertGreaterEqual(min(ay, by), zone["y0"] - 1e-6)
            self.assertLessEqual(max(ay, by), zone["y1"] + 1e-6)

    def test_edge_route_is_closed(self):
        grid = GridMap()
        self.assertTrue(grid.build_free_area((4.9, 6.2)))
        zone = grid.partition_zones()[0]
        route = planner.edge_route(zone, (zone["cx"], zone["cy"]))
        self.assertEqual(len(route), 5)
        self.assertEqual(route[0], route[-1])

    def test_lane_order_continues_on_near_side(self):
        segments = [
            ((0.0, 0.0), (4.0, 0.0)),
            ((0.0, 1.0), (4.0, 1.0)),
            ((0.0, 3.0), (4.0, 3.0)),
        ]
        ordered = planner.order_segments(segments, (4.2, 0.0), 0.0)
        self.assertEqual(ordered[0][0], (4.0, 0.0))
        self.assertEqual(ordered[1][0], (0.0, 1.0))

    def test_ring_route_does_not_intersect_obstacle(self):
        circle = {"x": 5.0, "y": 4.5, "r": 0.35}
        route = planner.ring_route(circle, (4.3, 4.5))
        self.assertEqual(len(route), 36)
        for x, y in route:
            self.assertGreater(
                math.hypot(x - 5.0, y - 4.5), 0.35 + world.ROBOT_RADIUS - 0.02
            )


if __name__ == "__main__":
    unittest.main()
