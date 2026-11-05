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
from grid_map import GridMap, OCCUPIED, UNKNOWN  # noqa: E402


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


def _segment_hits_circle(segment, cx, cy, radius):
    """线段是否与圆相交（只判中点与两个端点已足够用于测试）。"""
    (ax, ay), (bx, by) = segment
    for ratio in (0.0, 0.25, 0.5, 0.75, 1.0):
        x = ax + (bx - ax) * ratio
        y = ay + (by - ay) * ratio
        if math.hypot(x - cx, y - cy) <= radius:
            return True
    return False


class MappingTest(unittest.TestCase):
    def setUp(self):
        world.select_map(1)

    def test_scan_marks_boundary(self):
        grid = GridMap()
        x = 8.5 * world.MAP_SCALE - 0.70
        y = 2.0 * world.MAP_SCALE
        readings = world.vision_scan(x, y, 0.0)
        grid.update_from_scan((x, y, 0.0), readings, world.VISION_RANGE)
        self.assertGreater(grid.boundary_discovery(), 0.0)
        wall_col, wall_row = grid.cell_of(8.5 * world.MAP_SCALE - 0.06, y)
        self.assertEqual(grid.data[grid.index(wall_col, wall_row)], OCCUPIED)

    def test_auto_partition_covers_irregular_field(self):
        grid = GridMap()
        self.assertTrue(grid.build_free_area((world.START["x"], world.START["y"])))
        zones = grid.partition_zones()
        self.assertGreaterEqual(len(zones), 1)
        self.assertLessEqual(len(zones), 8)
        free_total = sum(1 for value in grid.free_mask if value)
        covered = sum(zone["free"] for zone in zones)
        self.assertGreater(covered, free_total * 0.80)
        for zone in zones:
            area = (zone["x1"] - zone["x0"]) * (zone["y1"] - zone["y0"])
            self.assertGreater(zone["free"] / max(area, 1e-6), 0.30)

    def test_partition_splits_dumbbell_but_keeps_regular_component(self):
        grid = GridMap()
        grid.mask_ready = True
        for row in range(20, 71):
            for col in range(10, 61):
                grid.free_mask[grid.index(col, row)] = 1
            for col in range(95, 146):
                grid.free_mask[grid.index(col, row)] = 1
        for row in range(43, 48):
            for col in range(61, 95):
                grid.free_mask[grid.index(col, row)] = 1
        zones = grid.partition_zones()
        self.assertEqual(len(zones), 2)
        for zone in zones:
            self.assertGreaterEqual(zone["free"] * grid.cell * grid.cell, 20.0)

        grid = GridMap()
        grid.mask_ready = True
        for row in range(20, 71):
            for col in range(20, 71):
                grid.free_mask[grid.index(col, row)] = 1
        zones = grid.partition_zones()
        self.assertEqual(len(zones), 1)

    def test_obstacle_baseline_restores_mapping_state(self):
        grid = GridMap()
        self.assertTrue(grid.build_free_area((world.START["x"], world.START["y"])))
        zones = grid.partition_zones()
        self.assertTrue(zones)
        zone = max(zones, key=lambda item: item["free"])
        cx, cy = zone["cx"], zone["cy"]
        col, row = grid.cell_of(cx, cy)
        index = grid.index(col, row)
        grid.stamp_obstacle(cx, cy, 0.30)
        self.assertEqual(grid.data[index], OCCUPIED)
        self.assertTrue(grid.capture_obstacle_baseline())
        # 割草期间出现新的障碍涂抹
        other = grid.index(col + 12, row + 12)
        grid.data[other] = OCCUPIED
        grid.mow_blocked[other] = 1
        self.assertTrue(grid.restore_obstacle_baseline())
        self.assertEqual(grid.data[other], UNKNOWN)
        self.assertFalse(grid.mow_blocked[other])
        # 建图阶段已知的障碍与区域轮廓保留
        self.assertEqual(grid.data[index], OCCUPIED)
        self.assertEqual(len(grid.zones), len(zones))
        self.assertEqual(
            [tuple(point) for point in grid.zones[0]["outline"]],
            [tuple(point) for point in zone["outline"]],
        )

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



    def test_unmowed_components_find_small_patch(self):
        grid = GridMap()
        grid.mask_ready = True
        zone_id = 7
        for col in range(20, 45):
            for row in range(20, 45):
                index = grid.index(col, row)
                grid.data[index] = 1
                grid.free_mask[index] = 1
                grid.zone_id_map[index] = zone_id
                grid.mowed[index] = 1
        for col in range(30, 35):
            for row in range(30, 35):
                grid.mowed[grid.index(col, row)] = 0
        components = grid.unmowed_components(zone_id, min_cells=20)
        self.assertEqual(len(components), 1)
        self.assertEqual(len(components[0]), 25)

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

    def test_obstacle_kind_is_estimated_by_area(self):
        grid = GridMap()
        # 2.6 m × 2.6 m 的占据块：拟合面积 > 5 m² → 大型
        for row in range(30, 56):
            for col in range(30, 56):
                grid.mark_occupied(col, row)
        # 0.3 m × 0.3 m：面积远小于 5 m² → 小型
        for row in range(70, 73):
            for col in range(70, 73):
                grid.mark_occupied(col, row)
        circles = grid.obstacle_circles(None)
        kinds = {(circle["kind"], round(math.pi * circle["r"] ** 2, 1)) for circle in circles}
        self.assertIn("large", {kind for kind, _ in kinds})
        self.assertIn("small", {kind for kind, _ in kinds})
        for kind, area in kinds:
            if kind == "large":
                self.assertGreaterEqual(area, 5.0)
            else:
                self.assertLess(area, 5.0)

    def test_zones_cover_all_free_cells_and_are_large(self):
        for map_id in range(1, 6):
            world.select_map(map_id)
            grid = GridMap()
            self.assertTrue(grid.build_free_area((world.START["x"], world.START["y"])))
            zones = grid.partition_zones()
            self.assertTrue(zones)
            self.assertEqual(grid.covered_free, grid.free_total)
            for zone in zones:
                self.assertGreaterEqual(zone["free"] * grid.cell * grid.cell, 19.9)
        world.select_map(1)

    def test_stamp_obstacle_blocks_navigation(self):
        grid = GridMap()
        grid.stamp_obstacle(10.0, 5.6, 0.5)
        center = grid.cell_of(10.0, 5.6)
        self.assertEqual(grid.data[grid.index(*center)], OCCUPIED)
        self.assertTrue(grid.is_blocked_world(10.7, 5.6))
        self.assertFalse(grid.is_blocked_world(11.2, 5.6))

    def test_stamp_footprint_only_paints_real_shape(self):
        grid = GridMap()
        # 1) 绕行一圈后占据栅格围成闭合轮廓：内部空洞应被补齐
        for index in range(120):
            angle = 2 * math.pi * index / 120.0
            col, row = grid.cell_of(
                5.0 + 0.50 * math.cos(angle), 4.0 + 0.50 * math.sin(angle)
            )
            grid.mark_occupied(col, row)
        stamped = grid.stamp_footprint(5.0, 4.0, 0.56)
        self.assertGreater(stamped, 0)
        center = grid.cell_of(5.0, 4.0)
        self.assertEqual(grid.data[grid.index(*center)], OCCUPIED)

        # 2) 只看到一段弧形（未闭合）时不能按包围圆整片涂抹
        arc = GridMap()
        for index in range(30):
            angle = -math.pi / 3 + (2 * math.pi / 3) * index / 29.0
            col, row = arc.cell_of(
                5.0 + 0.50 * math.cos(angle), 4.0 + 0.50 * math.sin(angle)
            )
            arc.mark_occupied(col, row)
        arc.stamp_footprint(5.0, 4.0, 0.56)
        inner = arc.cell_of(5.0, 4.0)
        self.assertNotEqual(arc.data[arc.index(*inner)], OCCUPIED)

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

    def test_plan_near_finds_reachable_cell_when_goal_is_covered(self):
        grid = GridMap()
        self.assertTrue(grid.build_free_area((world.START["x"], world.START["y"])))
        start = (world.START["x"], world.START["y"])
        goal = (start[0] + 2.0, start[1])
        col, row = grid.cell_of(*goal)
        for dc in range(-2, 3):
            for dr in range(-2, 3):
                grid.mark_occupied(col + dc, row + dr)
        path, distance = grid.plan_near(start, goal)
        self.assertTrue(path)
        self.assertLess(distance, 1.0)
        self.assertFalse(grid.is_occupied_world(path[-1][0], path[-1][1]))

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

    def test_contour_edge_has_no_ray_chords(self):
        grid = GridMap()
        self.assertTrue(grid.build_free_area((world.START["x"], world.START["y"])))
        zones = grid.partition_zones()
        irregular = []
        for zone in zones:
            area = max(1e-6, (zone["x1"] - zone["x0"]) * (zone["y1"] - zone["y0"]))
            fill = zone["free"] * grid.cell * grid.cell / area
            if fill < 0.92:
                irregular.append(zone)
        self.assertTrue(irregular)
        route = planner.edge_route_grid(grid, irregular[0], (irregular[0]["cx"], irregular[0]["cy"]))
        self.assertGreater(len(route), 8)
        # 区域包含障碍物时会连接外边界和障碍物边界两个闭环，整条路线可以不是闭合的。
        jumps = [
            math.hypot(route[index + 1][0] - route[index][0], route[index + 1][1] - route[index][1])
            for index in range(len(route) - 1)
        ]
        self.assertLess(max(jumps), 0.25)

    def test_edge_route_mask_stays_inside_zone(self):
        """新规则：区域矩形框 ∩ 自由区 → 腐蚀 → 轮廓，路径必须整段落在该区域内。"""
        for map_id in (1, 3, 5):
            world.select_map(map_id)
            grid = GridMap()
            self.assertTrue(grid.build_free_area((world.START["x"], world.START["y"])))
            for zone in grid.partition_zones():
                route = planner.edge_route_mask(grid, zone, (zone["cx"], zone["cy"]))
                self.assertGreater(len(route), 20)
                inside = sum(
                    1
                    for x, y in route
                    if grid.cell_in_zone(*grid.cell_of(x, y), zone["id"])
                )
                self.assertEqual(inside, len(route), "区域 %d 的沿边点跑出区域外" % zone["id"])
                # 闭合
                self.assertLess(math.hypot(route[0][0] - route[-1][0], route[0][1] - route[-1][1]), 1e-6)
                # 加密到 <= 0.16 m
                gaps = [
                    math.hypot(route[index + 1][0] - route[index][0], route[index + 1][1] - route[index][1])
                    for index in range(len(route) - 1)
                ]
                self.assertLess(max(gaps), 0.17)
                # 不再按"离多边形墙至少多少米"断言：沿边内缩已收到 EDGE_INSET
                # （0.02 m）以贴墙割草，而 map_3/map_5 的墙体在几何上是零宽缝隙，
                # 缝两侧到墙距离本来就很小。路径是否贴着区域边界由上面的
                # "整段落在区域内 + 闭合 + 加密到 0.16 m" 与下面的内缩矩形测试覆盖。

    def test_lane_segments_skip_unreachable_obstacle_interior(self):
        """绕障绕完一圈后，障碍物内部是"从没看到过"的未知格，但被已探明占据环
        封死。它不是机器人所在连通域的一部分，不能在新弓字线里出现。"""
        world.select_map(1)
        grid = GridMap()
        self.assertTrue(grid.build_free_area((world.START["x"], world.START["y"])))
        zone = grid.partition_zones()[0]
        cx, cy = zone["cx"], zone["cy"]
        radius = 0.6
        # 人为造一个**闭合**的占据环（内部保持未知），模拟"绕障一圈后就再没看到
        # 内部"：环要有厚度，否则格子缝隙会让内外连通。
        span = int(radius / grid.cell) + 3
        col0, row0 = grid.cell_of(cx, cy)
        for dc in range(-span, span + 1):
            for dr in range(-span, span + 1):
                x, y = grid.center_of(col0 + dc, row0 + dr)
                distance = math.hypot(x - cx, y - cy)
                if radius - grid.cell <= distance <= radius + grid.cell:
                    grid.data[grid.index(col0 + dc, row0 + dr)] = OCCUPIED
        grid.data_version += 1

        unfiltered = planner.lane_segments(grid, zone)
        inside_unfiltered = [
            segment for segment in unfiltered if _segment_hits_circle(segment, cx, cy, radius - 0.15)
        ]
        self.assertTrue(inside_unfiltered, "构造的障碍环内部本应被当成未知可割区")

        grid.set_reach_seed(world.START["x"], world.START["y"])
        filtered = planner.lane_segments(grid, zone)
        for segment in filtered:
            self.assertFalse(
                _segment_hits_circle(segment, cx, cy, radius - 0.15),
                "障碍物内部不可达，不该生成弓字线段",
            )

    def test_edge_route_mask_inset_matches_rectangle(self):
        """规则区域腐蚀后就等价于内缩矩形：周长接近 2*(w+h-2*inset)。"""
        world.select_map(3)
        grid = GridMap()
        self.assertTrue(grid.build_free_area((world.START["x"], world.START["y"])))
        zone = grid.partition_zones()[0]
        route = planner.edge_route_mask(grid, zone, (zone["cx"], zone["cy"]))
        length = sum(
            math.hypot(route[index + 1][0] - route[index][0], route[index + 1][1] - route[index][1])
            for index in range(len(route) - 1)
        )
        inset = planner.EDGE_INSET + grid.cell * 0.5
        expected = 2.0 * (zone["x1"] - zone["x0"] - 2 * inset) + 2.0 * (
            zone["y1"] - zone["y0"] - 2 * inset
        )
        self.assertLess(abs(length - expected) / expected, 0.12)
        self.assertLessEqual(max(x for x, _y in route), zone["x1"] - inset + grid.cell)
        self.assertGreaterEqual(min(x for x, _y in route), zone["x0"] + inset - grid.cell)

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

    def test_boundary_cells_count_only_sensable_side(self):
        """探明率的分母只能是场地内那一半墙格：外侧格子射线永远打不到。"""
        for map_id in (1, 3, 5):
            world.select_map(map_id)
            cells = world.boundary_cells()
            self.assertTrue(cells)
            for col, row in cells:
                x, y = (col + 0.5) * world.CELL, (row + 0.5) * world.CELL
                self.assertTrue(
                    world.point_in_polygon(x, y),
                    "边界统计包含场地外的格子 ({:.2f}, {:.2f})".format(x, y),
                )

    def test_boundary_gaps_only_reports_unknown_wall_cells(self):
        world.select_map(1)
        grid = GridMap()
        self.assertTrue(world.boundary_cells())
        gaps = grid.boundary_gaps()
        self.assertTrue(gaps)
        for gap in gaps:
            self.assertGreater(gap["run_m"], 0.0)
            self.assertGreater(gap["cells"], 0)

    def test_verified_footprint_never_turns_into_obstacle(self):
        """车体压过的格子是"一定能走"的硬证据，后续盖章不能再写成障碍。"""
        world.select_map(1)
        grid = GridMap()
        x = 8.5 * world.MAP_SCALE - 0.70
        y = 2.0 * world.MAP_SCALE
        col, row = grid.cell_of(x, y)
        grid.update_from_scan((x, y, 0.0), [], world.VISION_RANGE)
        self.assertTrue(grid.verified_free[grid.index(col, row)])
        grid.mark_occupied(col, row)
        self.assertNotEqual(grid.data[grid.index(col, row)], OCCUPIED)
        self.assertEqual(grid.blocked[grid.index(col, row)], 0)

    def test_wall_pending_matches_full_scan(self):
        """增量维护的"未探明墙格"必须和逐格重算的结果完全一致。"""
        world.select_map(1)
        grid = GridMap()
        robot_x, robot_y = world.START["x"], world.START["y"]
        for step in range(120):
            angle = step * 0.31
            pose = (
                robot_x + 0.6 * math.cos(angle),
                robot_y + 0.6 * math.sin(angle),
                angle,
            )
            grid.update_from_scan(pose, world.vision_scan(*pose), world.VISION_RANGE)
            if step % 20:
                continue
            cols, rows, data = grid.cols, grid.rows, grid.data
            full = set()
            for col, row in grid.wall_cells:
                index = row * cols + col
                if data[index] != UNKNOWN:
                    continue
                seen = False
                for dc in (-1, 0, 1):
                    for dr in (-1, 0, 1):
                        c2, r2 = col + dc, row + dr
                        if 0 <= c2 < cols and 0 <= r2 < rows:
                            if data[r2 * cols + c2] == OCCUPIED:
                                seen = True
                if not seen:
                    full.add(index)
            self.assertEqual(set(grid._wall_pending_set()), full)
            self.assertAlmostEqual(
                grid.boundary_discovery(),
                1.0 - len(full) / float(len(grid.wall_cells)),
                places=6,
            )

    def test_raycast_matches_brute_force(self):
        """分桶 + 按格缓存后的射线求交结果必须与逐段暴力求交一致。"""
        for map_id in (1, 5):
            world.select_map(map_id)
            samples = [
                (2.0, 2.0, 0.3),
                (world.WIDTH * 0.5, world.HEIGHT * 0.5, 1.1),
                (1.0, world.HEIGHT - 1.0, -0.7),
                (world.WIDTH - 1.0, 1.2, 2.4),
            ]
            for x, y, angle in samples:
                expected = world.VISION_RANGE
                dx, dy = math.cos(angle), math.sin(angle)
                for ax, ay, bx, by in (
                    world.POLYGON_SEGMENTS + world.OBSTACLE_SEGMENTS
                ):
                    ex, ey = bx - ax, by - ay
                    denom = dx * ey - dy * ex
                    if -1e-9 < denom < 1e-9:
                        continue
                    qx, qy = ax - x, ay - y
                    t = (qx * ey - qy * ex) / denom
                    if t <= 1e-6 or t >= expected:
                        continue
                    u = (qx * dy - qy * dx) / denom
                    if -1e-9 <= u <= 1.0 + 1e-9:
                        expected = t
                self.assertAlmostEqual(
                    world.raycast(x, y, angle), expected, places=9
                )

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
