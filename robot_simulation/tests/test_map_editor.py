import math
import os
import sys
import unittest

BACKEND_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "backend"
)
sys.path.insert(0, BACKEND_DIR)

import map_editor  # noqa: E402
import world  # noqa: E402
from grid_map import GridMap  # noqa: E402


def square(cx, cy, half):
    return [
        (cx - half, cy - half),
        (cx + half, cy - half),
        (cx + half, cy + half),
        (cx - half, cy + half),
        (cx - half, cy - half),
    ]


class DraftRegionTest(unittest.TestCase):
    def setUp(self):
        self.draft = map_editor.Draft("测试地图")

    def test_open_trace_is_rejected(self):
        with self.assertRaises(ValueError) as ctx:
            self.draft.add_region([(3, 3), (8, 3), (8, 8), (3, 8)])
        self.assertIn("未闭合", str(ctx.exception))

    def test_too_small_area_is_rejected(self):
        with self.assertRaises(ValueError) as ctx:
            self.draft.add_region(square(4.0, 4.0, 0.5))
        self.assertIn("面积", str(ctx.exception))

    def test_closed_trace_becomes_region(self):
        region = self.draft.add_region(square(5.0, 5.0, 2.0))
        self.assertEqual(region["kind"], "region")
        self.assertAlmostEqual(region["area"], 16.0, places=2)
        # 末尾重复的闭合点会被去掉，只保留顶点
        self.assertEqual(len(region["points"]), 4)
        self.assertIsNotNone(self.draft.region_at(5.0, 5.0))
        self.assertIsNone(self.draft.region_at(12.0, 12.0))

    def test_passage_needs_two_regions(self):
        self.draft.add_region(square(5.0, 5.0, 2.0))
        with self.assertRaises(ValueError):
            self.draft.add_passage([(4.0, 5.0), (6.0, 5.0)])
        self.draft.add_region(square(15.0, 15.0, 2.0))
        passage = self.draft.add_passage([(6.0, 5.0), (12.0, 12.0), (14.0, 15.0)])
        self.assertEqual(passage["from"], self.draft.regions[0]["id"])
        self.assertEqual(passage["to"], self.draft.regions[1]["id"])

    def test_passage_outside_regions_is_rejected(self):
        self.draft.add_region(square(5.0, 5.0, 2.0))
        self.draft.add_region(square(15.0, 15.0, 2.0))
        with self.assertRaises(ValueError):
            self.draft.add_passage([(0.5, 0.5), (14.0, 15.0)])


class DraftElementTest(unittest.TestCase):
    def setUp(self):
        self.draft = map_editor.Draft("测试地图")
        self.draft.add_region(square(6.0, 6.0, 3.0))

    def test_dock_rules(self):
        with self.assertRaises(ValueError) as ctx:
            self.draft.add_dock(12.0, 12.0)
        self.assertIn("区域内部", str(ctx.exception))
        with self.assertRaises(ValueError) as ctx:
            self.draft.add_dock(3.3, 6.0)
        self.assertIn("边界", str(ctx.exception))
        dock = self.draft.add_dock(6.0, 6.0, 0.45)
        self.assertAlmostEqual(dock["r"], 0.45)
        with self.assertRaises(ValueError) as ctx:
            self.draft.add_dock(7.0, 7.0, 0.45)
        self.assertIn("只能有一个", str(ctx.exception))

    def test_dock_rejects_obstacle_overlap(self):
        self.draft.add_static([(6.0, 6.0)], 0.35)
        with self.assertRaises(ValueError) as ctx:
            self.draft.add_dock(6.0, 6.0, 0.45)
        self.assertIn("障碍物", str(ctx.exception))

    def test_painting_makes_one_solid_stroke(self):
        item = self.draft.add_static([(4.0, 4.0), (4.4, 4.0), (4.8, 4.0)], 0.30)
        self.assertEqual(len(self.draft.static), 1)
        self.assertEqual(item["kind"], "brush")
        self.assertAlmostEqual(item["length"], 0.8, places=2)
        # 续笔：起点接在末端就并进同一条笔迹
        self.draft.add_static([(4.85, 4.02), (5.2, 4.1)], 0.30)
        self.assertEqual(len(self.draft.static), 1)
        self.assertGreater(self.draft.static[0]["length"], 1.1)
        # 换半径就是另一条笔迹
        self.draft.add_static([(6.0, 6.0)], 0.50)
        self.assertEqual(len(self.draft.static), 2)

    def test_no_go_needs_real_size(self):
        with self.assertRaises(ValueError):
            self.draft.add_no_go(4.0, 4.0, 4.2, 4.2)
        zone = self.draft.add_no_go(7.0, 7.0, 4.0, 5.0)
        self.assertAlmostEqual(zone["x0"], 4.0)
        self.assertAlmostEqual(zone["y1"], 7.0)

    def test_dynamic_span_is_clamped_to_canvas(self):
        item = self.draft.add_dynamic(23.5, 12.0, 0.3, 4.0, 0.0)
        self.assertLessEqual(item["bx"], self.draft.width - 0.4 + 1e-6)
        self.assertGreaterEqual(item["ax"], 0.4 - 1e-6)


class DraftDeleteTest(unittest.TestCase):
    def setUp(self):
        self.draft = map_editor.Draft("测试地图")

    def test_deleting_region_drops_its_passages(self):
        first = self.draft.add_region(square(5.0, 5.0, 2.0))
        self.draft.add_region(square(15.0, 15.0, 2.0))
        self.draft.add_passage([(6.0, 5.0), (14.0, 15.0)])
        result = self.draft.delete("region", first["id"])
        self.assertEqual(result["removed_passages"], 1)
        self.assertEqual(self.draft.passages, [])

    def test_hit_test_prefers_surface_elements(self):
        self.draft.add_region(square(6.0, 6.0, 3.0))
        self.draft.add_dock(6.0, 6.0, 0.45)
        zone = self.draft.add_no_go(4.0, 4.0, 8.0, 8.0)
        hit = self.draft.hit_test(6.0, 6.0)
        self.assertEqual(hit["kind"], "no_go")
        self.assertEqual(hit["id"], zone["id"])

    def test_hit_test_uses_tight_dock_radius(self):
        self.draft.add_region(square(6.0, 6.0, 3.0))
        dock = self.draft.add_dock(6.0, 6.0, 0.45)
        self.assertEqual(self.draft.hit_test(6.0, 6.0)["kind"], "dock")
        # 桩外 0.6 m 的点不算命中充电桩，而是命中区域
        farther = self.draft.hit_test(6.0, 6.7)
        self.assertEqual(farther["kind"], "region")
        self.assertNotEqual(farther["id"], dock["id"])

    def test_hit_test_empty_spot(self):
        self.draft.add_region(square(6.0, 6.0, 3.0))
        self.assertIsNone(self.draft.hit_test(20.0, 20.0))


class DraftSaveTest(unittest.TestCase):
    def setUp(self):
        self.draft = map_editor.Draft("花园")
        self.created = []

    def tearDown(self):
        for map_id in self.created:
            if map_id in world.MAPS:
                world.delete_map(map_id)

    def test_problems_until_minimum_elements_exist(self):
        self.assertIn("至少需要一个区域", self.draft.problems())
        self.draft.add_region(square(6.0, 6.0, 3.0))
        self.assertIn("至少需要一个充电桩", self.draft.problems())
        self.draft.add_dock(6.0, 6.0, 0.45)
        self.assertEqual(self.draft.problems(), [])
        self.assertTrue(self.draft.can_save())

    def test_disconnected_regions_block_saving(self):
        self.draft.add_region(square(6.0, 6.0, 3.0))
        self.draft.add_region(square(16.0, 16.0, 3.0))
        self.draft.add_dock(6.0, 6.0, 0.45)
        problems = self.draft.problems()
        self.assertTrue(any("通道" in text for text in problems))
        self.assertFalse(self.draft.can_save())
        self.draft.add_passage([(8.0, 6.0), (14.0, 16.0)])
        self.assertEqual(self.draft.problems(), [])

    def test_definition_carries_dock_obstacles_and_flag(self):
        self.draft.add_region(square(6.0, 6.0, 3.0))
        self.draft.add_dock(6.0, 6.0, 0.45)
        self.draft.add_no_go(3.6, 3.6, 4.4, 4.4)
        self.draft.add_static([(8.0, 8.0)], 0.30)
        self.draft.add_dynamic(9.0, 10.0, 0.3, 1.6, 0.5)
        self.assertTrue(self.draft.can_save())
        definition = self.draft.to_definition()
        self.assertTrue(definition.get("editor_built"))
        self.assertTrue(definition.get("native"))
        self.assertGreaterEqual(len(definition["polygon"]), 4)
        self.assertEqual(len(definition["no_go"]), 1)
        self.assertEqual(len(definition["dynamic"]), 1)
        # 涂抹出来的障碍物即使面积小于 MIN_HOLE_AREA 也要保留实体几何
        # （轮廓按 0.1 m 栅格描出来，面积会比理论圆略大一点）
        painted = [
            item
            for item in definition["obstacles"]
            if 0.2 <= item.get("area", 0.0) <= 0.45
        ]
        self.assertEqual(len(painted), 1)
        self.assertEqual(painted[0]["kind"], "small")
        map_id = world.register_map(definition)
        self.created.append(map_id)
        previous = world.MAP_ID
        try:
            world.select_map(map_id)
            self.assertTrue(world.collision(8.2, 8.2, 0.05))
        finally:
            world.select_map(previous)
        listed = {item["id"]: item for item in world.map_list()}
        self.assertTrue(listed[map_id]["editor"])
        self.assertFalse(listed[1]["editor"])

    def test_saved_map_is_navigable(self):
        self.draft.add_region(square(6.0, 6.0, 4.0))
        self.draft.add_dock(6.0, 6.0, 0.45)
        definition = self.draft.to_definition()
        map_id = world.register_map(definition)
        self.created.append(map_id)
        previous = world.MAP_ID
        try:
            world.select_map(map_id)
            self.assertEqual(world.MAP_ID, map_id)
            # 桩在区域正中，四周都能走通
            for angle in (0.0, math.pi / 2, math.pi, -math.pi / 2):
                x = 6.0 + math.cos(angle) * 2.5
                y = 6.0 + math.sin(angle) * 2.5
                self.assertTrue(world.point_in_polygon(x, y))
                self.assertFalse(world.collision(x, y, world.CELL))
        finally:
            world.select_map(previous)


if __name__ == "__main__":
    unittest.main()


class DraftCropTest(unittest.TestCase):
    def setUp(self):
        self.created = []

    def tearDown(self):
        for map_id in self.created:
            if map_id in world.MAPS:
                world.delete_map(map_id)

    def test_blank_canvas_is_100m(self):
        draft = map_editor.Draft()
        self.assertEqual(draft.width, 100.0)
        self.assertEqual(draft.height, 100.0)

    def test_saved_map_is_cropped_to_elements(self):
        draft = map_editor.Draft("裁剪测试")
        draft.add_region(square(60.0, 60.0, 4.0))
        draft.add_dock(60.0, 60.0, 0.45)
        bounds = draft.content_bounds()
        # 元素只占了 8 m 见方，外接框不应该还是 100 m
        self.assertLess(bounds[2] - bounds[0], 12.0)
        self.assertLess(bounds[3] - bounds[1], 12.0)
        definition = draft.to_definition()
        self.assertLess(max(definition["width"], definition["height"]), 14.0)
        # 位置坐标也被平移到了新地图里（左下方 0.2 m 的 padding）
        dock = definition["dock"]
        self.assertLess(dock["x"], 14.0)
        self.assertGreater(dock["x"], 1.0)

    def test_loading_existing_map_keeps_geometry(self):
        source = world.MAPS[1]
        draft = map_editor.Draft()
        draft.load_from_definition(source, scale=world.MAP_SCALE)
        self.assertEqual(len(draft.regions), 1)
        self.assertEqual(len(draft.docks), 1)
        area = draft.regions[0]["area"]
        expected = abs(map_editor._polygon_area(source["polygon"])) * world.MAP_SCALE ** 2
        self.assertAlmostEqual(area, expected, delta=expected * 0.02)
        definition = draft.to_definition()
        self.assertAlmostEqual(
            definition["width"],
            source["width"] * world.MAP_SCALE,
            delta=3.0,
        )

    def test_passage_is_single_lane(self):
        self.assertAlmostEqual(map_editor.PASSAGE_HALF_WIDTH, 0.50, places=3)


class PassageZoneTest(unittest.TestCase):
    """通道只通行、不割草：两个区域不会被通道粘成一个区域。"""

    def setUp(self):
        draft = map_editor.Draft("通道测试")
        draft.add_region(square(6.0, 6.0, 4.0))
        draft.add_region(square(20.0, 6.0, 4.0))
        draft.add_dock(6.0, 6.0, 0.45)
        draft.add_passage([(9.6, 6.0), (16.4, 6.0)])
        definition = draft.to_definition("通道测试")
        self.map_id = world.register_map(definition)
        world.select_map(self.map_id)
        self.grid = GridMap()
        # free_mask 模拟"建图走通后的连通自由区"：整张场地多边形内部都算自由
        self.grid.free_mask = bytearray(self.grid.cols * self.grid.rows)
        self.grid.mask_ready = True
        for index in range(self.grid.cols * self.grid.rows):
            col = index % self.grid.cols
            row = index // self.grid.cols
            x, y = self.grid.center_of(col, row)
            if world.point_in_polygon(x, y):
                self.grid.free_mask[index] = 1
        points = world.PASSAGES[0]["points"]
        self.mid_x = (points[0][0] + points[-1][0]) * 0.5
        self.mid_y = (points[0][1] + points[-1][1]) * 0.5
        self.region_point = (
            points[0][0] - 2.0,
            points[0][1],
        )

    def tearDown(self):
        if self.map_id in world.MAPS:
            world.delete_map(self.map_id)

    def test_passage_cells_are_not_mowable(self):
        self.assertGreater(sum(1 for value in self.grid.no_mow if value), 0)
        self.assertTrue(self.grid.cell_no_mow(*self.grid.cell_of(self.mid_x, self.mid_y)))
        self.assertFalse(self.grid.cell_mowable(*self.grid.cell_of(self.mid_x, self.mid_y)))
        self.assertTrue(
            self.grid.cell_mowable(*self.grid.cell_of(*self.region_point))
        )

    def test_passage_does_not_merge_zones(self):
        zones = self.grid.partition_zones()
        self.assertEqual(len(zones), 2)
        passage_index = self.grid.index(*self.grid.cell_of(self.mid_x, self.mid_y))
        self.assertEqual(self.grid.zone_id_map[passage_index], 0)
        for zone in zones:
            self.assertGreater(zone["free"] * self.grid.cell ** 2, 10.0)

    def test_mowing_a_passage_does_not_count_as_coverage(self):
        self.grid.partition_zones()
        before = sum(self.grid.zone_mowed.values())
        self.grid.mark_mowed(self.mid_x, self.mid_y, 0.45)
        self.assertEqual(sum(self.grid.zone_mowed.values()), before)


class DynamicDraftTest(unittest.TestCase):
    def test_dynamic_keeps_custom_path_and_speed(self):
        draft = map_editor.Draft()
        draft.add_region(square(6.0, 6.0, 4.0))
        draft.add_dock(6.0, 6.0, 0.45)
        item = draft.add_dynamic(6.0, 6.0, 0.30, 1.0, 0.0)
        self.assertAlmostEqual(item["speed"], world.DYNAMIC_SPEED, places=3)
        updated = draft.update_dynamic(
            item["id"], 4.5, 4.5, 7.5, 7.5, speed=0.35
        )
        self.assertAlmostEqual(updated["ax"], 4.5, places=2)
        self.assertAlmostEqual(updated["by"], 7.5, places=2)
        self.assertAlmostEqual(updated["speed"], 0.35, places=2)
        self.assertGreater(updated["span"], 4.0)
        with self.assertRaises(ValueError):
            draft.update_dynamic(item["id"], 5.0, 5.0, 5.1, 5.1)
        definition = draft.to_definition()
        self.assertAlmostEqual(definition["dynamic"][0]["speed"], 0.35, places=2)


class RoundTripTest(unittest.TestCase):
    """读进来再存一次：区域 / 通道 / 障碍 / 禁区 / 动态障碍都不能走样。"""

    def setUp(self):
        self.created = []
        draft = map_editor.Draft("往返")
        draft.add_region(square(6.0, 6.0, 4.0))
        draft.add_region(square(20.0, 6.0, 4.0))
        draft.add_dock(6.0, 6.0, 0.45)
        draft.add_passage([(9.6, 6.0), (16.4, 6.0)])
        draft.add_static([(8.0, 8.0)], 0.40)
        draft.add_no_go(5.0, 8.5, 7.0, 9.5)
        draft.add_dynamic(19.0, 8.0, 0.30, 2.0, 0.0, 18.0, 8.0, 20.0, 8.0, speed=0.30)
        self.definition = draft.to_definition("往返")

    def tearDown(self):
        for map_id in self.created:
            if map_id in world.MAPS:
                world.delete_map(map_id)
        if world.MAP_ID not in world.MAPS:
            world.select_map(1)

    def test_definition_keeps_regions_and_passage_links(self):
        self.assertEqual(len(self.definition["regions"]), 2)
        self.assertEqual(len(self.definition["passages"]), 1)
        passage = self.definition["passages"][0]
        self.assertEqual(passage["from"], self.definition["regions"][0]["id"])
        self.assertEqual(passage["to"], self.definition["regions"][1]["id"])

    def test_reload_and_resave_is_stable(self):
        definition = self.definition
        for _ in range(3):
            draft = map_editor.Draft()
            draft.load_from_definition(definition)
            self.assertEqual(len(draft.regions), 2)
            self.assertEqual(len(draft.passages), 1)
            self.assertEqual(len(draft.no_gos), 1)
            self.assertEqual(len(draft.dynamic), 1)
            self.assertAlmostEqual(draft.dynamic[0]["speed"], 0.30, places=2)
            self.assertAlmostEqual(sum(item["area"] for item in draft.poly_obstacles), 0.50, delta=0.10)
            self.assertEqual(draft.problems(), [])
            definition = draft.to_definition("往返")
        self.assertLess(definition["width"], 40.0)

    def test_saved_map_exposes_passage_to_simulation(self):
        definition = self.definition
        map_id = world.register_map(definition)
        self.created.append(map_id)
        previous = world.MAP_ID
        try:
            world.select_map(map_id)
            self.assertEqual(len(world.PASSAGES), 1)
            self.assertAlmostEqual(world.PASSAGES[0]["half"], 0.50, places=3)
            self.assertEqual(len(world.DYNAMIC), 1)
            self.assertAlmostEqual(world.DYNAMIC[0]["speed"], 0.30, places=2)
            self.assertEqual(len(world.NO_GO), 1)
        finally:
            if previous in world.MAPS:
                world.select_map(previous)


class PassageOverlapTest(unittest.TestCase):
    """通道压在区域上的部分要照常割，只有纯通道段不割。"""

    def setUp(self):
        draft = map_editor.Draft("重叠通道")
        draft.add_region(square(6.0, 6.0, 4.0))
        draft.add_region(square(20.0, 6.0, 4.0))
        draft.add_dock(6.0, 6.0, 0.45)
        draft.add_passage([(5.0, 6.0), (21.0, 6.0)])
        self.definition = draft.to_definition("重叠通道")
        self.map_id = world.register_map(self.definition)
        world.select_map(self.map_id)
        self.grid = GridMap()
        self.crop = self.definition["import_meta"]["crop"]

    def tearDown(self):
        if self.map_id in world.MAPS:
            world.delete_map(self.map_id)

    def world_point(self, x, y):
        return x - self.crop[0] + 0.2, y - self.crop[1] + 0.2

    def test_region_part_of_passage_stays_mowable(self):
        for x in (3.0, 4.5, 7.0, 9.0, 17.0, 19.5, 22.5):
            wx, wy = self.world_point(x, 6.0)
            self.assertFalse(self.grid.no_mow_world(wx, wy), "x={}".format(x))
            self.assertTrue(self.grid.cell_mowable(*self.grid.cell_of(wx, wy)))

    def test_only_exclusive_part_is_no_mow(self):
        exclusive = 0
        for x in (10.2, 12.0, 13.0, 15.8):
            wx, wy = self.world_point(x, 6.0)
            self.assertTrue(self.grid.no_mow_world(wx, wy), "x={}".format(x))
            self.assertFalse(self.grid.cell_mowable(*self.grid.cell_of(wx, wy)))
            exclusive += 1
        self.assertEqual(exclusive, 4)
        self.assertGreater(exclusive, 0)

    def test_exclusive_passage_does_not_join_zones(self):
        self.grid.free_mask = bytearray(self.grid.cols * self.grid.rows)
        self.grid.mask_ready = True
        for index in range(self.grid.cols * self.grid.rows):
            col = index % self.grid.cols
            row = index // self.grid.cols
            x, y = self.grid.center_of(col, row)
            if world.point_in_polygon(x, y):
                self.grid.free_mask[index] = 1
        zones = self.grid.partition_zones()
        self.assertEqual(len(zones), 2)
        wx, wy = self.world_point(13.0, 6.0)
        self.assertEqual(self.grid.zone_id_map[self.grid.index(*self.grid.cell_of(wx, wy))], 0)


class RegionPartitionTest(unittest.TestCase):
    """地图编辑器保存的地图：用户画了几个区域，割草就分几个区域。"""

    def setUp(self):
        self.draft = map_editor.Draft("分区")
        # 两块区域在栅格里是连着的（边缘只隔 0.05 m），再加一条短通道满足保存校验
        self.draft.add_region(square(10.0, 10.0, 4.0))
        self.draft.add_region(square(18.05, 10.0, 4.0))
        self.draft.add_dock(7.0, 10.0, 0.45)
        self.draft.add_passage([(13.5, 10.0), (14.55, 10.0)])
        self.definition = self.draft.to_definition("分区")
        self.map_id = world.register_map(self.definition)
        world.select_map(self.map_id)
        self.grid = GridMap()
        # 不做建图，直接把场地内所有格子当成"可达自由区"
        self.grid.free_mask = bytearray(self.grid.cols * self.grid.rows)
        self.grid.mask_ready = True
        for index in range(self.grid.cols * self.grid.rows):
            x, y = self.grid.center_of(index % self.grid.cols, index // self.grid.cols)
            if world.point_in_polygon(x, y):
                self.grid.free_mask[index] = 1

    def tearDown(self):
        if self.map_id in world.MAPS:
            world.delete_map(self.map_id)

    def test_each_edited_region_is_its_own_zone(self):
        zones = self.grid.partition_zones()
        self.assertEqual(len(zones), 2, "两个区域被并成了一个")
        polygons = [item["points"] for item in world.REGIONS]
        for zone in zones:
            self.assertGreater(zone["free"], 1000)
            self.assertTrue(
                any(world.point_in_polygon(zone["cx"], zone["cy"], poly) for poly in polygons),
                "区域质心不在任何画好的区域里",
            )

    def test_zone_cells_stay_inside_one_region(self):
        zones = self.grid.partition_zones()
        polygons = [item["points"] for item in world.REGIONS]
        for zone in zones:
            outside = 0
            for index, assigned in enumerate(self.grid.zone_id_map):
                if assigned != zone["id"]:
                    continue
                x, y = self.grid.center_of(index % self.grid.cols, index // self.grid.cols)
                if not any(world.point_in_polygon(x, y, poly) for poly in polygons):
                    outside += 1
            # 区域边缘（多边形比栅格小半格）上的一圈格子会按最近的区域补进去，
            # 数量必须很少，不能是整块别的区域
            self.assertLess(outside, zone["free"] * 0.05)

    def test_small_region_is_not_merged(self):
        draft = map_editor.Draft("小区域")
        draft.add_region(square(10.0, 10.0, 4.0))
        draft.add_region(square(20.0, 10.0, 1.2))  # 约 5.8 m²，远小于最小区块面积
        draft.add_dock(7.0, 10.0, 0.45)
        draft.add_passage([(13.8, 10.0), (18.9, 10.0)])
        map_id = world.register_map(draft.to_definition("小区域"))
        try:
            world.select_map(map_id)
            grid = GridMap()
            grid.free_mask = bytearray(grid.cols * grid.rows)
            grid.mask_ready = True
            for index in range(grid.cols * grid.rows):
                x, y = grid.center_of(index % grid.cols, index // grid.cols)
                if world.point_in_polygon(x, y):
                    grid.free_mask[index] = 1
            zones = grid.partition_zones()
            self.assertEqual(len(zones), 2, "小区域被并进隔壁区域了")
        finally:
            if map_id in world.MAPS:
                world.delete_map(map_id)
            world.select_map(1)


class UndoTest(unittest.TestCase):
    def setUp(self):
        self.draft = map_editor.Draft("撤销")
        self.draft.add_region(square(6.0, 6.0, 4.0))
        self.draft.add_dock(6.0, 6.0, 0.45)

    def test_undo_restores_deleted_element(self):
        brush = self.draft.add_static([(4.5, 4.5), (5.0, 4.5)], 0.3)
        self.assertEqual(len(self.draft.static), 1)
        self.draft.delete("static", brush["id"])
        self.assertEqual(len(self.draft.static), 0)
        self.draft.undo()
        self.assertEqual(len(self.draft.static), 1)
        self.assertAlmostEqual(self.draft.static[0]["length"], 0.5, places=2)

    def test_undo_restores_deleted_region_and_passage(self):
        self.draft.add_region(square(20.0, 6.0, 4.0))
        self.draft.add_passage([(9.6, 6.0), (16.4, 6.0)])
        first = self.draft.regions[0]["id"]
        self.draft.delete("region", first)
        self.assertEqual(len(self.draft.regions), 1)
        self.assertEqual(len(self.draft.passages), 0)
        self.draft.undo()
        self.assertEqual(len(self.draft.regions), 2)
        self.assertEqual(len(self.draft.passages), 1)

    def test_failed_action_does_not_consume_undo(self):
        self.draft.add_no_go(4.0, 4.0, 8.0, 8.0)
        with self.assertRaises(ValueError):
            self.draft.add_no_go(4.0, 4.0, 4.1, 4.1)
        self.draft.undo()
        self.assertEqual(len(self.draft.no_gos), 0)
        # 失败的操作没有占掉一层历史：再撤一步能退回到"没有充电桩"的状态
        self.draft.undo()
        self.assertEqual(len(self.draft.docks), 0)
        self.assertEqual(len(self.draft.regions), 1)

    def test_undo_without_history(self):
        draft = map_editor.Draft("空")
        with self.assertRaises(ValueError):
            draft.undo()


class BrushGeometryTest(unittest.TestCase):
    def test_stroke_outline_is_a_solid_band(self):
        outline = map_editor._stroke_outline([(0.0, 0.0), (4.0, 0.0)], 0.4)
        area = abs(map_editor._polygon_area(outline))
        # 长 4 m、宽 0.8 m 的实心带 + 两个半圆帽
        self.assertAlmostEqual(area, 4 * 0.8 + math.pi * 0.16, delta=0.5)
        # 顶点数要收敛（不能是几百个点的圆点串）
        self.assertLess(len(outline), 60)

    def test_brush_saved_as_one_polygon(self):
        draft = map_editor.Draft("笔迹")
        draft.add_region(square(6.0, 6.0, 4.0))
        draft.add_dock(6.0, 6.0, 0.45)
        draft.add_static([(5.0, 5.0), (6.0, 5.0), (7.0, 5.5)], 0.35)
        definition = draft.to_definition("笔迹")
        painted = [item for item in definition["obstacles"] if item["area"] > 0.5]
        self.assertEqual(len(painted), 1)
        self.assertGreater(painted[0]["area"], 0.9)


class CenterLoadedMapTest(unittest.TestCase):
    def test_loaded_map_is_centered_on_canvas(self):
        draft = map_editor.Draft()
        draft.load_from_definition(world.MAPS[1], scale=world.MAP_SCALE)
        x0, y0, x1, y1 = draft.content_bounds()
        self.assertAlmostEqual((x0 + x1) / 2, draft.width / 2, delta=2.0)
        self.assertAlmostEqual((y0 + y1) / 2, draft.height / 2, delta=2.0)
        self.assertGreaterEqual(draft.width, 100.0)


class BrushMergeTest(unittest.TestCase):
    """实心画笔只保留外围轮廓：重叠的多笔合成一个实心块。"""

    def test_overlapping_strokes_merge_into_one_outline(self):
        draft = map_editor.Draft("合并")
        draft.add_region(square(40.0, 40.0, 10.0))
        draft.add_dock(40.0, 40.0, 0.45)
        draft.add_static([(35.0, 35.0), (40.0, 35.0)], 0.4)
        draft.add_static([(37.0, 35.1), (38.0, 35.1)], 0.4)
        self.assertEqual(len(draft.static), 2)
        outlines = draft.brush_outlines()
        self.assertEqual(len(outlines), 1)
        area = abs(map_editor._polygon_area(outlines[0]))
        # 6 m 长、0.8 m 宽 + 两端半圆帽
        self.assertAlmostEqual(area, 5.0 * 0.8 + math.pi * 0.16, delta=1.2)
        definition = draft.to_definition("合并")
        painted = [item for item in definition["obstacles"] if item["area"] > 0.5]
        self.assertEqual(len(painted), 1)

    def test_separate_strokes_stay_separate(self):
        draft = map_editor.Draft("分离")
        draft.add_region(square(40.0, 40.0, 10.0))
        draft.add_dock(40.0, 40.0, 0.45)
        draft.add_static([(34.0, 34.0), (35.0, 34.0)], 0.3)
        draft.add_static([(44.0, 44.0), (45.0, 44.0)], 0.3)
        self.assertEqual(len(draft.brush_outlines()), 2)

    def test_outline_has_no_inner_stroke_edges(self):
        draft = map_editor.Draft("无内边")
        draft.add_region(square(40.0, 40.0, 10.0))
        draft.add_dock(40.0, 40.0, 0.45)
        for offset in (0.0, 0.15, 0.30):
            draft.add_static([(35.0, 35.0 + offset), (40.0, 35.0 + offset)], 0.35)
        outlines = draft.brush_outlines()
        self.assertEqual(len(outlines), 1)
        # 轮廓点数要收敛：不能是三条笔迹各自一圈拼出来的一堆顶点
        self.assertLess(len(outlines[0]), 40)
        # 合并后的面积 ≈ 5 m × 1.05 m（三条 0.7 m 宽的带子叠在一起）
        area = abs(map_editor._polygon_area(outlines[0]))
        self.assertLess(area, 6.2)
        self.assertGreater(area, 4.0)
