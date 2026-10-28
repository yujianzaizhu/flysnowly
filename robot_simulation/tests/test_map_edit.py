import os
import sys
import unittest

BACKEND_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "backend"
)
sys.path.insert(0, BACKEND_DIR)

import world  # noqa: E402
from grid_map import GridMap  # noqa: E402


class MapEditTest(unittest.TestCase):
    def setUp(self):
        world.select_map(1)
        world.clear_edits()

    def tearDown(self):
        world.clear_edits()

    def test_static_obstacle_becomes_real_geometry(self):
        base = len(world.OBSTACLE_SHAPES)
        item = world.add_static_obstacle(5.0, 4.5, 0.45)
        self.assertEqual(len(world.OBSTACLE_SHAPES), base + 1)
        self.assertEqual(world.OBSTACLE_SHAPES[-1]["edit"], "static")
        self.assertEqual(world.OBSTACLE_SHAPES[-1]["id"], item["id"])
        self.assertTrue(world.collision(5.0, 4.5, 0.18))
        self.assertLess(world.raycast(4.0, 4.5, 0.0, 3.0), 0.75)

    def test_obstacle_outside_field_is_rejected(self):
        with self.assertRaises(ValueError):
            world.add_static_obstacle(0.05, 0.05, 0.30)
        with self.assertRaises(ValueError):
            world.add_no_go_zone(40.0, 40.0, 42.0, 42.0)

    def test_no_go_zone_is_a_rectangle(self):
        item = world.add_no_go_zone(6.0, 5.0, 4.0, 3.0)
        self.assertAlmostEqual(item["x0"], 4.0)
        self.assertAlmostEqual(item["y0"], 3.0)
        self.assertAlmostEqual(item["x1"], 6.0)
        self.assertAlmostEqual(item["y1"], 5.0)
        self.assertAlmostEqual(item["x"], 5.0)
        self.assertAlmostEqual(item["y"], 4.0)
        with self.assertRaises(ValueError):
            world.add_no_go_zone(4.0, 4.0, 4.1, 4.1)

    def test_dynamic_obstacle_patrols_between_endpoints(self):
        item = world.add_dynamic_obstacle(5.0, 4.0, 0.30, 2.0, 0.0)
        self.assertAlmostEqual(world.DYNAMIC[0]["ax"], item["ax"])
        start = (world.DYNAMIC[0]["x"], world.DYNAMIC[0]["y"])
        for _ in range(40):
            world.update_dynamic(0.1)
        moved = (world.DYNAMIC[0]["x"], world.DYNAMIC[0]["y"])
        self.assertGreater(abs(moved[0] - start[0]) + abs(moved[1] - start[1]), 0.2)
        self.assertLessEqual(abs(moved[0] - item["ax"]), 2.0 + 1e-6)
        # 动态障碍物同样是实体，参与视觉与碰撞（占据圆按实时位置刷新）
        live = world.DYNAMIC[0]
        self.assertTrue(
            any(
                abs(ox - live["x"]) < 0.05 and abs(oy - live["y"]) < 0.05
                for ox, oy, _ in world.OBSTACLES
            )
        )
        self.assertLess(world.raycast(live["x"] - 1.0, live["y"], 0.0, 3.0), 1.3)

    def test_edits_are_kept_per_map(self):
        world.add_static_obstacle(5.0, 4.5, 0.40)
        world.add_no_go_zone(3.0, 2.5, 5.0, 4.5)
        world.select_map(2)
        self.assertEqual(world.edit_dict()["static"], [])
        self.assertEqual(world.NO_GO, [])
        world.select_map(1)
        self.assertEqual(len(world.edit_dict()["static"]), 1)
        self.assertEqual(len(world.NO_GO), 1)
        self.assertEqual(len(world.OBSTACLE_SHAPES), len(world.BASE_SHAPES) + 1)

    def test_delete_and_clear(self):
        static = world.add_static_obstacle(5.0, 4.5, 0.40)
        dynamic = world.add_dynamic_obstacle(4.0, 3.0, 0.30, 1.5)
        no_go = world.add_no_go_zone(2.0, 2.0, 4.0, 3.5)
        world.remove_edit("static", static["id"])
        self.assertEqual(len(world.DYNAMIC), 1)
        world.remove_edit("dynamic", dynamic["id"])
        self.assertEqual(world.DYNAMIC, [])
        world.remove_edit("no_go", no_go["id"])
        self.assertEqual(world.NO_GO, [])
        with self.assertRaises(ValueError):
            world.remove_edit("static", 9999)
        with self.assertRaises(ValueError):
            world.remove_edit("unknown", 1)

    def test_no_go_zone_blocks_mowing_and_planning(self):
        grid = GridMap()
        grid.build_free_area((world.START["x"], world.START["y"]))
        item = world.add_no_go_zone(5.0, 4.0, 7.0, 6.0)
        self.assertGreater(grid.apply_no_go(world.NO_GO), 0)
        inside_col, inside_row = grid.cell_of(item["x"], item["y"])
        self.assertTrue(grid.cell_blocked_mow(inside_col, inside_row))
        self.assertTrue(grid.blocked[grid.index(inside_col, inside_row)])
        # 矩形外的栅格不受影响
        outside_col, outside_row = grid.cell_of(item["x1"] + 1.2, item["y"])
        self.assertFalse(grid.cell_blocked_mow(outside_col, outside_row))
        corner_col, corner_row = grid.cell_of(item["x0"] + 0.15, item["y0"] + 0.15)
        self.assertTrue(grid.cell_blocked_mow(corner_col, corner_row))
        # 禁区不是实体障碍，不应进入占据栅格
        self.assertNotEqual(grid.data[grid.index(inside_col, inside_row)], 2)


if __name__ == "__main__":
    unittest.main()
