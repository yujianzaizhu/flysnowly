import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import params  # noqa: E402

BACKEND_DIR = params.BACKEND_DIR
if BACKEND_DIR not in sys.path:
    sys.path.insert(0, BACKEND_DIR)

import map_import  # noqa: E402
import world  # noqa: E402

REAL_MAP = "/home/fibocom/heyufei/data/map/map"


class MapImportTest(unittest.TestCase):
    def tearDown(self):
        world.select_map(1)

    def write_pgm(self, root, width=40, height=30):
        pixels = bytearray(width * height)
        for row in range(4, height - 4):
            for col in range(4, width - 4):
                pixels[row * width + col] = 255
        for row in range(10, 18):
            for col in range(15, 23):
                pixels[row * width + col] = 0
        path = os.path.join(root, "grid_map.pgm")
        with open(path, "wb") as handle:
            handle.write(
                "P5\n{} {}\n255\n".format(width, height).encode("ascii")
                + bytes(pixels)
            )
        return path

    def test_build_from_pgm_directory_and_register(self):
        with tempfile.TemporaryDirectory() as root:
            self.write_pgm(root)
            os.makedirs(os.path.join(root, "path"))
            with open(os.path.join(root, "path", "default.json"), "w") as handle:
                handle.write('{"resolution": "0.1"}')
            definition = map_import.build_map(root, max_extent=12.0)
            self.assertTrue(definition["native"])
            self.assertTrue(definition["imported"])
            self.assertGreater(len(definition["polygon"]), 3)
            self.assertGreaterEqual(len(definition["obstacles"]), 1)
            self.assertAlmostEqual(definition["width"], 4.4, places=1)
            self.assertAlmostEqual(definition["height"], 3.4, places=1)
            map_id = world.register_map(definition)
            world.select_map(map_id)
            self.assertTrue(world.point_in_polygon(world.START["x"], world.START["y"]))
            self.assertFalse(world.collision(world.START["x"], world.START["y"]))
            self.assertFalse(world.collision(world.DOCK["x"], world.DOCK["y"]))
            data = world.world_dict()
            self.assertTrue(data["imported"])
            self.assertGreater(data["obstacle_shapes"][0]["outline"][0][0], 0.0)
            self.assertLess(
                world.raycast(world.START["x"], world.START["y"], 0.0, 20.0),
                20.0,
            )

    @unittest.skipUnless(os.path.isdir(REAL_MAP), "真实地图目录不存在")
    def test_build_real_map_directory(self):
        definition = map_import.build_map(REAL_MAP)
        self.assertEqual(definition["import_meta"]["source_width"], 1180)
        self.assertAlmostEqual(definition["import_meta"]["resolution"], 0.05)
        self.assertLessEqual(max(definition["width"], definition["height"]), 24.5)
        self.assertGreaterEqual(len(definition["obstacles"]), 1)


class OpenFieldTest(unittest.TestCase):
    def test_open_field_has_no_boundary_obstacles(self):
        definition = map_import.build_open_field(6.0, 30.0)
        self.assertTrue(definition["open_field"])
        self.assertGreaterEqual(definition["width"], map_import.OPEN_FIELD_MIN)
        self.assertGreaterEqual(definition["height"], 30.0 + map_import.OPEN_FIELD_MARGIN - 1.0)
        self.assertEqual(definition["obstacles"], [])
        self.assertEqual(len(definition["polygon"]), 4)
        self.assertAlmostEqual(definition["dock"]["x"], definition["width"] / 2.0, places=3)
        self.assertAlmostEqual(definition["start"]["y"], definition["height"] / 2.0, places=3)

    def test_open_field_covers_small_and_large_tracks(self):
        small = map_import.build_open_field(0.2, 0.3)
        large = map_import.build_open_field(28.0, 4.0)
        huge = map_import.build_open_field(180.0, 4.0)
        # 空白场地固定 100 × 100 m 起：小轨迹也给这么大
        self.assertGreaterEqual(small["width"], 100.0)
        self.assertGreaterEqual(small["height"], 100.0)
        self.assertAlmostEqual(small["width"], small["height"], places=3)
        self.assertAlmostEqual(large["width"], small["width"], places=3)
        # 轨迹比场地还长时继续放大
        self.assertGreater(huge["width"], small["width"])
        self.assertGreaterEqual(huge["width"], 180.0)

    def test_open_field_matches_raster_pipeline(self):
        """空白场地直接按矩形构造：结果必须和"栅格 → 提边界 → 简化"那套完全一致。

        100 万格的空白栅格跑一遍通用流程要 4 s，连续播包切到无图日志时界面就卡在
        这一次上，所以改成了几何直出；字段值必须一模一样，否则场地尺寸/边缘会漂。
        """

        def legacy(span_x, span_y):
            width = max(
                map_import.OPEN_FIELD_MIN, float(span_x) + map_import.OPEN_FIELD_MARGIN
            )
            height = max(
                map_import.OPEN_FIELD_MIN, float(span_y) + map_import.OPEN_FIELD_MARGIN
            )
            cols = max(8, int(round(width / world.CELL)))
            rows = max(8, int(round(height / world.CELL)))
            raw = bytearray([254]) * (cols * rows)
            tag = "{}x{}".format(round(width, 1), round(height, 1))
            definition = map_import._definition(
                cols,
                rows,
                raw,
                world.CELL,
                "probe",
                source_path="bag:open:{}".format(tag),
                map_file="open:{}".format(tag),
                max_extent=max(width, height) + 1.0,
            )
            definition["name"] = "probe"
            definition["open_field"] = True
            cx = round(definition["width"] / 2.0, 3)
            cy = round(definition["height"] / 2.0, 3)
            definition["dock"] = {"x": cx, "y": cy, "r": 0.45}
            definition["start"] = {"x": cx, "y": cy, "theta": 0.0}
            return definition

        for span in ((0.0, 0.0), (0.2, 0.3), (28.0, 4.0), (86.0, 86.0)):
            expected = legacy(*span)
            got = map_import.build_open_field(*span, name="probe")
            self.assertEqual(got["width"], expected["width"], "span {}".format(span))
            self.assertEqual(got["height"], expected["height"], "span {}".format(span))
            self.assertEqual(
                [tuple(round(v, 4) for v in point) for point in got["polygon"]],
                [tuple(round(v, 4) for v in point) for point in expected["polygon"]],
                "span {}".format(span),
            )
            self.assertEqual(got["import_meta"], expected["import_meta"])
            self.assertEqual(got["dock"], expected["dock"])
            self.assertEqual(got["start"], expected["start"])


if __name__ == "__main__":
    unittest.main()
