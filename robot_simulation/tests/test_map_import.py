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


if __name__ == "__main__":
    unittest.main()
