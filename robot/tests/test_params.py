import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import params  # noqa: E402

BACKEND_DIR = params.BACKEND_DIR
if BACKEND_DIR not in sys.path:
    sys.path.insert(0, BACKEND_DIR)

import config  # noqa: E402
import world  # noqa: E402
from grid_map import GridMap  # noqa: E402
from robot_car import RobotCar  # noqa: E402
from tasks import TaskManager  # noqa: E402


class RobotParameterTest(unittest.TestCase):
    def tearDown(self):
        params.reset_params()

    def test_apply_robot_and_mowing_parameters(self):
        settings = params.apply_params(
            robot_radius=0.22,
            mower_width=0.50,
            vision_range=1.4,
            vision_rays=61,
            mowing_mode="B",
            battery_low=25.0,
            battery_resume=75.0,
        )
        self.assertAlmostEqual(world.ROBOT_RADIUS, 0.22)
        self.assertAlmostEqual(world.MOWER_WIDTH, 0.50)
        self.assertAlmostEqual(world.VISION_RANGE, 1.4)
        self.assertEqual(world.VISION_RAYS, 61)
        self.assertAlmostEqual(settings.battery_low, 25.0)
        self.assertAlmostEqual(settings.battery_resume, 75.0)
        world.select_map(1)
        car = RobotCar()
        self.assertEqual(len(car.scan()), 61)
        manager = TaskManager(car, GridMap())
        self.assertEqual(manager.mowing_mode, "B")

    def test_mode_is_validated(self):
        with self.assertRaises(ValueError):
            config.configure(mowing_mode="C")


if __name__ == "__main__":
    unittest.main()
