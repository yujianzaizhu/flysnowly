import math
import os
import sys
import unittest

BACKEND_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "backend"
)
sys.path.insert(0, BACKEND_DIR)

import robot_car  # noqa: E402
import world  # noqa: E402
from robot_car import RobotCar  # noqa: E402


def run(car, seconds, dt=0.02):
    for _ in range(int(round(seconds / dt))):
        car.step(dt)


class KinematicsTest(unittest.TestCase):
    def setUp(self):
        world.select_map(1)

    def test_forward_moves_along_heading(self):
        car = RobotCar()
        car.pose = [1.0, 7.8, 0.0]
        start = list(car.pose)
        car.handle_command("forward")
        run(car, 2.0)
        self.assertGreater(car.pose[0], start[0] + 0.2)
        self.assertAlmostEqual(car.pose[1], start[1], places=6)
        self.assertGreater(car.v, 0.3)

    def test_left_turn_increases_heading(self):
        car = RobotCar()
        car.handle_command("left")
        run(car, 0.6)
        self.assertGreater(car.pose[2], 0.2)
        self.assertLess(abs(car.pose[0] - world.START["x"]), 1e-6)

    def test_rear_wheels_differential(self):
        car = RobotCar()
        car.handle_command("left")
        run(car, 0.8)
        state = car.get_state()
        self.assertGreater(state["velocity"]["right_wheel"], state["velocity"]["left_wheel"])
        self.assertEqual(state["robot"]["drive"], "rear_diff")

    def test_wall_stops_robot(self):
        car = RobotCar()
        car.pose = [world.WIDTH - 0.70, world.HEIGHT - 1.50, 0.0]
        car.handle_command("forward")
        run(car, 2.0)
        self.assertLessEqual(car.pose[0], world.WIDTH - world.ROBOT_RADIUS + 1e-6)
        self.assertGreaterEqual(car.collisions, 1)
        self.assertAlmostEqual(car.v, 0.0, places=6)


class VisionTest(unittest.TestCase):
    def setUp(self):
        world.select_map(1)

    def test_vision_has_100_degree_fov(self):
        car = RobotCar()
        rays = car.scan()
        self.assertEqual(len(rays), world.VISION_RAYS)
        angles = [math.degrees(a) for a, _ in rays]
        self.assertAlmostEqual(max(angles), 50.0, delta=0.1)
        self.assertAlmostEqual(min(angles), -50.0, delta=0.1)

    def test_front_distance_to_wall(self):
        car = RobotCar()
        car.pose = [world.WIDTH - 0.70, world.HEIGHT - 1.50, 0.0]
        car.scan()
        self.assertAlmostEqual(car.get_state()["vision"]["front"], 0.7, places=2)

    def test_front_distance_to_obstacle(self):
        car = RobotCar()
        target = min(
            world.OBSTACLES,
            key=lambda item: abs(item[0] - 2.0 * world.MAP_SCALE)
            + abs(item[1] - 6.7 * world.MAP_SCALE),
        )
        car.pose = [target[0] - target[2] - 0.45, target[1], 0.0]
        car.scan()
        self.assertAlmostEqual(car.get_state()["vision"]["front"], 0.45, places=2)

    def test_vision_range_is_one_metre(self):
        car = RobotCar()
        car.pose = [5.0, 4.0, 0.0]
        car.scan()
        self.assertLessEqual(max(d for _, d in car.vision), 1.0 + 1e-9)


class BatteryTest(unittest.TestCase):
    def setUp(self):
        world.select_map(1)

    def test_driving_drains_battery(self):
        car = RobotCar()
        car.pose = [1.0, 7.8, 0.0]
        car.handle_command("forward")
        run(car, 5.0)
        self.assertLess(car.battery, 100.0)

    def test_idle_drain_is_ten_percent_of_normal(self):
        car = RobotCar()
        car.battery = 100.0
        run(car, 10.0)
        normal = robot_car.DRAIN_FACTOR * (
            robot_car.IDLE_DRAIN + 0.50
        )
        self.assertAlmostEqual(
            car.battery, 100.0 - normal * 0.1 * 10.0, places=2
        )

    def test_driving_drains_faster_than_idle(self):
        car = RobotCar()
        car.battery = 100.0
        car.pose = [1.0, 7.8, 0.0]
        car.handle_command("forward")
        run(car, 4.0)
        driving = 100.0 - car.battery
        car.stop()
        car.battery = 100.0
        run(car, 4.0)
        self.assertGreater(driving, 100.0 - car.battery)

    def test_dock_charges_battery(self):
        car = RobotCar()
        car.battery = 50.0
        car.pose = [world.DOCK["x"], world.DOCK["y"], 0.0]
        run(car, 2.0)
        self.assertTrue(car.charging)
        self.assertGreater(car.battery, 55.0)

    def test_depleted_battery_stops_robot(self):
        car = RobotCar()
        car.pose = [1.0, 7.8, 0.0]
        car.battery = 0.2
        car.handle_command("forward")
        run(car, 5.0)
        self.assertTrue(car.depleted)
        self.assertAlmostEqual(car.v, 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
