"""四轮后驱小车模型：运动学、视觉传感器、电量与充电。

四个轮子中后轮由左右两个电机差速驱动，前轮为随动轮；
运动学按后轴差速模型计算，视觉传感器为 100° FOV / 1 m 深度相机。
"""

import math
import threading
from collections import deque

import world

MAX_LINEAR = 0.60
MAX_ANGULAR = 1.8
LINEAR_ACCEL = 1.3
ANGULAR_ACCEL = 4.5

CHARGE_RATE = 8.0
IDLE_DRAIN = 0.02
DRAIN_FACTOR = 0.6

COMMANDS = {
    "forward": (0.55, 0.0),
    "backward": (-0.32, 0.0),
    "left": (0.0, MAX_ANGULAR),
    "right": (0.0, -MAX_ANGULAR),
    "forward_left": (0.4, 1.1),
    "forward_right": (0.4, -1.1),
    "backward_left": (-0.28, 1.1),
    "backward_right": (-0.28, -1.1),
    "stop": (0.0, 0.0),
}


def approach(current, target, max_delta):
    if current < target:
        return min(current + max_delta, target)
    return max(current - max_delta, target)


class RobotCar(object):
    def __init__(self):
        self.lock = threading.RLock()
        self.events = deque(maxlen=80)
        self.reset(first_run=True)

    # ----------------------------------------------------------- 状态
    def reset(self, first_run=False):
        with self.lock:
            self.pose = [world.START["x"], world.START["y"], world.START["theta"]]
            self.v = 0.0
            self.w = 0.0
            self.target_v = 0.0
            self.target_w = 0.0
            self.battery = 100.0
            self.charging = False
            self.depleted = False
            self.collisions = 0
            self.bump_cooldown = 0.0
            self.distance_traveled = 0.0
            self.sim_time = 0.0
            self.vision = world.vision_scan(*self.pose)
            self.events.clear()
            self.log("info", "系统就绪，等待任务" if first_run else "小车已复位")

    def pose_tuple(self):
        return self.pose[0], self.pose[1], self.pose[2]

    def log(self, level, message):
        self.events.append(
            {
                "time": round(self.sim_time, 2),
                "level": level,
                "message": message,
            }
        )

    # ----------------------------------------------------------- 指令
    def handle_command(self, name):
        if name not in COMMANDS:
            raise ValueError("未知指令: {}".format(name))
        with self.lock:
            self.target_v, self.target_w = COMMANDS[name]

    def drive(self, linear, angular):
        with self.lock:
            self.target_v = max(-MAX_LINEAR, min(MAX_LINEAR, linear))
            self.target_w = max(-MAX_ANGULAR, min(MAX_ANGULAR, angular))

    def stop(self):
        self.drive(0.0, 0.0)

    # ----------------------------------------------------------- 传感器
    def scan(self):
        with self.lock:
            self.vision = world.vision_scan(*self.pose)
            return list(self.vision)

    # ----------------------------------------------------------- 仿真
    def step(self, dt):
        if dt <= 0:
            return
        with self.lock:
            self.sim_time += dt
            if self.battery <= 0.0:
                self.target_v = 0.0
                self.target_w = 0.0

            self.v = approach(self.v, self.target_v, LINEAR_ACCEL * dt)
            self.w = approach(self.w, self.target_w, ANGULAR_ACCEL * dt)

            x, y, theta = self.pose
            ntheta = world.wrap_angle(theta + self.w * dt)
            nx = x + self.v * math.cos(theta) * dt
            ny = y + self.v * math.sin(theta) * dt

            if self._collides(nx, ny):
                self.v = 0.0
                self.pose[2] = ntheta
                if self.bump_cooldown <= 0.0:
                    self.collisions += 1
                    self.bump_cooldown = 0.6
                    self.log("warn", "检测到碰撞，已制动")
            else:
                self.pose = [nx, ny, ntheta]
                self.distance_traveled += abs(self.v) * dt

            self.bump_cooldown = max(0.0, self.bump_cooldown - dt)
            self._update_battery(dt)

    def _collides(self, x, y):
        return world.collision(x, y)

    def _update_battery(self, dt):
        distance = math.hypot(world.DOCK["x"] - self.pose[0], world.DOCK["y"] - self.pose[1])
        was_charging = self.charging
        self.charging = distance < world.DOCK["r"] and abs(self.v) < 0.05

        if self.charging:
            if self.battery < 100.0:
                self.battery = min(100.0, self.battery + CHARGE_RATE * dt)
            if not was_charging:
                self.log("event", "已停靠充电桩，开始充电")
        else:
            drain = DRAIN_FACTOR * (
                IDLE_DRAIN
                + 0.50 * abs(self.v) / MAX_LINEAR
                + 0.08 * abs(self.w) / MAX_ANGULAR
            )
            self.battery = max(0.0, self.battery - drain * dt)
            if was_charging:
                self.log("info", "离开充电桩")
            if self.battery <= 0.0 and not self.depleted:
                self.depleted = True
                self.target_v = 0.0
                self.target_w = 0.0
                self.log("warn", "电量耗尽，小车已停止")

    # ----------------------------------------------------------- 遥测
    def _vision_summary(self):
        if not self.vision:
            return {
                "front": world.VISION_RANGE,
                "left": world.VISION_RANGE,
                "right": world.VISION_RANGE,
                "nearest": world.VISION_RANGE,
                "ray_count": 0,
                "fov_deg": round(math.degrees(world.VISION_FOV), 1),
                "range": world.VISION_RANGE,
            }
        front = min(d for a, d in self.vision if abs(math.degrees(a)) <= 12.0)
        left = min(d for a, d in self.vision if 25.0 <= math.degrees(a) <= 50.0)
        right = min(d for a, d in self.vision if -50.0 <= math.degrees(a) <= -25.0)
        return {
            "front": round(front, 3),
            "left": round(left, 3),
            "right": round(right, 3),
            "nearest": round(min(d for _, d in self.vision), 3),
            "ray_count": len(self.vision),
            "fov_deg": round(math.degrees(world.VISION_FOV), 1),
            "range": world.VISION_RANGE,
        }

    def get_state(self):
        with self.lock:
            half = world.TRACK_WIDTH / 2.0
            return {
                "time": round(self.sim_time, 2),
                "pose": {
                    "x": round(self.pose[0], 3),
                    "y": round(self.pose[1], 3),
                    "theta": round(self.pose[2], 4),
                },
                "velocity": {
                    "linear": round(self.v, 3),
                    "angular": round(self.w, 3),
                    "left_wheel": round(self.v - self.w * half, 3),
                    "right_wheel": round(self.v + self.w * half, 3),
                },
                "battery": round(self.battery, 1),
                "charging": self.charging,
                "depleted": self.depleted,
                "collisions": self.collisions,
                "distance": round(self.distance_traveled, 2),
                "vision": self._vision_summary(),
                "robot": {
                    "radius": world.ROBOT_RADIUS,
                    "track": world.TRACK_WIDTH,
                    "wheelbase": world.WHEEL_BASE,
                    "max_speed": MAX_LINEAR,
                    "mower_width": world.MOWER_WIDTH,
                    "drive": "rear_diff",
                },
                "log": list(self.events),
            }
