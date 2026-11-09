"""导航仿真的轻量机器人模型。

只提供 tea 桥需要的位姿、仿真时间和日志，不导入内部仿真的 world、地图或任务。
"""

import math
import threading
from collections import deque


def _approach(current, target, max_delta):
    if current < target:
        return min(current + max_delta, target)
    return max(current - max_delta, target)


class NavRobot(object):
    def __init__(self):
        self.lock = threading.RLock()
        self.events = deque(maxlen=500)
        self.pose = [0.0, 0.0, 0.0]
        self.v = 0.0
        self.w = 0.0
        self.target_v = 0.0
        self.target_w = 0.0
        self.sim_time = 0.0
        self.distance = 0.0
        self.battery = 100.0
        self.log("info", "导航进程仿真已就绪")

    def log(self, level, message):
        self.events.append(
            {"time": round(self.sim_time, 2), "level": level, "message": message}
        )

    def drive(self, linear, angular):
        with self.lock:
            self.target_v = max(-0.60, min(0.60, float(linear)))
            self.target_w = max(-1.80, min(1.80, float(angular)))

    def stop(self):
        self.drive(0.0, 0.0)

    def step(self, dt):
        with self.lock:
            self.sim_time += dt
            self.v = _approach(self.v, self.target_v, 1.3 * dt)
            self.w = _approach(self.w, self.target_w, 4.5 * dt)
            x, y, theta = self.pose
            self.pose = [
                x + self.v * math.cos(theta) * dt,
                y + self.v * math.sin(theta) * dt,
                (theta + self.w * dt + math.pi) % (2 * math.pi) - math.pi,
            ]
            self.distance += abs(self.v) * dt

    def get_state(self):
        with self.lock:
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
                    "left_wheel": round(self.v - self.w * 0.13, 3),
                    "right_wheel": round(self.v + self.w * 0.13, 3),
                },
                "battery": round(self.battery, 1),
                "distance": round(self.distance, 2),
                "log": list(self.events),
            }
