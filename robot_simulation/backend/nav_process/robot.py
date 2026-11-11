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
        # 导航速度源: 点到点任务期间由 tea cmd_vel 驱动, 与手动 drive 互斥
        self.nav_mode = False
        self.nav_v = 0.0
        self.nav_w = 0.0
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
            if self.nav_mode:
                # 导航期间手动输入由 server 409 拦截, 这里兜底忽略, 防止速度源打架
                return
            self.target_v = max(-0.60, min(0.60, float(linear)))
            self.target_w = max(-1.80, min(1.80, float(angular)))

    def drive_nav(self, linear, angular):
        """导航速度源入口: 仅导航模式下生效, 限幅与手动一致。"""

        with self.lock:
            if not self.nav_mode:
                return
            self.nav_v = max(-0.60, min(0.60, float(linear)))
            self.nav_w = max(-1.80, min(1.80, float(angular)))

    def set_nav_mode(self, active):
        """进入/退出导航模式。进入时清手动目标避免残留速度; 退出时清导航目标停车。"""

        with self.lock:
            self.nav_mode = bool(active)
            if self.nav_mode:
                self.target_v = 0.0
                self.target_w = 0.0
            self.nav_v = 0.0
            self.nav_w = 0.0

    def stop(self):
        self.drive(0.0, 0.0)

    def step(self, dt):
        with self.lock:
            self.sim_time += dt
            target_v, target_w = (self.nav_v, self.nav_w) if self.nav_mode else (self.target_v, self.target_w)
            self.v = _approach(self.v, target_v, 1.3 * dt)
            self.w = _approach(self.w, target_w, 4.5 * dt)
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
