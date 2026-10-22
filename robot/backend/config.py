"""集中式可配置参数。

测试、控制台或二次开发只需要调用 ``configure(...)`` 即可覆盖默认值，
无需改动 world/robot/tasks 中的实现常量。
"""

import math


class Settings(object):
    def __init__(self):
        # 机器人本体
        self.robot_radius = 0.18
        self.mower_width = 0.44
        self.track_width = 0.26
        self.wheelbase = 0.30
        self.max_linear = 0.60
        self.max_angular = 1.80
        # 视觉
        self.vision_fov_deg = 100.0
        self.vision_range = 1.0
        self.vision_rays = 41
        # 电量
        self.battery_low = 30.0
        self.battery_resume = 80.0
        self.charge_rate = 8.0
        self.battery_drain_factor = 0.6
        # 割草
        self.mowing_mode = "A"
        self.edge_inset = 0.22
        self.lane_spacing = 0.30
        self.ring_margin = 0.34
        self.nav_speed = 0.38
        self.map_speed = 0.50
        self.mow_speed = 0.48
        # 障碍物分类（来自视觉建图，不使用真值）
        self.large_obstacle_extent = 0.85
        self.small_obstacle_radius = 0.45
        # 弓字重规划 / 断点
        self.max_zone_plan_passes = 8


settings = Settings()

_ALLOWED_MODES = ("A", "B")
_RANGES = {
    "robot_radius": (0.08, 0.50),
    "mower_width": (0.10, 1.00),
    "track_width": (0.10, 0.80),
    "wheelbase": (0.10, 0.80),
    "max_linear": (0.10, 2.00),
    "max_angular": (0.20, 4.00),
    "vision_fov_deg": (30.0, 180.0),
    "vision_range": (0.30, 4.00),
    "vision_rays": (9, 181),
    "battery_low": (5.0, 60.0),
    "battery_resume": (40.0, 100.0),
    "charge_rate": (0.5, 30.0),
    "battery_drain_factor": (0.10, 2.00),
    "edge_inset": (0.12, 0.60),
    "lane_spacing": (0.15, 0.80),
    "ring_margin": (0.20, 1.50),
    "nav_speed": (0.10, 2.00),
    "map_speed": (0.10, 2.00),
    "mow_speed": (0.10, 2.00),
    "large_obstacle_extent": (0.40, 2.00),
    "small_obstacle_radius": (0.10, 1.00),
    "max_zone_plan_passes": (1, 30),
}


def configure(**kwargs):
    """覆盖参数并立即应用到各模块。"""
    for key, value in kwargs.items():
        if not hasattr(settings, key):
            raise ValueError("未知配置项: {}".format(key))
        if key == "mowing_mode":
            value = str(value).upper()
            if value not in _ALLOWED_MODES:
                raise ValueError("割草模式仅支持 A/B")
        elif key in _RANGES:
            low, high = _RANGES[key]
            try:
                value = float(value)
            except (TypeError, ValueError):
                raise ValueError("参数 {} 必须是数字".format(key))
            if not (low <= value <= high):
                raise ValueError("参数 {} 超出范围 {}-{}".format(key, low, high))
            if key in ("vision_rays", "max_zone_plan_passes"):
                value = int(round(value))
        setattr(settings, key, value)
    apply()


def apply():
    """把配置同步到 world / planner / robot_car / tasks 的运行时变量。"""
    import grid_map
    import robot_car
    import planner
    import tasks
    import world

    world.ROBOT_RADIUS = float(settings.robot_radius)
    world.MOWER_WIDTH = float(settings.mower_width)
    world.TRACK_WIDTH = float(settings.track_width)
    world.WHEEL_BASE = float(settings.wheelbase)
    world.VISION_FOV = math.radians(float(settings.vision_fov_deg))
    world.VISION_RANGE = float(settings.vision_range)
    world.VISION_RAYS = max(9, int(settings.vision_rays))

    robot_car.MAX_LINEAR = float(settings.max_linear)
    robot_car.MAX_ANGULAR = float(settings.max_angular)
    robot_car.CHARGE_RATE = float(settings.charge_rate)
    robot_car.DRAIN_FACTOR = float(settings.battery_drain_factor)

    planner.EDGE_INSET = float(settings.edge_inset)
    planner.LANE_SPACING = float(settings.lane_spacing)
    planner.RING_MARGIN = float(settings.ring_margin)

    tasks.LOW_BATTERY = float(settings.battery_low)
    tasks.RESUME_BATTERY = float(settings.battery_resume)
    tasks.NAV_SPEED = float(settings.nav_speed)
    tasks.MAP_SPEED = float(settings.map_speed)
    tasks.MOW_SPEED = float(settings.mow_speed)
    tasks.DEFAULT_MOWING_MODE = settings.mowing_mode
    tasks.MAX_ZONE_PLAN_PASSES = int(settings.max_zone_plan_passes)

    grid_map.LARGE_OBSTACLE_EXTENT = float(settings.large_obstacle_extent)
    grid_map.SMALL_OBSTACLE_RADIUS = float(settings.small_obstacle_radius)


def reset():
    global settings
    settings = Settings()
    apply()


apply()
