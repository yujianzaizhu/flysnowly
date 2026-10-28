"""测试/外部参数入口。

修改下面的字典后调用 ``apply_params()``，即可覆盖机器人本体、视觉、
电量阈值、割草模式与障碍物分类参数。所有测试和仿真保持同一套配置。
"""

import os
import sys

BACKEND_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "backend"
)
if BACKEND_DIR not in sys.path:
    sys.path.insert(0, BACKEND_DIR)

import config  # noqa: E402

ROBOT_PARAMS = {
    "robot_radius": 0.18,
    "mower_width": 0.44,
    "track_width": 0.26,
    "wheelbase": 0.30,
    "max_linear": 0.60,
    "max_angular": 1.80,
    "vision_fov_deg": 100.0,
    "vision_range": 1.0,
    "vision_rays": 41,
}

BATTERY_PARAMS = {
    "battery_low": 30.0,
    "battery_resume": 80.0,
    "charge_rate": 8.0,
    "battery_drain_factor": 0.6,
    "dock_radius": 0.70,
}

MOWING_PARAMS = {
    "mowing_mode": "A",
    "edge_inset": 0.22,
    "lane_spacing": 0.30,
    "ring_margin": 0.34,
    "nav_speed": 0.38,
    "map_speed": 0.50,
    "mow_speed": 0.48,
    "edge_speed": 0.50,
    "inner_corner_advance": 0.16,
    "large_obstacle_area": 5.0,
    "small_obstacle_radius": 0.45,
    "max_zone_plan_passes": 30,
    "min_zone_area": 20.0,
}


def apply_params(**overrides):
    """把三组参数字典合并后写入运行时配置。"""
    merged = {}
    merged.update(ROBOT_PARAMS)
    merged.update(BATTERY_PARAMS)
    merged.update(MOWING_PARAMS)
    merged.update(overrides)
    config.configure(**merged)
    return config.settings


def reset_params():
    config.reset()
