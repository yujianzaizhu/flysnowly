"""A/B 割草模式效率对比脚本（独立运行，不参与 unittest）。

用法:
    cd robot
    PYTHONPATH=backend python3 tests/benchmark_modes.py
"""

import json
import os
import sys

BACKEND_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "backend")
if BACKEND_DIR not in sys.path:
    sys.path.insert(0, BACKEND_DIR)

import config  # noqa: E402
import world  # noqa: E402
from grid_map import GridMap  # noqa: E402
from robot_car import RobotCar  # noqa: E402
from tasks import TaskManager  # noqa: E402

DT = 0.02
MAPS = (1, 3, 4)


def make_system(map_id):
    world.select_map(map_id)
    robot = RobotCar()
    grid = GridMap()
    manager = TaskManager(robot, grid)
    return robot, grid, manager


def map_field(robot, grid, manager):
    manager.start_mapping()
    for _ in range(int(300 / DT)):
        manager.tick(DT)
        if manager.phase == "done":
            break
    return [zone["id"] for zone in grid.zones]


def run_mode(map_id, zone_id, mode):
    config.reset()
    # 关闭电量回充对策略效率对比的干扰，只比较 A/B 的规划与行驶效率。
    config.configure(battery_low=5.0, battery_resume=40.0)
    robot, grid, manager = make_system(map_id)
    zones = map_field(robot, grid, manager)
    if zone_id not in zones:
        return None
    zone = next(item for item in grid.zones if item["id"] == zone_id)
    robot.battery = 100.0
    start_time = robot.sim_time
    start_distance = robot.distance_traveled
    manager.start_mowing([zone_id], mode=mode)
    for _ in range(int(900 / DT)):
        manager.tick(DT)
        if manager.phase == "done":
            break
    duration = robot.sim_time - start_time
    return {
        "map": map_id,
        "zone": zone_id,
        "mode": mode,
        "area_m2": round(zone["free"] * grid.cell * grid.cell, 2),
        "duration_s": round(duration, 1),
        "distance_m": round(robot.distance_traveled - start_distance, 2),
        "coverage_pct": round(grid.zone_coverage(zone_id) * 100.0, 2),
        "collisions": robot.collisions,
        "segments": manager.stats.get("segments_planned", 0),
        "rings": manager.stats.get("rings", 0),
        "large_replans": manager.stats.get("large_replans", 0),
        "charges": manager.stats.get("charge_count", 0),
        "stuck_recoveries": manager.stats.get("stuck_recoveries", 0),
        "phase": manager.phase,
    }


def main():
    rows = []
    for map_id in MAPS:
        # 先建图确定同一区域，再分别用 A/B 模式执行
        robot, grid, manager = make_system(map_id)
        zone_ids = map_field(robot, grid, manager)
        if not zone_ids:
            continue
        zone = min(grid.zones, key=lambda item: item["free"])
        zone_id = zone["id"]
        for mode in ("A", "B"):
            result = run_mode(map_id, zone_id, mode)
            if result:
                rows.append(result)
                print(json.dumps(result, ensure_ascii=False), flush=True)
    with open("/tmp/mode_benchmark.json", "w") as handle:
        json.dump(rows, handle, ensure_ascii=False, indent=2)
    print("DONE")


if __name__ == "__main__":
    main()
