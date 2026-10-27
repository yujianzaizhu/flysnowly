"""A/B 割草模式整图效率对比（多地图、多次试验、并行执行）。

用法:
    cd robot
    python3 tests/benchmark_modes.py
    python3 tests/benchmark_modes.py --trials 15 --workers 8

每次试验都会从随机自由位置出发，以随机区域顺序割完整张地图；A/B 两种
模式使用完全相同的起始位姿和区域顺序。默认 5 张地图 × 2 种模式 × 11 次，
共 110 次整图任务。结果写入 /tmp/mode_benchmark_full.json。
"""

import argparse
import copy
import json
import math
import multiprocessing
import os
import pickle
import random
import statistics
import sys
import time

BACKEND_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "backend")
if BACKEND_DIR not in sys.path:
    sys.path.insert(0, BACKEND_DIR)

import config  # noqa: E402
import world  # noqa: E402
from grid_map import GridMap  # noqa: E402
from robot_car import RobotCar  # noqa: E402
from tasks import TaskManager  # noqa: E402

MAPS = (1, 2, 3, 4, 5)
MODES = ("A", "B")
MAP_DT = 0.05
RUN_DT = 0.08
MAX_MAP_SECONDS = 420.0
MAX_MOW_SECONDS = 3000.0
RESULT_PATH = "/tmp/mode_benchmark_full.json"
GRID_DIR = "/tmp/robot_benchmark_grids"


def _grid_path(map_id):
    return os.path.join(GRID_DIR, "map{}.pkl".format(map_id))


def _configure():
    config.reset()
    # 只比较规划/行驶效率，关闭低电回充对试验的干扰。
    config.configure(battery_low=5.0, battery_resume=40.0)


def _prepare_map(map_id):
    _configure()
    world.select_map(map_id)
    robot = RobotCar()
    grid = GridMap()
    manager = TaskManager(robot, grid)
    manager.start_mapping()
    ticks = int(MAX_MAP_SECONDS / MAP_DT)
    for index in range(ticks):
        if index % 20 == 0:
            robot.battery = 100.0
        manager.tick(MAP_DT)
        if manager.phase == "done":
            break
    os.makedirs(GRID_DIR, exist_ok=True)
    with open(_grid_path(map_id), "wb") as handle:
        pickle.dump(grid, handle)
    return {
        "map": map_id,
        "phase": manager.phase,
        "sim_time": round(robot.sim_time, 1),
        "zones": len(grid.zones),
        "free_cells": grid.free_total,
        "collisions": robot.collisions,
    }


def _load_grid(map_id):
    with open(_grid_path(map_id), "rb") as handle:
        return pickle.load(handle)


def _trial_seed(map_id, trial):
    return map_id * 10000 + trial * 97 + 13


def _pick_start(grid, rng):
    # 在起点/充电桩附近选取开阔自由点，保证与全场连通且不会一出生就
    # 卡在狭窄墙角；不同试验通过位置微调和区域顺序形成差异。
    base_x, base_y = world.START["x"], world.START["y"]
    candidates = []
    for row in range(grid.rows):
        for col in range(grid.cols):
            index = grid.index(col, row)
            if not grid.free_mask[index] or grid.blocked[index]:
                continue
            x, y = grid.center_of(col, row)
            if math.hypot(x - base_x, y - base_y) > 1.8:
                continue
            if world.boundary_distance(x, y) < 0.58:
                continue
            if any(
                math.hypot(x - ox, y - oy) < oradius + world.ROBOT_RADIUS + 0.25
                for ox, oy, oradius in world.OBSTACLES
            ):
                continue
            if world.collision(x, y, world.ROBOT_RADIUS + 0.03):
                continue
            candidates.append((x, y))
    if candidates:
        return candidates[rng.randrange(len(candidates))]
    return base_x, base_y


def _coverage(grid):
    total = sum(grid.zone_totals.values())
    mowed = sum(grid.zone_mowed.values())
    return (mowed / float(total)) if total else 0.0


def _run_one(map_id, trial, mode, grid, start, order, theta):
    robot = RobotCar()
    robot.pose = [start[0], start[1], theta]
    robot.battery = 100.0
    manager = TaskManager(robot, grid)
    start_time = robot.sim_time
    start_distance = robot.distance_traveled
    manager.start_mowing(order, mode=mode)
    ticks = int(MAX_MOW_SECONDS / RUN_DT)
    for index in range(ticks):
        if index % 25 == 0:
            robot.battery = 100.0
        manager.tick(RUN_DT)
        if manager.phase == "done":
            break
    return {
        "map": map_id,
        "trial": trial,
        "mode": mode,
        "start": [round(start[0], 2), round(start[1], 2)],
        "theta": round(theta, 2),
        "order": list(order),
        "success": manager.phase == "done",
        "phase": manager.phase,
        "duration_s": round(robot.sim_time - start_time, 1),
        "distance_m": round(robot.distance_traveled - start_distance, 2),
        "coverage_pct": round(_coverage(grid) * 100.0, 2),
        "collisions": robot.collisions,
        "stuck_recoveries": manager.stats.get("stuck_recoveries", 0),
        "rings": manager.stats.get("rings", 0),
        "large_replans": manager.stats.get("large_replans", 0),
        "edge_skips": manager.stats.get("edge_skips", 0),
        "segments": manager.stats.get("segments_planned", 0),
    }


def _run_trial(task):
    map_id, trial = task
    _configure()
    world.select_map(map_id)
    grid = _load_grid(map_id)
    zone_ids = [zone["id"] for zone in grid.zones]
    if not zone_ids:
        return []
    rng = random.Random(_trial_seed(map_id, trial))
    order = list(zone_ids)
    rng.shuffle(order)
    start = _pick_start(grid, rng)
    # 朝向场地中心，避免出生时正对墙角造成无效的原地转向。
    theta = math.atan2(world.HEIGHT / 2.0 - start[1], world.WIDTH / 2.0 - start[0])
    rows = []
    for mode in MODES:
        run_grid = copy.deepcopy(grid)
        run_grid.reset_mowed()
        rows.append(_run_one(map_id, trial, mode, run_grid, start, order, theta))
    return rows


def _summary_row(rows):
    if not rows:
        return None
    durations = [row["duration_s"] for row in rows]
    distances = [row["distance_m"] for row in rows]
    coverages = [row["coverage_pct"] for row in rows]
    return {
        "runs": len(rows),
        "success_pct": round(100.0 * sum(1 for row in rows if row["success"]) / len(rows), 1),
        "duration_mean": round(statistics.mean(durations), 1),
        "duration_median": round(statistics.median(durations), 1),
        "duration_p95": round(sorted(durations)[max(0, int(len(durations) * 0.95) - 1)], 1),
        "distance_mean": round(statistics.mean(distances), 2),
        "coverage_mean": round(statistics.mean(coverages), 2),
        "collisions": sum(row["collisions"] for row in rows),
        "stuck": sum(row["stuck_recoveries"] for row in rows),
        "rings": sum(row["rings"] for row in rows),
        "large_replans": sum(row["large_replans"] for row in rows),
    }


def _summarize(rows):
    summary = {"overall": {}, "by_map": {}}
    for mode in MODES:
        selected = [row for row in rows if row["mode"] == mode]
        summary["overall"][mode] = _summary_row(selected)
    for map_id in MAPS:
        summary["by_map"][str(map_id)] = {}
        for mode in MODES:
            selected = [
                row
                for row in rows
                if row["mode"] == mode and row["map"] == map_id
            ]
            summary["by_map"][str(map_id)][mode] = _summary_row(selected)

    paired = []
    keyed = {(row["map"], row["trial"], row["mode"]): row for row in rows}
    for map_id in MAPS:
        for trial in sorted({row["trial"] for row in rows if row["map"] == map_id}):
            a = keyed.get((map_id, trial, "A"))
            b = keyed.get((map_id, trial, "B"))
            if not a or not b:
                continue
            paired.append(
                {
                    "map": map_id,
                    "trial": trial,
                    "duration_delta": round(a["duration_s"] - b["duration_s"], 1),
                    "distance_delta": round(a["distance_m"] - b["distance_m"], 2),
                    "coverage_delta": round(a["coverage_pct"] - b["coverage_pct"], 2),
                }
            )
    if paired:
        summary["paired"] = {
            "count": len(paired),
            "a_faster_runs": sum(1 for item in paired if item["duration_delta"] < 0),
            "a_shorter_runs": sum(1 for item in paired if item["distance_delta"] < 0),
            "duration_delta_mean": round(
                statistics.mean(item["duration_delta"] for item in paired), 1
            ),
            "distance_delta_mean": round(
                statistics.mean(item["distance_delta"] for item in paired), 2
            ),
            "coverage_delta_mean": round(
                statistics.mean(item["coverage_delta"] for item in paired), 2
            ),
        }
    return summary


def _print_summary(summary):
    print("\n=== 整图 A/B 汇总 ===")
    print("模式 | 次数 | 成功率 | 平均时长(s) | 中位时长(s) | P95(s) | 平均路程(m) | 平均覆盖(%) | 碰撞 | 卡滞恢复")
    for mode in MODES:
        row = summary["overall"][mode]
        print(
            "{} | {} | {}% | {} | {} | {} | {} | {} | {} | {}".format(
                mode,
                row["runs"],
                row["success_pct"],
                row["duration_mean"],
                row["duration_median"],
                row["duration_p95"],
                row["distance_mean"],
                row["coverage_mean"],
                row["collisions"],
                row["stuck"],
            )
        )
    print("\n=== 分地图平均时长 / 路程 / 覆盖 ===")
    print("地图 | A 时长 | B 时长 | A 路程 | B 路程 | A 覆盖 | B 覆盖")
    for map_id in MAPS:
        a = summary["by_map"][str(map_id)]["A"]
        b = summary["by_map"][str(map_id)]["B"]
        print(
            "{} | {} | {} | {} | {} | {} | {}".format(
                map_id,
                a["duration_mean"],
                b["duration_mean"],
                a["distance_mean"],
                b["distance_mean"],
                a["coverage_mean"],
                b["coverage_mean"],
            )
        )
    paired = summary.get("paired")
    if paired:
        print(
            "\n配对试验 {} 组：A 更快 {} 组，A 路程更短 {} 组；"
            "A-B 平均时长差 {} s，平均路程差 {} m，平均覆盖差 {} 个百分点".format(
                paired["count"],
                paired["a_faster_runs"],
                paired["a_shorter_runs"],
                paired["duration_delta_mean"],
                paired["distance_delta_mean"],
                paired["coverage_delta_mean"],
            )
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=11, help="每张地图每模式的试验次数")
    parser.add_argument("--workers", type=int, default=min(8, multiprocessing.cpu_count()))
    parser.add_argument("--skip-map", action="store_true", help="复用已有建图缓存")
    args = parser.parse_args()

    if not args.skip_map or not all(os.path.exists(_grid_path(m)) for m in MAPS):
        print("Preparing mapped grids for {} maps...".format(len(MAPS)), flush=True)
        with multiprocessing.Pool(processes=min(len(MAPS), args.workers)) as pool:
            mapped = pool.map(_prepare_map, MAPS)
        for item in mapped:
            print("mapped", json.dumps(item, ensure_ascii=False), flush=True)
        if any(item["phase"] != "done" for item in mapped):
            print("Mapping failed; abort benchmark.", flush=True)
            return 1

    tasks = [(map_id, trial) for map_id in MAPS for trial in range(args.trials)]
    rows = []
    started = time.time()
    with multiprocessing.Pool(processes=args.workers) as pool:
        for index, result in enumerate(pool.imap_unordered(_run_trial, tasks), 1):
            rows.extend(result)
            if index % 5 == 0 or index == len(tasks):
                elapsed = time.time() - started
                print(
                    "progress {}/{} trials, {:.0f}s elapsed".format(
                        index, len(tasks), elapsed
                    ),
                    flush=True,
                )
    rows.sort(key=lambda row: (row["map"], row["trial"], row["mode"]))
    summary = _summarize(rows)
    with open(RESULT_PATH, "w") as handle:
        json.dump({"rows": rows, "summary": summary}, handle, ensure_ascii=False, indent=2)
    _print_summary(summary)
    print("\nSaved to {}".format(RESULT_PATH))
    return 0


if __name__ == "__main__":
    sys.exit(main())
