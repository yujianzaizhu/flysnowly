"""任务控制：自动建图（含孤岛识别）、自动分区、割草、回充与断点续作。"""

import math
import threading
from collections import deque

import planner
import world

LOW_BATTERY = 30.0
RESUME_BATTERY = 80.0
DEFAULT_MOWING_MODE = "A"
MAX_ZONE_PLAN_PASSES = 30
PATCH_MIN_CELLS = 20
PATCH_MAX_CELLS = 900

NAV_SPEED = 0.38
MAP_SPEED = 0.50
MOW_SPEED = 0.48
EDGE_SPEED = 0.50
INNER_CORNER_ADVANCE = 0.16

MIN_OUTER_AREA = 12.0
ISLAND_AREA = 8.0

PHASE_LABELS = {
    "idle": "待机",
    "map_seek": "寻找场地边界",
    "map_follow": "逆时针巡边建图",
    "map_escape": "脱离孤岛",
    "stopped": "任务已暂停",
    "to_zone": "前往割草区域",
    "zone_edge": "沿区域边缘割草",
    "zone_plan": "规划弓字轨迹",
    "to_segment": "前往弓字起点",
    "zone_stripes": "弓字割草",
    "detour": "绕障并回到历史路径",
    "ring": "绕障碍物一周",
    "charge_go": "返回充电桩",
    "charging": "充电中",
    "charge_return": "返回任务断点",
    "align": "对齐断点航向",
    "to_dock": "返回充电桩",
    "done": "任务完成",
    "escape": "后退脱困",
    "corner_clean": "内直角清理",
}

MOW_PHASES = {"zone_edge", "zone_stripes", "detour", "ring", "corner_clean"}
CHARGE_EXEMPT = {
    "charging", "charge_go", "charge_return", "align", "done", "idle", "to_dock",
    "stopped",
}


class TaskManager(object):
    def __init__(self, robot, grid):
        self.robot = robot
        self.grid = grid
        self.lock = threading.RLock()
        self.task_serial = 0
        self._last_collisions = 0
        self._reset_state()

    # ----------------------------------------------------------- 基础状态
    def _reset_state(self):
        self.task = None
        self.phase = "idle"
        self.message = "等待任务"
        self.zone_order = []
        self.zone_index = 0
        self.zone_id = None
        self.zone_stage = None
        self.edge_points = []
        self._edge_route_cache = {}
        self.edge_index = 0
        self.segments = []
        self.segment_index = 0
        self.segment = None
        self.segment_end = None
        self.route = []
        self.route_index = 0
        self.nav_goal = None
        self.route_kind = None
        self.replan_cooldown = 0.0
        self.breakpoint = None
        self.charge_reason = None
        self.resume_requested = False
        self.zone_plan_passes = 0
        self.failed_segments = set()
        self.lane_axis = "h"
        self.rings_done = False
        self.mowing_mode = DEFAULT_MOWING_MODE
        self.edge_done = False
        self.edge_resume_target = -1
        self.edge_resume_attempts = 0
        self.edge_ringed_circles = []
        self.edge_corners = []
        self.edge_corner_done = set()
        self.corner_index = None
        self.corner_vertex = None
        self.corner_kind = None
        self.corner_in_heading = 0.0
        self.corner_stage = None
        self.corner_reverse_remaining = 0.0
        self.corner_time = 0.0
        self.edge_start_sim = None
        self.edge_guard_log_time = 0.0
        self.zone_bump_count = 0
        self.zone_escape_count = 0
        self.escape_path = []
        self.escape_index = 0
        self.escape_resume = None
        self.escape_time = 0.0
        self.patch_cleanup_done = False
        self.coverage_retries = 0
        self.ring_clipped = False
        self.ring_lost_time = 0.0
        self.ring_scan_time = 0.0
        self.ring_scan_heading = 0.0
        self.task_start_sim = None
        self.task_end_sim = None
        self.stats = {
            "segments_planned": 0,
            "rings": 0,
            "large_replans": 0,
            "edge_skips": 0,
            "stuck_recoveries": 0,
            "charge_count": 0,
        }
        self.ring_points = []
        self.ring_index = 0
        self.ring_after = None
        self.ring_started = False
        self.ring_circles = []
        self.ring_circle = None
        self.phase_time = 0.0
        self.last_phase = None
        self.stuck_time = 0.0
        self.stuck_ref = None
        self.detour_start = None
        self.detour_distance = 0.0
        self.detour_source = None
        self.align_target = 0.0
        self.bypassed_circles = []
        self.history = deque(maxlen=800)
        self._prev_pose = None

        self.map_path = []
        self.map_path_dist = 0.0
        self.map_islands = []
        self.map_seek_log_time = 0.0
        self.map_escape_heading = 0.0
        self.map_escape_time = 0.0
        self.map_escape_dist = 0.0

    def _log(self, level, message):
        self.message = message
        self.robot.log(level, message)

    def _pose(self):
        return self.robot.pose[0], self.robot.pose[1], self.robot.pose[2]

    def _zone(self, zone_id=None):
        target = self.zone_id if zone_id is None else zone_id
        for zone in self.grid.zones:
            if zone["id"] == target:
                return zone
        return None

    # ----------------------------------------------------------- 对外指令
    def start_mapping(self):
        with self.lock:
            self._reset_state()
            self.grid.reset_map()
            self.task_serial += 1
            self.task_start_sim = self.robot.sim_time
            self.task_end_sim = None
            self.task = {"type": "map", "mode": self.mowing_mode}
            self.phase = "map_seek"
            self._log("event", "开始自动建图：从当前位置逆时针巡边")

    def start_mowing(self, zone_order, mode=None):
        with self.lock:
            available = [zone["id"] for zone in self.grid.zones]
            if not available:
                raise ValueError("请先完成自动建图，生成割草区域")
            selected_mode = str(mode or DEFAULT_MOWING_MODE).upper()
            if selected_mode not in ("A", "B"):
                raise ValueError("割草模式仅支持 A/B")
            order = _validate_zones(zone_order, available)
            self._reset_state()
            self.mowing_mode = selected_mode
            self.task_serial += 1
            self.task_start_sim = self.robot.sim_time
            self.task_end_sim = None
            self.grid.reset_mowed()
            self.task = {"type": "mow", "zones": order, "mode": selected_mode}
            self.zone_order = order
            self.zone_index = 0
            self.zone_id = order[0]
            self.phase = "to_zone"
            self._log(
                "event",
                "开始自动割草（{} 模式），区域顺序：{}".format(
                    selected_mode, ", ".join(str(z) for z in order)
                ),
            )
            self._plan_to_zone_corner()

    def stop_task(self, reason="任务已停止", resumable=True):
        with self.lock:
            if not self.task and self.charge_reason is None:
                self.robot.stop()
                return
            if resumable:
                self._save_breakpoint()
                self.robot.stop()
                self.phase = "stopped"
                self.last_phase = None
                self.message = reason
                self._log("warn", "{}（点击继续任务可恢复）".format(reason))
            else:
                self._log("warn", reason)
                self._reset_state()
                self.robot.stop()

    def request_charge(self):
        with self.lock:
            if self.phase in ("charge_go", "charge_return", "charging"):
                return
            self._save_breakpoint()
            self.charge_reason = "manual"
            self.resume_requested = False
            self.phase = "charge_go"
            self._log("event", "收到手动回充指令，保存任务并返回充电桩")
            self._set_route((world.DOCK["x"], world.DOCK["y"]), "to_dock")

    def request_resume(self):
        with self.lock:
            if self.phase in ("stopped", "paused") and self.breakpoint:
                self._log("event", "收到继续任务指令，从暂停断点恢复")
                self._restore_breakpoint()
                return
            if self.phase not in ("charging", "charge_go"):
                return
            self.resume_requested = True
            self._log("event", "收到继续任务指令")

    def reset(self):
        with self.lock:
            self._reset_state()

    # ----------------------------------------------------------- 主循环
    def tick(self, dt):
        with self.lock:
            readings = self.robot.scan()
            pose = self._pose()
            self.grid.update_from_scan(pose, readings, world.VISION_RANGE)

            step = 0.0
            if self._prev_pose is not None:
                step = math.hypot(pose[0] - self._prev_pose[0], pose[1] - self._prev_pose[1])
            self._prev_pose = pose
            if step > 0.02 and self.phase != "escape":
                self.history.append((pose[0], pose[1]))

            self._check_battery()
            if self.task is None and self.phase == "idle":
                controls = None
            else:
                controls = self._control(dt, step)
                self.robot.drive(*controls)
            self.robot.step(dt)

            if self.phase in MOW_PHASES:
                radius = None
                if self.phase in ("zone_edge", "corner_clean", "ring", "detour"):
                    # 贴边时刀盘能扫到墙体所在栅格的中心，补偿连续边界
                    # 与栅格中心之间的量化误差。
                    radius = world.MOWER_WIDTH / 2.0 + world.CELL * 1.3
                self.grid.mark_mowed(self.robot.pose[0], self.robot.pose[1], radius)

            self._update_progress(dt)

            if self.robot.collisions > self._last_collisions:
                self._last_collisions = self.robot.collisions
                self._on_bump()

            self.replan_cooldown = max(0.0, self.replan_cooldown - dt)

    def _update_progress(self, dt):
        if self.phase != self.last_phase:
            self.last_phase = self.phase
            self.phase_time = 0.0
            self.stuck_ref = None
        else:
            self.phase_time += dt

        pose = (self.robot.pose[0], self.robot.pose[1])
        if self.stuck_ref is None:
            self.stuck_ref = pose
            self.stuck_time = 0.0
        elif math.hypot(pose[0] - self.stuck_ref[0], pose[1] - self.stuck_ref[1]) > 0.05:
            self.stuck_ref = pose
            self.stuck_time = 0.0
        else:
            self.stuck_time += dt

        if self.stuck_time > 4.0 and self.phase in (
            "zone_edge",
            "zone_stripes",
            "ring",
            "detour",
            "to_zone",
        ):
            self._recover_stuck()
        if self.phase == "to_segment" and (self.phase_time > 10.0 or self.stuck_time > 3.0):
            self.replan_cooldown = 0.0
            if self._replan_route():
                self.stuck_time = 0.0
                self.stuck_ref = None
                self.phase_time = 0.0
            elif self.phase == "to_segment":
                self._skip_segment()
        if self.phase == "corner_clean" and self.phase_time > 4.0:
            self._finish_corner()
        if self.phase == "ring" and self.phase_time > 35.0:
            self._log("warn", "绕障超时，结束本次绕障并重新规划")
            self._finish_ring()
        if (
            self.phase == "zone_edge"
            and self.edge_start_sim is not None
            and self.robot.sim_time - self.edge_start_sim > self._edge_time_limit()
        ):
            self._log("warn", "沿边超时，跳过剩余边缘并继续任务")
            self._finish_edge_phase()
        if self.phase == "map_follow" and self.stuck_time > 2.2:
            # 优先沿历史路径倒车脱困；失败时再执行原来的远离障碍方向脱离。
            if self._begin_escape("map_follow"):
                self.map_path = []
                self.map_path_dist = 0.0
                self.stuck_time = 0.0
                self.stuck_ref = None
                self._log("warn", "巡边受阻，沿历史路径后退脱困")
                return
            nearest = _front_distance(self.robot.vision, 50.0)
            x, y, theta = self._pose()
            if nearest is not None:
                hit_x = x + math.cos(theta + nearest[1]) * nearest[0]
                hit_y = y + math.sin(theta + nearest[1]) * nearest[0]
                self.map_escape_heading = math.atan2(y - hit_y, x - hit_x)
            else:
                self.map_escape_heading = world.wrap_angle(theta + math.pi)
            self.map_escape_time = 0.0
            self.map_escape_dist = 0.0
            self.map_path = []
            self.map_path_dist = 0.0
            self.phase = "map_escape"
            self.last_phase = None
            self._log("warn", "巡边受阻，脱离障碍物后重新寻找外墙")

    def _recover_stuck(self):
        phase = self.phase
        self.stats["stuck_recoveries"] += 1
        self._log("warn", "检测到卡滞（{}），尝试后退沿历史路径脱困".format(phase))
        if phase == "zone_stripes" and self.segment is not None:
            self.failed_segments.add(_segment_key(self.segment))
        if self.zone_escape_count < 3 and self._begin_escape(phase):
            self.zone_escape_count += 1
            self.stuck_time = 0.0
            self.stuck_ref = None
            self.last_phase = None
            return
        if phase == "ring":
            self._finish_ring()
        elif phase == "detour":
            self._finish_detour()
        elif phase == "zone_edge":
            circle = self._edge_circle_from_occupancy()
            if circle is not None and not self._is_edge_ringed(circle):
                if self._begin_edge_ring(circle):
                    return
            self._advance_edge_after_collision(0.55)
        elif phase == "zone_stripes":
            if self.segment is not None:
                self.failed_segments.add(_segment_key(self.segment))
            self.phase = "zone_plan"
        elif phase == "to_zone":
            self._plan_to_zone_corner()
        self.stuck_time = 0.0
        self.stuck_ref = None
        self.last_phase = None

    def _begin_escape(self, resume_phase):
        x, y, _ = self._pose()
        path = []
        length = 0.0
        last = (x, y)
        for point in reversed(self.history):
            distance = math.hypot(point[0] - last[0], point[1] - last[1])
            if distance < 0.04:
                continue
            path.append((point[0], point[1]))
            length += distance
            last = point
            if length >= 1.0:
                break
        if not path or length < 0.20:
            return False
        self.escape_path = path
        self.escape_index = 0
        self.escape_resume = resume_phase
        self.escape_time = 0.0
        self.phase = "escape"
        self.route = []
        self.last_phase = None
        self._log("warn", "沿历史路径后退脱困（{:.1f} m）".format(length))
        return True

    def _escape_step(self, dt):
        self.escape_time += dt
        x, y, theta = self._pose()
        while self.escape_index < len(self.escape_path):
            target = self.escape_path[self.escape_index]
            if math.hypot(target[0] - x, target[1] - y) >= 0.12:
                break
            self.escape_index += 1
        if self.escape_index >= len(self.escape_path) or self.escape_time > 4.0:
            self._resume_after_escape()
            return 0.0, 0.0
        target = self.escape_path[self.escape_index]
        bearing = math.atan2(target[1] - y, target[0] - x)
        # 倒车时车头应朝向目标的反方向，再以负速度沿历史路径退回。
        desired = world.wrap_angle(bearing + math.pi)
        err = world.wrap_angle(desired - theta)
        w = world.clamp(1.8 * err, -1.5, 1.5)
        return -0.22, w

    def _resume_after_escape(self):
        resume = self.escape_resume
        self.escape_path = []
        self.escape_index = 0
        self.escape_resume = None
        self.escape_time = 0.0
        # 倒车后的位姿与旧历史轨迹不连续，避免下一次脱困误用旧路径。
        self.history.clear()
        self.history.append(self._pose()[:2])
        if resume == "map_follow":
            self.map_path = []
            self.map_path_dist = 0.0
            self.phase = "map_seek"
        elif resume == "zone_edge":
            self.phase = "zone_edge"
        elif resume in ("zone_stripes", "zone_plan", "to_segment"):
            self.phase = "zone_plan"
        elif resume == "ring":
            self._finish_ring()
        elif resume == "detour":
            self._finish_detour()
        elif resume == "to_zone":
            self.phase = "to_zone"
            self._plan_to_zone_corner()
        else:
            self.phase = "zone_plan"
        self.last_phase = None

    def _skip_segment(self):
        self._log("warn", "弓字线段起点不可达，跳过并重新规划剩余区域")
        if self.segment is not None:
            self.failed_segments.add(_segment_key(self.segment))
        self.segment_index += 1
        if self.segment_index < len(self.segments):
            self.phase = "to_segment"
            self._goto_segment(self.segment_index)
            self.last_phase = None
        else:
            self.phase = "zone_plan"
            self.last_phase = None

    def _check_battery(self):
        if not self.task:
            return
        if self.phase in CHARGE_EXEMPT:
            return
        if self.charge_reason:
            return
        if self.robot.battery <= LOW_BATTERY:
            self._save_breakpoint()
            self.charge_reason = "low"
            self.stats["charge_count"] += 1
            self.phase = "charge_go"
            self._log(
                "warn",
                "电量低于 {:.0f}%，保存任务并自动返回充电桩".format(LOW_BATTERY),
            )
            self._set_route((world.DOCK["x"], world.DOCK["y"]), "to_dock")

    # ----------------------------------------------------------- 控制分发
    def _control(self, dt, step):
        phase = self.phase
        if phase in ("idle", "done"):
            return 0.0, 0.0
        if phase == "map_seek":
            return self._map_seek()
        if phase == "map_follow":
            return self._map_follow(dt)
        if phase == "map_escape":
            return self._map_escape(dt, step)
        if phase == "to_zone":
            return self._nav_step(NAV_SPEED)
        if phase == "zone_edge":
            return self._edge_step()
        if phase == "escape":
            return self._escape_step(dt)
        if phase == "corner_clean":
            return self._corner_step(dt, step)
        if phase == "zone_plan":
            self._plan_stripes()
            return 0.0, 0.0
        if phase == "to_segment":
            return self._nav_step(NAV_SPEED)
        if phase == "zone_stripes":
            return self._stripe_step()
        if phase == "detour":
            return self._detour_step(step)
        if phase == "ring":
            return self._ring_step(dt)
        if phase in ("charge_go", "to_dock", "charge_return"):
            return self._nav_step(NAV_SPEED)
        if phase == "charging":
            return self._charging_step()
        if phase == "align":
            return self._align_step()
        return 0.0, 0.0

    # ----------------------------------------------------------- 建图
    def _map_seek(self):
        nearest = _front_distance(self.robot.vision, 20.0)
        x, y, theta = self._pose()
        if nearest is not None and nearest[0] < 0.85:
            hit_x = x + math.cos(theta + nearest[1]) * nearest[0]
            hit_y = y + math.sin(theta + nearest[1]) * nearest[0]
            if self._near_known_island(hit_x, hit_y):
                return 0.12, 1.3
            # 只有贴近场地外墙的命中才作为建图起点；
            # 场内障碍物先绕开，避免把孤岛当作场地边界。
            if world.boundary_distance(hit_x, hit_y) > 0.42:
                if self.robot.sim_time - self.map_seek_log_time > 2.0:
                    self.map_seek_log_time = self.robot.sim_time
                    self._log("info", "视觉命中场内障碍物，绕开后继续寻找外墙")
                steer = -1.0 if nearest[1] >= 0.0 else 1.0
                return 0.26, steer
            self.phase = "map_follow"
            self.map_path = [(x, y)]
            self.map_path_dist = 0.0
            self._log("event", "视觉发现边界，开始逆时针巡边")
            return self._map_follow(0.02)
        return MAP_SPEED, 0.0

    def _map_follow(self, dt):
        v, w = self._follow_surface(MAP_SPEED, 0.45)
        x, y, _ = self._pose()
        if not self.map_path:
            self.map_path.append((x, y))
        else:
            last_x, last_y = self.map_path[-1]
            moved = math.hypot(x - last_x, y - last_y)
            if moved > 0.08:
                self.map_path.append((x, y))
                self.map_path_dist += moved
                if len(self.map_path) > 4000:
                    del self.map_path[:1000]

        result = self._check_mapping_loop()
        if result is not None:
            return 0.0, 0.0
        return v, w

    def _check_mapping_loop(self):
        path = self.map_path
        if len(path) < 20:
            return None
        x, y, _ = self._pose()

        if (
            self.map_path_dist > 10.0
            and math.hypot(x - path[0][0], y - path[0][1]) < 0.55
        ):
            area = abs(_polygon_area(path))
            if area >= MIN_OUTER_AREA:
                self._finish_mapping()
                return "complete"
            self._escape_island(path)
            return "island"

        for index in range(len(path) - 20):
            previous = path[index]
            if math.hypot(x - previous[0], y - previous[1]) < 0.30:
                segment = path[index:]
                if _path_length(segment) > 1.8:
                    area = abs(_polygon_area(segment))
                    if (
                        0.2 < area < ISLAND_AREA
                        and self._loop_encloses_obstacle(segment)
                    ):
                        self._escape_island(segment)
                        return "island"
                break
        return None

    def _loop_encloses_obstacle(self, segment):
        """闭环内部不能是已探明自由区；外凸半岛会形成这类假闭环。"""
        if len(segment) < 3:
            return False
        cx = sum(point[0] for point in segment) / len(segment)
        cy = sum(point[1] for point in segment) / len(segment)
        col, row = self.grid.cell_of(cx, cy)
        for dc in (-1, 0, 1):
            for dr in (-1, 0, 1):
                if not self.grid.in_bounds(col + dc, row + dr):
                    continue
                if self.grid.data[self.grid.index(col + dc, row + dr)] == 2:
                    return True
        if self.grid.data[self.grid.index(col, row)] == 1:
            return False

        xs = [point[0] for point in segment]
        ys = [point[1] for point in segment]
        col0 = max(0, int(min(xs) / self.grid.cell))
        col1 = min(self.grid.cols - 1, int(max(xs) / self.grid.cell))
        row0 = max(0, int(min(ys) / self.grid.cell))
        row1 = min(self.grid.rows - 1, int(max(ys) / self.grid.cell))
        total = 0
        free = 0
        for r in range(row0, row1 + 1):
            for c in range(col0, col1 + 1):
                x, y = self.grid.center_of(c, r)
                if not world.point_in_polygon(x, y, segment):
                    continue
                total += 1
                if self.grid.data[self.grid.index(c, r)] == 1:
                    free += 1
        if total < 4:
            return True
        return free / float(total) < 0.25

    def _escape_island(self, path):
        if not path:
            self.phase = "map_seek"
            return
        cx = sum(point[0] for point in path) / len(path)
        cy = sum(point[1] for point in path) / len(path)
        radius = max(math.hypot(point[0] - cx, point[1] - cy) for point in path) + 0.2
        self.map_islands.append({"x": round(cx, 2), "y": round(cy, 2), "r": round(radius, 2)})
        x, y, _ = self._pose()
        self.map_escape_heading = math.atan2(y - cy, x - cx)
        self.map_escape_time = 0.0
        self.map_escape_dist = 0.0
        self.phase = "map_escape"
        self.map_path = []
        self.map_path_dist = 0.0
        self._log("warn", "识别到孤岛，已脱离并重新寻找真正的场地边界")

    def _map_escape(self, dt, step):
        x, y, theta = self._pose()
        self.map_escape_time += dt
        self.map_escape_dist += step
        err = world.wrap_angle(self.map_escape_heading - theta)
        v = 0.45 if abs(err) < 1.0 else 0.0
        w = world.clamp(2.0 * err, -1.5, 1.5)
        front = _front_distance(self.robot.vision, 18.0)
        if front is not None and front[0] < 0.45:
            v = 0.15
            w = 1.2
        if self.map_escape_dist > 2.5 or self.map_escape_time > 5.0:
            self.phase = "map_seek"
            self._log("info", "脱离完成，继续寻找场地边界")
        return v, w

    def _near_known_island(self, x, y):
        for island in self.map_islands:
            if math.hypot(x - island["x"], y - island["y"]) < island["r"] + 0.5:
                return True
        return False

    def _finish_mapping(self):
        self.grid.build_free_area(self._pose()[:2])
        zones = self.grid.partition_zones()
        self._log(
            "event",
            "建图完成：自动划分 {} 个割草区域，返回充电桩".format(len(zones)),
        )
        self.phase = "to_dock"
        self._set_route((world.DOCK["x"], world.DOCK["y"]), "map_dock")

    # ----------------------------------------------------------- 区域导航
    def _plan_to_zone_corner(self):
        zone = self._zone()
        if zone is None:
            self.phase = "idle"
            return
        x, y, _ = self._pose()
        corners = sorted(
            self._zone_edge_points(zone),
            key=lambda point: math.hypot(point[0] - x, point[1] - y),
        )
        for target in corners:
            path, endpoint, near_distance = self._find_route(target)
            if not path:
                continue
            if near_distance > 0.35:
                continue
            if self._zone_id_at(endpoint) != self.zone_id:
                continue
            self.nav_goal = endpoint
            self.route_kind = "to_zone"
            self.route = path
            self.route_index = 1 if len(path) > 1 else 0
            return
        self._skip_unreachable_zone()

    def _zone_edge_points(self, zone):
        key = (zone["id"], self.grid.data_version)
        cached = self._edge_route_cache.get(key)
        if cached is not None:
            return list(cached)
        width = max(1e-6, (zone["x1"] - zone["x0"]) * (zone["y1"] - zone["y0"]))
        fill = zone["free"] * self.grid.cell * self.grid.cell / width
        if fill < 0.92:
            points = planner.edge_route_grid(self.grid, zone, self._pose()[:2])
        else:
            points = planner.edge_route(zone, self._pose()[:2])
            if len(points) >= 3 and _polygon_area(points) < 0.0:
                points = list(reversed(points))
        self._edge_route_cache[key] = list(points)
        return points

    def _enter_zone(self):
        zone = self._zone()
        if zone is None:
            self.phase = "zone_plan"
            return
        self.edge_points = []
        self.edge_index = 0
        self.edge_resume_target = -1
        self.edge_resume_attempts = 0
        self.edge_ringed_circles = []
        self.zone_plan_passes = 0
        self.zone_bump_count = 0
        self.zone_escape_count = 0
        self.escape_path = []
        self.escape_index = 0
        self.escape_resume = None
        self.escape_time = 0.0
        self.patch_cleanup_done = False
        self.coverage_retries = 0
        self.failed_segments = set()
        self.lane_axis = "h"
        self.rings_done = False
        self.edge_done = False
        if self.mowing_mode == "B":
            self.zone_stage = "stripes"
            self.phase = "zone_plan"
            self._log("event", "进入区域 {}：B 模式，先规划弓字覆盖".format(self.zone_id))
        else:
            self.edge_points = self._zone_edge_points(zone)
            self._prepare_edge_corners()
            self.zone_stage = "edge"
            self.edge_start_sim = self.robot.sim_time
            self.phase = "zone_edge"
            self._log("event", "进入区域 {}：先沿边缘割草一周".format(self.zone_id))

    def _edge_time_limit(self):
        length = _path_length(self.edge_points) if self.edge_points else 0.0
        expected = length / max(0.20, EDGE_SPEED)
        return world.clamp(expected * 2.2 + 30.0, 120.0, 600.0)

    def _edge_step(self):
        if not self.edge_points:
            self.phase = "zone_plan"
            return 0.0, 0.0
        x, y, theta = self._pose()
        while self.edge_index < len(self.edge_points):
            point = self.edge_points[self.edge_index]
            reached = math.hypot(point[0] - x, point[1] - y) < 0.18
            passed = False
            if not reached and self.edge_index > 0:
                ax, ay = self.edge_points[self.edge_index - 1]
                vx, vy = point[0] - ax, point[1] - ay
                length_sq = vx * vx + vy * vy
                if length_sq > 1e-9:
                    length = math.sqrt(length_sq)
                    along = ((x - point[0]) * vx + (y - point[1]) * vy) / length
                    cross = abs((x - point[0]) * vy - (y - point[1]) * vx) / length
                    passed = (
                        along > 0.02
                        and cross < 0.12
                        and math.hypot(point[0] - x, point[1] - y) < 0.65
                    )
            if not reached and not passed:
                break
            self.edge_index += 1
            self.edge_resume_target = -1
            self.edge_resume_attempts = 0
        if self.edge_index >= len(self.edge_points):
            self._finish_edge_phase()
            return 0.0, 0.0
        if self._maybe_start_corner():
            return 0.0, 0.0

        # 纯跟踪预瞄：在路径投影点前方约 0.30 m 取连续点。
        # 规则区域边缘只有少数长边，不能直接跳到下一个角点。
        target = self._edge_lookahead_target(x, y)
        if target is None:
            self._finish_edge_phase()
            return 0.0, 0.0
        circle = self._edge_blocking_circle(target)
        if circle is not None:
            if not self._begin_edge_ring(circle):
                self._advance_edge_past_circle(circle)
            return 0.0, 0.0
        if self._front_hits_zone():
            circle = self._nearest_circle(x, y, 1.20)
            if circle is not None:
                if not self._begin_edge_ring(circle):
                    self._advance_edge_past_circle(circle)
                return 0.0, 0.0
            # 未聚类的前方命中可能只是贴墙噪声；保持沿边控制，
            # 由视觉壁障保护处理，不能因此跳过整圈边缘点。
        err = world.wrap_angle(math.atan2(target[1] - y, target[0] - x) - theta)
        v = EDGE_SPEED * max(0.25, 1.0 - abs(err) / 1.5) if abs(err) < 1.2 else 0.0
        if v == 0.0 and abs(err) < 2.5 and self.robot.vision:
            # 大角度转向时保留极低爬行速度，形成连续弧线而不是原地停顿。
            if min(distance for _, distance in self.robot.vision) > 0.45:
                v = 0.06
        w = world.clamp(2.2 * err, -1.4, 1.4)
        return self._wall_guard(v, w)


    def _prepare_edge_corners(self):
        self.edge_corners = []
        self.edge_corner_done = set()
        points = self.edge_points
        if len(points) < 5:
            return
        cumulative = [0.0]
        for index in range(1, len(points)):
            cumulative.append(
                cumulative[-1]
                + math.hypot(
                    points[index][0] - points[index - 1][0],
                    points[index][1] - points[index - 1][1],
                )
            )
        total = cumulative[-1]
        if total < 0.8:
            return

        def point_at(distance):
            distance = world.clamp(distance, 0.0, total)
            low, high = 0, len(cumulative) - 1
            while low < high:
                mid = (low + high) // 2
                if cumulative[mid] < distance:
                    low = mid + 1
                else:
                    high = mid
            index = max(1, low)
            previous = index - 1
            segment = cumulative[index] - cumulative[previous]
            ratio = (distance - cumulative[previous]) / segment if segment > 0 else 0.0
            ax, ay = points[previous]
            bx, by = points[index]
            return (ax + (bx - ax) * ratio, ay + (by - ay) * ratio)

        last = None
        for index in range(0, len(points) - 1):
            along = cumulative[index]
            before_distance = along - 0.30 if along >= 0.30 else total + along - 0.30
            after_distance = along + 0.30 if along + 0.30 <= total else (along + 0.30) - total
            before = point_at(before_distance)
            vertex = points[index]
            after = point_at(after_distance)
            incoming = math.atan2(vertex[1] - before[1], vertex[0] - before[0])
            outgoing = math.atan2(after[1] - vertex[1], after[0] - vertex[0])
            turn = world.wrap_angle(outgoing - incoming)
            # 逆时针沿边时，左转是外凸直角（预瞄自然转过即可），
            # 右转才是内凹直角，需要在尖角处前进-后退补扫。
            if -2.2 < turn < -1.05:
                if last is not None and math.hypot(vertex[0] - last[1], vertex[1] - last[2]) < 0.45:
                    continue
                self.edge_corners.append((index, vertex[0], vertex[1], incoming, turn))
                last = (index, vertex[0], vertex[1])

    def _maybe_start_corner(self):
        if not self.edge_corners:
            return False
        x, y, _ = self._pose()
        for index, cx, cy, incoming, _turn in self.edge_corners:
            if index in self.edge_corner_done:
                continue
            if index < self.edge_index - 2:
                continue
            if index > self.edge_index + 12:
                continue
            if math.hypot(cx - x, cy - y) >= 0.55:
                continue
            self.corner_index = index
            # 内凹角的两条边都内缩，原本停在路径顶点仍会漏掉尖角。
            # 沿入射方向多前进一小段，让割草宽度覆盖到真正的直角尖端。
            overshoot = max(0.0, INNER_CORNER_ADVANCE)
            self.corner_vertex = (
                cx + math.cos(incoming) * overshoot,
                cy + math.sin(incoming) * overshoot,
            )
            self.corner_kind = "inner"
            self.corner_in_heading = incoming
            self.corner_stage = "forward"
            self.corner_reverse_remaining = overshoot + 0.20
            self.corner_time = 0.0
            self.phase = "corner_clean"
            self.last_phase = None
            self._log("info", "进入内直角，前进 {:.2f} m 清理尖角再后退转向".format(overshoot))
            return True
        return False

    def _corner_step(self, dt, step):
        self.corner_time += dt
        x, y, theta = self._pose()
        if self.corner_stage == "forward":
            target = self.corner_vertex
            distance = math.hypot(target[0] - x, target[1] - y)
            if distance < 0.08:
                self.corner_stage = "reverse"
                self.corner_time = 0.0
                return 0.0, 0.0
            err = world.wrap_angle(math.atan2(target[1] - y, target[0] - x) - theta)
            v = 0.16 * max(0.25, 1.0 - abs(err) / 1.5) if abs(err) < 1.2 else 0.0
            w = world.clamp(2.0 * err, -1.3, 1.3)
            return self._wall_guard(v, w)
        err = world.wrap_angle(self.corner_in_heading - theta)
        w = world.clamp(1.4 * err, -1.2, 1.2)
        if step > 0.0:
            self.corner_reverse_remaining -= step
        if self.corner_reverse_remaining <= 0.0 or self.corner_time > 1.8:
            self._finish_corner()
            return 0.0, 0.0
        return -0.15, w

    def _finish_corner(self):
        if self.corner_index is not None:
            self.edge_corner_done.add(self.corner_index)
            # 从尖角的下一个边缘点继续，不能被预瞄产生的远期索引带过整段路径。
            self.edge_index = self.corner_index + 1
        self.corner_index = None
        self.corner_vertex = None
        self.corner_kind = None
        self.corner_stage = None
        self.corner_time = 0.0
        if self.phase == "corner_clean":
            self.phase = "zone_edge"
        self.last_phase = None

    def _edge_lookahead_target(self, x, y, distance=0.30):
        points = self.edge_points
        if not points:
            return None
        if len(points) == 1:
            return points[0]
        if self.edge_index <= 0:
            self.edge_index = 1
        cursor = min(self.edge_index, len(points) - 1)
        ax, ay = points[cursor - 1]
        bx, by = points[cursor]
        vx, vy = bx - ax, by - ay
        length = math.hypot(vx, vy)
        if length <= 1e-9:
            return points[cursor]
        t = world.clamp(((x - ax) * vx + (y - ay) * vy) / (length * length), 0.0, 1.0)
        remaining = (1.0 - t) * length
        if remaining >= distance:
            ratio = t + distance / length
            return (ax + vx * ratio, ay + vy * ratio)
        need = distance - remaining
        while cursor + 1 < len(points):
            ax, ay = points[cursor]
            bx, by = points[cursor + 1]
            segment = math.hypot(bx - ax, by - ay)
            if segment >= need:
                ratio = need / segment if segment > 0 else 0.0
                return (ax + (bx - ax) * ratio, ay + (by - ay) * ratio)
            need -= segment
            cursor += 1
        return points[-1]


    def _advance_edge_after_collision(self, distance=0.80):
        x, y, _ = self._pose()
        index = self.edge_index + 1
        while index < len(self.edge_points):
            point = self.edge_points[index]
            if math.hypot(point[0] - x, point[1] - y) >= distance:
                break
            index += 1
        if index >= len(self.edge_points):
            self._finish_edge_phase()
            return
        self.edge_index = index
        self.phase = "zone_edge"
        self.last_phase = None
        self.stats["edge_skips"] += 1
        self._log("info", "沿边遇到未聚类障碍，跳过受阻边缘段继续沿边")

    def _advance_edge_past_circle(self, circle):
        circles = self.grid.obstacle_circles(self._zone())
        index = self.edge_index + 1
        while index < len(self.edge_points):
            point = self.edge_points[index]
            outside = (
                math.hypot(point[0] - circle["x"], point[1] - circle["y"])
                > circle["r"] + 0.28
            )
            if outside and not self._point_inside_obstacle(point, circles):
                break
            index += 1
        if index >= len(self.edge_points):
            self._finish_edge_phase()
            return
        self.edge_index = index
        self.phase = "zone_edge"
        self.last_phase = None
        self.stats["edge_skips"] += 1

    def _is_edge_ringed(self, circle):
        for known in self.edge_ringed_circles:
            if math.hypot(circle["x"] - known["x"], circle["y"] - known["y"]) < 0.45:
                return True
        return False

    def _edge_circle_from_occupancy(self):
        x, y, _ = self._pose()
        near = self.grid.nearest_occupied(x, y, 0.90)
        if near is None:
            return None
        _, ox, oy = near
        if world.boundary_distance(ox, oy) < 0.42:
            return None
        return {"x": ox, "y": oy, "r": 0.24, "kind": "small", "extent": 0.5}

    def _begin_edge_ring(self, circle):
        if self._is_edge_ringed(circle):
            return False
        x, y, _ = self._pose()
        points = planner.ring_route(circle, (x, y), grid=self.grid, zone=self._zone())
        self.edge_ringed_circles.append(dict(circle))
        if not points:
            return False
        self.detour_source = "edge"
        self.ring_circle = circle
        self.ring_points = points
        self.ring_index = 0
        self.ring_after = "detour"
        self.ring_started = False
        self.ring_clipped = len(points) < planner.RING_STEPS
        self.ring_lost_time = 0.0
        self.ring_scan_time = 0.0
        self.phase = "ring"
        self.route = []
        self.stats["rings"] += 1
        self._log(
            "warn",
            "沿边遇到障碍物，逆时针绕障（半径 {:.2f} m）".format(circle["r"]),
        )
        return True

    def _edge_blocking_circle(self, target):
        x, y, _ = self._pose()
        circles = self.grid.obstacle_circles(self._zone()) + self.edge_ringed_circles
        for circle in circles:
            distance_to_robot = math.hypot(circle["x"] - x, circle["y"] - y)
            target_distance = math.hypot(target[0] - x, target[1] - y)
            if distance_to_robot <= circle["r"] + 0.36:
                if target_distance <= 0.36:
                    blocked = (
                        math.hypot(target[0] - circle["x"], target[1] - circle["y"])
                        < circle["r"] + 0.20
                    )
                else:
                    ux = (target[0] - x) / target_distance
                    uy = (target[1] - y) / target_distance
                    sx = x + ux * 0.28
                    sy = y + uy * 0.28
                    blocked = (
                        world._segment_distance(
                            circle["x"], circle["y"], sx, sy, target[0], target[1]
                        )
                        < circle["r"] + 0.20
                    )
            else:
                blocked = (
                    world._segment_distance(
                        circle["x"], circle["y"], x, y, target[0], target[1]
                    )
                    < circle["r"] + 0.20
                )
            if blocked:
                return circle
        return None

    def _skip_blocked_edge_points(self):
        """一次跳到已被绕行障碍物之后的边缘点，避免逐点空转。"""
        circles = self.grid.obstacle_circles(self._zone()) + self.edge_ringed_circles
        start = self.edge_index
        index = start
        while index < len(self.edge_points):
            state = self._edge_path_obstacle(self.edge_points[index], circles)
            if state == "detour":
                self._begin_detour("edge")
                return
            if state is None and index > start:
                break
            # 当前点本身可用、但预瞄点被绕过的障碍挡住时，至少前进一点，
            # 否则下一帧仍会选中同一个预瞄点并原地空转。
            index += 1
        if index >= len(self.edge_points):
            self._finish_edge_phase()
            return
        self.edge_index = index
        self.stats["edge_skips"] += 1
        if self.robot.sim_time - self.edge_guard_log_time > 2.0:
            self.edge_guard_log_time = self.robot.sim_time
            self._log("info", "边缘路径遇到已绕行障碍物，跳至障碍物后的边缘点")

    def _edge_path_obstacle(self, target, circles=None):
        """沿边时主动检查已探明障碍物是否压在当前边缘路径上。

        返回 detour（未绕过，主动绕障）/ skip（已绕过，跳过阻挡点）/ None。
        """
        x, y, _ = self._pose()
        if circles is None:
            circles = self.grid.obstacle_circles(self._zone())
        for circle in circles:
            distance_to_robot = math.hypot(circle["x"] - x, circle["y"] - y)
            target_distance = math.hypot(target[0] - x, target[1] - y)
            if distance_to_robot <= circle["r"] + 0.40:
                # 起点就在障碍物旁时，忽略从机器人当前点出发的前 0.30 m，
                # 检查剩余线段是否穿过障碍；仅看目标点会误判“目标在外侧就
                # 可直达”，导致机器人原地转向却始终撞向障碍。
                if target_distance <= 0.40:
                    blocked = (
                        math.hypot(target[0] - circle["x"], target[1] - circle["y"])
                        < circle["r"] + 0.20
                    )
                else:
                    ux = (target[0] - x) / target_distance
                    uy = (target[1] - y) / target_distance
                    sx = x + ux * 0.30
                    sy = y + uy * 0.30
                    blocked = (
                        world._segment_distance(
                            circle["x"], circle["y"], sx, sy, target[0], target[1]
                        )
                        < circle["r"] + 0.20
                    )
            else:
                blocked = (
                    world._segment_distance(
                        circle["x"], circle["y"], x, y, target[0], target[1]
                    )
                    < circle["r"] + 0.20
                )
            if blocked:
                return "skip" if self._is_bypassed(circle) else "detour"
        return None

    def _wall_guard(self, v, w):
        readings = self.robot.vision
        if not readings:
            return v, w
        relative, distance = min(readings, key=lambda item: item[1])
        # 弓字巡迹速度较高，需要比贴边模式留更大的制动距离。
        if self.phase in ("zone_stripes", "to_segment"):
            limit = 0.25
        elif self.phase in ("ring", "detour"):
            limit = 0.24
        else:
            limit = 0.20
        if distance < limit:
            steer = -1.4 if relative >= 0.0 else 1.4
            return 0.0, world.clamp(w + steer, -1.6, 1.6)
        return v, w

    # ----------------------------------------------------------- 弓字覆盖
    def _plan_stripes(self):
        if self.zone_stage == "patch" and self.patch_cleanup_done:
            self._complete_zone()
            return
        zone = self._zone()
        if not self.task or zone is None:
            if self.task and self.task["type"] == "mow":
                self._complete_zone()
            else:
                self.phase = "idle"
            return
        for circle in self.grid.obstacle_circles(zone):
            # 大型障碍常用包围圆近似多瓣形状，直接盖章会堵死狭窄回廊；
            # 其真实占据栅格+膨胀已足够保证割草避让。
            if circle.get("kind") != "large":
                self.grid.stamp_obstacle(circle["x"], circle["y"], circle["r"])
        raw_segments = planner.lane_segments(self.grid, zone, axis=self.lane_axis)
        filtered = [
            segment for segment in raw_segments if _segment_key(segment) not in self.failed_segments
        ]
        pose = self._pose()
        self.segments = planner.order_segments(filtered, pose[:2], pose[2])
        self.segment_index = 0
        self.zone_plan_passes += 1
        if self.zone_plan_passes > MAX_ZONE_PLAN_PASSES:
            self._finish_zone_coverage()
            return
        if not self.segments and self.lane_axis == "h":
            coverage = self.grid.zone_coverage(self.zone_id)
            if coverage < 0.93:
                self.lane_axis = "v"
                self.zone_plan_passes = 0
                vertical = planner.lane_segments(self.grid, zone, axis="v")
                filtered = [
                    segment
                    for segment in vertical
                    if _segment_key(segment) not in self.failed_segments
                ]
                self.segments = planner.order_segments(filtered, pose[:2], pose[2])
                if self.segments:
                    self._log("info", "区域 {} 进行纵向补割".format(self.zone_id))
        if not self.segments:
            self._finish_zone_coverage()
            return
        self.stats["segments_planned"] += len(self.segments)
        self._log(
            "info",
            "区域 {} 规划弓字线段 {} 条".format(self.zone_id, len(self.segments)),
        )
        self._goto_segment(0)

    def _start_final_edge(self):
        zone = self._zone()
        if zone is None:
            self._complete_zone()
            return
        self.edge_points = self._zone_edge_points(zone)
        self.edge_index = 0
        self.edge_ringed_circles = []
        self._prepare_edge_corners()
        self.zone_stage = "edge_final"
        self.edge_done = False
        self.edge_start_sim = self.robot.sim_time
        self.phase = "zone_edge"
        self._log("info", "区域 {} 弓字完成，开始沿边割草".format(self.zone_id))

    def _finish_zone_coverage(self):
        """B 模式先补割再沿边；A 模式先沿边再补割。"""
        if self.mowing_mode == "B":
            if not self.edge_done:
                self._start_final_edge()
            else:
                self._complete_zone()
            return
        if self.lane_axis == "v" and self._plan_rings():
            return
        self._complete_zone()

    def _segment_entry(self, a, b):
        distance = math.hypot(b[0] - a[0], b[1] - a[1])
        count = max(1, int(math.ceil(distance / 0.15)))
        for index in range(count + 1):
            t = index / float(count)
            x = a[0] + (b[0] - a[0]) * t
            y = a[1] + (b[1] - a[1]) * t
            if self.grid.is_blocked_world(x, y):
                continue
            if self.grid.is_occupied_world(x, y):
                continue
            zone = self.grid.zone_at(x, y)
            if zone is None or zone["id"] != self.zone_id:
                continue
            return (x, y)
        return None

    def _goto_segment(self, index):
        if index >= len(self.segments):
            self.phase = "zone_plan"
            return
        self.segment_index = index
        a, b = self.segments[index]
        entry = self._segment_entry(a, b)
        if entry is None:
            self.failed_segments.add(_segment_key((a, b)))
            self.phase = "zone_plan"
            return
        self.segment = (entry, b)
        self.segment_end = b
        x, y, _ = self._pose()
        if math.hypot(entry[0] - x, entry[1] - y) < 0.25:
            self.route = []
            self.route_kind = None
            self.phase = "zone_stripes"
        else:
            if self._set_route(entry, "to_segment"):
                self.phase = "to_segment"

    def _stripe_step(self):
        if not self.segment:
            self.phase = "zone_plan"
            return 0.0, 0.0
        x, y, theta = self._pose()
        target = self.segment_end
        distance = math.hypot(target[0] - x, target[1] - y)
        if distance < 0.12:
            self.segment_index += 1
            if self.segment_index < len(self.segments):
                self._goto_segment(self.segment_index)
            elif self.zone_stage == "patch":
                self._complete_zone()
            else:
                self.phase = "zone_plan"
            return 0.0, 0.0

        if self._front_hits_zone():
            self._begin_detour("stripes")
            return 0.0, 0.0

        err = world.wrap_angle(math.atan2(target[1] - y, target[0] - x) - theta)
        v = MOW_SPEED * max(0.18, 1.0 - abs(err) / 1.5) if abs(err) < 1.2 else 0.0
        w = world.clamp(2.4 * err, -1.5, 1.5)
        return self._wall_guard(v, w)

    def _coverage_target(self):
        zone = self._zone()
        if zone is None:
            return 0.85
        total = max(1, self.grid.zone_totals.get(self.zone_id, 0))
        mowable = 0
        for row in range(self.grid.rows):
            for col in range(self.grid.cols):
                if not self.grid.cell_in_zone(col, row, self.zone_id):
                    continue
                if (
                    self.grid.cell_mowable(col, row)
                    and not self.grid.cell_blocked_mow(col, row)
                ):
                    mowable += 1
        return max(0.80, 0.95 * mowable / float(total))

    def _plan_patch_cleanup(self):
        if self.patch_cleanup_done:
            return False
        self.patch_cleanup_done = True
        zone = self._zone()
        if zone is None:
            return False
        components = self.grid.unmowed_components(
            self.zone_id, min_cells=PATCH_MIN_CELLS
        )
        patches = [
            cells
            for cells in components
            if PATCH_MIN_CELLS <= len(cells) <= PATCH_MAX_CELLS
        ]
        if not patches:
            return False
        patch_cells = set()
        for cells in patches:
            patch_cells.update(cells)
        raw_segments = planner.lane_segments(self.grid, zone, axis=self.lane_axis)
        segments = []
        for segment in raw_segments:
            if _segment_length(segment) < 0.18:
                continue
            a, b = segment
            touches = False
            for ratio in (0.0, 0.25, 0.5, 0.75, 1.0):
                x = a[0] + (b[0] - a[0]) * ratio
                y = a[1] + (b[1] - a[1]) * ratio
                cell = self.grid.cell_of(x, y)
                if cell in patch_cells:
                    touches = True
                    break
            if touches:
                segments.append(segment)
        if not segments:
            # 窄小补丁可能无法生成正常行距的弓字，直接沿补丁长轴扫一遍。
            for cells in patches:
                xs = [self.grid.center_of(c, r)[0] for c, r in cells]
                ys = [self.grid.center_of(c, r)[1] for c, r in cells]
                if max(xs) - min(xs) >= max(ys) - min(ys):
                    y = (min(ys) + max(ys)) / 2.0
                    segment = ((min(xs), y), (max(xs), y))
                else:
                    x = (min(xs) + max(xs)) / 2.0
                    segment = ((x, min(ys)), (x, max(ys)))
                segments.append(segment)
        if not segments:
            return False
        pose = self._pose()
        self.segments = planner.order_segments(segments, pose[:2], pose[2])
        self.segment_index = 0
        self.segment = None
        self.zone_stage = "patch"
        self.zone_plan_passes = 0
        self.failed_segments = set()
        area = len(patch_cells) * self.grid.cell * self.grid.cell
        self._log(
            "event",
            "区域 {} 发现 {} 块小块漏扫（{:.2f} m²），开始补扫".format(
                self.zone_id, len(patches), area
            ),
        )
        self._goto_segment(0)
        return True

    def _complete_zone(self):
        if self._plan_patch_cleanup():
            return
        coverage = self.grid.zone_coverage(self.zone_id)
        if self.coverage_retries < 1 and coverage < self._coverage_target():
            remaining = self.grid.unmowed_components(self.zone_id, PATCH_MIN_CELLS)
            if remaining:
                self.coverage_retries += 1
                self._log(
                    "warn",
                    "区域 {} 覆盖率 {:.0f}% 仍低于可达目标，继续补割剩余连通块".format(
                        self.zone_id, coverage * 100.0
                    ),
                )
                self.patch_cleanup_done = False
                self.failed_segments = set()
                self.zone_plan_passes = 0
                self.zone_stage = "stripes"
                self.lane_axis = "v" if self.lane_axis == "h" else "h"
                self.segments = []
                self.segment = None
                self.phase = "zone_plan"
                return
        self._log(
            "event",
            "区域 {} 割草完成，覆盖率 {:.0f}%".format(self.zone_id, coverage * 100.0),
        )
        self.segments = []
        self.segment = None
        self.zone_index += 1
        if self.zone_index < len(self.zone_order):
            self.zone_id = self.zone_order[self.zone_index]
            self.zone_stage = None
            self.zone_plan_passes = 0
            self.phase = "to_zone"
            self._plan_to_zone_corner()
            self._log("info", "前往下一个区域 {}".format(self.zone_id))
        else:
            self.zone_id = None
            self.phase = "to_dock"
            self._log("event", "全部区域割草完成，返回充电桩")
            self._set_route((world.DOCK["x"], world.DOCK["y"]), "mow_dock")

    # ----------------------------------------------------------- 绕障
    def _front_hits_zone(self, max_angle=18.0, max_distance=0.55):
        x, y, theta = self._pose()
        best = None
        for relative, distance in self.robot.vision:
            if abs(math.degrees(relative)) > max_angle:
                continue
            if distance >= max_distance:
                continue
            if best is None or distance < best[1]:
                best = (relative, distance)
        if best is None:
            return False
        hit_x = x + math.cos(theta + best[0]) * best[1]
        hit_y = y + math.sin(theta + best[0]) * best[1]
        zone = self._zone()
        if zone is None:
            return False
        margin = 0.10
        inside = (
            zone["x0"] + margin <= hit_x <= zone["x1"] - margin
            and zone["y0"] + margin <= hit_y <= zone["y1"] - margin
        )
        # 只有视觉确认、且远离场地外墙的占据栅格才视为场内障碍物。
        # 外凸半岛的墙体命中不能触发绕障，否则会反复跳过整段沿边路径。
        return (
            inside
            and world.boundary_distance(hit_x, hit_y) > 0.42
            and self.grid.occupied_near(hit_x, hit_y, 0.12)
        )

    def _begin_detour(self, source):
        if self.phase in ("detour", "ring"):
            return
        x, y, _ = self._pose()
        circle = self._nearest_circle(x, y, 1.0)

        if circle is not None and self._is_bypassed(circle):
            self._log("info", "该障碍物已绕行过，跳过二次绕障并重新规划")
            if source in ("edge", "zone_edge"):
                # 已绕过的障碍仍挡在预瞄方向时，一次推进到该障碍后的
                # 边缘点；不要递归调用 skip，否则会反复回到同一点。
                self._advance_edge_past_circle(circle)
            else:
                if self.segment is not None:
                    self.failed_segments.add(_segment_key(self.segment))
                self.phase = "zone_plan"
            self.last_phase = None
            return

        if (
            self.mowing_mode == "B"
            and source == "stripes"
            and circle is not None
            and circle.get("kind") == "large"
        ):
            self.grid.stamp_obstacle(circle["x"], circle["y"], circle["r"])
            self.bypassed_circles.append(dict(circle))
            if self.segment is not None:
                self.failed_segments.add(_segment_key(self.segment))
            self.phase = "zone_plan"
            self.last_phase = None
            self.stats["large_replans"] += 1
            self._log("info", "遇到大型障碍物：掉头并重新规划弓字")
            return

        if circle is not None:
            zone = self._zone()
            points = planner.ring_route(circle, (x, y), grid=self.grid, zone=zone)
            if not points:
                self.bypassed_circles.append(dict(circle))
                self._log("warn", "障碍物周围被其他障碍遮挡，跳过绕障并重新规划")
                if source == "edge":
                    self._resume_edge_route()
                else:
                    if self.segment is not None:
                        self.failed_segments.add(_segment_key(self.segment))
                    self.phase = "zone_plan"
                self.last_phase = None
                return
            self.detour_source = source
            self.ring_circle = circle
            self.ring_points = points
            self.ring_index = 0
            self.ring_after = "detour"
            self.ring_started = False
            self.ring_clipped = len(points) < planner.RING_STEPS
            self.ring_lost_time = 0.0
            self.ring_scan_time = 0.0
            self.phase = "ring"
            self.route = []
            self.stats["rings"] += 1
            self._log(
                "warn",
                "巡迹遇到障碍物，绕障碍物一周（半径 {:.2f} m）".format(circle["r"]),
            )
            if self.ring_clipped:
                self._log("info", "绕障路径触及区域边界，只沿本区域内的弧段绕行")
            return

        # 障碍物靠近场地边界时可能无法聚类成圆，先沿可见表面绕行；
        # 若再次碰撞，_on_bump 会跳过当前弓字段并限制同区碰撞次数。
        self.detour_start = (x, y)
        self.detour_distance = 0.0
        self.detour_source = source
        self.phase = "detour"
        self.route = []
        self._log("warn", "巡迹遇到障碍物，沿表面绕障")

    def _nearest_circle(self, x, y, limit):
        zone = self._zone()
        circles = self.grid.obstacle_circles(zone)
        best = None
        for circle in circles:
            distance = math.hypot(circle["x"] - x, circle["y"] - y)
            if distance <= circle["r"] + limit and (best is None or distance < best[0]):
                best = (distance, circle)
        return best[1] if best else None

    def _is_bypassed(self, circle):
        for known in self.bypassed_circles:
            if math.hypot(circle["x"] - known["x"], circle["y"] - known["y"]) < max(
                0.55, known["r"] + 0.4
            ):
                return True
        return False

    def _ring_step(self, dt=0.02):
        if not self.ring_points:
            self._finish_ring()
            return 0.0, 0.0
        x, y, theta = self._pose()
        circle = self.ring_circle
        if circle is not None and circle.get("kind") == "small":
            if self.ring_scan_time > 0.0:
                err = world.wrap_angle(self.ring_scan_heading - theta)
                self.ring_scan_time -= dt
                if abs(err) < 0.18 or self.ring_scan_time <= 0.0:
                    self.ring_scan_time = 0.0
                    self.ring_lost_time = 0.0
                else:
                    return 0.0, world.clamp(2.2 * err, -1.5, 1.5)
            elif self._obstacle_visible(circle):
                self.ring_lost_time = 0.0
            else:
                self.ring_lost_time += dt
                if self.ring_lost_time > 0.5:
                    self.ring_lost_time = 0.0
                    self.ring_scan_time = 0.8
                    self.ring_scan_heading = math.atan2(circle["y"] - y, circle["x"] - x)
                    self._log("info", "小障碍物进入视觉盲区，花瓣式旋转探明后继续绕障")
                    err = world.wrap_angle(self.ring_scan_heading - theta)
                    return 0.0, world.clamp(2.2 * err, -1.5, 1.5)
        if not self.ring_started:
            target = self.ring_points[min(1, len(self.ring_points) - 1)]
            err = world.wrap_angle(math.atan2(target[1] - y, target[0] - x) - theta)
            if abs(err) > 0.30:
                return 0.0, world.clamp(2.0 * err, -1.5, 1.5)
            self.ring_started = True
        target = self.ring_points[self.ring_index]
        if math.hypot(target[0] - x, target[1] - y) < 0.15:
            self.ring_index += 1
            if self.ring_index >= len(self.ring_points):
                self._finish_ring()
                return 0.0, 0.0
            target = self.ring_points[self.ring_index]
        err = world.wrap_angle(math.atan2(target[1] - y, target[0] - x) - theta)
        v = 0.34 * max(0.2, 1.0 - abs(err) / 1.5) if abs(err) < 1.3 else 0.0
        w = world.clamp(2.2 * err, -1.4, 1.4)
        return self._wall_guard(v, w)

    def _obstacle_visible(self, circle):
        range_limit = world.VISION_RANGE - 0.02
        for relative, distance in self.robot.vision:
            if distance >= range_limit:
                continue
            x, y, theta = self._pose()
            hit_x = x + math.cos(theta + relative) * distance
            hit_y = y + math.sin(theta + relative) * distance
            if math.hypot(hit_x - circle["x"], hit_y - circle["y"]) < circle["r"] + 0.20:
                return True
        return False

    def _finish_ring(self):
        self.ring_points = []
        self.ring_index = 0
        self.ring_started = False
        after = self.ring_after
        self.ring_after = None
        if self.ring_circle is not None:
            self.bypassed_circles.append(dict(self.ring_circle))
            self.ring_circle = None
        clipped = self.ring_clipped
        self.ring_clipped = False
        self.ring_scan_time = 0.0
        self.ring_lost_time = 0.0
        if after == "detour":
            if clipped:
                self._log("info", "绕障在区域边界结束，沿区域边界继续任务")
            self._finish_detour()
        elif after == "rings":
            self._next_ring()
        else:
            self.phase = "zone_plan"

    def _detour_step(self, step):
        self.detour_distance += step
        v, w = self._follow_surface(0.32, 0.45, front_limit=0.55)
        x, y, _ = self._pose()
        zone = self._zone()
        if zone is not None and not (
            zone["x0"] + 0.05 <= x <= zone["x1"] - 0.05
            and zone["y0"] + 0.05 <= y <= zone["y1"] - 0.05
        ):
            if self.robot.sim_time - self.edge_guard_log_time > 2.0:
                self.edge_guard_log_time = self.robot.sim_time
                self._log("info", "沿边绕障触及区域边界，返回本区域边缘")
            self._finish_detour()
            return 0.0, 0.0
        if self.detour_start is not None:
            returned = math.hypot(x - self.detour_start[0], y - self.detour_start[1]) < 0.20
            if self.detour_distance > 2.2 and returned:
                self._finish_detour()
                return 0.0, 0.0
        if self.detour_distance > 9.0:
            self._finish_detour()
            return 0.0, 0.0
        return self._wall_guard(v, w)

    def _finish_detour(self):
        self._log("event", "绕障完成并回到历史路径")
        source = self.detour_source
        self.detour_source = None
        self.detour_start = None
        self.detour_distance = 0.0
        if source == "edge" and self.edge_points:
            self._resume_edge_route()
        else:
            self.phase = "zone_plan"

    def _finish_edge_phase(self):
        self.edge_done = True
        self.edge_resume_target = -1
        self.edge_resume_attempts = 0
        self.edge_ringed_circles = []
        self.edge_corners = []
        self.edge_corner_done = set()
        self.corner_index = None
        self.corner_vertex = None
        self.corner_kind = None
        self.corner_in_heading = 0.0
        self.corner_stage = None
        self.corner_reverse_remaining = 0.0
        self.corner_time = 0.0
        self.edge_start_sim = None
        self._log("info", "区域 {} 边缘已完成".format(self.zone_id))
        if self.zone_stage == "edge_final":
            self._complete_zone()
        else:
            self.zone_stage = "stripes"
            self.phase = "zone_plan"

    def _resume_edge_route(self):
        if not self.edge_points:
            self.zone_stage = "stripes"
            self.phase = "zone_plan"
            return
        x, y, _ = self._pose()
        circles = self.grid.obstacle_circles(self._zone()) + self.edge_ringed_circles
        candidates = []
        for index in range(self.edge_index, len(self.edge_points)):
            point = self.edge_points[index]
            if self._point_inside_obstacle(point, circles):
                continue
            direct = self.grid.line_clear((x, y), point)
            score = math.hypot(point[0] - x, point[1] - y) + (0.0 if direct else 1.5)
            candidates.append((score, index, direct))
        if not candidates:
            self._finish_edge_phase()
            return
        # 沿环线优先恢复“当前进度之后的下一个边缘点”，不能因为
        # 末端闭合点几何上更近就直接跳到终点，否则会跳过半个区域。
        candidates.sort(key=lambda item: (item[1], item[0]))
        for _, index, direct in candidates[:12]:
            self.edge_index = index
            target = self.edge_points[index]
            self.edge_resume_target = index
            self.edge_resume_attempts = 0
            if direct:
                self.phase = "zone_edge"
                return
            # 障碍物仍挡在机器人与其后的边缘点之间时，先用 A* 贴边
            # 绕回边缘；失败则尝试下一个更远的边缘点，不能原地等待。
            if self._set_route(target, "edge_resume"):
                self.phase = "to_segment"
                return
        self._log("warn", "绕障后找不到可达边缘点，结束本区域沿边并继续割草")
        self._finish_edge_phase()

    def _point_inside_obstacle(self, point, circles):
        for circle in circles:
            if math.hypot(point[0] - circle["x"], point[1] - circle["y"]) < circle["r"] + 0.18:
                return True
        return False

    def _on_bump(self):
        if self.phase in ("zone_stripes", "zone_edge"):
            self.zone_bump_count += 1
        if self.phase == "zone_stripes" and self.zone_bump_count >= 2:
            if self.segment is not None:
                self.failed_segments.add(_segment_key(self.segment))
            self.zone_bump_count = 0
            self._log("warn", "同一区域连续碰撞，跳过受阻弓字段并重新规划剩余区域")
            self.phase = "zone_plan"
            return
        if self.phase == "zone_edge":
            circle = self._nearest_circle(self._pose()[0], self._pose()[1], 1.20)
            if circle is not None and not self._is_edge_ringed(circle):
                if self._begin_edge_ring(circle):
                    return
            if circle is None:
                circle = self._edge_circle_from_occupancy()
                if circle is not None and not self._is_edge_ringed(circle):
                    if self._begin_edge_ring(circle):
                        return
            if circle is not None:
                self._advance_edge_past_circle(circle)
            else:
                self._advance_edge_after_collision(0.80)
            return
        if self.phase == "ring":
            self._log("warn", "环形绕障受阻，跳过该障碍并重新规划")
            if self.ring_circle is not None:
                self.bypassed_circles.append(dict(self.ring_circle))
                self.ring_circle = None
            self.ring_points = []
            self.ring_index = 0
            if self.ring_after == "detour":
                self._finish_detour()
            elif self.ring_after == "rings":
                self._next_ring()
            else:
                self.phase = "zone_plan"
            return
        if self.phase == "detour":
            self._log("warn", "绕障受阻，重新贴回历史路径")
            if self.detour_source == "edge":
                self._finish_detour()
            else:
                self.detour_start = None
                self.detour_distance = 0.0
                self.detour_source = None
                if self.segment is not None:
                    self.failed_segments.add(_segment_key(self.segment))
                self.phase = "zone_plan"
            return
        if self.phase == "zone_plan":
            circle = self._nearest_circle(self._pose()[0], self._pose()[1], 1.0)
            if circle is not None:
                self.grid.stamp_obstacle(circle["x"], circle["y"], circle["r"])
                self.bypassed_circles.append(dict(circle))
            self.phase = "zone_plan"
        elif self.phase == "zone_stripes":
            self._begin_detour(self.phase)
        elif self.phase == "to_segment":
            if self.segment is not None:
                self.failed_segments.add(_segment_key(self.segment))
            self.phase = "zone_plan"
        elif self.phase in ("to_zone", "charge_go", "charge_return", "to_dock"):
            self._replan_route()

    # ----------------------------------------------------------- 沿边控制
    def _follow_surface(self, speed, desired, front_limit=0.38):
        readings = self.robot.vision
        front = _front_distance(readings, 15.0)
        right = _right_surface_distance(readings)
        if front is not None and front[0] < front_limit:
            return 0.16, 1.6
        nearest = min(d for _, d in readings) if readings else world.VISION_RANGE
        if nearest < 0.30:
            return 0.14, 1.6
        if right is None:
            return 0.35, -0.9
        if right < 0.30:
            return 0.30, 1.2
        if right < desired - 0.08:
            return min(speed, 0.40), 0.7
        if right > desired + 0.20:
            return min(speed, 0.40), -0.8
        return speed, 0.0

    # ----------------------------------------------------------- 导航
    def _find_route(self, goal):
        x, y, _ = self._pose()
        path = self.grid.plan((x, y), goal)
        if not path and self.grid.relax_around(x, y, 0.30):
            # 机器人当前点可能被相邻障碍的保守膨胀包围；该点本身已证明
            # 可通行，先释放脚下膨胀再尝试一次，避免区域间导航死锁。
            path = self.grid.plan((x, y), goal)
        if path:
            return path, path[-1], 0.0
        near_path, distance = self.grid.plan_near((x, y), goal)
        if near_path:
            return near_path, near_path[-1], distance
        return [], goal, float("inf")

    def _set_route(self, goal, kind):
        goal = (float(goal[0]), float(goal[1]))
        path, endpoint, near_distance = self._find_route(goal)
        limits = {
            "to_zone": 2.0,
            "to_segment": 0.8,
            "edge_resume": 1.4,
            "breakpoint": 4.0,
            "charge_return": 4.0,
            "to_dock": 4.0,
            "map_dock": 4.0,
            "mow_dock": 4.0,
        }
        if path and kind == "to_zone" and self._zone_id_at(endpoint) != self.zone_id:
            path = []
        if path and near_distance > limits.get(kind, 1.5):
            self._log(
                "warn",
                "目标点被障碍覆盖且最近可达点距离 {:.1f} m，改为重新规划".format(
                    near_distance
                ),
            )
            path = []
        if not path:
            self.route = []
            self.route_index = 0
            self.nav_goal = None
            self.route_kind = kind
            self._log("warn", "目标点不可达，尝试跳过或重新规划")
            self._handle_unreachable(kind)
            return False
        if near_distance > 0.30:
            self._log(
                "warn",
                "目标点被障碍覆盖，改去最近可达点（偏离 {:.2f} m）".format(
                    near_distance
                ),
            )
        self.nav_goal = endpoint
        self.route_kind = kind
        self.route = path
        self.route_index = 1 if len(path) > 1 else 0
        return True

    def _zone_id_at(self, point):
        zone = self.grid.zone_at(point[0], point[1])
        return zone["id"] if zone is not None else None

    def _handle_unreachable(self, kind):
        if kind == "to_segment":
            if self.segment is not None:
                self.failed_segments.add(_segment_key(self.segment))
            self.phase = "zone_plan"
        elif kind == "to_zone":
            self._skip_unreachable_zone()
        elif kind == "edge_resume":
            self._advance_edge_after_collision(0.55)
        elif kind == "breakpoint":
            self._restore_breakpoint()
        else:
            self.robot.stop()
            self.phase = "idle"

    def _skip_unreachable_zone(self):
        self._log("warn", "区域 {} 没有可达的割草入口，跳过该区域".format(self.zone_id))
        self.zone_index += 1
        if self.zone_index < len(self.zone_order):
            self.zone_id = self.zone_order[self.zone_index]
            self.zone_stage = None
            self.phase = "to_zone"
            self._plan_to_zone_corner()
        else:
            self.zone_id = None
            self.phase = "to_dock"
            self._set_route((world.DOCK["x"], world.DOCK["y"]), "mow_dock")

    def _replan_route(self):
        if self.nav_goal is None or self.replan_cooldown > 0.0:
            return False
        path, endpoint, near_distance = self._find_route(self.nav_goal)
        if path and near_distance <= 1.6:
            self.nav_goal = endpoint
            self.route = path
            self.route_index = 1 if len(path) > 1 else 0
            self.replan_cooldown = 0.8
            return True
        self._handle_unreachable(self.route_kind)
        self.replan_cooldown = 0.8
        return False

    def _nav_step(self, speed):
        if not self.route or self.route_index >= len(self.route):
            self._on_route_done()
            return 0.0, 0.0

        x, y, theta = self._pose()
        target = self.route[self.route_index]
        if self.grid.is_blocked_world(target[0], target[1]) or self.grid.is_occupied_world(
            target[0], target[1]
        ):
            if self.replan_cooldown <= 0.0:
                self._replan_route()
            return 0.0, 0.0
        distance = math.hypot(target[0] - x, target[1] - y)
        if distance < 0.20:
            self.route_index += 1
            if self.route_index >= len(self.route):
                self._on_route_done()
                return 0.0, 0.0
            target = self.route[self.route_index]
            distance = math.hypot(target[0] - x, target[1] - y)

        err = world.wrap_angle(math.atan2(target[1] - y, target[0] - x) - theta)
        v = speed * max(0.15, 1.0 - abs(err) / 1.6) if abs(err) < 1.15 else 0.0
        w = world.clamp(2.4 * err, -1.5, 1.5)

        front = _front_distance(self.robot.vision, 16.0)
        # 目标在侧后方时先原地转向目标；此时前向障碍不会造成平移碰撞，
        # 若仍按前向避障左右切换，会在狭窄回廊中来回摆动而卡死。
        if front is not None and front[0] < 0.42 and abs(err) < 1.0:
            v = 0.0
            w = -1.3 if front[1] >= 0.0 else 1.3
            return v, w

        readings = self.robot.vision
        if readings and v > 0.05:
            relative, distance = min(readings, key=lambda item: item[1])
            if distance < 0.34:
                v = min(v, 0.22)
                w += -1.3 if relative >= 0.0 else 1.3
        return v, w

    def _on_route_done(self):
        kind = self.route_kind
        self.route = []
        self.route_index = 0
        self.nav_goal = None
        self.route_kind = None
        if kind == "to_zone":
            self._enter_zone()
        elif kind == "to_segment":
            self.phase = "zone_stripes"
        elif kind == "edge_resume":
            self.phase = "zone_edge"
        elif kind == "breakpoint":
            self._arrive_breakpoint()
        elif kind in ("to_dock", "map_dock", "mow_dock"):
            self.robot.stop()
            if self.charge_reason:
                self.phase = "charging"
            else:
                self.phase = "done"
                self.task_end_sim = self.robot.sim_time
                self._log("event", "任务完成，已停靠充电桩")
        else:
            self.phase = "idle"

    # ----------------------------------------------------------- 环形补割
    def _plan_rings(self):
        if self.rings_done:
            return False
        zone = self._zone()
        if zone is None:
            return False
        circles = [
            circle
            for circle in self.grid.obstacle_circles(zone)
            if not self._is_bypassed(circle)
        ]
        self.ring_circles = circles
        if not self.ring_circles:
            self.rings_done = True
            return False
        self._log(
            "info",
            "区域 {} 对 {} 个障碍物进行环形补割".format(self.zone_id, len(self.ring_circles)),
        )
        self._next_ring()
        return True

    def _next_ring(self):
        if not self.ring_circles:
            self.rings_done = True
            self.phase = "zone_plan"
            return
        circle = self.ring_circles.pop(0)
        points = planner.ring_route(
            circle, self._pose()[:2], grid=self.grid, zone=self._zone()
        )
        if not points:
            self.bypassed_circles.append(dict(circle))
            self._next_ring()
            return
        self.ring_circle = circle
        self.ring_points = points
        self.ring_index = 0
        self.ring_after = "rings"
        self.ring_started = False
        self.ring_clipped = len(points) < planner.RING_STEPS
        self.phase = "ring"

    # ----------------------------------------------------------- 充电与断点
    def _save_breakpoint(self):
        if not self.task:
            return
        self.breakpoint = {
            "pose": self._pose(),
            "phase": self.phase,
            "zone_id": self.zone_id,
            "zone_index": self.zone_index,
            "zone_stage": self.zone_stage,
            "edge_points": list(self.edge_points),
            "edge_index": self.edge_index,
            "edge_done": self.edge_done,
            "edge_resume_target": self.edge_resume_target,
            "edge_resume_attempts": self.edge_resume_attempts,
            "edge_ringed": [dict(circle) for circle in self.edge_ringed_circles],
            "escape_resume": self.escape_resume,
            "edge_elapsed": (
                max(0.0, self.robot.sim_time - self.edge_start_sim)
                if self.edge_start_sim is not None
                else 0.0
            ),
            "map_path": list(self.map_path),
            "map_path_dist": self.map_path_dist,
            "map_islands": list(self.map_islands),
            "lane_axis": self.lane_axis,
            "bypassed": [dict(circle) for circle in self.bypassed_circles],
            "rings_done": self.rings_done,
            "failed_segments": list(self.failed_segments),
            "zone_plan_passes": self.zone_plan_passes,
            "patch_cleanup_done": self.patch_cleanup_done,
            "coverage_retries": self.coverage_retries,
        }

    def _charging_step(self):
        if self.robot.charging:
            if self.charge_reason == "low" and self.robot.battery >= RESUME_BATTERY:
                self._log(
                    "event",
                    "电量已充至 {:.0f}%，返回断点继续任务".format(RESUME_BATTERY),
                )
                self._resume_from_charge()
            elif self.charge_reason == "manual" and self.resume_requested:
                self._log("event", "按控制台要求继续任务")
                self._resume_from_charge()
        return 0.0, 0.0

    def _resume_from_charge(self):
        self.charge_reason = None
        self.resume_requested = False
        if not self.breakpoint:
            self.phase = "idle"
            return
        target = self.breakpoint["pose"][:2]
        self.phase = "charge_return"
        self._set_route(target, "breakpoint")

    def _arrive_breakpoint(self):
        if not self.breakpoint:
            self.phase = "idle"
            return
        saved_phase = self.breakpoint["phase"]
        if saved_phase == "map_follow":
            self.phase = "align"
            self.align_target = self.breakpoint["pose"][2]
            return
        self._restore_breakpoint()

    def _align_step(self):
        _, _, theta = self._pose()
        err = world.wrap_angle(self.align_target - theta)
        if abs(err) < 0.12:
            self._restore_breakpoint()
            return 0.0, 0.0
        return 0.0, world.clamp(1.6 * err, -1.6, 1.6)

    def _restore_breakpoint(self):
        saved = self.breakpoint
        if not saved:
            self.phase = "idle"
            return
        self.breakpoint = None
        self.zone_id = saved.get("zone_id")
        self.zone_index = saved.get("zone_index", 0)
        self.zone_stage = saved.get("zone_stage")
        self.mowing_mode = saved.get("mode", self.mowing_mode)
        self.edge_done = saved.get("edge_done", False)
        self.edge_resume_target = saved.get("edge_resume_target", -1)
        self.edge_resume_attempts = saved.get("edge_resume_attempts", 0)
        self.edge_ringed_circles = [dict(circle) for circle in saved.get("edge_ringed", [])]
        if saved.get("task_start_sim") is not None:
            self.task_start_sim = saved["task_start_sim"]
        self.lane_axis = saved.get("lane_axis", "h")
        self.bypassed_circles = [dict(circle) for circle in saved.get("bypassed", [])]
        self.rings_done = saved.get("rings_done", False)
        self.failed_segments = set(saved.get("failed_segments", []))
        self.zone_plan_passes = saved.get("zone_plan_passes", 0)
        self.patch_cleanup_done = saved.get("patch_cleanup_done", False)
        self.coverage_retries = saved.get("coverage_retries", 0)
        if saved.get("zone_stage") == "patch":
            self.patch_cleanup_done = False
        phase = saved["phase"]
        self._log("event", "已返回断点，继续之前的任务")
        if phase == "map_seek":
            self.phase = "map_seek"
        elif phase == "map_follow":
            self.map_path = list(saved.get("map_path", []))
            self.map_path_dist = saved.get("map_path_dist", 0.0)
            self.map_islands = list(saved.get("map_islands", []))
            self.phase = "map_follow"
        elif phase == "map_escape":
            self.phase = "map_seek"
        elif (
            saved.get("zone_stage") in ("edge", "edge_final")
            and saved.get("edge_points")
        ):
            self.edge_points = saved["edge_points"]
            self.edge_index = saved.get("edge_index", 0)
            self._prepare_edge_corners()
            # 暂停/回充期间 sim_time 仍会增长，不能把这段时间算进沿边超时。
            self.edge_start_sim = self.robot.sim_time - saved.get("edge_elapsed", 0.0)
            if self.edge_index >= len(self.edge_points):
                self._finish_edge_phase()
            else:
                self.phase = "zone_edge"
        elif phase in ("zone_stripes", "to_segment", "detour", "zone_plan", "ring", "escape"):
            self.phase = saved.get("escape_resume") or "zone_plan"
        elif phase == "to_zone":
            self.phase = "to_zone"
            self._plan_to_zone_corner()
        else:
            self.phase = "idle"

    # ----------------------------------------------------------- 状态输出
    def get_state(self):
        with self.lock:
            state = self.robot.get_state()
            zone_list = []
            order = self.zone_order if self.task and self.task["type"] == "mow" else []
            total_cells = 0
            mowed_cells = 0
            all_total = 0
            all_mowed = 0
            for zone in self.grid.zones:
                total = self.grid.zone_totals.get(zone["id"], 0)
                mowed = self.grid.zone_mowed.get(zone["id"], 0)
                all_total += total
                all_mowed += mowed
                if zone["id"] in order:
                    total_cells += total
                    mowed_cells += mowed
                zone_list.append(
                    {
                        "id": zone["id"],
                        "name": zone["name"],
                        "x0": zone["x0"],
                        "y0": zone["y0"],
                        "x1": zone["x1"],
                        "y1": zone["y1"],
                        "total": total,
                        "mowed": mowed,
                        "coverage": round(min(1.0, mowed / float(total)) if total else 0.0, 3),
                        "selected": zone["id"] in order,
                        "current": zone["id"] == self.zone_id,
                    }
                )
            overall = (mowed_cells / float(total_cells)) if total_cells else 0.0
            overall_all = (all_mowed / float(all_total)) if all_total else 0.0
            end_sim = (
                self.task_end_sim
                if self.task_end_sim is not None
                else self.robot.sim_time
            )
            duration = (
                max(0.0, end_sim - self.task_start_sim)
                if self.task_start_sim is not None
                else 0.0
            )

            task = {
                "type": self.task["type"] if self.task else None,
                "phase": self.phase,
                "phase_label": PHASE_LABELS.get(self.phase, self.phase),
                "zone_order": list(order),
                "current_zone": self.zone_id,
                "coverage": round(overall, 4),
                "overall_coverage": round(overall_all, 4),
                "mode": self.mowing_mode,
                "distance": round(state.get("distance", 0.0), 2),
                "duration": round(duration, 1),
                "mowed_cells": all_mowed,
                "total_cells": all_total,
                "stats": dict(self.stats),
                "map_progress": round(self.grid.boundary_discovery(), 3),
                "charge_reason": self.charge_reason,
                "message": self.message,
                "serial": self.task_serial,
                "breakpoint": (
                    {
                        "x": round(self.breakpoint["pose"][0], 2),
                        "y": round(self.breakpoint["pose"][1], 2),
                        "phase": self.breakpoint["phase"],
                    }
                    if self.breakpoint
                    else None
                ),
            }

            return dict(
                state,
                mode="manual" if not self.task else self.task["type"],
                mowing_mode=self.mowing_mode,
                task=task,
                zones=zone_list,
                obstacles=self.grid.obstacle_circles(None),
                map=self.grid.encode(),
                world=world.world_dict(),
                route=[[round(p[0], 2), round(p[1], 2)] for p in self.route],
                segments=[
                    [[round(a[0], 2), round(a[1], 2)], [round(b[0], 2), round(b[1], 2)]]
                    for a, b in self.segments
                ],
                edge_route=[[round(p[0], 2), round(p[1], 2)] for p in self.edge_points],
                ray_scan=[
                    [round(math.degrees(relative), 1), round(distance, 3)]
                    for relative, distance in self.robot.vision
                ],
                goal=(
                    [round(self.nav_goal[0], 2), round(self.nav_goal[1], 2)]
                    if self.nav_goal
                    else None
                ),
            )


def _segment_length(segment):
    a, b = segment
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _segment_key(segment):
    a, b = segment
    return (round(a[0], 1), round(a[1], 1), round(b[0], 1), round(b[1], 1))


def _validate_zones(raw, available):
    values = []
    if isinstance(raw, str):
        parts = raw.replace("，", ",").split(",")
    elif isinstance(raw, (list, tuple)):
        parts = raw
    else:
        parts = []
    for part in parts:
        try:
            value = int(part)
        except (TypeError, ValueError):
            continue
        if value in available and value not in values:
            values.append(value)
    return values or list(available)


def _front_distance(readings, half_angle_deg):
    best = None
    for relative, distance in readings:
        if abs(math.degrees(relative)) > half_angle_deg:
            continue
        if distance >= world.VISION_RANGE - 1e-6:
            continue
        if best is None or distance < best[0]:
            best = (distance, relative)
    return best


def _right_surface_distance(readings):
    values = []
    for relative, distance in readings:
        degrees = math.degrees(relative)
        if -50.0 <= degrees <= -25.0 and distance < world.VISION_RANGE - 1e-6:
            values.append(distance * math.sin(math.radians(-degrees)))
    if not values:
        return None
    return min(values)


def _polygon_area(points):
    if len(points) < 3:
        return 0.0
    area = 0.0
    count = len(points)
    for index in range(count):
        x1, y1 = points[index]
        x2, y2 = points[(index + 1) % count]
        area += x1 * y2 - x2 * y1
    return area / 2.0


def _path_length(points):
    total = 0.0
    for index in range(1, len(points)):
        total += math.hypot(points[index][0] - points[index - 1][0], points[index][1] - points[index - 1][1])
    return total
