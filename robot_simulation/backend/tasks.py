"""任务控制：自动建图（含孤岛识别）、自动分区、割草、回充与断点续作。"""

import math
import os
import threading
import time
from collections import deque

import bag_player
import config
import planner
import robot_car
import world

LOW_BATTERY = 30.0
RESUME_BATTERY = 80.0
DEFAULT_MOWING_MODE = "A"
MAX_ZONE_PLAN_PASSES = 30
# 禁区的规划膨胀（m）：必须明显大于反应式禁区保护的距离（车体半径 + 0.06），
# 这样按禁区切出来的弓字线不会落在保护带里，机器就不会在禁区旁边反复改路线。
NO_GO_MARGIN = 0.28
# 同位置连续重规划多少次算"车被顶住了"（不是这些弓字线走不到）
PLAN_STUCK_LIMIT = 5
# 每个区域最多为"被顶住"后退几次，避免来来回回退个不停
PLAN_STUCK_BACKTRACKS = 3
# 割草报告的轨迹采样间隔（m）
TRACK_SAMPLE_STEP = 0.16
PATCH_MIN_CELLS = 20
# 覆盖率不足时重补割的轮次上限，以及每轮至少要有多少提升才继续
COVERAGE_RETRY_LIMIT = 4
COVERAGE_RETRY_MIN_GAIN = 0.005
# 区域内还剩这么多平方米没割到，就再补割一轮
COVERAGE_RETRY_MIN_AREA = 1.5
PATCH_MAX_CELLS = 900

# 单次导航规划的时间预算（秒）：大图 A* 可能几百毫秒，必须限时，
# 否则一帧里多次规划叠加会把仿真线程卡住
ROUTE_PLAN_BUDGET = 0.30
# 单个 tick 内所有 A* 的时间上限：超了就这一帧先不做规划，下一帧再说
TICK_PLAN_BUDGET = 0.60
# 一次性导航目标（换区域/去弓字起点）可以给足：实景大图上单次 A* 要 0.1-0.5 s，
# 预算给小了会直接规划失败，被判成"目标不可达"而跳过整段弓字。
SET_ROUTE_PLAN_BUDGET = 0.60
# 建图找边界不是每帧都做（1.5 s 一次），可以给更宽的预算
MAP_PLAN_BUDGET = 0.35
# 回桩/换区域这类“低频但必须成功”的规划可以多花点时间（一帧卡顿总比回不去桩好）
DOCK_PLAN_BUDGET = 0.35

NAV_SPEED = 0.38
MAP_SPEED = 0.55
MOW_SPEED = 0.48
EDGE_SPEED = 0.50
INNER_CORNER_ADVANCE = 0.16

MIN_OUTER_AREA = 12.0
ISLAND_AREA = 8.0

# 回充电量预估：走到充电桩每米的耗电（%/m）与固定余量。
# 依据 robot_car 的耗电模型回推：耗电系数 0.6 时满速约 0.5 %/m，再乘 1.25 安全系数。
# 实际按当前耗电系数等比缩放（见 _return_reserve），改参数时预留电量自动跟着变。
RETURN_SOC_PER_METER = 0.65
RETURN_SOC_BASE_DRAIN = 0.6
RETURN_SOC_MARGIN = 3.0
# 建图巡边多久没有任何新边界进展就判定"困在小区域"
MAP_STALL_SECONDS = 30.0
# 沿墙/绕障判断"历史路径"时只认本区域内真正作业走过的轨迹：
# 区域之间的转场（to_zone）走的是另一条路，混进来会让脱困沿转场路线跑。
ZONE_TRAIL_PHASES = frozenset(
    (
        "zone_edge",
        "zone_stripes",
        "zone_plan",
        "to_segment",
        "corner_clean",
        "ring",
        "detour",
        "no_go_ring",
        "edge_resume",
    )
)
# 沿边跳过受阻段时至少跳过几个路径点（0.14 m/点 → 约 1.1 m）
EDGE_SKIP_MIN_POINTS = 8
# 割草阶段的安全网：这么久净位移还不到 1.5 m，就放弃本区域继续往下做
MOW_STALL_LIMIT = 180.0
MOW_STALL_PHASES = frozenset(
    ("to_zone", "to_segment", "zone_stripes", "zone_edge", "zone_plan")
)
# 原地打转超过这个角度（约两圈）还没挪窝，就按卡滞处理
MAX_SPIN_WITHOUT_MOVE = 12.6
# 位置 2.6 s 不动（或转了两圈还没挪窝）就判定卡滞
STUCK_TRIGGER_SECONDS = 2.0
# 顶着墙/障碍物还在踩油门（碰撞）超过这么久、而车没怎么动，就立刻倒车脱困，
# 不再傻等 STUCK_TRIGGER_SECONDS。
BUMP_STUCK_SECONDS = 0.5
# 去弓字起点（to_segment）最多磨多久 / 重规划几次就必须跳过这一段
SEGMENT_NAV_SECONDS = 5.0
SEGMENT_NAV_ATTEMPTS = 2
# 倒车脱困：速度、距离、时间上限。原来 -0.22 m/s 退 1 m 要 4.5 s，卡一次就是
# 一大截；车体半径才 0.18 m，退 1.2 m 足够让开墙面/障碍物拐角。
ESCAPE_SPEED = -0.34
ESCAPE_DISTANCE = 1.2
ESCAPE_TIME_LIMIT = 4.0
# 一个区域里最多用几次"沿历史路径倒车脱困"（再多就换阶段兜底，别在原地反复退）
ESCAPE_LIMIT = 5
# 这些阶段要按"净位移"判卡滞：10 s 内走不动 1 m 就算磨蹭
PROGRESS_GUARD_PHASES = frozenset(
    (
        "to_zone",
        "to_segment",
        "zone_stripes",
        "zone_edge",
        "ring",
        "detour",
        "to_dock",
        "charge_go",
        "charge_return",
        "map_fill",
        "map_seek",
        "map_escape",
    )
)
PROGRESS_GUARD_SECONDS = 10.0
PROGRESS_GUARD_DISTANCE = 1.0
# 回桩路线不通时，隔多久再试一次（期间继续手上的建图/割草）；连续失败会翻倍退避
CHARGE_RETRY_SECONDS = 30.0
CHARGE_RETRY_MAX = 240.0
# 未探明的墙面超过这个长度就算"明显缺口"，必须去补扫，不能直接结束建图
# （0.3 m 大约是 3 格墙：再短的缺口肉眼几乎看不出来，可以放过）
MAP_GAP_MAX_RUN = 0.3
# 同一轮建图最多补扫几次，避免在不可达缺口上无限绕
MAP_GAP_ATTEMPTS = 4
# 边界还没探明就想收工时，最多"再找一轮"几次（每次放开周边膨胀重新探索）
MAP_RECOVER_LIMIT = 6
# 建图兜底：边界探明率还不到这个数，才把场内未知区扫一遍（正常场地走完外墙 100%，
# 不会触发）。实景大图的分区墙也算"边界"，只绕外轮廓永远探不到。
MAP_FILL_TRIGGER = 0.95
# 兜底扫描的行距、最少剩余面积、时间与轮次上限、速度
MAP_FILL_SPACING = 0.80
MAP_FILL_MIN_AREA = 4.0
MAP_FILL_TIME_LIMIT = 1800.0
MAP_FILL_MAX_PASSES = 4
MAP_FILL_SPEED = 0.80
# "开过去看一眼"的目标点离自己/已访问点这么近就跳过
MAP_FILL_SPOT_RADIUS = 1.0
# 命令前进却原地不动超过这个时间/位移，就判定这条扫描线开不过去
MAP_FILL_STALL_SECONDS = 1.0
MAP_FILL_STALL_DISTANCE = 0.10
# 连续几次补扫都没有新增边界，就认为剩下的缺口补不上，收尾结束建图
MAP_GAP_WASTE_LIMIT = 3
# 补扫白跑一次后，这处缺口周边多大范围内不再重复跑
MAP_GAP_AVOID_RADIUS = 2.0
# 单次补扫至少要让边界探明率涨这么多，否则算"白跑一趟"
MAP_GAP_MIN_GAIN = 0.005
# 补扫观察点可以被认为是"够近"的最大距离（m）
MAP_GAP_REACH = 1.2
# 走到补扫观察点的时间上限（s）：实景大场地单程二三十米很正常
MAP_GAP_REACH_TIME = 90.0
# 补扫规划的时间预算：这一步低频但必须判准可达性，可以多花点时间
MAP_GAP_PLAN_BUDGET = 0.40
# 补扫时停在缺口内侧多远的位置观察
MAP_GAP_VIEW_OFFSET = 0.55
MAP_DONE_DISCOVERY = 0.85
# 收工时允许残留的未探明墙面总长（m）：低于它就算"边界找齐了"，高于它就再绕一圈。
# 之前给 0.6 m，收工时地图边上总会留一小段没涂抹，看着像"建图漏了一点"。
MAP_DONE_RESIDUAL = 0.25
# 最多绕几圈把最后一点墙刷上；超过就收工（不再无限找缺口）
MAP_CLOSE_MAX_LAPS = 2
# 绕障过程中允许擦碰 / 卡滞多少次还继续绕（绕障半径贴着障碍物时蹭到是常态，
# 一碰就放弃会让障碍物后面留一条没割的带子）
RING_BUMP_LIMIT = 5
# 绕障弧段连续被挡住多少次才退化成"贴面绕行"（以前一卡就跳过弧段，绕障断得很随意）
RING_PROBE_LIMIT = 8
# 退化 / 卡滞后仍绕不过去时，允许连续几次改成贴面绕行
RING_FALLBACK_LIMIT = 3
# 贴面绕障（沿障碍物表面绕）允许的行程与越界余量：
# 障碍物贴在区域边上时，绕行轨迹必然要探出区域框一点点，卡太死会一走就"绕障中断"
DETOUR_MAX_DISTANCE = 14.0
DETOUR_ZONE_SLACK = 0.35
# 一次绕障里最多停下来"花瓣式"找几次小障碍物（路径本身是几何算好的，
# 看不见也可以照着绕，扫太多次会把绕障拖到超时）
RING_SCAN_LIMIT = 3
# 寻找未探明边界连续卡住几次就结束建图回桩
MAP_SEEK_STUCK_LIMIT = 3
# 建图阶段累计活动时间（不含充电/回桩）上限：超过就收工。实景图上有些墙角
# 真的开不进去，宁可带着"边界探明 80%"的结论去分区，也不要无限找下去。
MAP_ACTIVE_TIME_LIMIT = 1500.0
# 边界进度这么久（秒）没有再涨过，就直接结束建图：实景图上有些墙角（长死胡同、
# 反斜坡）真的开不进去，硬找只会一直磨到没电。
MAP_NO_PROGRESS_LIMIT = 240.0
# 巡边过程中边界已探明到这个比例就直接收尾（避免"建图完成了还在建图"）
MAP_DONE_DISCOVERY_NOW = 0.98

# 路线种类 → 实际执行阶段（这些阶段都用 _nav_step 巡迹）
_ROUTE_PHASES = {
    "to_zone": "to_zone",
    "to_zone_stripes": "to_zone",
    "to_segment": "to_segment",
    "to_dock": "to_dock",
    "map_dock": "to_dock",
    "mow_dock": "to_dock",
    "charge_return": "charge_return",
    "edge_resume": "edge_resume",
    "map_fill": "map_fill",
}

_ROUTE_LABELS = {
    "to_zone": "割草区域",
    "to_zone_stripes": "割草区域",
    "to_segment": "弓字起点",
    "to_dock": "充电桩",
    "map_dock": "充电桩",
    "mow_dock": "充电桩",
    "charge_return": "充电桩",
    "edge_resume": "边缘断点",
    "breakpoint": "断点",
    "map_fill": "补扫未知区",
}

PHASE_LABELS = {
    "idle": "待机",
    "map_seek": "寻找场地边界",
    "map_follow": "逆时针巡边建图",
    "map_escape": "脱离孤岛",
    "map_fill": "点看未知区域",
    "stopped": "任务已暂停",
    "to_zone": "前往割草区域",
    "zone_edge": "沿区域边缘割草",
    "zone_plan": "规划弓字轨迹",
    "to_segment": "前往弓字起点",
    "zone_stripes": "弓字割草",
    "detour": "绕障并回到历史路径",
    "ring": "绕障碍物一周",
    "charge_go": "返回充电桩",
    "replay": "日志回放",
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
        self.replay = bag_player.BagReplay(robot)
        self.battery_series = []
        self.lock = threading.RLock()
        self.task_serial = 0
        self._last_collisions = 0
        self._reset_state()

    # ----------------------------------------------------------- 基础状态
    def _reset_mow_record(self, mode):
        """新一次割草任务：清空上一份数据记录。"""
        self.mow_track = []
        self.mow_plan_segments = {}
        self.mow_plan_edges = {}
        self.mow_zone_records = []
        self.mow_report = None
        self.mow_report_written = False
        self.mow_start_sim = self.robot.sim_time
        self.mow_start_battery = self.robot.battery
        self.mow_start_distance = self.robot.distance_traveled
        self.mow_start_collisions = self.robot.collisions
        self.mow_mode = mode
        self.zone_start_sim = None
        self.zone_stats_mark = dict(self.stats)
        self.plan_anchor = None
        self.plan_stuck = 0
        self.plan_stuck_backtracks = 0

    def _reset_state(self):
        self.task = None
        self.phase = "idle"
        self.dock_retries = 0
        self.charge_wait = 0.0
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
        self.segment_plan = None
        self.segment_reversed = False
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
        # lane_axis 是当前要用的弓字方向；main_lane_axis 是本区域的主方向，
        # 补割时可以换成另一个方向，但主覆盖阶段不再来回跳。
        self.lane_axis = "h"
        self.main_lane_axis = "h"
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
        self.patch_passes = 0
        self.last_plan_coverage = 0.0
        self.zone_total_plans = 0
        self.route_fail_streak = 0
        self.coverage_retries = 0
        self.coverage_retry_best = 0.0
        self.ring_clipped = False
        self.ring_lost_time = 0.0
        self.ring_bumps = 0
        self.ring_scan_count = 0
        self.ring_stuck = 0
        self.ring_time_limit = 35.0
        self.ring_probe = 0
        self.ring_surface_tries = 0
        self.ring_origin = None
        self.ring_radius = 0.0
        self.detour_ring_after = None
        self.bypass_skip_count = 0
        self.ring_scan_time = 0.0
        self.ring_scan_heading = 0.0
        self.task_start_sim = None
        self.task_end_sim = None
        # ---- 割草数据记录（任务结束时写成报告） ----
        self.mow_track = []              # 实际轨迹（按距离采样）
        self.mow_plan_segments = {}      # 规划过的弓字线段：key -> (a, b)
        self.mow_plan_edges = {}         # 每个区域的沿边规划线：zone_id -> points
        self.mow_zone_records = []       # 每个区域割完时的数据快照
        self.mow_report = None           # 最近一次生成的报告信息
        self.mow_report_written = False
        self.mow_start_sim = None
        self.mow_start_battery = None
        self.mow_start_distance = 0.0
        self.mow_start_collisions = 0
        self.zone_start_sim = None
        self.zone_stats_mark = None
        self.plan_anchor = None          # 上次"有进展"时的位置（重规划打转检测）
        self.plan_stuck = 0
        self.plan_stuck_backtracks = 0
        self.stats = {
            "segments_planned": 0,
            "rings": 0,
            "large_replans": 0,
            "edge_skips": 0,
            "stuck_recoveries": 0,
            "charge_count": 0,
        }
        self.no_go_ring_points = []
        self.no_go_ring_index = 0
        self.no_go_ring_started = False
        self.no_go_ring_resume = None
        self.no_go_ring_exit = None
        self.no_go_ring_exit_point = None
        self.no_go_ring_last = None
        self._edge_len_key = None
        self._edge_len_value = 0.0
        self._map_cache = None
        self.ring_points = []
        self.ring_index = 0
        self.ring_after = None
        self.ring_started = False
        self.ring_circles = []
        self.ring_circle = None
        self.phase_time = 0.0
        self.last_phase = None
        self.stuck_time = 0.0
        self._dyn_yield = False
        self.dock_stuck = 0
        self.stuck_ref = None
        self.stuck_heading = 0.0
        self.stuck_spin = 0.0
        self.progress_ref = None
        self.progress_ref_time = 0.0
        self.stall_ref = None
        self.stall_time = 0.0
        self.edge_stuck = 0
        self.detour_start = None
        self.detour_distance = 0.0
        self.detour_source = None
        self.align_target = 0.0
        self.bypassed_circles = []
        self.history = deque(maxlen=800)
        self.zone_history = deque(maxlen=800)
        self._prev_pose = None

        self.mapping_done = False
        self.map_path = []
        self._map_hash = {}
        self.map_path_dist = 0.0
        self.map_islands = []
        self.map_seek_log_time = 0.0
        self.map_last_discovery = 0.0
        self.map_progress_time = 0.0
        self.map_seek_goal = None
        self.map_seek_goal_time = -99.0
        # 补扫缺口用：观察点 + 已尝试次数（每轮建图复位）
        self.map_gap_goal = None
        self.map_gap_attempts = 0
        self.map_gap_goal_time = 0.0
        self.map_gap_scan_time = -1.0
        self.map_gap_point = None
        self.map_gap_baseline = 0.0
        self.map_gap_waste = 0
        self.map_gap_done = False
        self.map_gap_abandoned = []
        self.map_gap_reach = {}
        self.map_gap_unreachable_time = None
        self.map_gap_remaining = 0.0
        self.map_recover_total = 0
        self.zones_ready = False
        self.map_fill_segments = []
        self.map_fill_target = None
        self.map_fill_stage = "seek"
        self.map_fill_passes = 0
        self.map_fill_start_sim = None
        self.map_fill_last_area = None
        self.map_fill_progress = None
        self.map_fill_cmd_v = 0.0
        self.map_fill_visited = []
        self.map_loop_count = 0
        self.map_seek_stuck = 0
        self.map_best_discovery = 0.0
        self.map_best_time = 0.0
        self.map_active_time = 0.0
        self._map_watch_phase = None
        self._map_residual_cache = None
        self.map_seek_mode = False
        self.map_seek_mode_time = 0.0
        self.map_follow_side = 1
        self.map_side_flips = 0
        self.map_seek_route = []
        self.map_seek_index = 0
        self.map_seek_blacklist = []
        self.map_seek_last_pose = None
        self.map_seek_move_time = 0.0
        self.map_seek_best = None
        self.map_seek_progress_time = 0.0
        self.route_from_history = False
        self.route_loose = False
        self.route_waypoint = None
        self.route_waypoint_time = 0.0
        self.route_waypoint_best = 0.0
        self.segment_nav_attempts = 0
        self.zone_backtracks = 0
        self.dock_backtracks = 0
        self.dock_final_approach = 0
        self.backtrack_next = None
        self.charge_retry_time = 0.0
        self.charge_retry_delay = CHARGE_RETRY_SECONDS
        self.loose_plan_cooldown = 0.0
        self.tick_plan_left = TICK_PLAN_BUDGET
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
            # 用户手动添加的障碍物/禁区要一直保留（只能由用户自己删除），
            # 重新建图只清传感器建出来的栅格。
            self.grid.reset_map()
            self.task_serial += 1
            self.task_start_sim = self.robot.sim_time
            self.task_end_sim = None
            self.task = {"type": "map", "mode": self.mowing_mode}
            self.mapping_done = False
            self.map_gap_attempts = 0
            self.map_gap_goal = None
            self.map_gap_point = None
            self.map_gap_waste = 0
            self.map_gap_done = False
            self.map_gap_abandoned = []
            self.map_gap_reach = {}
            self.map_gap_unreachable_time = None
            self.map_gap_remaining = 0.0
            self.map_recover_total = 0
            self.zones_ready = False
            self.map_fill_segments = []
            self.map_fill_target = None
            self.map_fill_stage = "seek"
            self.map_fill_passes = 0
            self.map_fill_start_sim = None
            self.map_fill_last_area = None
            self.map_fill_progress = None
            self.map_fill_cmd_v = 0.0
            self.map_fill_visited = []
            self.map_loop_count = 0
            self.map_seek_stuck = 0
            self.map_best_discovery = 0.0
            self.map_best_time = self.robot.sim_time
            self.map_active_time = 0.0
            self._map_residual_cache = None
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
            explicit_order = _validate_zones(zone_order, available)
            if explicit_order:
                order = explicit_order
            else:
                # 留空就按"离当前车位置最近"排区域，少跑区域之间的空趟
                order = planner.order_zones_by_travel(self.grid.zones, self._pose()[:2])
            self._reset_state()
            self.mowing_mode = selected_mode
            self.task_serial += 1
            self.task_start_sim = self.robot.sim_time
            self.task_end_sim = None
            self.grid.reset_mowed()
            # 新任务从建图完成时的障碍层开始：清掉上次割草涂抹的障碍，边界建图保留。
            if self.grid.restore_obstacle_baseline():
                self._log("info", "已重置区域内障碍涂抹（保留边界建图），本次任务重新识别障碍")
            # 禁区膨胀必须大于"反应式保护"的判定距离（车体半径 + 0.06），否则弓字线
            # 会落在禁区保护带里：机器走到线段尽头就被判"前方是禁区"，停下来转向、
            # 重规划，看起来就是在禁区边上一直改路线来回跑。A/B 统一按 0.28 m。
            self.grid.apply_no_go(world.NO_GO, margin=NO_GO_MARGIN)
            # 覆盖分母按"当前真正能割的格子"重算：禁区、已探明障碍物不该算进分母
            self.grid.refresh_zone_totals()
            self.task = {"type": "mow", "zones": order, "mode": selected_mode}
            self._reset_mow_record(selected_mode)
            self.zone_order = order
            self.zone_index = 0
            self.zone_id = order[0]
            self.grid.set_reach_seed(*self._pose()[:2])
            self.ring_surface_tries = 0
            self.phase = "to_zone"
            self._log(
                "event",
                "开始自动割草（{} 模式），区域顺序：{}".format(
                    selected_mode, ", ".join(str(z) for z in order)
                ),
            )
            if not explicit_order:
                self._log(
                    "info",
                    "区域顺序按路程就近排（{}）：先近后远，减少区域之间的空跑".format(
                        "→".join(str(z) for z in order)
                    ),
                )
            try:
                self._plan_to_zone_corner()
            except Exception as exc:  # pragma: no cover - 兜底，别让规划异常卡死任务
                # 规划异常（以前沿边掩膜越界就会走到这里）不能留下一个"有任务但不动"
                # 的状态：记一条告警，直接转入弓字覆盖（沿边留给后续补割），机器照常干活。
                self._log(
                    "warn",
                    "前往区域 {} 的沿边路线规划失败（{}），改为先做弓字覆盖".format(
                        self.zone_id, exc
                    ),
                )
                self.route = []
                self.route_kind = None
                self.nav_goal = None
                if self._zone() is None:
                    self.phase = "idle"
                else:
                    self.zone_stage = "stripes"
                    self.phase = "zone_plan"

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

    # ----------------------------------------------------------- 日志回放
    def load_replay(self, info, transform=None):
        with self.lock:
            state = self.replay.load(info, transform)
            self.robot.ghost = False
            # 电量跟着日志走：日志里有电量话题就按它显示，否则锁在 100% 不变
            self.battery_series = list((info or {}).get("battery") or [])
            if self.battery_series:
                self.robot.battery_lock = None
                self._apply_replay_battery()
            else:
                self.robot.battery_lock = 100.0
                self.robot.battery = 100.0
            return state

    def _apply_replay_battery(self):
        if not self.battery_series:
            return
        when = self.replay.time
        best = self.battery_series[0]
        for stamp, value in self.battery_series:
            if stamp > when:
                break
            best = (stamp, value)
        self.robot.battery = max(0.0, min(100.0, float(best[1])))

    def start_replay(self):
        with self.lock:
            if self.task is not None:
                self.stop_task("开始日志回放，当前任务已取消", resumable=False)
            self._reset_state()
            self.task = {"type": "replay", "mode": "replay"}
            self.phase = "replay"
            self.task_start_sim = self.robot.sim_time
            self.robot.ghost = True
            state = self.replay.start()
            self._log(
                "event",
                "开始回放日志 {}：{} 个位姿，时长 {:.1f} s".format(
                    self.replay.name or "",
                    (self.replay.info or {}).get("summary", {}).get("poses", 0),
                    self.replay.duration,
                ),
            )
            return state

    def pause_replay(self):
        with self.lock:
            return self.replay.pause()

    def resume_replay(self):
        with self.lock:
            if self.task is None or self.task.get("type") != "replay":
                self.task = {"type": "replay", "mode": "replay"}
            self.phase = "replay"
            self.robot.ghost = True
            return self.replay.resume()

    def seek_replay(self, time):
        with self.lock:
            return self.replay.seek(time)

    def stop_replay(self):
        with self.lock:
            state = self.replay.stop()
            self.robot.ghost = False
            self.robot.battery_lock = None
            self.task = None
            self.phase = "idle"
            self.last_phase = None
            self._log("event", "结束日志回放")
            return state

    def clear_replay(self):
        with self.lock:
            self.replay.clear()
            self.robot.ghost = False
            self.robot.battery_lock = None
            self.battery_series = []
            if self.phase == "replay":
                self.task = None
                self.phase = "idle"
                self.last_phase = None

    def replay_image(self, time=None):
        """当前回放时刻的日志图像（割草分割图等）。"""
        if self.replay is None:
            return None
        when = self.replay.time if time is None else float(time)
        return self.replay.image_at(when)

    def replay_state(self, with_costmap=False):
        state = self.replay.state()
        state["phase"] = self.phase
        state["maps"] = (self.replay.info or {}).get("map_summary") or {}
        info = self.replay.info or {}
        state["battery_topic"] = info.get("battery_topic")
        state["battery_series"] = len(self.battery_series)
        state["battery_locked"] = self.robot.battery_lock is not None
        if with_costmap:
            state["costmap"] = self.replay.costmap_at(self.replay.time)
        return state

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
            self.tick_plan_left = TICK_PLAN_BUDGET
            world.update_dynamic(dt)
            if world.DYNAMIC:
                self.grid.apply_dynamic(world.DYNAMIC)
            readings = self.robot.scan()
            pose = self._pose()
            self.grid.update_from_scan(pose, readings, world.VISION_RANGE)

            step = 0.0
            if self._prev_pose is not None:
                step = math.hypot(pose[0] - self._prev_pose[0], pose[1] - self._prev_pose[1])
            self._prev_pose = pose
            if step > 0.0 and self.phase != "escape":
                # 按距离而不是按 tick 记录：低速时单步位移可能不足 2 cm，
                # 否则历史轨迹几乎为空，回充/脱困就没路可退。
                if not self.history or math.hypot(
                    pose[0] - self.history[-1][0], pose[1] - self.history[-1][1]
                ) >= 0.12:
                    self.history.append((pose[0], pose[1]))
                    if self.phase in ZONE_TRAIL_PHASES:
                        self.zone_history.append((pose[0], pose[1]))
                if (
                    self.mow_track is not None
                    and self.task is not None
                    and self.task.get("type") == "mow"
                    and (
                        not self.mow_track
                        or math.hypot(
                            pose[0] - self.mow_track[-1][0],
                            pose[1] - self.mow_track[-1][1],
                        )
                        >= TRACK_SAMPLE_STEP
                    )
                    and len(self.mow_track) < 40000
                ):
                    self.mow_track.append((round(pose[0], 3), round(pose[1], 3)))

            if self.replay.active:
                self.replay.tick(dt)
                self._apply_replay_battery()
                self.robot.step(dt)
                self._update_progress(dt)
                if self.replay.finished:
                    self._log(
                        "event",
                        "日志回放结束：仿真吻合度 {:.1f}%（平均误差 {:.2f} m）".format(
                            self.replay.fidelity() * 100.0,
                            self.replay.error_sum / max(1, self.replay.samples),
                        ),
                    )
                    self.stop_replay()
                return

            self._check_battery()
            if self.task is None and self.phase == "idle":
                controls = None
            else:
                controls = self._control(dt, step)
                controls = self._guard_no_go(controls, step)
                self._dyn_yield = False
                controls = self._guard_dynamic(controls)
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


    # 只在“反应式”运动阶段做禁区保护；路径导航阶段（to_zone/to_segment/回充）
    # 已经由栅格 blocked 层绕开禁区，再加保护会打断正常绕行。
    DOCK_PHASES = ("to_dock", "charge_go", "charge_return")

    # 反应式禁区保护只在"没有走过 A* / 没有按禁区切段"的运动阶段兜底。
    # 弓字（zone_stripes）的线段本身就按禁区膨胀切过了，再叠一层会在禁区拐角
    # 反复停车→转向→重规划，机器就在禁区边上左右来回跑；这里把它去掉。
    # 这些阶段允许"顶着障碍物不动 → 立刻倒车脱困"
    BUMP_ESCAPE_PHASES = ("zone_stripes", "zone_edge", "to_segment", "detour", "ring")

    NO_GO_GUARD_PHASES = (
        "map_seek",
        "map_follow",
        "map_escape",
        "zone_edge",
        "corner_clean",
        "detour",
        "ring",
    )

    DYNAMIC_YIELD_MARGIN = 0.34

    def _guard_dynamic(self, controls):
        """移动障碍物贴身时先制动让行，避免车体与移动物相撞后反复触发碰撞恢复。"""
        if not controls or not world.DYNAMIC:
            return controls
        x, y = self.robot.pose[0], self.robot.pose[1]
        for item in world.DYNAMIC:
            limit = item["r"] + world.ROBOT_RADIUS + self.DYNAMIC_YIELD_MARGIN
            if math.hypot(item["x"] - x, item["y"] - y) <= limit:
                self._dyn_yield = True
                return 0.0, 0.0
        return controls

    def _guard_no_go(self, controls, step=0.0):
        """沿边/割草/绕障等所有运动阶段都不允许进入禁区。"""
        if not controls or not world.NO_GO or self.phase not in self.NO_GO_GUARD_PHASES:
            return controls
        v, w = controls
        if abs(v) < 1e-6 and abs(w) < 1e-6:
            return controls
        clearance = 0.04
        x, y, theta = self._pose()
        direction = 1.0 if v >= 0.0 else -1.0
        look = max(0.20, min(0.45, abs(v) * 0.35 + 0.20))
        ahead_x = x + math.cos(theta) * look * direction
        ahead_y = y + math.sin(theta) * look * direction
        if not world.in_no_go(ahead_x, ahead_y, clearance):
            return controls
        # 前方进入禁区：停车并向远离禁区的方向转向
        if world.in_no_go(x, y, clearance):
            return 0.0, w if abs(w) > 0.2 else 1.2
        turn = 1.0 if not world.in_no_go(ahead_x + 0.15, ahead_y - 0.15, clearance) else -1.0
        if self.stuck_time > 1.5:
            return -0.15, -turn * 1.0
        return 0.0, turn * 1.2

    def _update_progress(self, dt):
        if self.phase != self.last_phase:
            self.last_phase = self.phase
            self.phase_time = 0.0
            self.stuck_ref = None
        else:
            self.phase_time += dt

        pose = (self.robot.pose[0], self.robot.pose[1])
        heading = self.robot.pose[2]
        if self.stuck_ref is not None:
            self.stuck_spin += abs(world.wrap_angle(heading - self.stuck_heading))
        if self.stuck_ref is None:
            self.stuck_ref = pose
            self.stuck_heading = heading
            self.stuck_spin = 0.0
            self.stuck_time = 0.0
        elif math.hypot(pose[0] - self.stuck_ref[0], pose[1] - self.stuck_ref[1]) > 0.05:
            self.stuck_ref = pose
            self.stuck_heading = heading
            self.stuck_spin = 0.0
            self.stuck_time = 0.0
        elif abs(world.wrap_angle(heading - self.stuck_heading)) > 0.30:
            # 原地掉头时位置不动，但朝向变了，说明车没卡住（否则会在墙角
            # 反复"判卡滞→后退→再掉头"，一直磨到电量耗尽）。
            self.stuck_heading = heading
            self.stuck_time = 0.0
            if self.stuck_spin > MAX_SPIN_WITHOUT_MOVE:
                # 转了两整圈还没走出一米：这不是掉头，是绕不出来
                self.stuck_time = STUCK_TRIGGER_SECONDS + 0.5
        else:
            self.stuck_time += dt

        # 慢速磨蹭也要算卡滞：机器人可能在 0.1 m 范围内来回蹭，每次位移都
        # 超过 0.05 m，逐帧判定永远不算卡住，于是一直磨到电量耗尽。
        if self.phase in PROGRESS_GUARD_PHASES:
            if self.progress_ref is None:
                self.progress_ref = (pose[0], pose[1])
                self.progress_ref_time = self.robot.sim_time
            elif (
                math.hypot(
                    pose[0] - self.progress_ref[0], pose[1] - self.progress_ref[1]
                )
                > PROGRESS_GUARD_DISTANCE
            ):
                self.progress_ref = (pose[0], pose[1])
                self.progress_ref_time = self.robot.sim_time
            elif self.robot.sim_time - self.progress_ref_time > PROGRESS_GUARD_SECONDS:
                self.progress_ref = (pose[0], pose[1])
                self.progress_ref_time = self.robot.sim_time
                self.stuck_time = max(self.stuck_time, STUCK_TRIGGER_SECONDS + 0.5)
        else:
            self.progress_ref = None

        # 安全网：割草阶段长时间原地磨蹭（换阶段也不算进展）就放弃本区域，
        # 保证任务一定能往下走，而不是耗到没电停在场地上。
        if self.phase in MOW_STALL_PHASES:
            if self.stall_ref is None:
                self.stall_ref = (pose[0], pose[1])
                self.stall_time = 0.0
            elif (
                math.hypot(pose[0] - self.stall_ref[0], pose[1] - self.stall_ref[1]) > 1.5
            ):
                self.stall_ref = (pose[0], pose[1])
                self.stall_time = 0.0
            else:
                self.stall_time += dt
                if self.stall_time > MOW_STALL_LIMIT:
                    self.stall_ref = None
                    self.stall_time = 0.0
                    self._abandon_zone()
                    return
        else:
            self.stall_ref = None
            self.stall_time = 0.0

        if self._dyn_yield:
            # 给动态障碍物让行属于正常停车，不能算卡滞，否则会触发无谓的脱困。
            self.stuck_ref = None
            self.stuck_time = 0.0
        if self.phase == "to_segment" and not self._dyn_yield and (
            self.phase_time > SEGMENT_NAV_SECONDS
            or self.stuck_time > STUCK_TRIGGER_SECONDS
        ):
            self.replan_cooldown = 0.0
            self.segment_nav_attempts += 1
            if self.segment_nav_attempts <= SEGMENT_NAV_ATTEMPTS and self._replan_route():
                self.stuck_time = 0.0
                self.stuck_ref = None
                self.phase_time = 0.0
            elif self.phase == "to_segment":
                # 同一个弓字起点反复规划都过不去：放过它，别再原地磨到没电
                self._log(
                    "warn",
                    "弓字起点 {} 连续规划仍不可达，跳过该段".format(self.segment_index + 1),
                )
                self._skip_segment()

        if self.stuck_time > STUCK_TRIGGER_SECONDS and self.phase in (
            "no_go_ring",
            "zone_edge",
            "zone_stripes",
            "ring",
            "detour",
            "to_zone",
            "to_dock",
            "charge_go",
            "charge_return",
            "map_fill",
        ):
            self._recover_stuck()
        if self.phase in ("map_seek", "map_follow", "map_escape", "map_fill"):
            discovery = self.grid.boundary_discovery()
            if self._map_watch_phase not in (
                "map_seek",
                "map_follow",
                "map_escape",
                "map_fill",
            ):
                # 刚回到建图（比如回充回来）：无进展计时重新开始，否则会把
                # 充电那段时间也算成"边界卡住"，一回来就收工。
                self.map_best_time = self.robot.sim_time
                self.map_best_discovery = discovery
            self._map_watch_phase = self.phase
            self.map_active_time += dt
            if self.map_active_time > MAP_ACTIVE_TIME_LIMIT and not self.mapping_done:
                self._log(
                    "warn",
                    "建图已经跑了 {:.0f} s（边界 {:.0f}%），剩余墙角开不过去，结束建图".format(
                        self.map_active_time, discovery * 100.0
                    ),
                )
                self._finish_mapping()
                return
            if discovery > self.map_best_discovery + 0.005:
                self.map_best_discovery = discovery
                self.map_best_time = self.robot.sim_time
                # 有进展就清掉"寻找边界卡住"的计数：只对连续无进展的卡住计数，
                # 否则开局蹭几下墙就把后面的机会全用完了。
                self.map_seek_stuck = 0
            elif (
                self.robot.sim_time - self.map_best_time > MAP_NO_PROGRESS_LIMIT
                and not self.mapping_done
            ):
                self._log(
                    "warn",
                    "边界进度停在 {:.0f}% 已经 {:.0f} s 没有变化，结束建图返回充电桩".format(
                        discovery * 100.0, self.robot.sim_time - self.map_best_time
                    ),
                )
                self._finish_mapping()
                return
        if self.phase == "corner_clean" and self.phase_time > 4.0:
            self._finish_corner()
        if self.phase == "no_go_ring" and self.phase_time > 25.0:
            self._log("warn", "绕禁区超时，结束绕行并继续任务")
            self._finish_no_go_ring()
        if self.phase == "ring" and self.phase_time > self.ring_time_limit:
            self._log("warn", "绕障超时，改用贴面绕行收尾")
            self._ring_to_surface("超时 {:.0f} s".format(self.phase_time))
        if (
            self.phase == "zone_edge"
            and self.edge_start_sim is not None
            and self.robot.sim_time - self.edge_start_sim > self._edge_time_limit()
        ):
            self._log("warn", "沿边超时，跳过剩余边缘并继续任务")
            self._finish_edge_phase()
        if self.phase in ("map_seek", "map_escape") and self.stuck_time > STUCK_TRIGGER_SECONDS:
            # 找边界时原地来回转、或者顶着墙磨：这才是最典型的"建图卡住"。
            # 先放弃当前前沿目标、沿历史轨迹退出去；连续几次都不行就结束建图回桩，
            # 不能一直磨到没电。
            self.map_seek_stuck += 1
            self.map_seek_goal = None
            self.map_gap_goal = None
            self.map_seek_route = []
            self.map_seek_index = 0
            self.map_seek_mode = False
            self.stuck_time = 0.0
            self.stuck_ref = None
            if self.map_seek_stuck > MAP_SEEK_STUCK_LIMIT:
                self._log(
                    "warn",
                    "寻找未探明边界连续 {} 次卡住，结束建图并返回充电桩".format(
                        self.map_seek_stuck - 1
                    ),
                )
                self._finish_mapping()
                return
            if self._begin_escape("map_seek"):
                self._log(
                    "warn",
                    "寻找边界受阻（第 {} 次），沿历史路径后退脱困".format(self.map_seek_stuck),
                )
                return
            self.map_escape_heading = world.wrap_angle(self._pose()[2] + math.pi)
            self.map_escape_time = 0.0
            self.map_escape_dist = 0.0
            self.phase = "map_escape"
            self.last_phase = None
            return
        if self.phase == "map_follow" and self.stuck_time > 2.2:
            # 优先沿历史路径倒车脱困；失败时再执行原来的远离障碍方向脱离。
            if self._begin_escape("map_follow"):
                self.map_path = []
                self._map_hash = {}
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
            self._map_hash = {}
            self.map_path_dist = 0.0
            self.phase = "map_escape"
            self.last_phase = None
            self._log("warn", "巡边受阻，脱离障碍物后重新寻找外墙")

    def _recover_stuck(self):
        phase = self.phase
        self.stats["stuck_recoveries"] += 1
        if self.route_kind == "backtrack":
            # 退回途中被卡住：就当退回结束，回到正常规划继续
            self._on_route_done()
            return
        self._log("warn", "检测到卡滞（{}），尝试后退沿历史路径脱困".format(phase))
        if phase == "zone_stripes" and self.segment is not None:
            self.failed_segments.add(_segment_key(self.segment_plan or self.segment))
        if self.zone_escape_count < ESCAPE_LIMIT and self._begin_escape(phase):
            self.zone_escape_count += 1
            self.stuck_time = 0.0
            self.stuck_ref = None
            self.last_phase = None
            return
        if phase in self.DOCK_PHASES:
            self.dock_stuck += 1
            if self.dock_stuck > 4:
                # 反复脱困还是卡在回桩路上：这一轮先放弃回桩，回去继续手上的活，
                # 否则会一直"后退→再撞→再后退"，把电量耗干停在场地上。
                self._give_up_docking(self.route_kind or "to_dock")
                return
            if self.dock_stuck > 2:
                # 连续脱困仍到不了桩：放开车体周边导航膨胀，给它一条出路再回桩
                released = self.grid.release_blocked_disc(self._pose()[0], self._pose()[1], 0.75)
                self._log(
                    "warn",
                    "回桩持续受阻，放开周边 {} 格膨胀后重新规划".format(released),
                )
            self.phase = phase
            self._set_route((world.DOCK["x"], world.DOCK["y"]), self.route_kind or "to_dock")
            self.stuck_time = 0.0
            self.stuck_ref = None
            self.stuck_spin = 0.0
            self.last_phase = None
            return
        if phase == "no_go_ring":
            self._finish_no_go_ring()
        elif phase == "ring":
            self.ring_stuck += 1
            if self.ring_stuck <= 3:
                released = self.grid.release_blocked_disc(
                    self._pose()[0], self._pose()[1], 0.55
                )
                self._ring_push_out(0.12)
                self._ring_tight_target()
                self._log(
                    "info",
                    "绕障卡滞（第 {} 次），放开周边 {} 格膨胀、弧段外推后继续绕行".format(
                        self.ring_stuck, released
                    ),
                )
                return
            # 几何弧段确实走不通：改成贴面绕行，别直接把绕障判死
            self._ring_to_surface("卡滞 {} 次".format(self.ring_stuck))
        elif phase == "detour":
            self._finish_detour()
        elif phase == "zone_edge":
            self.edge_stuck += 1
            circle = self._edge_circle_from_occupancy()
            if circle is not None and not self._is_edge_ringed(circle):
                if self._begin_edge_ring(circle):
                    return
            if self.edge_stuck > 3:
                # 连续脱困都走不动：剩下的边缘别硬啃了，交给后面的弓字覆盖
                self._log("warn", "沿边连续受阻，跳过剩余边缘，交给弓字覆盖")
                self._finish_edge_phase()
                return
            self._advance_edge_after_collision(1.20)
        elif phase == "zone_stripes":
            if self.segment is not None:
                self.failed_segments.add(_segment_key(self.segment_plan or self.segment))
            self.phase = "zone_plan"
        elif phase == "to_zone":
            self._plan_to_zone_corner()
        self.stuck_time = 0.0
        self.stuck_ref = None
        self.stuck_heading = 0.0
        self.last_phase = None

    def _begin_escape(self, resume_phase):
        x, y, _ = self._pose()
        path = []
        length = 0.0
        last = (x, y)
        trail = self.zone_history if resume_phase in ZONE_TRAIL_PHASES else self.history
        if len(trail) < 4:
            trail = self.history
        for point in reversed(trail):
            distance = math.hypot(point[0] - last[0], point[1] - last[1])
            if distance < 0.04:
                continue
            path.append((point[0], point[1]))
            length += distance
            last = point
            if length >= ESCAPE_DISTANCE:
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
        if self.escape_index >= len(self.escape_path) or self.escape_time > ESCAPE_TIME_LIMIT:
            self._resume_after_escape()
            return 0.0, 0.0
        target = self.escape_path[self.escape_index]
        bearing = math.atan2(target[1] - y, target[0] - x)
        # 倒车时车头应朝向目标的反方向，再以负速度沿历史路径退回。
        desired = world.wrap_angle(bearing + math.pi)
        err = world.wrap_angle(desired - theta)
        w = world.clamp(1.8 * err, -1.5, 1.5)
        return ESCAPE_SPEED, w

    def _resume_after_escape(self):
        resume = self.escape_resume
        self.escape_path = []
        self.escape_index = 0
        self.escape_resume = None
        self.escape_time = 0.0
        # 倒车后的位姿与旧历史轨迹不连续，避免下一次脱困误用旧路径。
        self.history.clear()
        self.history.append(self._pose()[:2])
        if resume == "map_seek":
            self.phase = "map_seek"
            self.map_seek_goal = None
            self.map_seek_route = []
            self.map_seek_index = 0
            self.last_phase = None
            return
        if resume == "map_fill":
            # 补扫脱困回来：只重置当前这条线，剩下的扫描线继续扫
            self.map_fill_stage = "seek"
            self.map_fill_target = None
            self.route = []
            self.route_kind = None
            self.route_index = 0
            self.phase = "map_fill"
            self.last_phase = None
            return
        if resume == "map_follow":
            if self.mapping_done:
                self.phase = "to_dock"
                self._set_route((world.DOCK["x"], world.DOCK["y"]), "map_dock")
                self.last_phase = None
                return
            self.map_path = []
            self._map_hash = {}
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
        elif resume in self.DOCK_PHASES:
            self.phase = resume
            self._set_route((world.DOCK["x"], world.DOCK["y"]), self.route_kind or "to_dock")
        else:
            self.phase = "zone_plan"
        self.last_phase = None

    def _skip_segment(self):
        self._log("warn", "弓字线段起点不可达，跳过并重新规划剩余区域")
        if self.segment is not None:
            self.failed_segments.add(_segment_key(self.segment_plan or self.segment))
        self.segment_index += 1
        if self.segment_index < len(self.segments):
            self.phase = "to_segment"
            self._goto_segment(self.segment_index)
            self.last_phase = None
        else:
            self.phase = "zone_plan"
            self.last_phase = None

    def _return_reserve(self):
        """按当前位置到充电桩的直线距离，估算"开回去需要多少电"（百分比）。"""
        distance = getattr(self.robot, "dock_distance", 0.0)
        scale = robot_car.DRAIN_FACTOR / RETURN_SOC_BASE_DRAIN
        return RETURN_SOC_MARGIN + distance * RETURN_SOC_PER_METER * scale

    def _check_battery(self):
        if not self.task:
            return
        if self.phase in CHARGE_EXEMPT:
            return
        if self.charge_reason:
            return
        if self.robot.sim_time < self.charge_retry_time:
            return
        battery = self.robot.battery
        reserve = self._return_reserve()
        if battery > max(LOW_BATTERY, reserve):
            return
        self._save_breakpoint()
        self.charge_reason = "low"
        self.stats["charge_count"] += 1
        self.phase = "charge_go"
        if battery <= LOW_BATTERY:
            message = "电量低于 {:.0f}%，保存任务并自动返回充电桩".format(LOW_BATTERY)
        else:
            message = (
                "电量 {:.0f}% 仅够回桩（距桩 {:.0f} m 需预留 {:.0f}%），提前返回充电桩"
            ).format(battery, getattr(self.robot, "dock_distance", 0.0), reserve)
        self._log("warn", message)
        self._set_route((world.DOCK["x"], world.DOCK["y"]), "to_dock")

    # ----------------------------------------------------------- 控制分发
    def _control(self, dt, step):
        phase = self.phase
        if phase in ("map_seek", "map_follow", "map_escape") and self.mapping_done:
            # 建图已经完成后又被断点/脱困恢复到巡边阶段：直接回桩，不再重复建图。
            self._log("info", "建图已完成，结束残余巡边并返回充电桩")
            self.phase = "to_dock"
            self._set_route((world.DOCK["x"], world.DOCK["y"]), "map_dock")
            return 0.0, 0.0
        if phase in ("idle", "done"):
            return 0.0, 0.0
        if phase == "map_seek":
            return self._map_seek()
        if phase == "map_follow":
            return self._map_follow(dt)
        if phase == "map_escape":
            return self._map_escape(dt, step)
        if phase == "map_fill":
            return self._map_fill_step(dt)
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
        if phase == "no_go_ring":
            return self._no_go_ring_step()
        if phase in ("charge_go", "to_dock", "charge_return"):
            return self._nav_step(NAV_SPEED)
        if phase == "charging":
            if not self.robot.charging and self.charge_reason is not None:
                # 停在充电阶段却没充上电：离桩太远，重新靠桩。
                self.charge_wait = getattr(self, "charge_wait", 0.0) + dt
                if self.charge_wait >= 3.0:
                    self.charge_wait = 0.0
                    self._log("warn", "未停在充电桩上，重新靠桩")
                    self.phase = "to_dock"
                    self._set_route((world.DOCK["x"], world.DOCK["y"]), "to_dock")
                    return 0.0, 0.0
            else:
                self.charge_wait = 0.0
            return self._charging_step()
        if phase == "align":
            return self._align_step()
        return 0.0, 0.0

    # ----------------------------------------------------------- 建图
    def _map_gap_scan(self):
        """到了缺口观察点就原地扫视，把没扫到的墙面补上；3 s 后放手继续。"""
        goal = self.map_gap_goal
        if goal is None:
            return None
        x, y, theta = self._pose()
        now = self.robot.sim_time
        distance = math.hypot(goal[0] - x, goal[1] - y)
        if distance > 0.75:
            # 实景图动辄 30-40 m 宽，走到远处的缺口要几十秒，超时给足
            if now - self.map_gap_goal_time > MAP_GAP_REACH_TIME:
                self._log("warn", "补扫点 {:.1f} m 内没有开过去，跳过这处缺口".format(distance))
                self._map_gap_close_attempt()
            return None
        if self.map_gap_scan_time < 0.0:
            self.map_gap_scan_time = now
        if now - self.map_gap_scan_time > 3.0:
            self._map_gap_close_attempt()
            return None
        error = world.wrap_angle(math.atan2(goal[1] - y, goal[0] - x) - theta)
        if abs(error) < 0.06:
            # 正对墙面就看四周，让 100° 视场扫过整段缺口
            return 0.0, 0.55
        return 0.0, world.clamp(1.3 * error, -1.2, 1.2)

    def _map_gap_close_attempt(self):
        """收尾一次补扫：没补上就把这处缺口记进黑名单，别再来回跑。"""
        discovery = self.grid.boundary_discovery()
        if self.map_gap_point is not None and discovery - self.map_gap_baseline < MAP_GAP_MIN_GAIN:
            self.map_gap_waste += 1
            self.map_gap_abandoned.append(self.map_gap_point)
            self._log(
                "info",
                "补扫没有新增边界（{:.0f}% → {:.0f}%），这处缺口先放过".format(
                    self.map_gap_baseline * 100.0, discovery * 100.0
                ),
            )
        self.map_gap_goal = None
        self.map_gap_point = None
        self.map_gap_scan_time = -1.0
        self.map_gap_reach = {}
        self.map_seek_mode = False
        self.map_progress_time = self.robot.sim_time
        self.map_last_discovery = discovery

    def _map_seek(self):
        scan = self._map_gap_scan()
        if scan is not None:
            return scan
        x, y, theta = self._pose()
        if self.map_seek_mode:
            if self.robot.sim_time - self.map_seek_mode_time > 90.0:
                self.map_seek_mode = False
                self._log("info", "探索超时，恢复贴边巡边")
            else:
                route = self._map_seek_route()
                controls = self._follow_seek_route(route) if route else None
                if controls is not None:
                    return controls
                if not self.map_seek_route:
                    # 探索模式下先离开当前小区域，不立刻又贴回同一段墙
                    nearest = _front_distance(self.robot.vision, 20.0)
                    if nearest is not None and nearest[0] < 0.65:
                        steer = -1.3 if nearest[1] >= 0.0 else 1.3
                        speed = 0.0 if nearest[0] < 0.36 else 0.16
                        return speed, steer
                    return MAP_SPEED, 0.0
        nearest = _front_distance(self.robot.vision, 20.0)
        near_boundary = nearest is not None and nearest[0] < 0.85 and self._hit_is_outer_wall(
            nearest, x, y, theta
        )
        if near_boundary:
            self.phase = "map_follow"
            self.map_path = [(x, y)]
            self.map_path_dist = 0.0
            self.map_seek_mode = False
            self.map_seek_route = []
            self.map_seek_goal = None
            self.map_progress_time = self.robot.sim_time
            self.map_last_discovery = self.grid.boundary_discovery()
            self.map_side_flips = 0
            self._log("event", "视觉发现边界，开始巡边建图")
            return self._map_follow(0.02)
        if nearest is not None and nearest[0] < 0.85:
            # 场内障碍物：先绕开，避免把孤岛当作场地边界
            hit_x = x + math.cos(theta + nearest[1]) * nearest[0]
            hit_y = y + math.sin(theta + nearest[1]) * nearest[0]
            if self._near_known_island(hit_x, hit_y):
                return 0.12, 1.3
            if self.robot.sim_time - self.map_seek_log_time > 2.0:
                self.map_seek_log_time = self.robot.sim_time
                self._log("info", "视觉命中场内障碍物，绕开后继续寻找外墙")
            steer = -1.0 if nearest[1] >= 0.0 else 1.0
            return 0.26, steer
        return self._seek_forward()

    def _seek_forward(self):
        """找边界时的直行：靠视觉近距保护减速转向，避免贴着实景墙蹭出碰撞。"""
        readings = self.robot.vision
        if not readings:
            return MAP_SPEED, 0.0
        relative, distance = min(readings, key=lambda item: item[1])
        if distance < 0.48:
            steer = -1.4 if relative >= 0.0 else 1.4
            return 0.0, steer
        if distance < 0.72 and abs(relative) < math.radians(65.0):
            steer = -0.9 if relative >= 0.0 else 0.9
            return 0.18, steer
        return MAP_SPEED, 0.0

    def _hit_is_outer_wall(self, nearest, x, y, theta):
        hit_x = x + math.cos(theta + nearest[1]) * nearest[0]
        hit_y = y + math.sin(theta + nearest[1]) * nearest[0]
        return world.boundary_distance(hit_x, hit_y) <= 0.42

    def _map_seek_route(self):
        """给"寻找未探明边界"阶段算一条能真正走出当前小区域的路线。

        建图时栅格上的导航膨胀可能把门口糊死，所以这里忽略膨胀层（只尊重
        已探明占据格）规划，靠视觉前向保护避免真的撞墙。
        """
        if self.map_seek_route and self.map_seek_index < len(self.map_seek_route):
            return self.map_seek_route
        now = self.robot.sim_time
        if now - self.map_seek_goal_time < 1.5:
            return []
        self.map_seek_goal_time = now
        goal = self._map_seek_target()
        if goal is None:
            # 没有可去的前沿了：再确认一次有没有残留缺口，都补齐才收工
            self.map_seek_route = []
            if self._mapping_complete_enough():
                self._finish_mapping()
                return []
            goal = self._map_seek_target()
            if goal is None:
                return []
        path = self.grid.plan(
            self._pose()[:2], goal, ignore_blocked=True, time_budget=MAP_PLAN_BUDGET
        )
        if not path:
            # A* 完全找不到路时不要直接朝目标怼过去（大场地里常是一头撞墙原地打转），
            # 先沿自己走过的轨迹退出去，走到哪儿再从长计议。
            path = self._history_route(goal)
            if path:
                self.grid.relax_trail(path, 0.30)
                self._log("warn", "去未探明边界没有可行路线，先沿历史轨迹退回")
        if not path:
            path = self.grid.plan(
                self._pose()[:2],
                goal,
                ignore_blocked=True,
                time_budget=MAP_GAP_PLAN_BUDGET,
            )
        if not path:
            path = [goal]
        self.map_seek_route = path
        self.map_seek_index = 0
        self.map_seek_best = None
        self.map_seek_progress_time = now
        return self.map_seek_route

    def _follow_seek_route(self, route):
        x, y, theta = self._pose()
        goal = route[-1]
        if math.hypot(goal[0] - x, goal[1] - y) < 0.9:
            # 到达未探明边界附近：交回贴边逻辑，由视觉去找真正的边界
            self.map_seek_mode = False
            self.map_seek_route = []
            self.map_seek_goal = None
            return None
        while self.map_seek_index < len(route) - 1:
            point = route[self.map_seek_index]
            if math.hypot(point[0] - x, point[1] - y) > 0.45:
                break
            self.map_seek_index += 1
        target = route[min(self.map_seek_index + 2, len(route) - 1)]
        now = self.robot.sim_time
        err = world.wrap_angle(math.atan2(target[1] - y, target[0] - x) - theta)
        front = _front_distance(self.robot.vision, 30.0)
        if abs(err) > 0.9:
            # 原地掉头时位置几乎不动，不能当成"推不动"——否则一转身就被判卡死
            self.map_seek_last_pose = None
            self.map_seek_move_time = now
            return 0.0, world.clamp(2.2 * err, -1.4, 1.4)
        if front is not None and front[0] < 0.42:
            self.map_seek_last_pose = None
            self.map_seek_move_time = now
            steer = -1.3 if front[1] >= 0.0 else 1.3
            return 0.12, steer
        if self.map_seek_last_pose is None:
            self.map_seek_last_pose = (x, y)
            self.map_seek_move_time = now
        elif math.hypot(x - self.map_seek_last_pose[0], y - self.map_seek_last_pose[1]) > 0.08:
            self.map_seek_last_pose = (x, y)
            self.map_seek_move_time = now
        elif now - self.map_seek_move_time > 3.0:
            # 顶住墙角推不动：跳过几个路径点换个方向，实在不行就把这个目标拉黑
            self.map_seek_last_pose = None
            self.map_seek_move_time = now
            self.map_seek_index += 4
            if self.map_seek_index >= len(route) - 1:
                self._blacklist_seek_goal(route[-1])
                self._log("warn", "前往未探明边界的路线走不通，换一个方向探索")
            return 0.0, 1.2
        speed = MAP_SPEED * max(0.4, 1.0 - abs(err) / 1.8)
        return speed, world.clamp(2.2 * err, -1.4, 1.4)

    def _blacklist_seek_goal(self, goal):
        self.map_seek_blacklist.append((goal[0], goal[1]))
        if len(self.map_seek_blacklist) > 12:
            del self.map_seek_blacklist[0]
        self.map_seek_goal = None
        self.map_seek_route = []
        self.map_seek_index = 0
        self.map_seek_last_pose = None
        if self.map_gap_goal is not None and math.hypot(
            goal[0] - self.map_gap_goal[0], goal[1] - self.map_gap_goal[1]
        ) < 1.5:
            # 补扫点本身就开不过去：收尾这次补扫，别在同一处死循环
            self._map_gap_close_attempt()

    def _seek_goal_blocked(self, point):
        for bx, by in self.map_seek_blacklist:
            if math.hypot(point[0] - bx, point[1] - by) < 2.0:
                return True
        return False

    def _map_seek_target(self):
        """每 2 s 更新一次“最远的未探明前沿”，让机器人真的离开困境小区域。"""
        now = self.robot.sim_time
        if self.map_seek_goal is not None and now - self.map_seek_goal_time < 2.0:
            x, y = self._pose()[0], self._pose()[1]
            if math.hypot(self.map_seek_goal[0] - x, self.map_seek_goal[1] - y) > 0.8:
                return self.map_seek_goal
        self.map_seek_goal_time = now
        if self.map_gap_goal is not None:
            # 有明确缺口时先去缺口内侧观察，比盲目找前沿更快
            goal = self.map_gap_goal
            self.map_seek_goal = goal
            return goal
        goal = self.grid.explore_goal(self._pose()[:2])
        if goal is not None and (
            self._near_known_island(goal[0], goal[1]) or self._seek_goal_blocked(goal)
        ):
            goal = None
        self.map_seek_goal = goal
        return goal

    def _map_follow(self, dt):
        # 实景地图（顶点多、锯齿边界）需要更大的贴墙距离，否则容易擦墙。
        if len(world.POLYGON) > 40:
            # 实景地图（多房间、直角多）前向余量放大，避免贴墙巡边时蹭墙
            v, w = self._follow_surface(MAP_SPEED, 0.55, front_limit=0.46)
        else:
            v, w = self._follow_surface(MAP_SPEED, 0.45)
        x, y, _ = self._pose()
        if not self.map_path:
            self._append_map_point(x, y)
        else:
            last_x, last_y = self.map_path[-1]
            moved = math.hypot(x - last_x, y - last_y)
            if moved > 0.08:
                self._append_map_point(x, y)
                self.map_path_dist += moved
                if len(self.map_path) > 4000:
                    del self.map_path[:1000]
                    self._rebuild_map_hash()

        if self._mapping_done_now():
            # 边界基本探明、残留缺口也很小：不必再绕一圈
            self._finish_mapping()
            return 0.0, 0.0
        result = self._check_mapping_loop()
        if result is not None:
            return 0.0, 0.0
        if self._mapping_stalled():
            return 0.0, 0.0
        return v, w

    def _gap_viewpoint(self, gap):
        """缺口对应的观察点：缺口内侧、机器人真的开得过去的一个点。

        先按栅格问一次"从当前位置到缺口最近能到哪"：能到 1.2 m 以内就直接用那个
        落点（窄道被膨胀糊死时缺口本身常常不可达）；实在到不了再看缺口往场地
        内侧退 0.55 m 的位置。两者都很远就说明这处缺口现在补不上。

        长缺口只试离机器人最近的两段：整段走过去太远，分段补扫也更容易补上。
        """
        cx = gap.get("x", 0.0)
        cy = gap.get("y", 0.0)
        key = (int(cx / 2.0), int(cy / 2.0))
        cached = self.map_gap_reach.get(key)
        if cached is not None and self.robot.sim_time - cached[0] < 20.0:
            return cached[1]
        x, y = self._pose()[0], self._pose()[1]
        targets = [(cx, cy)]
        viewpoint = None
        for target in targets:
            path, distance = self._gap_route(target[0], target[1])
            if path and distance <= MAP_GAP_REACH:
                point = path[-1]
                if self._map_scan_point_ok(point[0], point[1]):
                    viewpoint = point
                    break
            candidate = self._gap_inner_point(target[0], target[1])
            if candidate is None:
                continue
            if math.hypot(candidate[0] - x, candidate[1] - y) > 60.0:
                continue
            nearby, near_distance = self._gap_route(candidate[0], candidate[1])
            if nearby and near_distance <= MAP_GAP_REACH:
                viewpoint = nearby[-1]
                break
        self.map_gap_reach[key] = (self.robot.sim_time, viewpoint)
        return viewpoint

    def _gap_route(self, cx, cy):
        """从当前位置朝某个点探一次路：走到哪、还差多远。

        车体所在格子经常被自己的视觉膨胀糊住（``_nearest_open`` 找不到落点），
        那样任何规划都会瞬间返回不可达。车已经在的地方一定是能走的，所以先放开
        脚下这一小圈再问，避免把可达的缺口误判成"开不过去"。
        """
        x, y = self._pose()[0], self._pose()[1]
        for attempt in range(2):
            path, distance = self.grid.plan_near(
                (x, y),
                (cx, cy),
                max_expansions=150000,
                time_budget=MAP_GAP_PLAN_BUDGET,
            )
            if path:
                return path, distance
            if attempt == 0:
                self.grid.release_blocked_disc(x, y, 0.35)
        return path, distance

    def _gap_inner_point(self, cx, cy):
        """缺口往场地内侧退一点的位置：贴着墙站才能让 1 m 视场扫到整段墙。"""
        poly = world.POLYGON
        if poly:
            mx = sum(point[0] for point in poly) / float(len(poly))
            my = sum(point[1] for point in poly) / float(len(poly))
        else:
            mx, my = world.WIDTH * 0.5, world.HEIGHT * 0.5
        dx, dy = mx - cx, my - cy
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return None
        point = (cx + dx / length * MAP_GAP_VIEW_OFFSET, cy + dy / length * MAP_GAP_VIEW_OFFSET)
        return point if self._map_scan_point_ok(point[0], point[1]) else None

    def _map_scan_point_ok(self, x, y):
        """观察点必须落在场地内、且不贴墙，否则车开不过去。"""
        if not world.point_in_polygon(x, y):
            return False
        return world.boundary_distance(x, y) >= 0.25

    def _gap_abandoned(self, gap):
        """这处缺口白跑过一次就先放过，先把别的缺口补完再回头。"""
        for x, y in self.map_gap_abandoned:
            if math.hypot(gap.get("x", 0.0) - x, gap.get("y", 0.0) - y) <= MAP_GAP_AVOID_RADIUS:
                return True
        return False

    def _map_gap_give_up(self, discovery, reason):
        """剩下的缺口确实补不上了：要么继续找边界，要么明确收尾。"""
        if discovery < MAP_DONE_DISCOVERY and self.map_recover_total < MAP_RECOVER_LIMIT:
            # 边界才探明一半就收工显然不对：多半是车被自己围在某个角落了。
            # 放开周边膨胀、重新去找未探明边界，别急着宣布"建图完成"。
            self.map_recover_total += 1
            self.map_gap_unreachable_time = None
            self.map_gap_reach = {}
            self.map_gap_attempts = 0
            self.map_gap_waste = 0
            self.map_gap_abandoned = []
            x, y = self._pose()[0], self._pose()[1]
            released = self.grid.release_blocked_disc(x, y, 1.6)
            self.map_seek_mode = True
            self.map_seek_mode_time = self.robot.sim_time
            self.map_seek_goal = None
            self.map_seek_route = []
            self.map_progress_time = self.robot.sim_time
            self.last_phase = None
            self._log(
                "warn",
                "边界才探明 {:.0f}%（{}），放开周边 {} 格膨胀继续找边界".format(
                    discovery * 100.0, reason, released
                ),
            )
            return False
        self.map_gap_done = True
        self.map_gap_goal = None
        self.map_gap_point = None
        self.map_gap_scan_time = -1.0
        self.map_gap_remaining = 0.0
        for gap in self.grid.boundary_gaps():
            self.map_gap_remaining = max(self.map_gap_remaining, gap["run_m"])
        if discovery < MAP_DONE_DISCOVERY_NOW:
            self._log(
                "warn",
                "建图收尾：边界探明 {:.0f}%，{}（最大残留 {:.1f} m）".format(
                    discovery * 100.0, reason, self.map_gap_remaining
                ),
            )
        return True

    def _mapping_complete_enough(self):
        """建图能不能收工：边界探明率达标，且没有还补得上的明显缺口。

        只看全局百分比不够——1 m 视场下某段 5 m 的墙完全没扫到，百分比也可能有 85%。
        所以再按"未探明墙格聚簇"找缺口：能开过去的就去补扫，开不过去或补了几次
        都补不上的就记下来，避免在不可达的缺口上无限绕、建图永远完不成。
        """
        discovery = self.grid.boundary_discovery()
        if (
            discovery >= MAP_DONE_DISCOVERY_NOW
            and self._map_residual_length() <= MAP_DONE_RESIDUAL
        ):
            return True
        if self.map_gap_done:
            return True
        if self.map_gap_attempts >= MAP_GAP_ATTEMPTS:
            return self._map_gap_give_up(discovery, "已补扫 {} 次".format(self.map_gap_attempts))
        if self.map_gap_waste >= MAP_GAP_WASTE_LIMIT:
            return self._map_gap_give_up(discovery, "补扫没有新增边界")
        pending = []
        for gap in self.grid.boundary_gaps():
            if gap["run_m"] < MAP_GAP_MAX_RUN or self._gap_abandoned(gap):
                continue
            viewpoint = self._gap_viewpoint(gap)
            if viewpoint is not None:
                pending.append((gap, viewpoint))
        if not pending:
            if discovery >= MAP_DONE_DISCOVERY:
                return True
            # 判"开不过去"要连着确认一段时间：机器人可能正好卡在小区域里，
            # 一瞬的不可达不等于真的补不上。
            now = self.robot.sim_time
            if self.map_gap_unreachable_time is None:
                self.map_gap_unreachable_time = now
                return False
            if now - self.map_gap_unreachable_time < 15.0:
                return False
            return self._map_gap_give_up(discovery, "剩下的缺口都开不过去")
        self.map_gap_unreachable_time = None
        gap, viewpoint = pending[0]
        self.map_gap_attempts += 1
        self.map_gap_point = (gap.get("x", 0.0), gap.get("y", 0.0))
        self.map_gap_baseline = discovery
        self.map_gap_goal = viewpoint
        self.map_gap_goal_time = self.robot.sim_time
        self.map_gap_scan_time = -1.0
        self.map_seek_goal = viewpoint
        self.map_seek_route = []
        self.map_seek_mode = True
        self.map_seek_mode_time = self.robot.sim_time
        self._log(
            "info",
            "发现 {:.1f} m 未探明边界，前往补扫（第 {} 次）".format(
                gap["run_m"], self.map_gap_attempts
            ),
        )
        return False

    def _map_residual_length(self):
        """还没探明、且跨度够长的墙面总长（m），用来判断"最后那一点点"值不值得再绕一圈。"""
        version = self.grid.data_version
        cache = self._map_residual_cache
        if cache is not None and cache[0] == version:
            return cache[1]
        total = 0.0
        for gap in self.grid.boundary_gaps(max_gaps=12):
            if gap["run_m"] >= MAP_GAP_MAX_RUN:
                total += gap["run_m"]
        self._map_residual_cache = (version, total)
        return total

    def _mapping_done_now(self):
        """巡边途中能不能立刻收尾。

        只看百分比会在"最后差半米墙"时提前回桩（地图边上留一道没喷涂的口子），
        所以要求残留缺口也很小；绕满 `MAP_CLOSE_MAX_LAPS` 圈还是差就认了。
        """
        if self.grid.boundary_discovery() < MAP_DONE_DISCOVERY_NOW:
            return False
        if self._map_residual_length() <= MAP_DONE_RESIDUAL:
            return True
        return self.map_loop_count >= MAP_CLOSE_MAX_LAPS

    def _mapping_stalled(self):
        """巡边在小区域里来回蹭、边界进度不再明显增长时，改为主动去找未探明边界。"""
        discovery = self.grid.boundary_discovery()
        if discovery > self.map_last_discovery + 0.01:
            self.map_last_discovery = discovery
            self.map_progress_time = self.robot.sim_time
            return False
        if self.robot.sim_time - self.map_progress_time < MAP_STALL_SECONDS:
            return False
        segment = self.map_path[-240:] if len(self.map_path) > 240 else list(self.map_path)
        if len(segment) > 20:
            xs = [point[0] for point in segment]
            ys = [point[1] for point in segment]
            cx = sum(xs) / len(xs)
            cy = sum(ys) / len(ys)
            radius = max(math.hypot(px - cx, py - cy) for px, py in segment) + 0.25
            self.map_islands.append(
                {"x": round(cx, 2), "y": round(cy, 2), "r": round(radius, 2)}
            )
        pose = self._pose()
        released = self.grid.release_blocked_disc(pose[0], pose[1], 1.6)
        self.map_progress_time = self.robot.sim_time
        self.map_last_discovery = discovery
        self.map_path = []
        self._map_hash = {}
        self.map_path_dist = 0.0
        self.map_seek_goal = None
        self.map_seek_route = []
        self.map_seek_mode = True
        self.map_seek_mode_time = self.robot.sim_time
        self.phase = "map_seek"
        self.last_phase = None
        self._log(
            "warn",
            "巡边在 {:.0f}% 边界后长时间没有新进展（困在小区域），放开 {} 格膨胀改为寻找未探明边界".format(
                discovery * 100.0, released
            ),
        )
        return True

    MAP_HASH_CELL = 0.3

    def _append_map_point(self, x, y):
        index = len(self.map_path)
        self.map_path.append((x, y))
        key = (int(x / self.MAP_HASH_CELL), int(y / self.MAP_HASH_CELL))
        bucket = self._map_hash.get(key)
        if bucket is None:
            self._map_hash[key] = [index]
        else:
            bucket.append(index)

    def _rebuild_map_hash(self):
        self._map_hash = {}
        for index, point in enumerate(self.map_path):
            key = (
                int(point[0] / self.MAP_HASH_CELL),
                int(point[1] / self.MAP_HASH_CELL),
            )
            bucket = self._map_hash.get(key)
            if bucket is None:
                self._map_hash[key] = [index]
            else:
                bucket.append(index)

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
            discovery = self.grid.boundary_discovery()
            if area >= MIN_OUTER_AREA:
                self.map_loop_count += 1
                if self._mapping_done_now():
                    self._finish_mapping()
                    return "complete"
                if discovery >= MAP_DONE_DISCOVERY and self._mapping_complete_enough():
                    self._finish_mapping()
                    return "complete"
                if self.map_gap_goal is not None:
                    # 补扫那边已经指好观察点了：交给寻找未探明边界阶段
                    self.phase = "map_seek"
                    self.map_seek_mode = True
                    self.map_seek_route = []
                    return "gap"
                residual = self._map_residual_length()
                self._log(
                    "info",
                    "已绕完一圈，边界还差 {:.1f} m 没喷涂，沿墙再走一圈补齐（第 {} 圈）".format(
                        residual, self.map_loop_count
                    ),
                )
                self.map_path = [(x, y)]
                self._map_hash = {}
                self.map_path_dist = 0.0
                self.map_progress_time = self.robot.sim_time
                self.map_last_discovery = discovery
                self.map_seek_mode = False
                return "relap"
            # 闭环了但面积不够：说明只是在一个小区域里绕了一圈
            self._escape_island(path)
            return "island"

        bx = int(x / self.MAP_HASH_CELL)
        by = int(y / self.MAP_HASH_CELL)
        candidates = []
        for dc in (-1, 0, 1):
            for dr in (-1, 0, 1):
                bucket = self._map_hash.get((bx + dc, by + dr))
                if bucket:
                    candidates.extend(bucket)
        candidates.sort()
        limit = len(path) - 20
        for index in candidates:
            if index >= limit:
                continue
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
        self._map_hash = {}
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
            # 脱离方向上顶着墙：原地朝远离墙的一侧转，别硬往前挤
            v = 0.0 if front[0] < 0.32 else 0.10
            w = 1.4 if front[1] >= 0.0 else -1.4
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
        if self.mapping_done:
            return
        if self._start_map_fill():
            # 边界还没探全（实景大图的内部隔墙也算边界）：开进去补看一轮再收工
            return
        self.mapping_done = True
        # 建图结束后不再保留巡边进度，避免之后回充/恢复任务时又回到巡边阶段。
        self.map_path = []
        self._map_hash = {}
        self.map_path_dist = 0.0
        self.map_islands = []
        if self.breakpoint and str(self.breakpoint.get("phase", "")).startswith("map"):
            self.breakpoint = None
        self.map_gap_remaining = max(
            [gap["run_m"] for gap in self.grid.boundary_gaps()] or [0.0]
        )
        # 先回充电桩，到桩停稳之后再分区：分区要用到"停在哪"这个信息，
        # 而且回桩路上还在继续扫描，等停稳再切区域边界更准。
        self._log(
            "event",
            "建图完成：边界探明 {:.0f}%，返回充电桩，到桩后自动分区".format(
                self.grid.boundary_discovery() * 100.0
            ),
        )
        # 建图完成时墙体膨胀会把桩位/窄道堵死，放开桩周与机器人脚下。
        self.grid.relax_around(
            world.DOCK["x"], world.DOCK["y"], world.DOCK["r"] + 0.25, clear_occupied=True
        )
        pose = self._pose()
        self.grid.relax_around(pose[0], pose[1], 0.45)
        self.phase = "to_dock"
        self._set_route((world.DOCK["x"], world.DOCK["y"]), "map_dock")

    def _unknown_inside_area(self):
        """场地内还没建过图的面积（m²）。"""
        inside = world.polygon_mask(self.grid.cols, self.grid.rows, self.grid.cell)
        unknown = 0
        for index in range(len(inside)):
            if inside[index] and self.grid.data[index] == 0:
                unknown += 1
        return unknown * self.grid.cell * self.grid.cell

    def _unknown_spots(self, min_area=2.0, limit=16):
        """还没建过图的连通块质心（大块优先），用来"开过去看一眼"。"""
        grid = self.grid
        cols, rows = grid.cols, grid.rows
        inside = world.polygon_mask(cols, rows, grid.cell)
        data = grid.data
        seen = bytearray(cols * rows)
        spots = []
        for index in range(cols * rows):
            if seen[index] or not inside[index] or data[index] != 0:
                continue
            stack = [index]
            seen[index] = 1
            cells = []
            while stack:
                cur = stack.pop()
                cells.append(cur)
                col, row = cur % cols, cur // cols
                for dc in (-1, 0, 1):
                    for dr in (-1, 0, 1):
                        nc, nr = col + dc, row + dr
                        if not (0 <= nc < cols and 0 <= nr < rows):
                            continue
                        nindex = nr * cols + nc
                        if seen[nindex] or not inside[nindex] or data[nindex] != 0:
                            continue
                        seen[nindex] = 1
                        stack.append(nindex)
            if len(cells) * grid.cell * grid.cell < min_area:
                continue
            cx = sum((c % cols + 0.5) * grid.cell for c in cells) / len(cells)
            cy = sum((c // cols + 0.5) * grid.cell for c in cells) / len(cells)
            spots.append((len(cells), cx, cy))
        spots.sort(key=lambda item: -item[0])
        x, y = self._pose()[:2]
        result = []
        for _size, cx, cy in spots:
            # 自己脚下/去过的点再去一次毫无意义，只会把轮次烧完
            if math.hypot(cx - x, cy - y) < MAP_FILL_SPOT_RADIUS:
                continue
            if any(
                math.hypot(cx - px, cy - py) < MAP_FILL_SPOT_RADIUS
                for px, py in self.map_fill_visited
            ):
                continue
            result.append((round(cx, 2), round(cy, 2)))
            if len(result) >= limit:
                break
        return result

    def _start_map_fill(self):
        """边界还没探全时才兜底扫场内未知区。

        内置场地绕完外墙边界就是 100%，这里直接返回 False，不会多跑一趟；
        实景大图里"分区墙"也算边界，只绕外轮廓永远探不到，才需要开进去看。
        """
        if self.grid.boundary_discovery() >= MAP_FILL_TRIGGER:
            return False
        if self.map_fill_passes >= MAP_FILL_MAX_PASSES:
            return False
        if self.map_fill_start_sim is not None:
            if self.robot.sim_time - self.map_fill_start_sim > MAP_FILL_TIME_LIMIT:
                self._log("warn", "建图兜底扫描超时，结束扫描")
                return False
        area = self._unknown_inside_area()
        if area < MAP_FILL_MIN_AREA:
            return False
        # 上一轮几乎没扫掉什么（扫描线开不过去），改成直接去大块未知区中心看一眼
        stalled = (
            self.map_fill_last_area is not None
            and self.map_fill_last_area - area < max(2.0, self.map_fill_last_area * 0.05)
        )
        self.map_fill_last_area = area
        segments = []
        if not stalled:
            segments = planner.unknown_lane_segments(
                self.grid,
                spacing=MAP_FILL_SPACING,
                axis="h" if self.map_fill_passes % 2 == 0 else "v",
            )
        if not segments:
            spots = self._unknown_spots()
            segments = [((x, y), (x, y)) for x, y in spots]
        if not segments:
            return False
        self.map_fill_passes += 1
        if self.map_fill_start_sim is None:
            self.map_fill_start_sim = self.robot.sim_time
        self.map_fill_segments = list(segments)
        self.map_fill_target = None
        self.map_fill_stage = "seek"
        self.map_path = []
        self._map_hash = {}
        self.map_path_dist = 0.0
        self.route = []
        self.route_kind = None
        self.route_index = 0
        self.phase = "map_fill"
        self.last_phase = None
        self._log(
            "event",
            "边界探明 {:.0f}%，还有 {:.0f} m² 没建过图，开进去补看（第 {} 轮，{} 条扫描线）".format(
                self.grid.boundary_discovery() * 100.0,
                area,
                self.map_fill_passes,
                len(segments),
            ),
        )
        return True

    def _map_fill_step(self, dt):
        v, w = self._map_fill_command()
        # 扫描线常常正对墙面：命令前进却原地不动就换下一条，别一边撞一边磨
        x, y = self._pose()[:2]
        if v > 0.05 and self.map_fill_cmd_v > 0.05:
            progress = self.map_fill_progress
            if progress is None:
                self.map_fill_progress = [self.robot.sim_time, x, y, 0.0]
            elif math.hypot(x - progress[1], y - progress[2]) >= MAP_FILL_STALL_DISTANCE:
                progress[0], progress[1], progress[2], progress[3] = (
                    self.robot.sim_time,
                    x,
                    y,
                    0.0,
                )
            else:
                progress[3] += dt
                if progress[3] >= MAP_FILL_STALL_SECONDS:
                    self.map_fill_progress = None
                    self.map_fill_cmd_v = v
                    self._log("warn", "补看这条扫描线开不过去，跳过")
                    self._drop_map_fill_segment()
                    return 0.0, 0.0
        else:
            self.map_fill_progress = None
        self.map_fill_cmd_v = v
        return v, w

    def _drop_map_fill_segment(self):
        if self.map_fill_segments:
            segment = self.map_fill_segments.pop(0)
            if math.hypot(
                segment[0][0] - segment[1][0], segment[0][1] - segment[1][1]
            ) < 0.2:
                self.map_fill_visited.append(segment[0])
                self.map_fill_visited = self.map_fill_visited[-24:]
        self.map_fill_target = None
        self.map_fill_stage = "seek"
        self.route = []
        self.route_kind = None
        self.route_index = 0

    def _map_fill_command(self):
        x, y, theta = self._pose()
        if self.map_fill_stage == "seek":
            if not self.map_fill_segments:
                if self._start_map_fill():
                    return 0.0, 0.0
                self._finish_mapping()
                return 0.0, 0.0
            a, b = self.map_fill_segments[0]
            self.map_fill_target = b
            if math.hypot(a[0] - x, a[1] - y) < 0.8:
                self.map_fill_stage = "along"
                self.route = []
                self.route_index = 0
                return 0.0, 0.0
            if self.route and self.route_kind == "map_fill":
                return self._nav_step(MAP_FILL_SPEED)
            if self._set_route(a, "map_fill"):
                self.phase = "map_fill"
                return self._nav_step(MAP_FILL_SPEED)
            # 规划失败时 _handle_unreachable("map_fill") 已经丢掉这条线了
            return 0.0, 0.0
        target = self.map_fill_target
        if target is None or math.hypot(target[0] - x, target[1] - y) < 0.35:
            self._drop_map_fill_segment()
            return 0.0, 0.0
        err = world.wrap_angle(math.atan2(target[1] - y, target[0] - x) - theta)
        if abs(err) > 1.3:
            return 0.0, world.clamp(2.0 * err, -1.5, 1.5)
        v = MAP_FILL_SPEED * max(0.25, 1.0 - abs(err) / 1.5)
        w = world.clamp(2.2 * err, -1.5, 1.5)
        return self._wall_guard(v, w)

    def _partition_after_docking(self):
        """停稳在充电桩上之后再划分割草区域。"""
        if self.zones_ready:
            return
        self.grid.build_free_area(self._pose()[:2])
        zones = self.grid.partition_zones()
        self.grid.apply_no_go(world.NO_GO)
        self.grid.capture_obstacle_baseline()
        self.zones_ready = True
        if self.map_gap_remaining >= MAP_GAP_MAX_RUN:
            # 还差一截边界确实开不过去：把话说明白，别让人以为地图建全了
            self._log(
                "event",
                "建图完成：边界探明 {:.0f}%，约 {:.1f} m 边界开不过去没补上；已自动划分 {} 个割草区域".format(
                    self.grid.boundary_discovery() * 100.0,
                    self.map_gap_remaining,
                    len(zones),
                ),
            )
        else:
            self._log(
                "event",
                "建图完成：已自动划分 {} 个割草区域".format(len(zones)),
            )

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
        near_path, _distance = self.grid.plan_near(
            (x, y),
            (zone["cx"], zone["cy"]),
            zone_id=zone["id"],
        )
        if near_path:
            self.nav_goal = near_path[-1]
            self.route_kind = "to_zone_stripes"
            self.route = near_path
            self.route_index = 1 if len(near_path) > 1 else 0
            self._log(
                "warn",
                "区域 {} 的沿边入口不可达，改从可达内部点开始弓字清扫".format(
                    self.zone_id
                ),
            )
            return
        if self.zone_backtracks < 2 and self._start_backtrack(
            (zone["cx"], zone["cy"]), "to_zone"
        ):
            # 入口全不可达常常是车自己钻进死角了：先原路退出去再规划
            self.zone_backtracks += 1
            return
        self._skip_unreachable_zone()

    def _zone_edge_points(self, zone):
        key = (zone["id"], self.grid.data_version)
        cached = self._edge_route_cache.get(key)
        if cached is not None:
            return list(cached)
        # 统一走「区域矩形框 ∩ 建图自由区 → 腐蚀 → 轮廓」，规则/不规则区域同一套规则
        points = planner.edge_route_mask(self.grid, zone, self._pose()[:2])
        if len(points) >= 3 and _polygon_area(points) < 0.0:
            points = list(reversed(points))
        points = self._combine_edge_bounds(points, zone)
        self._edge_route_cache[key] = list(points)
        return points

    def _combine_edge_bounds(self, points, zone):
        """沿边路径同时满足“区域范围”和“地图边界”两个约束。

        - 区域范围：只用区域外接框（外扩 0.5 m）内的点，避免沿到相邻区域；
        - 地图边界：离场地外墙不足 车体半径 + 0.10 m 的点，沿指向区域中心的方向内移；
        - 禁区：直接剔除（A 模式会另外逆时针绕行禁区）。
        """
        if not points:
            return []
        # 沿边要贴着墙割：余量只留车体半径 + 5 cm（以前是 +10 cm，再加沿边
        # 内缩 0.22 m，刀盘离墙还有 20 多厘米，墙边一整条割不到）
        margin = world.ROBOT_RADIUS + 0.05
        x0 = zone["x0"] - 0.5
        x1 = zone["x1"] + 0.5
        y0 = zone["y0"] - 0.5
        y1 = zone["y1"] + 0.5
        cx = zone.get("cx", (zone["x0"] + zone["x1"]) * 0.5)
        cy = zone.get("cy", (zone["y0"] + zone["y1"]) * 0.5)
        result = []
        for point in points:
            x, y = point[0], point[1]
            if not (x0 <= x <= x1 and y0 <= y <= y1):
                continue
            if world.in_no_go(x, y, world.ROBOT_RADIUS * 0.5):
                continue
            clearance = world.boundary_distance(x, y)
            if clearance < margin:
                dx, dy = cx - x, cy - y
                length = math.hypot(dx, dy)
                if length < 1e-6:
                    continue
                shift = min(margin - clearance + 0.02, 0.45)
                x += dx / length * shift
                y += dy / length * shift
                if world.boundary_distance(x, y) < margin * 0.8:
                    continue
            if result and math.hypot(x - result[-1][0], y - result[-1][1]) < 0.04:
                continue
            result.append((x, y))
        return result

    def _record_zone_finish(self, zone_id):
        """把这个区域的割草数据存成快照（任务结束时写进报告）。"""
        if zone_id is None:
            return
        zone = self._zone(zone_id)
        cell = self.grid.cell
        free = self.grid.zone_totals.get(zone_id, 0)
        mowed = self.grid.zone_mowed.get(zone_id, 0)
        mark = self.zone_stats_mark or {}
        self.mow_zone_records.append(
            {
                "id": zone_id,
                "name": (zone or {}).get("name") or "区域 {}".format(zone_id),
                "area": round(free * cell * cell, 2),
                "mowed_area": round(mowed * cell * cell, 2),
                "coverage": round(min(1.0, mowed / free) if free else 0.0, 4),
                "start_sim": round(self.zone_start_sim or 0.0, 1),
                "end_sim": round(self.robot.sim_time, 1),
                "seconds": round(
                    max(0.0, self.robot.sim_time - (self.zone_start_sim or 0.0)), 1
                ),
                "rings": self.stats.get("rings", 0) - mark.get("rings", 0),
                "edge_skips": self.stats.get("edge_skips", 0) - mark.get("edge_skips", 0),
                "segments_planned": self.stats.get("segments_planned", 0)
                - mark.get("segments_planned", 0),
                "lane_axis": self.main_lane_axis,
                "coverage_retries": self.coverage_retries,
            }
        )

    def _write_mow_report(self):
        """全部区域割完后，把这次任务的数据写成 data/data 下的报告文档。"""
        if self.mow_report is not None:
            return self.mow_report
        try:
            import mow_report
        except Exception as exc:  # pragma: no cover - 报告模块缺失不影响割草主流程
            self._log("warn", "割草报告模块不可用：{}".format(exc))
            return None
        try:
            report = mow_report.generate(self)
        except Exception as exc:  # pragma: no cover - 写文件失败不该中断任务
            self._log("warn", "割草报告生成失败：{}".format(exc))
            return None
        self.mow_report = report
        self.mow_report_written = True
        folder = report.get("folder")
        self._log(
            "event",
            "割草报告已生成：{}（覆盖图 {}）".format(
                (folder + "/" + os.path.basename(report["html"]))
                if folder
                else os.path.basename(report["html"]),
                os.path.basename(report["png"]),
            ),
        )
        return report

    def _record_plan_edge(self, zone):
        """记下这个区域的沿边规划线（报告里画"规划路径"用）。"""
        if zone is None or not self.edge_points:
            return
        self.mow_plan_edges[zone["id"]] = [
            (round(point[0], 3), round(point[1], 3)) for point in self.edge_points
        ]

    def _record_plan_segments(self, segments):
        """记下规划过的弓字线段（去重，报告里画"规划路径"用）。"""
        if not segments:
            return
        for segment in segments:
            key = _segment_key(segment)
            if key in self.mow_plan_segments:
                continue
            a, b = segment
            self.mow_plan_segments[key] = (
                (round(a[0], 3), round(a[1], 3)),
                (round(b[0], 3), round(b[1], 3)),
            )

    def _plan_is_spinning(self, coverage_now):
        """重规划在原地打转时，沿来路退一段而不是把主方向的弓字线全标失败。

        返回 True 表示本帧已经处理过（调用方直接返回）。
        """
        pose = self._pose()
        if self.plan_anchor is None:
            moved = None
        else:
            moved = math.hypot(pose[0] - self.plan_anchor[0], pose[1] - self.plan_anchor[1])
        if (
            moved is None
            or moved > 0.30
            or coverage_now > self.last_plan_coverage + 0.005
        ):
            self.plan_anchor = (pose[0], pose[1])
            self.plan_stuck = 0
            return False
        self.plan_stuck += 1
        if self.plan_stuck < PLAN_STUCK_LIMIT:
            return False
        self.plan_stuck = 0
        self.plan_anchor = (pose[0], pose[1])
        if self.plan_stuck_backtracks >= PLAN_STUCK_BACKTRACKS:
            # 退不出来就老老实实往下走（交给后面的补割/漏扫兜底）
            self.failed_segments.clear()
            return False
        goal = None
        if self.segments:
            a, b = self.segments[0]
            goal = self._segment_entry(a, b) or a
        if goal is not None and self._start_backtrack(goal, "to_segment"):
            # 这些线段本身没走不通，清掉"失败"标记，退出后再按原方向重排
            self.failed_segments.clear()
            self.plan_stuck_backtracks += 1
            self.plan_anchor = None
            self._log("warn", "弓字起点附近被顶住，沿来路退回后按原方向重新规划")
            return True
        self.failed_segments.clear()
        return False

    def _preferred_lane_axis(self, zone):
        """整个区域的主弓字方向：沿着区域长边走。

        以前固定从横向开始，区域是"竖长"的时候横向弓字又短又多，割不完还得再
        转一遍纵向补割，同一片地上会先横后竖、看起来方向乱跳。现在进区域时按
        长边定一次方向，整个主覆盖阶段都用它（含绕障后的重规划）；只有漏扫补割
        可以换方向。
        """
        width = float(zone.get("x1", 0.0)) - float(zone.get("x0", 0.0))
        height = float(zone.get("y1", 0.0)) - float(zone.get("y0", 0.0))
        return "h" if width >= height else "v"

    def _node_lanes(self, zone, mode, axis=None):
        """区域内还没割到的弓字线段（蛇形顺序）。"""
        axis = axis or self.lane_axis
        return planner.lane_segments(self.grid, zone, axis=axis)

    def _reset_zone_progress(self):
        self.zone_history.clear()
        self.zone_history.append(self._pose()[:2])
        self.zone_backtracks = 0
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
        self.coverage_retry_best = 0.0
        self.failed_segments = set()
        self.lane_axis = "h"
        self.main_lane_axis = "h"
        self.rings_done = False
        self.edge_done = False
        self.segment_reversed = False
        self.dock_retries = 0
        self.charge_wait = 0.0

    def _enter_zone(self):
        zone = self._zone()
        if zone is None:
            self.phase = "zone_plan"
            return
        self._reset_zone_progress()
        self.lane_axis = self.main_lane_axis = self._preferred_lane_axis(zone)
        self.zone_start_sim = self.robot.sim_time
        self.zone_stats_mark = dict(self.stats)
        if self.mowing_mode == "B":
            self.zone_stage = "stripes"
            self.phase = "zone_plan"
            self._log("event", "进入区域 {}：B 模式，先规划弓字覆盖".format(self.zone_id))
        else:
            self.edge_points = self._zone_edge_points(zone)
            self._record_plan_edge(zone)
            self._prepare_edge_corners()
            self.zone_stage = "edge"
            self.edge_start_sim = self.robot.sim_time
            self.phase = "zone_edge"
            self._log("event", "进入区域 {}：先沿边缘割草一周".format(self.zone_id))

    def _enter_zone_stripes_only(self):
        zone = self._zone()
        if zone is None:
            self.phase = "zone_plan"
            return
        self._reset_zone_progress()
        self.lane_axis = self.main_lane_axis = self._preferred_lane_axis(zone)
        self.zone_start_sim = self.robot.sim_time
        self.zone_stats_mark = dict(self.stats)
        self.edge_done = True
        self.zone_stage = "stripes"
        self.phase = "zone_plan"
        self._log(
            "warn",
            "区域 {} 的沿边入口不可达，跳过沿边，清扫剩余可达弓字区域".format(
                self.zone_id
            ),
        )

    def _edge_time_limit(self):
        points = self.edge_points
        if not points:
            return 120.0
        key = (len(points), points[0], points[-1])
        if key != self._edge_len_key:
            self._edge_len_key = key
            self._edge_len_value = _path_length(points)
        expected = self._edge_len_value / max(0.20, EDGE_SPEED)
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
        blocked_zone = self._no_go_bypass_needed(target, "zone_edge")
        if blocked_zone is not None and self.mowing_mode == "A":
            if self._begin_no_go_ring(blocked_zone, "zone_edge"):
                return 0.0, 0.0
            self._advance_edge_past_no_go(blocked_zone)
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
        w = world.clamp(2.2 * err, -1.4, 1.4)
        if v == 0.0 and abs(err) < 2.5 and self.robot.vision:
            # 大角度转向时保留极低爬行速度，形成连续弧线而不是原地停顿。
            if min(distance for _, distance in self.robot.vision) > 0.45:
                v = 0.06
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
        # 至少跳过一小段：车顶在墙角时后面每个点都"够远"，一格格挪会挪几十分钟
        index = max(index, self.edge_index + EDGE_SKIP_MIN_POINTS)
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
        self.ring_bumps = 0
        self.ring_scan_count = 0
        self.ring_stuck = 0
        self.ring_probe = 0
        self.ring_origin = (x, y)
        self.ring_radius = self._ring_nominal_radius(points, circle)
        self.ring_time_limit = 20.0 + 0.7 * len(points)
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

    # 壁障保护只看前方 ±35°：侧向贴墙行驶时侧向射线本来就近，全视角取最小
    # 会让机器一贴墙就刹车/转向，速度掉到 0.1 m/s 都上不去。
    WALL_GUARD_SPAN = math.radians(35.0)
    # 建图阶段不追求速度，用更宽的视场兜住侧前方的墙角，避免巡边时蹭墙
    WALL_GUARD_WIDE_SPAN = math.radians(65.0)

    def _wall_guard(self, v, w):
        readings = self.robot.vision
        if not readings:
            return v, w
        span = (
            self.WALL_GUARD_WIDE_SPAN
            if self.phase in ("map_follow", "map_seek")
            else self.WALL_GUARD_SPAN
        )
        ahead = [
            (relative, distance)
            for relative, distance in readings
            if abs(relative) <= span
        ]
        if not ahead:
            return v, w
        relative, distance = min(ahead, key=lambda item: item[1])
        # 弓字巡迹速度较高，需要比贴边模式留更大的制动距离。
        if self.phase in ("zone_stripes", "to_segment"):
            limit = 0.32
        elif self.phase in ("ring", "detour"):
            limit = 0.30
        else:
            limit = 0.28
        if distance < limit:
            steer = -1.4 if relative >= 0.0 else 1.4
            return 0.0, world.clamp(w + steer, -1.6, 1.6)
        return v, w

    # ----------------------------------------------------------- 弓字覆盖
    def _plan_stripes(self):
        # 弓字只能铺在"从车当前位置真的开得到"的连通域里：绕障绕完一圈后，障碍物
        # 内部还是未知格，但它被已探明的占据环封住，不属于可达区。
        self.grid.set_reach_seed(*self._pose()[:2])
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
            if circle.get("kind") == "large":
                continue
            # 贴墙的视觉碎片不是场内障碍，盖章会在区域内形成封锁环。
            if world.boundary_distance(circle["x"], circle["y"]) < circle["r"] + 0.45:
                continue
            self.grid.stamp_footprint(circle["x"], circle["y"], circle["r"])
        # 只要覆盖率还在涨，就不把重规划计入"无效轮次"上限，避免大区域被提前结束。
        coverage_now = self.grid.zone_coverage(self.zone_id)
        # 原地重规划保护：连着好几次重规划、车却没挪窝（也没多割到），说明车被顶住了，
        # 不是这些弓字线走不到。以前这里靠"标记失败"往下走，会一股脑把主方向的线段
        # 全标掉，主覆盖被迫提前换方向（看起来"先横着排、又改竖着排"）。现在先沿来路
        # 退一段再按原方向重来。
        if self._plan_is_spinning(coverage_now):
            return
        if coverage_now > self.last_plan_coverage + 0.01:
            self.zone_plan_passes = 0
        self.last_plan_coverage = coverage_now
        raw_segments = self._node_lanes(zone, "COVERAGE")
        self._record_plan_segments(raw_segments)
        filtered = [
            segment for segment in raw_segments if _segment_key(segment) not in self.failed_segments
        ]
        pose = self._pose()
        self.segments = planner.order_segments(filtered, pose[:2], pose[2])
        self.segment_index = 0
        self.zone_plan_passes += 1
        self.zone_total_plans += 1
        if self.zone_plan_passes > MAX_ZONE_PLAN_PASSES or self.zone_total_plans > 400:
            self._finish_zone_coverage()
            return
        if not self.segments and self.lane_axis == self.main_lane_axis:
            coverage = self.grid.zone_coverage(self.zone_id)
            if coverage < 0.93:
                self.lane_axis = "v" if self.main_lane_axis == "h" else "h"
                self.zone_plan_passes = 0
                vertical = self._node_lanes(zone, "COVERAGE", axis="v")
                filtered = [
                    segment
                    for segment in vertical
                    if _segment_key(segment) not in self.failed_segments
                ]
                self._record_plan_segments(vertical)
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
        self._record_plan_edge(zone)
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
        if self.lane_axis != self.main_lane_axis and self._plan_rings():
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
        self.segment_nav_attempts = 0
        self.segment_reversed = False
        a, b = self.segments[index]
        self.segment_plan = (a, b)
        entry = self._segment_entry(a, b)
        if entry is None:
            if self._try_reverse_segment():
                return
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

    def _try_reverse_segment(self):
        """从线段另一端进入，避免入口被占时整条可达弓字线被丢弃。"""
        if self.segment_reversed or self.segment_index >= len(self.segments):
            return False
        a, b = self.segments[self.segment_index]
        self.segment_plan = (a, b)
        entry = self._segment_entry(b, a)
        if entry is None:
            return False
        self.segment_reversed = True
        self.segment = (entry, a)
        self.segment_end = a
        x, y, _ = self._pose()
        if math.hypot(entry[0] - x, entry[1] - y) < 0.25:
            self.route = []
            self.route_kind = None
            self.phase = "zone_stripes"
            return True
        if self._set_route(entry, "to_segment"):
            self.phase = "to_segment"
            return True
        return False

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
                # 允许第二轮补扫，清掉第一轮剩下的窄条/边角。
                if self.patch_passes < 2:
                    self.patch_cleanup_done = False
                self._complete_zone()
            else:
                self.phase = "zone_plan"
            return 0.0, 0.0

        if self._no_go_bypass_needed(target, "zone_stripes") is not None:
            # 弓字线本身已按禁区膨胀规划，走到禁区边说明这条线走不通：跳过并重规划。
            self._log("info", "弓字线接近禁区，跳过该段并重新规划剩余区域")
            if self.segment is not None:
                self.failed_segments.add(_segment_key(self.segment_plan or self.segment))
            self.phase = "zone_plan"
            self.last_phase = None
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
        self.patch_passes += 1
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
        raw_segments = self._node_lanes(zone, "OVERLAPING")
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
        self.segment_plan = None
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
        # 建图只探边界，场内障碍要等割草时才看到，因此第一次弓字之后的漏扫块
        # 会比"整场都建过图"时多得多：只要每轮补割还能真割到东西（≥0.5 个
        # 百分点），就继续补，别一刀切只补一轮。
        gain = coverage - self.coverage_retry_best
        remaining = self.grid.unmowed_components(self.zone_id, PATCH_MIN_CELLS)
        leftover = sum(len(cells) for cells in remaining) * self.grid.cell * self.grid.cell
        if (
            remaining
            and leftover >= COVERAGE_RETRY_MIN_AREA
            and self.coverage_retries < COVERAGE_RETRY_LIMIT
            and (self.coverage_retries == 0 or gain >= COVERAGE_RETRY_MIN_GAIN)
        ):
            self.coverage_retry_best = coverage
            self.coverage_retries += 1
            self._log(
                "warn",
                "区域 {} 还有 {:.1f} m² 没割到，继续补割剩余连通块（第 {} 轮）".format(
                    self.zone_id, leftover, self.coverage_retries
                ),
            )
            self.patch_cleanup_done = False
            self.failed_segments = set()
            self.zone_plan_passes = 0
            self.zone_stage = "stripes"
            self.lane_axis = "v" if self.lane_axis == "h" else "h"
            self.segments = []
            self.segment = None
            self.segment_plan = None
            self.phase = "zone_plan"
            return
        self._log(
            "event",
            "区域 {} 割草完成，覆盖率 {:.0f}%".format(self.zone_id, coverage * 100.0),
        )
        self._record_zone_finish(self.zone_id)
        self.segments = []
        self.segment = None
        self.segment_plan = None
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
            self._write_mow_report()
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
            self.bypass_skip_count += 1
            if self.bypass_skip_count in (3, 6, 12):
                # 反复挡在同一条线上：把障碍物盖章进栅格，让弓字线真的绕开它，
                # 否则会"重新规划→又撞上→再跳过"地空转
                self.grid.stamp_footprint(circle["x"], circle["y"], circle["r"])
                self._log(
                    "info",
                    "同一障碍物已连续 {} 次挡路，写入栅格后重新规划".format(
                        self.bypass_skip_count
                    ),
                )
            self._log("info", "该障碍物已绕行过，跳过二次绕障并重新规划")
            if source in ("edge", "zone_edge"):
                # 已绕过的障碍仍挡在预瞄方向时，一次推进到该障碍后的
                # 边缘点；不要递归调用 skip，否则会反复回到同一点。
                self._advance_edge_past_circle(circle)
            else:
                if self.segment is not None:
                    self.failed_segments.add(_segment_key(self.segment_plan or self.segment))
                self.phase = "zone_plan"
            self.last_phase = None
            return

        if (
            self.mowing_mode == "B"
            and source == "stripes"
            and circle is not None
            and circle.get("kind") == "large"
        ):
            self.grid.stamp_footprint(circle["x"], circle["y"], circle["r"])
            self._mark_bypassed(circle)
            if self.segment is not None:
                self.failed_segments.add(_segment_key(self.segment_plan or self.segment))
            self.phase = "zone_plan"
            self.last_phase = None
            self.stats["large_replans"] += 1
            self._log("info", "遇到大型障碍物：掉头并重新规划弓字")
            return

        if circle is not None:
            zone = self._zone()
            points = planner.ring_route(circle, (x, y), grid=self.grid, zone=zone)
            if not points:
                self._mark_bypassed(circle)
                self._log("warn", "障碍物周围被其他障碍遮挡，跳过绕障并重新规划")
                if source == "edge":
                    self._resume_edge_route()
                else:
                    if self.segment is not None:
                        self.failed_segments.add(_segment_key(self.segment_plan or self.segment))
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
            self.ring_bumps = 0
            self.ring_scan_count = 0
            self.ring_stuck = 0
            self.ring_probe = 0
            self.ring_origin = (x, y)
            self.ring_radius = self._ring_nominal_radius(points, circle)
            # 弧越长给的越宽松：30 个点 ≈ 6 m ≈ 20 s，留足余量
            self.ring_time_limit = 20.0 + 0.7 * len(points)
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

    def _mark_bypassed(self, circle):
        """把障碍物记成"已绕过"，同时盖进刀盘层。

        只盖 `mow_blocked`（不改成占据格）：地图上还是真实轮廓，但规划弓字线时
        会绕开这块，省掉"走到跟前 → 发现已绕过 → 跳过 → 重新规划"的反复折腾。
        """
        if circle is None:
            return
        if not self._is_bypassed(circle):
            self.bypassed_circles.append(dict(circle))
        try:
            self.grid.block_for_mowing(circle["x"], circle["y"], circle["r"])
        except (KeyError, TypeError):
            pass

    def _is_bypassed(self, circle):
        for known in self.bypassed_circles:
            if math.hypot(circle["x"] - known["x"], circle["y"] - known["y"]) < max(
                0.55, known["r"] + 0.4
            ):
                return True
        return False

    # ----------------------------------------------------------- 禁区绕行
    def _no_go_at(self, x, y, margin=0.0):
        """返回点所在（或 margin 范围内）的禁区矩形；A 模式绕禁区用。"""
        for zone in world.NO_GO:
            if zone.get("x0") is None:
                continue
            x0 = min(zone["x0"], zone["x1"]) - margin
            x1 = max(zone["x0"], zone["x1"]) + margin
            y0 = min(zone["y0"], zone["y1"]) - margin
            y1 = max(zone["y0"], zone["y1"]) + margin
            if x0 <= x <= x1 and y0 <= y <= y1:
                return zone
        return None

    def _no_go_bypass_needed(self, target, resume):
        """沿边/弓字前方是否被禁区挡住（命中则返回禁区矩形）。"""
        if not world.NO_GO:
            return None
        x, y, theta = self._pose()
        hit = self._no_go_at(target[0], target[1], world.ROBOT_RADIUS + 0.06)
        if hit is None:
            look = 0.55 if resume == "zone_edge" else 0.45
            ahead_x = x + math.cos(theta) * look
            ahead_y = y + math.sin(theta) * look
            hit = self._no_go_at(ahead_x, ahead_y, world.ROBOT_RADIUS + 0.06)
        return hit

    def _rect_ring_points(self, zone, margin):
        """逆时针绕禁区矩形一周的路径点（外扩 margin，按机器人最近点起始）。"""
        x0 = min(zone["x0"], zone["x1"]) - margin
        x1 = max(zone["x0"], zone["x1"]) + margin
        y0 = min(zone["y0"], zone["y1"]) - margin
        y1 = max(zone["y0"], zone["y1"]) + margin
        corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        route = []
        for index in range(4):
            ax, ay = corners[index]
            bx, by = corners[(index + 1) % 4]
            steps = max(1, int(math.ceil(math.hypot(bx - ax, by - ay) / 0.35)))
            for step in range(steps):
                ratio = step / float(steps)
                route.append((ax + (bx - ax) * ratio, ay + (by - ay) * ratio))
        if not route:
            return []
        # 禁区贴墙时，外扩后的绕行点可能压到外墙：向场地中心收回一点
        need = world.ROBOT_RADIUS + 0.10
        center_x = world.WIDTH * 0.5
        center_y = world.HEIGHT * 0.5
        cleaned = []
        for px, py in route:
            clearance = world.boundary_distance(px, py)
            if clearance < need:
                dx, dy = center_x - px, center_y - py
                length = math.hypot(dx, dy) or 1.0
                shift = min(need - clearance + 0.05, 1.2)
                px += dx / length * shift
                py += dy / length * shift
                if world.boundary_distance(px, py) < need * 0.85:
                    continue
            cleaned.append((px, py))
        if len(cleaned) < 4:
            return []
        x, y, _ = self._pose()
        start = min(
            range(len(cleaned)),
            key=lambda i: math.hypot(cleaned[i][0] - x, cleaned[i][1] - y),
        )
        return [cleaned[(start + offset) % len(cleaned)] for offset in range(len(cleaned))]

    def _no_go_ring_key(self, zone):
        return (zone.get("id"), self.edge_index, self.zone_id)

    def _begin_no_go_ring(self, zone, resume="zone_edge"):
        if self.phase == "no_go_ring":
            return False
        key = self._no_go_ring_key(zone)
        if self.no_go_ring_last == key:
            # 同一位置已经绕过一次仍然被判定受阻：改走兜底跳过，避免来回绕圈
            return False
        margin = max(0.24, config.settings.ring_margin)
        points = self._rect_ring_points(zone, margin)
        if len(points) < 4:
            return False
        self.no_go_ring_exit = None
        self.no_go_ring_exit_point = None
        if resume == "zone_edge":
            trimmed = self._trim_ring_to_exit(points, zone)
            # 找不到出口（例如边缘点还没生成）时退回整圈绕行
            if len(trimmed) >= 3:
                points = trimmed
        self.no_go_ring_last = key
        self.no_go_ring_points = points
        self.no_go_ring_index = 0
        self.no_go_ring_started = False
        self.no_go_ring_resume = resume
        self.phase = "no_go_ring"
        self.last_phase = None
        self.stats["no_go_rings"] = self.stats.get("no_go_rings", 0) + 1
        self._log(
            "info",
            "遇到禁区：逆时针绕行禁区一周后继续" + ("沿边" if resume == "zone_edge" else "割草"),
        )
        return True

    def _advance_edge_past_no_go(self, zone, skip=0.35):
        """无法绕行时，直接跳过贴在禁区附近的边缘点。"""
        while self.edge_index < len(self.edge_points):
            point = self.edge_points[self.edge_index]
            if self._no_go_at(point[0], point[1], skip) is None:
                break
            self.edge_index += 1
        self.edge_resume_target = -1
        self.edge_resume_attempts = 0
        if self.edge_index >= len(self.edge_points):
            self._finish_edge_phase()
        self.last_phase = None

    def _trim_ring_to_exit(self, points, zone, skip=0.45):
        """只保留从进入侧绕到出口侧的半圈，避免绕回原点又触发一次。"""
        x, y, _ = self._pose()
        count = len(self.edge_points)
        exit_index = None
        exit_point = None
        for step in range(count):
            index = (self.edge_index + step) % count
            point = self.edge_points[index]
            if math.hypot(point[0] - x, point[1] - y) < 0.60:
                continue
            if self._no_go_at(point[0], point[1], skip) is not None:
                continue
            exit_index = index
            exit_point = point
            break
        if exit_point is None:
            return []
        best = min(
            range(len(points)),
            key=lambda i: math.hypot(
                points[i][0] - exit_point[0], points[i][1] - exit_point[1]
            ),
        )
        if best < 1:
            return []
        self.no_go_ring_exit = exit_index
        self.no_go_ring_exit_point = exit_point
        return points[: best + 1]

    def _no_go_ring_step(self):
        if not self.no_go_ring_points:
            self._finish_no_go_ring()
            return 0.0, 0.0
        x, y, theta = self._pose()
        if not self.no_go_ring_started:
            target = self.no_go_ring_points[min(1, len(self.no_go_ring_points) - 1)]
            err = world.wrap_angle(math.atan2(target[1] - y, target[0] - x) - theta)
            if abs(err) > 0.30:
                return 0.0, world.clamp(2.0 * err, -1.5, 1.5)
            self.no_go_ring_started = True
        target = self.no_go_ring_points[self.no_go_ring_index]
        if math.hypot(target[0] - x, target[1] - y) < 0.16:
            self.no_go_ring_index += 1
            if self.no_go_ring_index >= len(self.no_go_ring_points):
                self._finish_no_go_ring()
                return 0.0, 0.0
            target = self.no_go_ring_points[self.no_go_ring_index]
        err = world.wrap_angle(math.atan2(target[1] - y, target[0] - x) - theta)
        speed = EDGE_SPEED if self.no_go_ring_resume == "zone_edge" else MOW_SPEED
        v = speed * max(0.22, 1.0 - abs(err) / 1.5) if abs(err) < 1.3 else 0.0
        w = world.clamp(2.2 * err, -1.4, 1.4)
        return self._wall_guard(v, w)

    def _finish_no_go_ring(self):
        resume = self.no_go_ring_resume
        self.no_go_ring_points = []
        self.no_go_ring_index = 0
        self.no_go_ring_started = False
        self.no_go_ring_resume = None
        if resume == "zone_edge":
            if self.no_go_ring_exit is not None:
                # 直接接到禁区出口之后的边缘点
                self.edge_index = self.no_go_ring_exit
            else:
                while self.edge_index < len(self.edge_points):
                    point = self.edge_points[self.edge_index]
                    if self._no_go_at(point[0], point[1], 0.45) is None:
                        break
                    self.edge_index += 1
            self.edge_resume_target = -1
            self.edge_resume_attempts = 0
            if self.edge_index >= len(self.edge_points):
                self._finish_edge_phase()
            else:
                self.phase = "zone_edge"
        else:
            if self.segment is not None:
                self.failed_segments.add(_segment_key(self.segment_plan or self.segment))
            self.phase = "zone_plan"
        self.last_phase = None

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
                if self.ring_lost_time > 0.5 and self.ring_scan_count < RING_SCAN_LIMIT:
                    self.ring_lost_time = 0.0
                    self.ring_scan_count += 1
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
        # 车被挤到绕障圆内侧时，目标点会跑到侧后方，照直开就会一头顶在障碍物上：
        # 先把车沿半径方向推回圆弧，再继续沿弧绕
        circle = self.ring_circle
        if circle is not None and self.ring_radius:
            cx, cy = circle["x"], circle["y"]
            radius = math.hypot(x - cx, y - cy)
            if 0.05 < radius < self.ring_radius - 0.08:
                out_x = cx + (x - cx) / radius * (self.ring_radius + 0.06)
                out_y = cy + (y - cy) / radius * (self.ring_radius + 0.06)
                err_out = world.wrap_angle(math.atan2(out_y - y, out_x - x) - theta)
                if abs(err_out) < 2.2:
                    return self._wall_guard(
                        0.26 * max(0.25, 1.0 - abs(err_out) / 1.6),
                        world.clamp(2.4 * err_out, -1.4, 1.4),
                    )
        target = self.ring_points[self.ring_index]
        if not self.grid.line_clear((x, y), target) and math.hypot(
            target[0] - x, target[1] - y
        ) > 0.35:
            # 弧段被挡住时按"外推 → 收紧预瞄 → 跳过弧点 → 贴面绕行"逐级降级，
            # 以前一卡就跳过 6 个点，绕障动不动就断在半路
            self.ring_probe += 1
            self._ring_push_out(0.06 + 0.02 * min(self.ring_probe, 8))
            tighter = self._ring_tight_target()
            if tighter is not None:
                target = tighter
            elif self.ring_probe % 4 == 0 and self._ring_skip_ahead():
                target = self.ring_points[self.ring_index]
            if self.ring_probe >= RING_PROBE_LIMIT:
                self._ring_to_surface("弧段连续受阻 {} 次".format(self.ring_probe))
                return 0.0, 0.0
        else:
            self.ring_probe = 0
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

    @staticmethod
    def _ring_nominal_radius(points, circle):
        """绕障弧段的标称半径（各点到障碍物中心的距离中位数）。"""
        if not points or circle is None:
            return 0.0
        values = sorted(
            math.hypot(point[0] - circle["x"], point[1] - circle["y"]) for point in points
        )
        return values[len(values) // 2]

    def _ring_tight_target(self, prefer=0.45):
        """弧段被挡时改用"离车最近、且直达"的弧点做预瞄（弦更短，贴着障碍物走）。"""
        x, y, _ = self._pose()
        points = self.ring_points
        if not points:
            return None
        best = None
        best_distance = 1e9
        for index in range(self.ring_index, len(points)):
            px, py = points[index]
            distance = math.hypot(px - x, py - y)
            if distance < 0.18:
                continue
            if distance > prefer + 0.35:
                break
            if not self.grid.line_clear((x, y), (px, py)):
                continue
            if best is None or distance < best_distance:
                best = index
                best_distance = distance
        if best is None:
            return None
        self.ring_index = best
        return points[best]

    def _ring_to_surface(self, reason):
        """几何弧段实在走不通：改成贴面绕行继续绕，而不是直接结束绕障。"""
        circle = self.ring_circle
        after = self.ring_after
        if self.ring_surface_tries >= RING_FALLBACK_LIMIT:
            self._log("warn", "绕障受阻（{}），跳过该障碍并重新规划".format(reason))
            if circle is not None:
                self._mark_bypassed(circle)
            self.ring_circle = None
            self.ring_points = []
            self.ring_index = 0
            self.ring_after = None
            if after == "rings":
                self._next_ring()
                return
            if self.detour_source == "stripes" and self.segment is not None:
                self.failed_segments.add(_segment_key(self.segment_plan or self.segment))
            self._finish_detour()
            return
        self.ring_surface_tries += 1
        if circle is not None:
            self._mark_bypassed(circle)
        self.ring_circle = None
        self.ring_points = []
        self.ring_index = 0
        self.ring_after = None
        # 从"绕障起点"开始量回来，走到起点附近才算绕完一圈
        self.detour_start = self.ring_origin or self._pose()[:2]
        self.detour_distance = 0.0
        self.detour_source = self.detour_source or "edge"
        self.detour_ring_after = after
        self.phase = "detour"
        self.route = []
        self._log(
            "info",
            "绕障弧段走不通（{}），改用贴面绕行继续绕（第 {} 次）".format(
                reason, self.ring_surface_tries
            ),
        )

    RING_SKIP_POINTS = 6

    def _ring_push_out(self, amount=0.14):
        """把还没走到的弧段整体往外推一点：蹭到障碍物说明绕障半径压得太紧。"""
        circle = self.ring_circle
        if circle is None or not self.ring_points:
            return 0
        cx, cy = circle["x"], circle["y"]
        moved = 0
        for index in range(self.ring_index, len(self.ring_points)):
            x, y = self.ring_points[index]
            distance = math.hypot(x - cx, y - cy)
            if distance < 1e-6:
                continue
            scale = (distance + amount) / distance
            self.ring_points[index] = (cx + (x - cx) * scale, cy + (y - cy) * scale)
            moved += 1
        return moved

    def _ring_skip_ahead(self):
        """绕障被挡住时先跳过弧段上的几个点，别一卡就直接结束整圈绕障。

        绕障后面还要接着割障碍物周围那一圈，过早收工会在障碍物旁边留下
        一条没割的带子。
        """
        if not self.ring_points:
            return False
        index = self.ring_index + self.RING_SKIP_POINTS
        if index >= len(self.ring_points) - 2:
            return False
        self.ring_index = index
        self.ring_started = True
        self._log("info", "绕障受阻，跳过弧段上的几个点继续绕行")
        return True

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
        self.ring_probe = 0
        after = self.ring_after
        self.ring_after = None
        if self.ring_circle is not None:
            self._mark_bypassed(self.ring_circle)
            self.ring_circle = None
        clipped = self.ring_clipped
        self.ring_clipped = False
        self.ring_scan_time = 0.0
        self.ring_lost_time = 0.0
        if after == "detour":
            if clipped:
                self._log("info", "绕障在区域边界结束，沿区域边界继续任务")
            if self.detour_source == "stripes" and self.segment is not None:
                # 这条弓字线绕不过去：作废它，剩下的区域重新规划，
                # 否则规划器会再排一条同样的线，车又撞回同一个障碍物
                self.failed_segments.add(_segment_key(self.segment_plan or self.segment))
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
        slack = DETOUR_ZONE_SLACK
        if zone is not None and not (
            zone["x0"] - slack <= x <= zone["x1"] + slack
            and zone["y0"] - slack <= y <= zone["y1"] + slack
        ):
            if self.robot.sim_time - self.edge_guard_log_time > 2.0:
                self.edge_guard_log_time = self.robot.sim_time
                self._log("info", "沿边绕障走远了，返回本区域边缘")
            self._finish_detour()
            return 0.0, 0.0
        if self.detour_start is not None:
            returned = math.hypot(x - self.detour_start[0], y - self.detour_start[1]) < 0.28
            if self.detour_distance > 2.2 and returned:
                self._finish_detour()
                return 0.0, 0.0
        if self.detour_distance > DETOUR_MAX_DISTANCE:
            self._log("info", "贴面绕障走了 {:.1f} m，回到正常路径".format(self.detour_distance))
            self._finish_detour()
            return 0.0, 0.0
        return self._wall_guard(v, w)

    def _finish_detour(self):
        self._log("event", "绕障完成并回到历史路径")
        source = self.detour_source
        ring_after = self.detour_ring_after
        self.detour_source = None
        self.detour_start = None
        self.detour_distance = 0.0
        self.detour_ring_after = None
        if ring_after == "rings":
            # 本来是在给障碍物做环形补割：接着绕下一个
            self._next_ring()
            return
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
        self.edge_stuck = 0
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
        # 贴住障碍物时先放开脚下导航膨胀：否则重规划会沿着同一圈膨胀再撞一次
        if self.phase in ("zone_stripes", "zone_edge", "to_segment", "detour", "ring"):
            pose = self._pose()
            self.grid.relax_around(pose[0], pose[1], 0.45)
        if (
            self.stuck_time > BUMP_STUCK_SECONDS
            and self.phase in self.BUMP_ESCAPE_PHASES
            and self.zone_escape_count < ESCAPE_LIMIT
            and self._begin_escape(self.phase)
        ):
            # 顶在墙/障碍物上还在往前踩：位置几乎没动，等 STUCK_TRIGGER_SECONDS 只是
            # 白白磨时间，直接沿来路倒车脱困（车顶在墙角时这是最有效的一招）。
            self.zone_escape_count += 1
            self.stats["stuck_recoveries"] += 1
            self.stuck_time = 0.0
            self.stuck_ref = None
            return
        if self.phase in ("zone_stripes", "zone_edge"):
            self.zone_bump_count += 1
        if self.phase == "zone_stripes" and self.zone_bump_count >= 2:
            if self.segment is not None:
                self.failed_segments.add(_segment_key(self.segment_plan or self.segment))
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
            # 几何圆弧被挤住（障碍贴墙、通道太窄）时改成贴面绕行：
            # 贴面控制会跟墙面保持距离，比硬压着圆弧走稳得多，也不会一碰就收工
            self.ring_bumps += 1
            self._ring_to_surface("擦碰障碍物 {} 次".format(self.ring_bumps))
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
                    self.failed_segments.add(_segment_key(self.segment_plan or self.segment))
                self.phase = "zone_plan"
            return
        if self.phase == "zone_plan":
            circle = self._nearest_circle(self._pose()[0], self._pose()[1], 1.0)
            if circle is not None:
                self.grid.stamp_footprint(circle["x"], circle["y"], circle["r"])
                self._mark_bypassed(circle)
            self.phase = "zone_plan"
        elif self.phase == "zone_stripes":
            self._begin_detour(self.phase)
        elif self.phase == "to_segment":
            if self.segment is not None:
                self.failed_segments.add(_segment_key(self.segment_plan or self.segment))
            self.phase = "zone_plan"
        elif self.phase in ("to_zone", "charge_go", "charge_return", "to_dock"):
            self._replan_route()

    # ----------------------------------------------------------- 沿边控制
    def _follow_surface(self, speed, desired, front_limit=0.38, side=1):
        """贴墙巡边；side=1 让墙在右侧（逆时针），side=-1 让墙在左侧（顺时针）。"""
        readings = self.robot.vision
        front = _front_distance(readings, 15.0)
        wall = _side_surface_distance(readings, side)
        if front is not None and front[0] < front_limit:
            return 0.16, 1.6 * side
        nearest = min(d for _, d in readings) if readings else world.VISION_RANGE
        if nearest < 0.30:
            return 0.14, 1.6 * side
        if wall is None:
            return 0.35, -0.9 * side
        if wall < 0.30:
            return 0.30, 1.2 * side
        if wall < desired - 0.08:
            return min(speed, 0.40), 0.7 * side
        if wall > desired + 0.20:
            return min(speed, 0.40), -0.8 * side
        return speed, 0.0

    # ----------------------------------------------------------- 导航
    def _plan_with_budget(self, start, goal, budget=None, **kwargs):
        """带 tick 级时间预算的 A*：大图单次可能上百毫秒，多调用叠加会卡住仿真线程。

        ``budget`` 显式给定时（回桩等低频关键规划）不受本帧预算限制，用完把本帧预算清零。
        """
        if budget is None:
            if self.tick_plan_left <= 0.005:
                return []
            budget = min(ROUTE_PLAN_BUDGET, self.tick_plan_left)
        began = time.monotonic()
        path = self.grid.plan(start, goal, time_budget=budget, **kwargs)
        self.tick_plan_left -= time.monotonic() - began
        return path

    def _find_route(self, goal, budget=None):
        self.route_loose = False
        if budget is None and self.tick_plan_left <= 0.005:
            return [], goal, float("inf")
        x, y, _ = self._pose()
        path = self._plan_with_budget((x, y), goal, budget=budget)
        if not path and self.grid.relax_around(x, y, 0.45):
            # 机器人当前点可能被相邻障碍的保守膨胀包围；该点本身已证明
            # 可通行，先释放脚下膨胀再尝试一次，避免区域间导航死锁。
            path = self._plan_with_budget((x, y), goal, budget=budget)
        if not path and self.grid.relax_around(goal[0], goal[1], 0.30):
            # 目标（弓字起点/桩位）落在膨胀带里时，放开目标周围再试。
            path = self._plan_with_budget((x, y), goal, budget=budget)
        if path:
            self.route_fail_streak = 0
            return path, path[-1], 0.0
        # 门口被视觉膨胀糊死时，允许沿“已探明自由区”穿过膨胀带（靠视觉前向保护避障）。
        # 这条 A* 在大图上要 0.3 s，必须限频，否则重规划循环会把仿真线程拖死。
        loose = None
        now = self.robot.sim_time
        if now >= self.loose_plan_cooldown:
            loose = self._plan_with_budget(
                (x, y), goal, budget=budget, allow_unknown=False, ignore_blocked=True
            )
            self.loose_plan_cooldown = now + (0.8 if loose else 2.0)
        if loose:
            self.grid.relax_trail(loose, 0.30)
            self.route_loose = True
            self.route_fail_streak = 0
            self._log("warn", "目标被导航膨胀挡住，放开途经膨胀后继续前往")
            return loose, loose[-1], 0.0
        near_path, distance = self.grid.plan_near((x, y), goal)
        if near_path and distance <= 0.35:
            self.route_fail_streak = 0
            return near_path, near_path[-1], distance
        self.route_fail_streak = getattr(self, "route_fail_streak", 0) + 1
        if self.route_fail_streak >= 6:
            # 连续多目标不可达：多半是视觉碎片盖章把区域封死，放开周边再试。
            self.route_fail_streak = 0
            self.grid.relax_around(x, y, 1.30)
            path = self._plan_with_budget((x, y), goal, budget=budget)
            if path:
                self._log("warn", "连续导航受阻，放开周边导航膨胀后重新规划")
                return path, path[-1], 0.0
            near_path, distance = self.grid.plan_near((x, y), goal)
            if near_path:
                self._log("warn", "连续导航受阻，改去最近可达点（偏离 {:.1f} m）".format(distance))
                return near_path, near_path[-1], distance
        return [], goal, float("inf")

    HISTORY_STEP = 0.6
    DOCK_ROUTE_KINDS = ("to_dock", "map_dock", "mow_dock", "charge_return")

    def _history_route(self, goal):
        """沿机器人真实走过的轨迹倒推一条回程路线。

        场地被障碍或窄口切成小区域时，A* 要么绕一大圈、要么因为视觉膨胀找不到路；
        走过的轨迹一定可通行，因此可以直接原路退回，先离开当前小区域再接管导航。
        """
        points = list(self.history)
        if len(points) < 4:
            return []
        x, y = self._pose()[0], self._pose()[1]
        route = []
        last = (x, y)
        for point in reversed(points):
            if math.hypot(point[0] - last[0], point[1] - last[1]) < self.HISTORY_STEP:
                continue
            route.append((point[0], point[1]))
            last = point
            if math.hypot(point[0] - goal[0], point[1] - goal[1]) <= 1.2:
                break
        if not route:
            return []
        route.reverse()
        if math.hypot(route[-1][0] - goal[0], route[-1][1] - goal[1]) > 0.45:
            route.append((float(goal[0]), float(goal[1])))
        return route

    def _history_detour_route(self, goal, path):
        """A* 路线异常（过长或不可达）时，判断"原路退回"是不是更划算。"""
        history = self._history_route(goal)
        if len(history) < 2:
            return None
        x, y = self._pose()[0], self._pose()[1]
        straight = math.hypot(goal[0] - x, goal[1] - y)
        history_length = _path_length(history)
        if history_length > max(30.0, straight * 4.0 + 20.0):
            return None
        if not path:
            return history
        if _path_length(path) > max(straight * 2.2, history_length * 1.1):
            return history
        return None

    def _find_dock_route(self, goal):
        """回桩专用规划：A* 不通时按“走过的路 → 已探明自由区”两级回退。"""
        self.route_loose = False
        path, endpoint, near_distance = self._find_route(goal, budget=DOCK_PLAN_BUDGET)
        if path and near_distance <= 1.0:
            return path, endpoint, near_distance
        history = self._history_route(goal)
        if len(history) >= 2 and _path_length(history) <= max(30.0, near_distance if near_distance < 1e6 else 30.0):
            self.grid.relax_trail(history, 0.30)
            self.route_from_history = True
            self.route_loose = True
            self._log(
                "warn",
                "回桩路线不可达，改为沿历史轨迹原路退回（{} 个路径点）".format(len(history)),
            )
            return history, history[-1], 0.0
        loose = self._plan_with_budget(
            self._pose()[:2],
            goal,
            budget=DOCK_PLAN_BUDGET,
            allow_unknown=False,
            ignore_blocked=True,
        )
        if loose:
            self.grid.relax_trail(loose, 0.35)
            self.route_loose = True
            self._log(
                "warn", "回桩路径被导航膨胀堵死，放开途经膨胀后沿已探明自由区回桩"
            )
            return loose, loose[-1], 0.0
        return path, endpoint, near_distance

    def _start_backtrack(self, goal, kind, length=6.0):
        """沿自己开过的轨迹原路退回一段，再重新规划到目标。

        场地被障碍/视觉膨胀切成死角时，A* 从当前位置可能哪儿都去不了；但来路
        是自己开过来的，一定走得通。退出去再规划，比原地反复重试有效得多。
        """
        points = list(self.history)
        if len(points) < 6:
            return False
        route = []
        total = 0.0
        last = self._pose()[:2]
        for point in reversed(points):
            distance = math.hypot(point[0] - last[0], point[1] - last[1])
            if distance < 0.12:
                continue
            route.append((point[0], point[1]))
            total += distance
            last = point
            if total >= length:
                break
        if len(route) < 3:
            return False
        self.backtrack_next = (float(goal[0]), float(goal[1]), kind)
        self.route = route
        self.route_index = 1
        self.route_kind = "backtrack"
        self.route_loose = True
        self.route_from_history = True
        self.route_waypoint = None
        self.nav_goal = route[-1]
        self.grid.relax_trail(route, 0.35)
        self.phase = _ROUTE_PHASES.get(kind, "to_zone")
        self.last_phase = None
        self._log(
            "warn",
            "去 {} 没有可行路线，先沿来路退回 {:.1f} m 再重新规划".format(
                _ROUTE_LABELS.get(kind, "目标"), total
            ),
        )
        return True

    def _set_route(self, goal, kind):
        goal = (float(goal[0]), float(goal[1]))
        if kind in ("to_dock", "map_dock", "mow_dock", "charge_return"):
            # 桩位附近常被障碍膨胀覆盖（甚至被视觉盖章刷成占据格），先放开桩周一圈，
            # 保证能开上桩 —— 车本来就停在桩上，这一小片一定是能走的。
            self.grid.relax_around(
                world.DOCK["x"], world.DOCK["y"], world.DOCK["r"] + 0.25, clear_occupied=True
            )
        if kind in self.DOCK_ROUTE_KINDS:
            path, endpoint, near_distance = self._find_dock_route(goal)
        else:
            path, endpoint, near_distance = self._find_route(
                goal, budget=SET_ROUTE_PLAN_BUDGET
            )
        self.route_from_history = False
        if kind in self.DOCK_ROUTE_KINDS:
            detour = self._history_detour_route(goal, path)
            if detour:
                self.grid.relax_trail(detour, 0.30)
                path = detour
                endpoint = detour[-1]
                near_distance = 0.0
                self.route_from_history = True
                self._log(
                    "warn",
                    "回桩路径受阻，改为沿历史轨迹原路退回（{} 个路径点）".format(
                        len(detour)
                    ),
                )
        limits = {
            "to_zone": 2.0,
            "to_segment": 0.8,
            "edge_resume": 1.4,
            "breakpoint": 4.0,
            "charge_return": 1.2,
            "to_dock": 1.2,
            "map_dock": 1.2,
            "mow_dock": 1.2,
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
        self.route_waypoint = None
        return True

    def _zone_id_at(self, point):
        zone = self.grid.zone_at(point[0], point[1])
        return zone["id"] if zone is not None else None

    def _handle_unreachable(self, kind):
        if kind == "map_fill":
            # 这条扫描线到不了：丢掉它，接着扫下一条
            if self.map_fill_segments:
                self.map_fill_segments.pop(0)
            self.map_fill_stage = "seek"
            self.map_fill_target = None
            self.route = []
            self.route_kind = None
            self.route_index = 0
            self.phase = "map_fill"
            self.last_phase = None
            return
        if kind == "to_segment":
            if self._try_reverse_segment():
                return
            if self.segment is not None:
                self.failed_segments.add(_segment_key(self.segment_plan or self.segment))
            self.phase = "zone_plan"
        elif kind in ("to_zone", "to_zone_stripes"):
            self._plan_to_zone_corner()
        elif kind == "edge_resume":
            self._advance_edge_after_collision(0.55)
        elif kind == "breakpoint":
            self._restore_breakpoint()
        elif kind in self.DOCK_ROUTE_KINDS:
            if self.dock_backtracks < 2 and self._start_backtrack(
                (world.DOCK["x"], world.DOCK["y"]), kind
            ):
                # 桩位找不到路，多半是车被自己围在死角里：先沿来路退出去
                self.dock_backtracks += 1
                self.dock_retries = 0
                return
            self.dock_retries += 1
            if self.dock_retries <= 3:
                x, y = self._pose()[:2]
                released = self.grid.release_blocked_disc(x, y, 2.0)
                self._log(
                    "warn",
                    "回桩目标不可达，放开车体周边 {} 格导航膨胀后重试（第 {} 次）".format(
                        released, self.dock_retries
                    ),
                )
                self.replan_cooldown = 0.0
                self._set_route((world.DOCK["x"], world.DOCK["y"]), kind)
                return
            self.dock_retries = 0
            self._give_up_docking(kind)
        else:
            self.robot.stop()
            self.phase = "idle"

    def _give_up_docking(self, kind):
        """这一轮回不了桩：退避后再试，期间回到断点继续建图/割草。"""
        self.dock_retries = 0
        self.dock_stuck = 0
        self.robot.stop()
        delay = max(CHARGE_RETRY_SECONDS, self.charge_retry_delay)
        self.charge_retry_time = self.robot.sim_time + delay
        self.charge_retry_delay = min(CHARGE_RETRY_MAX, delay * 2.0)
        if self.breakpoint is not None and self.task is not None:
            # 回桩路线暂时不通：先回到断点继续手上的活，过一会儿再试回桩，
            # 否则任务会永远停在原地，建图/割草再也接不上。连续失败就慢慢退避，
            # 免得把时间全花在反复尝试回桩上。
            self._log(
                "warn",
                "回桩路线暂时不通，先继续当前任务，{:.0f} s 后再试回桩".format(delay),
            )
            self.charge_reason = None
            self._restore_breakpoint()
        else:
            self._log("warn", "多次放开膨胀仍无法回桩，任务停留在当前位置")
            self.charge_reason = None
            self.phase = "idle"

    def _abandon_zone(self):
        """区域实在做不动了：记一笔，跳过剩余部分接着做下一个区域。"""
        if self.task is None or self.task.get("type") != "mow":
            return
        self._log(
            "warn",
            "区域 {} 长时间无法推进，跳过剩余部分（已覆盖 {:.0f}%）".format(
                self.zone_id, self.grid.zone_coverage(self.zone_id) * 100.0
            ),
        )
        self.segments = []
        self.segment = None
        self.segment_plan = None
        self.zone_index += 1
        if self.zone_index < len(self.zone_order):
            self.zone_id = self.zone_order[self.zone_index]
            self.zone_stage = None
            self.zone_plan_passes = 0
            self.phase = "to_zone"
            self._plan_to_zone_corner()
        else:
            self.zone_id = None
            self.phase = "to_dock"
            self._set_route((world.DOCK["x"], world.DOCK["y"]), "mow_dock")
        self.last_phase = None

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
        if self.route_from_history and self.route_kind in self.DOCK_ROUTE_KINDS:
            detour = self._history_route(self.nav_goal)
            if len(detour) >= 2:
                self.grid.relax_trail(detour, 0.30)
                self.route = detour
                self.route_index = 1 if len(detour) > 1 else 0
                self.replan_cooldown = 0.8
                return True
            self.route_from_history = False
        if self.route_kind in self.DOCK_ROUTE_KINDS:
            path, endpoint, near_distance = self._find_dock_route(self.nav_goal)
        else:
            path, endpoint, near_distance = self._find_route(self.nav_goal)
        if path and near_distance <= 1.6:
            self.nav_goal = endpoint
            self.route = path
            self.route_index = 1 if len(path) > 1 else 0
            self.route_waypoint = None
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
        blocked = self.grid.is_occupied_world(target[0], target[1]) or (
            not self.route_loose and self.grid.is_blocked_world(target[0], target[1])
        )
        if blocked:
            if self.replan_cooldown <= 0.0:
                self._replan_route()
            return 0.0, 0.0
        distance = math.hypot(target[0] - x, target[1] - y)
        err = world.wrap_angle(math.atan2(target[1] - y, target[0] - x) - theta)
        # 后驱车最小转弯半径约 0.3 m：路点近在身后时永远"够不到"，只能原地
        # 绕圈。这种情况直接放过这个路点，否则会一直转到电量耗尽。
        if distance < 0.20 or (distance < 0.45 and abs(err) > 2.1):
            self.route_index += 1
            if self.route_index >= len(self.route):
                self._on_route_done()
                return 0.0, 0.0
            target = self.route[self.route_index]
            distance = math.hypot(target[0] - x, target[1] - y)
            err = world.wrap_angle(math.atan2(target[1] - y, target[0] - x) - theta)
        if self._route_waypoint_stalled(target, distance):
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
        if front is not None and front[0] < 0.50 and abs(err) < 1.0:
            v = 0.0
            w = -1.3 if front[1] >= 0.0 else 1.3
            return v, w

        readings = self.robot.vision
        if readings and v > 0.05:
            relative, distance = min(readings, key=lambda item: item[1])
            if distance < 0.30:
                # 导航膨胀变小后规划路线会更贴墙，这里必须真的让开而不是只减速
                return 0.0, world.clamp(w + (-1.4 if relative >= 0.0 else 1.4), -1.6, 1.6)
            if distance < 0.34:
                v = min(v, 0.18)
                w += -1.3 if relative >= 0.0 else 1.3
        return v, w

    ROUTE_WAYPOINT_TIMEOUT = 6.0

    def _route_waypoint_stalled(self, target, distance):
        """同一个路点追了 6 s 还没靠近，就认为够不到（贴墙/转弯半径限制）。"""
        now = self.robot.sim_time
        if self.route_waypoint != target:
            self.route_waypoint = target
            self.route_waypoint_time = now
            self.route_waypoint_best = distance
            return False
        if distance < self.route_waypoint_best - 0.05:
            self.route_waypoint_best = distance
            self.route_waypoint_time = now
            return False
        if now - self.route_waypoint_time <= self.ROUTE_WAYPOINT_TIMEOUT:
            return False
        self.route_waypoint = None
        self.route_waypoint_time = now
        return True

    def _on_route_done(self):
        kind = self.route_kind
        self.route = []
        self.route_index = 0
        self.nav_goal = None
        self.route_kind = None
        self.route_loose = False
        self.route_from_history = False
        if kind == "backtrack":
            pending = self.backtrack_next
            self.backtrack_next = None
            if pending is not None:
                goal = (pending[0], pending[1])
                saved_kind = pending[2]
                if self._set_route(goal, saved_kind):
                    self.phase = _ROUTE_PHASES.get(saved_kind, "to_zone")
                    self._log(
                        "info",
                        "退回完成，重新前往{}".format(_ROUTE_LABELS.get(saved_kind, "目标")),
                    )
                    return
            self.phase = "zone_plan"
            return
        if kind == "to_zone":
            self._enter_zone()
        elif kind == "to_zone_stripes":
            self._enter_zone_stripes_only()
        elif kind == "to_segment":
            self.phase = "zone_stripes"
        elif kind == "edge_resume":
            self.phase = "zone_edge"
        elif kind == "breakpoint":
            self._arrive_breakpoint()
        elif kind in ("to_dock", "map_dock", "mow_dock"):
            self.robot.stop()
            x, y, _ = self._pose()
            distance = math.hypot(x - world.DOCK["x"], y - world.DOCK["y"])
            if distance > world.DOCK["r"] + 0.10 and self.dock_retries < 3:
                # 目标点被遮挡时最近可达点可能离桩较远，重试靠桩而不是直接进入充电。
                self.dock_retries += 1
                self._log(
                    "warn",
                    "距充电桩 {:.2f} m，未到桩，重新规划（第 {} 次）".format(
                        distance, self.dock_retries
                    ),
                )
                self.phase = "to_dock"
                self._set_route((world.DOCK["x"], world.DOCK["y"]), kind)
                return
            if distance > 0.35 and self.dock_final_approach < 2:
                # 路线停在桩旁但没压到桩上：直接对着桩心再走一小段
                self.dock_final_approach += 1
                self.route = [(world.DOCK["x"], world.DOCK["y"])]
                self.route_index = 0
                self.route_kind = kind
                self.nav_goal = (world.DOCK["x"], world.DOCK["y"])
                self.phase = "to_dock"
                return
            if distance > world.DOCK["r"] + 0.30:
                # 反复重试仍停在离桩很远的地方：别再假装"到桩了"进充电阶段，
                # 否则会一直"回桩→未停到桩→重新靠桩"循环到电量耗尽。
                self.route_kind = kind
                self._give_up_docking(kind)
                return
            self.dock_retries = 0
            self.dock_stuck = 0
            self.dock_backtracks = 0
            self.dock_final_approach = 0
            self.charge_retry_delay = CHARGE_RETRY_SECONDS
            if self.task and self.task.get("type") == "map" and self.mapping_done:
                # 建图任务：停稳在桩上之后才分区
                self._partition_after_docking()
            if self.mapping_done and self.task and self.task.get("type") == "map":
                # 建图任务已结束：回桩即完成，不再因为 charge_reason 触发巡边续做。
                self.charge_reason = None
                self.resume_requested = False
                self.breakpoint = None
            if self.charge_reason:
                self.phase = "charging"
            else:
                self.phase = "done"
                self.task_end_sim = self.robot.sim_time
                self._log("event", "任务完成，已停靠充电桩")
                self._log_task_summary()
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
            self._mark_bypassed(circle)
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
    def _map_payload(self):
        """栅格位图按 version 缓存：同一版本重复取状态时不再重新编码。"""
        version = self.grid.version
        cached = self._map_cache
        if cached is not None and cached[0] == version:
            return cached[1]
        payload = self.grid.encode()
        self._map_cache = (version, payload)
        return payload

    def _log_task_summary(self):
        """任务完成时把各类指标写进事件日志。"""
        task = self.task or {}
        kind = "建图" if task.get("type") == "map" else "割草"
        details = []
        mowed_total = 0
        total_cells = 0
        for zone in self.grid.zones:
            total = self.grid.zone_totals.get(zone["id"], 0)
            mowed = self.grid.zone_mowed.get(zone["id"], 0)
            total_cells += total
            mowed_total += mowed
            if total:
                details.append(
                    "区域{} {:.1f}%（{}/{} 格）".format(
                        zone["id"], 100.0 * mowed / total, mowed, total
                    )
                )
        overall = (mowed_total / float(total_cells)) if total_cells else 0.0
        start = self.task_start_sim if self.task_start_sim is not None else self.robot.sim_time
        end = self.task_end_sim if self.task_end_sim is not None else self.robot.sim_time
        duration = max(0.0, end - start)
        if kind == "建图":
            areas = "；".join(
                "区域{} {:.1f} m²".format(
                    zone["id"],
                    self.grid.zone_totals.get(zone["id"], 0) * self.grid.cell * self.grid.cell,
                )
                for zone in self.grid.zones
            )
            self._log(
                "event",
                "建图指标 · 边界建图 {:.1f}% · 自动分区 {} 个{}".format(
                    self.grid.boundary_discovery() * 100.0,
                    len(self.grid.zones),
                    "（" + areas + "）" if areas else "",
                ),
            )
        else:
            self._log(
                "event",
                "割草指标 · 总覆盖 {:.2f}%{}".format(
                    overall * 100.0,
                    "（" + "；".join(details) + "）" if details else "",
                ),
            )
        distance = getattr(self.robot, "distance_traveled", 0.0)
        collisions = self.robot.collisions
        self._log(
            "event",
            "{}指标 · 里程 {} m · 时长 {} · 弓字线段 {} 段 · 绕障 {} 次 · 脱困 {} 次 · "
            "回充 {} 次 · 碰撞 {} 次 · 平均无碰撞里程 {:.1f} m".format(
                kind,
                int(round(distance)),
                _format_duration(duration),
                self.stats.get("segments_planned", 0),
                self.stats.get("rings", 0),
                self.stats.get("stuck_recoveries", 0),
                self.stats.get("charge_count", 0),
                collisions,
                (distance / collisions) if collisions else distance,
            ),
        )

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
            "main_lane_axis": self.main_lane_axis,
            "bypassed": [dict(circle) for circle in self.bypassed_circles],
            "rings_done": self.rings_done,
            "failed_segments": list(self.failed_segments),
            "zone_plan_passes": self.zone_plan_passes,
            "patch_cleanup_done": self.patch_cleanup_done,
            "coverage_retries": self.coverage_retries,
            "map_active_time": self.map_active_time,
            "map_fill_passes": self.map_fill_passes,
            "map_fill_last_area": self.map_fill_last_area,
            "map_fill_stage": self.map_fill_stage,
            "map_fill_target": self.map_fill_target,
            "map_fill_segments": [list(seg) for seg in self.map_fill_segments],
            "map_seek_goal": self.map_seek_goal,
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
        self.main_lane_axis = saved.get("main_lane_axis", self.lane_axis)
        self.bypassed_circles = [dict(circle) for circle in saved.get("bypassed", [])]
        self.rings_done = saved.get("rings_done", False)
        self.failed_segments = set(saved.get("failed_segments", []))
        self.zone_plan_passes = saved.get("zone_plan_passes", 0)
        self.patch_cleanup_done = saved.get("patch_cleanup_done", False)
        self.coverage_retries = saved.get("coverage_retries", 0)
        self.map_active_time = saved.get("map_active_time", self.map_active_time)
        if saved.get("zone_stage") == "patch":
            self.patch_cleanup_done = False
        phase = saved["phase"]
        if str(phase).startswith("map") and self.mapping_done:
            # 建图已结束，恢复断点时不能再回到巡边，否则会出现反复建图。
            self._log("info", "建图已完成，不再恢复巡边断点，返回充电桩")
            self.phase = "to_dock"
            self._set_route((world.DOCK["x"], world.DOCK["y"]), "map_dock")
            return
        self._log("event", "已返回断点，继续之前的任务")
        if phase == "map_fill":
            # 建图兜底扫描：回充回来后**接着**扫，不要把扫到一半的队列清空重来
            self.map_fill_segments = [
                (tuple(seg[0]), tuple(seg[1]))
                for seg in (saved.get("map_fill_segments") or [])
            ]
            self.map_fill_passes = saved.get("map_fill_passes", self.map_fill_passes)
            self.map_fill_last_area = saved.get("map_fill_last_area")
            self.map_fill_stage = saved.get("map_fill_stage") or "seek"
            target = saved.get("map_fill_target")
            self.map_fill_target = tuple(target) if target else None
            self._map_watch_phase = None
            self.phase = "map_fill"
        elif phase == "map_seek":
            goal = saved.get("map_seek_goal")
            self.map_seek_goal = tuple(goal) if goal else None
            self._map_watch_phase = None
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
        elif str(phase).startswith("map"):
            # 建图阶段的任何其它子状态（例如 escape/align）都从"找边界"接着走，
            # 绝不能掉成 idle —— 那会让建图在回充之后彻底停住。
            self.phase = "map_seek"
        else:
            self.phase = "idle"

    # ----------------------------------------------------------- 状态输出
    def get_state(self, include_map=True):
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
                        "cx": zone.get("cx", (zone["x0"] + zone["x1"]) * 0.5),
                        "cy": zone.get("cy", (zone["y0"] + zone["y1"]) * 0.5),
                        "outline": zone.get("outline") or [],
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
                "map_gap_remaining": round(self.map_gap_remaining, 2),
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
                map=self._map_payload() if include_map else None,
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
    return values


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


def _side_surface_distance(readings, side=1):
    """side=1 取右侧墙面投影距离，side=-1 取左侧（贴哪面墙就用哪面）。"""
    values = []
    for relative, distance in readings:
        degrees = math.degrees(relative) * side
        if -50.0 <= degrees <= -25.0 and distance < world.VISION_RANGE - 1e-6:
            values.append(distance * math.sin(math.radians(-degrees)))
    if not values:
        return None
    return min(values)


def _right_surface_distance(readings):
    return _side_surface_distance(readings, 1)


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


def _format_duration(seconds):
    total = int(round(max(0.0, seconds)))
    return "{:02d}:{:02d}".format(total // 60, total % 60)


def _path_length(points):
    total = 0.0
    for index in range(1, len(points)):
        total += math.hypot(points[index][0] - points[index - 1][0], points[index][1] - points[index - 1][1])
    return total
