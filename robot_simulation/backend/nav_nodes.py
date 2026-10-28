"""导航节点：``fibot_nav_planner_node`` 与 ``fibot_nav_dock_node`` 的仿真实现。

两个节点按《导航节点对外接口与协议说明》暴露同样的动作服务、结果话题与状态话题，
内部把请求翻译成仿真栅格/区域上的规划结果并发布到对应输出话题；仿真任务管理器
（TaskManager）作为"上位机 + 底盘"使用这些话题驱动小车。

节点不直接驱动电机：``cmd_vel`` 由 NavBridge 按当前活动节点回填，保证与文档一致
（dock 与 planner 都可能发布 cmd_vel）同时不破坏仿真里已经打磨好的执行器逻辑。
"""

import math

import math

import nav_bus
import planner
import world
from nav_bus import (
    ACTION_START,
    ActionServer,
    BOW_HORIZONTAL,
    BOW_VERTICAL,
    MODE_BOUNDARY,
    MODE_COVERAGE,
    MODE_FOLLOW,
    MODE_NAVIGATION,
    MODE_OVERLAPING,
    MODE_VISUAL_BOUNDARY,
    MODE_VISUAL_RIDE_MOWING,
    NAV_NONE,
    SUB_NONE,
    nav_status,
    path_message,
    path_with_index,
    polygon_area,
    pose_stamped,
    zone_by_id,
)

RESULT_PLANNER = "usr_define_actions/NavTaskActionResult"
RESULT_DOCK = "usr_define_actions/AutoDockingActionResult"


def zone_edge_points(grid, zone, pose):
    """区域边界巡边路径：不规则区域沿自由区轮廓，规则区域用内缩矩形。"""
    width = max(1e-6, (zone["x1"] - zone["x0"]) * (zone["y1"] - zone["y0"]))
    fill = zone["free"] * grid.cell * grid.cell / width
    if fill < 0.92:
        points = planner.edge_route_grid(grid, zone, pose)
    else:
        points = planner.edge_route(zone, pose)
        if len(points) >= 3 and polygon_area(points) < 0.0:
            points = list(reversed(points))
    return points


def lane_segments(grid, zone, axis="h", only_unmowed=False):
    lanes = planner.lane_segments(grid, zone, axis=axis)
    if not only_unmowed:
        return lanes
    result = []
    for segment in lanes:
        a, b = segment
        touches = False
        for ratio in (0.0, 0.5, 1.0):
            x = a[0] + (b[0] - a[0]) * ratio
            y = a[1] + (b[1] - a[1]) * ratio
            col, row = grid.cell_of(x, y)
            if not grid.in_bounds(col, row):
                continue
            if grid.cell_mowed(col, row):
                continue
            if not grid.cell_in_zone(col, row, zone["id"]):
                continue
            touches = True
            break
        if touches:
            result.append(segment)
    return result


def _axis_of(bow_type):
    if int(bow_type or 0) == BOW_VERTICAL:
        return "v"
    if int(bow_type or 0) == BOW_HORIZONTAL:
        return "h"
    return "h"


class BaseNavNode(object):
    heartbeat_id = 0
    offline_code = 0
    name = "node"

    def __init__(self, bus, robot, grid):
        self.bus = bus
        self.robot = robot
        self.grid = grid
        self.inputs = {}
        self.actions = {}
        self.main_state = NAV_NONE
        self.sub_state = SUB_NONE
        self._heartbeat_at = None
        self._status_at = None
        self._last_command_at = None
        self.log_lines = []

    # --------------------------------------------------------- 通用
    def publish(self, topic, message):
        return self.bus.publish(topic, message, publisher=self.name)

    def log(self, text):
        self.log_lines.append((round(self.bus.clock, 2), text))
        del self.log_lines[:-12]

    def set_status(self, main_state, sub_state=None):
        main_state = int(main_state)
        changed = main_state != self.main_state or (
            sub_state is not None and int(sub_state) != self.sub_state
        )
        self.main_state = main_state
        if sub_state is not None:
            self.sub_state = int(sub_state)
        now = self.bus.clock
        # 状态话题按变化或 0.5 s 节流，避免 50 Hz 仿真把话题刷爆。
        if not changed and self._status_at is not None and now - self._status_at < 0.5:
            return
        self._status_at = now
        self.publish("nav_status", nav_status(self.main_state, self.sub_state))

    def heartbeat(self):
        now = self.bus.clock
        if self._heartbeat_at is not None and now - self._heartbeat_at < 0.5:
            return
        self._heartbeat_at = now
        self.publish(
            "process_heart_beat",
            {
                "type": "usr_define_health/ProcessHeartBeat",
                "node": self.name,
                "heartbeat_id": self.heartbeat_id,
                "offline_code": self.offline_code,
                "status": 1,
                "time": round(now, 3),
            },
        )

    # --------------------------------------------------------- 速度指令
    def command(self, pose, target, speed, tolerance=0.20, steer_gain=2.4, max_w=1.5,
                min_ratio=0.15, turn_limit=1.15):
        """按当前位姿与目标点计算 cmd_vel 并发布（节点自己的控制输出）。

        与真实节点一致：控制律在节点内，仿真只负责执行 ``cmd_vel``。
        """
        import static_config

        x, y, theta = pose
        dx = float(target[0]) - x
        dy = float(target[1]) - y
        distance = math.hypot(dx, dy)
        error = world.wrap_angle(math.atan2(dy, dx) - theta)
        v = speed * max(min_ratio, 1.0 - abs(error) / 1.6) if abs(error) < turn_limit else 0.0
        w = world.clamp(steer_gain * error, -max_w, max_w)
        # static_config 的 DWA 速度上限在节点侧生效（不改变底盘能力）
        node_max_v = static_config.node_param("max_linear")
        node_max_w = static_config.node_param("max_angular")
        if node_max_v:
            v = min(v, float(node_max_v))
        if node_max_w:
            w = world.clamp(w, -float(node_max_w), float(node_max_w))
        now = self.bus.clock
        if self._last_command_at is None or now - self._last_command_at >= 0.05:
            self._last_command_at = now
            self.bus.publish("cmd_vel", nav_bus.twist(v, w), publisher=self.name)
        return {
            "v": v,
            "w": w,
            "error": error,
            "distance": distance,
            "arrived": distance < tolerance,
            "node": self.name,
            "stamp": round(now, 3),
        }

    def _pose(self):
        combined = self.inputs.get("combined")
        if combined is not None:
            pose = combined.get("pose") or {}
            return (
                float(pose.get("x", 0.0)),
                float(pose.get("y", 0.0)),
                float(pose.get("theta", 0.0)),
            )
        return tuple(self.robot.pose)

    def snapshot(self):
        return {
            "name": self.name,
            "heartbeat_id": self.heartbeat_id,
            "offline_code": self.offline_code,
            "main_state": self.main_state,
            "main_state_name": nav_bus.NAV_MAIN_NAMES.get(self.main_state, "NONE"),
            "sub_state": self.sub_state,
            "sub_state_name": nav_bus.NAV_SUB_NAMES.get(self.sub_state, "NONE"),
            "actions": [action.snapshot() for action in self.actions.values()],
            "inputs": {
                key: (value.get("type") if isinstance(value, dict) else str(value))
                for key, value in sorted(self.inputs.items())
            },
            "log": ["{:.1f}s {}".format(stamp, text) for stamp, text in self.log_lines[-6:]],
        }

    def on_start(self, action_name, goal):  # pragma: no cover - 由子类实现
        raise ValueError("节点 {} 不支持动作 {}".format(self.name, action_name))

    def on_action_started(self, action_name, goal):
        self.log("{} START mode={}".format(action_name, goal.get("mode", "-")))

    # --------------------------------------------------------- 输入订阅
    def subscribe(self, topic, msg_type):
        def store(message):
            self.inputs[topic] = message

        self.bus.subscribe(topic, store, subscriber=self.name, msg_type=msg_type)


class PlannerNode(BaseNavNode):
    """``fibot_nav_planner_node``：点到点 / 覆盖 / 沿边 / 重割 规划。"""

    name = "planner"
    heartbeat_id = 800012
    offline_code = 600012

    def __init__(self, bus, robot, grid):
        super().__init__(bus, robot, grid)
        bus.advertise("cmd_vel", "geometry_msgs/Twist", publisher=self.name)
        bus.advertise("nav_status", "usr_define_msgs/NavigationStatus", publisher=self.name)
        bus.advertise("task_plan_path", "nav_msgs/Path", publisher=self.name)
        bus.advertise("nav_task_path", "nav_msgs/PathWithIndex", publisher=self.name)
        bus.advertise("follow_task_path", "nav_msgs/PathWithIndex", publisher=self.name)
        bus.advertise("current_path_index", "std_msgs/Int32", publisher=self.name)
        bus.advertise("plan_action_goal", "usr_define_actions/NavTaskActionGoal", publisher=self.name)

        # 文档 §5.3 输入话题
        self.subscribe("goal", "geometry_msgs/PoseStamped")
        self.subscribe("manager_info", "usr_define_manager/ManagerInfo")
        self.subscribe("is_on_connect_path", "std_msgs/Bool")
        self.subscribe("area_map", "nav_msgs/OccupancyGrid")
        self.subscribe("area_edge_map", "nav_msgs/OccupancyGrid")
        self.subscribe("whole_map", "nav_msgs/OccupancyGrid")
        self.subscribe("tf_dynamic", "tf2_msgs/TFMessage")

        self.plan = {"mode": MODE_NAVIGATION, "path": [], "segments": [], "edge": []}
        self.task_action = ActionServer(
            bus, "nav_task_action", "nav_task_action_result", RESULT_PLANNER, self
        )
        self.actions[self.task_action.name] = self.task_action
        # 同进程 IPA 覆盖/沿边/重叠路径服务（文档 §5.5）
        bus.bind("ipaCoverPath", self._ipa_coverage, port=nav_bus.SERVICE_PORTS["ipaCoverPath"], provider=self.name)
        bus.bind("ipaBoundPath", self._ipa_boundary, port=nav_bus.SERVICE_PORTS["ipaBoundPath"], provider=self.name)
        bus.bind("ipaOverlapPath", self._ipa_override, port=nav_bus.SERVICE_PORTS["ipaOverlapPath"], provider=self.name)

    # --------------------------------------------------------- 动作
    def on_start(self, action_name, goal):
        mode = int(goal.get("mode", MODE_NAVIGATION) or MODE_NAVIGATION)
        if mode == MODE_NAVIGATION:
            return self._start_navigation(goal)
        if mode == MODE_FOLLOW:
            return self._start_follow(goal)
        if mode in (MODE_COVERAGE, MODE_OVERLAPING):
            return self._start_coverage(goal, mode)
        if mode in (MODE_BOUNDARY, MODE_VISUAL_BOUNDARY, MODE_VISUAL_RIDE_MOWING):
            return self._start_boundary(goal, mode)
        raise ValueError("planner 不支持任务模式 {}".format(mode))

    def _start_navigation(self, goal):
        target = goal.get("target_pose") or {}
        goal_xy = (float(target.get("x", 0.0)), float(target.get("y", 0.0)))
        start = self._pose()[:2]
        path, deviation = self._plan_path(start, goal_xy)
        self.plan = {
            "mode": MODE_NAVIGATION,
            "path": path,
            "endpoint": path[-1] if path else goal_xy,
            "deviation": deviation,
            "segments": [],
            "edge": [],
        }
        message = path_message(path)
        message["deviation"] = round(deviation, 4) if deviation != float("inf") else -1.0
        message["task_mode"] = MODE_NAVIGATION
        self.publish("task_plan_path", message)
        self.publish("current_path_index", {"type": "std_msgs/Int32", "data": 0})
        self.publish(
            "plan_action_goal",
            {
                "type": "usr_define_actions/NavTaskActionGoal",
                "goal": {"mode": MODE_NAVIGATION, "target_pose": goal_xy, "path_name": goal.get("path_name", "")},
                "cmd": ACTION_START,
            },
        )
        return {"mode": MODE_NAVIGATION, "points": len(path)}

    def _start_follow(self, goal):
        raw = goal.get("path") or []
        points = [(float(point[0]), float(point[1])) for point in raw if len(point) >= 2]
        if not points:
            raise ValueError("FOLLOW 模式需要非空 path")
        indexed = [(point[0], point[1], 0) for point in points]
        self.publish("follow_task_path", path_with_index(indexed, MODE_FOLLOW))
        self.publish("task_plan_path", path_message(points))
        self.plan = {"mode": MODE_FOLLOW, "path": points, "segments": [], "edge": []}
        return {"mode": MODE_FOLLOW, "points": len(points)}

    def _start_coverage(self, goal, mode):
        zone = self._resolve_zone(goal)
        # 覆盖规划的数据源是 region_manager 的区域栅格图（REQ/REP）。
        area = self._bus_call("get_grid_map", {"id": zone["id"]})
        grid_map = (area or {}).get("grid_map")
        if grid_map is None:
            raise ValueError("region_manager 未提供区域 {} 的栅格图".format(zone["id"]))
        axis = _axis_of(goal.get("bow_type", BOW_HORIZONTAL))
        segments = lane_segments(self.grid, zone, axis, only_unmowed=(mode == MODE_OVERLAPING))
        segments = [
            segment
            for segment in segments
            if self._segment_in_area_map(segment, grid_map)
        ]
        poses = []
        for index, segment in enumerate(segments):
            poses.append((segment[0][0], segment[0][1], index))
            poses.append((segment[1][0], segment[1][1], index))
        self.publish("nav_task_path", path_with_index(poses, mode))
        self.publish("current_path_index", {"type": "std_msgs/Int32", "data": 0})
        polyline = []
        for segment in segments:
            polyline.append(segment[0])
            polyline.append(segment[1])
        via = path_message(polyline)
        via["task_mode"] = mode
        self.publish("task_plan_path", via)
        self.plan = {
            "mode": mode,
            "path": polyline,
            "segments": segments,
            "edge": [],
            "zone_id": zone["id"],
            "map_source": "region_manager/get_grid_map",
            "map_area": (grid_map or {}).get("zone", {}).get("free"),
        }
        return {
            "mode": mode,
            "zone_id": zone["id"],
            "axis": axis,
            "segments": len(segments),
            "length": round(nav_bus.path_length(polyline), 3),
        }

    def _start_boundary(self, goal, mode):
        zone = self._resolve_zone(goal)
        sub_map = (self._bus_call("get_sub_map_info", {"id": zone["id"]}) or {}).get("sub_map")
        if sub_map is None:
            raise ValueError("region_manager 未提供区域 {} 的子图信息".format(zone["id"]))
        points = zone_edge_points(self.grid, zone, self._pose()[:2])
        message = path_message(points)
        message["task_mode"] = mode
        self.publish("task_plan_path", message)
        self.plan = {
            "mode": mode,
            "path": points,
            "segments": [],
            "edge": points,
            "zone_id": zone["id"],
        }
        return {"mode": mode, "zone_id": zone["id"], "points": len(points)}

    # --------------------------------------------------------- 区域图数据源
    def _bus_call(self, service, request):
        if self.bus.service(service) is None:
            return None
        return self.bus.call(service, request, client=self.name)

    @staticmethod
    def _segment_in_area_map(segment, grid_map):
        """用 region_manager 的 OccupancyGrid 校验弓字线落在区域空闲栅格上。"""
        info = (grid_map or {}).get("info") or {}
        data = (grid_map or {}).get("data") or []
        resolution = float(info.get("resolution") or 0.0)
        width = int(info.get("width") or 0)
        height = int(info.get("height") or 0)
        if resolution <= 0.0 or width <= 0 or height <= 0 or not data:
            return True
        origin = info.get("origin") or {}
        ox = float(origin.get("x", 0.0))
        oy = float(origin.get("y", 0.0))
        for ratio in (0.05, 0.5, 0.95):
            x = segment[0][0] + (segment[1][0] - segment[0][0]) * ratio
            y = segment[0][1] + (segment[1][1] - segment[0][1]) * ratio
            col = int((x - ox) / resolution)
            row = int((y - oy) / resolution)
            if not (0 <= col < width and 0 <= row < height):
                continue
            value = data[row * width + col]
            if value == 100 or value == -1:
                return False
        return True

    # --------------------------------------------------------- IPA 服务
    def _ipa_coverage(self, request):
        return self._ipa_plan(request, MODE_COVERAGE)

    def _ipa_boundary(self, request):
        return self._ipa_plan(request, MODE_BOUNDARY)

    def _ipa_override(self, request):
        return self._ipa_plan(request, MODE_OVERLAPING)

    def _ipa_plan(self, request, mode):
        goal = dict(request.get("goal") or request)
        goal.setdefault("mode", mode)
        try:
            outcome = self.on_start("ipa", goal)
        except ValueError as exc:
            return {"type": "usr_define_services/IpaGetPathResponse", "poses": [], "error": str(exc)}
        return {
            "type": "usr_define_services/IpaGetPathResponse",
            "mode": mode,
            "poses": list(self.plan.get("path") or []),
            "segments": [list(segment) for segment in self.plan.get("segments") or []],
            "outcome": outcome,
        }

    # --------------------------------------------------------- 规划工具
    def _plan_path(self, start, goal):
        grid = self.grid
        path = grid.plan(start, goal)
        if not path and grid.relax_around(start[0], start[1], 0.45):
            path = grid.plan(start, goal)
        if not path and grid.relax_around(goal[0], goal[1], 0.30):
            path = grid.plan(start, goal)
        if path:
            self._fail_streak = 0
            return path, 0.0
        near, distance = grid.plan_near(start, goal)
        if near and distance <= 0.45:
            self._fail_streak = 0
            return near, distance
        self._fail_streak = getattr(self, "_fail_streak", 0) + 1
        if self._fail_streak >= 5:
            # 连续多目标不可达：多为视觉碎片/障碍膨胀把通道封死，放开周边再试。
            self._fail_streak = 0
            grid.relax_around(start[0], start[1], 1.20)
            grid.relax_around(goal[0], goal[1], 1.20)
            path = grid.plan(start, goal)
            if path:
                self.log("连续多目标不可达，放开周边膨胀后重新规划")
                return path, 0.0
            near, distance = grid.plan_near(start, goal)
            if near:
                self.log("连续多目标不可达，改走最近可达点（偏离 {:.1f} m）".format(distance))
                return near, distance
        if near:
            return near, distance
        return [], float("inf")

    def _resolve_zone(self, goal):
        zone_id = goal.get("zone_id", goal.get("path_name"))
        zone = zone_by_id(self.grid, zone_id)
        if zone is None:
            raise ValueError("未知区域: {}".format(zone_id))
        return zone

    def publish_pose(self):
        x, y, theta = self._pose()
        self.publish("current_path_index", {"type": "std_msgs/Int32", "data": 0})
        return pose_stamped(x, y, theta)


class DockNode(BaseNavNode):
    """``fibot_nav_dock_node``：视觉沿边 / 回充 / 退桩 / 沿边覆盖。"""

    name = "dock"
    heartbeat_id = 800020
    offline_code = 600020

    def __init__(self, bus, robot, grid):
        super().__init__(bus, robot, grid)
        bus.advertise("cmd_vel", "geometry_msgs/Twist", publisher=self.name)
        bus.advertise("nav_status", "usr_define_msgs/NavigationStatus", publisher=self.name)
        bus.advertise("global_plan", "nav_msgs/Path", publisher=self.name)
        bus.advertise("egway_poses", "nav_msgs/Path", publisher=self.name)
        bus.advertise("nav_error_pub", "std_msgs/Int32", publisher=self.name)
        bus.advertise("global_pose", "geometry_msgs/PoseStamped", publisher=self.name)
        bus.advertise("current_pose", "geometry_msgs/PoseStamped", publisher=self.name)
        bus.advertise("recharge_start_pose", "nav_msgs/Path", publisher=self.name)
        bus.advertise("taskOpt", "std_msgs/String", publisher=self.name)

        # 文档 §4.3 输入话题
        self.subscribe("fuse_pose_3d", "usr_define_msgs/Odom")
        self.subscribe("combined", "geometry_msgs/PoseWithCovarianceStamped")
        self.subscribe("imu_data", "usr_define_msgs/Imu")
        self.subscribe("slip_status", "std_msgs/Int32")
        self.subscribe("dock_pose_frame", "geometry_msgs/PoseStamped")
        self.subscribe("manager_screw_mower", "std_msgs/Bool")
        self.subscribe("odom_motor_current_status", "usr_define_msgs/MotorCurrentStatusAnalysis")
        self.subscribe("detect_obsured", "std_msgs/Int32")
        self.subscribe("odom_hardware_signal", "usr_define_msgs/HardwareSignal")
        self.subscribe("is_edgeway", "std_msgs/Int32")
        self.subscribe("region_area", "std_msgs/Float32")
        self.subscribe("tf_dynamic", "tf2_msgs/TFMessage")

        self.plan = {"path": [], "edge": [], "docking_pose": None, "mark_id": 0}
        for action_name in (
            "autodock_action",
            "leave_dock_action",
            "visual_egway_action",
            "egway_coverage_action",
        ):
            server = ActionServer(
                bus,
                action_name,
                "{}_result".format(action_name),
                RESULT_DOCK,
                self,
            )
            self.actions[action_name] = server

    # --------------------------------------------------------- 动作
    def on_start(self, action_name, goal):
        if action_name == "autodock_action":
            return self._start_autodock(goal)
        if action_name == "leave_dock_action":
            return self._start_leave_dock(goal)
        if action_name in ("visual_egway_action", "egway_coverage_action"):
            return self._start_egway(goal, action_name)
        raise ValueError("dock 不支持动作 {}".format(action_name))

    def _start_autodock(self, goal):
        docking = goal.get("docking_pose") or {}
        dock_x = float(docking.get("x", world.DOCK["x"]))
        dock_y = float(docking.get("y", world.DOCK["y"]))
        start = self._pose()[:2]
        path, deviation = self._plan_path(start, (dock_x, dock_y))
        self.plan = {
            "path": path,
            "edge": [],
            "docking_pose": {"x": dock_x, "y": dock_y, "theta": float(docking.get("theta", 0.0))},
            "mark_id": int(goal.get("mark_id", 0)),
            "deviation": deviation,
        }
        self.publish("global_plan", path_message(path))
        self.publish(
            "recharge_start_pose",
            path_message(path[-1:] if path else [(dock_x, dock_y)]),
        )
        self.publish("current_pose", pose_stamped(*self._pose()))
        return {
            "action": "autodock_action",
            "points": len(path),
            "docking_pose": self.plan["docking_pose"],
            "mark_id": self.plan["mark_id"],
        }

    def _start_leave_dock(self, goal):
        x, y, theta = self._pose()
        distance = float(goal.get("distance", max(0.45, world.DOCK["r"] + 0.25)))
        exit_point = (
            x + math.cos(theta) * distance,
            y + math.sin(theta) * distance,
        )
        path, _deviation = self._plan_path((x, y), exit_point)
        if not path:
            path = [(x, y), exit_point]
        self.plan = {"path": path, "edge": [], "docking_pose": None, "mark_id": 0}
        self.publish("global_plan", path_message(path))
        self.publish("current_pose", pose_stamped(x, y, theta))
        return {"action": "leave_dock_action", "points": len(path), "exit": list(exit_point)}

    def _start_egway(self, goal, action_name):
        zone = self._resolve_zone(goal)
        points = zone_edge_points(self.grid, zone, self._pose()[:2])
        self.plan = {"path": points, "edge": points, "docking_pose": None, "mark_id": 0, "zone_id": zone["id"]}
        message = path_message(points)
        message["task_mode"] = MODE_VISUAL_BOUNDARY
        self.publish("egway_poses", message)
        self.publish("global_plan", path_message(points))
        self.publish(
            "region_area",
            {
                "type": "std_msgs/Float32",
                "data": round(zone["free"] * self.grid.cell * self.grid.cell, 2),
            },
        )
        return {
            "action": action_name,
            "zone_id": zone["id"],
            "points": len(points),
            "length": round(nav_bus.path_length(points), 3),
        }

    # --------------------------------------------------------- 工具
    def _resolve_zone(self, goal):
        zone = zone_by_id(self.grid, goal.get("docking_waypoint", goal.get("zone_id", goal.get("path_name"))))
        if zone is None:
            raise ValueError("未知区域: {}".format(goal.get("docking_waypoint")))
        return zone

    def _plan_path(self, start, goal):
        grid = self.grid
        path = grid.plan(start, goal)
        if not path and grid.relax_around(start[0], start[1], 0.45):
            path = grid.plan(start, goal)
        if not path and grid.relax_around(goal[0], goal[1], 0.30):
            path = grid.plan(start, goal)
        if path:
            self._fail_streak = 0
            return path, 0.0
        near, distance = grid.plan_near(start, goal)
        if near and distance <= 0.45:
            self._fail_streak = 0
            return near, distance
        self._fail_streak = getattr(self, "_fail_streak", 0) + 1
        if self._fail_streak >= 5:
            # 连续多目标不可达：多为视觉碎片/障碍膨胀把通道封死，放开周边再试。
            self._fail_streak = 0
            grid.relax_around(start[0], start[1], 1.20)
            grid.relax_around(goal[0], goal[1], 1.20)
            path = grid.plan(start, goal)
            if path:
                self.log("连续多目标不可达，放开周边膨胀后重新规划")
                return path, 0.0
            near, distance = grid.plan_near(start, goal)
            if near:
                self.log("连续多目标不可达，改走最近可达点（偏离 {:.1f} m）".format(distance))
                return near, distance
        if near:
            return near, distance
        return [], float("inf")

    def publish_pose(self):
        x, y, theta = self._pose()
        self.publish("current_pose", pose_stamped(x, y, theta))
        self.publish("global_pose", pose_stamped(x, y, theta))
        return None


class RegionManagerNode(BaseNavNode):
    """``fibot_region_manager``：区域地图 / 边界 / 连通性 / 地图装载。"""

    name = "region_manager"
    heartbeat_id = 800022
    offline_code = 600022

    def __init__(self, bus, robot, grid):
        super().__init__(bus, robot, grid)
        for topic, msg_type in (
            ("whole_map", "nav_msgs/OccupancyGrid"),
            ("area_map", "nav_msgs/OccupancyGrid"),
            ("area_edge_map", "nav_msgs/OccupancyGrid"),
            ("map_info", "usr_define_msgs/MapInfo"),
            ("charge_pose", "nav_msgs/Path"),
            ("region_area", "std_msgs/Float32"),
            ("is_on_connect_path", "std_msgs/Bool"),
            ("path_connected_flag", "std_msgs/Bool"),
        ):
            bus.advertise(topic, msg_type, publisher=self.name)
        bus.bind("get_grid_map", self._get_grid_map, port=20017, provider=self.name)
        bus.bind("get_sub_map_info", self._get_sub_map_info, port=20019, provider=self.name)
        bus.bind("map_load", self._map_load, port=20020, provider=self.name)
        bus.bind("get_map_info", self._get_map_info, port=20018, provider=self.name)
        bus.bind("task_trajectory", self._task_trajectory, port=20027, provider=self.name)
        # 订阅建图节点：地图保存后重新装载区域
        bus.subscribe("trajectory_map_change", self._on_map_change, subscriber=self.name,
                      msg_type="std_msgs/Bool")
        bus.subscribe("mapping_path_info", self._on_sub_map, subscriber=self.name,
                      msg_type="usr_define_msgs/SubMapInfo")
        self.load_count = 0
        self.last_mapping = None

    # --------------------------------------------------------- 订阅回调
    def _on_map_change(self, message):
        if message.get("data"):
            self._map_load({"cmd": 4})  # LOAD_WHOLE_MAP

    def _on_sub_map(self, message):
        self.last_mapping = message

    # --------------------------------------------------------- 服务
    def _map_load(self, request):
        cmd = int(request.get("cmd", 4))
        if cmd in (5, 7):  # LOAD_EMPTY_MAP / LOAD_WHOLE_MAP
            self.grid.reset_map()
        zones = self.refresh_zones()
        self.load_count += 1
        self.publish_map()
        self.log("map_load cmd={} zones={}".format(cmd, len(zones)))
        return {
            "type": "usr_define_services/MapLoadResponse",
            "result": True,
            "cmd": cmd,
            "map_id_array": [zone["id"] for zone in zones],
            "path_id_array": [zone["id"] for zone in zones],
        }

    def refresh_zones(self):
        pose = self._pose()
        if not self.grid.build_free_area((pose[0], pose[1])):
            return list(self.grid.zones)
        return self.grid.partition_zones()

    def _get_grid_map(self, request):
        zone_id = request.get("id", request.get("zone_id"))
        zone = zone_by_id(self.grid, zone_id)
        if zone is None:
            return {"type": "usr_define_services/GetAreaGridMapResponse", "grid_map": None,
                    "error": "未知区域: {}".format(zone_id)}
        return {
            "type": "usr_define_services/GetAreaGridMapResponse",
            "grid_map": self.zone_grid(zone),
        }

    def _get_sub_map_info(self, request):
        zone = zone_by_id(self.grid, request.get("id", request.get("zone_id")))
        if zone is None:
            return {"type": "usr_define_services/GetSubMapInfoResponse", "sub_map": None}
        return {"type": "usr_define_services/GetSubMapInfoResponse", "sub_map": self.sub_map(zone)}

    def _get_map_info(self, request):
        return {
            "type": "usr_define_services/MapChangeResponse",
            "result": True,
            "map_info": self.map_info(),
        }

    def _task_trajectory(self, request):
        zones = list(self.grid.zones)
        poses = []
        for zone in zones:
            poses.append((zone["cx"], zone["cy"], zone["id"]))
        return {
            "type": "usr_define_services/TaskTrajectoryResponse",
            "result": True,
            "path": path_with_index(poses, MODE_COVERAGE),
            "zone_ids": [zone["id"] for zone in zones],
        }

    # --------------------------------------------------------- 消息
    def zone_grid(self, zone):
        """按区域输出 OccupancyGrid：-1 未知/区外，0 空闲未割，50 已割，100 占用。"""
        grid = self.grid
        data = []
        for row in range(grid.rows):
            for col in range(grid.cols):
                if not grid.cell_in_zone(col, row, zone["id"]):
                    data.append(-1)
                    continue
                if grid.data[grid.index(col, row)] == 2 or grid.cell_blocked_mow(col, row):
                    data.append(100)
                elif grid.cell_mowed(col, row):
                    data.append(50)
                else:
                    data.append(0)
        return {
            "type": "nav_msgs/OccupancyGrid",
            "info": {
                "resolution": grid.cell,
                "width": grid.cols,
                "height": grid.rows,
                "origin": {"x": 0.0, "y": 0.0},
            },
            "zone": {"id": zone["id"], "name": zone["name"], "free": zone["free"]},
            "data": data,
        }

    def sub_map(self, zone):
        return {
            "type": "usr_define_msgs/SubMapInfo",
            "type_id": 1,
            "name": zone["name"],
            "obj_id": zone["id"],
            "area": round(zone["free"] * self.grid.cell * self.grid.cell, 2),
            "pose_count": 0,
            "resolution": self.grid.cell,
            "origin": {"x": zone["x0"], "y": zone["y0"]},
            "width_in_meter": round(zone["x1"] - zone["x0"], 3),
            "height_in_meter": round(zone["y1"] - zone["y0"], 3),
            "poses": [],
        }

    def map_info(self):
        zones = list(self.grid.zones)
        return {
            "type": "usr_define_msgs/MapInfo",
            "res": self.grid.cell,
            "area": round(sum(zone["free"] for zone in zones) * self.grid.cell ** 2, 2),
            "station_pose_x": world.DOCK["x"],
            "station_pose_y": world.DOCK["y"],
            "recharge_pose_x": world.DOCK["x"],
            "recharge_pose_y": world.DOCK["y"],
            "origin": {"x": 0.0, "y": 0.0},
            "height_in_meter": round(self.grid.rows * self.grid.cell, 3),
            "width_in_meter": round(self.grid.cols * self.grid.cell, 3),
            "sub_maps": [self.sub_map(zone) for zone in zones],
        }

    def publish_map(self):
        zones = list(self.grid.zones)
        self.publish("map_info", self.map_info())
        self.publish(
            "charge_pose",
            path_message([(world.DOCK["x"], world.DOCK["y"])]),
        )
        self.publish(
            "whole_map",
            {
                "type": "nav_msgs/OccupancyGrid",
                "info": {
                    "resolution": self.grid.cell,
                    "width": self.grid.cols,
                    "height": self.grid.rows,
                    "origin": {"x": 0.0, "y": 0.0},
                },
                "data": [],
            },
        )
        if zones:
            current = zone_by_id(self.grid, getattr(self.grid, "current_zone_id", None))
            zone = current or zones[0]
            edge = self.bus.last("egway_poses")
            self.publish(
                "area_edge_map",
                edge or {"type": "nav_msgs/OccupancyGrid", "info": {}, "data": []},
            )
            self.publish("area_map", self.zone_grid(zone))
            self.publish(
                "region_area",
                {"type": "std_msgs/Float32", "data": round(zone["free"] * self.grid.cell ** 2, 2)},
            )

    def snapshot(self):
        data = super().snapshot()
        data["zones"] = len(self.grid.zones)
        data["map_loads"] = self.load_count
        return data


class TrajectoryMapNode(BaseNavNode):
    """``fibot_trajectory_map``：建图记录、路径记录、跟随路径生成。"""

    name = "trajectory_map"
    heartbeat_id = 800021
    offline_code = 600021

    # usr_define_services::PathRecordCmdId
    ALLOW_START_MAP = 0
    START_MAP = 1
    ALLOW_SAVE_MAP = 2
    SAVE_MAP = 3
    CANCEL_MAP = 4
    CLEAR_MAP = 5
    POINT = 6
    LOAD_MAP = 7

    def __init__(self, bus, robot, grid):
        super().__init__(bus, robot, grid)
        for topic, msg_type in (
            ("trajectory_map_change", "std_msgs/Bool"),
            ("mapping_path_info", "usr_define_msgs/SubMapInfo"),
            ("area_channge_info", "usr_define_msgs/AreaChangeInfo"),
            ("get_area_info", "std_msgs/Float32"),
        ):
            bus.advertise(topic, msg_type, publisher=self.name)
        bus.bind("record_path", self._record_path, port=20015, provider=self.name)
        bus.bind("follow_plan", self._follow_plan, port=20016, provider=self.name)
        self.mapping = False
        self.points = []
        self.last_point = None
        self.saved = {}
        self.record_interval = 0.05
        self.save_count = 0

    def set_record_interval(self, value):
        try:
            self.record_interval = max(0.01, float(value))
        except (TypeError, ValueError):
            pass

    def _record_path(self, request):
        command = int(request.get("command", self.POINT))
        pose = request.get("pose") or {}
        if command == self.START_MAP:
            self.mapping = True
            self.points = []
            self.last_point = (float(pose.get("x", 0.0)), float(pose.get("y", 0.0)))
            self.publish("mapping_path_info", self._mapping_info())
            self.log("开始建图记录")
            return {"type": "usr_define_services/PathRecordResponse", "result": True, "info": "map started"}
        if command == self.POINT:
            if self.mapping:
                x = float(pose.get("x", 0.0))
                y = float(pose.get("y", 0.0))
                if self.last_point is None:
                    self.last_point = (x, y)
                distance = math.hypot(x - self.last_point[0], y - self.last_point[1])
                if distance >= self.record_interval:
                    self.points.append({"x": round(x, 3), "y": round(y, 3),
                                        "theta": round(float(pose.get("theta", 0.0)), 3)})
                    self.last_point = (x, y)
            return {
                "type": "usr_define_services/PathRecordResponse",
                "result": True,
                "info": "recorded",
                "points": len(self.points),
            }
        if command == self.SAVE_MAP:
            self.mapping = False
            self.save_count += 1
            name = str(request.get("path_name") or "map")
            self.saved[name] = list(self.points)
            self.publish("mapping_path_info", self._mapping_info())
            self.publish("trajectory_map_change", {"type": "std_msgs/Bool", "data": True})
            self.publish(
                "get_area_info",
                {"type": "std_msgs/Float32", "data": float(len(self.points))},
            )
            self.log("保存地图 {}：{} 点".format(name, len(self.points)))
            return {
                "type": "usr_define_services/PathRecordResponse",
                "result": True,
                "info": "map saved",
                "points": len(self.points),
            }
        if command == self.CANCEL_MAP:
            self.mapping = False
            return {"type": "usr_define_services/PathRecordResponse", "result": True, "info": "canceled"}
        if command == self.CLEAR_MAP:
            self.points = []
            self.last_point = None
            return {"type": "usr_define_services/PathRecordResponse", "result": True, "info": "cleared"}
        if command == self.LOAD_MAP:
            name = str(request.get("path_name") or "map")
            self.points = list(self.saved.get(name, []))
            return {
                "type": "usr_define_services/PathRecordResponse",
                "result": True,
                "info": "loaded",
                "points": len(self.points),
            }
        return {
            "type": "usr_define_services/PathRecordResponse",
            "result": False,
            "info": "unsupported command {}".format(command),
        }

    def _follow_plan(self, request):
        name = str(request.get("follow_path_name") or "map")
        points = self.saved.get(name) or self.points
        return {
            "type": "usr_define_services/GetFollowPlanResponse",
            "plan": {
                "type": "usr_define_msgs/FollowPath",
                "poses": [
                    {"x": point.get("x", 0.0), "y": point.get("y", 0.0)} for point in points
                ],
            },
        }

    def _mapping_info(self):
        return {
            "type": "usr_define_msgs/SubMapInfo",
            "type_id": 1,
            "name": "mapping" if self.mapping else "saved",
            "pose_count": len(self.points),
            "poses": list(self.points[-200:]),
        }

    def snapshot(self):
        data = super().snapshot()
        data["mapping"] = self.mapping
        data["points"] = len(self.points)
        data["saves"] = self.save_count
        return data
