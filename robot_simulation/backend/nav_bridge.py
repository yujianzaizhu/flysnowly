"""仿真与导航节点之间的桥接层。

职责：
1. 把仿真状态（位姿/里程计/IMU/TF/硬件信号/区域面积）发布到节点订阅的输入话题；
2. 节点发布的 ``cmd_vel`` 回填为仿真底盘速度指令（镜像），保证"节点参与仿真"；
3. 给 TaskManager 提供"调用节点动作服务 → 读取节点输出话题"的规划接口；
4. 提供节点/话题/服务快照，供 HTTP 接口与前端展示。
"""

import math

import nav_bus
import world
from nav_bus import (
    ACTION_START,
    ACTION_PAUSE,
    ACTION_CONTINUE,
    BOW_HORIZONTAL,
    BOW_VERTICAL,
    MODE_COVERAGE,
    MODE_NAVIGATION,
    MODE_OVERLAPING,
    NAV_NONE,
    NAV_PAUSE,
    NAV_RUNNING,
    RESULT_SUCCEED,
    SUB_ESCAPE_ONGOING,
    SUB_NONE,
    SUB_OBSTACLE_ONGOING,
)
from nav_nodes import DockNode, PlannerNode, RegionManagerNode, TrajectoryMapNode

EDGE_PHASES = ("zone_edge", "corner_clean", "ring", "detour", "edge_resume", "map_follow")
OBSTACLE_PHASES = ("detour", "ring")
PAUSE_PHASES = ("stopped", "paused", "charging")


class NavBridge(object):
    """把 planner / dock 两个节点接到仿真里。"""

    def __init__(self, robot, grid):
        self.robot = robot
        self.grid = grid
        self.bus = nav_bus.NavBus()
        self.planner = PlannerNode(self.bus, robot, grid)
        self.dock = DockNode(self.bus, robot, grid)
        self.region = RegionManagerNode(self.bus, robot, grid)
        self.trajectory = TrajectoryMapNode(self.bus, robot, grid)
        self.record_interval = 0.05
        self._record_pose = None
        self.enabled = True
        self.dock_kind = None
        self.last_cmd = None
        self._cmd_at = None
        self._map_at = None

    # --------------------------------------------------------- 仿真输入
    def tick(self, manager=None):
        robot = self.robot
        now = robot.sim_time
        self.bus.clock = now
        x, y, theta = robot.pose
        self.bus.publish(
            "fuse_pose_3d",
            nav_bus.odom(x, y, theta, robot.target_v, robot.target_w),
            publisher="sim",
        )
        self.bus.publish("combined", nav_bus.pose_with_covariance(x, y, theta), publisher="sim")
        self.bus.publish("imu_data", nav_bus.imu(robot.w), publisher="sim")
        self.bus.publish("tf_dynamic", nav_bus.tf_message(x, y, theta), publisher="sim")
        self.bus.publish(
            "odom_hardware_signal",
            nav_bus.hardware_signal(
                collision=manager is not None and manager._last_collisions > 0,
                emergency=bool(getattr(robot, "depleted", False)),
            ),
            publisher="sim",
        )
        phase = manager.phase if manager is not None else "idle"
        self.bus.publish(
            "is_edgeway",
            {"type": "std_msgs/Int32", "data": 1 if phase in EDGE_PHASES else 0},
            publisher="sim",
        )
        self.bus.publish(
            "region_area",
            {"type": "std_msgs/Float32", "data": self._zone_area(manager)},
            publisher="sim",
        )

        if self._cmd_at is None or now - self._cmd_at >= 0.1:
            self._cmd_at = now
            if not self.enabled:
                # 系统算法模式：cmd_vel 由仿真底盘自己产生。
                self.last_cmd = self.bus.publish(
                    "cmd_vel",
                    nav_bus.twist(robot.target_v, robot.target_w),
                    publisher="sim",
                )
            else:
                active = self.dock if self.dock_kind else self.planner
                fresh = active._last_command_at is not None and now - active._last_command_at < 0.25
                if not fresh:
                    # 节点当前没有控制输出（待机/规划中）：保持零速指令。
                    self.last_cmd = self.bus.publish(
                        "cmd_vel", nav_bus.twist(0.0, 0.0), publisher=active.name
                    )

        main_state, sub_state = self._status(manager)
        self.planner.set_status(main_state, sub_state)
        self.dock.set_status(main_state, sub_state)
        self.region.set_status(main_state, sub_state)
        self.trajectory.set_status(main_state, sub_state)
        self.planner.heartbeat()
        self.dock.heartbeat()
        self.region.heartbeat()
        self.trajectory.heartbeat()
        self.planner.publish_pose()
        self.dock.publish_pose()

        if self._map_at is None or now - self._map_at >= 2.0:
            self._map_at = now
            if self.region.bus.last("map_info") is None and self.grid.zones:
                self.region.publish_map()

    def _zone_area(self, manager):
        if manager is None or manager.zone_id is None:
            return 0.0
        zone = nav_bus.zone_by_id(self.grid, manager.zone_id)
        if zone is None:
            return 0.0
        return round(zone["free"] * self.grid.cell * self.grid.cell, 2)

    def _map_message(self):
        return {
            "type": "nav_msgs/OccupancyGrid",
            # 大包栅格数据仍由 /api/state 提供，接口上只广播元信息，避免 2 Hz 大包。
            "info": {
                "resolution": self.grid.cell,
                "width": self.grid.cols,
                "height": self.grid.rows,
                "origin": {"x": 0.0, "y": 0.0},
            },
            "data": [],
        }

    def _status(self, manager):
        if manager is None:
            return self._action_state(NAV_NONE)
        phase = manager.phase
        if phase in PAUSE_PHASES:
            main_state = NAV_PAUSE
        elif phase in ("idle", "done", "map_done"):
            # 仿真空闲时，外部下发的节点动作仍应显示为运行中。
            return self._action_state(NAV_NONE)
        else:
            main_state = NAV_RUNNING
        sub_state = SUB_NONE
        if phase in OBSTACLE_PHASES:
            sub_state = SUB_OBSTACLE_ONGOING
        elif phase == "escape":
            sub_state = SUB_ESCAPE_ONGOING
        return main_state, sub_state

    def _action_state(self, fallback):
        actions = [self.planner.task_action] + list(self.dock.actions.values())
        if any(action.state == "running" for action in actions):
            return NAV_RUNNING, SUB_NONE
        if any(action.state == "paused" for action in actions):
            return NAV_PAUSE, SUB_NONE
        return fallback, SUB_NONE

    # --------------------------------------------------------- 节点仿真开关
    def set_enabled(self, enabled):
        """开启后规划与速度指令都走节点；关闭后全部走系统算法。"""
        self.enabled = bool(enabled)
        for node in (self.planner, self.dock):
            node.log("节点仿真{}".format("已启用" if self.enabled else "已关闭"))
        self.bus.publish(
            "nav_node_mode",
            {
                "type": "std_msgs/Bool",
                "data": self.enabled,
            },
            publisher="sim",
        )
        return self.enabled

    def command(self, kind, pose, target, speed, **kwargs):
        """让节点计算 cmd_vel；节点仿真关闭时返回 None（走系统算法）。"""
        if not self.enabled:
            return None
        node = self.dock if kind == "edge" else self.planner
        command = node.command(pose, target, speed, **kwargs)
        self.last_cmd = self.bus.last("cmd_vel")
        return command

    # --------------------------------------------------------- 规划接口
    def plan_navigation(self, start, goal):
        """planner 〔nav_task_action:START mode=NAVIGATION〕→ 读 task_plan_path。"""
        if not self.enabled:
            return None
        action = self.planner.task_action
        action.quiet_reset()
        response = self.bus.call(
            "nav_task_action",
            {
                "cmd": ACTION_START,
                "goal": {
                    "mode": MODE_NAVIGATION,
                    "target_pose": {"x": float(goal[0]), "y": float(goal[1])},
                },
            },
            client="task_manager",
        )
        if response.get("status") != RESULT_SUCCEED:
            return None
        message = self.bus.last("task_plan_path") or {}
        path = [(pose["x"], pose["y"]) for pose in message.get("poses") or []]
        deviation = float(message.get("deviation", -1.0))
        if deviation < 0.0:
            deviation = float("inf")
        return {
            "path": path,
            "endpoint": path[-1] if path else (float(goal[0]), float(goal[1])),
            "deviation": deviation,
        }

    def plan_coverage(self, zone, axis="h", mode="COVERAGE"):
        """planner 〔nav_task_action mode=COVERAGE/OVERLAPING〕→ 读 nav_task_path。"""
        if not self.enabled:
            return None
        task_mode = MODE_OVERLAPING if str(mode).upper() == "OVERLAPING" else MODE_COVERAGE
        bow = BOW_VERTICAL if axis == "v" else BOW_HORIZONTAL
        action = self.planner.task_action
        action.quiet_reset()
        response = self.bus.call(
            "nav_task_action",
            {
                "cmd": ACTION_START,
                "goal": {"mode": task_mode, "zone_id": zone["id"], "bow_type": bow},
            },
            client="task_manager",
        )
        if response.get("status") != RESULT_SUCCEED:
            return None
        message = self.bus.last("nav_task_path") or {}
        grouped = {}
        for pose in message.get("poses") or []:
            grouped.setdefault(int(pose.get("index", 0)), []).append((pose["x"], pose["y"]))
        segments = [
            (points[0], points[1])
            for _index, points in sorted(grouped.items())
            if len(points) >= 2
        ]
        return segments

    def plan_visual_egway(self, zone, pose=None):
        """dock 〔visual_egway_action:START〕→ 读 egway_poses（视觉沿边）。"""
        if not self.enabled:
            return None
        action = self.dock.actions["visual_egway_action"]
        action.quiet_reset()
        response = self.bus.call(
            "visual_egway_action",
            {
                "cmd": ACTION_START,
                "goal": {"docking_waypoint": zone["id"], "zone_id": zone["id"]},
            },
            client="task_manager",
        )
        if response.get("status") != RESULT_SUCCEED:
            return None
        message = self.bus.last("egway_poses") or {}
        return [(point["x"], point["y"]) for point in message.get("poses") or []]

    def plan_boundary(self, zone):
        """planner 〔nav_task_action mode=BOUNDARY〕→ 读 task_plan_path。"""
        if not self.enabled:
            return None
        action = self.planner.task_action
        action.quiet_reset()
        response = self.bus.call(
            "nav_task_action",
            {"cmd": ACTION_START, "goal": {"mode": nav_bus.MODE_BOUNDARY, "zone_id": zone["id"]}},
            client="task_manager",
        )
        if response.get("status") != RESULT_SUCCEED:
            return None
        message = self.bus.last("task_plan_path") or {}
        return [(point["x"], point["y"]) for point in message.get("poses") or []]

    # --------------------------------------------------------- 建图 / 地图
    def begin_mapping(self):
        """trajectory_map 〔record_path: START_MAP〕。"""
        if not self.enabled:
            return False
        self._record_pose = None
        response = self.bus.call(
            "record_path",
            {
                "command": TrajectoryMapNode.START_MAP,
                "path_name": "auto_map",
                "pose": {"x": self.robot.pose[0], "y": self.robot.pose[1],
                         "theta": self.robot.pose[2]},
            },
            client="task_manager",
        )
        return bool((response or {}).get("result"))

    def record_mapping_point(self, pose):
        """trajectory_map 〔record_path: POINT〕，按 static_config 的 distance_interval 抽点。"""
        if not self.enabled or not self.trajectory.mapping:
            return False
        x, y, theta = pose
        if self._record_pose is not None:
            last_x, last_y = self._record_pose
            if math.hypot(x - last_x, y - last_y) < self.record_interval:
                return False
        self._record_pose = (x, y)
        self.bus.call(
            "record_path",
            {
                "command": TrajectoryMapNode.POINT,
                "pose": {"x": x, "y": y, "theta": theta},
            },
            client="task_manager",
        )
        return True

    def finish_mapping(self):
        """记录 SAVE_MAP → region_manager 重新装载地图与区域，返回区域列表。"""
        if not self.enabled:
            return None
        self.bus.call(
            "record_path",
            {"command": TrajectoryMapNode.SAVE_MAP, "path_name": "auto_map"},
            client="task_manager",
        )
        response = self.bus.call("map_load", {"cmd": 4}, client="trajectory_map")
        if not (response or {}).get("result"):
            return None
        self.region.publish_map()
        return self.grid.zones

    def load_map(self, cmd=4):
        if not self.enabled:
            return None
        response = self.bus.call("map_load", {"cmd": int(cmd)}, client="console")
        self.region.publish_map()
        return response

    def zone_map(self, zone):
        response = self.bus.call("get_grid_map", {"id": zone["id"]}, client="console")
        return (response or {}).get("grid_map")

    def map_info(self):
        response = self.bus.call("get_map_info", {}, client="console")
        return (response or {}).get("map_info")

    # --------------------------------------------------------- 回充 / 退桩
    def begin_autodock(self):
        """dock 〔autodock_action:START〕：发布 global_plan / recharge_start_pose。"""
        if not self.enabled:
            return False
        action = self.dock.actions["autodock_action"]
        if action.state == "running" and self.dock_kind == "autodock":
            return True
        action.quiet_reset()
        response = self.bus.call(
            "autodock_action",
            {
                "cmd": ACTION_START,
                "goal": {
                    "docking_waypoint": "dock",
                    "docking_pose": {"x": world.DOCK["x"], "y": world.DOCK["y"], "theta": 0.0},
                    "mark_id": 0,
                },
            },
            client="task_manager",
        )
        if response.get("status") != RESULT_SUCCEED:
            return False
        self.dock_kind = "autodock"
        return True

    def finish_autodock(self, ok=True, note=""):
        action = self.dock.actions["autodock_action"]
        payload = {
            "is_success": bool(ok),
            "status": note or ("已停靠充电桩" if ok else "回充中断"),
            "docking_pose": {"x": world.DOCK["x"], "y": world.DOCK["y"], "theta": 0.0},
            "mark_id": 0,
        }
        if action.state in ("running", "paused"):
            if ok:
                action.succeed(payload)
            else:
                action.fail(payload["status"])
        self.dock_kind = None

    def begin_leave_dock(self):
        """dock 〔leave_dock_action:START〕：发布退桩 global_plan。"""
        if not self.enabled:
            return False
        action = self.dock.actions["leave_dock_action"]
        action.quiet_reset()
        response = self.bus.call(
            "leave_dock_action",
            {"cmd": ACTION_START, "goal": {"distance": max(0.45, world.DOCK["r"] + 0.25)}},
            client="task_manager",
        )
        if response.get("status") != RESULT_SUCCEED:
            return False
        self.dock_kind = "leave_dock"
        return True

    def finish_leave_dock(self, ok=True):
        action = self.dock.actions["leave_dock_action"]
        if action.state in ("running", "paused"):
            if ok:
                action.succeed({"is_success": True, "status": "已离开充电桩"})
            else:
                action.fail("退桩中断")
        if self.dock_kind == "leave_dock":
            self.dock_kind = None

    def pause_planner(self):
        """planner 〔nav_task_action:PAUSE〕，用于暂停任务。"""
        if self.planner.task_action.state == "running":
            return self.bus.call("nav_task_action", {"cmd": ACTION_PAUSE}, client="task_manager")
        return None

    def continue_planner(self):
        """planner 〔nav_task_action:CONTINUE〕，用于继续任务。"""
        if self.planner.task_action.state == "paused":
            return self.bus.call("nav_task_action", {"cmd": ACTION_CONTINUE}, client="task_manager")
        return None

    def finish_planner(self, ok=True, note=""):
        action = self.planner.task_action
        if action.state in ("running", "paused"):
            if ok:
                action.succeed({"message": note or "任务结束"})
            else:
                action.fail(note or "任务中断")

    # --------------------------------------------------------- 外部动作调用
    def call_action(self, node_name, action_name, request):
        entry = self.bus.service(action_name)
        if entry is None:
            return {
                "error": "没有动作 {}；可用动作：{}".format(
                    action_name, ", ".join(sorted(self.bus._services.keys()))
                )
            }
        return self.bus.call(action_name, request, client=node_name or "console")

    def reset_actions(self):
        """复位时把仍在运行的动作全部清空（不发结果，避免刷屏）。"""
        for node in (self.planner, self.dock):
            for action in node.actions.values():
                if action.state in ("running", "paused"):
                    action.quiet_reset()
            node.set_status(NAV_NONE)
        self.dock_kind = None

    # --------------------------------------------------------- static_config
    def apply_static_config(self, directory=None):
        """解析 static_config 并把导航参数导入仿真配置。"""
        import static_config

        parsed, runtime = static_config.load_and_apply(directory)
        interval = float(runtime["params"].get("record_interval") or 0.05)
        self.record_interval = max(0.01, interval)
        self.trajectory.set_record_interval(self.record_interval)
        self.planner.log("已导入 static_config：{} 项参数".format(len(runtime["params"])))
        return static_config.summary()

    def static_summary(self):
        import static_config

        try:
            return static_config.summary()
        except Exception:  # pragma: no cover - 解析异常不影响主流程
            return {"loaded": False}

    # --------------------------------------------------------- 快照
    def snapshot(self):
        data = self.bus.snapshot()
        data["nodes"] = [
            self.planner.snapshot(),
            self.dock.snapshot(),
            self.region.snapshot(),
            self.trajectory.snapshot(),
        ]
        data["enabled"] = self.enabled
        data["static_config"] = self.static_summary()
        data["dock_action"] = self.dock_kind
        data["cmd_vel"] = self.last_cmd or nav_bus.twist(0.0, 0.0)
        data["pose"] = dict(
            zip(
                ("x", "y", "theta"),
                (round(float(value), 3) for value in self.robot.pose),
            )
        )
        return data
