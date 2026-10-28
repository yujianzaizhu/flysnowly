"""导航节点总线：在仿真进程内模拟 ZMQ + protobuf 的话题/服务/动作语义。

接口名称、端口号、消息字段对齐《导航节点对外接口与协议说明》：
* 话题：frame0 = 话题名，frame1 = 载荷；这里用 ``(name, message)`` 语义表示，消息为 dict。
* 服务：REQ/REP，请求/应答各一个消息。
* 动作：25xxx 服务受理 + ``<action>_result`` 话题回传执行结果。

只使用标准库，消息即普通 dict，便于序列化到前端或写入测试断言。
"""

import math
from collections import deque

# --------------------------------------------------------------- 协议常量
PUB_PORT = 10001          # 话题 proxy 前端（节点发布）
SUB_PORT = 10002          # 话题 proxy 后端（节点订阅）
BIGDATA_PORT = 30000      # 大包数据端口段起点

SERVICE_PORTS = {
    "autodock_action": 25001,
    "leave_dock_action": 25002,
    "visual_egway_action": 25005,
    "egway_coverage_action": 25006,
    "nav_task_action": 25009,
    "ipaCoverPath": 20013,
    "ipaBoundPath": 20014,
    "ipaOverlapPath": 20026,
}

# usr_define_basics::ActionType
ACTION_NONE = 0
ACTION_START = 1
ACTION_CANCEL = 2
ACTION_PAUSE = 3
ACTION_CONTINUE = 4
ACTION_FINISHED = 5

# usr_define_actions::ResultStatus（服务受理应答）
RESULT_SUCCEED = 0
RESULT_REJECTED = 1
RESULT_UNSUPPORTED = 2
RESULT_FAILED = 3

RESULT_STATUS_NAMES = {
    RESULT_SUCCEED: "SUCCEED",
    RESULT_REJECTED: "REJECTED",
    RESULT_UNSUPPORTED: "UNSUPPORTED",
    RESULT_FAILED: "FAILED",
}

# actionlib_msgs::GoalStatus
GOAL_SUCCEEDED = "SUCCEEDED"
GOAL_PREEMPTED = "PREEMPTED"
GOAL_ABORTED = "ABORTED"

# usr_define_actions::TaskMode
MODE_NAVIGATION = 10
MODE_FOLLOW = 20
MODE_COVERAGE = 30
MODE_BOUNDARY = 40
MODE_OVERLAPING = 50
MODE_VISUAL_BOUNDARY = 60
MODE_VISUAL_RIDE_MOWING = 70

MODE_NAMES = {
    0: "NONE",
    10: "NAVIGATION",
    20: "FOLLOW",
    30: "COVERAGE",
    40: "BOUNDARY",
    50: "OVERLAPING",
    60: "VISUAL_BOUNDARY",
    70: "VISUAL_RIDE_MOWING",
}

# usr_define_actions::EdgeType / BowType
EDGE_DEFAULT = 0
EDGE_USER_DEFINE = 1
EDGE_RIDE_MOWING = 2

BOW_DEFAULT = 0
BOW_HORIZONTAL = 1
BOW_VERTICAL = 2

# usr_define_msgs::NavMainState / NavSubState
NAV_NONE = 0
NAV_RUNNING = 1
NAV_PAUSE = 2
NAV_FAIL = 3

SUB_NONE = 0
SUB_ESCAPE_ONGOING = 1
SUB_ESCAPE_FINISHED = 2
SUB_CROSS_ONGOING = 3
SUB_CROSS_FINISHED = 4
SUB_OBSTACLE_ONGOING = 5
SUB_OBSTACLE_FINISHED = 6
SUB_REGRESSION_FINISHED = 7
SUB_REGRESSION_FAILED = 8

NAV_MAIN_NAMES = {0: "NONE", 1: "RUNNING", 2: "PAUSE", 3: "FAIL"}
NAV_SUB_NAMES = {
    0: "NONE",
    1: "ESCAPE_ONGOING",
    2: "ESCAPE_FINISHED",
    3: "CROSS_ONGOING",
    4: "CROSS_FINISHED",
    5: "OBSTACLE_ONGOING",
    6: "OBSTACLE_FINISHED",
    7: "REGRESSION_FINISHED",
    8: "REGRESSION_FAILED",
}

CMD_NAMES = {
    ACTION_NONE: "NONE",
    ACTION_START: "START",
    ACTION_CANCEL: "CANCEL",
    ACTION_PAUSE: "PAUSE",
    ACTION_CONTINUE: "CONTINUE",
    ACTION_FINISHED: "FINISHED",
}


# --------------------------------------------------------------- 消息构造
def twist(linear, angular):
    return {
        "type": "geometry_msgs/Twist",
        "linear": {"x": round(float(linear), 4), "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": round(float(angular), 4)},
    }


def pose_stamped(x, y, theta=0.0, frame_id="map"):
    return {
        "type": "geometry_msgs/PoseStamped",
        "frame_id": frame_id,
        "pose": {
            "x": round(float(x), 4),
            "y": round(float(y), 4),
            "theta": round(float(theta), 4),
        },
    }


def pose_with_covariance(x, y, theta=0.0):
    message = pose_stamped(x, y, theta)
    message["type"] = "geometry_msgs/PoseWithCovarianceStamped"
    return message


def odom(x, y, theta, linear=0.0, angular=0.0):
    return {
        "type": "usr_define_msgs/Odom",
        "frame_id": "odom",
        "pose": {
            "x": round(float(x), 4),
            "y": round(float(y), 4),
            "theta": round(float(theta), 4),
        },
        "twist": {"linear": round(float(linear), 4), "angular": round(float(angular), 4)},
    }


def imu(angular_z):
    return {
        "type": "usr_define_msgs/Imu",
        "angular_velocity": {"x": 0.0, "y": 0.0, "z": round(float(angular_z), 4)},
        "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 0.0},
    }


def hardware_signal(collision=False, lifted=False, emergency=False, bumper=False):
    return {
        "type": "usr_define_msgs/HardwareSignal",
        "collision": bool(collision),
        "lifted": bool(lifted),
        "emergency_stop": bool(emergency),
        "bumper": bool(bumper),
    }


def tf_message(x, y, theta):
    return {
        "type": "tf2_msgs/TFMessage",
        "transforms": [
            {
                "frame_id": "map",
                "child_frame_id": "base_link",
                "translation": {
                    "x": round(float(x), 4),
                    "y": round(float(y), 4),
                    "z": 0.0,
                },
                "rotation": {"z": round(float(theta), 4)},
            }
        ],
    }


def path_message(points, frame_id="map"):
    return {
        "type": "nav_msgs/Path",
        "frame_id": frame_id,
        "poses": [
            {"x": round(float(point[0]), 4), "y": round(float(point[1]), 4)}
            for point in points
        ],
    }


def path_with_index(poses, task_mode=MODE_COVERAGE):
    """PathWithIndex：poses[i] = {x, y, index}，同 index 属于同一条弓字线。"""
    return {
        "type": "nav_msgs/PathWithIndex",
        "task_mode": int(task_mode),
        "poses": [
            {"x": round(float(item[0]), 4), "y": round(float(item[1]), 4), "index": int(item[2])}
            for item in poses
        ],
    }


def nav_status(main_state=NAV_NONE, sub_state=SUB_NONE):
    return {
        "type": "usr_define_msgs/NavigationStatus",
        "main_state": int(main_state),
        "sub_state": int(sub_state),
        "main_state_name": NAV_MAIN_NAMES.get(int(main_state), "NONE"),
        "sub_state_name": NAV_SUB_NAMES.get(int(sub_state), "NONE"),
    }


def action_response(status=RESULT_SUCCEED, msgs="", error_code=0):
    return {
        "type": "usr_define_actions/ActionResponse",
        "status": int(status),
        "status_name": RESULT_STATUS_NAMES.get(int(status), "FAILED"),
        "msgs": msgs,
        "error_code": int(error_code),
    }


def action_result(result_type, status, payload=None):
    return {
        "type": result_type,
        "status": status,
        "result": dict(payload or {}),
    }


def _preview(message):
    kind = message.get("type", "")
    if kind == "nav_msgs/Path":
        return "Path {} 点".format(len(message.get("poses") or []))
    if kind == "nav_msgs/PathWithIndex":
        indexes = {int(point.get("index", 0)) for point in message.get("poses") or []}
        return "PathWithIndex {} 点 / {} 段".format(
            len(message.get("poses") or []), len(indexes)
        )
    if kind == "geometry_msgs/Twist":
        return "v={:.2f} w={:.2f}".format(
            float(message["linear"]["x"]), float(message["angular"]["z"])
        )
    if kind == "usr_define_msgs/NavigationStatus":
        return "{}/{}".format(message.get("main_state_name"), message.get("sub_state_name"))
    if kind == "usr_define_health/ProcessHeartBeat":
        return "节点 {} 心跳".format(message.get("node"))
    if kind in (
        "usr_define_actions/NavTaskActionResult",
        "usr_define_actions/AutoDockingActionResult",
    ):
        return str(message.get("status"))
    if kind in ("std_msgs/Float32", "std_msgs/Int32", "std_msgs/String"):
        return str(message.get("data"))
    if kind == "std_msgs/Bool":
        return str(bool(message.get("data")))
    if kind == "geometry_msgs/PoseStamped":
        pose = message.get("pose") or {}
        return "({:.2f}, {:.2f})".format(float(pose.get("x", 0.0)), float(pose.get("y", 0.0)))
    if kind == "usr_define_msgs/Odom":
        pose = message.get("pose") or {}
        return "({:.2f}, {:.2f})".format(float(pose.get("x", 0.0)), float(pose.get("y", 0.0)))
    return kind


# --------------------------------------------------------------- 总线
class NavBus(object):
    """极简 PUB/SUB + REQ/REP 总线（进程内、同步、可确定性复现）。"""

    HISTORY = 12

    def __init__(self):
        self.clock = 0.0
        self._meta = {}
        self._subs = {}
        self._history = {}
        self._services = {}
        self._calls = deque(maxlen=40)
        self._counts = {}

    # --------------------------------------------------------- 话题
    def advertise(self, topic, msg_type="", publisher=None, port=PUB_PORT):
        meta = self._meta.setdefault(
            topic,
            {"type": msg_type, "port": port, "publishers": [], "subscribers": []},
        )
        if msg_type and not meta["type"]:
            meta["type"] = msg_type
        if publisher and publisher not in meta["publishers"]:
            meta["publishers"].append(publisher)
        self._history.setdefault(topic, deque(maxlen=self.HISTORY))
        return meta

    def subscribe(self, topic, callback, subscriber=None, msg_type=None, port=SUB_PORT):
        meta = self.advertise(topic, msg_type or "", port=port)
        if subscriber and subscriber not in meta["subscribers"]:
            meta["subscribers"].append(subscriber)
        self._subs.setdefault(topic, []).append(callback)
        return meta

    def publish(self, topic, message, publisher=None):
        meta = self._meta.get(topic)
        if meta is None:
            meta = self.advertise(topic, message.get("type", ""), publisher=publisher)
        elif publisher and publisher not in meta["publishers"]:
            meta["publishers"].append(publisher)
        payload = dict(message)
        payload.setdefault("stamp", round(self.clock, 3))
        self._counts[topic] = self._counts.get(topic, 0) + 1
        self._history[topic].append(payload)
        for callback in list(self._subs.get(topic, [])):
            callback(payload)
        return payload

    def last(self, topic):
        history = self._history.get(topic)
        return history[-1] if history else None

    def count(self, topic):
        return self._counts.get(topic, 0)

    def topics(self):
        return sorted(self._meta.keys())

    # --------------------------------------------------------- 服务
    def bind(self, name, handler, port=None, provider=None):
        entry = {
            "name": name,
            "port": int(port or SERVICE_PORTS.get(name, 0)),
            "provider": provider,
            "handler": handler,
            "calls": 0,
            "last_response": None,
        }
        self._services[name] = entry
        return entry

    def service(self, name):
        return self._services.get(name)

    def services(self):
        return sorted(self._services.values(), key=lambda item: item["name"])

    def call(self, name, request, client=None):
        entry = self._services.get(name)
        if entry is None:
            return action_response(RESULT_FAILED, "服务 {} 未注册".format(name))
        entry["calls"] += 1
        response = dict(entry["handler"](dict(request or {})) or {})
        entry["last_response"] = response
        self._calls.append(
            {
                "service": name,
                "port": entry["port"],
                "client": client,
                "cmd": CMD_NAMES.get(int(request.get("cmd", 0)), "-"),
                "status": response.get("status_name", "-"),
                "stamp": round(self.clock, 3),
            }
        )
        return response

    def recent_calls(self):
        return list(self._calls)

    # --------------------------------------------------------- 快照
    def snapshot(self, preview_topics=40):
        topics = []
        for name in sorted(self._meta.keys()):
            meta = self._meta[name]
            last = self.last(name)
            topics.append(
                {
                    "name": name,
                    "type": meta["type"],
                    "port": meta["port"],
                    "messages": self.count(name),
                    "publishers": list(meta["publishers"]),
                    "subscribers": list(meta["subscribers"]),
                    "last": _preview(last) if last else "-",
                }
            )
        services = [
            {
                "name": entry["name"],
                "port": entry["port"],
                "provider": entry["provider"],
                "calls": entry["calls"],
                "last_status": (entry["last_response"] or {}).get("status_name", "-"),
            }
            for entry in self.services()
        ]
        return {
            "clock": round(self.clock, 3),
            "topics": topics[:preview_topics],
            "topic_total": len(topics),
            "services": services,
            "calls": self.recent_calls()[-8:],
        }


class ActionServer(object):
    """动作服务：REP 受理 + 结果话题（文档中的"REP + result topic"两段式）。"""

    def __init__(self, bus, name, result_topic, result_type, node, port=None):
        self.bus = bus
        self.name = name
        self.result_topic = result_topic
        self.result_type = result_type
        self.node = node
        self.port = int(port or SERVICE_PORTS.get(name, 0))
        self.state = "idle"  # idle / running / paused / done
        self.goal = {}
        self.last_result = None
        self.started_at = None
        bus.advertise(result_topic, result_type, publisher=node.name)
        bus.bind(name, self._handle, port=self.port, provider=node.name)

    # --------------------------------------------------------- 受理规则
    def _handle(self, request):
        cmd = int(request.get("cmd", ACTION_NONE))
        if cmd == ACTION_START:
            if self.state == "running":
                return action_response(RESULT_REJECTED, "task is already working")
            if self.state == "paused":
                self.state = "running"
                self.node.set_status(NAV_RUNNING)
                return action_response(RESULT_SUCCEED, "now set to continue")
            return self._start(request.get("goal") or {})
        if cmd == ACTION_CANCEL:
            if self.state in ("idle", "done") and not self.goal:
                return action_response(RESULT_REJECTED, "no task to cancel")
            self.state = "done"
            self.node.set_status(NAV_NONE)
            self._publish_result(GOAL_PREEMPTED, {"message": "任务已取消"})
            return action_response(RESULT_SUCCEED, "task canceled")
        if cmd == ACTION_PAUSE:
            if self.state != "running":
                return action_response(RESULT_REJECTED, "no task is running")
            self.state = "paused"
            self.node.set_status(NAV_PAUSE)
            return action_response(RESULT_SUCCEED, "task paused")
        if cmd == ACTION_CONTINUE:
            if self.state == "paused":
                self.state = "running"
                self.node.set_status(NAV_RUNNING)
                return action_response(RESULT_SUCCEED, "task continued")
            if self.state in ("idle", "done") and self.goal:
                return self._start(self.goal)
            return action_response(RESULT_REJECTED, "no task to continue")
        if cmd == ACTION_FINISHED:
            if self.state == "idle":
                return action_response(RESULT_REJECTED, "no task is running")
            self.state = "done"
            self.node.set_status(NAV_NONE)
            self._publish_result(GOAL_SUCCEEDED, {"message": "任务结束"})
            return action_response(RESULT_SUCCEED, "task finished")
        return action_response(RESULT_UNSUPPORTED, "unsupported cmd {}".format(cmd))

    def _start(self, goal):
        self.goal = dict(goal or {})
        self.started_at = self.bus.clock
        self.node.set_status(NAV_RUNNING)
        try:
            outcome = self.node.on_start(self.name, self.goal)
        except ValueError as exc:
            self.state = "done"
            self.node.set_status(NAV_FAIL)
            self._publish_result(GOAL_ABORTED, {"message": str(exc)})
            return action_response(RESULT_FAILED, str(exc), error_code=1)
        self.state = "running"
        self.node.on_action_started(self.name, self.goal)
        response = action_response(RESULT_SUCCEED, "accepted")
        response["goal"] = dict(self.goal)
        response["outcome"] = dict(outcome or {})
        return response

    # --------------------------------------------------------- 结果
    def _publish_result(self, status, payload):
        message = action_result(self.result_type, status, payload)
        message["action"] = self.name
        message["goal"] = dict(self.goal)
        self.last_result = message
        self.bus.publish(self.result_topic, message, publisher=self.node.name)
        return message

    def succeed(self, payload=None):
        self.state = "done"
        self.node.set_status(NAV_NONE)
        return self._publish_result(GOAL_SUCCEEDED, payload)

    def fail(self, message):
        self.state = "done"
        self.node.set_status(NAV_FAIL)
        return self._publish_result(GOAL_ABORTED, {"message": str(message)})

    def quiet_reset(self):
        """仿真内部重规划：不发布结果，等价于客户端重新下发 START。"""
        self.state = "idle"

    def snapshot(self):
        return {
            "name": self.name,
            "port": self.port,
            "result_topic": self.result_topic,
            "state": self.state,
            "goal": dict(self.goal),
            "last_result": (self.last_result or {}).get("status") if self.last_result else None,
        }


def zone_by_id(grid, zone_id):
    try:
        wanted = int(zone_id)
    except (TypeError, ValueError):
        return None
    for zone in grid.zones:
        if zone["id"] == wanted:
            return zone
    return None


def polygon_area(points):
    area = 0.0
    for index, point in enumerate(points):
        nxt = points[(index + 1) % len(points)]
        area += point[0] * nxt[1] - nxt[0] * point[1]
    return 0.5 * area


def path_length(points):
    total = 0.0
    for index in range(1, len(points)):
        total += math.hypot(
            points[index][0] - points[index - 1][0],
            points[index][1] - points[index - 1][1],
        )
    return total
