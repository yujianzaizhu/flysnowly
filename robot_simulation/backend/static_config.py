"""解析设备 static_config，抽取导航可用的参数与接口名。

数据来源：``<repo>/data/static_config``（可用环境变量 ``FIBOT_STATIC_CONFIG`` 覆盖）。
只实现静态配置用到的 YAML 子集（嵌套 map + ``- value`` 列表 + 行内注释），
不引入第三方依赖。
"""

import os

DEFAULT_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir, "data", "static_config")
)

# 导航节点实际读取的配置文件
FILES = {
    "zmq": "zmq_comm_name.yaml",
    "nav_select": "rk3576_nav_params_select.yaml",
    "robot_nav": "rk3576_robot_nav_params_ins.yaml",
    "local_planner": "rk3576_local_planner_params.yaml",
    "ipa_coverage": "ipa_coverage_planning_server.yaml",
    "ipa_room": "ipa_room_exploration_server.yaml",
    "region_manager": "region_manager.yaml",
    "trajectory_map": "trajectory_map_generator.yaml",
    "base_config": "base_config.yaml",
}


# --------------------------------------------------------------- YAML 子集
def _strip_comment(line):
    quote = None
    for index, char in enumerate(line):
        if quote:
            if char == quote:
                quote = None
            continue
        if char in ("'", '"'):
            quote = char
        elif char == "#":
            return line[:index]
    return line


def _scalar(text):
    text = text.strip()
    if not text:
        return ""
    if (text[0] == text[-1]) and text[0] in ("'", '"') and len(text) > 1:
        return text[1:-1]
    low = text.lower()
    if low in ("true", "false"):
        return low == "true"
    try:
        return int(text)
    except ValueError:
        pass
    try:
        return float(text)
    except ValueError:
        pass
    return text


def parse_yaml(text):
    """极简 YAML 解析：支持 ``key: value``、嵌套 map 与 ``- value`` 列表。"""
    root = {}
    stack = [(-1, root)]
    lines = text.splitlines()
    index = 0
    while index < len(lines):
        raw = _strip_comment(lines[index]).rstrip()
        index += 1
        if not raw.strip():
            continue
        indent = len(raw) - len(raw.lstrip(" "))
        content = raw.strip()
        while stack and indent <= stack[-1][0]:
            stack.pop()
        parent = stack[-1][1]
        if content.startswith("- "):
            value = _scalar(content[2:])
            if not isinstance(parent, list):
                continue
            parent.append(value)
            continue
        if ":" not in content:
            continue
        key, _, rest = content.partition(":")
        key = key.strip()
        rest = rest.strip()
        if rest:
            parent[key] = _scalar(rest)
            continue
        # 判断下一层是 map 还是 list
        child = {}
        look = index
        while look < len(lines):
            probe = _strip_comment(lines[look]).rstrip()
            look += 1
            if not probe.strip():
                continue
            probe_indent = len(probe) - len(probe.lstrip(" "))
            if probe_indent <= indent:
                break
            if probe.strip().startswith("- "):
                child = []
            break
        parent[key] = child
        stack.append((indent, child))
    return root


# --------------------------------------------------------------- 读取
def _load_dir(path):
    docs = {}
    for key, name in FILES.items():
        full = os.path.join(path, name)
        if not os.path.isfile(full):
            docs[key] = {}
            continue
        with open(full, "r", encoding="utf-8", errors="replace") as handle:
            docs[key] = parse_yaml(handle.read())
    return docs


def _get(doc, *path, **kwargs):
    default = kwargs.get("default")
    node = doc
    for key in path:
        if not isinstance(node, dict) or key not in node:
            return default
        node = node[key]
    return node


def _num(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _pairs(section):
    """zmq_comm_name.yaml 的 ``key: [name, port]`` 结构 → {key: (name, port)}。"""
    result = {}
    if not isinstance(section, dict):
        return result
    for key, value in section.items():
        if isinstance(value, list) and len(value) >= 2:
            result[key] = (str(value[0]), int(_num(value[1])))
    return result


# --------------------------------------------------------------- 对外接口
def collect(directory=None):
    """解析 static_config，返回节点可用的接口名与导航参数。"""
    path = os.path.abspath(os.path.expanduser(directory or os.environ.get("FIBOT_STATIC_CONFIG") or DEFAULT_DIR))
    docs = _load_dir(path)
    zmq = docs.get("zmq") or {}
    topics = _pairs(zmq.get("topic"))
    services = _pairs(zmq.get("service"))
    actions = _pairs(zmq.get("action"))

    robot_nav = docs.get("robot_nav") or {}
    planner = docs.get("local_planner") or {}
    dwa = planner.get("DWAPlannerROS") or {}
    behavior = planner.get("Behavior") or {}
    backup = behavior.get("BackUpBehavior") or {}
    ipa = docs.get("ipa_coverage") or {}
    room = docs.get("ipa_room") or {}
    region = docs.get("region_manager") or {}
    trajectory = docs.get("trajectory_map") or {}
    select = docs.get("nav_select") or {}

    params = {
        # 速度（m/s、rad/s）
        "max_linear": _num(dwa.get("max_vel_x"), 0.40),
        "max_angular": _num(dwa.get("max_rot_vel"), 0.80),
        "nav_speed": _num(dwa.get("max_vel_x"), 0.40),
        "mow_speed": _num(robot_nav.get("state_bow_straight_vel"), 0.40),
        "edge_speed": _num(robot_nav.get("state_vis_egway_straight_vel"), 0.25),
        "coverage_speed": _num(planner.get("coverage_task_vel"), 0.35),
        "edge_task_vel": _num(planner.get("edge_task_vel"), 0.30),
        "leave_dock_back_vel": _num(robot_nav.get("state_leavedock_back_vel"), 0.20),
        "back_vel": _num(robot_nav.get("state_bow_back_vel"), 0.30),
        # 跟踪与容差
        "controller_frequency": _num(dwa.get("controller_frequency"), 10.0),
        "xy_goal_tolerance": _num(dwa.get("xy_goal_tolerance"), 0.15),
        "yaw_goal_tolerance": _num(dwa.get("yaw_goal_tolerance"), 0.20),
        "tolerance_distance": _num(planner.get("tolerance_distance"), 0.25),
        "pure_lfc": _num(robot_nav.get("follow_pure_Lfc"), 0.05),
        "path_width": _num(planner.get("path_width"), 0.80),
        # 障碍与避让
        "obstacle_distance": _num(robot_nav.get("obstacle_distance"), 0.20),
        "detect_obstacle_distance": _num(planner.get("detect_obstacle_distance"), 0.85),
        "straight_coll_dis": _num(robot_nav.get("straight_coll_dis"), 0.22),
        "forward_detection_front_len": _num(planner.get("forward_detection_front_len"), 0.41),
        "max_times_escape": _num(robot_nav.get("max_times_escape"), 60.0),
        "backup_distance": _num(backup.get("move_distance"), 0.25),
        # 覆盖 / 沿边（IPA）
        "ipa_robot_radius": _num(ipa.get("robot_radius"), 0.30),
        "coverage_radius": _num(ipa.get("coverage_radius"), -25.0),
        "shrink_distance": _num(ipa.get("shrink_distance"), 0.10),
        "grid_obstacle_offset": _num(room.get("grid_obstacle_offset"), 0.15),
        "min_cell_area": _num(room.get("min_cell_area"), 200.0),
        "path_eps": _num(room.get("path_eps"), 0.05),
        "room_exploration_algorithm": int(_num(room.get("room_exploration_algorithm"), 8)),
        "cell_visiting_order": int(_num(room.get("cell_visiting_order"), 1)),
        # 回充
        "dock_max_observe_distance": _num(robot_nav.get("dock_max_observe_distance"), 1.30),
        "dock_min_observe_distance": _num(robot_nav.get("dock_min_observe_distance"), 0.90),
        "start_dock_recharge_distance": _num(robot_nav.get("start_dock_recharge_distance"), 1.40),
        "dock_retry_count": int(_num(robot_nav.get("dock_retry_count"), 3)),
        # 地图 / 记录
        "record_interval": _num(trajectory.get("distance_interval"), 0.05),
        "record_max_size": int(_num(trajectory.get("max_size"), 100000)),
        "map_home_dir": str(trajectory.get("home_dir") or ""),
        "map_record_frame": str(trajectory.get("record_frame") or "world"),
        "map_base_link_frame": str(trajectory.get("base_link_frame") or "base_link1"),
        "region_dilate_size": _num(region.get("dilate_size"), 0.70),
        "path_dilate_size": _num(region.get("path_dilate_size"), 0.40),
        "visual_edge_dilate_size": _num(region.get("visual_edge_dilate_size"), 0.20),
        "visual_ride_mowing_dilate_size": _num(region.get("visual_ride_mowing_dilate_size"), 0.35),
        "path_end_point_dilate_size": _num(region.get("path_end_point_dilate_size"), 1.50),
        # 车体
        "robot_long": _num(robot_nav.get("robot_long"), 0.42),
        "robot_wide": _num(robot_nav.get("robot_wide"), 0.13),
        "robot_rear": _num(robot_nav.get("robot_rear"), 0.10),
        "robot_nav_file": str(select.get("robot_nav_file") or ""),
        "robot_costmap_file": str(select.get("robot_costmap_file") or ""),
    }

    return {
        "source": path,
        "files": {
            key: os.path.join(path, name)
            for key, name in FILES.items()
            if os.path.isfile(os.path.join(path, name))
        },
        "topics": {key: {"name": value[0], "port": value[1]} for key, value in topics.items()},
        "services": {key: {"name": value[0], "port": value[1]} for key, value in services.items()},
        "actions": {key: {"name": value[0], "port": value[1]} for key, value in actions.items()},
        "params": params,
    }


# 参数 → 仿真配置项（只导入对仿真有明确意义的项；其余作为节点参数展示）
# 注意：DWA 的 max_vel_x / max_rot_vel 是导航控制器的速度上限，不是底盘能力，
# 因此只作为节点控制参数（node_max_linear / node_max_angular），不覆盖底盘最大速度，
# 否则建图/脱困时转向不足反而会撞墙。
CONFIG_MAP = (
    ("nav_speed", "nav_speed"),
    ("mow_speed", "mow_speed"),
    ("edge_speed", "edge_speed"),
)

# 节点运行时参数（不进入 config.settings，供 nav_nodes 使用）
NODE_KEYS = (
    "max_linear",
    "max_angular",
    "coverage_speed",
    "edge_task_vel",
    "xy_goal_tolerance",
    "yaw_goal_tolerance",
    "tolerance_distance",
    "record_interval",
    "region_dilate_size",
    "path_dilate_size",
    "visual_edge_dilate_size",
    "dock_max_observe_distance",
    "dock_min_observe_distance",
    "start_dock_recharge_distance",
    "dock_retry_count",
    "obstacle_distance",
    "max_times_escape",
    "grid_obstacle_offset",
    "ipa_robot_radius",
    "controller_frequency",
    "robot_long",
    "robot_wide",
)

NAV_RUNTIME = {
    "source": None,
    "loaded": False,
    "params": {},
    "config_values": {},
    "topic_overrides": {},
    "service_overrides": {},
}


def node_param(name, default=None):
    params = NAV_RUNTIME.get("params") or {}
    if name in params:
        return params[name]
    return default


def apply_config(parsed):
    """把可用的导航参数写入仿真配置，并记录节点运行时参数。"""
    import config

    values = {}
    for src, dst in CONFIG_MAP:
        if src in parsed["params"]:
            values[dst] = parsed["params"][src]
    config.configure(**values)

    NAV_RUNTIME.update(
        {
            "source": parsed["source"],
            "loaded": True,
            "params": dict(parsed["params"]),
            "config_values": dict(values),
            "topic_overrides": {
                key: value["name"] for key, value in parsed["topics"].items()
            },
            "service_overrides": {
                key: (value["name"], value["port"])
                for key, value in parsed["services"].items()
            },
            "action_overrides": {
                key: (value["name"], value["port"])
                for key, value in parsed["actions"].items()
            },
        }
    )
    return NAV_RUNTIME


def load_and_apply(directory=None):
    parsed = collect(directory)
    if not parsed["files"]:
        raise ValueError("目录中没有可用的 static_config 文件: {}".format(parsed["source"]))
    return parsed, apply_config(parsed)


def summary():
    """给接口/界面用的精简结果。"""
    if not NAV_RUNTIME.get("loaded"):
        return {"loaded": False, "source": None, "params": {}, "config_values": {}}
    params = NAV_RUNTIME["params"]
    return {
        "loaded": True,
        "source": NAV_RUNTIME["source"],
        "params": {key: params[key] for key in sorted(params)},
        "config_values": dict(NAV_RUNTIME["config_values"]),
        "topic_count": len(NAV_RUNTIME.get("topic_overrides") or {}),
        "service_count": len(NAV_RUNTIME.get("service_overrides") or {}),
        "action_count": len(NAV_RUNTIME.get("action_overrides") or {}),
        "node_keys": list(NODE_KEYS),
    }
