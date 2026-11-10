"""读取 rosbag2（SQLite3 存储）历史日志，抽取回放需要的轨迹与话题信息。

只依赖标准库：bag 文件是 SQLite 数据库，消息体是 protobuf 二进制，
这里实现了最小可用的 protobuf wire 解析，按字段号取 double 数组，
不依赖 .proto 定义（字段号由 bag 实际内容确认）：

* ``base_pose``   field3 -> {1: position(x,y,z), 2: orientation(x,y,z,w)}
* ``odom_data``   同上
* ``cmd_vel``     field1 -> linear(x,y,z)，field2 -> angular(x,y,z)
"""

import glob
import math
import os
import sqlite3
import struct

BAG_SUFFIXES = (".bag", ".db3", ".sqlite", ".sqlite3")
POSE_TOPICS = ("base_pose", "odom_data")
# 动态坐标变换（world → base_link1 等）：孩子帧的位置就是"地图坐标系下的真实位姿"。
# base_pose / odom_data 常常是另一套局部坐标系（实测要转 ~150° 才能对上地图），
# 用它们回放会让轨迹落错地方、代价地图看着"不跟车"。
TF_TOPIC = "tf_dynamic"
TF_POSE_CHILDREN = ("base_link1", "base_link", "base_car_link", "wio_link_local")
# 选作位姿源的判据：真的在动、落在地图自由区、且和代价地图窗口中心基本重合
TF_MIN_SPAN = 2.0
TF_MIN_FREE_RATIO = 0.90
TF_MAX_WINDOW_GAP = 0.60
# 地图 / 代价地图话题（都在 Zmqmsg 表里，消息体是带 map_info 的占据栅格）
MAP_TOPICS = ("whole_map", "grid_costmap", "static_costmap")
COSTMAP_TOPIC = "grid_costmap"
MAX_COSTMAP_FRAMES = 48
MAX_POSE_SAMPLES = 40000
# 图像话题（images 表）：割草分割图 / 感知调试图，用作回放时的"视频"窗口
IMAGE_TOPIC_HINTS = ("grasssegimg", "grass_seg", "segimg", "perception_debug_image")
MAX_IMAGE_FRAMES = 2400
MAX_IMAGE_BYTES = 64 * 1024 * 1024
# 话题 → 界面上对应的图层（用于在话题列表里逐个屏蔽显示）
TOPIC_LAYERS = {
    "whole_map": "map",
    "static_costmap": "static_costmap",
    "grid_costmap": "costmap",
    "base_pose": "pose",
    "odom_data": "pose",
    "obstaclecloud": "cloud",
}
# 电量话题的关键字（日志里有就按它显示，没有就保持 100%）
BATTERY_HINTS = ("battery", "battery_state", "batt", "power", "soc", "bms")
# 障碍物点云话题（PointCloud2：x/y/z + label，按 camera 帧发布）
CLOUD_TOPIC = "obstaclecloud"
# 深度相机在车体上的安装偏移（base_link1 → depth_camera，米）
CLOUD_CAMERA_OFFSET = (0.35, 0.04)
MAX_CLOUD_FRAMES = 240
MAX_CLOUD_POINTS = 90
# 视锥角点用 z = -2 占位，不是真实障碍点
CLOUD_FRUSTUM_Z = -2.0
# 同一时刻重复的位姿（odom_data 里大量存在，TI 差 <1 ms）：只留最后一条，
# 否则插值跨度接近 0，回放里的参考点会来回抽动。
POSE_DUP_WINDOW = 0.001
# 相邻位姿的"等价速度"超过它、且位移够大 → 日志自己发生了重定位（换坐标系/重新定位），
# 不是控制误差，回放时允许同步对齐（界面画成红色跳线），不要硬追。
POSE_JUMP_SPEED = 1.5
POSE_JUMP_STEP = 0.35

WIRE_VARINT = 0
WIRE_64BIT = 1
WIRE_LENGTH = 2
WIRE_32BIT = 5


class BagError(ValueError):
    pass


def _parse_message(buf):
    """解析一条 protobuf 消息为 [(field, wire, value)]。"""
    items = []
    index = 0
    size = len(buf)
    while index < size:
        key = 0
        shift = 0
        while True:
            if index >= size:
                raise BagError("protobuf 截断")
            byte = buf[index]
            index += 1
            key |= (byte & 0x7F) << shift
            shift += 7
            if not byte & 0x80:
                break
        field = key >> 3
        wire = key & 0x07
        if wire == WIRE_VARINT:
            value = 0
            shift = 0
            while True:
                byte = buf[index]
                index += 1
                value |= (byte & 0x7F) << shift
                shift += 7
                if not byte & 0x80:
                    break
            items.append((field, wire, value))
        elif wire == WIRE_64BIT:
            items.append((field, wire, struct.unpack("<d", buf[index : index + 8])[0]))
            index += 8
        elif wire == WIRE_32BIT:
            items.append((field, wire, struct.unpack("<f", buf[index : index + 4])[0]))
            index += 4
        elif wire == WIRE_LENGTH:
            length = 0
            shift = 0
            while True:
                byte = buf[index]
                index += 1
                length |= (byte & 0x7F) << shift
                shift += 7
                if not byte & 0x80:
                    break
            items.append((field, wire, buf[index : index + length]))
            index += length
        else:
            raise BagError("不支持的 protobuf wire 类型: {}".format(wire))
    return items


def _first_bytes(items, field):
    for item_field, wire, value in items:
        if item_field == field and wire == WIRE_LENGTH:
            return value
    return None


def _numbers(items):
    """取第一条出现的数值字段（protobuf 省略零值，缺失按 0 处理）。"""
    values = {}
    for field, wire, value in items:
        if wire == WIRE_LENGTH:
            continue
        values.setdefault(field, value)
    return values


def _yaw_from_quaternion(qx, qy, qz, qw):
    return math.atan2(
        2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)
    )


def _decode_pose(blob):
    try:
        top = _parse_message(blob)
    except BagError:
        return None
    body = _first_bytes(top, 3)
    if body is None:
        return None
    try:
        payload = _parse_message(body)
    except BagError:
        return None
    position = _first_bytes(payload, 1)
    if position is None:
        return None
    try:
        pos = _numbers(_parse_message(position))
        orientation = _first_bytes(payload, 2)
        quat = _numbers(_parse_message(orientation)) if orientation else {}
    except BagError:
        return None
    qx = quat.get(1, 0.0)
    qy = quat.get(2, 0.0)
    qz = quat.get(3, 0.0)
    qw = quat.get(4, 1.0)
    return (
        pos.get(1, 0.0),
        pos.get(2, 0.0),
        _yaw_from_quaternion(qx, qy, qz, qw),
    )


def _decode_twist(blob):
    try:
        top = _parse_message(blob)
    except BagError:
        return 0.0, 0.0
    linear = 0.0
    angular = 0.0
    linear_body = _first_bytes(top, 1)
    if linear_body:
        try:
            linear = _numbers(_parse_message(linear_body)).get(1, 0.0)
        except BagError:
            linear = 0.0
    angular_body = _first_bytes(top, 2)
    if angular_body:
        try:
            angular = _numbers(_parse_message(angular_body)).get(3, 0.0)
        except BagError:
            angular = 0.0
    return linear, angular


def _decode_header_stamp(blob):
    """取消息头里的墙钟时间戳（返回 Unix 秒）；不是纪元时间（太小）就返回 None。

    两种写法都认：直接一个 int64 纳秒（本仓库的日志），或者
    ``builtin_interfaces/Time {sec, nanosec}`` 子消息（标准 ROS 2 头）。
    """
    try:
        top = _parse_message(blob)
        header = _first_bytes(top, 1)
        if header is None:
            return None
        for field, wire, value in _parse_message(header):
            if field != 2:
                continue
            if wire == WIRE_VARINT:
                return value / 1e9 if value >= 1e18 else None
            if wire == WIRE_LENGTH:
                inner = _numbers(_parse_message(value))
                seconds = inner.get(1, 0) + inner.get(2, 0) / 1e9
                return seconds if seconds >= 1e9 else None
    except (BagError, IndexError, struct.error):
        return None
    return None


def _decode_tf(blob):
    """解析 tf_dynamic：{header, child_frame, transform{translation, rotation}}。

    只关心"孩子帧在地图坐标系里的位置"，所以返回 (x, y, yaw, child)。
    """
    try:
        top = _parse_message(blob)
        body = _first_bytes(top, 1)
        if body is None:
            return None
        inner = _parse_message(body)
        child = _first_bytes(inner, 2)
        transform = _first_bytes(inner, 3)
        if transform is None:
            return None
        payload = _parse_message(transform)
        position = _first_bytes(payload, 1)
        if position is None:
            return None
        pos = _numbers(_parse_message(position))
        orientation = _first_bytes(payload, 2)
        quat = _numbers(_parse_message(orientation)) if orientation else {}
    except (BagError, IndexError, struct.error):
        return None
    name = (child or b"").decode("utf-8", "replace")
    yaw = _yaw_from_quaternion(
        quat.get(1, 0.0), quat.get(2, 0.0), quat.get(3, 0.0), quat.get(4, 1.0)
    )
    return pos.get(1, 0.0), pos.get(2, 0.0), yaw, name


def _decode_grid(blob):
    """解析占据栅格/代价地图消息：{resolution,width,height,origin,data}。"""
    try:
        top = _parse_message(blob)
    except BagError:
        return None
    info_body = _first_bytes(top, 2)
    data = _first_bytes(top, 3)
    if info_body is None or data is None:
        return None
    try:
        info = _parse_message(info_body)
    except BagError:
        return None
    numbers = _numbers(info)
    resolution = numbers.get(2)
    width = numbers.get(3)
    height = numbers.get(4)
    if not resolution or not width or not height:
        return None
    origin_x = origin_y = 0.0
    origin_body = _first_bytes(info, 5)
    if origin_body:
        try:
            pose = _parse_message(origin_body)
            position = _first_bytes(pose, 1)
            if position:
                pos = _numbers(_parse_message(position))
                origin_x = pos.get(1, 0.0)
                origin_y = pos.get(2, 0.0)
        except BagError:
            pass
    width = int(width)
    height = int(height)
    if len(data) < width * height:
        return None
    return {
        "resolution": float(resolution),
        "width": width,
        "height": height,
        "origin": (float(origin_x), float(origin_y)),
        "data": data[: width * height],
    }


def encode_cells(data, width, height, step=1, kind="occupancy"):
    """把栅格编码成前端可画的字符画（occupancy: '0' 空 / '1' 占；cost: '0'-'9'）。"""
    step = max(1, int(step))
    rows = []
    for row in range(0, height, step):
        base = row * width
        chars = []
        for col in range(0, width, step):
            value = data[base + col]
            if kind == "cost":
                chars.append("0123456789"[min(9, int(value) // 10)])
            else:
                chars.append("1" if value >= 50 else "0")
        rows.append("".join(chars))
    return rows


def summarize_grid(grid):
    data = grid["data"]
    occupied = sum(1 for value in data if value >= 50)
    cost_cells = sum(1 for value in data if 0 < value < 50)
    return {
        "resolution": round(grid["resolution"], 3),
        "width": grid["width"],
        "height": grid["height"],
        "origin": [round(grid["origin"][0], 3), round(grid["origin"][1], 3)],
        "size": [
            round(grid["width"] * grid["resolution"], 2),
            round(grid["height"] * grid["resolution"], 2),
        ],
        "occupied": occupied,
        "occupied_ratio": round(occupied / float(len(data)), 4) if data else 0.0,
        "cost_cells": cost_cells,
    }


def _standard_deviation(values, mean):
    if len(values) < 2:
        return 0.0
    total = sum((value - mean) ** 2 for value in values)
    return math.sqrt(total / float(len(values) - 1))


def _resample(points, max_points=MAX_POSE_SAMPLES):
    if len(points) <= max_points:
        return points
    step = len(points) / float(max_points)
    return [points[min(len(points) - 1, int(index * step))] for index in range(max_points)]


def _merge_duplicate_poses(points):
    """合并时间戳几乎相同的重复位姿（只留最后一条）。

    ``odom_data`` 里每条消息几乎都有 4~6% 的重复时间戳（TI 只差几十纳秒），
    留着会让线性插值的跨度趋近 0，参考点在同一时刻来回抽动，看起来就是"位姿跳变"。
    """
    if len(points) < 2:
        return points, 0
    merged = []
    dropped = 0
    for point in points:
        if merged and abs(point[0] - merged[-1][0]) <= POSE_DUP_WINDOW:
            merged[-1] = point
            dropped += 1
        else:
            merged.append(point)
    return merged, dropped


def _pose_step(previous, current):
    return math.hypot(
        current[1] - previous[1], current[2] - previous[2]
    )


def find_spikes(points):
    """单点毛刺：这一帧离前后两帧都远，前后两帧彼此却很近（定位抖一下又回来）。"""
    spikes = set()
    for index in range(1, len(points) - 1):
        before = _pose_step(points[index - 1], points[index])
        after = _pose_step(points[index], points[index + 1])
        bridge = _pose_step(points[index - 1], points[index + 1])
        if before >= POSE_JUMP_STEP and after >= POSE_JUMP_STEP and bridge < POSE_JUMP_STEP:
            spikes.add(index)
    return spikes


def find_relocations(points, spikes=None):
    """找出日志自己发生重定位的采样点下标。

    判定：这一步位移够大且等价速度够快。"跳过去又跳回来"的是单点毛刺
    （见 ``find_spikes``），交给回放侧的去毛刺处理，不能当成重定位 ——
    否则回放会为了两帧噪声瞬移两次。连续几次大跳（掉台阶时常见）都要算。
    """
    spikes = find_spikes(points) if spikes is None else spikes
    cuts = []
    count = len(points)
    for index in range(1, count):
        if index in spikes or (index - 1) in spikes:
            continue
        dt = points[index][0] - points[index - 1][0]
        if dt <= 1e-6:
            continue
        step = _pose_step(points[index - 1], points[index])
        if step < POSE_JUMP_STEP or step / dt < POSE_JUMP_SPEED:
            continue
        cuts.append(index)
    return cuts


def _pick_image_topic(available):
    lowered = [(name, name.lower()) for name in available]
    for hint in IMAGE_TOPIC_HINTS:
        for name, low in lowered:
            if low == hint:
                return name
    for hint in IMAGE_TOPIC_HINTS:
        for name, low in lowered:
            if hint in low:
                return name
    for name, low in lowered:
        if "seg" in low or "grass" in low:
            return name
    for name, low in lowered:
        if "image" in low or "camera" in low:
            return name
    return available[0] if available else None


def _read_images(con, tables, start):
    """读取图像话题（images 表），按回放相对时间返回 JPEG/PNG 原始字节。"""
    if "images" not in tables:
        return None
    try:
        rows = con.execute(
            "select TOPIC, count(*) from images group by TOPIC"
        ).fetchall()
    except sqlite3.Error:
        return None
    names = [
        row[0].decode("utf-8", "replace") if isinstance(row[0], bytes) else str(row[0])
        for row in rows
    ]
    if not names:
        return None
    topic = _pick_image_topic(names)
    if topic is None:
        return None
    frames = []
    width = height = 0
    encoding = ""
    total = 0
    for timestamp, w, h, enc, data in con.execute(
        "select TI, width, height, encoding, data from images where TOPIC=? order by TI",
        (topic.encode("utf-8"),),
    ):
        if not data:
            continue
        total += len(data)
        if total > MAX_IMAGE_BYTES:
            break
        if not width:
            width = int(w or 0)
            height = int(h or 0)
            encoding = (enc.decode("utf-8", "replace") if isinstance(enc, bytes) else str(enc or ""))
        frames.append((round(float(timestamp) - start, 4), bytes(data)))
    if not frames:
        return None
    if len(frames) > MAX_IMAGE_FRAMES:
        step = len(frames) / float(MAX_IMAGE_FRAMES)
        frames = [frames[min(len(frames) - 1, int(index * step))] for index in range(MAX_IMAGE_FRAMES)]
    return {
        "topic": topic,
        "width": width,
        "height": height,
        "encoding": encoding,
        "frames": frames,
    }


def _pick_battery_topic(available):
    for hint in BATTERY_HINTS:
        for name in available:
            if name.lower() == hint:
                return name
    for hint in BATTERY_HINTS:
        for name in available:
            if hint in name.lower():
                return name
    return None


def _flatten_scalars(items, prefix="", depth=0, out=None):
    """把 protobuf 里的标量字段拍平成 {"1.2": 95.0}，最多展开三层子消息。"""
    if out is None:
        out = {}
    for field, wire, value in items:
        key = "{}{}".format(prefix, field)
        if wire == WIRE_LENGTH:
            if depth >= 3 or not value:
                continue
            try:
                inner = _parse_message(value)
            except BagError:
                continue
            _flatten_scalars(inner, key + ".", depth + 1, out)
            continue
        out[key] = float(value)
    return out


def _read_battery(con, topic, start):
    """从电量话题里取百分比时间线：[(相对时间, 电量%)]。

    消息是 protobuf，字段号因厂商而异，这里只做保守猜测：把所有标量字段都收集起来，
    取"整段都落在 0–100 之间"且最像电量的那个字段（优先有变化的字段）。
    """
    samples = []
    try:
        rows = con.execute(
            "select TI, DA from Zmqmsg where TOPIC=? order by TI", (topic.encode("utf-8"),)
        ).fetchall()
    except sqlite3.Error:
        return None
    for timestamp, blob in rows:
        if not blob:
            continue
        try:
            items = _parse_message(blob)
        except BagError:
            continue
        values = _flatten_scalars(items)
        if values:
            samples.append((round(float(timestamp) - start, 4), values))
        if len(samples) >= 2000:
            break
    if not samples:
        return None
    fields = set()
    for _t, values in samples:
        fields.update(values.keys())
    best = None
    for field in sorted(fields):
        series = [(t, values[field]) for t, values in samples if field in values]
        if len(series) < max(2, len(samples) // 2):
            continue
        if not all(-0.5 <= value <= 100.5 for _t, value in series):
            continue
        spread = max(value for _t, value in series) - min(value for _t, value in series)
        # 有变化的字段更像电量；都不变化时取第一个落在区间内的
        score = (1 if spread > 0.05 else 0, len(series))
        if best is None or score > best[0]:
            best = (score, series)
    if best is None:
        return None
    return best[1]


def _bag_entry(path, key, group):
    try:
        stat = os.stat(path)
    except OSError:
        return None
    return {
        "name": os.path.basename(path),
        "key": key,
        "group": group,
        "path": path,
        "size": stat.st_size,
        "mtime": stat.st_mtime,
    }


def list_bags(directory):
    """列出目录下的日志文件，支持"一个文件夹一组日志"（递归一层）。"""
    directory = os.path.abspath(os.path.expanduser(str(directory)))
    if not os.path.isdir(directory):
        return []
    bags = []
    for path in sorted(glob.glob(os.path.join(directory, "*"))):
        if os.path.isdir(path):
            for inner in sorted(glob.glob(os.path.join(path, "*"))):
                if not os.path.isfile(inner):
                    continue
                if os.path.splitext(inner)[1].lower() not in BAG_SUFFIXES:
                    continue
                entry = _bag_entry(
                    inner,
                    os.path.join(os.path.basename(path), os.path.basename(inner)),
                    os.path.basename(path),
                )
                if entry is not None:
                    bags.append(entry)
            continue
        if os.path.splitext(path)[1].lower() not in BAG_SUFFIXES:
            continue
        entry = _bag_entry(path, os.path.basename(path), "")
        if entry is not None:
            bags.append(entry)
    bags.sort(key=lambda item: (item["group"], item["mtime"]), reverse=False)
    return bags


def list_groups(directory):
    """目录下的日志分组（子文件夹名，根目录用空串表示）。"""
    groups = []
    for bag in list_bags(directory):
        if bag["group"] not in groups:
            groups.append(bag["group"])
    return groups


def _topic_rows(con, topic):
    return con.execute(
        "select TI, DA from Zmqmsg where TOPIC=? order by TI", (topic.encode("utf-8"),)
    )


def _costmap_information(grid):
    """这一帧有多少"能看的内容"：非致命格子占比（全致命的窗口是没信息的空窗口）。"""
    data = grid["data"]
    if not data:
        return 0.0
    return sum(1 for value in data if value < 90) / float(len(data))


def _pick_costmap_source(frames):
    """从同一话题的多种窗口里挑出"有内容"的那一组（全致命的空窗口丢掉）。"""
    if not frames:
        return []
    groups = {}
    for stamp, grid in frames:
        key = (round(grid["resolution"], 4), grid["width"], grid["height"])
        groups.setdefault(key, []).append((stamp, grid))
    key = max(
        groups,
        key=lambda item: (
            round(_costmap_information(groups[item][len(groups[item]) // 2][1]), 3),
            len(groups[item]),
        ),
    )
    return groups[key]


def _decode_pointcloud(blob):
    """解析 PointCloud2 风格的障碍物点云，返回 (点数, step, data)。"""
    try:
        top = _parse_message(blob)
    except (BagError, IndexError, struct.error):
        return None
    width = point_step = None
    data = None
    for field, wire, value in top:
        if field == 3 and wire == WIRE_VARINT:
            width = value
        elif field == 6 and wire == WIRE_VARINT:
            point_step = value
        elif field == 8 and wire == WIRE_LENGTH:
            data = value
    if not width or not point_step or data is None:
        return None
    if len(data) < width * point_step:
        return None
    return width, point_step, data


def read_obstacle_clouds(con, poses, start):
    """把障碍物点云换算到地图坐标（相机帧 → 车体 → 世界）。

    只在地图坐标系位姿可用时才有意义（否则点云不知道往哪儿放），
    返回 ``[(相对时间, [x1, y1, x2, y2, ...]), ...]``，已降采样。
    """
    if not poses or len(poses) < 2:
        return []
    frames = []
    try:
        rows = list(_topic_rows(con, CLOUD_TOPIC))
    except sqlite3.Error:
        return []
    if not rows:
        return []
    # 大多数帧只是"视锥角点"（4 个点、没有真实障碍点），所以按"有内容的帧数"
    # 上限来截断，而不是按消息条数抽稀
    camera_x, camera_y = CLOUD_CAMERA_OFFSET
    cursor = 0
    for stamp, blob in rows:
        decoded = _decode_pointcloud(blob)
        if decoded is None:
            continue
        width, point_step, data = decoded
        while cursor + 1 < len(poses) and poses[cursor + 1][0] <= stamp:
            cursor += 1
        pose = poses[cursor]
        yaw = pose[3]
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        point_stride = max(1, width // MAX_CLOUD_POINTS)
        points = []
        for index in range(0, width, point_stride):
            chunk = data[index * point_step : (index + 1) * point_step]
            if len(chunk) < 12:
                continue
            local_x, local_y, local_z = struct.unpack("<fff", chunk[:12])
            if abs(local_z - CLOUD_FRUSTUM_Z) < 1e-3:
                continue
            base_x = camera_x + local_x
            base_y = camera_y + local_y
            world_x = pose[1] + base_x * cos_yaw - base_y * sin_yaw
            world_y = pose[2] + base_x * sin_yaw + base_y * cos_yaw
            points.append(round(world_x, 2))
            points.append(round(world_y, 2))
        if points:
            frames.append((round(stamp - start, 3), points))
            if len(frames) >= MAX_CLOUD_FRAMES:
                break
    return frames


def _normalize_costmap_frames(frames, start):
    """把代价地图帧整理成回放好用的形式。

    同一个 ``grid_costmap`` 话题上会混着两种窗口，实测是 10 m@0.03 m（有内容）
    和 3 m@0.01 m（整帧 100% 致命代价、原点固定不变，等于空窗口）交替发布。
    混着叠加会看起来"一帧大、一帧小、位置还对不上"，所以只保留**最有信息量**
    的那一组（全致命的空窗口在有别的选择时直接丢掉，帧数只做并列时的次序）。
    时间戳再换成相对回放时间：帧间隔并不均匀，按索引比例取帧会和位姿对不上。
    """
    picked = _pick_costmap_source(frames)
    step = max(1, len(picked) // MAX_COSTMAP_FRAMES)
    return [(round(stamp - start, 3), grid) for stamp, grid in picked[::step]]


def pose_frame_offset(timeline, frames):
    """用局部代价地图反推"位姿坐标系 → 地图坐标系"的平移量。

    局部代价地图是按车滚动发布的窗口，多张窗口的中心连起来就是车在地图坐标系里
    走过的位置；把它们和同一时刻的日志位姿逐帧对齐，取中位数就得到两个坐标系之间
    的平移（``spread`` 是离散度：窗口真是"以车为中心"发布时它接近 0）。

    有了这个平移量，回放时就能把轨迹摆到日志地图里**真正走过的位置**——机器人会落在
    代价地图窗口里，窗口自然跟着车走；而窗口本身按它自己的 origin 摆，和静态地图
    逐格对齐。返回 ``None`` 表示日志里没有可用的代价地图窗口。
    """
    if not frames or len(timeline) < 2:
        return None
    offsets = []
    cursor = 0
    for stamp, grid in frames:
        while cursor + 1 < len(timeline) and timeline[cursor + 1][0] <= stamp:
            cursor += 1
        pose = timeline[cursor]
        half = grid["width"] * grid["resolution"] / 2.0
        offsets.append(
            (grid["origin"][0] + half - pose[1], grid["origin"][1] + half - pose[2])
        )
    if not offsets:
        return None
    xs = sorted(item[0] for item in offsets)
    ys = sorted(item[1] for item in offsets)
    dx = xs[len(xs) // 2]
    dy = ys[len(ys) // 2]
    dev_x = math.sqrt(sum((item - dx) ** 2 for item in xs) / len(xs))
    dev_y = math.sqrt(sum((item - dy) ** 2 for item in ys) / len(ys))
    # 摆好之后车是不是真的落在这个窗口里：这才是"窗口跟着车走"的判据
    inside = 0
    cursor = 0
    for stamp, grid in frames:
        while cursor + 1 < len(timeline) and timeline[cursor + 1][0] <= stamp:
            cursor += 1
        pose = timeline[cursor]
        width = grid["width"] * grid["resolution"]
        height = grid["height"] * grid["resolution"]
        x = pose[1] + dx
        y = pose[2] + dy
        if (
            grid["origin"][0] - 0.5 <= x <= grid["origin"][0] + width + 0.5
            and grid["origin"][1] - 0.5 <= y <= grid["origin"][1] + height + 0.5
        ):
            inside += 1
    inside_ratio = inside / float(len(frames))
    return {
        "dx": round(dx, 3),
        "dy": round(dy, 3),
        "spread": round(max(dev_x, dev_y), 3),
        "frames": len(offsets),
        "inside": round(inside_ratio, 3),
        "centered": inside_ratio >= 0.9,
    }


def _tf_track(tracks):
    """从各孩子帧里挑出"地图坐标系下的机器人位姿"。

    ``tf_dynamic`` 同一个话题里混着多个孩子帧（base_link1 / camera_link /
    wio_link_local / imu …）。机器人的那个有三个特征：真的在动、落在地图自由格上、
    并且和局部代价地图窗口中心基本重合（窗口是随车滚动的）。判据不满足就返回
    ``None``，调用方继续用 base_pose / odom_data。
    """
    best = None
    for name in TF_POSE_CHILDREN:
        points = tracks.get(name)
        if not points or len(points) < 10:
            continue
        xs = [point[1] for point in points]
        ys = [point[2] for point in points]
        span = max(max(xs) - min(xs), max(ys) - min(ys))
        sample = points[:: max(1, len(points) // 200)]
        free_ratio = _free_ratio_of(tracks.get("_map"), sample)
        gap = _window_gap(tracks.get("_frames"), points)
        score = (free_ratio >= TF_MIN_FREE_RATIO, span)
        if span < TF_MIN_SPAN or free_ratio < TF_MIN_FREE_RATIO:
            continue
        if gap is not None and gap > TF_MAX_WINDOW_GAP:
            continue
        candidate = (score, name, span, free_ratio, gap)
        if best is None or candidate[0] > best[0]:
            best = candidate
    if best is None:
        return None
    return {
        "name": best[1],
        "span": round(best[2], 2),
        "free_ratio": round(best[3], 3),
        "window_gap": None if best[4] is None else round(best[4], 2),
    }


def _free_ratio_of(grid, points):
    if not grid or not points:
        return 0.0
    resolution = grid["resolution"]
    width = grid["width"]
    height = grid["height"]
    ox, oy = grid["origin"]
    data = grid["data"]
    inside = 0
    for _stamp, x, y, _yaw in points:
        col = int((x - ox) / resolution)
        row = int((y - oy) / resolution)
        if 0 <= col < width and 0 <= row < height and data[row * width + col] < 50:
            inside += 1
    return inside / float(len(points))


def _window_gap(frames, points):
    """位姿与局部代价地图窗口中心的中位距离（窗口随车滚动时约等于 0）。"""
    if not frames or not points:
        return None
    cursor = 0
    gaps = []
    for stamp, grid in frames:
        while cursor + 1 < len(points) and points[cursor + 1][0] <= stamp:
            cursor += 1
        point = points[cursor]
        half_x = grid["width"] * grid["resolution"] / 2.0
        half_y = grid["height"] * grid["resolution"] / 2.0
        gaps.append(
            math.hypot(
                grid["origin"][0] + half_x - point[1],
                grid["origin"][1] + half_y - point[2],
            )
        )
    gaps.sort()
    return gaps[len(gaps) // 2]


def read_bag(path):
    """读取日志：返回时间轴、位姿轨迹、速度指令与话题统计。"""
    path = os.path.abspath(os.path.expanduser(str(path)))
    if not os.path.isfile(path):
        raise BagError("日志文件不存在: {}".format(os.path.basename(path)))
    try:
        con = sqlite3.connect("file:{}?mode=ro".format(path), uri=True)
    except sqlite3.Error as exc:
        raise BagError("无法打开日志: {}".format(exc))
    try:
        tables = {
            row[0]
            for row in con.execute("select name from sqlite_master where type='table'")
        }
        if "Zmqmsg" not in tables:
            raise BagError("不是受支持的 rosbag2(sqlite3) 日志")
        rows = con.execute(
            "select TOPIC, count(*), min(TI), max(TI) from Zmqmsg group by TOPIC"
        ).fetchall()
        topics = []
        for topic, count, first, last in rows:
            name = topic.decode("utf-8", "replace") if isinstance(topic, bytes) else str(topic)
            span = max(1e-6, float(last or first or 0.0) - float(first or 0.0))
            topics.append(
                {
                    "name": name,
                    "count": int(count),
                    "hz": round(count / span, 1) if span > 0 else 0.0,
                    "layer": TOPIC_LAYERS.get(name),
                }
            )
        topics.sort(key=lambda item: item["count"], reverse=True)

        tracks = {}
        deduped = {}
        # 日志当时的墙钟时间（消息头里的 Unix 时间戳）：界面上的"世界时间"用它，
        # 而不是仿真时钟。tf 消息头常常没有纪元时间，所以顺路在 base_pose 里取。
        wall_start = None
        for topic in POSE_TOPICS:
            points = []
            for timestamp, blob in _topic_rows(con, topic):
                pose = _decode_pose(blob)
                if pose is None:
                    continue
                if wall_start is None:
                    wall_start = _decode_header_stamp(blob)
                points.append((float(timestamp), pose[0], pose[1], pose[2]))
                if len(points) >= MAX_POSE_SAMPLES * 2:
                    break
            if points:
                points, dropped = _merge_duplicate_poses(points)
                deduped[topic] = dropped
                tracks[topic] = _resample(points)

        commands = []
        for timestamp, blob in _topic_rows(con, "cmd_vel"):
            linear, angular = _decode_twist(blob)
            commands.append((float(timestamp), linear, angular))

        maps = {}
        for topic in MAP_TOPICS:
            best = None
            frames = []
            for timestamp, blob in _topic_rows(con, topic):
                grid = _decode_grid(blob)
                if grid is None:
                    continue
                if topic == COSTMAP_TOPIC:
                    frames.append((float(timestamp), grid))
                elif best is None or grid["data"].count(100) >= best["data"].count(100):
                    # 取占据最完整的一帧作为该日志的地图
                    best = grid
            if topic == COSTMAP_TOPIC and frames:
                maps[topic] = frames
            elif best is not None:
                maps[topic] = best

        # 地图坐标系下的真实位姿（tf_dynamic 的 world → base_link1）。
        # base_pose / odom_data 常常是另一套局部坐标系，用它们回放会把轨迹摆错地方、
        # 代价地图看着"不跟车"，所以有可用的 tf 位姿就优先用它。
        tf_track = None
        tf_info = None
        if maps.get("whole_map") and any(
            item["name"] == TF_TOPIC for item in topics
        ):
            tf_tracks = {}
            for timestamp, blob in _topic_rows(con, TF_TOPIC):
                decoded = _decode_tf(blob)
                if decoded is None or decoded[3] not in TF_POSE_CHILDREN:
                    continue
                tf_tracks.setdefault(decoded[3], []).append(
                    (float(timestamp), decoded[0], decoded[1], decoded[2])
                )
            tf_tracks["_map"] = maps.get("whole_map")
            frames = maps.get(COSTMAP_TOPIC) or []
            tf_tracks["_frames"] = _pick_costmap_source(frames)
            tf_info = _tf_track(tf_tracks)
            if tf_info is not None:
                tf_info["frames"] = len(tf_tracks[tf_info["name"]])
    except sqlite3.Error as exc:
        con.close()
        raise BagError("读取日志失败: {}".format(exc))

    if not tracks and not tf_info:
        raise BagError("日志里没有位姿数据（base_pose / odom_data / tf_dynamic），无法回放")

    if tf_info is not None:
        pose_topic = tf_info["name"]
        poses = _resample(tf_tracks[pose_topic])
        pose_frame = "map"
    else:
        # 优先使用 base_pose（世界坐标），没有时退回里程计
        pose_topic = "base_pose" if "base_pose" in tracks else "odom_data"
        poses = tracks[pose_topic]
        pose_frame = "local"
    start = poses[0][0]
    try:
        images = _read_images(con, tables, start)
    except sqlite3.Error:
        images = None
    # 障碍物点云（深度相机）：只有位姿在地图坐标系里才换算得出来
    clouds = []
    if pose_frame == "map":
        clouds = read_obstacle_clouds(con, poses, start)
    battery = None
    battery_topic = _pick_battery_topic([topic["name"] for topic in topics])
    if battery_topic:
        try:
            battery = _read_battery(con, battery_topic, start)
        except (sqlite3.Error, BagError):
            battery = None
    con.close()
    timeline = [(round(t - start, 4), x, y, yaw) for t, x, y, yaw in poses]
    command_timeline = [
        (round(t - start, 4), linear, angular) for t, linear, angular in commands
        if t >= start - 0.5
    ]
    # 日志自身的重定位时刻（相对时间）：回放时同步对齐，不硬追
    spikes = find_spikes(timeline)
    relocations = [
        round(timeline[index][0], 4)
        for index in find_relocations(timeline, spikes)
        if 0.0 < timeline[index][0] <= (timeline[-1][0] if timeline else 0.0)
    ]
    frame_offset = None
    if COSTMAP_TOPIC in maps:
        maps[COSTMAP_TOPIC] = _normalize_costmap_frames(maps[COSTMAP_TOPIC], start)
        if pose_frame != "map":
            # 位姿已经在地图坐标系里时不需要再平移（反推出来的平移只会引入误差）
            frame_offset = pose_frame_offset(timeline, maps[COSTMAP_TOPIC])

    duration = timeline[-1][0] if timeline else 0.0
    xs = [point[1] for point in timeline]
    ys = [point[2] for point in timeline]
    length = 0.0
    for index in range(1, len(timeline)):
        length += math.hypot(
            timeline[index][1] - timeline[index - 1][1],
            timeline[index][2] - timeline[index - 1][2],
        )
    mean_x = sum(xs) / float(len(xs))
    mean_y = sum(ys) / float(len(ys))
    map_summary = {}
    for topic, value in maps.items():
        if topic == COSTMAP_TOPIC:
            if value:
                map_summary[topic] = summarize_grid(value[0][1])
                map_summary[topic]["frames"] = len(value)
            continue
        map_summary[topic] = summarize_grid(value)

    if images:
        summary_hz = round(len(images["frames"]) / duration, 1) if duration > 0 else 0.0
        topics.append(
            {
                "name": images["topic"],
                "count": len(images["frames"]),
                "hz": summary_hz,
                "table": "images",
                "size": "{}x{}".format(images["width"], images["height"]),
                "layer": "image",
            }
        )

    return {
        "path": path,
        "name": os.path.basename(path),
        "maps": maps,
        "map_summary": map_summary,
        "images": images,
        "battery": battery,
        "battery_topic": battery_topic if battery else None,
        "pose_topic": pose_topic,
        "pose_frame": pose_frame,
        "wall_start": None if wall_start is None else round(wall_start, 3),
        "clouds": clouds,
        "tf_pose": tf_info,
        "pose_frame_offset": frame_offset,
        "relocations": relocations,
        "duration": round(duration, 2),
        "poses": timeline,
        "commands": command_timeline,
        "topics": topics,
        "summary": {
            "poses": len(timeline),
            "rate": round(len(timeline) / duration, 1) if duration > 0 else 0.0,
            "path_length": round(length, 2),
            "bounds": {
                "x0": round(min(xs), 2),
                "x1": round(max(xs), 2),
                "y0": round(min(ys), 2),
                "y1": round(max(ys), 2),
            },
            "center": (round(mean_x, 3), round(mean_y, 3)),
            "spread": (
                round(_standard_deviation(xs, mean_x), 3),
                round(_standard_deviation(ys, mean_y), 3),
            ),
            "commands": len(command_timeline),
            "relocations": len(relocations),
            "pose_spikes": len(spikes),
            "duplicate_poses": deduped.get(pose_topic, 0),
        },
    }
