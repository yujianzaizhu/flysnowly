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
}
# 电量话题的关键字（日志里有就按它显示，没有就保持 100%）
BATTERY_HINTS = ("battery", "battery_state", "batt", "power", "soc", "bms")

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
        for topic in POSE_TOPICS:
            points = []
            for timestamp, blob in _topic_rows(con, topic):
                pose = _decode_pose(blob)
                if pose is None:
                    continue
                points.append((float(timestamp), pose[0], pose[1], pose[2]))
                if len(points) >= MAX_POSE_SAMPLES * 2:
                    break
            if points:
                tracks[topic] = _resample(points)

        commands = []
        for timestamp, blob in _topic_rows(con, "cmd_vel"):
            linear, angular = _decode_twist(blob)
            commands.append((float(timestamp), linear, angular))

        maps = {}
        costmap_frames = []
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
                step = max(1, len(frames) // MAX_COSTMAP_FRAMES)
                maps[topic] = frames[::step]
                costmap_frames = maps[topic]
            elif best is not None:
                maps[topic] = best
    except sqlite3.Error as exc:
        con.close()
        raise BagError("读取日志失败: {}".format(exc))

    if not tracks:
        raise BagError("日志里没有 base_pose / odom_data 位姿数据，无法回放")

    # 优先使用 base_pose（世界坐标），没有时退回里程计
    pose_topic = "base_pose" if "base_pose" in tracks else "odom_data"
    poses = tracks[pose_topic]
    start = poses[0][0]
    try:
        images = _read_images(con, tables, start)
    except sqlite3.Error:
        images = None
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
        },
    }
