"""日志读取与回放测试：用临时 sqlite bag 覆盖解析、映射与吻合度统计。"""

import datetime
import math
import os
import sqlite3
import struct
import sys
import tempfile
import unittest

BACKEND_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "backend"
)
sys.path.insert(0, BACKEND_DIR)

import bag_player  # noqa: E402
import bag_reader  # noqa: E402
import map_import  # noqa: E402
import world  # noqa: E402
from grid_map import GridMap  # noqa: E402
from robot_car import RobotCar  # noqa: E402
from tasks import TaskManager  # noqa: E402

DT = 0.02


def _varint(value):
    out = bytearray()
    while True:
        byte = value & 0x7F
        value >>= 7
        if value:
            out.append(byte | 0x80)
        else:
            out.append(byte)
            return bytes(out)


def _field_bytes(field, payload):
    return _varint(field << 3 | 2) + _varint(len(payload)) + payload


def _field_double(field, value):
    return _varint(field << 3 | 1) + struct.pack("<d", value)


def _pose_message(timestamp, x, y, yaw, wall_ns=None):
    half = yaw / 2.0
    position = _field_double(1, x) + _field_double(2, y) + _field_double(3, 0.0)
    orientation = (
        _field_double(1, 0.0)
        + _field_double(2, 0.0)
        + _field_double(3, math.sin(half))
        + _field_double(4, math.cos(half))
    )
    body = _field_bytes(1, position) + _field_bytes(2, orientation)
    # wall_ns 给定时写标准的 builtin_interfaces/Time{sec,nanosec}（Unix 纪元），
    # 否则写毫秒级的相对戳（不是纪元时间，用来验证"没有墙钟时间"的分支）
    seq = _varint(1 << 3) + _varint(1)
    if wall_ns is not None:
        seconds = int(wall_ns) // 1000000000
        nanos = int(wall_ns) % 1000000000
        stamp_field = _field_bytes(2, _varint(1 << 3) + _varint(seconds) + _varint(2 << 3) + _varint(nanos))
    else:
        stamp_field = _field_bytes(2, _varint(1 << 3) + _varint(int(timestamp * 1000)))
    header = _field_bytes(1, seq) + stamp_field
    return _field_bytes(1, header) + _field_bytes(3, body)


def _pose_stamped_message(timestamp, x, y, yaw=0.0):
    """PoseStamped：field1=header、field2=pose{1:position, 2:orientation}。"""
    header = (
        _field_bytes(1, _varint(1 << 3) + _varint(1))
        + _field_bytes(2, _varint(1 << 3) + _varint(int(timestamp * 1000)))
        + _field_bytes(3, b"world")
    )
    half = yaw / 2.0
    position = _field_double(1, x) + _field_double(2, y) + _field_double(3, 0.0)
    orientation = (
        _field_double(1, 0.0)
        + _field_double(2, 0.0)
        + _field_double(3, math.sin(half))
        + _field_double(4, math.cos(half))
    )
    body = _field_bytes(1, position) + _field_bytes(2, orientation)
    return _field_bytes(1, header) + _field_bytes(2, body)


def build_dock_bag(path, dock=(3.0, 2.0), dock_topic=b"dock_pose"):
    """一条带"充电桩位置"话题的日志（桩位是日志里明确发布的）。"""
    con = sqlite3.connect(path)
    con.execute("create table Zmqmsg (TOPIC BLOB NOT NULL, TI REAL NOT NULL, DA BLOB NOT NULL)")
    start = 1000.0
    for index in range(30):
        stamp = start + index * 0.1
        x = 0.1 * index
        con.execute(
            "insert into Zmqmsg values (?, ?, ?)",
            (b"base_pose", stamp,
             _pose_message(stamp, x, 0.0, 0.0, wall_ns=WALL_NS + index * 100000000)),
        )
        con.execute(
            "insert into Zmqmsg values (?, ?, ?)",
            (b"tf_dynamic", stamp, _tf_message(stamp, x, 2.0, 0.0)),
        )
    con.execute(
        "insert into Zmqmsg values (?, ?, ?)",
        (dock_topic, start, _pose_stamped_message(start, dock[0], dock[1])),
    )
    con.commit()
    con.close()
    return path


def _map_message(timestamp, resolution, width, height, origin, cells):
    """构造一条带 map_info 的占据栅格消息（数据一字节一格，0 自由 / 100 占据）。"""
    header = (
        _field_bytes(1, _varint(1 << 3) + _varint(1))
        + _field_bytes(2, _varint(1 << 3) + _varint(int(timestamp * 1000)))
        + _field_bytes(3, b"world")
    )
    pose = _field_bytes(
        1, _field_double(1, origin[0]) + _field_double(2, origin[1])
    )
    info = (
        _varint(2 << 3 | 5)
        + struct.pack("<f", resolution)
        + _varint(3 << 3) + _varint(width)
        + _varint(4 << 3) + _varint(height)
        + _field_bytes(5, pose)
    )
    return _field_bytes(1, header) + _field_bytes(2, info) + _field_bytes(3, cells)


def _battery_message(percent):
    """battery_state 风格消息：field1 是百分比（float）。"""
    return _field_bytes(1, _field_double(1, percent) + _field_double(2, 12.0))


def build_battery_bag(path, samples=60, dt=0.1, levels=(90.0, 80.0, 70.0)):
    con = sqlite3.connect(path)
    con.execute("create table Zmqmsg (TOPIC BLOB NOT NULL, TI REAL NOT NULL, DA BLOB NOT NULL)")
    start = 1000.0
    for index in range(samples):
        t = index * dt
        con.execute(
            "insert into Zmqmsg values (?, ?, ?)",
            (b"base_pose", start + t, _pose_message(start + t, 0.02 * index, 0.0, 0.0)),
        )
    step = max(1, samples // len(levels))
    for i, level in enumerate(levels):
        t = start + i * step * dt
        con.execute(
            "insert into Zmqmsg values (?, ?, ?)",
            (b"battery_state", t, _battery_message(level)),
        )
    con.commit()
    con.close()
    return path


def build_map_bag(path, samples=120, dt=0.1, resolution=0.05, side=120,
                  free_box=(20, 100, 20, 100), map_origin=(-1.0, -0.5)):
    """带 whole_map / grid_costmap 的 bag：位姿在局部坐标系、地图在另一坐标系。"""
    free_c0, free_c1, free_r0, free_r1 = free_box
    cells = bytearray()
    for row in range(side):
        for col in range(side):
            inside = free_c0 <= col < free_c1 and free_r0 <= row < free_r1
            cells.append(0 if inside else 100)
    cost = bytearray([100] * (40 * 40))
    for row in range(10, 30):
        for col in range(10, 30):
            cost[row * 40 + col] = 0
    con = sqlite3.connect(path)
    con.execute("create table Zmqmsg (TOPIC BLOB NOT NULL, TI REAL NOT NULL, DA BLOB NOT NULL)")
    start = 1000.0
    for index in range(samples):
        t = index * dt
        x = 0.02 * index
        yaw = 0.0
        con.execute(
            "insert into Zmqmsg values (?, ?, ?)",
            (b"base_pose", start + t, _pose_message(start + t, x, 0.0, yaw)),
        )
    con.execute(
        "insert into Zmqmsg values (?, ?, ?)",
        (b"whole_map", start, _map_message(start, resolution, side, side, map_origin, bytes(cells))),
    )
    for index in range(3):
        con.execute(
            "insert into Zmqmsg values (?, ?, ?)",
            (b"grid_costmap", start + index * 1.0,
             _map_message(start + index * 1.0, resolution, 40, 40, (-0.5, -0.5), bytes(cost))),
        )
    con.commit()
    con.close()
    return path


JPEG_A = b"\xff\xd8\xff\xe0FRAME-A\xff\xd9"
JPEG_B = b"\xff\xd8\xff\xe0FRAME-B\xff\xd9"


def _add_images(con, start, topic="perception_debug_image", frames=((0.0, JPEG_A), (5.0, JPEG_B))):
    con.execute(
        "create table images (TOPIC BLOB NOT NULL, TI REAL NOT NULL, seq INTEGER NOT NULL,"
        " stamp INTEGER NOT NULL, frame_id TEXT NOT NULL, width INTEGER NOT NULL,"
        " height INTEGER NOT NULL, encoding TEXT NOT NULL, is_bigendian INTEGER NOT NULL,"
        " step INTEGER NOT NULL, data_len INTEGER NOT NULL, data BLOB NOT NULL)"
    )
    for seq, (offset, data) in enumerate(frames):
        con.execute(
            "insert into images values (?,?,?,?,?,?,?,?,?,?,?,?)",
            (topic.encode("utf-8"), start + offset, seq, int((start + offset) * 1000), "cam",
             180, 153, "jpeg", 0, 540, len(data), data),
        )


def build_bag(path, samples=120, dt=0.1, with_images=False, origin=(0.0, 0.0)):
    con = sqlite3.connect(path)
    con.execute("create table Zmqmsg (TOPIC BLOB NOT NULL, TI REAL NOT NULL, DA BLOB NOT NULL)")
    start = 1000.0
    for index in range(samples):
        t = index * dt
        # 一段直线 + 一段原地转向，覆盖两种跟踪情形
        if index < samples // 2:
            x, y, yaw = 0.03 * index, 0.0, 0.0
        else:
            x, y = 0.03 * (samples // 2), 0.0
            yaw = math.radians(6.0 * (index - samples // 2))
        x += float(origin[0])
        y += float(origin[1])
        con.execute(
            "insert into Zmqmsg values (?, ?, ?)",
            (b"base_pose", start + t, _pose_message(start + t, x, y, yaw)),
        )
    if with_images:
        _add_images(con, start)
    con.commit()
    con.close()
    return path


def build_jumpy_bag(path, samples=120, dt=0.1):
    """带"单点毛刺"和"真重定位"的日志。

    * 第 40 个采样点整体偏移 1.5 m 后又回原位 → 单点毛刺，必须被滤掉；
    * 第 80 个采样点起永久平移 +3 m → 日志自己重定位，回放时要同步对齐。
    """
    con = sqlite3.connect(path)
    con.execute("create table Zmqmsg (TOPIC BLOB NOT NULL, TI REAL NOT NULL, DA BLOB NOT NULL)")
    start = 1000.0
    for index in range(samples):
        stamp = start + index * dt
        x = 0.02 * index
        if index == 40:
            x += 1.5
        if index >= 80:
            x += 3.0
        con.execute(
            "insert into Zmqmsg values (?, ?, ?)",
            (b"base_pose", stamp, _pose_message(stamp, x, 0.0, 0.0)),
        )
    con.commit()
    con.close()
    return path


def build_centered_costmap_bag(path, samples=80, dt=0.1):
    """代价地图窗口一直"以车为中心"滚动发布的日志（帧原点是车的位置减半个窗口）。"""
    con = sqlite3.connect(path)
    con.execute("create table Zmqmsg (TOPIC BLOB NOT NULL, TI REAL NOT NULL, DA BLOB NOT NULL)")
    start = 1000.0
    cells = bytearray([100] * (40 * 40))
    for row in range(8, 32):
        for col in range(8, 32):
            cells[row * 40 + col] = 0
    for index in range(samples):
        stamp = start + index * dt
        x = 0.02 * index
        con.execute(
            "insert into Zmqmsg values (?, ?, ?)",
            (b"base_pose", stamp, _pose_message(stamp, x, 0.0, 0.0)),
        )
    for index in range(0, samples, 5):
        stamp = start + index * dt
        x = 0.02 * index
        con.execute(
            "insert into Zmqmsg values (?, ?, ?)",
            (b"grid_costmap", stamp,
             _map_message(stamp, 0.05, 40, 40, (x - 1.0, -1.0), bytes(cells))),
        )
    # 静态地图：一条穿过 y=0 的自由带（位姿轨迹走在这条带里）
    side = 120
    whole = bytearray([100] * (side * side))
    for row in range(20, 41):
        for col in range(side):
            whole[row * side + col] = 0
    con.execute(
        "insert into Zmqmsg values (?, ?, ?)",
        (b"whole_map", start, _map_message(start, 0.05, side, side, (-1.0, -1.5), bytes(whole))),
    )
    con.commit()
    con.close()
    return path


def _tf_message(timestamp, x, y, yaw, child="base_link1"):
    """world → child 的动态坐标变换（field1=header, 2=child, 3=transform）。"""
    header = (
        _field_bytes(1, _varint(1 << 3) + _varint(1))
        + _field_bytes(2, _varint(1 << 3) + _varint(int(timestamp * 1000)))
        + _field_bytes(3, b"world")
    )
    half = yaw / 2.0
    translation = _field_double(1, x) + _field_double(2, y) + _field_double(3, 0.0)
    rotation = (
        _field_double(1, 0.0)
        + _field_double(2, 0.0)
        + _field_double(3, math.sin(half))
        + _field_double(4, math.cos(half))
    )
    transform = _field_bytes(1, translation) + _field_bytes(2, rotation)
    # 真实消息是 field1 = {header, child_frame, transform} 的包裹层
    return _field_bytes(
        1,
        _field_bytes(1, header)
        + _field_bytes(2, child.encode("utf-8"))
        + _field_bytes(3, transform),
    )


WALL_NS = 1789000000 * 1000000000  # 测试用的墙钟起点（Unix 纪元，纳秒）


# 真实的深度相机视锥角点（相机帧，z = -2 是角点占位）：近端 0.45 m、远端 1.35 m 的梯形
CLOUD_FRUSTUM = (
    (0.45, -0.13, -2.0),
    (0.45, 0.13, -2.0),
    (1.35, -0.80, -2.0),
    (1.35, 0.80, -2.0),
)


def _cloud_message(timestamp, points, frustum=CLOUD_FRUSTUM):
    """PointCloud2 风格障碍物点云：field3=width、6=point_step、8=data。"""
    header = (
        _field_bytes(1, _varint(1 << 3) + _varint(1))
        + _field_bytes(2, _varint(1 << 3) + _varint(int(timestamp * 1000)))
        + _field_bytes(3, b"world")
    )
    all_points = list(points) + list(frustum)
    blob = bytearray()
    for x, y, z in all_points:
        # 每点 32 字节：12 字节坐标 + 预留（真实消息里有 label 等字段）
        blob += struct.pack("<fff", x, y, z) + bytes(20)
    return (
        _field_bytes(1, header)
        + _varint(3 << 3) + _varint(len(all_points))
        + _varint(6 << 3) + _varint(32)
        + _field_bytes(8, bytes(blob))
    )


COSTMAP_SIDE = 120   # 代价地图窗口 120 格 × 0.05 m = 6 m


def build_map_frame_bag(path, samples=120, dt=0.1, clouds=False, wall_offset_ns=0,
                        fixed_costmap=False, include_map=True):
    """位姿在"局部坐标系"里、但 tf 里有地图坐标系真实位姿、代价地图窗口随车滚动的日志。"""
    con = sqlite3.connect(path)
    con.execute("create table Zmqmsg (TOPIC BLOB NOT NULL, TI REAL NOT NULL, DA BLOB NOT NULL)")
    start = 1000.0
    side = 120
    whole = bytearray([100] * (side * side))
    for row in range(20, 60):
        for col in range(20, 100):
            whole[row * side + col] = 0
    if include_map:
        con.execute(
            "insert into Zmqmsg values (?, ?, ?)",
            (b"whole_map", start, _map_message(start, 0.05, side, side, (0.0, 0.0), bytes(whole))),
        )
    # 窗口 6 m × 6 m（0.05 m 一格）：要能把相机前方 2 m 内的点云套进来，
    # 否则"点云只保留在代价地图范围内"会把测试点全裁掉
    cells = bytearray([0] * (COSTMAP_SIDE * COSTMAP_SIDE))
    for row in range(0, COSTMAP_SIDE):
        cells[row * COSTMAP_SIDE] = 100
    for index in range(samples):
        stamp = start + index * dt
        # 局部坐标系位姿（带 90° 旋转，和地图坐标系不同）
        local_x = 0.02 * index
        con.execute(
            "insert into Zmqmsg values (?, ?, ?)",
            (b"base_pose", stamp,
             _pose_message(
                 stamp, local_x, 0.0, 0.0,
                 wall_ns=WALL_NS + wall_offset_ns + index * int(dt * 1e9),
             )),
        )
        # 地图坐标系真实位姿：沿 x 走，y 固定在 2.0（自由带里）
        world_x = 1.5 + 0.02 * index
        con.execute(
            "insert into Zmqmsg values (?, ?, ?)",
            (b"tf_dynamic", stamp, _tf_message(stamp, world_x, 2.0, 0.0)),
        )
        if index % 10 == 0:
            con.execute(
                "insert into Zmqmsg values (?, ?, ?)",
                (b"grid_costmap", stamp,
                 _map_message(
                     stamp, 0.05, COSTMAP_SIDE, COSTMAP_SIDE,
                     (
                         1.5 - COSTMAP_SIDE * 0.05 / 2.0 if fixed_costmap else world_x - COSTMAP_SIDE * 0.05 / 2.0,
                         2.0 - COSTMAP_SIDE * 0.05 / 2.0,
                     ),
                     bytes(cells),
                 )),
            )
        if clouds:
            con.execute(
                "insert into Zmqmsg values (?, ?, ?)",
                (b"obstaclecloud", stamp,
                 _cloud_message(stamp, ((1.0, 0.5, 0.10), (1.5, -0.5, 0.20), (2.0, 0.0, 0.05)))),
            )
    con.commit()
    con.close()
    return path


def build_cloud_refresh_bag(path, hits=3, misses=6, dt=0.15, obstacle=None,
                            costmap_value=None):
    """同一位置先测到障碍、再连续几帧什么都没测到（模拟"同一个地方第二次探测"）。

    ``obstacle`` 可以换成自定义的相机帧点（测亚格精度用）；
    ``costmap_value`` 给定时同时发布 4 m × 4 m 的局部代价地图（全填这个值），
    用来验证"代价地图说这里是空地就把旧点云刷掉"。
    """
    con = sqlite3.connect(path)
    con.execute("create table Zmqmsg (TOPIC BLOB NOT NULL, TI REAL NOT NULL, DA BLOB NOT NULL)")
    start = 1000.0
    side = 120
    whole = bytearray([100] * (side * side))
    for row in range(20, 60):
        for col in range(20, 100):
            whole[row * side + col] = 0
    con.execute(
        "insert into Zmqmsg values (?, ?, ?)",
        (b"whole_map", start, _map_message(start, 0.05, side, side, (0.0, 0.0), bytes(whole))),
    )
    # 相机正前方 1.2 m 处的一个障碍（相机帧坐标，落在视锥落地面里）
    if obstacle is None:
        obstacle = ((1.20, 0.00, 0.06), (1.22, 0.05, 0.06))
    total = hits + misses
    if costmap_value is not None:
        cost_cells = bytes([costmap_value] * (40 * 40))
        for index in range(total + 2):
            stamp = start + index * dt
            con.execute(
                "insert into Zmqmsg values (?, ?, ?)",
                (b"grid_costmap", stamp,
                 _map_message(stamp, 0.1, 40, 40, (0.0, 0.0), cost_cells)),
            )
    for index in range(total):
        stamp = start + index * dt
        con.execute(
            "insert into Zmqmsg values (?, ?, ?)",
            (b"base_pose", stamp,
             _pose_message(stamp, 0.0, 0.0, 0.0, wall_ns=WALL_NS + index * int(dt * 1e9))),
        )
        con.execute(
            "insert into Zmqmsg values (?, ?, ?)",
            (b"tf_dynamic", stamp, _tf_message(stamp, 2.0, 2.0, 0.0)),
        )
        points = obstacle if index < hits else ()
        con.execute(
            "insert into Zmqmsg values (?, ?, ?)",
            (b"obstaclecloud", stamp, _cloud_message(stamp, points)),
        )
    # 位姿源要"真的在动"才会被采用（跨度 ≥2 m）：后面补一段纯位移（不带点云），
    # 这样上面那几帧点云仍然按 (2.0, 2.0) 的位姿换算。
    for index in range(1, 25):
        stamp = start + (total - 1 + index) * dt
        pose_x = 2.0 + 0.12 * index
        con.execute(
            "insert into Zmqmsg values (?, ?, ?)",
            (b"base_pose", stamp,
             _pose_message(stamp, pose_x, 0.0, 0.0, wall_ns=WALL_NS + int(stamp * 1e9))),
        )
        con.execute(
            "insert into Zmqmsg values (?, ?, ?)",
            (b"tf_dynamic", stamp, _tf_message(stamp, pose_x, 2.0, 0.0)),
        )
    con.commit()
    con.close()
    return path


class BagReaderTest(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp(prefix="robot-bag-")
        self.path = os.path.join(self.tmpdir, "sample.bag")
        build_bag(self.path)

    def tearDown(self):
        try:
            os.remove(self.path)
            os.rmdir(self.tmpdir)
        except OSError:
            pass

    def test_list_bags_finds_file(self):
        bags = bag_reader.list_bags(self.tmpdir)
        self.assertEqual([item["name"] for item in bags], ["sample.bag"])

    def test_read_bag_extracts_pose_track(self):
        info = bag_reader.read_bag(self.path)
        self.assertEqual(info["pose_topic"], "base_pose")
        self.assertEqual(info["summary"]["poses"], 120)
        self.assertAlmostEqual(info["duration"], 11.9, places=2)
        self.assertAlmostEqual(info["poses"][0][0], 0.0, places=2)
        self.assertGreater(info["summary"]["path_length"], 1.0)
        self.assertIn("base_pose", [topic["name"] for topic in info["topics"]])

    def test_replay_tracks_log_and_reports_fidelity(self):
        info = bag_reader.read_bag(self.path)
        world.select_map(1)
        robot = RobotCar()
        grid = GridMap()
        manager = TaskManager(robot, grid)
        state = manager.load_replay(info)
        self.assertTrue(state["loaded"])
        self.assertGreater(len(manager.replay.reference_path()), 10)
        manager.start_replay()
        self.assertTrue(robot.ghost)
        self.assertIsNotNone(robot.replay_limits)
        steps = 0
        while manager.replay.active and steps < int((info["duration"] + 10) / DT):
            manager.tick(DT)
            steps += 1
        result = manager.replay_state()
        self.assertGreaterEqual(result["fidelity"], 0.9)
        self.assertFalse(manager.replay.active)
        self.assertIsNone(robot.replay_limits)
        self.assertEqual(manager.phase, "idle")

    def test_replay_seek_moves_robot_to_reference(self):
        info = bag_reader.read_bag(self.path)
        world.select_map(1)
        robot = RobotCar()
        grid = GridMap()
        manager = TaskManager(robot, grid)
        manager.load_replay(info)
        manager.start_replay()
        state = manager.seek_replay(info["duration"] * 0.5)
        reference = bag_player._interp(manager.replay.track, state["time"])
        self.assertAlmostEqual(robot.pose[0], reference[0], places=3)
        self.assertAlmostEqual(robot.pose[1], reference[1], places=3)

    def test_log_map_definition_exposes_replay_transform(self):
        path = os.path.join(self.tmpdir, "map.bag")
        build_map_bag(path)
        info = bag_reader.read_bag(path)
        log_map = info["maps"]["whole_map"]
        definition = map_import.build_map_from_grid(
            log_map["resolution"], log_map["width"], log_map["height"],
            log_map["data"], grid_origin=log_map["origin"], name="log",
        )
        transform = definition["replay_transform"]
        self.assertEqual(transform["origin"], [-1.0, -0.5])
        self.assertEqual(transform["scale"], 1.0)
        self.assertGreater(transform["offset"][0], 0.0)
        self.assertGreater(len(definition["polygon"]), 2)

    def test_replay_anchor_track_inside_log_map(self):
        path = os.path.join(self.tmpdir, "map.bag")
        build_map_bag(path)
        info = bag_reader.read_bag(path)
        log_map = info["maps"]["whole_map"]
        definition = map_import.build_map_from_grid(
            log_map["resolution"], log_map["width"], log_map["height"],
            log_map["data"], grid_origin=log_map["origin"], name="log",
        )
        world.select_map(world.register_map(definition))
        robot = RobotCar()
        manager = TaskManager(robot, GridMap())
        state = manager.load_replay(info, definition["replay_transform"])
        self.assertTrue(state["loaded"])
        track = manager.replay.track
        inside = [
            point for point in track
            if world.point_in_polygon(point[1], point[2], definition["polygon"])
        ]
        self.assertEqual(len(inside), len(track))
        # 偏移量把轨迹中心对到日志地图自由区中心
        free_center = manager.replay._free_center_log()
        track_center = info["summary"]["center"]
        offset = manager.replay.offset
        self.assertAlmostEqual(free_center[0] + manager.replay.map_offset[0],
                               track_center[0] + offset[0], places=6)
        self.assertAlmostEqual(free_center[1] + manager.replay.map_offset[1],
                               track_center[1] + offset[1], places=6)

    def test_costmap_frame_uses_own_origin(self):
        """局部代价地图按自己的 origin 摆放（和静态地图同一坐标系），不跟着车身跑。

        原来把窗口中心硬拽到车身上，窗口内容却一直不动，叠加层会和真实地图
        错开十几米 —— 就是"局部地图和真实地图吻合不上"。
        """
        path = os.path.join(self.tmpdir, "map.bag")
        build_map_bag(path)
        info = bag_reader.read_bag(path)
        log_map = info["maps"]["whole_map"]
        definition = map_import.build_map_from_grid(
            log_map["resolution"], log_map["width"], log_map["height"],
            log_map["data"], grid_origin=log_map["origin"], name="log",
        )
        world.select_map(world.register_map(definition))
        manager = TaskManager(RobotCar(), GridMap())
        manager.load_replay(info, definition["replay_transform"])
        replay = manager.replay
        transform = definition["replay_transform"]
        scale = transform["scale"]
        expect_x = (-0.5) * scale + replay.map_offset[0]
        expect_y = (-0.5) * scale + replay.map_offset[1]
        for ratio in (0.0, 0.5, 1.0):
            layer = replay.costmap_at(replay.duration * ratio)
            self.assertAlmostEqual(layer["origin"][0], expect_x, places=2)
            self.assertAlmostEqual(layer["origin"][1], expect_y, places=2)

    def test_list_bags_sorted_by_recorded_time(self):
        """日志按"录制时间"排序：文件修改时间是拷贝顺序，经常和录制顺序相反。"""
        nested = os.path.join(self.tmpdir, "逆时针建图")
        os.makedirs(nested)
        for name in (
            "zmqbag_1_2026-08-29-06-34-42.bag",
            "zmqbag_2_2026-08-29-06-31-33.bag",
            "zmqbag_3_2026-08-29-06-29-34.bag",
        ):
            build_bag(os.path.join(nested, name), samples=4)
        names = [
            item["name"]
            for item in bag_reader.list_bags(self.tmpdir)
            if item["group"] == "逆时针建图"
        ]
        self.assertEqual(
            names,
            [
                "zmqbag_3_2026-08-29-06-29-34.bag",
                "zmqbag_2_2026-08-29-06-31-33.bag",
                "zmqbag_1_2026-08-29-06-34-42.bag",
            ],
        )

    def test_record_stamp_ignores_epoch_name(self):
        self.assertIsNone(bag_reader.record_stamp("zmqbag_1_1970-01-01-00-00-08.bag"))
        self.assertIsNone(bag_reader.record_stamp("clip.bag"))
        stamp = bag_reader.record_stamp("zmqbag_2_2026-08-29-06-31-33.bag")
        self.assertAlmostEqual(
            stamp, datetime.datetime(2026, 8, 29, 6, 31, 33).timestamp(), places=0
        )

    def test_list_bags_groups_subfolders(self):
        nested = os.path.join(self.tmpdir, "掉下台阶侧翻")
        os.makedirs(nested)
        build_bag(os.path.join(nested, "clip.bag"))
        root = os.path.join(self.tmpdir, "root.bag")
        build_bag(root)
        entries = bag_reader.list_bags(self.tmpdir)
        keys = {(entry["group"], entry["key"]) for entry in entries}
        self.assertIn(("掉下台阶侧翻", os.path.join("掉下台阶侧翻", "clip.bag")), keys)
        self.assertIn(("", "root.bag"), keys)
        self.assertIn("掉下台阶侧翻", bag_reader.list_groups(self.tmpdir))

    def test_read_bag_extracts_images(self):
        path = os.path.join(self.tmpdir, "img.bag")
        build_bag(path, with_images=True)
        info = bag_reader.read_bag(path)
        images = info["images"]
        self.assertEqual(images["topic"], "perception_debug_image")
        self.assertEqual((images["width"], images["height"]), (180, 153))
        self.assertEqual(len(images["frames"]), 2)
        self.assertEqual(images["frames"][0][1], JPEG_A)
        self.assertAlmostEqual(images["frames"][1][0], 5.0, places=2)
        self.assertIn("perception_debug_image", [topic["name"] for topic in info["topics"]])

    def test_image_topic_prefers_grass_seg(self):
        chosen = bag_reader._pick_image_topic(
            ["perception_debug_image", "GrassSegImg", "depth_image"]
        )
        self.assertEqual(chosen, "GrassSegImg")

    def test_replay_image_at_nearest_frame(self):
        path = os.path.join(self.tmpdir, "img.bag")
        build_bag(path, with_images=True)
        info = bag_reader.read_bag(path)
        world.select_map(1)
        manager = TaskManager(RobotCar(), GridMap())
        state = manager.load_replay(info)
        self.assertEqual(state["image_frames"], 2)
        first = manager.replay_image(0.4)
        self.assertTrue(first["image"].startswith("data:image/jpeg;base64,"))
        self.assertAlmostEqual(first["time"], 0.0, places=2)
        later = manager.replay_image(4.6)
        self.assertAlmostEqual(later["time"], 5.0, places=2)
        self.assertEqual(later["index"], 1)

    def test_clear_drops_loaded_log(self):
        path = os.path.join(self.tmpdir, "img.bag")
        build_bag(path, with_images=True)
        info = bag_reader.read_bag(path)
        world.select_map(1)
        manager = TaskManager(RobotCar(), GridMap())
        manager.load_replay(info)
        manager.clear_replay()
        state = manager.replay_state()
        self.assertFalse(state["loaded"])
        self.assertEqual(state["image_frames"], 0)
        self.assertIsNone(manager.replay_image(0.0))
        self.assertEqual(manager.phase, "idle")

    def test_battery_topic_is_parsed(self):
        path = os.path.join(self.tmpdir, "batt.bag")
        build_battery_bag(path)
        info = bag_reader.read_bag(path)
        self.assertEqual(info["battery_topic"], "battery_state")
        levels = [round(value, 1) for _t, value in info["battery"]]
        self.assertEqual(levels, [90.0, 80.0, 70.0])

    def test_dock_topic_is_parsed_when_present(self):
        """日志里明确发布了桩位话题 → 解出坐标；没有该话题 → None（回放不放桩）。"""
        path = os.path.join(self.tmpdir, "dock.bag")
        build_dock_bag(path, dock=(3.25, 2.5))
        info = bag_reader.read_bag(path)
        self.assertEqual(info["dock_topic"], "dock_pose")
        self.assertIsNotNone(info["dock"])
        self.assertAlmostEqual(info["dock"][0], 3.25, places=3)
        self.assertAlmostEqual(info["dock"][1], 2.5, places=3)

    def test_bag_without_dock_topic_has_no_dock(self):
        info = bag_reader.read_bag(self.path)
        self.assertIsNone(info["dock_topic"])
        self.assertIsNone(info["dock"])

    def test_replay_pins_battery_when_topic_missing(self):
        info = bag_reader.read_bag(self.path)
        world.select_map(1)
        robot = RobotCar()
        manager = TaskManager(robot, GridMap())
        manager.load_replay(info)
        self.assertIsNone(info["battery_topic"])
        self.assertEqual(robot.battery_lock, 100.0)
        manager.start_replay()
        for _ in range(200):
            manager.tick(DT)
        self.assertAlmostEqual(robot.battery, 100.0, places=3)
        # 停止回放后日志仍载着：电量继续钉住，不会因为仿真空转慢慢掉
        manager.stop_replay()
        self.assertEqual(robot.battery_lock, 100.0)
        manager.clear_replay()
        self.assertIsNone(robot.battery_lock)

    def test_replay_follows_battery_topic(self):
        path = os.path.join(self.tmpdir, "batt.bag")
        build_battery_bag(path)
        info = bag_reader.read_bag(path)
        world.select_map(1)
        robot = RobotCar()
        manager = TaskManager(robot, GridMap())
        manager.load_replay(info)
        # 日志有电量话题：回放期间电量直接钉在日志值上（仿真不放电），
        # 所以暂停回放、甚至回放结束后都不会掉电。
        self.assertEqual(robot.battery_lock, 90.0)
        manager.start_replay()
        readings = []
        for _ in range(240):
            manager.tick(DT * 5)
            if not manager.replay.active:
                break
            readings.append(robot.battery)
        self.assertLess(max(readings), 91.0)
        self.assertGreater(min(readings), 69.0)
        # 每一帧都必须精确等于日志里的某个值（以前会额外被仿真放电扣掉一点）
        levels = {round(value, 1) for _stamp, value in info["battery"]}
        for reading in readings:
            self.assertIn(round(reading, 1), levels)
        manager.pause_replay()
        for _ in range(200):
            manager.tick(DT)
        self.assertGreater(robot.battery, 69.0)
        manager.stop_replay()
        manager.clear_replay()
        self.assertIsNone(robot.battery_lock)

    def test_reject_missing_bag(self):
        with self.assertRaises(bag_reader.BagError):
            bag_reader.read_bag(os.path.join(self.tmpdir, "not-exist.bag"))

    def test_pose_spike_is_filtered_but_relocation_kept(self):
        path = os.path.join(self.tmpdir, "jumpy.bag")
        build_jumpy_bag(path)
        info = bag_reader.read_bag(path)
        # 只有永久平移算重定位；单点毛刺不算
        self.assertEqual(len(info["relocations"]), 1)
        self.assertAlmostEqual(info["relocations"][0], 8.0, places=1)
        world.select_map(1)
        manager = TaskManager(RobotCar(), GridMap())
        manager.load_replay(info)
        replay = manager.replay
        steps = []
        for index in range(1, len(replay.track)):
            start, end = replay.track[index - 1], replay.track[index]
            steps.append(
                (math.hypot(end[1] - start[1], end[2] - start[2]), end[0])
            )
        # 毛刺被滤掉：除了重定位那一处，相邻点位移都在 2 cm 量级
        small = [step for step in steps if step[0] < 0.1]
        self.assertEqual(len(small), len(steps) - 1)
        # 重定位保留成一次跳变
        jumps = [step for step in steps if step[0] >= 1.0]
        self.assertEqual(len(jumps), 1)
        self.assertGreater(jumps[0][0], 2.5)
        self.assertLess(jumps[0][0], 3.5)

    def test_replay_teleports_only_at_relocation(self):
        path = os.path.join(self.tmpdir, "jumpy.bag")
        build_jumpy_bag(path)
        info = bag_reader.read_bag(path)
        world.select_map(1)
        robot = RobotCar()
        manager = TaskManager(robot, GridMap())
        manager.load_replay(info)
        manager.start_replay()
        steps = 0
        biggest = 0.0
        while manager.replay.active and steps < int((info["duration"] + 10) / DT):
            previous = tuple(robot.pose)
            manager.tick(DT)
            biggest = max(biggest, math.hypot(robot.pose[0] - previous[0], robot.pose[1] - previous[1]))
            steps += 1
        # 只有日志重定位那一次同步对齐，其余每步都是连续运动
        self.assertEqual(manager.replay.teleports, 1)
        self.assertGreater(biggest, 2.0)
        self.assertGreaterEqual(manager.replay.fidelity(), 0.9)

    def test_duplicate_pose_timestamps_are_merged(self):
        path = os.path.join(self.tmpdir, "dup.bag")
        con = sqlite3.connect(path)
        con.execute("create table Zmqmsg (TOPIC BLOB NOT NULL, TI REAL NOT NULL, DA BLOB NOT NULL)")
        start = 1000.0
        for index in range(60):
            stamp = start + index * 0.1
            for repeat in range(3):
                x = 0.02 * index
                con.execute(
                    "insert into Zmqmsg values (?, ?, ?)",
                    (b"odom_data", stamp + repeat * 0.0002, _pose_message(stamp, x, 0.0, 0.0)),
                )
        con.commit()
        con.close()
        info = bag_reader.read_bag(path)
        self.assertEqual(info["summary"]["poses"], 60)
        self.assertEqual(info["summary"]["duplicate_poses"], 120)
        times = [point[0] for point in info["poses"]]
        self.assertEqual(len(times), len(set(times)))

    def test_costmap_frames_skip_all_lethal_window(self):
        """同一话题上混着"有内容"和"整帧致命"的两种窗口时，保留有内容的那个。"""
        path = os.path.join(self.tmpdir, "cost.bag")
        con = sqlite3.connect(path)
        con.execute("create table Zmqmsg (TOPIC BLOB NOT NULL, TI REAL NOT NULL, DA BLOB NOT NULL)")
        start = 1000.0
        for index in range(30):
            stamp = start + index * 0.1
            con.execute(
                "insert into Zmqmsg values (?, ?, ?)",
                (b"base_pose", stamp, _pose_message(stamp, 0.02 * index, 0.0, 0.0)),
            )
        dead = bytes([100] * (20 * 20))
        live = bytearray([0] * (40 * 40))
        for row in range(10, 30):
            for col in range(10, 30):
                live[row * 40 + col] = 100
        for index in range(6):
            stamp = start + index * 0.5
            con.execute(
                "insert into Zmqmsg values (?, ?, ?)",
                (b"grid_costmap", stamp,
                 _map_message(stamp, 0.05, 20, 20, (0.0, 0.0), dead)),
            )
        for index in range(2):
            stamp = start + 1.0 + index * 0.5
            con.execute(
                "insert into Zmqmsg values (?, ?, ?)",
                (b"grid_costmap", stamp,
                 _map_message(stamp, 0.05, 40, 40, (-0.5, -0.5), bytes(live))),
            )
        con.commit()
        con.close()
        info = bag_reader.read_bag(path)
        frames = info["maps"]["grid_costmap"]
        self.assertEqual(len(frames), 2)
        self.assertEqual(frames[0][1]["width"], 40)
        self.assertAlmostEqual(frames[0][0], 1.0, places=2)

    def test_costmap_frames_use_single_window_source(self):
        path = os.path.join(self.tmpdir, "map.bag")
        build_map_bag(path)
        info = bag_reader.read_bag(path)
        frames = info["maps"]["grid_costmap"]
        self.assertEqual(len(frames), 3)
        # 时间戳换成相对回放时间，按时间取帧才对得上位姿
        self.assertAlmostEqual(frames[0][0], 0.0, places=2)
        self.assertAlmostEqual(frames[-1][0], 2.0, places=2)

    def test_pose_frame_offset_follows_rolling_costmap(self):
        path = os.path.join(self.tmpdir, "centered.bag")
        build_centered_costmap_bag(path)
        info = bag_reader.read_bag(path)
        offset = info["pose_frame_offset"]
        self.assertIsNotNone(offset)
        self.assertTrue(offset["centered"])
        # 窗口中心就是车位：位姿坐标系和地图坐标系一致，平移量为 0
        self.assertAlmostEqual(offset["dx"], 0.0, places=2)
        self.assertAlmostEqual(offset["dy"], 0.0, places=2)
        self.assertGreaterEqual(offset["inside"], 0.9)

    def test_replay_uses_costmap_alignment_when_window_follows_robot(self):
        path = os.path.join(self.tmpdir, "centered.bag")
        build_centered_costmap_bag(path)
        info = bag_reader.read_bag(path)
        log_map = info["maps"]["whole_map"]
        definition = map_import.build_map_from_grid(
            log_map["resolution"], log_map["width"], log_map["height"],
            log_map["data"], grid_origin=log_map["origin"], name="log",
        )
        world.select_map(world.register_map(definition))
        manager = TaskManager(RobotCar(), GridMap())
        manager.load_replay(info, definition["replay_transform"])
        replay = manager.replay
        self.assertEqual(replay.alignment, "costmap")
        # 轨迹按代价地图反推的平移摆放：车落在窗口里
        layer = replay.costmap_at(0.0)
        center_x = layer["origin"][0] + layer["width"] * layer["resolution"] / 2.0
        center_y = layer["origin"][1] + layer["height"] * layer["resolution"] / 2.0
        self.assertAlmostEqual(center_x, replay.track[0][1], places=1)
        self.assertAlmostEqual(center_y, replay.track[0][2], places=1)

    def test_tf_pose_is_preferred_when_in_map_frame(self):
        """tf 里的地图坐标系位姿优先用：代价地图窗口正好套住机器人。"""
        path = os.path.join(self.tmpdir, "mapframe.bag")
        build_map_frame_bag(path)
        info = bag_reader.read_bag(path)
        self.assertEqual(info["pose_frame"], "map")
        self.assertEqual(info["pose_topic"], "base_link1")
        self.assertIsNotNone(info["tf_pose"])
        self.assertAlmostEqual(info["poses"][0][1], 1.5, places=2)
        # 窗口随车滚动 → 反推平移无用武之地
        self.assertIsNone(info["pose_frame_offset"])

    def test_wall_time_comes_from_log_headers(self):
        """世界时间用日志消息头里的墙钟时间，不是仿真时钟。"""
        path = os.path.join(self.tmpdir, "mapframe.bag")
        build_map_frame_bag(path)
        info = bag_reader.read_bag(path)
        self.assertIsNotNone(info["wall_start"])
        self.assertAlmostEqual(info["wall_start"], WALL_NS / 1e9, places=1)
        world.select_map(1)
        manager = TaskManager(RobotCar(), GridMap())
        manager.load_replay(info)
        replay = manager.replay
        first = replay.state()["wall_time"]
        self.assertAlmostEqual(first, info["wall_start"], places=1)
        target = replay.duration * 0.5
        replay.seek(target)
        self.assertAlmostEqual(
            replay.state()["wall_time"], info["wall_start"] + target, places=1
        )

    def test_tf_pose_accepted_when_costmap_window_is_fixed(self):
        """代价地图窗口固定不跟车时，"窗口中心与车位重合"这条判据要跳过。

        往草堆走那批日志就是这个情况：窗口固定 → 用重合判据会把正确的 tf 位姿否掉，
        回放退回局部坐标，轨迹（弓字）就被摆错位置。
        """
        path = os.path.join(self.tmpdir, "fixedcm.bag")
        build_map_frame_bag(path, fixed_costmap=True)
        info = bag_reader.read_bag(path)
        self.assertEqual(info["pose_frame"], "map")
        self.assertEqual(info["tf_pose"]["name"], "base_link1")
        # 窗口固定 → 不做窗口重合检查
        self.assertIsNone(info["tf_pose"]["window_gap"])

    def test_tf_pose_accepted_without_log_map(self):
        """日志里没有可解析的地图（whole_map 是空消息）时，仍要用 tf 的地图位姿。

        逆时针建图那批日志就是这样：以前必须先有地图才去试 tf 位姿，结果退回
        base_pose（局部坐标、朝向差几十度），代价地图看着就"位置不对"。
        """
        path = os.path.join(self.tmpdir, "nomap.bag")
        build_map_frame_bag(path, include_map=False)
        info = bag_reader.read_bag(path)
        self.assertIsNone((info.get("maps") or {}).get("whole_map"))
        self.assertEqual(info["pose_frame"], "map")
        self.assertEqual(info["tf_pose"]["name"], "base_link1")
        self.assertIsNone(info["tf_pose"]["free_ratio"])

    def test_replay_does_not_react_to_obstacles(self):
        """日志播放只重放：不扫视觉、不刷障碍层、不显示仿真视觉。"""
        path = os.path.join(self.tmpdir, "mapframe.bag")
        build_map_frame_bag(path)
        info = bag_reader.read_bag(path)
        world.select_map(1)
        robot = RobotCar()
        manager = TaskManager(robot, GridMap())
        manager.load_replay(info)
        manager.start_replay()
        manager.tick(DT)
        self.assertEqual(robot.vision, [])
        self.assertEqual(manager.get_state()["ray_scan"], [])
        self.assertEqual(robot.collisions, 0)

    def test_obstacle_cloud_is_mapped_into_map_frame(self):
        """深度相机点云按位姿 + 相机偏移换算到地图坐标，视锥角点跳过。"""
        path = os.path.join(self.tmpdir, "cloud.bag")
        build_map_frame_bag(path, clouds=True)
        info = bag_reader.read_bag(path)
        self.assertEqual(info["pose_frame"], "map")
        frames = info["clouds"]
        self.assertEqual(len(frames), 120)
        stamp, points, view = frames[0]
        self.assertAlmostEqual(stamp, 0.0, places=2)
        # 每帧 3 个真实点（4 个 z=-2 的视锥角点单独换算成"看过的地面"四边形）
        self.assertEqual(len(points), 6)
        self.assertEqual(len(view), 8)
        pose_x, pose_y = info["poses"][0][1], info["poses"][0][2]
        cam_x, cam_y = bag_reader.CLOUD_CAMERA_OFFSET
        self.assertAlmostEqual(points[0], round(pose_x + cam_x + 1.0, 2), places=2)
        self.assertAlmostEqual(points[1], round(pose_y + cam_y + 0.5, 2), places=2)
        world.select_map(1)
        manager = TaskManager(RobotCar(), GridMap())
        manager.load_replay(info)
        layer = manager.replay.cloud_at(0.0)
        self.assertEqual(layer["frames"], 1)
        # 3 个点落在 3 个格子里
        self.assertEqual(layer["count"], 3)

    def test_cloud_points_are_shifted_into_simulation_frame(self):
        """点云必须和轨迹用同一套换算，否则点云会飘到地图外面。"""
        path = os.path.join(self.tmpdir, "cloudframe.bag")
        build_map_frame_bag(path, clouds=True)
        info = bag_reader.read_bag(path)
        log_map = info["maps"]["whole_map"]
        definition = map_import.build_map_from_grid(
            log_map["resolution"], log_map["width"], log_map["height"],
            log_map["data"], grid_origin=log_map["origin"], name="log",
        )
        world.select_map(world.register_map(definition))
        manager = TaskManager(RobotCar(), GridMap())
        manager.load_replay(info, definition["replay_transform"])
        replay = manager.replay
        layer = replay.cloud_at(1.0)
        pose = replay.track[10]
        self.assertAlmostEqual(replay.scale, 1.0, places=3)
        # 累计点云的第一点来自第一帧
        expected = [
            round(info["clouds"][0][1][index] + replay.map_offset[index % 2], 2)
            for index in range(2)
        ]
        got = layer["points"][:2]
        self.assertLess(math.hypot(got[0] - expected[0], got[1] - expected[1]), 0.3)
        # 离车身不超过 3 m（点云是相机看到的前方障碍）
        self.assertLess(abs(layer["points"][0] - pose[1]), 3.0)
        self.assertLess(abs(layer["points"][1] - pose[2]), 3.0)

    def test_cloud_is_refreshed_when_area_looks_empty_again(self):
        """同一处第二次探测没东西了：旧点云要被刷掉，而不是一直累积。"""
        path = os.path.join(self.tmpdir, "refresh.bag")
        build_cloud_refresh_bag(path, hits=3, misses=bag_player.CLOUD_MISS_LIMIT + 2, dt=0.15)
        info = bag_reader.read_bag(path)
        world.select_map(1)
        manager = TaskManager(RobotCar(), GridMap())
        manager.load_replay(info)
        replay = manager.replay
        # 第 3 帧（最后一次测到障碍）时还有点云：3 帧 × 2 个原始点（不按格子合并）
        seen = replay.cloud_at(2 * 0.15)
        self.assertIsNotNone(seen)
        self.assertEqual(seen["count"], 6)
        # 中间几帧"看到了但没东西"：不到阈值仍保留（深度相机是稀疏采样，不能一帧就删）
        almost = replay.cloud_at((3 + bag_player.CLOUD_MISS_LIMIT - 2) * 0.15)
        self.assertIsNotNone(almost)
        self.assertEqual(almost["count"], 6)
        # 连续 CLOUD_MISS_LIMIT 帧都空 → 刷新掉
        gone = replay.cloud_at((3 + bag_player.CLOUD_MISS_LIMIT) * 0.15)
        self.assertIsNone(gone)

    def test_cloud_keeps_raw_observation_coordinates(self):
        """点云保留**原始观测坐标**：不按 0.2 m 格子合并、不吸到格心。

        以前一格只留一个点、坐标还取"亚格平滑值"，画出来是稀疏的描边；RViz 里那层是
        原始点的"喷雾"，同一处多次观测叠成一片。这里三个点都在同一格，必须原样保留三条。
        """
        path = os.path.join(self.tmpdir, "subcell.bag")
        hits = ((1.20, 0.00, 0.06), (1.24, 0.03, 0.06), (1.22, 0.05, 0.06))
        build_cloud_refresh_bag(path, hits=3, misses=0, dt=0.15, obstacle=hits)
        info = bag_reader.read_bag(path)
        world.select_map(1)
        manager = TaskManager(RobotCar(), GridMap())
        manager.load_replay(info)
        layer = manager.replay.cloud_at(info["duration"])
        self.assertEqual(layer["count"], 9)   # 3 帧 × 3 点
        offset = manager.replay.offset
        raw = info["clouds"][0][1]
        expected = sorted(
            (round(raw[index] + offset[0], 2), round(raw[index + 1] + offset[1], 2))
            for index in range(0, 6, 2)
        )
        # 位置原样保留（每帧都测到同样三个点，所以三条会重复出现三次）
        got = sorted(set(zip(layer["points"][0::2], layer["points"][1::2])))
        self.assertEqual(got, expected)

    def test_cloud_is_clipped_to_costmap_window(self):
        """点云只保留在局部代价地图范围内：窗口外的点直接丢掉（和代价地图那层对齐）。

        深度相机最远看 1.35 m，正常情况下都在窗口里；车开远以后留在原地的旧点会跑出
        10 m 窗口，那时就不再显示——RViz 里那层点云也是"局部"的。
        """
        # 一个点在窗口内（相机前方 1.2 m），一个点在窗口外（前方 3 m，超出 2 m 窗口）
        obstacle = ((1.20, 0.00, 0.06), (3.00, 0.00, 0.06))
        inside_path = os.path.join(self.tmpdir, "cloudclip.bag")
        build_cloud_refresh_bag(
            inside_path, hits=3, misses=0, dt=0.15, obstacle=obstacle, costmap_value=100
        )
        info = bag_reader.read_bag(inside_path)
        world.select_map(1)
        manager = TaskManager(RobotCar(), GridMap())
        manager.load_replay(info)
        layer = manager.replay.cloud_at(info["duration"])
        self.assertIsNotNone(layer)
        self.assertEqual(layer["count"], 3)      # 3 帧 × 只有窗口内那 1 个点
        cam_x, cam_y = bag_reader.CLOUD_CAMERA_OFFSET
        pose = manager.replay.track[0]
        expected_x = round(pose[1] + cam_x + obstacle[0][0], 2)
        self.assertTrue(all(value == expected_x for value in layer["points"][0::2]))

        # 没有代价地图窗口时不裁剪：两个点都在
        free_path = os.path.join(self.tmpdir, "cloudnoclip.bag")
        build_cloud_refresh_bag(free_path, hits=3, misses=0, dt=0.15, obstacle=obstacle)
        info = bag_reader.read_bag(free_path)
        manager = TaskManager(RobotCar(), GridMap())
        manager.load_replay(info)
        layer = manager.replay.cloud_at(info["duration"])
        self.assertEqual(layer["count"], 6)

    def test_cloud_reports_point_ages_for_fading(self):
        """每个点要带"距最后一次观测多久"，前端按它淡出；同时下发淡出窗口。"""
        path = os.path.join(self.tmpdir, "cloudage.bag")
        build_cloud_refresh_bag(path, hits=3, misses=0, dt=0.15)
        info = bag_reader.read_bag(path)
        world.select_map(1)
        manager = TaskManager(RobotCar(), GridMap())
        manager.load_replay(info)
        first = manager.replay.cloud_at(0.0)
        self.assertEqual(first["count"], len(first["ages"]))
        self.assertEqual(first["ages"][0], 0.0)
        self.assertAlmostEqual(first["fade"], bag_player.CLOUD_FADE_SECONDS, places=3)
        # 之后没有新的观测：年龄跟着回放时间长大（暂停时不会长大，因为按回放时间算）
        later = manager.replay.cloud_at(10.0)
        self.assertEqual(later["count"], 6)
        self.assertGreaterEqual(min(later["ages"]), 9.0)
        # 超过留存窗口（CLOUD_FADE_SECONDS）就从记忆地图里删掉，而不是只藏起来
        gone = manager.replay.cloud_at(bag_player.CLOUD_FADE_SECONDS + 0.5)
        self.assertIsNone(gone)
        # 一开始就跳到超过窗口的时刻，结果一致（增量老化和从头重建要等价）
        fresh = TaskManager(RobotCar(), GridMap())
        fresh.load_replay(info)
        self.assertIsNone(fresh.replay.cloud_at(bag_player.CLOUD_FADE_SECONDS + 0.5))
        # 往回拖会重新生成（重建分支不受老化影响）
        again = manager.replay.cloud_at(0.3)
        self.assertEqual(again["count"], 6)
        self.assertLess(max(again["ages"]), 0.35)

    def test_starting_a_task_releases_replay_battery_pin(self):
        """回放把电量钉在日志值上；开始普通任务后要解下来，否则任务永远不掉电。"""
        path = os.path.join(self.tmpdir, "batt2.bag")
        build_battery_bag(path)
        info = bag_reader.read_bag(path)
        world.select_map(1)
        robot = RobotCar()
        manager = TaskManager(robot, GridMap())
        manager.load_replay(info)
        self.assertEqual(robot.battery_lock, 90.0)
        manager.start_mapping()
        self.assertIsNone(robot.battery_lock)

    def test_cloud_incremental_state_matches_full_rebuild(self):
        """按帧增量维护的结果要和"从 0 重建"完全一致（拖进度条来回拖不会错位）。"""
        path = os.path.join(self.tmpdir, "cloud.bag")
        build_map_frame_bag(path, clouds=True)
        info = bag_reader.read_bag(path)
        world.select_map(1)
        stepped = TaskManager(RobotCar(), GridMap())
        stepped.load_replay(info)
        for step in range(0, 12):
            layer = stepped.replay.cloud_at(step * 0.9)
        fresh = TaskManager(RobotCar(), GridMap())
        fresh.load_replay(info)
        target = fresh.replay.cloud_at(11 * 0.9)
        self.assertIsNotNone(layer)
        self.assertIsNotNone(target)
        self.assertEqual(layer["total"], target["total"])
        self.assertEqual(sorted(layer["points"]), sorted(target["points"]))

    def test_cloud_accumulates_until_time(self):
        path = os.path.join(self.tmpdir, "cloud.bag")
        build_map_frame_bag(path, clouds=True)
        info = bag_reader.read_bag(path)
        world.select_map(1)
        manager = TaskManager(RobotCar(), GridMap())
        manager.load_replay(info)
        early = manager.replay.cloud_at(0.5)
        late = manager.replay.cloud_at(info["duration"])
        self.assertGreater(late["frames"], early["frames"])
        self.assertGreater(late["count"], early["count"])

    def test_world_time_keeps_milliseconds(self):
        path = os.path.join(self.tmpdir, "wall.bag")
        build_map_frame_bag(path, wall_offset_ns=345000000)
        info = bag_reader.read_bag(path)
        world.select_map(1)
        manager = TaskManager(RobotCar(), GridMap())
        manager.load_replay(info)
        replay = manager.replay
        replay.seek(0.55)
        value = replay.state()["wall_time"]
        expected = info["wall_start"] + 0.55
        self.assertAlmostEqual(value, expected, places=2)
        # 不能只有 0.1 s 精度
        self.assertGreater(abs(value - round(value, 1)), 0.001)

    def test_costmap_cells_row_zero_is_min_y(self):
        """栅格行 0 是最小 y：写图时要翻转，否则整块代价地图上下镜像。"""
        path = os.path.join(self.tmpdir, "mapframe.bag")
        build_map_frame_bag(path)
        info = bag_reader.read_bag(path)
        frames = info["maps"]["grid_costmap"]
        grid = frames[0][1]
        rows = bag_reader.encode_cells(
            grid["data"], grid["width"], grid["height"], step=1, kind="cost"
        )
        # 造数据时第 0 列是致命代价 → 每行行首都应是 '9'
        self.assertTrue(all(row[0] == "9" for row in rows))
        # 行 0 对应 origin 里的最小 y（也就是仿真坐标里最下面那一行）
        self.assertEqual(rows[0][1:], "0" * (len(rows[0]) - 1))

    def test_map_frame_replay_keeps_costmap_around_robot(self):
        path = os.path.join(self.tmpdir, "mapframe.bag")
        build_map_frame_bag(path)
        info = bag_reader.read_bag(path)
        log_map = info["maps"]["whole_map"]
        definition = map_import.build_map_from_grid(
            log_map["resolution"], log_map["width"], log_map["height"],
            log_map["data"], grid_origin=log_map["origin"], name="log",
        )
        world.select_map(world.register_map(definition))
        robot = RobotCar()
        manager = TaskManager(robot, GridMap())
        manager.load_replay(info, definition["replay_transform"])
        replay = manager.replay
        self.assertEqual(replay.alignment, "map")
        self.assertEqual(replay.pose_frame, "map")
        manager.start_replay()
        steps = 0
        worst = 0.0
        while replay.active and steps < 2000:
            manager.tick(DT)
            steps += 1
            layer = replay.costmap_at(replay.time, step=4)
            if not layer:
                continue
            width = layer["width"] * layer["resolution"]
            height = layer["height"] * layer["resolution"]
            gap = math.hypot(
                layer["origin"][0] + width / 2.0 - robot.pose[0],
                layer["origin"][1] + height / 2.0 - robot.pose[1],
            )
            worst = max(worst, gap)
        # 机器人在回放全程都留在这个窗口里（窗口跟着车走）
        self.assertLess(worst, 1.0)


if __name__ == "__main__":
    unittest.main()
