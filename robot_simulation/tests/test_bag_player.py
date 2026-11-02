"""日志读取与回放测试：用临时 sqlite bag 覆盖解析、映射与吻合度统计。"""

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


def _pose_message(timestamp, x, y, yaw):
    half = yaw / 2.0
    position = _field_double(1, x) + _field_double(2, y) + _field_double(3, 0.0)
    orientation = (
        _field_double(1, 0.0)
        + _field_double(2, 0.0)
        + _field_double(3, math.sin(half))
        + _field_double(4, math.cos(half))
    )
    body = _field_bytes(1, position) + _field_bytes(2, orientation)
    header = _field_bytes(1, _varint(1 << 3) + _varint(1)) + _field_bytes(
        2, _varint(1 << 3) + _varint(int(timestamp * 1000))
    )
    return _field_bytes(1, header) + _field_bytes(3, body)


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


def build_bag(path, samples=120, dt=0.1, with_images=False):
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
        con.execute(
            "insert into Zmqmsg values (?, ?, ?)",
            (b"base_pose", start + t, _pose_message(start + t, x, y, yaw)),
        )
    if with_images:
        _add_images(con, start)
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

    def test_costmap_frame_centered_on_track(self):
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
        for ratio in (0.0, 0.5, 1.0):
            layer = replay.costmap_at(replay.duration * ratio)
            ref = replay._track_point_at(replay.duration * ratio)
            center_x = layer["origin"][0] + layer["width"] * layer["resolution"] / 2.0
            center_y = layer["origin"][1] + layer["height"] * layer["resolution"] / 2.0
            self.assertAlmostEqual(center_x, ref[0], places=2)
            self.assertAlmostEqual(center_y, ref[1], places=2)

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
        manager.stop_replay()
        self.assertIsNone(robot.battery_lock)

    def test_replay_follows_battery_topic(self):
        path = os.path.join(self.tmpdir, "batt.bag")
        build_battery_bag(path)
        info = bag_reader.read_bag(path)
        world.select_map(1)
        robot = RobotCar()
        manager = TaskManager(robot, GridMap())
        manager.load_replay(info)
        self.assertIsNone(robot.battery_lock)
        manager.start_replay()
        readings = []
        for _ in range(240):
            manager.tick(DT * 5)
            if not manager.replay.active:
                break
            readings.append(robot.battery)
        self.assertLess(max(readings), 91.0)
        self.assertGreater(min(readings), 69.0)
        manager.stop_replay()

    def test_reject_missing_bag(self):
        with self.assertRaises(bag_reader.BagError):
            bag_reader.read_bag(os.path.join(self.tmpdir, "not-exist.bag"))


if __name__ == "__main__":
    unittest.main()
