import json
import math
import os
import struct
import sys
import tempfile
import unittest

BACKEND_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "backend"
)
sys.path.insert(0, BACKEND_DIR)

import config  # noqa: E402
import map_editor  # noqa: E402
import mow_report  # noqa: E402
import world  # noqa: E402
from grid_map import GridMap  # noqa: E402
from robot_car import RobotCar  # noqa: E402
from tasks import TaskManager  # noqa: E402

DT = 0.02


def make_system(map_id=5):
    world.select_map(map_id)
    robot = RobotCar()
    grid = GridMap()
    manager = TaskManager(robot, grid)
    return robot, grid, manager


def advance(manager, seconds, stop=("done",)):
    for _ in range(int(round(seconds / DT))):
        manager.robot.battery = 100.0
        manager.tick(DT)
        if manager.phase in stop:
            return


class PngTest(unittest.TestCase):
    def test_encodes_valid_png_header(self):
        rows = [bytes([255, 0, 0] * 3) for _ in range(4)]
        data = mow_report.encode_png(3, 4, rows)
        self.assertTrue(data.startswith(b"\x89PNG\r\n\x1a\n"))
        length = struct.unpack(">I", data[8:12])[0]
        self.assertEqual(data[12:16], b"IHDR")
        width, height = struct.unpack(">II", data[16:24])
        self.assertEqual((width, height), (3, 4))
        self.assertGreater(length, 0)


class ReportTest(unittest.TestCase):
    """割草结束后要在 data/data（这里指向临时目录）写一份报告文档。"""

    @classmethod
    def setUpClass(cls):
        cls.output = tempfile.mkdtemp(prefix="mow_report_case_")
        cls.previous = mow_report.OUTPUT_DIR
        mow_report.OUTPUT_DIR = cls.output
        config.configure(
            max_linear=1.5,
            nav_speed=1.0,
            map_speed=1.0,
            mow_speed=1.3,
            edge_speed=0.85,
        )
        cls.robot, cls.grid, cls.manager = make_system(5)
        cls.manager.start_mapping()
        advance(cls.manager, 900.0)
        cls.manager.start_mowing(None, "A")
        advance(cls.manager, 2400.0, stop=("done", "to_dock"))

    @classmethod
    def tearDownClass(cls):
        config.reset()
        mow_report.OUTPUT_DIR = cls.previous
        world.select_map(3)

    def test_report_files_are_written(self):
        report = self.manager.mow_report
        self.assertIsNotNone(report, "割草结束后没有生成报告")
        for key in ("html", "png", "json"):
            self.assertTrue(os.path.isfile(report[key]), key)
            self.assertGreater(os.path.getsize(report[key]), 500, key)
        # 每次任务一个以时间命名的文件夹，三件套放在里面
        folder = report["dir"]
        self.assertEqual(os.path.dirname(folder), os.path.abspath(self.output))
        self.assertRegex(os.path.basename(folder), r"^\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}$")
        self.assertEqual(
            sorted(os.listdir(folder)), ["coverage.png", "data.json", "report.html"]
        )
        self.assertEqual(os.path.basename(report["html"]), "report.html")
        self.assertEqual(os.path.basename(report["png"]), "coverage.png")
        self.assertEqual(os.path.basename(report["json"]), "data.json")

    def test_report_document_has_summary_and_cover_map(self):
        report = self.manager.mow_report
        with open(report["html"], encoding="utf-8") as handle:
            html = handle.read()
        self.assertIn("割草任务报告", html)
        self.assertIn("任务摘要", html)
        self.assertIn("覆盖图与规划路径", html)
        self.assertIn("data:image/png;base64,", html)
        self.assertIn("逐区域数据", html)

    def test_json_keeps_coverage_plan_and_track(self):
        report = self.manager.mow_report
        with open(report["json"], encoding="utf-8") as handle:
            payload = json.load(handle)
        self.assertEqual(payload["task"]["mode"], "A")
        self.assertEqual(payload["task"]["zone_order"], [1])
        coverage = payload["coverage"]
        self.assertGreater(coverage["zone_total"], 0)
        self.assertGreater(coverage["mowed_area"], 0.0)
        self.assertAlmostEqual(
            coverage["ratio"],
            coverage["zone_mowed"] / float(coverage["zone_total"]),
            places=3,
        )
        self.assertGreater(coverage["cells"]["mowed"], 0)
        raster = coverage["raster"]
        self.assertIsNotNone(raster, "覆盖栅格没有写进 JSON")
        self.assertEqual(len(raster), self.grid.rows)
        self.assertEqual(len(raster[0]), self.grid.cols)
        self.assertGreater(raster and sum(row.count("M") for row in raster), 0)
        self.assertTrue(payload["zones"])
        self.assertGreater(len(payload["plan"]["stripes"]), 5, "没有记录弓字规划线")
        self.assertGreater(len(payload["plan"]["edges"][0]["points"]), 20, "没有记录沿边规划线")
        self.assertGreater(len(payload["track"]), 50, "没有记录实际轨迹")
        self.assertEqual(payload["zone_records"][0]["id"], 1)
        self.assertGreater(payload["zone_records"][0]["coverage"], 0.9)

    def test_coverage_image_matches_grid_aspect(self):
        payload = self.manager.mow_report["payload"]
        self.assertEqual(payload["coverage_map"]["image"], "png")
        width = payload["coverage_map"]["width"]
        height = payload["coverage_map"]["height"]
        ratio = width / float(height)
        expected = self.grid.cols / float(self.grid.rows)
        self.assertLess(abs(ratio - expected), 0.05)
        with open(self.manager.mow_report["png"], "rb") as handle:
            head = handle.read(33)
        self.assertTrue(head.startswith(b"\x89PNG"))
        size = struct.unpack(">II", head[16:24])
        self.assertEqual(size, (width, height))


class ReportFolderTest(unittest.TestCase):
    """两次任务生成两个按时间命名的文件夹，互不覆盖。"""

    def test_two_tasks_create_two_folders(self):
        output = tempfile.mkdtemp(prefix="mow_report_folders_")
        previous = mow_report.OUTPUT_DIR
        mow_report.OUTPUT_DIR = output
        config.configure(max_linear=1.5, nav_speed=1.0, map_speed=1.0, mow_speed=1.3, edge_speed=0.85)
        try:
            world.select_map(1)
            robot = RobotCar()
            grid = GridMap()
            manager = TaskManager(robot, grid)
            manager.start_mapping()
            advance(manager, 900.0)
            folders = []
            for _ in range(2):
                robot.battery = 100.0
                manager.start_mowing(None, "A")
                advance(manager, 2200.0, stop=("done", "to_dock"))
                self.assertIsNotNone(manager.mow_report)
                folders.append(manager.mow_report["dir"])
            self.assertEqual(len(set(folders)), 2, "两次任务应该落在两个文件夹")
            for folder in folders:
                self.assertTrue(os.path.isdir(folder))
                self.assertEqual(
                    sorted(os.listdir(folder)), ["coverage.png", "data.json", "report.html"]
                )
        finally:
            mow_report.OUTPUT_DIR = previous
            config.reset()
            world.select_map(3)


class ZoneReportDataTest(unittest.TestCase):
    """多区域任务：每个区域都有一份数据快照。"""

    def setUp(self):
        self.output = tempfile.mkdtemp(prefix="mow_report_multi_")
        self.previous = mow_report.OUTPUT_DIR
        mow_report.OUTPUT_DIR = self.output
        config.configure(max_linear=1.5, nav_speed=1.0, map_speed=1.0, mow_speed=1.3, edge_speed=0.85)
        world.select_map(1)
        draft = map_editor.Draft("报告双区")
        square = lambda cx, cy, half: [
            (cx - half, cy - half),
            (cx + half, cy - half),
            (cx + half, cy + half),
            (cx - half, cy + half),
            (cx - half, cy - half),
        ]
        draft.add_region(square(10.0, 10.0, 4.0))
        draft.add_region(square(20.0, 10.0, 4.0))
        draft.add_dock(7.0, 10.0, 0.45)
        draft.add_passage([(13.0, 10.0), (17.0, 10.0)])
        self.map_id = world.register_map(draft.to_definition("报告双区"))
        world.select_map(self.map_id)
        self.robot = RobotCar()
        self.grid = GridMap()
        self.manager = TaskManager(self.robot, self.grid)

    def tearDown(self):
        config.reset()
        mow_report.OUTPUT_DIR = self.previous
        if self.map_id in world.MAPS:
            world.delete_map(self.map_id)
        world.select_map(3)

    def test_each_zone_is_recorded(self):
        self.manager.start_mapping()
        advance(self.manager, 900.0)
        self.manager.start_mowing([2, 1], "A")
        advance(self.manager, 2400.0, stop=("done", "to_dock"))
        report = self.manager.mow_report
        self.assertIsNotNone(report)
        records = report["payload"]["zone_records"]
        self.assertEqual([item["id"] for item in records], [2, 1])
        for item in records:
            self.assertGreater(item["area"], 10.0)
            self.assertGreater(item["coverage"], 0.9)
            self.assertGreaterEqual(item["seconds"], 0.0)
        self.assertEqual(report["payload"]["task"]["zone_order"], [2, 1])
        self.assertEqual(len(report["payload"]["plan"]["edges"]), 2)


if __name__ == "__main__":
    unittest.main()
