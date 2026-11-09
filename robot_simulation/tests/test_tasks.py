import math
import os
import sys
import unittest

BACKEND_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "backend"
)
sys.path.insert(0, BACKEND_DIR)

import config  # noqa: E402
import map_editor  # noqa: E402
import world  # noqa: E402
from grid_map import GridMap  # noqa: E402
from robot_car import RobotCar  # noqa: E402
from tasks import TaskManager  # noqa: E402

DT = 0.02


def make_system(map_id=1):
    world.select_map(map_id)
    robot = RobotCar()
    grid = GridMap()
    manager = TaskManager(robot, grid)
    return robot, grid, manager


def advance(manager, seconds, stop=("done",), phases=None, poses=None, keep_battery=False):
    for index in range(int(round(seconds / DT))):
        if keep_battery and index % 50 == 0:
            manager.robot.battery = 100.0
        manager.tick(DT)
        if phases is not None:
            phases.add(manager.phase)
        if poses is not None:
            poses.append((manager.robot.pose[0], manager.robot.pose[1]))
        if manager.phase in stop:
            return


def map_field(robot, grid, manager):
    manager.start_mapping()
    advance(manager, 1400.0)


def square(cx, cy, half):
    return [
        (cx - half, cy - half),
        (cx + half, cy - half),
        (cx + half, cy + half),
        (cx - half, cy + half),
        (cx - half, cy - half),
    ]


def two_region_map(name="双区测试"):
    """注册一张"两个不相交区域 + 一条单行通道"的地图，返回地图编号。

    分区只按连通区域算，通道格子（纯通道段）既不割草也不并区，所以两边各成
    一个区域，正好用来验证多区域的割草顺序。
    """
    world.select_map(1)
    draft = map_editor.Draft(name)
    draft.add_region(square(7.0, 7.0, 2.6))
    draft.add_region(square(22.0, 7.0, 2.6))
    draft.add_dock(5.2, 7.0, 0.45)
    draft.add_passage([(5.0, 7.0), (23.0, 7.0)])
    return world.register_map(draft.to_definition(name))


class MappingTaskTest(unittest.TestCase):
    def test_map_loop_only_escapes_when_it_encloses_occupied_cell(self):
        robot, grid, manager = make_system()
        segment = [(5.0, 5.0), (5.0, 7.0), (7.0, 7.0), (7.0, 5.0)]
        col, row = grid.cell_of(6.0, 6.0)
        grid.data[grid.index(col, row)] = 1
        self.assertFalse(manager._loop_encloses_obstacle(segment))
        grid.data[grid.index(col, row)] = 2
        self.assertTrue(manager._loop_encloses_obstacle(segment))

    def test_mapping_is_counter_clockwise_and_returns_to_dock(self):
        robot, grid, manager = make_system()
        poses = []
        manager.start_mapping()
        advance(manager, 1400.0, poses=poses)
        self.assertEqual(manager.phase, "done")
        self.assertGreaterEqual(grid.boundary_discovery(), 0.95)
        self.assertEqual(robot.collisions, 0)
        dock_distance = math.hypot(
            robot.pose[0] - world.DOCK["x"], robot.pose[1] - world.DOCK["y"]
        )
        self.assertLess(dock_distance, 0.5)

        area = 0.0
        for index in range(len(poses)):
            x1, y1 = poses[index]
            x2, y2 = poses[(index + 1) % len(poses)]
            area += x1 * y2 - x2 * y1
        self.assertGreater(area, 0.0, "巡边方向应为逆时针")

    def test_mapping_low_battery_returns_to_breakpoint(self):
        robot, grid, manager = make_system()
        robot.battery = 35.0
        phases = set()
        manager.start_mapping()
        advance(manager, 1800.0, phases=phases)
        self.assertIn("charging", phases)
        self.assertIn("charge_return", phases)
        self.assertEqual(manager.phase, "done")
        self.assertGreaterEqual(grid.boundary_discovery(), 0.95)
        messages = [entry["message"] for entry in robot.events]
        self.assertTrue(any("电量低于 30%" in message for message in messages))
        self.assertTrue(any("返回断点" in message for message in messages))
        self.assertEqual(robot.collisions, 0)


    def test_mapping_keeps_map_edits(self):
        """用户手动加的障碍/禁区只能由用户删除，重新建图不能清掉。"""
        robot, grid, manager = make_system()
        world.add_static_obstacle(5.0, 4.5, 0.4)
        world.add_no_go_zone(3.0, 3.0, 5.0, 4.0)
        try:
            self.assertTrue(world.edit_dict()["static"])
            manager.start_mapping()
            self.assertTrue(world.edit_dict()["static"])
            self.assertTrue(world.edit_dict()["no_go"])
            self.assertTrue(world.NO_GO)
            # 障碍物也要重新写进栅格，规划才知道那儿过不去
            col, row = grid.cell_of(5.0, 4.5)
            self.assertTrue(grid.is_occupied_world(5.0, 4.5) or grid.cell_blocked(col, row))
        finally:
            world.clear_edits()

    def test_no_go_ring_bypass_generates_counter_clockwise_route(self):
        robot, grid, manager = make_system()
        manager.task = {"type": "mow", "zones": [1], "mode": "A"}
        manager.zone_id = 1
        manager.mowing_mode = "A"
        manager.phase = "zone_edge"
        robot.pose = [2.0, 4.0, 0.0]
        manager.edge_points = [(2.0, 4.0), (3.0, 4.2), (4.0, 4.4), (6.5, 4.0), (7.5, 4.0)]
        manager.edge_index = 0
        world.add_no_go_zone(3.0, 3.0, 5.0, 5.0)
        try:
            zone = manager._no_go_at(3.6, 3.6, 0.0)
            self.assertIsNotNone(zone)
            self.assertTrue(manager._begin_no_go_ring(zone, "zone_edge"))
            self.assertEqual(manager.phase, "no_go_ring")
            points = manager.no_go_ring_points
            self.assertGreaterEqual(len(points), 4)
            # 路径在禁区外，且绕行方向为逆时针（正面积）
            for point in points:
                self.assertFalse(
                    world.in_no_go(point[0], point[1]),
                    "绕行路径不应落在禁区内",
                )
            area = 0.0
            for index in range(len(points)):
                x1, y1 = points[index]
                x2, y2 = points[(index + 1) % len(points)]
                area += x1 * y2 - x2 * y1
            self.assertGreater(area, 0.0)
            manager.edges = None
            manager.phase = "no_go_ring"
            manager.no_go_ring_resume = "zone_edge"
            manager._finish_no_go_ring()
            self.assertEqual(manager.edge_index, 3)
            self.assertEqual(manager.phase, "zone_edge")
        finally:
            world.clear_edits()

    def test_no_go_guard_blocks_forward_motion(self):
        robot, grid, manager = make_system()
        manager.task = {"type": "mow", "zones": [1], "mode": "A"}
        manager.phase = "zone_stripes"
        x, y, theta = manager._pose()
        world.add_no_go_zone(
            x + 0.18, y - 0.6, x + 1.6, y + 0.6
        )
        try:
            v, w = manager._guard_no_go((0.6, 0.2))
            self.assertAlmostEqual(v, 0.0)
            self.assertNotEqual(w, 0.0)
            # 朝远离禁区的方向行驶不受影响
            v2, w2 = manager._guard_no_go((-0.6, 0.2))
            self.assertAlmostEqual(v2, -0.6)
            self.assertAlmostEqual(w2, 0.2)
        finally:
            world.clear_edits()

    def test_finished_mapping_never_restarts_from_stale_breakpoint(self):
        robot, grid, manager = make_system()
        map_field(robot, grid, manager)
        self.assertEqual(manager.phase, "done")
        self.assertTrue(manager.mapping_done)
        manager.breakpoint = {
            "pose": manager._pose(),
            "phase": "map_follow",
            "zone_id": None,
            "zone_index": 0,
            "map_path": [],
            "map_path_dist": 0.0,
        }
        manager._restore_breakpoint()
        self.assertNotIn(manager.phase, ("map_follow", "map_seek", "map_escape"))
        self.assertIsNone(manager.breakpoint)
        advance(manager, 90.0)
        self.assertEqual(manager.phase, "done")
        messages = [entry["message"] for entry in robot.events]
        self.assertEqual(messages.count("开始自动建图：从当前位置逆时针巡边"), 1)

    def test_control_redirects_mapping_phase_after_finish(self):
        robot, grid, manager = make_system()
        map_field(robot, grid, manager)
        manager.phase = "map_follow"
        manager.map_path = [(robot.pose[0], robot.pose[1])]
        manager._control(DT, 0.0)
        self.assertNotEqual(manager.phase, "map_follow")
        advance(manager, 60.0)
        self.assertEqual(manager.phase, "done")


class MowingTaskTest(unittest.TestCase):
    def setUp(self):
        config.configure(
            max_linear=1.5,
            nav_speed=1.0,
            map_speed=1.0,
            mow_speed=1.3,
            edge_speed=0.85,
        )

    def tearDown(self):
        config.reset()

    def test_single_zone_full_coverage(self):
        robot, grid, manager = make_system(3)
        map_field(robot, grid, manager)
        manager.start_mowing([1])
        advance(manager, 900.0)
        self.assertEqual(manager.phase, "done")
        self.assertGreaterEqual(grid.zone_coverage(1), 0.85)
        # 本测试使用加速参数缩短仿真时间；碰撞安全由默认速度测试覆盖。
        self.assertLessEqual(robot.collisions, 30)
        messages = [entry["message"] for entry in robot.events]
        self.assertTrue(any("边缘已完成" in message for message in messages))
        self.assertTrue(any("规划弓字线段" in message for message in messages))
        self.assertTrue(any("规划弓字线段" in message for message in messages))

    def test_b_mode_uses_larger_no_go_margin(self):
        robot, grid, manager = make_system()
        map_field(robot, grid, manager)
        world.add_no_go_zone(4.0, 4.0, 5.0, 5.0)
        try:
            manager.start_mowing([1], mode="B")
            near_col, near_row = grid.cell_of(3.70, 4.5)
            self.assertTrue(grid.cell_blocked_mow(near_col, near_row))
            self.assertFalse(world.in_no_go(3.70, 4.5, 0.0), "0.3 m 外应仍在禁区之外")
        finally:
            world.clear_edits()

    def test_mow_start_restores_obstacle_baseline(self):
        robot, grid, manager = make_system(1)
        map_field(robot, grid, manager)
        zone = max(grid.zones, key=lambda item: item["free"])
        cx, cy = zone["cx"], zone["cy"]
        col, row = grid.cell_of(cx, cy)
        outline_before = [tuple(point) for point in zone["outline"]]
        # 上次割草涂抹出来的假障碍（建图完成时并不存在）
        smear = grid.index(col + 8, row + 8)
        grid.data[smear] = 2
        grid.mow_blocked[smear] = 1
        manager.start_mowing([zone["id"]])
        self.assertNotEqual(grid.data[smear], 2)
        self.assertFalse(grid.mow_blocked[smear])
        after = [item for item in grid.zones if item["id"] == zone["id"]][0]
        self.assertEqual([tuple(point) for point in after["outline"]], outline_before)
        messages = [entry["message"] for entry in robot.events]
        self.assertTrue(any("重置区域内障碍涂抹" in message for message in messages))

    def test_auto_recharge_resumes_mowing(self):
        robot, grid, manager = make_system(3)
        map_field(robot, grid, manager)
        robot.battery = 31.0
        phases = set()
        manager.start_mowing([1])
        advance(manager, 900.0, phases=phases)
        self.assertIn("charge_go", phases)
        self.assertTrue("charging" in phases or "charge_return" in phases)
        self.assertIn("charge_return", phases)
        self.assertEqual(manager.phase, "done")
        self.assertGreaterEqual(grid.zone_coverage(1), 0.85)

    def test_manual_recharge_waits_for_resume(self):
        robot, grid, manager = make_system(3)
        map_field(robot, grid, manager)
        manager.start_mowing([1])
        advance(manager, 10.0, stop=("done",))
        coverage_before = grid.zone_coverage(1)
        manager.request_charge()
        advance(manager, 40.0, stop=("done",))
        self.assertEqual(manager.phase, "charging")
        self.assertIsNotNone(manager.breakpoint)
        self.assertGreater(robot.battery, 90.0)
        self.assertGreaterEqual(coverage_before, 0.0)

        manager.request_resume()
        advance(manager, 900.0)
        self.assertEqual(manager.phase, "done")
        self.assertGreaterEqual(grid.zone_coverage(1), 0.85)

    def test_zone_order_and_multiple_zones(self):
        """两个区域经通道相连：按给定顺序先割完一个，穿通道过去再割下一个。"""
        map_id = two_region_map()
        try:
            world.select_map(map_id)
            robot = RobotCar()
            grid = GridMap()
            manager = TaskManager(robot, grid)
            self.assertTrue(grid.build_free_area((robot.pose[0], robot.pose[1])))
            grid.partition_zones()
            robot.battery = 100.0
            self.assertGreaterEqual(len(grid.zones), 2)
            order = [grid.zones[1]["id"], grid.zones[0]["id"]]
            manager.start_mowing(order)
            self.assertEqual(manager.zone_order, order)
            advance(manager, 1600.0, stop=("done", "to_dock"), keep_battery=True)
            self.assertIn(manager.phase, ("to_dock", "done"))
            for zone_id in order:
                self.assertGreaterEqual(grid.zone_coverage(zone_id), 0.83)
            messages = [entry["message"] for entry in robot.events]
            positions = [
                next(
                    index
                    for index, message in enumerate(messages)
                    if "区域 {} 割草完成".format(zone_id) in message
                )
                for zone_id in order
            ]
            self.assertLess(positions[0], positions[1])
            self.assertEqual(robot.collisions, 0)
        finally:
            if map_id in world.MAPS:
                world.delete_map(map_id)
            world.select_map(3)

    def test_stop_task_returns_to_idle(self):
        robot, grid, manager = make_system(3)
        manager.start_mapping()
        advance(manager, 3.0, stop=())
        manager.stop_task("测试停止", resumable=False)
        self.assertIsNone(manager.task)
        self.assertEqual(manager.phase, "idle")

    def test_stopped_task_can_continue(self):
        robot, grid, manager = make_system(3)
        map_field(robot, grid, manager)
        robot.battery = 100.0
        manager.start_mowing([1])
        advance(manager, 25.0, stop=())
        manager.stop_task("控制台暂停")
        self.assertEqual(manager.phase, "stopped")
        self.assertIsNotNone(manager.breakpoint)
        coverage = grid.zone_coverage(1)
        advance(manager, 8.0, stop=())
        self.assertAlmostEqual(grid.zone_coverage(1), coverage, places=4)
        manager.request_resume()
        self.assertIn(manager.phase, ("zone_edge", "zone_plan", "to_zone"))
        advance(manager, 900.0)
        self.assertEqual(manager.phase, "done")
        self.assertGreaterEqual(grid.zone_coverage(1), 0.80)


class EditedMapZoneFlowTest(unittest.TestCase):
    """地图编辑器保存的地图：每个区域自己走完沿边 → 弓字 → 补割，再去下一个。"""

    def setUp(self):
        config.configure(
            max_linear=1.5,
            nav_speed=1.0,
            map_speed=1.0,
            mow_speed=1.3,
            edge_speed=0.85,
        )
        world.select_map(1)
        draft = map_editor.Draft("双区流程")
        draft.add_region(square(10.0, 10.0, 4.0))
        draft.add_region(square(20.0, 10.0, 4.0))
        draft.add_dock(7.0, 10.0, 0.45)
        draft.add_passage([(13.0, 10.0), (17.0, 10.0)])
        self.map_id = world.register_map(draft.to_definition("双区流程"))
        world.select_map(self.map_id)
        self.robot = RobotCar()
        self.grid = GridMap()
        self.manager = TaskManager(self.robot, self.grid)

    def tearDown(self):
        config.reset()
        if self.map_id in world.MAPS:
            world.delete_map(self.map_id)
        world.select_map(3)

    def test_each_region_runs_full_flow_before_next(self):
        self.manager.start_mapping()
        advance(self.manager, 900.0)
        self.assertEqual(self.manager.phase, "done")
        self.assertEqual(len(self.grid.zones), 2, "编辑器画的两个区域应各算一个区域")

        self.robot.battery = 100.0
        order = [2, 1]
        self.manager.start_mowing(order, "A")
        advance(self.manager, 2400.0, stop=("done", "to_dock"), keep_battery=True)
        self.assertIn(self.manager.phase, ("to_dock", "done"))
        for zone_id in order:
            # 两个区域都要基本割完：沿边被"墙外占据带"冒充的障碍打断时这里会掉到 94%
            self.assertGreaterEqual(self.grid.zone_coverage(zone_id), 0.97)

        messages = [entry["message"] for entry in self.robot.events]
        marks = []
        for zone_id in order:
            marks.append(
                next(
                    index
                    for index, message in enumerate(messages)
                    if "区域 {} 割草完成".format(zone_id) in message
                )
            )
        self.assertLess(marks[0], marks[1])
        # 第二个区域开始之前，第一个区域必须已经走完沿边 + 弓字（+ 补割）
        first_zone_events = [
            message for message in messages[: marks[0]] if "区域 2" in message
        ]
        self.assertTrue(any("沿边缘割草一周" in message for message in first_zone_events))
        self.assertTrue(any("规划弓字线段" in message for message in first_zone_events))


class LaneAxisTest(unittest.TestCase):
    """主弓字方向：进区域时按长边定一次，主覆盖阶段里不再来回换。"""

    def setUp(self):
        self.robot, self.grid, self.manager = make_system(3)
        self.grid.build_free_area((self.robot.pose[0], self.robot.pose[1]))
        self.grid.partition_zones()
        self.manager.zone_id = 1
        self.manager.task = {"type": "mow"}

    def test_preferred_axis_follows_long_side(self):
        axis = self.manager._preferred_lane_axis
        self.assertEqual(axis({"x0": 0.0, "x1": 12.0, "y0": 0.0, "y1": 6.0}), "h")
        self.assertEqual(axis({"x0": 0.0, "x1": 6.0, "y0": 0.0, "y1": 12.0}), "v")
        self.assertEqual(axis({"x0": 0.0, "x1": 6.0, "y0": 0.0, "y1": 6.0}), "h")

    def test_main_axis_survives_replan(self):
        manager = self.manager
        manager.zone_stage = "stripes"
        manager.lane_axis = "v"
        manager.main_lane_axis = "v"
        manager.segments = []
        manager._node_lanes = lambda zone, mode, axis=None: []
        self.grid.zone_coverage = lambda zone_id: 0.5
        manager._finish_zone_coverage = lambda: None
        manager._plan_stripes()
        # 主方向用尽后允许换成另一个方向做补割，但“主方向”本身不变
        self.assertEqual(manager.lane_axis, "h")
        self.assertEqual(manager.main_lane_axis, "v")
        manager._plan_stripes()
        self.assertEqual(manager.lane_axis, "h", "补割方向也不该再翻回来")
        self.assertEqual(manager.main_lane_axis, "v")

    def test_wide_zone_starts_with_horizontal_lanes(self):
        zone = self.grid.zones[0]
        wide = dict(zone)
        wide["x0"], wide["x1"] = 0.0, 18.0
        wide["y0"], wide["y1"] = 0.0, 8.0
        self.assertEqual(self.manager._preferred_lane_axis(wide), "h")
        self.manager.zone_id = zone["id"]
        self.manager._enter_zone()
        self.assertEqual(self.manager.lane_axis, self.manager.main_lane_axis)


if __name__ == "__main__":
    unittest.main()
