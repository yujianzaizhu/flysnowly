"""栅格地图：视觉建图、自由区填充、自动分区、覆盖统计与 A* 规划。"""

import heapq
import math
import time

import world

UNKNOWN = 0
FREE = 1
OCCUPIED = 2

# 导航膨胀：mark_occupied 会写 (1 + INFLATE_CELLS) 圈，1 圈 = 0.25 m，
# 够一个半径 0.18 m 的车体通过；再大就会把 1 m 宽的走廊整条封死。
INFLATE_CELLS = 1
MOW_INFLATE_CELLS = 1
# 已确认障碍物的盖章余量：刀盘/车体都要真的绕开，留 2 格（0.2 m），
# 比整片涂抹小得多，又能避免弓字线贴着障碍物蹭过去发生碰撞。
STAMP_INFLATE_CELLS = 2
# 墙面占据格往墙外侧偏移多远：占据带画在场地外轮廓线之外，墙内侧不被吃掉
SURFACE_STAMP_OUTSET = 0.08
# 车体实际压过的格子半径：这是"这里一定能走"的硬证据。
# 只认车体中心那一格（半径远小于车体半径），目的是防止视觉盖章把机器人自己
# 走过的通道糊成障碍；不敢顺带放开更多导航膨胀，否则规划会贴着墙走。
VERIFY_RADIUS = 0.06
LARGE_OBSTACLE_EXTENT = 0.85
LARGE_OBSTACLE_AREA = 5.0
MAX_OBSTACLE_CELLS = 1200
MAX_OBSTACLE_EXTENT = 4.0
SMALL_OBSTACLE_RADIUS = 0.45
MIN_ZONE_AREA = 20.0
REGULAR_SOLIDITY = 0.90


class GridMap(object):
    def __init__(self):
        self.cell = world.CELL
        self.cols = world.COLS
        self.rows = world.ROWS
        self.version = 0
        self.data_version = 1
        self._circle_cache_version = -1
        self._circle_cache = []
        self._boundary_cache_version = -1
        self._boundary_cache = 0.0
        self._wall_known = []
        self._wall_known_version = -1
        # 还没探明的墙格索引集合。增量维护：传感器一旦把某个格子写成已知，
        # 就把该格（以及写了占据格时的 8 邻域）从集合里摘掉；为 None 表示需要
        # 全量重建（复位/换图之后）。以前每 tick 全量扫一遍 wall_cells，
        # 332 顶点的实景地图上一趟就是 8 ms，高倍速下整个仿真被它拖住。
        self._wall_pending = None
        # 从车当前位置出发的连通可达区：绕障绕完一圈后，障碍物内部那些"没见过的
        # 未知格"会被已探明占据环封住，不属于可达连通域，不能再当成可割点生成弓字。
        self.reach_mask = bytearray(self.cols * self.rows)
        self._reach_seed = None
        self._reach_version = -1
        self.wall_cells = world.boundary_cells()
        self.zones = []
        self.zone_totals = {}
        self.zone_mowed = {}
        self.zone_id_map = bytearray(self.cols * self.rows)
        self.zone_id_map = bytearray(self.cols * self.rows)
        self.free_mask = bytearray(self.cols * self.rows)
        self.mask_ready = False
        self._reset_arrays()

    def _reset_arrays(self):
        self.data_version = getattr(self, "data_version", 0) + 1
        self._obstacle_baseline = None
        self._wall_pending = None
        self._reach_version = -1
        self.data = bytearray(self.cols * self.rows)
        self.mowed = bytearray(self.cols * self.rows)
        self.blocked = bytearray(self.cols * self.rows)
        self.mow_blocked = bytearray(self.cols * self.rows)
        self.dyn_blocked = bytearray(self.cols * self.rows)
        self.verified_free = bytearray(self.cols * self.rows)
        self.free_mask = bytearray(self.cols * self.rows)
        self.mask_ready = False
        self.zones = []
        self.zone_totals = {}
        self.zone_mowed = {}

    def reset_map(self):
        self._reset_arrays()
        # 用户手动添加的障碍物不属于传感器建图，重建栅格后要原样写回去
        self.apply_static_edits(world.static_edits())
        self.version += 1

    def apply_static_edits(self, items):
        """把用户添加的静态障碍写进占据/膨胀层（圆形：中心 + 半径）。"""
        for item in items or []:
            self.stamp_obstacle(float(item["x"]), float(item["y"]), float(item["r"]))

    def reset_mowed(self):
        self.mowed = bytearray(self.cols * self.rows)
        for zone_id in self.zone_mowed:
            self.zone_mowed[zone_id] = 0
        self.version += 1

    def capture_obstacle_baseline(self):
        """记录建图完成时的障碍层，供每次新割草任务回到干净起点。"""
        self._obstacle_baseline = (
            bytearray(self.data),
            bytearray(self.blocked),
            bytearray(self.mow_blocked),
        )
        return True

    def restore_obstacle_baseline(self):
        """把障碍层恢复到建图完成时的状态：清掉上次割草涂抹的障碍，保留边界建图。"""
        baseline = getattr(self, "_obstacle_baseline", None)
        if baseline is None:
            return False
        self.data = bytearray(baseline[0])
        self.blocked = bytearray(baseline[1])
        self.mow_blocked = bytearray(baseline[2])
        self._wall_pending = None
        self.data_version += 1
        self.version += 1
        return True

    def apply_no_go(self, zones, margin=0.08):
        """把禁区写入导航/割草膨胀层：机器人不进入、刀盘不切割，但不算实体障碍。

        禁区为矩形（x0/y0/x1/y1）；仍兼容旧的圆形（x/y/r）描述。
        margin 用于额外膨胀（B 模式弓字线会用更大的值，避免贴边卡住）。
        """
        margin = max(0.0, float(margin))
        marked = 0
        for zone in zones or []:
            if "x0" in zone:
                x0 = min(float(zone["x0"]), float(zone["x1"]))
                x1 = max(float(zone["x0"]), float(zone["x1"]))
                y0 = min(float(zone["y0"]), float(zone["y1"]))
                y1 = max(float(zone["y0"]), float(zone["y1"]))
                col0, row0 = self.cell_of(x0 - margin - 0.1, y0 - margin - 0.1)
                col1, row1 = self.cell_of(x1 + margin + 0.1, y1 + margin + 0.1)
                for row in range(row0, row1 + 1):
                    for col in range(col0, col1 + 1):
                        x, y = self.center_of(col, row)
                        if not (
                            x0 - margin <= x <= x1 + margin
                            and y0 - margin <= y <= y1 + margin
                        ):
                            continue
                        index = self.index(col, row)
                        if not self.blocked[index]:
                            self.blocked[index] = 1
                            marked += 1
                        self.mow_blocked[index] = 1
                continue
            radius = float(zone.get("r", 1.0)) + margin
            cx = float(zone["x"])
            cy = float(zone["y"])
            col0, row0 = self.cell_of(cx - radius - 0.1, cy - radius - 0.1)
            col1, row1 = self.cell_of(cx + radius + 0.1, cy + radius + 0.1)
            for row in range(row0, row1 + 1):
                for col in range(col0, col1 + 1):
                    x, y = self.center_of(col, row)
                    if math.hypot(x - cx, y - cy) > radius:
                        continue
                    index = self.index(col, row)
                    if not self.blocked[index]:
                        self.blocked[index] = 1
                        marked += 1
                    self.mow_blocked[index] = 1
        if marked:
            self.version += 1
        return marked

    # ----------------------------------------------------------- 坐标
    def index(self, col, row):
        return row * self.cols + col

    def in_bounds(self, col, row):
        return 0 <= col < self.cols and 0 <= row < self.rows

    def cell_of(self, x, y):
        col = int(world.clamp(x / self.cell, 0, self.cols - 1))
        row = int(world.clamp(y / self.cell, 0, self.rows - 1))
        return col, row

    def center_of(self, col, row):
        return (col + 0.5) * self.cell, (row + 0.5) * self.cell

    def world_to_center(self, x, y):
        return self.center_of(*self.cell_of(x, y))

    # ----------------------------------------------------------- 建图
    def mark_free(self, col, row):
        if not self.in_bounds(col, row):
            return
        index = self.index(col, row)
        if self.data[index] == UNKNOWN:
            self.data[index] = FREE
            self.data_version += 1

    def _fill_rows(self, buffer, center_col, center_row, half, value=1):
        """按行切片把 (center ± half) 的方块写进 buffer，比逐格循环快很多。

        返回是否真的改动了内容（用于 data_version 失效判断）。
        """
        cols = self.cols
        c0 = center_col - half
        c1 = center_col + half + 1
        if c0 < 0:
            c0 = 0
        if c1 > cols:
            c1 = cols
        if c1 <= c0:
            return False
        r0 = center_row - half
        r1 = center_row + half + 1
        if r0 < 0:
            r0 = 0
        if r1 > self.rows:
            r1 = self.rows
        span = c1 - c0
        fill = bytes((value,)) * span
        changed = False
        for row in range(r0, r1):
            base = row * cols + c0
            if buffer[base:base + span] != fill:
                buffer[base:base + span] = fill
                changed = True
        return changed

    def mark_occupied(self, col, row, data_col=None, data_row=None):
        """写入墙面命中点。

        ``col/row`` 是墙面实际位置，导航/刀盘膨胀按它算；``data_col/data_row`` 是
        占据格要画的位置（默认同一个格子）。实景建图时占据格会画到墙的外侧，
        这样界面上的墙体带落在外轮廓线之外，不会把可割区域啃掉一圈。
        """
        if not self.in_bounds(col, row):
            return
        dcol = col if data_col is None else data_col
        drow = row if data_row is None else data_row
        if not self.in_bounds(dcol, drow):
            dcol, drow = col, row
        # 3×3 占据块 + 对应膨胀：按行切片写入（等价于原来逐格 9×(5×5) 的写法）
        changed = self._fill_rows(self.data, dcol, drow, 1, OCCUPIED)
        if self._wall_pending:
            self._note_known(drow * self.cols + dcol, spread=2)
        self._fill_rows(self.blocked, col, row, 1 + INFLATE_CELLS)
        self._fill_rows(self.mow_blocked, col, row, 1 + MOW_INFLATE_CELLS)
        if self._restore_verified(dcol, drow, 1):
            changed = True
        if self._restore_verified(col, row, 1 + INFLATE_CELLS):
            changed = True
        self.version += 1
        if changed:
            self.data_version += 1

    def _verify_footprint(self, x, y):
        """把车体当前压住的格子标成"确实能走"。

        视觉盖章（凸包填充、3×3 抹笔画）会把 1 m 级的窄通道糊成障碍，机器人
        自己把门口封死之后就再也规划不出去。车体此刻正压在这些格子上，是最硬的
        证据：这些格子必须是自由格、不能有导航膨胀，之后也不许再被写成障碍。
        """
        cell = self.cell
        radius = max(VERIFY_RADIUS, cell * 0.5)
        col0, row0 = self.cell_of(x - radius, y - radius)
        col1, row1 = self.cell_of(x + radius, y + radius)
        changed = False
        for row in range(row0, row1 + 1):
            for col in range(col0, col1 + 1):
                cx, cy = self.center_of(col, row)
                if math.hypot(cx - x, cy - y) > radius:
                    continue
                index = self.index(col, row)
                if not self.verified_free[index]:
                    self.verified_free[index] = 1
                if self.data[index] == OCCUPIED:
                    self.data[index] = FREE
                    changed = True
                if self.data[index] != UNKNOWN:
                    self._note_known(index)
                if self.blocked[index]:
                    self.blocked[index] = 0
        if changed:
            self.data_version += 1
            self.version += 1

    def _restore_verified(self, col, row, half):
        """盖章之后把车体压过的格子还原成自由格（它们不可能真的是障碍）。"""
        changed = False
        col0 = max(0, col - half)
        col1 = min(self.cols - 1, col + half)
        row0 = max(0, row - half)
        row1 = min(self.rows - 1, row + half)
        for rr in range(row0, row1 + 1):
            base = rr * self.cols
            for cc in range(col0, col1 + 1):
                index = base + cc
                if not self.verified_free[index]:
                    continue
                if self.data[index] == OCCUPIED:
                    self.data[index] = FREE
                    changed = True
                if self.data[index] != UNKNOWN:
                    self._note_known(index)
                if self.blocked[index]:
                    self.blocked[index] = 0
        return changed

    def _block_around(self, col, row):
        self._fill_rows(self.blocked, col, row, INFLATE_CELLS)

    def _block_mow_around(self, col, row):
        self._fill_rows(self.mow_blocked, col, row, MOW_INFLATE_CELLS)

    def stamp_obstacle(self, cx, cy, radius):
        """写入一个**已知**的圆形障碍（测试/显式给定几何时使用）。

        视觉聚类得到的障碍请用 ``stamp_footprint``：按真实轮廓外扩，不整片抹圆。
        """
        r_mow = radius + 0.10
        r_block = radius + 0.20
        col0, row0 = self.cell_of(cx - r_block - 0.1, cy - r_block - 0.1)
        col1, row1 = self.cell_of(cx + r_block + 0.1, cy + r_block + 0.1)
        changed = False
        for row in range(row0, row1 + 1):
            for col in range(col0, col1 + 1):
                if not self.in_bounds(col, row):
                    continue
                x, y = self.center_of(col, row)
                distance = math.hypot(x - cx, y - cy)
                index = self.index(col, row)
                if distance <= radius:
                    if self.data[index] != OCCUPIED:
                        self.data[index] = OCCUPIED
                        changed = True
                        self._note_known(index, spread=1)
                if distance <= r_mow:
                    self.mow_blocked[index] = 1
                if distance <= r_block:
                    self.blocked[index] = 1
        if changed:
            self.version += 1
            self.data_version += 1

    def stamp_footprint(self, cx, cy, radius):
        """按障碍物**实际占据的栅格**写入导航/刀盘层，不再用包围圆整片涂抹。

        视觉聚类给出的圆只用于绕障几何（绕行起点、花瓣探明），不能当成障碍物体积
        整片刷进栅格 —— 那样障碍物周围会多出一大圈假障碍，割草永远补不上。

        这里用已探明占据栅格的**凸包**代表障碍物轮廓：只把观测到的轮廓围出来的
        区域补成实心（视觉看不到的背面与内部空洞），再沿真实轮廓外扩必要的车体、
        刀盘余量。凸包始终不大于包围圆，所以障碍物附近的窄带可以正常割到。

        返回写入的栅格数，便于统计与调试。
        """
        pad = 0.30
        col0, row0 = self.cell_of(cx - radius - pad, cy - radius - pad)
        col1, row1 = self.cell_of(cx + radius + pad, cy + radius + pad)
        col0, row0 = max(0, col0), max(0, row0)
        col1, row1 = min(self.cols - 1, col1), min(self.rows - 1, row1)
        if col1 - col0 < 2 or row1 - row0 < 2:
            return 0

        # 只取离聚类中心最近的那个连通块，避免把旁边另一处障碍或视觉噪点
        # 一起包进同一个凸包（那会把两块之间的空地也涂成障碍）。
        seed = None
        best = None
        for row in range(row0, row1 + 1):
            for col in range(col0, col1 + 1):
                if self.data[self.index(col, row)] != OCCUPIED:
                    continue
                x, y = self.center_of(col, row)
                distance = math.hypot(x - cx, y - cy)
                if best is None or distance < best:
                    best = distance
                    seed = (col, row)
        if seed is None:
            return 0
        points = []
        seen = {seed}
        stack = [seed]
        while stack:
            col, row = stack.pop()
            points.append((col, row))
            for dc in (-1, 0, 1):
                for dr in (-1, 0, 1):
                    nc, nr = col + dc, row + dr
                    if not (col0 <= nc <= col1 and row0 <= nr <= row1):
                        continue
                    if (nc, nr) in seen:
                        continue
                    if self.data[self.index(nc, nr)] != OCCUPIED:
                        continue
                    seen.add((nc, nr))
                    stack.append((nc, nr))
        if len(points) < 3:
            return 0
        hull = _convex_hull(sorted(points))
        if len(hull) < 3:
            return 0

        # 逐行求凸包与扫描线的交点，填充两极之间的栅格
        spans = {}
        count = len(hull)
        for index in range(count):
            ac, ar = hull[index]
            bc, br = hull[(index + 1) % count]
            if ar == br:
                continue
            low, high = (ar, br) if ar < br else (br, ar)
            for row in range(max(row0, low), min(row1, high) + 1):
                t = (row - ar) / float(br - ar)
                col = ac + (bc - ac) * t
                span = spans.get(row)
                if span is None:
                    spans[row] = [col, col]
                else:
                    span[0] = min(span[0], col)
                    span[1] = max(span[1], col)

        changed = False
        total = 0
        for row, span in spans.items():
            lo = max(col0, int(math.floor(span[0])))
            hi = min(col1, int(math.ceil(span[1])))
            for col in range(lo, hi + 1):
                index = self.index(col, row)
                if self.verified_free[index]:
                    # 车体压过的格子不可能是障碍：凸包填充在窄通道里会糊死通道
                    continue
                if self.data[index] != OCCUPIED:
                    self.data[index] = OCCUPIED
                    changed = True
                    self._note_known(index, spread=1)
                self._fill_rows(self.blocked, col, row, INFLATE_CELLS)
                self._fill_rows(self.mow_blocked, col, row, STAMP_INFLATE_CELLS)
                total += 1
        if changed:
            self.version += 1
            self.data_version += 1
        return total

    def update_from_scan(self, pose, readings, max_range):
        x, y, theta = pose
        self._verify_footprint(x, y)
        cell = self.cell
        cols = self.cols
        rows = self.rows
        data = self.data
        changed_data = False
        pending = self._wall_pending

        # 自由区采样：内联 + 步进累加，避免每条射线每格都调用一次函数（建图期占一半开销）
        need_free = not self.mask_ready
        if need_free:
            col = int(x / cell)
            row = int(y / cell)
            if 0 <= col < cols and 0 <= row < rows:
                index = row * cols + col
                if data[index] == UNKNOWN:
                    data[index] = FREE
                    changed_data = True
                    self._note_known(index)
        step = cell
        limit = max_range - 1e-6
        for relative, distance in readings:
            heading = theta + relative
            dx = math.cos(heading)
            dy = math.sin(heading)
            if need_free:
                samples = int(distance / step)
                if samples > 0:
                    px = x
                    py = y
                    sx = dx * step
                    sy = dy * step
                    for _ in range(samples):
                        px += sx
                        py += sy
                        c = int(px / cell)
                        r = int(py / cell)
                        if 0 <= c < cols and 0 <= r < rows:
                            index = r * cols + c
                            if data[index] == UNKNOWN:
                                data[index] = FREE
                                changed_data = True
                                if pending is not None:
                                    pending.discard(index)
            if distance < limit:
                # 命中点在墙面上：膨胀按这里算，占据格画到墙外侧半格多一点，
                # 让墙线画在场界线的外侧（墙内侧一格都不吃）。
                px = x + dx * distance
                py = y + dy * distance
                if world.in_dynamic_obstacle(px, py, 0.08):
                    # 动态障碍物是移动物体，只进临时层，不能写进建图数据
                    continue
                out = distance + SURFACE_STAMP_OUTSET
                before = self.data_version
                self.mark_occupied(
                    int(px / cell),
                    int(py / cell),
                    int((x + dx * out) / cell),
                    int((y + dy * out) / cell),
                )
                changed_data = changed_data or self.data_version != before
        if changed_data:
            self.data_version += 1
        self.version += 1

    def unknown_wall_cells(self):
        """还没探明的墙格。

        判定"探明"不只看自己那一格有没有被写过：占据格现在画在墙的外侧
        （`SURFACE_STAMP_OUTSET`），墙内侧那一格常常还是未知，但隔壁就是探到的
        墙面，说明这段墙已经看到了。所以只要 3×3 邻域里有占据格也算探明，
        否则探明率会永远差最后一格。
        """
        if not self.wall_cells:
            return []
        pending = self._wall_pending_set()
        if self._wall_known_version == self.data_version:
            return list(self._wall_known)
        cols = self.cols
        unknown = [(index % cols, index // cols) for index in pending]
        unknown.sort()
        self._wall_known = unknown
        self._wall_known_version = self.data_version
        return list(unknown)

    def _wall_pending_set(self):
        """尚未探明的墙格索引集合（全量重建只在复位/换图之后发生一次）。"""
        pending = self._wall_pending
        if pending is not None:
            return pending
        cols = self.cols
        rows = self.rows
        data = self.data
        pending = set()
        for col, row in self.wall_cells:
            index = row * cols + col
            if data[index] != UNKNOWN:
                continue
            seen = False
            for dc in (-1, 0, 1):
                c2 = col + dc
                if c2 < 0 or c2 >= cols:
                    continue
                for dr in (-1, 0, 1):
                    r2 = row + dr
                    if r2 < 0 or r2 >= rows:
                        continue
                    if data[r2 * cols + c2] == OCCUPIED:
                        seen = True
                        break
                if seen:
                    break
            if not seen:
                pending.add(index)
        self._wall_pending = pending
        return pending

    def _note_known(self, index, spread=0):
        """格子写成已知后把它（及 ``spread`` 半径的邻域）从待探明墙格里摘掉。

        ``data`` 只会从 UNKNOWN 变成 FREE/OCCUPIED，不会退回 UNKNOWN，
        所以这个集合只增不减，增量维护与全量扫描结果一致。
        """
        pending = self._wall_pending
        if not pending:
            return
        pending.discard(index)
        if spread <= 0:
            return
        cols = self.cols
        rows = self.rows
        col = index % cols
        row = index // cols
        for dc in range(-spread, spread + 1):
            c2 = col + dc
            if c2 < 0 or c2 >= cols:
                continue
            for dr in range(-spread, spread + 1):
                r2 = row + dr
                if r2 < 0 or r2 >= rows:
                    continue
                pending.discard(r2 * cols + c2)

    def boundary_discovery(self):
        if not self.wall_cells:
            return 0.0
        if self._boundary_cache_version == self.data_version:
            return self._boundary_cache
        pending = self._wall_pending_set()
        self._boundary_cache_version = self.data_version
        self._boundary_cache = 1.0 - len(pending) / float(len(self.wall_cells))
        return self._boundary_cache

    def boundary_gaps(self, max_gaps=6):
        """未探明的墙格聚成"缺口"，按大小排序，供建图补扫定位。

        只看 `wall_cells`（紧贴真实边界的栅格）里还没被传感器写入过的格子，
        8 邻域聚簇后给出簇心、格数与沿墙跨度，跨度大的就是明显没扫到的一段墙。
        """
        if not self.wall_cells:
            return []
        unknown = set(self.unknown_wall_cells())
        if not unknown:
            return []
        gaps = []
        remaining = set(unknown)
        while remaining:
            seed = remaining.pop()
            stack = [seed]
            cluster = [seed]
            while stack:
                col, row = stack.pop()
                for dc in (-1, 0, 1):
                    for dr in (-1, 0, 1):
                        if dc == 0 and dr == 0:
                            continue
                        neighbour = (col + dc, row + dr)
                        if neighbour in remaining:
                            remaining.remove(neighbour)
                            stack.append(neighbour)
                            cluster.append(neighbour)
            cols_in = [col for col, _row in cluster]
            rows_in = [row for _col, row in cluster]
            span = max(max(cols_in) - min(cols_in), max(rows_in) - min(rows_in)) + 1
            gaps.append(
                {
                    "cells": len(cluster),
                    "x": round(
                        (sum(cols_in) / float(len(cluster)) + 0.5) * self.cell, 2
                    ),
                    "y": round(
                        (sum(rows_in) / float(len(cluster)) + 0.5) * self.cell, 2
                    ),
                    "run_m": round(span * self.cell, 2),
                }
            )
        gaps.sort(key=lambda item: (-item["cells"], -item["run_m"]))
        return gaps[:max_gaps]

    # ----------------------------------------------------------- 自由区与分区
    def build_free_area(self, seed):
        """从种子点漫游出机器人真正能走到的内部自由区。

        只沿"场地内 + 非占据 + 没有导航膨胀"的格子扩散：膨胀层就是规划的通行
        判据，用它一起算出来的自由区才和 A* 的连通性一致。之前未知格一律算自由，
        区域里会混进一堆被膨胀封死的死角，弓字起点全都"不可达"，只能一条条跳过，
        割草永远做不完。
        """
        mask = bytearray(self.cols * self.rows)
        start_col, start_row = self.cell_of(seed[0], seed[1])
        if not self.in_bounds(start_col, start_row):
            return False
        cols = self.cols
        data = self.data
        blocked = self.blocked
        start_index = self.index(start_col, start_row)
        inside = world.polygon_mask(self.cols, self.rows, self.cell)
        if not inside[start_index]:
            return False
        stack = [start_index]
        mask[start_index] = 1
        # 用下标而不是坐标元组做 DFS：实景图 7 万格时能省一半以上开销
        offsets = (-cols - 1, -cols, -cols + 1, -1, 1, cols - 1, cols, cols + 1)
        while stack:
            current = stack.pop()
            current_col = current % cols
            for offset in offsets:
                nindex = current + offset
                if nindex < 0 or nindex >= len(data):
                    continue
                if mask[nindex] or not inside[nindex]:
                    continue
                if data[nindex] == OCCUPIED or blocked[nindex]:
                    continue
                # 行偏移会跨行，必须校验列变化
                if offset in (-1, 1):
                    if current_col + offset < 0 or current_col + offset >= cols:
                        continue
                elif offset in (-cols - 1, -cols + 1, cols - 1, cols + 1):
                    if abs((nindex % cols) - current_col) != 1:
                        continue
                mask[nindex] = 1
                stack.append(nindex)

        self.free_mask = mask
        self.mask_ready = True
        self.version += 1
        return True

    def partition_zones(self):
        """按实际连通自由区划分区域；规则连通域保持整体，哑铃形才拆分。"""
        self.zones = []
        self.zone_totals = {}
        self.zone_mowed = {}
        min_zone_free = max(
            100, int(round(MIN_ZONE_AREA / (self.cell * self.cell)))
        )
        components = self._free_components()
        if not components:
            return self.zones

        zone_cells = []
        for component in components:
            zone_cells.extend(self._split_component(component, min_zone_free))
        zone_cells = self._merge_small_components(zone_cells, min_zone_free)

        normalized = []
        for cells in zone_cells:
            if not cells:
                continue
            cols_ = [cell % self.cols for cell in cells]
            rows_ = [cell // self.cols for cell in cells]
            c0, c1 = min(cols_), max(cols_)
            r0, r1 = min(rows_), max(rows_)
            x0 = c0 * self.cell
            y0 = r0 * self.cell
            x1 = (c1 + 1) * self.cell
            y1 = (r1 + 1) * self.cell
            cx = sum((cell % self.cols + 0.5) * self.cell for cell in cells) / len(cells)
            cy = sum((cell // self.cols + 0.5) * self.cell for cell in cells) / len(cells)
            normalized.append(
                {
                    "x0": round(x0, 2),
                    "y0": round(y0, 2),
                    "x1": round(x1, 2),
                    "y1": round(y1, 2),
                    "cx": round(cx, 2),
                    "cy": round(cy, 2),
                    "free": len(cells),
                    "outline": self.zone_outline(cells),
                    "cells": cells,
                }
            )

        normalized.sort(key=lambda zone: (-zone["cy"], zone["cx"]))
        self.zone_id_map = bytearray(self.cols * self.rows)
        for index, zone in enumerate(normalized, start=1):
            zone["id"] = index
            zone["name"] = "区域 {}".format(index)
            self.zones.append(zone)
            self.zone_totals[index] = zone["free"]
            self.zone_mowed[index] = 0
            for cell in zone.pop("cells"):
                self.zone_id_map[cell] = index
        self.free_total = sum(1 for value in self.free_mask if value)
        self.covered_free = sum(self.zone_totals.values())
        self.version += 1
        return self.zones

    def zone_outline(self, cells):
        """把区域栅格集合转成闭合折线（沿栅格外沿），供界面绘制真实区域形状。"""
        if not cells:
            return []
        members = set(cells)
        edges = {}
        for index in members:
            col = index % self.cols
            row = index // self.cols
            x0 = col * self.cell
            y0 = row * self.cell
            x1 = x0 + self.cell
            y1 = y0 + self.cell
            if index - self.cols not in members:  # 下边 (x0,y0) -> (x1,y0)
                edges.setdefault((round(x0, 2), round(y0, 2)), []).append(
                    (round(x1, 2), round(y0, 2))
                )
            if index + 1 not in members or col + 1 >= self.cols:  # 右边
                edges.setdefault((round(x1, 2), round(y0, 2)), []).append(
                    (round(x1, 2), round(y1, 2))
                )
            if index + self.cols not in members:  # 上边
                edges.setdefault((round(x1, 2), round(y1, 2)), []).append(
                    (round(x0, 2), round(y1, 2))
                )
            if index - 1 not in members or col == 0:  # 左边
                edges.setdefault((round(x0, 2), round(y1, 2)), []).append(
                    (round(x0, 2), round(y0, 2))
                )
        loops = []
        remaining = sum(len(v) for v in edges.values())
        guard = remaining + 8
        while edges and guard > 0:
            guard -= 1
            start = next(iter(edges))
            loop = [start]
            current = start
            closed = False
            while guard > 0:
                guard -= 1
                options = edges.get(current)
                if not options:
                    break
                nxt = options.pop()
                if not options:
                    edges.pop(current, None)
                if nxt == start:
                    closed = True
                    break
                loop.append(nxt)
                current = nxt
            if closed and len(loop) >= 4:
                loops.append(loop)
        if not loops:
            return []
        loops.sort(key=lambda loop: abs(_polygon_area(loop)), reverse=True)
        loop = loops[0]
        simplified = []
        count = len(loop)
        for i in range(count):
            prev = loop[(i - 1) % count]
            cur = loop[i]
            nxt = loop[(i + 1) % count]
            cross = (cur[0] - prev[0]) * (nxt[1] - cur[1]) - (cur[1] - prev[1]) * (
                nxt[0] - cur[0]
            )
            if abs(cross) > 1e-9:
                simplified.append((round(cur[0], 2), round(cur[1], 2)))
        return simplified if len(simplified) >= 3 else []

    def _free_components(self):
        visited = set()
        components = []
        for start in range(self.cols * self.rows):
            if not self.free_mask[start] or start in visited:
                continue
            visited.add(start)
            stack = [start]
            cells = []
            while stack:
                index = stack.pop()
                cells.append(index)
                col = index % self.cols
                row = index // self.cols
                for dc in (-1, 0, 1):
                    for dr in (-1, 0, 1):
                        if dc == 0 and dr == 0:
                            continue
                        nc, nr = col + dc, row + dr
                        if not self.in_bounds(nc, nr):
                            continue
                        nindex = self.index(nc, nr)
                        if self.free_mask[nindex] and nindex not in visited:
                            visited.add(nindex)
                            stack.append(nindex)
            components.append(set(cells))
        return components

    def _component_distances(self, start, cells):
        from collections import deque

        cols = self.cols
        offsets = (-cols - 1, -cols, -cols + 1, -1, 1, cols - 1, cols, cols + 1)
        size = cols * self.rows
        distances = {start: 0}
        queue = deque([start])
        while queue:
            index = queue.popleft()
            step = distances[index] + 1
            col = index % cols
            for offset in offsets:
                nindex = index + offset
                if nindex < 0 or nindex >= size:
                    continue
                if offset in (-1, 1):
                    ncol = col + offset
                    if ncol < 0 or ncol >= cols:
                        continue
                elif offset in (-cols - 1, -cols + 1, cols - 1, cols + 1):
                    if abs(nindex % cols - col) != 1:
                        continue
                if nindex not in cells or nindex in distances:
                    continue
                distances[nindex] = step
                queue.append(nindex)
        return distances

    def _split_component(self, cells, min_zone_free):
        if len(cells) < 2 * min_zone_free:
            return [cells]
        hull_area = self._convex_hull_area(cells)
        solidity = len(cells) / max(1.0, hull_area)
        if solidity >= REGULAR_SOLIDITY:
            return [cells]

        start = next(iter(cells))
        first_dist = self._component_distances(start, cells)
        endpoint_a = max(first_dist, key=first_dist.get)
        dist_a = self._component_distances(endpoint_a, cells)
        endpoint_b = max(dist_a, key=dist_a.get)
        dist_b = self._component_distances(endpoint_b, cells)
        part_a = set()
        part_b = set()
        for index in cells:
            if dist_a.get(index, 1e18) <= dist_b.get(index, 1e18):
                part_a.add(index)
            else:
                part_b.add(index)
        if (
            len(part_a) < min_zone_free
            or len(part_b) < min_zone_free
            or not part_a
            or not part_b
        ):
            return [cells]
        return [part_a, part_b]

    def _convex_hull_area(self, cells):
        points = set()
        for index in cells:
            col = index % self.cols
            row = index // self.cols
            boundary = False
            for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nc, nr = col + dc, row + dr
                if not self.in_bounds(nc, nr) or self.index(nc, nr) not in cells:
                    boundary = True
                    break
            if boundary:
                points.update(
                    ((col, row), (col + 1, row), (col, row + 1), (col + 1, row + 1))
                )
        points = sorted(points)
        if len(points) < 3:
            return float(len(cells))
        return _polygon_area(_convex_hull(points))

    def _merge_small_components(self, zones, min_zone_free):
        groups = [set(cells) for cells in zones if cells]
        while len(groups) > 1:
            small = [index for index, cells in enumerate(groups) if len(cells) < min_zone_free]
            if not small:
                break
            index = min(small, key=lambda item: len(groups[item]))
            best = None
            for other_index, other in enumerate(groups):
                if other_index == index:
                    continue
                gap = self._cell_group_gap(groups[index], other)
                if best is None or gap < best[0]:
                    best = (gap, other_index)
            if best is None:
                break
            other_index = best[1]
            groups[index].update(groups[other_index])
            groups.pop(other_index)
        return groups

    def _cell_group_gap(self, a, b):
        def bounds(cells):
            cols_ = [cell % self.cols for cell in cells]
            rows_ = [cell // self.cols for cell in cells]
            return min(cols_), min(rows_), max(cols_), max(rows_)

        ax0, ay0, ax1, ay1 = bounds(a)
        bx0, by0, bx1, by1 = bounds(b)
        dx = max(0, bx0 - ax1 - 1, ax0 - bx1 - 1)
        dy = max(0, by0 - ay1 - 1, ay0 - by1 - 1)
        return dx + dy

    def _build_prefix(self):
        prefix = [[0] * (self.cols + 1) for _ in range(self.rows + 1)]
        for row in range(self.rows):
            row_sum = 0
            for col in range(self.cols):
                row_sum += 1 if self.free_mask[self.index(col, row)] else 0
                prefix[row + 1][col + 1] = prefix[row][col + 1] + row_sum
        return prefix

    def zone_at(self, x, y):
        col, row = self.cell_of(x, y)
        if self.in_bounds(col, row) and self.zone_id_map:
            zone_id = self.zone_id_map[self.index(col, row)]
            if zone_id:
                for zone in self.zones:
                    if zone["id"] == zone_id:
                        return zone
        return None

    def cell_in_zone(self, col, row, zone_id):
        if not self.zone_id_map:
            return True
        if not self.in_bounds(col, row):
            return False
        return self.zone_id_map[self.index(col, row)] == zone_id

    # ----------------------------------------------------------- 覆盖
    def mark_mowed(self, x, y, radius=None):
        # 栅格中心与连续刀盘边界存在最多半格量化误差，补 0.3 格避免
        # 沿边时把实际已割到的区域误记成漏扫。
        radius = (
            world.MOWER_WIDTH / 2.0 + world.CELL * 0.3
            if radius is None
            else radius
        )
        col0, row0 = self.cell_of(x - radius, y - radius)
        col1, row1 = self.cell_of(x + radius, y + radius)
        for col in range(col0, col1 + 1):
            for row in range(row0, row1 + 1):
                cx, cy = self.center_of(col, row)
                if (cx - x) ** 2 + (cy - y) ** 2 > radius ** 2:
                    continue
                index = self.index(col, row)
                if self.mask_ready and not self.free_mask[index]:
                    continue
                if self.mowed[index]:
                    continue
                self.mowed[index] = 1
                zone = self.zone_at(cx, cy)
                if zone is not None:
                    self.zone_mowed[zone["id"]] += 1

    def zone_coverage(self, zone_id):
        total = self.zone_totals.get(zone_id, 0)
        if not total:
            return 0.0
        return min(1.0, self.zone_mowed.get(zone_id, 0) / float(total))

    def cell_mowed(self, col, row):
        return bool(self.mowed[self.index(col, row)])

    def cell_blocked(self, col, row):
        return bool(self.blocked[self.index(col, row)])

    def cell_blocked_mow(self, col, row):
        return bool(self.mow_blocked[self.index(col, row)])

    def cell_mowable(self, col, row):
        index = self.index(col, row)
        if self.data[index] == OCCUPIED:
            return False
        if self.mask_ready and not self.free_mask[index]:
            return False
        if self._reach_seed is not None:
            if self._reach_version != self.data_version:
                self._rebuild_reach()
            if not self.reach_mask[index]:
                return False
        return True

    def set_reach_seed(self, x, y):
        """指定"可达性"的起点（一般是车当前位置）。不设置时不做可达过滤。"""
        self._reach_seed = (float(x), float(y))
        self._reach_version = -1

    def _rebuild_reach(self):
        """从 seed 做一次连通域填充（按行扫描，几十毫秒级）。

        只把"场地内 + 非占据格"当作可走：已探明障碍物形成的占据环会把障碍物
        内部那些从没被看到的未知格封在外面，于是弓字不会再往障碍物内部铺。
        """
        cols = self.cols
        rows = self.rows
        data = self.data
        inside = world.polygon_mask(cols, rows, self.cell)
        walkable = bytearray(cols * rows)
        for index in range(cols * rows):
            if inside[index] and data[index] != OCCUPIED:
                walkable[index] = 1
        mask = bytearray(cols * rows)
        seed = self._reach_seed
        col, row = self.cell_of(seed[0], seed[1])
        col = min(max(col, 0), cols - 1)
        row = min(max(row, 0), rows - 1)
        start = row * cols + col
        if not walkable[start]:
            start = None
            radius = max(2, int(1.0 / self.cell))
            for dr in range(-radius, radius + 1):
                r2 = row + dr
                if not (0 <= r2 < rows):
                    continue
                for dc in range(-radius, radius + 1):
                    c2 = col + dc
                    if 0 <= c2 < cols and walkable[r2 * cols + c2]:
                        start = r2 * cols + c2
                        break
                if start is not None:
                    break
        if start is None:
            self.reach_mask = mask
            self._reach_version = self.data_version
            return
        self._fill_reach(mask, walkable, start)
        walkable_total = sum(walkable)
        if sum(mask) < 0.5 * walkable_total:
            # 车被临时围在一个小角落里（视觉碎片把门口糊死）时，以充电桩所在的
            # 连通域兜底，别因为可达区太小把整场弓字都裁掉。
            dock_col, dock_row = self.cell_of(world.DOCK["x"], world.DOCK["y"])
            dock_index = dock_row * cols + dock_col
            if walkable[dock_index] and not mask[dock_index]:
                self._fill_reach(mask, walkable, dock_index)
        self.reach_mask = mask
        self._reach_version = self.data_version

    def _fill_reach(self, mask, walkable, start):
        """按行扫描的连通域填充（8 邻域不必要，4 邻域足够判断"封死"）。"""
        cols = self.cols
        rows = self.rows
        stack = [(start // cols, start % cols)]
        while stack:
            r, c = stack.pop()
            base = r * cols
            if not walkable[base + c] or mask[base + c]:
                continue
            left = c
            while left > 0 and walkable[base + left - 1] and not mask[base + left - 1]:
                left -= 1
            right = c
            while right < cols - 1 and walkable[base + right + 1] and not mask[base + right + 1]:
                right += 1
            for cc in range(left, right + 1):
                mask[base + cc] = 1
            for nr in (r - 1, r + 1):
                if nr < 0 or nr >= rows:
                    continue
                nbase = nr * cols
                cc = left
                while cc <= right:
                    if walkable[nbase + cc] and not mask[nbase + cc]:
                        stack.append((nr, cc))
                        while cc <= right and walkable[nbase + cc] and not mask[nbase + cc]:
                            cc += 1
                    cc += 1

    def apply_dynamic(self, circles):
        """动态障碍物临时占据层：每 tick 重建，不写入建图数据（否则会留下永久"拖影"）。

        只用于行进路径避让，不参与弓字线规划与可割判定 —— 移动物会让栅格每帧变化，
        一旦参与规划就要不断重排弓字线，既拖慢速度又留下大量漏扫。
        贴近时的制动由 ``TaskManager._guard_dynamic`` 负责。
        """
        self.dyn_blocked = bytearray(self.cols * self.rows)
        marked = 0
        for circle in circles or []:
            cx = float(circle["x"])
            cy = float(circle["y"])
            radius = float(circle["r"])
            r_block = radius + 0.30
            col0, row0 = self.cell_of(cx - r_block - 0.1, cy - r_block - 0.1)
            col1, row1 = self.cell_of(cx + r_block + 0.1, cy + r_block + 0.1)
            for row in range(row0, row1 + 1):
                for col in range(col0, col1 + 1):
                    x, y = self.center_of(col, row)
                    if math.hypot(x - cx, y - cy) > r_block:
                        continue
                    self.dyn_blocked[self.index(col, row)] = 1
                    marked += 1
        return marked

    def unmowed_components(self, zone_id, min_cells=1):
        """返回区域内未割草且可割草栅格的连通块。"""
        visited = set()
        components = []
        for row in range(self.rows):
            for col in range(self.cols):
                index = self.index(col, row)
                if index in visited:
                    continue
                if not self.cell_in_zone(col, row, zone_id):
                    continue
                if (
                    not self.cell_mowable(col, row)
                    or self.cell_blocked_mow(col, row)
                    or self.cell_mowed(col, row)
                ):
                    continue
                queue = [(col, row)]
                visited.add(index)
                cells = []
                while queue:
                    c, r = queue.pop()
                    cells.append((c, r))
                    for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        nc, nr = c + dc, r + dr
                        if not self.in_bounds(nc, nr):
                            continue
                        nindex = self.index(nc, nr)
                        if nindex in visited:
                            continue
                        if not self.cell_in_zone(nc, nr, zone_id):
                            continue
                        if (
                            not self.cell_mowable(nc, nr)
                            or self.cell_blocked_mow(nc, nr)
                            or self.cell_mowed(nc, nr)
                        ):
                            continue
                        visited.add(nindex)
                        queue.append((nc, nr))
                if len(cells) >= min_cells:
                    components.append(cells)
        return components

    def release_blocked_disc(self, x, y, radius):
        """放开某点周边导航层的膨胀（最后一招：车体卡在狭窄缝隙时给它一条出路）。

        只清 ``blocked``（导航层），不动 ``data``（建图占据）和 ``mow_blocked``
        （刀盘层），所以不会把障碍物当成空地去割。
        """
        col0, row0 = self.cell_of(x - radius, y - radius)
        col1, row1 = self.cell_of(x + radius, y + radius)
        released = 0
        for row in range(row0, row1 + 1):
            for col in range(col0, col1 + 1):
                if not self.in_bounds(col, row):
                    continue
                cx, cy = self.center_of(col, row)
                if math.hypot(cx - x, cy - y) > radius:
                    continue
                index = self.index(col, row)
                if self.blocked[index] and self.data[index] != OCCUPIED:
                    self.blocked[index] = 0
                    released += 1
        if released:
            self.version += 1
        return released

    def is_blocked_world(self, x, y):
        col, row = self.cell_of(x, y)
        index = self.index(col, row)
        return bool(self.blocked[index] or self.dyn_blocked[index])

    def is_open_world(self, x, y):
        col, row = self.cell_of(x, y)
        if not self.in_bounds(col, row):
            return False
        return self._open(col, row)

    def is_occupied_world(self, x, y):
        col, row = self.cell_of(x, y)
        return self.data[self.index(col, row)] == OCCUPIED

    def occupied_near(self, x, y, radius=0.12):
        col, row = self.cell_of(x, y)
        span = max(1, int(math.ceil(radius / self.cell)))
        for dc in range(-span, span + 1):
            for dr in range(-span, span + 1):
                c, r = col + dc, row + dr
                if self.in_bounds(c, r) and self.data[self.index(c, r)] == OCCUPIED:
                    return True
        return False

    def explore_goal(self, pose, step=5, min_distance=2.0):
        """找一个“还没探明、但紧贴已探明自由区”的栅格作为建图去向（只在自身栅格上搜索）。

        建图被困在小区域时，用它把机器人引到最近的未探明边界：最近的更稳，
        最远的往往要穿过还没探明的区域。
        """
        cols, rows = self.cols, self.rows
        data = self.data
        best = None
        best_distance = None
        x0, y0 = pose[0], pose[1]
        step = max(2, int(step))
        for row in range(step, rows - step, step):
            base = row * cols
            for col in range(step, cols - step, step):
                if data[base + col] != UNKNOWN:
                    continue
                neighbour_free = False
                for dc, dr in ((step, 0), (-step, 0), (0, step), (0, -step)):
                    if data[(row + dr) * cols + col + dc] == FREE:
                        neighbour_free = True
                        break
                if not neighbour_free:
                    continue
                x, y = self.center_of(col, row)
                distance = math.hypot(x - x0, y - y0)
                if distance < min_distance:
                    continue
                if best_distance is None or distance < best_distance:
                    best_distance = distance
                    best = (x, y)
        return best

    def boundary_distance_world(self, x, y):
        """到最近“已探明占据栅格”的距离；用于避开已经贴住的墙根。"""
        col, row = self.cell_of(x, y)
        span = 3
        best = span * self.cell
        for dc in range(-span, span + 1):
            for dr in range(-span, span + 1):
                c, r = col + dc, row + dr
                if not self.in_bounds(c, r):
                    continue
                if self.data[self.index(c, r)] != OCCUPIED:
                    continue
                cx, cy = self.center_of(c, r)
                best = min(best, math.hypot(cx - x, cy - y))
        return best

    def relax_trail(self, points, radius=0.35):
        """沿一条“机器人真的走过”的轨迹放开导航膨胀，保证原路可以退回。"""
        changed = 0
        for point in points:
            if self.relax_around(point[0], point[1], radius):
                changed += 1
        return changed

    def relax_around(self, x, y, radius=0.30, clear_occupied=False):
        """临时清除机器人所在位置的导航膨胀（机器人已证明该点可通行）。

        ``clear_occupied`` 用于充电桩这类"车确实停得下"的固定点：视觉盖章有时
        会把桩位刷成占据格，那样规划永远到不了桩，只能把这一小片恢复成自由格。
        """
        changed = False
        col, row = self.cell_of(x, y)
        span = max(1, int(math.ceil(radius / self.cell)))
        for dc in range(-span, span + 1):
            for dr in range(-span, span + 1):
                c, r = col + dc, row + dr
                if not self.in_bounds(c, r):
                    continue
                index = self.index(c, r)
                ox, oy = self.center_of(c, r)
                if math.hypot(ox - x, oy - y) > radius:
                    continue
                if self.data[index] == OCCUPIED:
                    if not clear_occupied:
                        continue
                    self.data[index] = FREE
                    self.verified_free[index] = 1
                    self.data_version += 1
                    changed = True
                if self.blocked[index]:
                    self.blocked[index] = 0
                    changed = True
        return changed

    def nearest_occupied(self, x, y, radius=0.55):
        best = None
        col, row = self.cell_of(x, y)
        span = max(1, int(math.ceil(radius / self.cell)))
        for dc in range(-span, span + 1):
            for dr in range(-span, span + 1):
                c, r = col + dc, row + dr
                if not self.in_bounds(c, r):
                    continue
                if self.data[self.index(c, r)] != OCCUPIED:
                    continue
                ox, oy = self.center_of(c, r)
                distance = math.hypot(ox - x, oy - y)
                if distance <= radius and (best is None or distance < best[0]):
                    best = (distance, ox, oy)
        return best

    # ----------------------------------------------------------- 障碍聚类
    def obstacle_circles(self, zone=None):
        if self._circle_cache_version != self.data_version:
            self._circle_cache = self._compute_obstacle_circles()
            self._circle_cache_version = self.data_version
        if zone is None:
            return self._circle_cache
        expanded = (
            zone["x0"] - 0.6,
            zone["y0"] - 0.6,
            zone["x1"] + 0.6,
            zone["y1"] + 0.6,
        )
        return [
            circle
            for circle in self._circle_cache
            if expanded[0] <= circle["x"] <= expanded[2]
            and expanded[1] <= circle["y"] <= expanded[3]
        ]

    def _compute_obstacle_circles(self):
        visited = set()
        circles = []
        for row in range(self.rows):
            for col in range(self.cols):
                index = self.index(col, row)
                if index in visited or self.data[index] != OCCUPIED:
                    continue
                queue = [(col, row)]
                visited.add(index)
                cells = []
                while queue:
                    c, r = queue.pop()
                    cells.append((c, r))
                    for dc in (-1, 0, 1):
                        for dr in (-1, 0, 1):
                            nc, nr = c + dc, r + dr
                            if not self.in_bounds(nc, nr):
                                continue
                            nindex = self.index(nc, nr)
                            if nindex in visited or self.data[nindex] != OCCUPIED:
                                continue
                            visited.add(nindex)
                            queue.append((nc, nr))
                    if len(cells) > MAX_OBSTACLE_CELLS:
                        break

                if len(cells) < 6 or len(cells) > MAX_OBSTACLE_CELLS:
                    continue
                xs = []
                ys = []
                for c, r in cells:
                    x, y = self.center_of(c, r)
                    xs.append(x)
                    ys.append(y)
                if (
                    max(xs) - min(xs) > MAX_OBSTACLE_EXTENT
                    or max(ys) - min(ys) > MAX_OBSTACLE_EXTENT
                ):
                    continue
                # 只使用视觉建图得到的占据栅格，不读取场地真值。
                min_x, max_x = min(xs), max(xs)
                min_y, max_y = min(ys), max(ys)
                extent = max(max_x - min_x, max_y - min_y)
                cx = (min_x + max_x) / 2.0
                cy = (min_y + max_y) / 2.0
                if world.boundary_distance(cx, cy) < 0.42:
                    continue
                radius = max(
                    math.hypot(x - cx, y - cy) for x, y in zip(xs, ys)
                ) + 0.04
                if extent < 0.55:
                    fitted = _fit_circle(zip(xs, ys), cx, cy)
                    if fitted is not None:
                        fx, fy, fr = fitted
                        if math.hypot(fx - cx, fy - cy) < 0.35:
                            cx, cy = fx, fy
                            radius = max(fr, radius - 0.06)
                # 只观测到弧段时质心会偏移、半径偏大；用包围盒半宽收一下，
                # 避免障碍物周围被"涂抹"出大片不可割区域。
                radius = min(radius, 0.55 * extent + 0.06)
                radius = max(0.20, min(radius, 2.00))
                # 按拟合圆估算面积：小于 5 m² 的一律视为小型障碍物（绕障 + 花瓣探明）。
                area = math.pi * radius * radius
                kind = "large" if area >= LARGE_OBSTACLE_AREA else "small"
                circles.append(
                    {
                        "x": round(cx, 2),
                        "y": round(cy, 2),
                        "r": round(radius, 2),
                        "kind": kind,
                        "extent": round(extent, 2),
                    }
                )
        return circles

    # ----------------------------------------------------------- 规划
    def plan(
        self,
        start,
        goal,
        allow_unknown=True,
        ignore_blocked=False,
        max_expansions=40000,
        time_budget=0.09,
    ):
        start_col, start_row = self._nearest_open(start)
        goal_col, goal_row = self._nearest_open(goal)
        if start_col is None or goal_col is None:
            return []

        start_index = self.index(start_col, start_row)
        goal_index = self.index(goal_col, goal_row)
        if start_index == goal_index:
            return [self.center_of(goal_col, goal_row)]

        open_heap = [(0.0, start_index)]
        came_from = {}
        cost_so_far = {start_index: 0.0}
        found = False
        expansions = 0

        deadline = time.monotonic() + max(0.0, time_budget)
        while open_heap and expansions < max_expansions:
            _, current = heapq.heappop(open_heap)
            expansions += 1
            if expansions & 0x1FF == 0 and time.monotonic() > deadline:
                # 超时放弃：宁可这一轮找不到路，也不要把仿真线程卡住
                return []
            if current == goal_index:
                found = True
                break
            col = current % self.cols
            row = current // self.cols
            for dc, dr, step_cost in _NEIGHBORS:
                nc, nr = col + dc, row + dr
                if not self.in_bounds(nc, nr):
                    continue
                nindex = self.index(nc, nr)
                if not ignore_blocked and self.blocked[nindex]:
                    continue
                cdata = self.data[nindex]
                if cdata == OCCUPIED:
                    continue
                if self.mask_ready and not self.free_mask[nindex]:
                    continue
                extra = 0.0
                if cdata == UNKNOWN:
                    if not allow_unknown:
                        continue
                    extra = 0.45
                elif ignore_blocked:
                    # 忽略膨胀层时，膨涨区仍按“可疑”加一点代价，尽量走已探明走廊
                    if self.blocked[nindex]:
                        extra = 0.6
                new_cost = cost_so_far[current] + step_cost + extra
                if nindex not in cost_so_far or new_cost < cost_so_far[nindex] - 1e-9:
                    cost_so_far[nindex] = new_cost
                    priority = new_cost + _octile(nc, nr, goal_col, goal_row)
                    heapq.heappush(open_heap, (priority, nindex))
                    came_from[nindex] = current

        if not found:
            return []

        path = []
        node = goal_index
        while node != start_index:
            col = node % self.cols
            row = node // self.cols
            path.append(self.center_of(col, row))
            node = came_from.get(node)
            if node is None:
                return []
        path.append(self.center_of(start_col, start_row))
        path.reverse()
        return self.smooth(path)

    def plan_near(
        self,
        start,
        goal,
        zone_id=None,
        max_expansions=200000,
        time_budget=0.35,
    ):
        """当目标点不可达时，规划到当前位置连通域内离目标最近的栅格。

        默认给足时间和扩展次数：实景大图上连通域有 9 万格，预算给小了会中途放弃，
        把"其实开得过去"的目标误判成不可达（外部再据此跳过整段弓字/区域）。
        """
        from collections import deque

        start_col, start_row = self._nearest_open(start)
        if start_col is None:
            return [], float("inf")
        start_index = self.index(start_col, start_row)
        goal_index = self.index(*self.cell_of(*goal))
        queue = deque([start_index])
        came_from = {start_index: None}

        def metric(index):
            if zone_id is not None and self.zone_id_map[index] != zone_id:
                return float("inf")
            x, y = self.center_of(index % self.cols, index // self.cols)
            return math.hypot(x - goal[0], y - goal[1])

        best_index = start_index
        best_distance = metric(start_index)
        expansions = 0
        deadline = time.monotonic() + max(0.0, time_budget)
        while queue and expansions < max_expansions:
            current = queue.popleft()
            expansions += 1
            if expansions & 0x3FF == 0 and time.monotonic() > deadline:
                break
            col = current % self.cols
            row = current // self.cols
            distance = metric(current)
            if distance < best_distance:
                best_distance = distance
                best_index = current
                if best_distance <= 0.55:
                    break
            if current == goal_index:
                best_index = current
                best_distance = 0.0
                break
            for dc, dr, _cost in _NEIGHBORS:
                nc, nr = col + dc, row + dr
                if not self._open(nc, nr):
                    continue
                nindex = self.index(nc, nr)
                if nindex in came_from:
                    continue
                if self.data[nindex] == OCCUPIED:
                    continue
                came_from[nindex] = current
                queue.append(nindex)
        if best_index == start_index and best_distance > 0.55:
            return [], best_distance
        path = []
        node = best_index
        while node is not None:
            path.append(self.center_of(node % self.cols, node // self.cols))
            node = came_from.get(node)
        path.reverse()
        return self.smooth(path), best_distance

    def smooth(self, path):
        if len(path) <= 2:
            return path
        result = [path[0]]
        index = 0
        while index < len(path) - 1:
            probe = len(path) - 1
            while probe > index + 1 and not self.line_clear(path[index], path[probe]):
                probe -= 1
            result.append(path[probe])
            index = probe
        return result

    def line_clear(self, a, b):
        ax, ay = a
        bx, by = b
        distance = math.hypot(bx - ax, by - ay)
        steps = max(1, int(distance / 0.05))
        for index in range(1, steps + 1):
            t = index / float(steps)
            x = ax + (bx - ax) * t
            y = ay + (by - ay) * t
            if self.is_blocked_world(x, y):
                return False
            if self.mask_ready:
                col, row = self.cell_of(x, y)
                if not self.free_mask[self.index(col, row)]:
                    return False
        return True

    def _nearest_open(self, point):
        col, row = self.cell_of(*point)
        if self._open(col, row):
            return col, row
        best = None
        for radius in range(1, 12):
            for dc in range(-radius, radius + 1):
                for dr in range(-radius, radius + 1):
                    if max(abs(dc), abs(dr)) != radius:
                        continue
                    c, r = col + dc, row + dr
                    if self._open(c, r):
                        d = math.hypot(dc, dr)
                        if best is None or d < best[0]:
                            best = (d, c, r)
            if best is not None:
                return best[1], best[2]
        return None, None

    def _open(self, col, row):
        if not self.in_bounds(col, row):
            return False
        index = self.index(col, row)
        if self.blocked[index] or self.dyn_blocked[index]:
            return False
        if self.mask_ready and not self.free_mask[index]:
            return False
        return True

    # ----------------------------------------------------------- 编码
    def encode(self):
        occupancy = bytes(self.data).translate(_ASCII_TABLE).decode("ascii")
        mowed = bytes(self.mowed).translate(_ASCII_TABLE).decode("ascii")
        return {
            "cell": self.cell,
            "cols": self.cols,
            "rows": self.rows,
            "occupancy": occupancy,
            "mowed": mowed,
            "version": self.version,
        }


_ASCII_TABLE = bytes.maketrans(b"\x00\x01\x02", b"012")


def _polygon_area(points):
    area = 0.0
    for index, point in enumerate(points):
        nxt = points[(index + 1) % len(points)]
        area += point[0] * nxt[1] - nxt[0] * point[1]
    return abs(area) * 0.5


def _convex_hull(points):
    def cross(origin, a, b):
        return (a[0] - origin[0]) * (b[1] - origin[1]) - (
            a[1] - origin[1]
        ) * (b[0] - origin[0])

    lower = []
    for point in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


_NEIGHBORS = (
    (1, 0, 1.0),
    (-1, 0, 1.0),
    (0, 1, 1.0),
    (0, -1, 1.0),
    (1, 1, 1.4142),
    (1, -1, 1.4142),
    (-1, 1, 1.4142),
    (-1, -1, 1.4142),
)


def _octile(col, row, goal_col, goal_row):
    dx = abs(col - goal_col)
    dy = abs(row - goal_row)
    return (dx + dy) + (1.4142 - 2.0) * min(dx, dy)


def _fit_circle(points, fallback_x, fallback_y):
    """对障碍物表面栅格做代数圆拟合，部分视野下也能还原圆心与半径。"""
    points = list(points)
    if len(points) < 8:
        return None
    n = float(len(points))
    sx = sum(point[0] for point in points)
    sy = sum(point[1] for point in points)
    sxx = sum(point[0] * point[0] for point in points)
    syy = sum(point[1] * point[1] for point in points)
    sxy = sum(point[0] * point[1] for point in points)
    sxz = sum(point[0] * (point[0] ** 2 + point[1] ** 2) for point in points)
    syz = sum(point[1] * (point[0] ** 2 + point[1] ** 2) for point in points)
    sz = sum(point[0] ** 2 + point[1] ** 2 for point in points)
    matrix = [
        [sxx, sxy, sx, sxz],
        [sxy, syy, sy, syz],
        [sx, sy, n, sz],
    ]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda row: abs(matrix[row][col]))
        if abs(matrix[pivot][col]) < 1e-9:
            return None
        matrix[col], matrix[pivot] = matrix[pivot], matrix[col]
        divisor = matrix[col][col]
        for index in range(col, 4):
            matrix[col][index] /= divisor
        for row in range(3):
            if row == col:
                continue
            factor = matrix[row][col]
            for index in range(col, 4):
                matrix[row][index] -= factor * matrix[col][index]
    a = matrix[0][3] / 2.0
    b = matrix[1][3] / 2.0
    c = matrix[2][3]
    value = c + a * a + b * b
    if value <= 0.0:
        return None
    radius = math.sqrt(value)
    if radius < 0.15 or radius > 1.5:
        return None
    if math.hypot(a - fallback_x, b - fallback_y) > 1.0:
        return None
    return a, b, radius


