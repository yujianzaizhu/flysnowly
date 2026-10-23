"""栅格地图：视觉建图、自由区填充、自动分区、覆盖统计与 A* 规划。"""

import heapq
import math

import world

UNKNOWN = 0
FREE = 1
OCCUPIED = 2

INFLATE_CELLS = 2
MOW_INFLATE_CELLS = 2
LARGE_OBSTACLE_EXTENT = 0.85
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
        self.data = bytearray(self.cols * self.rows)
        self.mowed = bytearray(self.cols * self.rows)
        self.blocked = bytearray(self.cols * self.rows)
        self.mow_blocked = bytearray(self.cols * self.rows)
        self.free_mask = bytearray(self.cols * self.rows)
        self.mask_ready = False
        self.zones = []
        self.zone_totals = {}
        self.zone_mowed = {}

    def reset_map(self):
        self._reset_arrays()
        self.version += 1

    def reset_mowed(self):
        self.mowed = bytearray(self.cols * self.rows)
        for zone_id in self.zone_mowed:
            self.zone_mowed[zone_id] = 0
        self.version += 1

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

    def mark_occupied(self, col, row):
        if not self.in_bounds(col, row):
            return
        changed = False
        for dc in (-1, 0, 1):
            for dr in (-1, 0, 1):
                c, r = col + dc, row + dr
                if not self.in_bounds(c, r):
                    continue
                index = self.index(c, r)
                if self.data[index] != OCCUPIED:
                    self.data[index] = OCCUPIED
                    changed = True
                self._block_around(c, r)
                self._block_mow_around(c, r)
        self.version += 1
        if changed:
            self.data_version += 1

    def _block_around(self, col, row):
        for dc in range(-INFLATE_CELLS, INFLATE_CELLS + 1):
            for dr in range(-INFLATE_CELLS, INFLATE_CELLS + 1):
                c, r = col + dc, row + dr
                if self.in_bounds(c, r):
                    self.blocked[self.index(c, r)] = 1

    def _block_mow_around(self, col, row):
        for dc in range(-MOW_INFLATE_CELLS, MOW_INFLATE_CELLS + 1):
            for dr in range(-MOW_INFLATE_CELLS, MOW_INFLATE_CELLS + 1):
                c, r = col + dc, row + dr
                if self.in_bounds(c, r):
                    self.mow_blocked[self.index(c, r)] = 1

    def stamp_obstacle(self, cx, cy, radius):
        """把已确认的圆形障碍物写入栅格，供割草与导航精确避让。"""
        r_occ = radius
        r_mow = radius + 0.25
        r_block = radius + 0.32
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
                if distance <= r_occ:
                    if self.data[index] != OCCUPIED:
                        self.data[index] = OCCUPIED
                        changed = True
                if distance <= r_mow:
                    self.mow_blocked[index] = 1
                if distance <= r_block:
                    self.blocked[index] = 1
        if changed:
            self.version += 1
            self.data_version += 1

    def update_from_scan(self, pose, readings, max_range):
        x, y, theta = pose
        cell = self.cell
        cols = self.cols
        rows = self.rows
        data = self.data
        changed_data = False

        def mark_free_grid(px, py):
            nonlocal changed_data
            col = int(px / cell)
            row = int(py / cell)
            if 0 <= col < cols and 0 <= row < rows:
                index = row * cols + col
                if data[index] == UNKNOWN:
                    data[index] = FREE
                    changed_data = True

        if not self.mask_ready:
            mark_free_grid(x, y)
        # 每格采样一次即可；0.05 m 采样会在同一栅格内重复写入多次。
        step = cell
        for relative, distance in readings:
            heading = theta + relative
            dx, dy = math.cos(heading), math.sin(heading)
            if not self.mask_ready:
                samples = int(distance / step)
                for index in range(1, samples + 1):
                    mark_free_grid(x + dx * step * index, y + dy * step * index)
            if distance < max_range - 1e-6:
                px = x + dx * max(0.0, distance - 0.02)
                py = y + dy * max(0.0, distance - 0.02)
                before = self.data_version
                self.mark_occupied(int(px / cell), int(py / cell))
                changed_data = changed_data or self.data_version != before
        if changed_data:
            self.data_version += 1
        self.version += 1

    def boundary_discovery(self):
        if not self.wall_cells:
            return 0.0
        if self._boundary_cache_version == self.data_version:
            return self._boundary_cache
        seen = 0
        data = self.data
        cols = self.cols
        for col, row in self.wall_cells:
            if data[row * cols + col] != UNKNOWN:
                seen += 1
        self._boundary_cache_version = self.data_version
        self._boundary_cache = seen / float(len(self.wall_cells))
        return self._boundary_cache

    # ----------------------------------------------------------- 自由区与分区
    def build_free_area(self, seed):
        """从种子点漫游出内部自由区（未知视为可通行，墙体为障碍）。"""
        mask = bytearray(self.cols * self.rows)
        start_col, start_row = self.cell_of(seed[0], seed[1])
        if not self.in_bounds(start_col, start_row):
            return False
        stack = [(start_col, start_row)]
        mask[self.index(start_col, start_row)] = 1
        touched_border = False
        while stack:
            col, row = stack.pop()
            if col == 0 or row == 0 or col == self.cols - 1 or row == self.rows - 1:
                touched_border = True
            for dc in (-1, 0, 1):
                for dr in (-1, 0, 1):
                    if dc == 0 and dr == 0:
                        continue
                    nc, nr = col + dc, row + dr
                    if not self.in_bounds(nc, nr):
                        continue
                    nindex = self.index(nc, nr)
                    if mask[nindex] or self.data[nindex] == OCCUPIED:
                        continue
                    mask[nindex] = 1
                    stack.append((nc, nr))

        if touched_border:
            mask = bytearray(self.cols * self.rows)
            for row in range(self.rows):
                for col in range(self.cols):
                    x, y = self.center_of(col, row)
                    if world.point_in_polygon(x, y) and self.data[self.index(col, row)] != OCCUPIED:
                        mask[self.index(col, row)] = 1
        else:
            # 用已构建的场地边界裁掉凹角处可能泄漏出去的少量栅格
            for row in range(self.rows):
                for col in range(self.cols):
                    index = self.index(col, row)
                    if mask[index]:
                        x, y = self.center_of(col, row)
                        if not world.point_in_polygon(x, y):
                            mask[index] = 0

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

        distances = {start: 0}
        queue = deque([start])
        while queue:
            index = queue.popleft()
            col = index % self.cols
            row = index // self.cols
            for dc, dr, _cost in _NEIGHBORS:
                nc, nr = col + dc, row + dr
                if not self.in_bounds(nc, nr):
                    continue
                nindex = self.index(nc, nr)
                if nindex not in cells or nindex in distances:
                    continue
                distances[nindex] = distances[index] + 1
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
        for zone in self.zones:
            if zone["x0"] <= x <= zone["x1"] and zone["y0"] <= y <= zone["y1"]:
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
        if self.data[self.index(col, row)] == OCCUPIED:
            return False
        if self.mask_ready and not self.free_mask[self.index(col, row)]:
            return False
        return True

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

    def is_blocked_world(self, x, y):
        col, row = self.cell_of(x, y)
        return bool(self.blocked[self.index(col, row)])

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

    def relax_around(self, x, y, radius=0.30):
        """临时清除机器人所在位置的导航膨胀（机器人已证明该点可通行）。"""
        changed = False
        col, row = self.cell_of(x, y)
        span = max(1, int(math.ceil(radius / self.cell)))
        for dc in range(-span, span + 1):
            for dr in range(-span, span + 1):
                c, r = col + dc, row + dr
                if not self.in_bounds(c, r):
                    continue
                index = self.index(c, r)
                if self.data[index] == OCCUPIED:
                    continue
                ox, oy = self.center_of(c, r)
                if math.hypot(ox - x, oy - y) <= radius and self.blocked[index]:
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
                    if len(cells) > 400:
                        break

                if len(cells) < 6 or len(cells) > 400:
                    continue
                xs = []
                ys = []
                for c, r in cells:
                    x, y = self.center_of(c, r)
                    xs.append(x)
                    ys.append(y)
                if max(xs) - min(xs) > 1.6 or max(ys) - min(ys) > 1.6:
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
                radius = max(0.20, min(radius, 1.40))
                kind = (
                    "large"
                    if extent >= LARGE_OBSTACLE_EXTENT or len(cells) >= 90
                    else "small"
                )
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
    def plan(self, start, goal, allow_unknown=True):
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

        while open_heap and expansions < 40000:
            _, current = heapq.heappop(open_heap)
            expansions += 1
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
                if self.blocked[nindex]:
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

    def plan_near(self, start, goal):
        """当目标点不可达时，规划到当前位置连通域内离目标最近的栅格。"""
        from collections import deque

        start_col, start_row = self._nearest_open(start)
        if start_col is None:
            return [], float("inf")
        start_index = self.index(start_col, start_row)
        goal_index = self.index(*self.cell_of(*goal))
        queue = deque([start_index])
        came_from = {start_index: None}

        def metric(index):
            x, y = self.center_of(index % self.cols, index // self.cols)
            return math.hypot(x - goal[0], y - goal[1])

        best_index = start_index
        best_distance = metric(start_index)
        expansions = 0
        while queue and expansions < 80000:
            current = queue.popleft()
            expansions += 1
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
        if self.blocked[index]:
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


