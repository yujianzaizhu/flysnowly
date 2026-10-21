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


class GridMap(object):
    def __init__(self):
        self.cell = world.CELL
        self.cols = world.COLS
        self.rows = world.ROWS
        self.version = 0
        self.wall_cells = world.boundary_cells()
        self.zones = []
        self.zone_totals = {}
        self.zone_mowed = {}
        self.free_mask = bytearray(self.cols * self.rows)
        self.mask_ready = False
        self._reset_arrays()

    def _reset_arrays(self):
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

    def mark_occupied(self, col, row):
        if not self.in_bounds(col, row):
            return
        for dc in (-1, 0, 1):
            for dr in (-1, 0, 1):
                c, r = col + dc, row + dr
                if not self.in_bounds(c, r):
                    continue
                index = self.index(c, r)
                self.data[index] = OCCUPIED
                self._block_around(c, r)
                self._block_mow_around(c, r)
        self.version += 1

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
                    self.data[index] = OCCUPIED
                    changed = True
                if distance <= r_mow:
                    self.mow_blocked[index] = 1
                if distance <= r_block:
                    self.blocked[index] = 1
        if changed:
            self.version += 1

    def update_from_scan(self, pose, readings, max_range):
        x, y, theta = pose
        self.mark_free(*self.cell_of(x, y))
        step = 0.05
        for relative, distance in readings:
            heading = theta + relative
            dx, dy = math.cos(heading), math.sin(heading)
            samples = int(distance / step)
            for index in range(1, samples + 1):
                px = x + dx * step * index
                py = y + dy * step * index
                self.mark_free(*self.cell_of(px, py))
            if distance < max_range - 1e-6:
                px = x + dx * max(0.0, distance - 0.02)
                py = y + dy * max(0.0, distance - 0.02)
                self.mark_occupied(*self.cell_of(px, py))
        self.version += 1

    def boundary_discovery(self):
        if not self.wall_cells:
            return 0.0
        seen = 0
        for col, row in self.wall_cells:
            if self.data[self.index(col, row)] != UNKNOWN:
                seen += 1
        return seen / float(len(self.wall_cells))

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
        """递归矩形切分自由区，生成尽量规则的割草区域。"""
        cols = [
            (col, row)
            for row in range(self.rows)
            for col in range(self.cols)
            if self.free_mask[self.index(col, row)]
        ]
        self.zones = []
        self.zone_totals = {}
        self.zone_mowed = {}
        if not cols:
            return self.zones

        prefix = self._build_prefix()
        min_col = min(col for col, _ in cols)
        max_col = max(col for col, _ in cols)
        min_row = min(row for _, row in cols)
        max_row = max(row for _, row in cols)

        rects = []

        def free_count(c0, r0, c1, r1):
            if c1 < c0 or r1 < r0:
                return 0
            return (
                prefix[r1 + 1][c1 + 1]
                - prefix[r0][c1 + 1]
                - prefix[r1 + 1][c0]
                + prefix[r0][c0]
            )

        def recurse(c0, r0, c1, r1, depth):
            count = free_count(c0, r0, c1, r1)
            if count <= 0:
                return
            width = c1 - c0 + 1
            height = r1 - r0 + 1
            area = width * height
            fill = count / float(area)
            if (
                fill >= 0.97
                or (area <= 2000 and fill >= 0.78)
                or area <= 400
                or depth >= 8
                or width <= 4
                or height <= 4
            ):
                rects.append((c0, r0, c1, r1))
                return

            best = None
            if width >= height:
                for cut in range(c0 + 3, c1 - 2, 2):
                    left = free_count(c0, r0, cut, r1)
                    right = free_count(cut + 1, r0, c1, r1)
                    if left == 0 or right == 0:
                        continue
                    strip = free_count(max(c0, cut - 1), r0, min(c1, cut + 1), r1)
                    cost = strip * 6.0 + abs(left - right) * 0.8
                    if best is None or cost < best[0]:
                        best = (cost, "v", cut)
            else:
                for cut in range(r0 + 3, r1 - 2, 2):
                    top = free_count(c0, r0, c1, cut)
                    bottom = free_count(c0, cut + 1, c1, r1)
                    if top == 0 or bottom == 0:
                        continue
                    strip = free_count(c0, max(r0, cut - 1), c1, min(r1, cut + 1))
                    cost = strip * 6.0 + abs(top - bottom) * 0.8
                    if best is None or cost < best[0]:
                        best = (cost, "h", cut)
            if best is None:
                rects.append((c0, r0, c1, r1))
                return
            if best[1] == "v":
                recurse(c0, r0, best[2], r1, depth + 1)
                recurse(best[2] + 1, r0, c1, r1, depth + 1)
            else:
                recurse(c0, r0, c1, best[2], depth + 1)
                recurse(c0, best[2] + 1, c1, r1, depth + 1)

        recurse(min_col, min_row, max_col, max_row, 0)

        # 合并相邻且接近矩形的分区，避免产生过多碎片
        while True:
            best = None
            for i in range(len(rects)):
                a = rects[i]
                if a is None:
                    continue
                for j in range(i + 1, len(rects)):
                    b = rects[j]
                    if b is None:
                        continue
                    union = None
                    if a[0] == b[0] and a[2] == b[2] and (a[3] + 1 == b[1] or b[3] + 1 == a[1]):
                        union = (a[0], min(a[1], b[1]), a[2], max(a[3], b[3]))
                    elif a[1] == b[1] and a[3] == b[3] and (a[2] + 1 == b[0] or b[2] + 1 == a[0]):
                        union = (min(a[0], b[0]), a[1], max(a[2], b[2]), a[3])
                    if union is None:
                        continue
                    area = (union[2] - union[0] + 1) * (union[3] - union[1] + 1)
                    if area > 2200:
                        continue
                    free = free_count(*union)
                    fill = free / float(area)
                    if fill < 0.64:
                        continue
                    score = fill - area / 12000.0
                    if best is None or score > best[0]:
                        best = (score, i, j, union)
            if best is None:
                break
            _, i, j, union = best
            rects[i] = union
            rects[j] = None
        rects = [rect for rect in rects if rect is not None]

        # 吸收仍偏小的分区，减少狭窄碎块
        while True:
            target = None
            for i, a in enumerate(rects):
                if a is None or free_count(*a) >= 250:
                    continue
                for j, b in enumerate(rects):
                    if b is None or i == j:
                        continue
                    if not (
                        a[0] <= b[2] + 2
                        and b[0] <= a[2] + 2
                        and a[1] <= b[3] + 2
                        and b[1] <= a[3] + 2
                    ):
                        continue
                    union = (
                        min(a[0], b[0]),
                        min(a[1], b[1]),
                        max(a[2], b[2]),
                        max(a[3], b[3]),
                    )
                    area = (union[2] - union[0] + 1) * (union[3] - union[1] + 1)
                    if area > 3000:
                        continue
                    free = free_count(*union)
                    if free / float(area) < 0.5:
                        continue
                    target = (i, j, union)
                    break
                if target is not None:
                    break
            if target is None:
                break
            i, j, union = target
            rects[i] = union
            rects[j] = None
        rects = [rect for rect in rects if rect is not None]

        normalized = []
        for c0, r0, c1, r1 in rects:
            count = free_count(c0, r0, c1, r1)
            if count < 40:
                continue
            box_area = (c1 - c0 + 1) * (r1 - r0 + 1)
            if box_area > 400 and count / float(box_area) < 0.30:
                continue
            x0 = c0 * self.cell
            y0 = r0 * self.cell
            x1 = (c1 + 1) * self.cell
            y1 = (r1 + 1) * self.cell
            normalized.append(
                {
                    "x0": round(x0, 2),
                    "y0": round(y0, 2),
                    "x1": round(x1, 2),
                    "y1": round(y1, 2),
                    "cx": round((x0 + x1) / 2.0, 2),
                    "cy": round((y0 + y1) / 2.0, 2),
                    "free": count,
                }
            )

        if not normalized:
            x0, y0 = self.center_of(min_col, min_row)
            x1, y1 = self.center_of(max_col, max_row)
            normalized.append(
                {
                    "x0": round(x0 - self.cell, 2),
                    "y0": round(y0 - self.cell, 2),
                    "x1": round(x1 + self.cell, 2),
                    "y1": round(y1 + self.cell, 2),
                    "cx": round((x0 + x1) / 2.0, 2),
                    "cy": round((y0 + y1) / 2.0, 2),
                    "free": len(cols),
                }
            )

        normalized.sort(key=lambda zone: (-zone["cy"], zone["cx"]))
        for index, zone in enumerate(normalized, start=1):
            zone["id"] = index
            zone["name"] = "区域 {}".format(index)
            self.zones.append(zone)
            self.zone_totals[index] = 0
            self.zone_mowed[index] = 0
        for row in range(self.rows):
            for col in range(self.cols):
                if not self.free_mask[self.index(col, row)]:
                    continue
                x, y = self.center_of(col, row)
                zone = self.zone_at(x, y)
                if zone is not None:
                    self.zone_totals[zone["id"]] += 1
        self.version += 1
        return self.zones

    def _build_prefix(self):
        prefix = [[0] * (self.cols + 1) for _ in range(self.rows + 1)]
        for row in range(self.rows):
            row_sum = 0
            for col in range(self.cols):
                row_sum += 1 if self.free_mask[self.index(col, row)] else 0
                prefix[row + 1][col + 1] = prefix[row][col + 1] + row_sum
        return prefix

    def zone_at(self, x, y):
        for zone in self.zones:
            if zone["x0"] <= x <= zone["x1"] and zone["y0"] <= y <= zone["y1"]:
                return zone
        return None

    # ----------------------------------------------------------- 覆盖
    def mark_mowed(self, x, y, radius=None):
        radius = world.MOWER_WIDTH / 2.0 if radius is None else radius
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

    # ----------------------------------------------------------- 障碍聚类
    def obstacle_circles(self, zone=None):
        visited = set()
        circles = []
        expanded = None
        if zone is not None:
            expanded = (
                zone["x0"] - 0.6,
                zone["y0"] - 0.6,
                zone["x1"] + 0.6,
                zone["y1"] + 0.6,
            )
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
                if expanded is not None:
                    if not (expanded[0] <= cx <= expanded[2] and expanded[1] <= cy <= expanded[3]):
                        continue
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


