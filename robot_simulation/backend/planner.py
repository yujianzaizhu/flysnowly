"""区域边缘路径与弓字覆盖路径规划。"""

import math

import world

LANE_SPACING = 0.30
EDGE_INSET = 0.02
RING_MARGIN = 0.34
SQRT2 = math.sqrt(2.0)


def _polygon_area(points):
    area = 0.0
    for index, point in enumerate(points):
        nxt = points[(index + 1) % len(points)]
        area += point[0] * nxt[1] - nxt[0] * point[1]
    return area * 0.5


def _path_length(points):
    return sum(
        math.hypot(points[index][0] - points[index - 1][0], points[index][1] - points[index - 1][1])
        for index in range(1, len(points))
    )


def _densify(points, max_step=0.16):
    if len(points) < 2:
        return points
    result = [points[0]]
    for target in points[1:]:
        ax, ay = result[-1]
        distance = math.hypot(target[0] - ax, target[1] - ay)
        count = max(1, int(math.ceil(distance / max_step)))
        for index in range(1, count + 1):
            ratio = index / float(count)
            result.append(
                (ax + (target[0] - ax) * ratio, ay + (target[1] - ay) * ratio)
            )
    return result


def edge_corners(zone, inset=None):
    inset = EDGE_INSET if inset is None else inset
    """区域边缘（内缩）矩形的四个角，按逆时针排列。"""
    return [
        (zone["x0"] + inset, zone["y0"] + inset),
        (zone["x1"] - inset, zone["y0"] + inset),
        (zone["x1"] - inset, zone["y1"] - inset),
        (zone["x0"] + inset, zone["y1"] - inset),
    ]


def edge_route(zone, start_xy, inset=None):
    inset = EDGE_INSET if inset is None else inset
    """从离机器人最近的角开始绕区域边缘一周。"""
    corners = edge_corners(zone, inset)
    index = min(
        range(len(corners)),
        key=lambda i: math.hypot(corners[i][0] - start_xy[0], corners[i][1] - start_xy[1]),
    )
    route = [corners[(index + offset) % len(corners)] for offset in range(len(corners))]
    route.append(route[0])
    return route


def _zone_free_cells(grid, zone):
    """区域矩形框 ∩ 建图自由区：区域内属于该区域、且建图时判为自由的栅格。"""
    zone_id = zone["id"]
    cols = grid.cols
    zone_map = grid.zone_id_map
    free = grid.free_mask
    cells = set()
    for index, assigned in enumerate(zone_map):
        if assigned == zone_id and free[index]:
            cells.add((index % cols, index // cols))
    return cells


def _chamfer_distance(cells, cols, c0, c1, r0, r1, grid_rows=None):
    """bbox 内到最近"非掩膜格"的距离（格），4/8 邻域权重 1 与 √2 的两遍扫描。

    等价于"以掩膜为前景做欧氏腐蚀"的距离场，比逐格多方向采样快得多。
    """
    inf = float("inf")
    grid_rows = cols if grid_rows is None else grid_rows
    width = c1 - c0 + 1
    height = r1 - r0 + 1
    dist = [0.0] * (width * height)
    for col, row in cells:
        if c0 <= col <= c1 and r0 <= row <= r1:
            dist[(row - r0) * width + (col - c0)] = inf
    # 栅格图之外按"边界外"处理：贴着画面边缘的自由格距离记 1 格，
    # 否则整幅地图都是自由区时（掩膜铺满整张图）算不出边界，腐蚀会失效。
    for col, row in cells:
        if c0 <= col <= c1 and r0 <= row <= r1:
            if col in (0, cols - 1) or row in (0, grid_rows - 1):
                dist[(row - r0) * width + (col - c0)] = 1.0
    for row in range(height):
        base = row * width
        for col in range(width):
            index = base + col
            if dist[index] == 0.0:
                continue
            best = dist[index]
            if col > 0:
                left = dist[index - 1] + 1.0
                if left < best:
                    best = left
            if row > 0:
                up = index - width
                value = dist[up] + 1.0
                if value < best:
                    best = value
                if col > 0:
                    value = dist[up - 1] + SQRT2
                    if value < best:
                        best = value
                if col + 1 < width:
                    value = dist[up + 1] + SQRT2
                    if value < best:
                        best = value
            dist[index] = best
    for row in range(height - 1, -1, -1):
        base = row * width
        for col in range(width - 1, -1, -1):
            index = base + col
            if dist[index] == 0.0:
                continue
            best = dist[index]
            if col + 1 < width:
                right = dist[index + 1] + 1.0
                if right < best:
                    best = right
            if row + 1 < height:
                down = index + width
                value = dist[down] + 1.0
                if value < best:
                    best = value
                if col > 0:
                    value = dist[down - 1] + SQRT2
                    if value < best:
                        best = value
                if col + 1 < width:
                    value = dist[down + 1] + SQRT2
                    if value < best:
                        best = value
            dist[index] = best
    return dist, width, c0, r0


def _erode(cells, dist, width, c0, r0, threshold):
    """按距离阈值腐蚀：距离 >= threshold 格的点保留。"""
    result = set()
    for cell in cells:
        col, row = cell
        value = dist[(row - r0) * width + (col - c0)]
        if value >= threshold:
            result.add(cell)
    return result


def _mask_border(cells):
    """掩膜边界格：4 邻域有任一格不在掩膜内。"""
    border = []
    for col, row in cells:
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            if (col + dc, row + dr) not in cells:
                border.append((col, row))
                break
    return border


def _components_8(cells):
    """8 邻域连通分量。"""
    unvisited = set(cells)
    components = []
    while unvisited:
        seed = unvisited.pop()
        stack = [seed]
        component = [seed]
        while stack:
            col, row = stack.pop()
            for dc in (-1, 0, 1):
                for dr in (-1, 0, 1):
                    if dc == 0 and dr == 0:
                        continue
                    neighbour = (col + dc, row + dr)
                    if neighbour in unvisited:
                        unvisited.remove(neighbour)
                        stack.append(neighbour)
                        component.append(neighbour)
        components.append(component)
    return components


def _trace_loop(grid, component, start_xy):
    """在 1 格厚的边界环上，从离机器人最近的格起按最小转向走一圈。"""
    loop_cells = set(component)
    start_cell = min(
        loop_cells,
        key=lambda cell: math.hypot(
            grid.center_of(*cell)[0] - start_xy[0],
            grid.center_of(*cell)[1] - start_xy[1],
        ),
    )
    walk = [start_cell]
    visited = {start_cell}
    current = start_cell
    previous = None
    while True:
        candidates = []
        for dc in (-1, 0, 1):
            for dr in (-1, 0, 1):
                if dc == 0 and dr == 0:
                    continue
                neighbour = (current[0] + dc, current[1] + dr)
                if neighbour in loop_cells and neighbour not in visited:
                    candidates.append(neighbour)
        if not candidates:
            break
        if previous is None:
            chosen = min(
                candidates,
                key=lambda cell: abs(cell[0] - current[0]) + abs(cell[1] - current[1]),
            )
        else:
            previous_heading = math.atan2(
                current[1] - previous[1], current[0] - previous[0]
            )

            def score(cell):
                heading = math.atan2(cell[1] - current[1], cell[0] - current[0])
                turn = abs(world.wrap_angle(heading - previous_heading))
                diagonal = abs(cell[0] - current[0]) + abs(cell[1] - current[1]) > 1
                return turn + (0.45 if diagonal else 0.0)

            chosen = min(candidates, key=score)
        walk.append(chosen)
        visited.add(chosen)
        previous, current = current, chosen
    points = [grid.center_of(col, row) for col, row in walk]
    if len(points) < 2:
        return []
    points.append(points[0])
    return points


def edge_route_mask(grid, zone, start_xy, inset=None):
    """沿边轨迹 = 腐蚀后的「区域矩形框 ∩ 建图自由区」的外轮廓。

    - 交集：只保留该区域内、建图时判为自由的栅格，天然满足"不出区域、不进占据/场外"；
    - 腐蚀：按 inset 做欧氏腐蚀（默认 EDGE_INSET），把路径压到离边界一个车体半径量级的位置，
      让刀盘能盖住最后一条草、又不会贴墙；
    - 轮廓：取腐蚀后掩膜的外轮廓并按最小转向闭合，最后按 0.16 m 加密。

    规则矩形区域腐蚀后仍是一个内缩矩形（拐角变圆，对后驱小车更平滑），
    不规则区域则自动贴合真实边界，因此不需要再用"填充率"去判形状。
    """
    inset = EDGE_INSET if inset is None else inset
    cells = _zone_free_cells(grid, zone)
    if len(cells) < 20:
        return edge_route(zone, start_xy)
    cell = grid.cell
    c0, r0 = grid.cell_of(zone["x0"], zone["y0"])
    c1, r1 = grid.cell_of(zone["x1"], zone["y1"])
    c0 = max(0, c0 - 1)
    r0 = max(0, r0 - 1)
    c1 = min(grid.cols - 1, c1 + 1)
    r1 = min(grid.rows - 1, r1 + 1)
    dist, width, _, _ = _chamfer_distance(cells, grid.cols, c0, c1, r0, r1, grid.rows)
    # 距离场量的是"格心"，真实边界在格心外半格，这里补上半个格
    threshold = max(0.5, (float(inset) + 0.5 * cell) / max(1e-6, cell))
    eroded = set()
    # 窄通道里腐蚀过头会让掩膜消失，逐级放缓而不是直接放弃
    for _ in range(5):
        eroded = _erode(cells, dist, width, c0, r0, threshold)
        if len(eroded) >= 20:
            break
        threshold = max(1.0, threshold - 0.5)
    if len(eroded) < 20:
        return edge_route(zone, start_xy)

    loops = []
    for component in _components_8(_mask_border(eroded)):
        if len(component) < 8:
            continue
        points = _trace_loop(grid, component, start_xy)
        if len(points) < 3:
            continue
        if _polygon_area(points) < 0.0:
            points = list(reversed(points))
        loops.append(points)
    if not loops:
        return edge_route(zone, start_xy)
    loops.sort(key=lambda points: -_path_length(points))
    # 外边界先由沿边阶段完成；区域内部障碍交给后续环形补割
    loop = loops[0]
    entry = min(
        range(len(loop) - 1),
        key=lambda index: math.hypot(loop[index][0] - start_xy[0], loop[index][1] - start_xy[1]),
    )
    route = loop[entry:-1] + loop[:entry] + [loop[entry]]
    return _densify(route)


def edge_route_grid(grid, zone, start_xy, clearance=None):
    """兼容旧名字：现在统一走「交集 + 腐蚀」的掩膜方案。"""
    return edge_route_mask(grid, zone, start_xy, inset=clearance)


def lane_segments(grid, zone, spacing=None, axis="h"):
    spacing = LANE_SPACING if spacing is None else spacing
    """对区域内尚未割草的连通部分生成弓字线段（蛇形顺序）。

    axis="h" 为横向弓字，axis="v" 为纵向补割。
    """
    if axis == "v":
        x_start = zone["x0"] + 0.22
        x_end = zone["x1"] - 0.22
        y_lo = zone["y0"] + 0.22
        y_hi = zone["y1"] - 0.22
    else:
        x_start = zone["x0"] + 0.22
        x_end = zone["x1"] - 0.22
        y_lo = zone["y0"] + 0.22
        y_hi = zone["y1"] - 0.22

    lanes = []
    lane_index = 0
    fixed = x_start if axis == "v" else y_lo
    fixed_end = x_end if axis == "v" else y_hi
    while fixed <= fixed_end + 1e-6:
        if axis == "v":
            runs = _lane_runs(grid, y_lo, y_hi, fixed, vertical=True, zone_id=zone["id"])
        else:
            runs = _lane_runs(grid, x_start, x_end, fixed, zone_id=zone["id"])
        runs = [run for run in runs if run[1] - run[0] >= 0.25]
        if runs:
            lanes.append((lane_index, fixed, runs))
        lane_index += 1
        fixed += spacing

    segments = []
    for lane_index, fixed, runs in lanes:
        if lane_index % 2 == 0:
            ordered = runs
        else:
            ordered = list(reversed(runs))
        for start, end in ordered:
            if axis == "v":
                a = (fixed, start)
                b = (fixed, end)
            else:
                a = (start, fixed)
                b = (end, fixed)
            if lane_index % 2 == 1:
                a, b = b, a
            segments.append((a, b))
    return segments


def order_segments(segments, start_xy, heading=0.0):
    """按“就近 + 保持航向”的顺序重排弓字段，减少跨区域折返。

    障碍物把一条弓字切成两段时，机器人会先完成当前一侧的相邻段，
    再前往另一侧，避免来回横穿障碍物。
    """
    remaining = [tuple(segment) for segment in segments]
    ordered = []
    x, y = start_xy
    current_heading = heading
    while remaining:
        best = None
        for index, segment in enumerate(remaining):
            a, b = segment
            for start, end in ((a, b), (b, a)):
                distance = math.hypot(start[0] - x, start[1] - y)
                bearing = math.atan2(start[1] - y, start[0] - x)
                turn = abs(world.wrap_angle(bearing - current_heading))
                score = distance + 0.45 * turn
                if distance < 0.12:
                    score -= 0.25
                if best is None or score < best[0]:
                    best = (score, index, start, end)
        _, index, start, end = best
        ordered.append((start, end))
        remaining.pop(index)
        x, y = end
        current_heading = math.atan2(end[1] - start[1], end[0] - start[0])
    return ordered


def unknown_lane_segments(grid, spacing=None, axis="h", min_run=0.60):
    """还没建过图的内部区域 → 扫描弓字线段（建图兜底用）。

    只有外墙都找不到、边界探明率上不去的时候才会用到：正常场地绕完外墙就
    100% 了，不该为了"看内部"多跑一趟。
    """
    from grid_map import UNKNOWN

    spacing = LANE_SPACING + 0.15 if spacing is None else spacing
    inside = world.polygon_mask(grid.cols, grid.rows, grid.cell)
    cols, rows = grid.cols, grid.rows
    data = grid.data
    blocked = grid.blocked

    def unknown(col, row):
        index = row * cols + col
        return inside[index] and data[index] == UNKNOWN and not blocked[index]

    segments = []
    cell = grid.cell
    if axis == "v":
        index_max, run_max = cols, rows
    else:
        index_max, run_max = rows, cols
    fixed = cell
    while fixed <= index_max * cell - cell * 0.5:
        index_fixed = int(fixed / cell)
        run = None
        gap = 0
        for step in range(run_max):
            col, row = (index_fixed, step) if axis == "v" else (step, index_fixed)
            if unknown(col, row):
                if run is None:
                    run = [step, step]
                else:
                    run[1] = step
                gap = 0
            elif run is not None:
                gap += 1
                if gap > 2:
                    if (run[1] - run[0] + 1) * cell >= min_run:
                        segments.append(_segment_for(axis, index_fixed, run, cell))
                    run = None
                    gap = 0
        if run is not None and (run[1] - run[0] + 1) * cell >= min_run:
            segments.append(_segment_for(axis, index_fixed, run, cell))
        fixed += spacing
    start = (cols * cell * 0.5, rows * cell * 0.5)
    return order_segments(segments, start, 0.0)


def _segment_for(axis, index_fixed, run, cell):
    a = (run[0] + 0.5) * cell
    b = (run[1] + 0.5) * cell
    if axis == "v":
        return ((index_fixed + 0.5) * cell, a), ((index_fixed + 0.5) * cell, b)
    return (a, (index_fixed + 0.5) * cell), (b, (index_fixed + 0.5) * cell)


def _lane_runs(grid, start, end, fixed, vertical=False, zone_id=None):
    runs = []
    current = None
    gap = 0
    steps = int(round((end - start) / world.CELL)) + 1
    for index in range(steps):
        value = min(end, start + index * world.CELL)
        x, y = (fixed, value) if vertical else (value, fixed)
        col, row = grid.cell_of(x, y)
        usable = (
            not grid.cell_blocked_mow(col, row)
            and grid.cell_mowable(col, row)
            and not grid.cell_mowed(col, row)
            and (zone_id is None or grid.cell_in_zone(col, row, zone_id))
        )
        if usable:
            if current is None:
                current = [value, value]
            else:
                current[1] = value
            gap = 0
        elif current is not None:
            gap += 1
            if gap > 2:
                runs.append((current[0], current[1]))
                current = None
                gap = 0
    if current is not None:
        runs.append((current[0], current[1]))
    return runs


def zone_free_ratio(grid, zone):
    return grid.zone_coverage(zone["id"])


RING_STEPS = 36


def _observed_radius(grid, cx, cy, angle, fallback, clearance):
    """沿视线方向找已探明障碍物的最外边界，返回贴边绕行半径。"""
    if grid is None:
        return fallback
    dx = math.cos(angle)
    dy = math.sin(angle)
    step = max(0.02, grid.cell * 0.45)
    limit = fallback + grid.cell
    distance = step
    last_hit = None
    while distance <= limit:
        x = cx + dx * distance
        y = cy + dy * distance
        if grid.is_occupied_world(x, y):
            last_hit = distance
        distance += step
    if last_hit is None:
        return fallback
    return max(0.30 + clearance, min(fallback, last_hit + clearance))


def ring_route(circle, start_xy, margin=None, steps=RING_STEPS, grid=None, zone=None):
    margin = RING_MARGIN if margin is None else margin
    """沿已探明障碍物外轮廓绕行；被区域边界遮挡时只返回连续弧段。"""
    fallback = max(circle["r"], 0.30) + margin
    cx, cy = circle["x"], circle["y"]
    start_angle = math.atan2(start_xy[1] - cy, start_xy[0] - cx)
    raw = []
    for index in range(1, steps + 1):
        angle = start_angle + 2.0 * math.pi * index / steps
        radius = _observed_radius(grid, cx, cy, angle, fallback, margin)
        raw.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle)))
    if grid is None:
        return raw
    circles = grid.obstacle_circles(None)

    def clear(point):
        x, y = point
        if not world.point_in_polygon(x, y) or world.boundary_distance(x, y) < 0.24:
            return False
        if zone is not None:
            if not (
                zone["x0"] + 0.22 <= x <= zone["x1"] - 0.22
                and zone["y0"] + 0.22 <= y <= zone["y1"] - 0.22
            ):
                return False
        for other in circles:
            if math.hypot(other["x"] - cx, other["y"] - cy) < 0.25:
                continue
            if math.hypot(x - other["x"], y - other["y"]) < other["r"] + 0.25:
                return False
        return True

    open_flags = [clear(point) for point in raw]
    if not any(open_flags):
        return []
    start = 0
    while not open_flags[start]:
        start += 1
    arc = []
    index = start
    for _ in range(steps):
        if not open_flags[index]:
            break
        arc.append(raw[index])
        index = (index + 1) % steps
        if index == start:
            break
    return arc if len(arc) >= 4 else []
