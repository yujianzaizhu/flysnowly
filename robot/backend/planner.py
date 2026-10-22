"""区域边缘路径与弓字覆盖路径规划。"""

import math

import world

LANE_SPACING = 0.30
EDGE_INSET = 0.22
RING_MARGIN = 0.38


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


def edge_route_grid(grid, zone, start_xy, clearance=None):
    clearance = max(0.14, EDGE_INSET - 0.02) if clearance is None else clearance
    """沿区域自由区边界（内缩 clearance）生成闭合巡边路径。"""
    cells = []
    for row in range(grid.rows):
        for col in range(grid.cols):
            index = grid.index(col, row)
            if not grid.free_mask[index]:
                continue
            x, y = grid.center_of(col, row)
            found = grid.zone_at(x, y)
            if found is None or found["id"] != zone["id"]:
                continue
            cells.append((col, row, x, y))
    if len(cells) < 20:
        return edge_route(zone, start_xy)

    safe = set()
    for col, row, x, y in cells:
        ok = True
        for angle_index in range(12):
            angle = 2.0 * math.pi * angle_index / 12.0
            nx = x + clearance * math.cos(angle)
            ny = y + clearance * math.sin(angle)
            nc, nr = grid.cell_of(nx, ny)
            if not grid.in_bounds(nc, nr):
                ok = False
                break
            nindex = grid.index(nc, nr)
            if not grid.free_mask[nindex]:
                ok = False
                break
            found = grid.zone_at(nx, ny)
            if found is None or found["id"] != zone["id"]:
                ok = False
                break
        if ok:
            safe.add((col, row))
    if len(safe) < 8:
        return edge_route(zone, start_xy)

    boundary = []
    for col, row in safe:
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            if (col + dc, row + dr) not in safe:
                boundary.append((col, row))
                break
    unvisited = set(boundary)
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
    loop_cells = set(max(components, key=len))
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
            chosen = min(candidates, key=lambda cell: abs(cell[0] - current[0]) + abs(cell[1] - current[1]))
        else:
            previous_heading = math.atan2(current[1] - previous[1], current[0] - previous[0])

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
        return edge_route(zone, start_xy)
    points.append(points[0])
    return points


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
