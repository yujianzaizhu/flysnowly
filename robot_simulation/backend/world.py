"""多张不规则场地、多边形几何、视觉射线与世界状态。"""

import math

CELL = 0.1
# 场地面积扩大为原来的 2 倍（宽、高各约 ×1.414），机器人/视觉尺寸不变。
MAP_AREA_FACTOR = 2.0
MAP_SCALE = math.sqrt(MAP_AREA_FACTOR)
ROBOT_RADIUS = 0.18
TRACK_WIDTH = 0.26
WHEEL_BASE = 0.30
MOWER_WIDTH = 0.44

VISION_FOV = math.radians(100.0)
VISION_RANGE = 1.0
VISION_RAYS = 41

MAPS = {
    1: {
        "id": 1,
        "name": "L 形庭院",
        "width": 12.5,
        "height": 8.5,
        "polygon": [
            (0.0, 0.0), (8.5, 0.0), (8.5, 3.2),
            (12.5, 3.2), (12.5, 8.5), (0.0, 8.5),
        ],
        "obstacles": [
            {"kind": "large", "parts": [(1.82, 1.95, 0.40), (2.18, 2.05, 0.32), (2.00, 2.30, 0.25)]},
            {"kind": "small", "parts": [(6.70, 1.60, 0.30), (6.95, 1.68, 0.22)]},
            {"kind": "large", "parts": [(7.80, 6.80, 0.40), (8.15, 6.90, 0.30), (8.00, 7.15, 0.25)]},
            {"kind": "small", "parts": [(6.40, 2.30, 0.25), (6.65, 2.35, 0.20)]},
            {"kind": "small", "parts": [(2.00, 6.70, 0.35), (2.16, 6.55, 0.20)]},
        ],
        "dock": {"x": 4.0, "y": 6.2, "r": 0.45},
        "start": {"x": 4.9, "y": 6.2, "theta": 0.0},
    },
    2: {
        "id": 2,
        "name": "阶梯花园",
        "width": 13.5,
        "height": 9.0,
        "polygon": [
            (0.0, 0.0), (9.0, 0.0), (9.0, 2.2),
            (11.5, 2.2), (11.5, 4.6), (13.5, 4.6),
            (13.5, 9.0), (0.0, 9.0),
        ],
        "obstacles": [
            {"kind": "large", "parts": [(6.80, 4.80, 0.30), (7.15, 4.75, 0.30), (7.00, 5.10, 0.25)]},
            {"kind": "large", "parts": [(2.00, 2.10, 0.40), (2.35, 2.20, 0.30), (2.20, 2.45, 0.22)]},
            {"kind": "small", "parts": [(5.90, 7.40, 0.32), (6.15, 7.45, 0.22)]},
            {"kind": "small", "parts": [(10.90, 7.00, 0.28), (11.15, 7.05, 0.20)]},
            {"kind": "small", "parts": [(10.40, 3.60, 0.25), (10.65, 3.55, 0.18)]},
            {"kind": "small", "parts": [(2.00, 6.80, 0.35), (2.15, 6.65, 0.18)]},
        ],
        "dock": {"x": 4.4, "y": 4.8, "r": 0.45},
        "start": {"x": 5.3, "y": 4.8, "theta": 0.0},
    },
    3: {
        "id": 3,
        "name": "U 形回廊",
        "width": 13.0,
        "height": 8.5,
        "polygon": [
            (0.0, 0.0), (13.0, 0.0), (13.0, 8.5),
            (9.5, 8.5), (9.5, 3.0), (3.5, 3.0),
            (3.5, 8.5), (0.0, 8.5),
        ],
        "obstacles": [
            {"kind": "large", "parts": [(7.85, 1.50, 0.30), (8.15, 1.55, 0.30), (8.00, 1.80, 0.24)]},
            {"kind": "large", "parts": [(2.05, 1.55, 0.32), (2.35, 1.65, 0.28), (2.20, 1.90, 0.22)]},
            {"kind": "small", "parts": [(11.10, 5.50, 0.33), (11.35, 5.55, 0.22)]},
            {"kind": "small", "parts": [(1.50, 6.80, 0.32), (1.70, 6.65, 0.20)]},
            {"kind": "small", "parts": [(11.10, 2.00, 0.30), (11.35, 2.05, 0.20)]},
        ],
        "dock": {"x": 5.6, "y": 1.5, "r": 0.45},
        "start": {"x": 6.5, "y": 1.5, "theta": 0.0},
    },
    4: {
        "id": 4,
        "name": "斜角草坪",
        "width": 13.5,
        "height": 8.5,
        "polygon": [
            (0.0, 0.0), (10.5, 0.0), (13.5, 3.0),
            (13.5, 8.5), (0.0, 8.5),
        ],
        "obstacles": [
            {"kind": "large", "parts": [(7.80, 3.00, 0.32), (8.15, 3.10, 0.30), (8.00, 3.35, 0.24)]},
            {"kind": "large", "parts": [(1.85, 1.95, 0.36), (2.18, 2.05, 0.30), (2.00, 2.30, 0.22)]},
            {"kind": "small", "parts": [(2.90, 7.00, 0.30), (3.15, 7.05, 0.20)]},
            {"kind": "small", "parts": [(9.35, 7.20, 0.35), (9.65, 7.25, 0.25)]},
            {"kind": "small", "parts": [(11.80, 6.40, 0.28), (12.05, 6.35, 0.20)]},
        ],
        "dock": {"x": 4.8, "y": 4.2, "r": 0.45},
        "start": {"x": 5.7, "y": 4.2, "theta": 0.0},
    },
    5: {
        "id": 5,
        "name": "双庭院",
        "width": 13.5,
        "height": 8.5,
        "polygon": [
            (0.0, 0.0), (5.5, 0.0), (5.5, 2.8),
            (8.5, 2.8), (8.5, 0.0), (13.5, 0.0),
            (13.5, 8.5), (8.5, 8.5), (8.5, 5.7),
            (5.5, 5.7), (5.5, 8.5), (0.0, 8.5),
        ],
        "obstacles": [
            {"kind": "large", "parts": [(9.30, 4.20, 0.32), (9.65, 4.30, 0.30), (9.50, 4.55, 0.22)]},
            {"kind": "large", "parts": [(1.85, 1.95, 0.36), (2.18, 2.05, 0.30), (2.00, 2.30, 0.22)]},
            {"kind": "small", "parts": [(1.90, 6.80, 0.33), (2.10, 6.65, 0.20)]},
            {"kind": "small", "parts": [(11.35, 2.20, 0.30), (11.60, 2.25, 0.20)]},
            {"kind": "small", "parts": [(11.35, 6.60, 0.30), (11.60, 6.55, 0.20)]},
            {"kind": "small", "parts": [(3.70, 3.40, 0.30), (3.95, 3.45, 0.20)]},
        ],
        "dock": {"x": 6.3, "y": 4.25, "r": 0.45},
        "start": {"x": 7.2, "y": 4.25, "theta": 0.0},
    },
}

MAP_ID = 1
MAP_NAME = ""
WIDTH = 1.0
HEIGHT = 1.0
COLS = 1
ROWS = 1
ACTIVE_SCALE = MAP_SCALE
POLYGON = []
POLYGON_SEGMENTS = []
OBSTACLE_SEGMENTS = []
OBSTACLE_SHAPES = []
OBSTACLE_OUTLINES = []
OBSTACLES = []
OBSTACLE_POLYGONS = []
DOCK_RADIUS = 0.70
DOCK = {"x": 0.0, "y": 0.0, "r": DOCK_RADIUS}
START = {"x": 0.0, "y": 0.0, "theta": 0.0}
IMPORTED_MAPS = {}
NEXT_IMPORTED_ID = 101


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def select_map(map_id):
    """切换当前场地；调用方负责重建 GridMap / TaskManager。"""
    global MAP_ID, MAP_NAME, WIDTH, HEIGHT, COLS, ROWS, ACTIVE_SCALE
    global POLYGON, POLYGON_SEGMENTS, OBSTACLE_SEGMENTS
    global OBSTACLE_SHAPES, OBSTACLE_OUTLINES, OBSTACLES, OBSTACLE_POLYGONS, DOCK, START
    definition = MAPS[int(map_id)]
    MAP_ID = definition["id"]
    MAP_NAME = definition["name"]
    native = bool(definition.get("native"))
    scale = 1.0 if native else MAP_SCALE
    ACTIVE_SCALE = scale
    WIDTH = definition["width"] if native else definition["width"] * scale
    HEIGHT = definition["height"] if native else definition["height"] * scale
    COLS = int(round(WIDTH / CELL))
    ROWS = int(round(HEIGHT / CELL))
    POLYGON = [(x * scale, y * scale) for x, y in definition["polygon"]]
    POLYGON_SEGMENTS = _segments(POLYGON)
    raw_shapes = definition.get("obstacles", [])
    OBSTACLE_SHAPES = []
    for shape in raw_shapes:
        item = {"kind": shape["kind"]}
        item["parts"] = [
            (x * scale, y * scale, radius) for x, y, radius in shape.get("parts", [])
        ]
        if "outline" in shape:
            item["outline"] = [(x * scale, y * scale) for x, y in shape["outline"]]
        OBSTACLE_SHAPES.append(item)
    OBSTACLE_OUTLINES = [obstacle_outline(shape) for shape in OBSTACLE_SHAPES]
    OBSTACLES = [part for shape in OBSTACLE_SHAPES for part in shape["parts"]]
    OBSTACLE_POLYGONS = [
        shape["outline"]
        for shape in OBSTACLE_SHAPES
        if len(shape.get("outline", [])) >= 3
    ]
    OBSTACLE_SEGMENTS = [
        segment
        for polygon in OBSTACLE_POLYGONS
        for segment in _segments(polygon)
    ]
    DOCK = {
        "x": definition["dock"]["x"] * scale,
        "y": definition["dock"]["y"] * scale,
        "r": DOCK_RADIUS,
    }
    START = {
        "x": definition["start"]["x"] * scale,
        "y": definition["start"]["y"] * scale,
        "theta": definition["start"]["theta"],
    }


def register_map(definition):
    """注册导入地图并返回其编号；重复导入同一来源时覆盖原编号。"""
    global NEXT_IMPORTED_ID
    source = definition.get("source_path")
    if source:
        for map_id, known in IMPORTED_MAPS.items():
            if known.get("source_path") == source:
                definition["id"] = map_id
                MAPS[map_id] = definition
                IMPORTED_MAPS[map_id] = definition
                return map_id
    map_id = NEXT_IMPORTED_ID
    NEXT_IMPORTED_ID += 1
    definition["id"] = map_id
    MAPS[map_id] = definition
    IMPORTED_MAPS[map_id] = definition
    return map_id


def delete_map(map_id):
    map_id = int(map_id)
    if map_id not in IMPORTED_MAPS:
        raise ValueError("内置地图不可删除")
    MAPS.pop(map_id, None)
    IMPORTED_MAPS.pop(map_id, None)
    return map_id


def map_list():
    return [
        {
            "id": definition["id"],
            "name": definition["name"],
            "width": round(
                definition["width"]
                if definition.get("native")
                else definition["width"] * MAP_SCALE,
                1,
            ),
            "height": round(
                definition["height"]
                if definition.get("native")
                else definition["height"] * MAP_SCALE,
                1,
            ),
            "imported": bool(definition.get("imported")),
        }
        for definition in sorted(MAPS.values(), key=lambda item: item["id"])
    ]


def point_in_polygon(x, y, polygon=None):
    polygon = POLYGON if polygon is None else polygon
    inside = False
    count = len(polygon)
    j = count - 1
    for i in range(count):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi
        ):
            inside = not inside
        j = i
    return inside


def _segment_distance(px, py, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    length_sq = vx * vx + vy * vy
    if length_sq <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = clamp(((px - ax) * vx + (py - ay) * vy) / length_sq, 0.0, 1.0)
    return math.hypot(px - (ax + vx * t), py - (ay + vy * t))


def _segments(polygon):
    count = len(polygon)
    return [
        (polygon[index][0], polygon[index][1], polygon[(index + 1) % count][0], polygon[(index + 1) % count][1])
        for index in range(count)
    ]


def boundary_distance(x, y, polygon=None):
    polygon = POLYGON if polygon is None else polygon
    best = 1e9
    count = len(polygon)
    for index in range(count):
        ax, ay = polygon[index]
        bx, by = polygon[(index + 1) % count]
        best = min(best, _segment_distance(x, y, ax, ay, bx, by))
    return best


def _ray_segment(px, py, dx, dy, ax, ay, bx, by):
    ex, ey = bx - ax, by - ay
    denom = dx * ey - dy * ex
    if abs(denom) < 1e-9:
        return None
    qx, qy = ax - px, ay - py
    t = (qx * ey - qy * ex) / denom
    u = (qx * dy - qy * dx) / denom
    if t > 1e-6 and -1e-9 <= u <= 1.0 + 1e-9:
        return t
    return None


def _ray_circle(px, py, dx, dy, cx, cy, radius):
    fx, fy = px - cx, py - cy
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - radius * radius
    disc = b * b - c
    if disc < 0.0:
        return None
    root = math.sqrt(disc)
    near = -b - root
    far = -b + root
    if near > 1e-6:
        return near
    if far > 1e-6:
        return far
    return None


def raycast(x, y, angle, max_range=None):
    max_range = VISION_RANGE if max_range is None else max_range
    dx, dy = math.cos(angle), math.sin(angle)
    best = max_range
    for ax, ay, bx, by in POLYGON_SEGMENTS:
        hit = _ray_segment(x, y, dx, dy, ax, ay, bx, by)
        if hit is not None:
            best = min(best, hit)
    for ox, oy, radius in OBSTACLES:
        hit = _ray_circle(x, y, dx, dy, ox, oy, radius)
        if hit is not None:
            best = min(best, hit)
    for ax, ay, bx, by in OBSTACLE_SEGMENTS:
        hit = _ray_segment(x, y, dx, dy, ax, ay, bx, by)
        if hit is not None:
            best = min(best, hit)
    return clamp(best, 0.0, max_range)


def vision_scan(x, y, theta, fov=None, max_range=None, rays=None):
    fov = VISION_FOV if fov is None else fov
    max_range = VISION_RANGE if max_range is None else max_range
    rays = VISION_RAYS if rays is None else rays
    readings = []
    for index in range(rays):
        relative = -fov / 2.0 + fov * index / float(rays - 1)
        distance = raycast(x, y, theta + relative, max_range)
        readings.append((relative, distance))
    return readings


def collision(x, y, radius=None):
    radius = ROBOT_RADIUS if radius is None else radius
    if not point_in_polygon(x, y):
        return True
    if boundary_distance(x, y) < radius:
        return True
    for ox, oy, oradius in OBSTACLES:
        if (x - ox) ** 2 + (y - oy) ** 2 < (oradius + radius) ** 2:
            return True
    for polygon in OBSTACLE_POLYGONS:
        if point_in_polygon(x, y, polygon):
            return True
        if boundary_distance(x, y, polygon) < radius:
            return True
    return False


def boundary_cells():
    """多边形边界附近的栅格集合，用于建图进度统计。"""
    cells = set()
    count = len(POLYGON)
    for index in range(count):
        ax, ay = POLYGON[index]
        bx, by = POLYGON[(index + 1) % count]
        length = math.hypot(bx - ax, by - ay)
        steps = max(1, int(length / (CELL * 0.5)))
        for step in range(steps + 1):
            t = step / float(steps)
            x = ax + (bx - ax) * t
            y = ay + (by - ay) * t
            for dc in (-1, 0, 1):
                for dr in (-1, 0, 1):
                    col = int(clamp((x / CELL) + dc, 0, COLS - 1))
                    row = int(clamp((y / CELL) + dr, 0, ROWS - 1))
                    cells.add((col, row))
    return cells


def obstacle_outline(shape, steps=48):
    """场地真实障碍物轮廓（仅用于界面展示，不参与机器人感知）。"""
    if len(shape.get("outline", [])) >= 3:
        return [tuple(point) for point in shape["outline"]]
    parts = shape.get("parts", [])
    if not parts:
        return []
    total = sum(radius * radius for _, _, radius in parts)
    cx = sum(x * radius * radius for x, _, radius in parts) / total
    cy = sum(y * radius * radius for _, y, radius in parts) / total
    outline = []
    for index in range(steps):
        angle = 2.0 * math.pi * index / steps
        dx, dy = math.cos(angle), math.sin(angle)
        best = 0.0
        for ox, oy, radius in parts:
            vx, vy = ox - cx, oy - cy
            b = vx * dx + vy * dy
            c = vx * vx + vy * vy - radius * radius
            discriminant = b * b - c
            if discriminant < 0.0:
                continue
            hit = b + math.sqrt(discriminant)
            if hit > best:
                best = hit
        if best <= 0.0:
            ox, oy, radius = min(
                parts, key=lambda part: math.hypot(part[0] - cx, part[1] - cy)
            )
            distance = math.hypot(ox - cx, oy - cy)
            best = distance + radius
        outline.append((round(cx + dx * best, 3), round(cy + dy * best, 3)))
    return outline


def world_dict():
    shapes = [
        dict(shape, outline=OBSTACLE_OUTLINES[index])
        for index, shape in enumerate(OBSTACLE_SHAPES)
    ]
    definition = MAPS.get(MAP_ID, {})
    return {
        "id": MAP_ID,
        "name": MAP_NAME,
        "width": WIDTH,
        "height": HEIGHT,
        "cell": CELL,
        "map_scale": ACTIVE_SCALE,
        "map_area_factor": MAP_AREA_FACTOR,
        "imported": bool(definition.get("imported")),
        "source_path": definition.get("source_path"),
        "import_meta": definition.get("import_meta", {}),
        "cols": COLS,
        "rows": ROWS,
        "polygon": POLYGON,
        "obstacles": OBSTACLES,
        "obstacle_shapes": shapes,
        "dock": DOCK,
        "start": START,
        "maps": map_list(),
        "robot": {
            "radius": ROBOT_RADIUS,
            "track": TRACK_WIDTH,
            "wheelbase": WHEEL_BASE,
            "mower_width": MOWER_WIDTH,
            "drive": "rear_diff",
        },
        "vision": {
            "fov_deg": math.degrees(VISION_FOV),
            "range": VISION_RANGE,
            "rays": VISION_RAYS,
        },
    }


select_map(1)
