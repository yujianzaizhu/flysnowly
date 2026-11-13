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
# 是否在地图上画充电桩：日志回放时**日志里没有明确的桩位**就不画
# （内部 DOCK 仍然保留一个锚点，任务照常能"回充"）
DOCK_VISIBLE = True
# "地图纪元"：场地被清空成**空白地图**（无地图的建图日志重新起笔）时 +1。
# 前端靠它判断"前一张地图已经被清掉"，把轨迹/点云这些属于旧地图的东西一起清掉。
MAP_EPOCH = 0
START = {"x": 0.0, "y": 0.0, "theta": 0.0}
IMPORTED_MAPS = {}
NEXT_IMPORTED_ID = 101
# 地图编辑（参数-地图）：静态障碍 / 动态障碍 / 禁区，按地图 id 保存
EDIT_STATE = {}
EDIT_KINDS = ("static", "dynamic", "no_go")
NEXT_EDIT_ID = 1
BASE_SHAPES = []
DYNAMIC = []
NO_GO = []
# 通道（只通行、不割草）：来自地图定义，select_map 时按缩放系数换算
PASSAGES = []
# 地图定义里保存的区域多边形（米）：用来区分"通道与区域重叠的部分"和
# "纯通道部分"——重叠部分照常割草，只有纯通道不割
REGIONS = []
DYNAMIC_SPEED = 0.15
# 面积阈值：小于该面积的障碍物按“小型”处理
LARGE_OBSTACLE_AREA = 5.0
STATIC_SHAPES = []
STATIC_OUTLINES = []
STATIC_PARTS = []
STATIC_POLYGONS = []
STATIC_SEGMENTS = []
WORLD_VERSION = 0
_WORLD_CACHE = {}


SEG_BUCKET = 1.5
# 线段池：(ax, ay, ex, ey)，ex/ey 是方向分量，省掉射线求交时的两次减法。
# 分桶存的是池下标，同一条线只落在它真正穿过的桶里（实景地图有 300+ 顶点、
# 单条斜墙的包围盒能盖住几百个桶，按包围盒入桶会让每条射线重复测十几遍）。
_SEG_POOL = []
_SEG_BUCKETS = {}
_EDGE_POOL = []
_EDGE_BUCKETS = {}
_SEG_CELL_CACHE = {}
_SEG_CACHE_RANGE = 0.0
_EDGE_CELL_CACHE = {}


def _bucket_index(x, y):
    return int(math.floor(x / SEG_BUCKET)), int(math.floor(y / SEG_BUCKET))


def _bucket_cells(ax, ay, bx, by):
    """线段穿过的桶（按 0.4 桶长采样，斜墙也不会漏桶）。"""
    length = math.hypot(bx - ax, by - ay)
    steps = max(1, int(length / (SEG_BUCKET * 0.4)) + 1)
    cells = []
    last = None
    for step in range(steps + 1):
        t = step / float(steps)
        cell = _bucket_index(ax + (bx - ax) * t, ay + (by - ay) * t)
        if cell != last:
            cells.append(cell)
            last = cell
    return cells


def _index_segment(buckets, pool, segment):
    ax, ay, bx, by = segment
    index = len(pool)
    pool.append((ax, ay, bx - ax, by - ay))
    for cell in _bucket_cells(ax, ay, bx, by):
        buckets.setdefault(cell, []).append(index)
    return index


def _rebuild_segment_index():
    """把外墙/障碍物线段按 1.5 m 方格分桶，射线与墙面距离只查邻近桶。"""
    global _SEG_BUCKETS, _EDGE_BUCKETS, _SEG_POOL, _EDGE_POOL, _SEG_CACHE_RANGE
    pool = []
    buckets = {}
    edge_pool = []
    edge_buckets = {}
    for segment in POLYGON_SEGMENTS:
        index = _index_segment(buckets, pool, segment)
        _index_segment(edge_buckets, edge_pool, segment)
    for segment in OBSTACLE_SEGMENTS:
        _index_segment(buckets, pool, segment)
    _SEG_POOL = pool
    _SEG_BUCKETS = buckets
    _EDGE_POOL = edge_pool
    _EDGE_BUCKETS = edge_buckets
    _SEG_CELL_CACHE.clear()
    _EDGE_CELL_CACHE.clear()
    # 按格缓存只有在查询半径不超过视觉量程时才成立（缓存按量程建桶）
    _SEG_CACHE_RANGE = VISION_RANGE + 1e-9


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def bump_map_epoch():
    """场地被清空成空白地图时调用：让前端把属于上一张地图的绘制内容清掉。"""
    global MAP_EPOCH
    MAP_EPOCH += 1
    return MAP_EPOCH


def select_map(map_id):
    """切换当前场地；调用方负责重建 GridMap / TaskManager。"""
    global MAP_ID, MAP_NAME, WIDTH, HEIGHT, COLS, ROWS, ACTIVE_SCALE
    global POLYGON, POLYGON_SEGMENTS, OBSTACLE_SEGMENTS
    global OBSTACLE_SHAPES, OBSTACLE_OUTLINES, OBSTACLES, OBSTACLE_POLYGONS, DOCK, START
    global DOCK_VISIBLE
    global BASE_SHAPES, DYNAMIC, NO_GO, PASSAGES, REGIONS
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
    BASE_SHAPES = []
    for shape in raw_shapes:
        item = {"kind": shape["kind"]}
        item["parts"] = [
            (x * scale, y * scale, radius) for x, y, radius in shape.get("parts", [])
        ]
        if "outline" in shape:
            item["outline"] = [(x * scale, y * scale) for x, y in shape.get("outline")]
        BASE_SHAPES.append(item)
    bucket = _edit_bucket(MAP_ID)
    DYNAMIC = []
    for item in bucket["dynamic"]:
        entry = dict(item)
        entry["x"] = entry["ax"]
        entry["y"] = entry["ay"]
        entry["t"] = 0.0
        DYNAMIC.append(entry)
    NO_GO = [dict(item) for item in bucket["no_go"]]
    PASSAGES = [
        {
            # 老地图/导入地图可能没写 id：按顺序补一个，界面标注就不会出现 #undefined
            "id": item.get("id") if item.get("id") is not None else index + 1,
            "points": [(x * scale, y * scale) for x, y in item.get("points") or []],
            "half": float(item.get("half", 0.4)) * scale,
            "from": item.get("from"),
            "to": item.get("to"),
        }
        for index, item in enumerate(definition.get("passages") or [])
        if len(item.get("points") or []) >= 2
    ]
    REGIONS = [
        {
            "id": item.get("id"),
            "points": [(x * scale, y * scale) for x, y in item.get("points") or []],
        }
        for item in definition.get("regions") or []
        if len(item.get("points") or []) >= 3
    ]
    if not REGIONS:
        # 老地图没有存区域列表：整块场地就是一个区域
        REGIONS = [{"id": 1, "points": list(POLYGON)}]
    _rebuild_static_geometry()
    _mark_world_dirty()
    DOCK = {
        "x": definition["dock"]["x"] * scale,
        "y": definition["dock"]["y"] * scale,
        "r": DOCK_RADIUS,
    }
    # 换地图一律先把桩显示出来；日志回放没有桩位时由 server.load_bag 再关掉
    DOCK_VISIBLE = True
    START = {
        "x": definition["start"]["x"] * scale,
        "y": definition["start"]["y"] * scale,
        "theta": definition["start"]["theta"],
    }


def _edit_bucket(map_id=None):
    map_id = MAP_ID if map_id is None else int(map_id)
    bucket = EDIT_STATE.get(map_id)
    if bucket is None:
        # 地图定义里自带的动态障碍与禁区（编辑器保存的地图会写进来）先搬进
        # 编辑桶，之后用户再怎么改/删都走同一套接口
        dynamic = []
        for item in MAPS.get(map_id, {}).get("dynamic") or []:
            entry = {
                "id": _next_edit_id(),
                "x": round(float(item.get("x", item.get("ax", 0.0))), 2),
                "y": round(float(item.get("y", item.get("ay", 0.0))), 2),
                "r": round(float(item.get("r", 0.30)), 2),
                "ax": round(float(item.get("ax", item.get("x", 0.0))), 2),
                "ay": round(float(item.get("ay", item.get("y", 0.0))), 2),
                "bx": round(float(item.get("bx", item.get("x", 0.0))), 2),
                "by": round(float(item.get("by", item.get("y", 0.0))), 2),
                "speed": clamp(float(item.get("speed", DYNAMIC_SPEED)), 0.05, 0.60),
            }
            dynamic.append(entry)
        no_go = []
        for item in MAPS.get(map_id, {}).get("no_go") or []:
            entry = dict(item)
            entry["id"] = _next_edit_id()
            no_go.append(entry)
        bucket = {"static": [], "dynamic": dynamic, "no_go": no_go}
        EDIT_STATE[map_id] = bucket
    return bucket


def _next_edit_id():
    global NEXT_EDIT_ID
    value = NEXT_EDIT_ID
    NEXT_EDIT_ID += 1
    return value


def _mark_world_dirty():
    global WORLD_VERSION
    WORLD_VERSION += 1


def _rebuild_static_geometry():
    """重建静态障碍几何（地图定义 + 静态编辑），并刷新动态障碍占据。"""
    global STATIC_SHAPES, STATIC_OUTLINES, STATIC_PARTS
    global STATIC_POLYGONS, STATIC_SEGMENTS
    bucket = _edit_bucket()
    shapes = [dict(shape) for shape in BASE_SHAPES]
    for item in bucket["static"]:
        area = math.pi * item["r"] * item["r"]
        shapes.append(
            {
                "kind": "large" if area >= LARGE_OBSTACLE_AREA else "small",
                "parts": [(item["x"], item["y"], item["r"])],
                "edit": "static",
                "id": item["id"],
            }
        )
    STATIC_SHAPES = shapes
    STATIC_OUTLINES = [obstacle_outline(shape) for shape in shapes]
    STATIC_PARTS = [part for shape in shapes for part in shape["parts"]]
    STATIC_POLYGONS = [
        shape["outline"] for shape in shapes if len(shape.get("outline", [])) >= 3
    ]
    STATIC_SEGMENTS = [
        segment for polygon in STATIC_POLYGONS for segment in _segments(polygon)
    ]
    _refresh_dynamic_geometry()
    _rebuild_segment_index()


def _refresh_dynamic_geometry():
    """只刷新动态障碍物的圆形占据；静态轮廓与线段不重算（性能关键）。"""
    global OBSTACLE_SHAPES, OBSTACLE_OUTLINES, OBSTACLES
    global OBSTACLE_POLYGONS, OBSTACLE_SEGMENTS
    OBSTACLE_SHAPES = STATIC_SHAPES
    OBSTACLE_OUTLINES = STATIC_OUTLINES
    OBSTACLE_POLYGONS = STATIC_POLYGONS
    OBSTACLE_SEGMENTS = STATIC_SEGMENTS
    if DYNAMIC:
        OBSTACLES = STATIC_PARTS + [
            (item["x"], item["y"], item["r"]) for item in DYNAMIC
        ]
    else:
        OBSTACLES = STATIC_PARTS


def _inside_field(x, y, margin=0.35):
    if not point_in_polygon(x, y):
        return False
    return boundary_distance(x, y) >= margin


def add_static_obstacle(x, y, radius=0.35):
    x, y = float(x), float(y)
    if not _inside_field(x, y):
        raise ValueError("请把障碍物放在场地内部")
    radius = clamp(float(radius), 0.12, 1.50)
    item = {
        "id": _next_edit_id(),
        "x": round(x, 2),
        "y": round(y, 2),
        "r": round(radius, 2),
    }
    _edit_bucket()["static"].append(item)
    _rebuild_static_geometry()
    _mark_world_dirty()
    return item


def add_dynamic_obstacle(x, y, radius=0.30, span=1.6, heading=0.0, speed=None):
    x, y = float(x), float(y)
    if not _inside_field(x, y):
        raise ValueError("请把障碍物放在场地内部")
    radius = clamp(float(radius), 0.12, 1.00)
    span = clamp(float(span), 0.4, 6.0)
    heading = float(heading)
    half = span / 2.0
    dx, dy = math.cos(heading) * half, math.sin(heading) * half
    ax, ay = _clamp_inside(x - dx, y - dy)
    bx, by = _clamp_inside(x + dx, y + dy)
    item = {
        "id": _next_edit_id(),
        "x": round(x, 2),
        "y": round(y, 2),
        "r": round(radius, 2),
        "ax": round(ax, 2),
        "ay": round(ay, 2),
        "bx": round(bx, 2),
        "by": round(by, 2),
        "speed": clamp(
            DYNAMIC_SPEED if speed is None else float(speed), 0.05, 0.60
        ),
    }
    _edit_bucket()["dynamic"].append(item)
    entry = dict(item)
    entry["t"] = 0.0
    DYNAMIC.append(entry)
    _refresh_dynamic_geometry()
    _mark_world_dirty()
    return item


def _clamp_inside(x, y, margin=0.45):
    """把巡逻端点收进场地内，避免动态障碍物跑到墙外。"""
    if point_in_polygon(x, y) and boundary_distance(x, y) >= margin:
        return x, y
    cx, cy = WIDTH / 2.0, HEIGHT / 2.0
    for step in range(1, 21):
        k = step / 20.0
        px = x + (cx - x) * k
        py = y + (cy - y) * k
        if point_in_polygon(px, py) and boundary_distance(px, py) >= margin:
            return px, py
    return cx, cy


def add_no_go_zone(x0, y0, x1, y1=None, height=None):
    """矩形禁区：用两个对角点（或起点 + 宽高）描述，机器人不进入、刀盘不切割。"""
    if y1 is None:
        raise ValueError("禁区需要两个对角点")
    x0, y0 = float(x0), float(y0)
    x1, y1 = float(x1), float(y1)
    if x1 < x0:
        x0, x1 = x1, x0
    if y1 < y0:
        y0, y1 = y1, y0
    x0 = clamp(x0, 0.0, WIDTH)
    x1 = clamp(x1, 0.0, WIDTH)
    y0 = clamp(y0, 0.0, HEIGHT)
    y1 = clamp(y1, 0.0, HEIGHT)
    if x1 - x0 < 0.30 or y1 - y0 < 0.30:
        raise ValueError("禁区矩形太小，请拖出至少 0.3 m × 0.3 m 的区域")
    center_x = (x0 + x1) / 2.0
    center_y = (y0 + y1) / 2.0
    if not point_in_polygon(center_x, center_y):
        raise ValueError("请把禁区放在场地内部")
    item = {
        "id": _next_edit_id(),
        "x0": round(x0, 2),
        "y0": round(y0, 2),
        "x1": round(x1, 2),
        "y1": round(y1, 2),
        "x": round(center_x, 2),
        "y": round(center_y, 2),
    }
    _edit_bucket()["no_go"].append(item)
    NO_GO.append(dict(item))
    _mark_world_dirty()
    return item


def dynamic_clearance(x, y, radius=None):
    """到最近动态障碍物表面的距离；负数表示车体已经与其重叠，无动态障碍物时返回 None。"""
    radius = ROBOT_RADIUS if radius is None else radius
    best = None
    for item in DYNAMIC:
        gap = math.hypot(x - item["x"], y - item["y"]) - item["r"] - radius
        if best is None or gap < best:
            best = gap
    return best


def in_dynamic_obstacle(x, y, margin=0.0):
    """点是否落在某个动态障碍物圆内（含余量）；用于避免把移动物体写进建图数据。"""
    for item in DYNAMIC:
        if math.hypot(x - item["x"], y - item["y"]) <= item["r"] + margin:
            return True
    return False


def in_no_go(x, y, margin=0.0):
    """判断点是否落在任一禁区矩形内（margin>0 时向外扩张）。"""
    for zone in NO_GO:
        if zone.get("x0") is None:
            cx = zone.get("x", 0.0)
            cy = zone.get("y", 0.0)
            if math.hypot(x - cx, y - cy) <= float(zone.get("r", 0.0)) + margin:
                return True
            continue
        x0 = min(zone["x0"], zone["x1"]) - margin
        x1 = max(zone["x0"], zone["x1"]) + margin
        y0 = min(zone["y0"], zone["y1"]) - margin
        y1 = max(zone["y0"], zone["y1"]) + margin
        if x0 <= x <= x1 and y0 <= y <= y1:
            return True
    return False


def update_dynamic_path(item_id, ax, ay, bx, by):
    """编辑动态障碍物的巡逻路径端点（A → B 往返）。"""
    item_id = int(item_id)
    bucket = _edit_bucket()
    target = None
    for entry in bucket["dynamic"]:
        if int(entry["id"]) == item_id:
            target = entry
            break
    if target is None:
        raise ValueError("没有找到该动态障碍物")
    ax, ay = _clamp_inside(float(ax), float(ay))
    bx, by = _clamp_inside(float(bx), float(by))
    if math.hypot(bx - ax, by - ay) < 0.40:
        raise ValueError("动态障碍物路径太短，至少 0.4 m")
    target["ax"] = round(ax, 2)
    target["ay"] = round(ay, 2)
    target["bx"] = round(bx, 2)
    target["by"] = round(by, 2)
    for entry in DYNAMIC:
        if int(entry["id"]) == item_id:
            entry["ax"] = target["ax"]
            entry["ay"] = target["ay"]
            entry["bx"] = target["bx"]
            entry["by"] = target["by"]
            entry["x"] = target["ax"]
            entry["y"] = target["ay"]
            entry["t"] = 0.0
            break
    _refresh_dynamic_geometry()
    _mark_world_dirty()
    return dict(target)


def remove_edit(kind, item_id):
    if kind not in EDIT_KINDS:
        raise ValueError("未知的编辑类型")
    item_id = int(item_id)
    bucket = _edit_bucket()
    keep = [item for item in bucket[kind] if int(item["id"]) != item_id]
    if len(keep) == len(bucket[kind]):
        raise ValueError("没有找到要删除的对象")
    bucket[kind] = keep
    if kind == "static":
        _rebuild_static_geometry()
    elif kind == "dynamic":
        DYNAMIC[:] = [item for item in DYNAMIC if int(item["id"]) != item_id]
        _refresh_dynamic_geometry()
    else:
        NO_GO[:] = [item for item in NO_GO if int(item["id"]) != item_id]
    _mark_world_dirty()
    return item_id


def static_edits():
    """用户手动添加的静态障碍快照（重建栅格后要重新写回去）。"""
    return [dict(item) for item in _edit_bucket()["static"]]


def clear_edits(kind=None):
    kinds = EDIT_KINDS if kind is None else (kind,)
    if kind is not None and kind not in EDIT_KINDS:
        raise ValueError("未知的编辑类型")
    bucket = _edit_bucket()
    for entry in kinds:
        bucket[entry] = []
    if "static" in kinds or "dynamic" in kinds:
        DYNAMIC[:] = []
        _rebuild_static_geometry()
    if "no_go" in kinds:
        NO_GO[:] = []
    _mark_world_dirty()


def update_dynamic(dt):
    """推进动态障碍物（两点往返巡逻）。"""
    moved = False
    for item in DYNAMIC:
        length = math.hypot(item["bx"] - item["ax"], item["by"] - item["ay"])
        if length < 0.05:
            continue
        item["t"] = (item.get("t", 0.0) + item["speed"] * dt / length) % 2.0
        phase = item["t"]
        ratio = phase if phase <= 1.0 else 2.0 - phase
        item["x"] = item["ax"] + (item["bx"] - item["ax"]) * ratio
        item["y"] = item["ay"] + (item["by"] - item["ay"]) * ratio
        moved = True
    if moved:
        _refresh_dynamic_geometry()


def edit_dict():
    bucket = _edit_bucket()
    return {
        "static": [dict(item) for item in bucket["static"]],
        "dynamic": [dict(item) for item in bucket["dynamic"]],
        "no_go": [dict(item) for item in bucket["no_go"]],
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
            "editor": bool(definition.get("editor_built")),
        }
        for definition in sorted(MAPS.values(), key=lambda item: item["id"])
        # 编辑中的临时"空白地图"不进地图列表，保存后才会作为新地图出现
        if not definition.get("editor_draft")
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


def nearest_boundary_point(x, y, polygon=None):
    """外墙折线上离 (x, y) 最近的点；用来把沿边路线压到离墙固定距离。"""
    polygon = POLYGON if polygon is None else polygon
    count = len(polygon)
    best = None
    for index in range(count):
        ax, ay = polygon[index]
        bx, by = polygon[(index + 1) % count]
        vx, vy = bx - ax, by - ay
        length_sq = vx * vx + vy * vy
        if length_sq <= 1e-12:
            t = 0.0
        else:
            t = clamp(((x - ax) * vx + (y - ay) * vy) / length_sq, 0.0, 1.0)
        px = ax + vx * t
        py = ay + vy * t
        distance = math.hypot(x - px, y - py)
        if best is None or distance < best[0]:
            best = (distance, px, py)
    if best is None:
        return (x, y, 0.0)
    return (best[1], best[2], best[0])


def boundary_distance(x, y, polygon=None):
    if polygon is None or polygon is POLYGON:
        # 快速路径：只算邻近分桶里的外墙线段（远距离查询回退全量）
        buckets = _EDGE_BUCKETS
        pool = _EDGE_POOL
        if buckets:
            cell = _bucket_index(x, y)
            cands = _EDGE_CELL_CACHE.get(cell)
            if cands is None:
                found = set()
                for jj in (cell[1] - 1, cell[1], cell[1] + 1):
                    for ii in (cell[0] - 1, cell[0], cell[0] + 1):
                        row = buckets.get((ii, jj))
                        if row:
                            found.update(row)
                cands = tuple(found)
                _EDGE_CELL_CACHE[cell] = cands
            best = 1e9
            for index in cands:
                ax, ay, ex, ey = pool[index]
                distance = _segment_distance(x, y, ax, ay, ax + ex, ay + ey)
                if distance < best:
                    best = distance
            if best <= SEG_BUCKET:
                return best
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
    """射线求交（内联几何运算，热路径上省掉函数调用开销）。"""
    limit = VISION_RANGE if max_range is None else max_range
    dx = math.cos(angle)
    dy = math.sin(angle)
    best = limit
    # 墙面线段（只查射线覆盖范围内的分桶；同一条线可能落在多个桶里，用戳记去重）
    pool = _SEG_POOL
    if pool and limit <= _SEG_CACHE_RANGE:
        # 一次查表拿到"这一格附近可能命中的全部线段"（以格为单位缓存），
        # 省掉每条射线 9 次分桶查找与去重戳记：视觉 41 条射线 × 每 tick 都算，
        # 这里省下来的就是倍速上限。
        cell = (int(math.floor(x / SEG_BUCKET)), int(math.floor(y / SEG_BUCKET)))
        cands = _SEG_CELL_CACHE.get(cell)
        if cands is None:
            i, j = cell
            found = set()
            for ii in range(i - 1, i + 2):
                for jj in range(j - 1, j + 2):
                    row = _SEG_BUCKETS.get((ii, jj))
                    if row:
                        found.update(row)
            cands = tuple(found)
            _SEG_CELL_CACHE[cell] = cands
        for index in cands:
            ax, ay, ex, ey = pool[index]
            denom = dx * ey - dy * ex
            if denom > -1e-9 and denom < 1e-9:
                continue
            qx = ax - x
            qy = ay - y
            t = (qx * ey - qy * ex) / denom
            if t <= 1e-6 or t >= best:
                continue
            u = (qx * dy - qy * dx) / denom
            if -1e-9 <= u <= 1.0 + 1e-9:
                best = t
    elif pool:
        buckets = _SEG_BUCKETS
        i0 = int(math.floor((x - limit) / SEG_BUCKET))
        i1 = int(math.floor((x + limit) / SEG_BUCKET))
        j0 = int(math.floor((y - limit) / SEG_BUCKET))
        j1 = int(math.floor((y + limit) / SEG_BUCKET))
        for i in range(i0, i1 + 1):
            for j in range(j0, j1 + 1):
                row = buckets.get((i, j))
                if not row:
                    continue
                for index in row:
                    ax, ay, ex, ey = pool[index]
                    denom = dx * ey - dy * ex
                    if denom > -1e-9 and denom < 1e-9:
                        continue
                    qx = ax - x
                    qy = ay - y
                    t = (qx * ey - qy * ex) / denom
                    if t <= 1e-6 or t >= best:
                        continue
                    u = (qx * dy - qy * dx) / denom
                    if -1e-9 <= u <= 1.0 + 1e-9:
                        best = t
    else:
        for ax, ay, bx, by in POLYGON_SEGMENTS + OBSTACLE_SEGMENTS:
            ex = bx - ax
            ey = by - ay
            denom = dx * ey - dy * ex
            if denom > -1e-9 and denom < 1e-9:
                continue
            qx = ax - x
            qy = ay - y
            t = (qx * ey - qy * ex) / denom
            if t <= 1e-6 or t >= best:
                continue
            u = (qx * dy - qy * dx) / denom
            if -1e-9 <= u <= 1.0 + 1e-9:
                best = t
    # 障碍物圆
    for ox, oy, radius in OBSTACLES:
        fx = x - ox
        fy = y - oy
        b = fx * dx + fy * dy
        c = fx * fx + fy * fy - radius * radius
        disc = b * b - c
        if disc < 0.0:
            continue
        root = math.sqrt(disc)
        hit = -b - root
        if hit <= 1e-6:
            hit = -b + root
        if 1e-6 < hit < best:
            best = hit
    return best if best < limit else limit


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
    """静态几何碰撞（墙体 + 原始障碍物 + 静态障碍物编辑）。

    动态障碍物不在这里判定：它会主动移动、甚至扫过车体，交给
    ``dynamic_clearance`` 单独处理，避免车体被移动物卡在原地。
    """
    radius = ROBOT_RADIUS if radius is None else radius
    if not point_in_polygon(x, y):
        return True
    if boundary_distance(x, y) < radius:
        return True
    for ox, oy, oradius in STATIC_PARTS:
        if (x - ox) ** 2 + (y - oy) ** 2 < (oradius + radius) ** 2:
            return True
    for polygon in OBSTACLE_POLYGONS:
        if point_in_polygon(x, y, polygon):
            return True
        if boundary_distance(x, y, polygon) < radius:
            return True
    return False


_POLYGON_MASK_CACHE = {}


def polygon_mask(cols, rows, cell=None):
    """把场地多边形栅格化成按行填充的位图（缓存），避免逐格 point_in_polygon。

    导入的实景地图有 300+ 个顶点，逐格判断要 20M+ 次线段测试（实测 13 s），
    用扫描线填充后只需 rows × edges 次。
    """
    from math import ceil, floor

    cell = CELL if cell is None else cell
    key = (WORLD_VERSION, int(cols), int(rows), round(cell, 6))
    cached = _POLYGON_MASK_CACHE.get(key)
    if cached is not None:
        return cached
    mask = bytearray(cols * rows)
    count = len(POLYGON)
    if count < 3:
        _POLYGON_MASK_CACHE[key] = mask
        return mask
    for row in range(rows):
        y = (row + 0.5) * cell
        crossings = []
        for index in range(count):
            x1, y1 = POLYGON[index]
            x2, y2 = POLYGON[(index + 1) % count]
            if (y1 > y) == (y2 > y):
                continue
            crossings.append(x1 + (y - y1) * (x2 - x1) / (y2 - y1))
        if not crossings:
            continue
        crossings.sort()
        base = row * cols
        for pair in range(0, len(crossings) - 1, 2):
            left = crossings[pair]
            right = crossings[pair + 1]
            col0 = max(0, int(ceil(left / cell - 0.5)))
            col1 = min(cols - 1, int(floor(right / cell - 0.5)))
            if col1 < col0:
                continue
            mask[base + col0 : base + col1 + 1] = b"\x01" * (col1 - col0 + 1)
    if len(_POLYGON_MASK_CACHE) > 4:
        _POLYGON_MASK_CACHE.clear()
    _POLYGON_MASK_CACHE[key] = mask
    return mask


def boundary_cells():
    """多边形边界附近的栅格集合，用于建图进度统计。

    只收中心点落在场地内的格子。射线永远命中墙面内侧，所以边界线外侧那一圈
    格子根本不可能被传感器写入；把它们算进分母会让探明率永远差一半，还会在
    补扫时把机器人派到永远补不上的"缺口"上。
    """
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
                    if not point_in_polygon((col + 0.5) * CELL, (row + 0.5) * CELL):
                        continue
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
    cache = _WORLD_CACHE
    if cache.get("version") != WORLD_VERSION:
        cache = _build_world_static()
        cache["version"] = WORLD_VERSION
        _WORLD_CACHE.clear()
        _WORLD_CACHE.update(cache)
    result = dict(cache)
    result.pop("version", None)
    # 桩可见性会随回放变化，不进静态缓存
    result["dock_visible"] = DOCK_VISIBLE
    # 地图纪元也随回放变化（没地图的日志会把场地清成空白地图）
    result["epoch"] = MAP_EPOCH
    result["obstacles"] = OBSTACLES
    result["dynamic"] = [
        {
            "id": item["id"],
            "x": round(item.get("x", 0.0), 2),
            "y": round(item.get("y", 0.0), 2),
            "r": item["r"],
            "ax": item["ax"],
            "ay": item["ay"],
            "bx": item["bx"],
            "by": item["by"],
        }
        for item in DYNAMIC
    ]
    result["no_go"] = [dict(item) for item in NO_GO]
    result["regions"] = [
        {
            "id": item.get("id"),
            "points": [[round(x, 3), round(y, 3)] for x, y in item["points"]],
        }
        for item in REGIONS
    ]
    result["passages"] = [
        {
            "id": item.get("id"),
            "points": [[round(x, 2), round(y, 2)] for x, y in item["points"]],
            "half": round(item["half"], 3),
            "from": item.get("from"),
            "to": item.get("to"),
        }
        for item in PASSAGES
    ]
    result["edits"] = edit_dict()
    return result


def _build_world_static():
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
