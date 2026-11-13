"""可视化地图编辑器：用画出来的区域 / 通道 / 障碍生成一张新地图。

工作方式：进入编辑模式后先切到一张"空白场地"（只有一块很大的空旷区域，没有
墙体），使用者手动驾驶小车、在地图上画区域和通道，最后保存成一张新地图
（注册进地图列表，编号从 101 起）。编辑过程中所有几何都存在这里的 Draft 里，
不会动到原地图；保存时才栅格化成一张真正的仿真场地。
"""

import copy
import math

import map_import
import world

# 编辑画布尺寸（米）：一块 100 × 100 m 的空白场地，想画多大画多大；
# 保存时再按实际用到的元素范围裁剪成真正的地图尺寸。
DEFAULT_WIDTH = 100.0
DEFAULT_HEIGHT = 100.0
# 保存时在元素外接框之外留出的余量（米）
CROP_MARGIN = 1.2
# 区域闭合容差（米）：首尾点距离超过它就算"没闭合"
REGION_CLOSE_TOLERANCE = 1.20
# 区域最小面积（m²）
REGION_MIN_AREA = 2.0
# 通道半宽（米）：单行通道约 1.0 m 宽（0.8 m 时建图会蹭墙、进不去，
# 实测 1.0 m 全程 0 碰撞），只够一辆车顺着走，不用于割草
PASSAGE_HALF_WIDTH = 0.50
# 充电桩默认半径与离墙/离障碍的最小间隙
DOCK_RADIUS = 0.45
DOCK_CLEARANCE = 0.20
# 画笔默认半径（静态障碍涂抹）
BRUSH_RADIUS = 0.25
# 充电桩 3 m 内不许放禁区 / 障碍物（车端协议 FORBIDDEN_NEAR_STATION 的要求，
# 这里按本系统的场地尺度放宽到 3 m，给回桩留出转向与对桩空间）
DOCK_KEEPOUT = 3.0

# 错误码：编号沿用上级系统（车端地图协议）的编码，只挑出本编辑器用得上的那些，
# 界面上按「E11 创建禁区失败：…」显示，日志/测试也能按名字断言。
ERROR_CODES = {
    "NO_ENOUGH_WORKAREA_BEFORE_CREATE_TIE": (1, "创建通道前请先画至少一个区域"),
    "START_POINT_OF_TIE_IS_OUT_OF_MAP": (3, "通道起点在场地范围外"),
    "CURRENT_POINT_IS_OUT_OF_MAP": (5, "位置在地图范围外：请把点落在 100 × 100 m 场地内"),
    "TOO_FEW_POINTS_BEFORE_SAVE_MAP": (6, "保存地图失败：有效面积太小（小于 1 m²）"),
    "NO_MAP_AND_CREATE_NOT_WORKAREA": (7, "还没有工作区：请先画至少一个区域"),
    "PATH_EMPTY_WHEN_SAVE_MAP": (8, "保存地图失败：地图数据为空"),
    "FORBIDDEN_CROSSING_PATH": (10, "创建禁区失败：禁区不能压到通道"),
    "FORBIDDEN_NEAR_STATION": (11, "充电桩 3 m 范围内不能设置禁区或障碍物"),
    "PATH_CROSSING_FORBIDDEN": (12, "创建通道失败：通道不能穿过禁区"),
    "END_POINT_OF_TIE_IS_OUT_OF_MAP": (13, "通道终点在场地范围外"),
    "WRONG_DATA_STRUCT": (16, "保存地图失败：地图数据异常"),
    "CANT_FIND_SPECIFIC_ID": (17, "删除失败：没有找到对应的元素"),
    "DELETE_SINGLE_MAP_FAIL": (18, "删除地图失败"),
    "LOAD_MAP_FAIL": (19, "加载地图失败"),
    "CREATE_CHARGE_STATION_FAIL": (20, "创建充电桩失败"),
    "START_AND_END_POINT_OF_TUNNEL_NOT_IN_DIFFERENT_AREAS": (
        22,
        "创建通道错误：通道两端必须分别落在两个不同的区域内",
    ),
}


class EditorError(ValueError):
    """带错误码的编辑错误（错误码沿用上级系统协议，见 ERROR_CODES）。

    文案统一成 "E11 规则（具体对象）" 的形式，界面直接显示 ``str(err)`` 即可。
    """

    def __init__(self, code, detail=None):
        number, text = ERROR_CODES.get(code, (0, code))
        self.code = code
        self.number = number
        self.detail = detail
        if detail and detail != text:
            message = "E{:02d} {}（{}）".format(number, text, detail)
        else:
            message = "E{:02d} {}".format(number, text)
        super().__init__(message)

# 最多记住多少步编辑（供"撤销"用）
UNDO_LIMIT = 30

_ROUND = 3


def _clean_points(points):
    cleaned = []
    for item in points or []:
        try:
            x = float(item[0])
            y = float(item[1])
        except (TypeError, ValueError, IndexError):
            continue
        if cleaned and math.hypot(x - cleaned[-1][0], y - cleaned[-1][1]) < 0.02:
            continue
        cleaned.append((round(x, _ROUND), round(y, _ROUND)))
    return cleaned


def _polygon_area(points):
    area = 0.0
    count = len(points)
    for index in range(count):
        x1, y1 = points[index]
        x2, y2 = points[(index + 1) % count]
        area += x1 * y2 - x2 * y1
    return abs(area) * 0.5


def _point_in_polygon(x, y, points):
    inside = False
    count = len(points)
    j = count - 1
    for i in range(count):
        xi, yi = points[i]
        xj, yj = points[j]
        if ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi
        ):
            inside = not inside
        j = i
    return inside


def _distance_to_polyline(x, y, points):
    best = 1e9
    if len(points) == 1:
        return math.hypot(x - points[0][0], y - points[0][1])
    for index in range(len(points) - 1):
        ax, ay = points[index]
        bx, by = points[index + 1]
        vx, vy = bx - ax, by - ay
        length_sq = vx * vx + vy * vy
        if length_sq <= 1e-12:
            distance = math.hypot(x - ax, y - ay)
        else:
            t = ((x - ax) * vx + (y - ay) * vy) / length_sq
            t = max(0.0, min(1.0, t))
            distance = math.hypot(x - (ax + vx * t), y - (ay + vy * t))
        if distance < best:
            best = distance
    return best


def _distance_point_to_rect(x, y, rect):
    """点到矩形（x0, y0, x1, y1）的最短距离，点在矩形内为 0。"""
    x0, y0, x1, y1 = rect
    dx = max(x0 - x, 0.0, x - x1)
    dy = max(y0 - y, 0.0, y - y1)
    return math.hypot(dx, dy)


def _polyline_hits_rect(points, rect, half=0.0):
    """折线（按半径 half 加粗）是否压到矩形。"""
    x0, y0, x1, y1 = rect
    x0, y0 = x0 - half, y0 - half
    x1, y1 = x1 + half, y1 + half
    if not points:
        return False
    if len(points) == 1:
        x, y = points[0]
        return x0 <= x <= x1 and y0 <= y <= y1
    for index in range(len(points) - 1):
        ax, ay = points[index]
        bx, by = points[index + 1]
        steps = max(1, int(math.hypot(bx - ax, by - ay) / 0.1))
        for step in range(steps + 1):
            t = step / float(steps)
            x = ax + (bx - ax) * t
            y = ay + (by - ay) * t
            if x0 <= x <= x1 and y0 <= y <= y1:
                return True
    return False


def _densify(points, step=0.05):
    out = []
    for index in range(len(points) - 1):
        ax, ay = points[index]
        bx, by = points[index + 1]
        length = math.hypot(bx - ax, by - ay)
        count = max(1, int(length / step))
        for k in range(count):
            t = k / float(count)
            out.append((ax + (bx - ax) * t, ay + (by - ay) * t))
    out.append(tuple(points[-1]))
    return out


def _simplify_open(points, tolerance=0.02):
    """Douglas-Peucker：去直段上的冗余点，但保留每个拐角。"""
    if len(points) < 3:
        return list(points)
    keep = [False] * len(points)
    keep[0] = keep[-1] = True
    stack = [(0, len(points) - 1)]
    while stack:
        start, end = stack.pop()
        ax, ay = points[start]
        bx, by = points[end]
        vx, vy = bx - ax, by - ay
        length = math.hypot(vx, vy)
        best = -1.0
        best_index = -1
        for index in range(start + 1, end):
            px, py = points[index]
            if length < 1e-9:
                distance = math.hypot(px - ax, py - ay)
            else:
                distance = abs((px - ax) * vy - (py - ay) * vx) / length
            if distance > best:
                best = distance
                best_index = index
        if best_index >= 0 and best > tolerance:
            keep[best_index] = True
            stack.append((start, best_index))
            stack.append((best_index, end))
    return [points[index] for index in range(len(points)) if keep[index]]


def _stroke_outline(points, radius, cap_segments=8):
    """把"实心画笔"笔迹（折线 + 半径）转成一个闭合多边形轮廓。

    两侧各按法线偏移 radius，两端补半圆帽，得到真正的实心带（而不是一串圆点）。
    """
    pts = _densify(points, 0.12) if len(points) > 1 else [tuple(points[0])]
    if len(pts) < 2:
        return _circle_outline(pts[0][0], pts[0][1], radius, cap_segments * 3)
    left = []
    right = []
    count = len(pts)
    for index, (x, y) in enumerate(pts):
        if index == 0:
            tx = pts[1][0] - x
            ty = pts[1][1] - y
        elif index == count - 1:
            tx = x - pts[index - 1][0]
            ty = y - pts[index - 1][1]
        else:
            tx = pts[index + 1][0] - pts[index - 1][0]
            ty = pts[index + 1][1] - pts[index - 1][1]
        length = math.hypot(tx, ty)
        if length < 1e-9:
            continue
        nx, ny = -ty / length, tx / length
        left.append((x + nx * radius, y + ny * radius))
        right.append((x - nx * radius, y - ny * radius))
    left = _simplify_open(left)
    right = _simplify_open(right)
    outline = list(left)
    # 尾部半圆帽：从左侧绕到右侧
    end = pts[-1]
    angle = math.atan2(end[1] - pts[-2][1], end[0] - pts[-2][0])
    for k in range(1, cap_segments):
        a = angle + math.pi / 2 - math.pi * k / cap_segments
        outline.append(
            (end[0] + math.cos(a) * radius, end[1] + math.sin(a) * radius)
        )
    outline.extend(reversed(right))
    # 起始半圆帽：从右侧绕回左侧
    start = pts[0]
    angle = math.atan2(start[1] - pts[1][1], start[0] - pts[1][0])
    for k in range(1, cap_segments):
        a = angle + math.pi / 2 - math.pi * k / cap_segments
        outline.append(
            (start[0] + math.cos(a) * radius, start[1] + math.sin(a) * radius)
        )
    return [(round(x, 4), round(y, 4)) for x, y in outline]


def _circle_outline(cx, cy, radius, segments=24):
    return [
        (
            round(cx + radius * math.cos(index * 2.0 * math.pi / segments), 4),
            round(cy + radius * math.sin(index * 2.0 * math.pi / segments), 4),
        )
        for index in range(segments)
    ]


def _trace_mask_loops(width, height, mask, ox, oy, cell):
    """把 0/1 掩膜转成闭合折线（外环 + 洞），坐标换算回米。

    用法与 `GridMap.zone_outline` 同源：沿每格的"没有邻居"的边连成闭环。
    """
    edges = {}
    for index, value in enumerate(mask):
        if not value:
            continue
        col = index % width
        row = index // width
        x0 = ox + col * cell
        y0 = oy + row * cell
        x1 = x0 + cell
        y1 = y0 + cell
        left = index - 1 if col > 0 else -1
        right = index + 1 if col + 1 < width else -1
        down = index - width if row > 0 else -1
        up = index + width if row + 1 < height else -1
        if down < 0 or not mask[down]:
            edges.setdefault((round(x0, 4), round(y0, 4)), []).append(
                (round(x1, 4), round(y0, 4))
            )
        if right < 0 or not mask[right]:
            edges.setdefault((round(x1, 4), round(y0, 4)), []).append(
                (round(x1, 4), round(y1, 4))
            )
        if up < 0 or not mask[up]:
            edges.setdefault((round(x1, 4), round(y1, 4)), []).append(
                (round(x0, 4), round(y1, 4))
            )
        if left < 0 or not mask[left]:
            edges.setdefault((round(x0, 4), round(y1, 4)), []).append(
                (round(x0, 4), round(y0, 4))
            )
    loops = []
    guard = sum(len(v) for v in edges.values()) + 8
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
    return loops


def _simplify_ring_closed(points, tolerance=0.03):
    """闭环的 Douglas-Peucker 抽稀（去掉栅格台阶上的冗余点）。"""
    if len(points) < 4:
        return list(points)
    ring = list(points) + [points[0]]
    simplified = _simplify_open(ring, tolerance)
    if len(simplified) >= 2 and simplified[-1] == simplified[0]:
        simplified.pop()
    return simplified if len(simplified) >= 3 else list(points)


def _polyline_length(points):
    total = 0.0
    for index in range(len(points) - 1):
        total += math.hypot(
            points[index + 1][0] - points[index][0],
            points[index + 1][1] - points[index][1],
        )
    return total


def _distance_to_boundary(x, y, points):
    """点到闭合折线（区域边界）的最近距离，用来保证充电桩不压边。"""
    if len(points) < 3:
        return 0.0
    ring = list(points) + [points[0]]
    return _distance_to_polyline(x, y, ring)


class Draft(object):
    """一张正在编辑的地图。"""

    def __init__(self, name="自定义地图"):
        self.name = str(name or "自定义地图")
        self.width = DEFAULT_WIDTH
        self.height = DEFAULT_HEIGHT
        self.regions = []
        self.passages = []
        self.docks = []
        self.no_gos = []
        self.static = []
        # 从已有地图读进来的多边形障碍（涂抹生成的仍存在 static 里）
        self.poly_obstacles = []
        self.dynamic = []
        self._next_id = 1
        self.map_id = None
        # 撤销栈：每次改动前压一份几何快照（纯数据，几十 KB 级）
        self._history = []
        # 笔迹合并轮廓的缓存
        self._brush_cache_key = None
        self._brush_cache = []
        # 几何改动计数：笔迹轮廓、内容外接框的缓存键只比这个数 + 列表对象，
        # 不再每次把上万个点拼进键里（编辑时每秒要算十几次）
        self._paint_rev = 0
        self._bounds_cache = None

    # ------------------------------------------------------------- 撤销 / 平移
    def _snapshot(self):
        return {
            "regions": copy.deepcopy(self.regions),
            "passages": copy.deepcopy(self.passages),
            "docks": copy.deepcopy(self.docks),
            "no_gos": copy.deepcopy(self.no_gos),
            "static": copy.deepcopy(self.static),
            "poly_obstacles": copy.deepcopy(self.poly_obstacles),
            "dynamic": copy.deepcopy(self.dynamic),
            "next_id": self._next_id,
            "name": self.name,
        }

    def _push_history(self):
        self._paint_rev += 1
        self._history.append(self._snapshot())
        if len(self._history) > UNDO_LIMIT:
            self._history.pop(0)

    def _geometry_key(self):
        return (
            repr(self.regions),
            repr(self.passages),
            repr(self.docks),
            repr(self.no_gos),
            repr(self.static),
            repr(self.poly_obstacles),
            repr(self.dynamic),
        )

    def undo(self):
        """撤回上一步编辑（主要是防误删）。"""
        self._paint_rev += 1
        while self._history:
            state = self._history.pop()
            if self._state_key(state) == self._geometry_key():
                # 失败的操作也会压一份历史，这种"没改动"的快照直接跳过
                continue
            self.regions = state["regions"]
            self.passages = state["passages"]
            self.docks = state["docks"]
            self.no_gos = state["no_gos"]
            self.static = state["static"]
            self.poly_obstacles = state["poly_obstacles"]
            self.dynamic = state["dynamic"]
            self._next_id = state["next_id"]
            return len(self._history)
        raise ValueError("没有可以撤销的操作了")

    @staticmethod
    def _state_key(state):
        return (
            repr(state["regions"]),
            repr(state["passages"]),
            repr(state["docks"]),
            repr(state["no_gos"]),
            repr(state["static"]),
            repr(state["poly_obstacles"]),
            repr(state["dynamic"]),
        )

    @property
    def can_undo(self):
        return bool(self._history)

    def _translate(self, dx, dy):
        """把全部几何平移（载入已有地图时用它把图摆到画布中央）。"""
        self._paint_rev += 1

        def move_point(point):
            return (round(point[0] + dx, _ROUND), round(point[1] + dy, _ROUND))

        for region in self.regions:
            region["points"] = [move_point(point) for point in region["points"]]
        for corridor in self.passages:
            corridor["points"] = [move_point(point) for point in corridor["points"]]
        for item in self.docks:
            item["x"] = round(item["x"] + dx, _ROUND)
            item["y"] = round(item["y"] + dy, _ROUND)
        for item in self.no_gos:
            item["x0"] = round(item["x0"] + dx, _ROUND)
            item["x1"] = round(item["x1"] + dx, _ROUND)
            item["y0"] = round(item["y0"] + dy, _ROUND)
            item["y1"] = round(item["y1"] + dy, _ROUND)
        for item in self.static:
            item["points"] = [move_point(point) for point in item["points"]]
        for item in self.poly_obstacles:
            item["points"] = [move_point(point) for point in item["points"]]
        for item in self.dynamic:
            item["x"] = round(item["x"] + dx, _ROUND)
            item["y"] = round(item["y"] + dy, _ROUND)
            item["ax"] = round(item["ax"] + dx, _ROUND)
            item["ay"] = round(item["ay"] + dy, _ROUND)
            item["bx"] = round(item["bx"] + dx, _ROUND)
            item["by"] = round(item["by"] + dy, _ROUND)

    # ------------------------------------------------------------- 基础
    def _id(self):
        value = self._next_id
        self._next_id += 1
        return value

    def _inside_canvas(self, x, y, margin=0.0):
        return (
            margin <= x <= self.width - margin
            and margin <= y <= self.height - margin
        )

    def region_at(self, x, y):
        for region in self.regions:
            if _point_in_polygon(x, y, region["points"]):
                return region
        return None

    def _keeper_hit(self, x, y):
        """(x, y) 落在充电桩 3 m 内的禁区/障碍物吗？返回提示文字，否则 None。"""
        keep = DOCK_KEEPOUT
        for zone in self.no_gos:
            rect = (zone["x0"], zone["y0"], zone["x1"], zone["y1"])
            if _distance_point_to_rect(x, y, rect) < keep:
                return "充电桩 {} m 范围内有禁区 #{}".format(int(keep), zone["id"])
        for item in self.static:
            if _distance_to_polyline(x, y, item["points"]) - item["r"] < keep:
                return "充电桩 {} m 范围内有障碍物 #{}".format(int(keep), item["id"])
        for item in self.poly_obstacles:
            if _point_in_polygon(x, y, item["points"]) or (
                _distance_to_boundary(x, y, item["points"]) < keep
            ):
                return "充电桩 {} m 范围内有障碍物 #{}".format(int(keep), item["id"])
        for item in self.dynamic:
            if math.hypot(x - item["x"], y - item["y"]) - item["r"] < keep:
                return "充电桩 {} m 范围内有动态障碍物 #{}".format(int(keep), item["id"])
        return None

    def _dock_keepout_hit(self, points, radius=0.0):
        """这条笔迹/轮廓是否进了充电桩的 3 m 净空：进了就返回 (桩, 距离)。"""
        if not self.docks:
            return None
        dock = self.docks[0]
        if len(points) >= 3 and _point_in_polygon(dock["x"], dock["y"], points):
            distance = 0.0
        else:
            distance = _distance_to_polyline(dock["x"], dock["y"], points)
        distance = max(0.0, distance - radius)
        if distance < DOCK_KEEPOUT:
            return dock, distance
        return None

    def _obstacle_hit(self, x, y, padding=0.0):
        for item in self.static:
            if _distance_to_polyline(x, y, item["points"]) <= item["r"] + padding:
                return item
        for item in self.poly_obstacles:
            if _point_in_polygon(x, y, item["points"]):
                return item
        return None

    # ------------------------------------------------------------- 元素
    def add_region(self, points):
        self._push_history()
        cleaned = _clean_points(points)
        if len(cleaned) < 4:
            raise ValueError("区域轨迹太短：请驾驶小车绕一圈再保存")
        gap = math.hypot(
            cleaned[0][0] - cleaned[-1][0], cleaned[0][1] - cleaned[-1][1]
        )
        if gap > REGION_CLOSE_TOLERANCE:
            raise ValueError(
                "区域未闭合：起点与终点相距 {:.2f} m，请继续沿轨迹行驶回到起点附近".format(gap)
            )
        closed = list(cleaned)
        closed[-1] = closed[0]
        area = _polygon_area(closed[:-1])
        if area < REGION_MIN_AREA:
            raise ValueError(
                "区域面积 {:.2f} m² 小于 {:.1f} m²，请把范围画大一些".format(
                    area, REGION_MIN_AREA
                )
            )
        region = {
            "id": self._id(),
            "kind": "region",
            "points": closed[:-1],
            "area": round(area, 2),
        }
        self.regions.append(region)
        return region

    def add_passage(self, points):
        self._push_history()
        if not self.regions:
            raise EditorError("NO_ENOUGH_WORKAREA_BEFORE_CREATE_TIE")
        cleaned = _clean_points(points)
        if len(cleaned) < 2:
            raise EditorError(
                "START_AND_END_POINT_OF_TUNNEL_NOT_IN_DIFFERENT_AREAS",
                "通道轨迹太短，请从区域 A 开进区域 B 再保存",
            )
        if not self._inside_canvas(cleaned[0][0], cleaned[0][1]):
            raise EditorError("START_POINT_OF_TIE_IS_OUT_OF_MAP")
        if not self._inside_canvas(cleaned[-1][0], cleaned[-1][1]):
            raise EditorError("END_POINT_OF_TIE_IS_OUT_OF_MAP")
        for zone in self.no_gos:
            rect = (zone["x0"], zone["y0"], zone["x1"], zone["y1"])
            if _polyline_hits_rect(cleaned, rect, PASSAGE_HALF_WIDTH):
                raise EditorError("PATH_CROSSING_FORBIDDEN", "禁区 #{}".format(zone["id"]))
        start_region = self.region_at(cleaned[0][0], cleaned[0][1])
        end_region = self.region_at(cleaned[-1][0], cleaned[-1][1])
        if start_region is None or end_region is None:
            where = "起点" if start_region is None else "终点"
            raise EditorError(
                "START_AND_END_POINT_OF_TUNNEL_NOT_IN_DIFFERENT_AREAS",
                where + "不在任何区域内部",
            )
        if start_region["id"] == end_region["id"]:
            raise EditorError(
                "START_AND_END_POINT_OF_TUNNEL_NOT_IN_DIFFERENT_AREAS",
                "两端都落在区域 #{} 里".format(start_region["id"]),
            )
        corridor = {
            "id": self._id(),
            "kind": "passage",
            "points": cleaned,
            "half": PASSAGE_HALF_WIDTH,
            "from": start_region["id"],
            "to": end_region["id"],
        }
        self.passages.append(corridor)
        return corridor

    def add_dock(self, x, y, radius=None):
        self._push_history()
        x, y = float(x), float(y)
        radius = DOCK_RADIUS if radius is None else float(radius)
        radius = max(0.35, min(1.20, radius))
        if self.docks:
            raise ValueError("最多只能有一个充电桩，请先删除现有的充电桩")
        region = self.region_at(x, y)
        if region is None:
            raise ValueError("充电桩必须放在某个区域内部")
        if _distance_to_boundary(x, y, region["points"]) < radius + DOCK_CLEARANCE:
            raise ValueError("充电桩离区域边界太近，请往区域中间挪一点")
        if self._obstacle_hit(x, y, radius + DOCK_CLEARANCE) is not None:
            raise ValueError("充电桩范围内有障碍物，请换一个位置")
        for item in self.dynamic:
            if math.hypot(x - item["x"], y - item["y"]) <= radius + item["r"] + DOCK_CLEARANCE:
                raise ValueError("充电桩范围内有动态障碍物，请换一个位置")
        close = self._keeper_hit(x, y)
        if close is not None:
            raise EditorError("CREATE_CHARGE_STATION_FAIL", close)
        dock = {
            "id": self._id(),
            "kind": "dock",
            "x": round(x, _ROUND),
            "y": round(y, _ROUND),
            "r": round(radius, _ROUND),
        }
        self.docks.append(dock)
        return dock

    def add_no_go(self, x0, y0, x1, y1):
        self._push_history()
        x0, y0 = float(x0), float(y0)
        x1, y1 = float(x1), float(y1)
        if x1 < x0:
            x0, x1 = x1, x0
        if y1 < y0:
            y0, y1 = y1, y0
        if not self.regions:
            raise EditorError("NO_MAP_AND_CREATE_NOT_WORKAREA")
        if x1 - x0 < 0.4 or y1 - y0 < 0.4:
            raise ValueError("禁区太小：请拖出至少 0.4 m × 0.4 m 的矩形")
        if not (self._inside_canvas(x0, y0) and self._inside_canvas(x1, y1)):
            raise EditorError("CURRENT_POINT_IS_OUT_OF_MAP")
        rect = (x0, y0, x1, y1)
        for corridor in self.passages:
            if _polyline_hits_rect(corridor["points"], rect, corridor["half"]):
                raise EditorError("FORBIDDEN_CROSSING_PATH", "通道 #{}".format(corridor["id"]))
        if self.docks:
            dock = self.docks[0]
            distance = _distance_point_to_rect(dock["x"], dock["y"], rect)
            if distance < DOCK_KEEPOUT:
                raise EditorError(
                    "FORBIDDEN_NEAR_STATION",
                    "禁区离充电桩 #{:.0f} 只有 {:.1f} m".format(dock["id"], distance),
                )
        item = {
            "id": self._id(),
            "kind": "no_go",
            "x0": round(x0, _ROUND),
            "y0": round(y0, _ROUND),
            "x1": round(x1, _ROUND),
            "y1": round(y1, _ROUND),
        }
        self.no_gos.append(item)
        return item

    def add_static(self, points, radius=None):
        """实心画笔涂抹：一次落笔就是一条带半径的笔迹（不再是若干圆点）。

        起点接在上一条笔迹末端、且半径相同时直接续笔，不会越涂越碎。
        """
        self._push_history()
        radius = BRUSH_RADIUS if radius is None else float(radius)
        radius = max(0.10, min(1.20, radius))
        cleaned = _clean_points(points)
        cleaned = [point for point in cleaned if self._inside_canvas(point[0], point[1], 0.0)]
        if not cleaned:
            raise EditorError("CURRENT_POINT_IS_OUT_OF_MAP")
        close = self._dock_keepout_hit(cleaned, radius)
        if close is not None:
            dock, distance = close
            raise EditorError(
                "FORBIDDEN_NEAR_STATION",
                "障碍物离充电桩 #{:.0f} 只有 {:.1f} m".format(dock["id"], distance),
            )
        last_id = None
        for item in self.static:
            if abs(item["r"] - radius) > 1e-6:
                continue
            tail = item["points"][-1]
            head = cleaned[0]
            if math.hypot(tail[0] - head[0], tail[1] - head[1]) <= max(0.35, radius):
                last_id = item["id"]
                break
        if last_id is not None:
            for item in self.static:
                if item["id"] == last_id:
                    item["points"].extend(cleaned[1:])
                    item["length"] = round(_polyline_length(item["points"]), 2)
                    return item
        item = {
            "id": self._id(),
            "kind": "brush",
            "points": cleaned,
            "r": round(radius, _ROUND),
            "length": round(_polyline_length(cleaned), 2),
        }
        self.static.append(item)
        return item

    def add_dynamic(
        self,
        x,
        y,
        radius=None,
        span=None,
        heading=0.0,
        ax=None,
        ay=None,
        bx=None,
        by=None,
        speed=None,
    ):
        x, y = float(x), float(y)
        self._push_history()
        radius = 0.30 if radius is None else float(radius)
        radius = max(0.12, min(1.00, radius))
        span = 1.6 if span is None else float(span)
        span = max(0.4, min(6.0, span))
        if not self._inside_canvas(x, y, 0.4):
            raise EditorError("CURRENT_POINT_IS_OUT_OF_MAP")
        if ax is not None and bx is not None:
            ax, ay = float(ax), float(ay)
            bx, by = float(bx), float(by)
            if not (self._inside_canvas(ax, ay, 0.3) and self._inside_canvas(bx, by, 0.3)):
                raise EditorError("CURRENT_POINT_IS_OUT_OF_MAP")
            if math.hypot(bx - ax, by - ay) < 0.4:
                raise ValueError("动态障碍物路径太短，至少 0.4 m")
            x = (ax + bx) / 2.0
            y = (ay + by) / 2.0
            span = math.hypot(bx - ax, by - ay)
            heading = math.atan2(by - ay, bx - ax)
        else:
            half = span / 2.0
            dx = math.cos(float(heading)) * half
            dy = math.sin(float(heading)) * half
            ax = max(0.4, min(self.width - 0.4, x - dx))
            ay = max(0.4, min(self.height - 0.4, y - dy))
            bx = max(0.4, min(self.width - 0.4, x + dx))
            by = max(0.4, min(self.height - 0.4, y + dy))
        path = [(ax, ay), (bx, by)]
        close = self._dock_keepout_hit(path, radius)
        if close is not None:
            dock, distance = close
            raise EditorError(
                "FORBIDDEN_NEAR_STATION",
                "动态障碍物离充电桩 #{:.0f} 只有 {:.1f} m".format(dock["id"], distance),
            )
        item = {
            "id": self._id(),
            "kind": "dynamic",
            "x": round(x, _ROUND),
            "y": round(y, _ROUND),
            "r": round(radius, _ROUND),
            "span": round(span, _ROUND),
            "heading": round(float(heading), 4),
            "ax": round(ax, _ROUND),
            "ay": round(ay, _ROUND),
            "bx": round(bx, _ROUND),
            "by": round(by, _ROUND),
            "speed": round(
                world.DYNAMIC_SPEED
                if speed is None
                else max(0.05, min(0.60, float(speed))),
                3,
            ),
        }
        self.dynamic.append(item)
        return item

    def update_dynamic(self, item_id, ax=None, ay=None, bx=None, by=None, speed=None, radius=None):
        """改动态障碍物的巡逻路径 / 速度 / 半径。"""
        self._push_history()
        item_id = int(item_id)
        target = None
        for item in self.dynamic:
            if item["id"] == item_id:
                target = item
                break
        if target is None:
            raise ValueError("没有找到该动态障碍物")
        if ax is not None and bx is not None:
            ax, ay = float(ax), float(ay)
            bx, by = float(bx), float(by)
            if not (self._inside_canvas(ax, ay, 0.3) and self._inside_canvas(bx, by, 0.3)):
                raise ValueError("动态障碍物的巡逻端点要放在地图范围内")
            if math.hypot(bx - ax, by - ay) < 0.4:
                raise ValueError("动态障碍物路径太短，至少 0.4 m")
            close = self._dock_keepout_hit([(ax, ay), (bx, by)], target["r"])
            if close is not None:
                dock, distance = close
                raise EditorError(
                    "FORBIDDEN_NEAR_STATION",
                    "动态障碍物离充电桩 #{:.0f} 只有 {:.1f} m".format(dock["id"], distance),
                )
            target["ax"] = round(ax, _ROUND)
            target["ay"] = round(ay, _ROUND)
            target["bx"] = round(bx, _ROUND)
            target["by"] = round(by, _ROUND)
            target["x"] = round((ax + bx) / 2.0, _ROUND)
            target["y"] = round((ay + by) / 2.0, _ROUND)
            target["span"] = round(math.hypot(bx - ax, by - ay), _ROUND)
            target["heading"] = round(math.atan2(by - ay, bx - ax), 4)
        if speed is not None:
            target["speed"] = round(max(0.05, min(0.60, float(speed))), 3)
        if radius is not None:
            target["r"] = round(max(0.12, min(1.00, float(radius))), _ROUND)
        return target

    def add_obstacle_polygon(self, points):
        self._push_history()
        """多边形障碍（从已有地图读进来时用；涂抹仍然走 add_static）。"""
        cleaned = _clean_points(points)
        if len(cleaned) < 3:
            raise ValueError("障碍轮廓至少需要 3 个点")
        area = _polygon_area(cleaned)
        if area < 0.05:
            raise ValueError("障碍轮廓面积太小")
        for point in cleaned:
            if not self._inside_canvas(point[0], point[1]):
                raise EditorError("CURRENT_POINT_IS_OUT_OF_MAP")
        close = self._dock_keepout_hit(cleaned)
        if close is not None:
            dock, distance = close
            raise EditorError(
                "FORBIDDEN_NEAR_STATION",
                "障碍物离充电桩 #{:.0f} 只有 {:.1f} m".format(dock["id"], distance),
            )
        item = {
            "id": self._id(),
            "kind": "obstacle",
            "points": cleaned,
            "area": round(area, 2),
        }
        self.poly_obstacles.append(item)
        return item

    def delete(self, kind, item_id):
        self._push_history()
        kind = str(kind or "")
        try:
            item_id = int(item_id)
        except (TypeError, ValueError):
            raise EditorError("CANT_FIND_SPECIFIC_ID", "要删除的元素编号无效")
        buckets = {
            "region": self.regions,
            "passage": self.passages,
            "dock": self.docks,
            "no_go": self.no_gos,
            "static": self.static,
            "obstacle": self.poly_obstacles,
            "dynamic": self.dynamic,
        }
        bucket = buckets.get(kind)
        if bucket is None:
            raise ValueError("未知的元素类型: {}".format(kind))
        for index, item in enumerate(bucket):
            if item["id"] == item_id:
                bucket.pop(index)
                if kind == "region":
                    # 区域删掉后，连到它的通道也一并清掉，避免出现悬空通道
                    keep = [
                        passage
                        for passage in self.passages
                        if item_id not in (passage["from"], passage["to"])
                    ]
                    removed = len(self.passages) - len(keep)
                    self.passages = keep
                    return {"deleted": item, "removed_passages": removed}
                return {"deleted": item, "removed_passages": 0}
        raise EditorError("CANT_FIND_SPECIFIC_ID")

    def hit_test(self, x, y):
        """按"越小越优先"的顺序命中测试，供界面点击删除使用。"""
        x, y = float(x), float(y)
        # 面元素优先（看得见的先命中），点元素只用一点点容差，避免误删旁边的东西
        for item in self.no_gos:
            if item["x0"] <= x <= item["x1"] and item["y0"] <= y <= item["y1"]:
                return {"kind": "no_go", "id": item["id"]}
        for item in self.static:
            if _distance_to_polyline(x, y, item["points"]) <= item["r"] + 0.05:
                return {"kind": "static", "id": item["id"]}
        for item in self.poly_obstacles:
            if _point_in_polygon(x, y, item["points"]):
                return {"kind": "obstacle", "id": item["id"]}
        for item in self.dynamic:
            if math.hypot(x - item["x"], y - item["y"]) <= item["r"] + 0.05:
                return {"kind": "dynamic", "id": item["id"]}
        for item in self.docks:
            if math.hypot(x - item["x"], y - item["y"]) <= item["r"] + 0.05:
                return {"kind": "dock", "id": item["id"]}
        for item in self.passages:
            if _distance_to_polyline(x, y, item["points"]) <= item["half"] + 0.10:
                return {"kind": "passage", "id": item["id"]}
        for item in reversed(self.regions):
            if _point_in_polygon(x, y, item["points"]):
                return {"kind": "region", "id": item["id"]}
        return None

    # ------------------------------------------------------------- 栅格化
    def content_bounds(self, margin=CROP_MARGIN):
        """所有元素的联合外接框（对齐到栅格）：保存时按它裁剪地图。

        编辑期间界面每 100 ms 拉一次 `/api/state`，草稿里带上 bounds，
        涂抹几千个点时逐点拼两个大列表要 10 ms/次；改成直接累计 min/max
        并按"改动计数 + 列表对象"缓存结果。
        """
        cell = world.CELL
        token = (
            self._paint_rev,
            id(self.static),
            id(self.poly_obstacles),
            id(self.regions),
            id(self.passages),
            id(self.docks),
            id(self.no_gos),
            id(self.dynamic),
            self.width,
            self.height,
            margin,
        )
        cached = self._bounds_cache
        if cached and cached[0] == token:
            return cached[1]
        xs = []
        ys = []

        def add(x, y, pad=0.0):
            xs.append(x - pad)
            xs.append(x + pad)
            ys.append(y - pad)
            ys.append(y + pad)

        for region in self.regions:
            for x, y in region["points"]:
                add(x, y)
        for corridor in self.passages:
            for x, y in corridor["points"]:
                add(x, y, corridor["half"])
        for item in self.static:
            for x, y in item["points"]:
                add(x, y, item["r"])
        for item in self.poly_obstacles:
            for x, y in item["points"]:
                add(x, y)
        for item in self.no_gos:
            add(item["x0"], item["y0"])
            add(item["x1"], item["y1"])
        for item in self.docks:
            add(item["x"], item["y"], item["r"])
        for item in self.dynamic:
            add(item["x"], item["y"], item["r"])
            add(item["ax"], item["ay"], item["r"])
            add(item["bx"], item["by"], item["r"])
        if not xs or not ys:
            result = (0.0, 0.0, self.width, self.height)
            self._bounds_cache = (token, result)
            return result
        x0 = max(0.0, min(xs) - margin)
        y0 = max(0.0, min(ys) - margin)
        x1 = min(self.width, max(xs) + margin)
        y1 = min(self.height, max(ys) + margin)
        x0 = math.floor(x0 / cell) * cell
        y0 = math.floor(y0 / cell) * cell
        x1 = math.ceil(x1 / cell) * cell
        y1 = math.ceil(y1 / cell) * cell
        if x1 - x0 < cell:
            x1 = min(self.width, x0 + cell)
        if y1 - y0 < cell:
            y1 = min(self.height, y0 + cell)
        result = (round(x0, 4), round(y0, 4), round(x1, 4), round(y1, 4))
        self._bounds_cache = (token, result)
        return result

    def rasterize(self, bounds=None):
        """把区域 + 通道刷成"可走掩膜"，障碍与禁区从里面挖掉。

        返回 ``(width, height, mask, ox, oy)``：掩膜按 ``bounds``（米）裁剪过，
        世界坐标 ``(x, y)`` 对应格 ``((x-ox)/cell, (y-oy)/cell)``。
        """
        cell = world.CELL
        x0, y0, x1, y1 = self.content_bounds() if bounds is None else bounds
        width = max(1, int(round((x1 - x0) / cell)))
        height = max(1, int(round((y1 - y0) / cell)))
        mask = bytearray(width * height)

        def stamp_disc(cx, cy, radius, value):
            col0 = max(0, int((cx - radius - x0) / cell))
            col1 = min(width - 1, int((cx + radius - x0) / cell))
            row0 = max(0, int((cy - radius - y0) / cell))
            row1 = min(height - 1, int((cy + radius - y0) / cell))
            for row in range(row0, row1 + 1):
                y = y0 + (row + 0.5) * cell
                base = row * width
                for col in range(col0, col1 + 1):
                    x = x0 + (col + 0.5) * cell
                    if math.hypot(x - cx, y - cy) <= radius:
                        mask[base + col] = value

        def fill_polygon(points, value):
            count = len(points)
            if count < 3:
                return
            ys = [point[1] for point in points]
            row0 = max(0, int((min(ys) - y0) / cell))
            row1 = min(height - 1, int((max(ys) - y0) / cell))
            for row in range(row0, row1 + 1):
                y = y0 + (row + 0.5) * cell
                crossings = []
                for index in range(count):
                    ax, ay = points[index]
                    bx, by = points[(index + 1) % count]
                    if (ay > y) == (by > y):
                        continue
                    crossings.append(ax + (bx - ax) * (y - ay) / (by - ay + 1e-12))
                if not crossings:
                    continue
                crossings.sort()
                base = row * width
                for index in range(0, len(crossings) - 1, 2):
                    col0 = max(0, int(math.ceil((crossings[index] - x0) / cell - 0.5)))
                    col1 = min(
                        width - 1,
                        int(math.floor((crossings[index + 1] - x0) / cell - 0.5)),
                    )
                    for col in range(col0, col1 + 1):
                        mask[base + col] = value

        for region in self.regions:
            fill_polygon(region["points"], 1)

        for corridor in self.passages:
            points = corridor["points"]
            step = cell * 0.5
            half = corridor["half"]
            for index in range(len(points) - 1):
                ax, ay = points[index]
                bx, by = points[index + 1]
                length = math.hypot(bx - ax, by - ay)
                count = max(1, int(length / step))
                for step_index in range(count + 1):
                    t = step_index / float(count)
                    stamp_disc(ax + (bx - ax) * t, ay + (by - ay) * t, half, 1)
            stamp_disc(points[0][0], points[0][1], half, 1)
            stamp_disc(points[-1][0], points[-1][1], half, 1)

        # 障碍与禁区不挖进掩膜：它们在导出的地图定义里是显式几何（实体障碍 /
        # 禁区层），挖洞只会让"读进来再存一次"的障碍越描越粗。
        return width, height, mask, x0, y0

    def region_cells(self, width, mask, ox=0.0, oy=0.0, height=None):
        """每个区域在栅格里的代表格（取区域内部的格）。"""
        reps = []
        rows = height if height else max(1, len(mask) // max(1, width))
        cell = world.CELL
        for region in self.regions:
            points = region["points"]
            cx = sum(point[0] for point in points) / len(points)
            cy = sum(point[1] for point in points) / len(points)
            col = min(width - 1, max(0, int((cx - ox) / cell)))
            row = min(rows - 1, max(0, int((cy - oy) / cell)))
            index = row * width + col
            if not mask[index]:
                index = None
                for offset in range(1, 40):
                    done = False
                    for dc in range(-offset, offset + 1):
                        for dr in range(-offset, offset + 1):
                            c2 = col + dc
                            r2 = row + dr
                            if 0 <= c2 < width and 0 <= r2 < rows:
                                candidate = r2 * width + c2
                                if candidate < len(mask) and mask[candidate]:
                                    index = candidate
                                    done = True
                                    break
                        if done:
                            break
                    if done:
                        break
            reps.append({"region": region["id"], "index": index})
        return reps

    def connectivity(self):
        """按栅格连通性判断"所有区域是否都被通道连在一起"。"""
        width, height, mask, ox, oy = self.rasterize()
        reps = self.region_cells(width, mask, ox, oy, height)
        if not self.regions:
            return True, reps
        seed = None
        for rep in reps:
            if rep["index"] is not None:
                seed = rep["index"]
                break
        if seed is None:
            return False, reps
        seen = bytearray(len(mask))
        stack = [seed]
        seen[seed] = 1
        while stack:
            current = stack.pop()
            col = current % width
            row = current // width
            for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                c2 = col + dc
                r2 = row + dr
                if c2 < 0 or c2 >= width or r2 < 0 or r2 >= height:
                    continue
                index = r2 * width + c2
                if seen[index] or not mask[index]:
                    continue
                seen[index] = 1
                stack.append(index)
        for rep in reps:
            if rep["index"] is None or not seen[rep["index"]]:
                return False, reps
        return True, reps

    def passage_links(self):
        """通道实际连起来的区域对：看通道两端落在哪个区域里（不信任可能过期的 id）。"""
        links = []
        for corridor in self.passages:
            points = corridor.get("points") or []
            if len(points) < 2:
                continue
            start = self.region_at(points[0][0], points[0][1])
            end = self.region_at(points[-1][0], points[-1][1])
            if start is None or end is None:
                continue
            if start["id"] == end["id"]:
                continue
            links.append((start["id"], end["id"]))
        return links

    def unlinked_regions(self):
        """没被通道连到"第一个区域"上的区域编号（区域挨着但没有通道也算没连）。"""
        ids = [region["id"] for region in self.regions]
        if len(ids) < 2:
            return []
        parent = {rid: rid for rid in ids}

        def find(rid):
            while parent[rid] != rid:
                parent[rid] = parent[parent[rid]]
                rid = parent[rid]
            return rid

        for a, b in self.passage_links():
            root_a, root_b = find(a), find(b)
            if root_a != root_b:
                parent[root_a] = root_b
        root = find(ids[0])
        return [rid for rid in ids if find(rid) != root]

    # ------------------------------------------------------------- 校验 / 导出
    def problems(self):
        """保存前的校验项。

        连通性只看通道图（区域数 + 通道数，几十次点在多边形内判断）；
        以前这里有一步"把整张草稿栅格化 + 全图洪水填充"（`connectivity()`），
        编辑期间界面每 100 ms 拉一次 `/api/state`，每次都要跑两遍、在 100 m
        空白场地上要 40~60 ms，一直占着 GIL，仿真线程被拖得走不动、手动控制发卡。
        通道本身就是刷进掩膜里的，通道图连通即掩膜连通，那一步是多余的。
        """
        issues = []
        if not self.regions:
            issues.append("至少需要一个区域")
        if not self.docks:
            issues.append("至少需要一个充电桩")
        if len(self.regions) > 1:
            # 只有"通道"才算连通：两个区域挨在一起（掩膜连成一片）不算
            stranded = self.unlinked_regions()
            if stranded:
                issues.append(
                    "区域 {} 还没有用通道连通（区域挨着不算，需要画一条通道）".format(
                        "、".join("#{}".format(rid) for rid in stranded)
                    )
                )
        return issues

    def can_save(self):
        return not self.problems()

    def to_definition(self, name=None):
        issues = self.problems()
        if issues:
            raise ValueError("；".join(issues))
        if not self.regions:
            raise EditorError("PATH_EMPTY_WHEN_SAVE_MAP")
        area = sum(float(region.get("area") or 0.0) for region in self.regions)
        if area < 1.0:
            raise EditorError("TOO_FEW_POINTS_BEFORE_SAVE_MAP")
        try:
            width, height, mask, ox, oy = self.rasterize()
        except (ValueError, IndexError, TypeError) as error:
            raise EditorError("WRONG_DATA_STRUCT", str(error))
        if not any(mask):
            raise EditorError("WRONG_DATA_STRUCT", "栅格化之后没有任何可通行区域")
        map_name = str(name or self.name or "自定义地图").strip() or "自定义地图"
        definition = map_import.build_map_from_mask(
            width,
            height,
            mask,
            map_name,
            max_extent=max(self.width * 4.0, 200.0),
            source_path="editor:{}".format(map_name),
        )
        # 保留使用者起的名字（导入器会给 PGM 地图统一加"·实景"后缀）
        definition["name"] = map_name
        definition["editor_built"] = True
        padding = map_import.PADDING

        def tx(x):
            return round(x - ox + padding, 4)

        def ty(y):
            return round(y - oy + padding, 4)

        # 用使用者自己放的充电桩/起点（_definition 里的自动选点只是兜底）
        dock = self.docks[0]
        dock_x = tx(dock["x"])
        dock_y = ty(dock["y"])
        definition["dock"] = {"x": dock_x, "y": dock_y, "r": round(dock["r"], 3)}
        definition["start"] = {
            "x": round(min(definition["width"] - 0.6, dock_x + 0.9), 4),
            "y": dock_y,
            "theta": 0.0,
        }
        definition["no_go"] = [
            {
                "id": index + 1,
                "x0": tx(item["x0"]),
                "y0": ty(item["y0"]),
                "x1": tx(item["x1"]),
                "y1": ty(item["y1"]),
            }
            for index, item in enumerate(self.no_gos)
        ]
        definition["dynamic"] = [
            {
                "id": index + 1,
                "x": tx(item["x"]),
                "y": ty(item["y"]),
                "r": item["r"],
                "ax": tx(item["ax"]),
                "ay": ty(item["ay"]),
                "bx": tx(item["bx"]),
                "by": ty(item["by"]),
                "span": item["span"],
                "heading": item["heading"],
                "speed": item.get("speed", world.DYNAMIC_SPEED),
            }
            for index, item in enumerate(self.dynamic)
        ]
        # 区域多边形：保存下来，下次"载入编辑"才能恢复成多个区域（
        # 只靠 polygon 会把用通道连起来的多个区域合成一大块）
        definition["regions"] = [
            {
                "id": index + 1,
                "points": [[tx(x), ty(y)] for x, y in item["points"]],
                "area": round(item["area"], 2),
            }
            for index, item in enumerate(self.regions)
        ]

        # 通道：只通行、不割草。要单独存下来，仿真侧据此刷 no_mow 层
        definition["passages"] = [
            {
                "id": index + 1,
                "points": [[tx(x), ty(y)] for x, y in item["points"]],
                "half": item["half"],
                "from": item.get("from"),
                "to": item.get("to"),
            }
            for index, item in enumerate(self.passages)
        ]

        # 涂抹出来的障碍物直接用圆轮廓表示、读进来的多边形障碍原样保留：
        # 掩膜里挖掉的洞小于 MIN_HOLE_AREA 时不会生成多边形的洞，会被丢掉。
        painted = []
        # 涂抹的笔迹先合并成"外围轮廓"（同一片涂抹只留最外圈），
        # 保存出来的障碍物是实心块，内部不再有一条条笔触
        for loop in self.brush_outlines():
            outline = [(tx(x), ty(y)) for x, y in loop]
            area = abs(_polygon_area(outline)) if len(outline) >= 3 else 0.0
            if area < 0.05:
                continue
            painted.append(
                {
                    "kind": "large" if area >= 1.5 else "small",
                    "parts": [],
                    "outline": outline,
                    "area": round(area, 2),
                }
            )
        for item in self.poly_obstacles:
            painted.append(
                {
                    "kind": "large" if item["area"] >= 1.5 else "small",
                    "parts": [],
                    "outline": [[tx(x), ty(y)] for x, y in item["points"]],
                    "area": round(item["area"], 2),
                }
            )
        holes = []
        for shape in definition.get("obstacles") or []:
            outline = shape.get("outline") or []
            if not outline:
                continue
            cx = sum(point[0] for point in outline) / len(outline)
            cy = sum(point[1] for point in outline) / len(outline)
            if any(
                _point_in_polygon(cx, cy, [(tx(x), ty(y)) for x, y in loop])
                for loop in self.brush_outlines()
            ):
                continue
            holes.append(shape)
        definition["obstacles"] = painted + holes

        meta = definition.setdefault("import_meta", {})
        meta["editor"] = True
        meta["regions"] = len(self.regions)
        meta["passages"] = len(self.passages)
        meta["docks"] = len(self.docks)
        meta["crop"] = [round(ox, 3), round(oy, 3)]
        return definition

    # ------------------------------------------------------------- 笔迹轮廓
    def brush_outlines(self):
        """把实心画笔的笔迹合成"外围轮廓"：同一片涂抹只留最外圈的闭合多边形。

        这样保存出来的障碍物是一个实心块（内部没有一条条笔触边界），
        反复涂抹也不会越描越碎。结果按笔迹指纹缓存。
        """
        strokes = [item for item in self.static if item.get("kind") == "brush"]
        # 便宜的有效性判断：每次改动都会走 _push_history（计数 +1），撤销/载入会
        # 换掉列表对象（id 变），不需要再把所有点拼进键里（涂抹多时一次几毫秒）
        key = (self._paint_rev, id(self.static), id(self.poly_obstacles))
        if self._brush_cache_key == key:
            return [list(loop) for loop in self._brush_cache]
        if not strokes:
            self._brush_cache_key = key
            self._brush_cache = []
            return []
        cell = world.CELL
        xs = []
        ys = []
        for item in strokes:
            for x, y in item["points"]:
                xs.extend((x - item["r"], x + item["r"]))
                ys.extend((y - item["r"], y + item["r"]))
        ox = math.floor(min(xs) / cell) * cell
        oy = math.floor(min(ys) / cell) * cell
        width = max(1, int(math.ceil((max(xs) - ox) / cell)) + 1)
        height = max(1, int(math.ceil((max(ys) - oy) / cell)) + 1)
        mask = bytearray(width * height)
        # 每条笔迹按"胶囊"（线段 + 半径）刷进掩膜
        for item in strokes:
            radius = item["r"]
            pts = item["points"]
            samples = []
            if len(pts) == 1:
                samples.append(pts[0])
            else:
                for index in range(len(pts) - 1):
                    ax, ay = pts[index]
                    bx, by = pts[index + 1]
                    length = math.hypot(bx - ax, by - ay)
                    count = max(1, int(length / (cell * 0.5)))
                    for k in range(count + 1):
                        t = k / float(count)
                        samples.append((ax + (bx - ax) * t, ay + (by - ay) * t))
            for sx, sy in samples:
                col0 = max(0, int((sx - radius - ox) / cell))
                col1 = min(width - 1, int((sx + radius - ox) / cell))
                row0 = max(0, int((sy - radius - oy) / cell))
                row1 = min(height - 1, int((sy + radius - oy) / cell))
                for row in range(row0, row1 + 1):
                    cy = oy + (row + 0.5) * cell
                    base = row * width
                    for col in range(col0, col1 + 1):
                        cx = ox + (col + 0.5) * cell
                        if math.hypot(cx - sx, cy - sy) <= radius:
                            mask[base + col] = 1
        loops = []
        for loop in _trace_mask_loops(width, height, mask, ox, oy, cell):
            simplified = _simplify_ring_closed(loop, 0.04)
            if len(simplified) >= 3 and abs(_polygon_area(simplified)) > 0.02:
                loops.append(simplified)
        # 只保留最外圈的几个大环：内部孔洞不进地图（障碍物按实心处理）
        loops.sort(key=lambda loop: abs(_polygon_area(loop)), reverse=True)
        biggest = abs(_polygon_area(loops[0])) if loops else 0.0
        loops = [
            loop for loop in loops if abs(_polygon_area(loop)) >= max(0.05, biggest * 0.25)
        ]
        self._brush_cache_key = key
        self._brush_cache = loops
        return [list(loop) for loop in loops]

    # ------------------------------------------------------------- 读入已有地图
    def load_from_definition(self, definition, name=None, scale=1.0):
        """把一张已有地图的几何读进草稿，用来在其基础上改。

        导入的地图坐标整体加过 ``map_import.PADDING``，这里减回去；
        非 native 的内置地图在仿真里会乘一次 ``world.MAP_SCALE``，
        这里按同一个比例换算，编辑后保存出来尺寸才和原图一致。
        """
        if not definition:
            raise EditorError("LOAD_MAP_FAIL", "没有找到要编辑的地图")
        padding = map_import.PADDING if definition.get("imported") else 0.0
        scale = float(scale) if scale else 1.0

        def px(x):
            return round((float(x) - padding) * scale, _ROUND)

        def py(y):
            return round((float(y) - padding) * scale, _ROUND)

        self.regions = []
        self.passages = []
        self.docks = []
        self.no_gos = []
        self.static = []
        self.poly_obstacles = []
        self.dynamic = []
        self._next_id = 1

        stored = definition.get("regions") or []
        if stored:
            max_id = 0
            for item in stored:
                points = [(px(x), py(y)) for x, y in item.get("points") or []]
                if len(points) < 3:
                    continue
                region_id = int(item.get("id") or 0)
                max_id = max(max_id, region_id)
                self.regions.append(
                    {
                        "id": region_id or self._id(),
                        "kind": "region",
                        "points": points,
                        "area": round(_polygon_area(points), 2),
                    }
                )
            self._next_id = max(self._next_id, max_id + 1)
        else:
            polygon = [
                point for point in definition.get("polygon") or [] if len(point) >= 2
            ]
            if len(polygon) >= 3:
                points = [(px(x), py(y)) for x, y in polygon]
                self.regions.append(
                    {
                        "id": self._id(),
                        "kind": "region",
                        "points": points,
                        "area": round(_polygon_area(points), 2),
                    }
                )
        for shape in definition.get("obstacles") or []:
            outline = shape.get("outline") or []
            if len(outline) < 3:
                continue
            try:
                self.add_obstacle_polygon([(px(x), py(y)) for x, y in outline])
            except ValueError:
                continue
        dock = definition.get("dock")
        if dock:
            self.docks.append(
                {
                    "id": self._id(),
                    "kind": "dock",
                    "x": px(dock["x"]),
                    "y": py(dock["y"]),
                    "r": round(float(dock.get("r", DOCK_RADIUS)), _ROUND),
                }
            )
        for zone in definition.get("no_go") or []:
            if "x0" not in zone:
                continue
            self.no_gos.append(
                {
                    "id": self._id(),
                    "kind": "no_go",
                    "x0": px(zone["x0"]),
                    "y0": py(zone["y0"]),
                    "x1": px(zone["x1"]),
                    "y1": py(zone["y1"]),
                }
            )
        for corridor in definition.get("passages") or []:
            points = [(px(x), py(y)) for x, y in corridor.get("points") or []]
            if len(points) < 2:
                continue
            self.passages.append(
                {
                    "id": self._id(),
                    "kind": "passage",
                    "points": points,
                    "half": round(float(corridor.get("half", PASSAGE_HALF_WIDTH)), _ROUND),
                    "from": corridor.get("from"),
                    "to": corridor.get("to"),
                }
            )
        for item in definition.get("dynamic") or []:
            try:
                added = self.add_dynamic(
                    px(item.get("x", 0.0)),
                    py(item.get("y", 0.0)),
                    item.get("r"),
                    item.get("span"),
                    item.get("heading", 0.0),
                    px(item["ax"]),
                    py(item["ay"]),
                    px(item["bx"]),
                    py(item["by"]),
                )
            except (KeyError, ValueError):
                continue
            if item.get("speed"):
                added["speed"] = round(float(item["speed"]), 3)
        # 画布至少 100 × 100 m，比画布还大的图要把画布放大到装得下
        if self.regions:
            xs = [point[0] for region in self.regions for point in region["points"]]
            ys = [point[1] for region in self.regions for point in region["points"]]
            self.width = max(DEFAULT_WIDTH, math.ceil(max(xs) + CROP_MARGIN))
            self.height = max(DEFAULT_HEIGHT, math.ceil(max(ys) + CROP_MARGIN))
        # 把读进来的图整体挪到空白画布中央（不然图会贴在左下角，
        # 画布大半是空的，画布缩放也跟着别扭）
        bounds = self.content_bounds()
        dx = self.width / 2.0 - (bounds[0] + bounds[2]) / 2.0
        dy = self.height / 2.0 - (bounds[1] + bounds[3]) / 2.0
        if abs(dx) > 1e-6 or abs(dy) > 1e-6:
            self._translate(dx, dy)
        self._history = []
        if name:
            self.name = str(name).strip() or self.name
        else:
            self.name = str(definition.get("name") or self.name)
        return self

    def state(self):
        # 校验只算一次：以前 problems() 被 state() 和 can_save() 各调一遍，
        # 编辑期间每秒十几次、每次都跑栅格化+洪水填充
        issues = self.problems()
        return {
            "active": True,
            "blank": True,
            "width": self.width,
            "height": self.height,
            "name": self.name,
            "map_id": self.map_id,
            "regions": [dict(item) for item in self.regions],
            "passages": [dict(item) for item in self.passages],
            "docks": [dict(item) for item in self.docks],
            "no_gos": [dict(item) for item in self.no_gos],
            # 笔迹只给界面"列表 + 轮廓"用，原始点阵每次几百 KB，不再整包下发
            "static": [
                {
                    "id": item["id"],
                    "kind": item.get("kind"),
                    "r": item["r"],
                    "length": item.get("length"),
                    "point_count": len(item.get("points") or ()),
                }
                for item in self.static
            ],
            "brush_outlines": self.brush_outlines(),
            "poly_obstacles": [dict(item) for item in self.poly_obstacles],
            "dynamic": [dict(item) for item in self.dynamic],
            "bounds": list(self.content_bounds()),
            "can_undo": self.can_undo,
            "problems": issues,
            "can_save": not issues,
        }


def blank_field_definition(
    width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT, name="空白地图", start=None
):
    """编辑期间的临时场地：一块没有任何墙体的大空地。

    ``start`` 可以把小车放到指定位置（载入已有地图时放在图旁边，
    免得自动缩放到草稿后车跑到画布外面去了）。
    """
    sx, sy = (width * 0.5, height * 0.5) if start is None else (start[0], start[1])
    sx = max(1.0, min(width - 1.0, float(sx)))
    sy = max(1.0, min(height - 1.0, float(sy)))
    return {
        "id": None,
        "name": name,
        "native": True,
        "imported": True,
        "editor_built": True,
        "editor_draft": True,
        "source_path": "editor:draft",
        "width": width,
        "height": height,
        "polygon": [
            (0.0, 0.0),
            (width, 0.0),
            (width, height),
            (0.0, height),
        ],
        "obstacles": [],
        "dock": {"x": sx, "y": sy, "r": DOCK_RADIUS},
        "start": {"x": min(width - 0.6, sx + 0.9), "y": sy, "theta": 0.0},
    }
