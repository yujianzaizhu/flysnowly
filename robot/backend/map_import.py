"""把真实 PGM 栅格地图转换成控制台可直接运行的模拟场地。

支持目录（包含 grid_map.pgm / path/default.json / map_info.json）或单个
PGM 文件。只依赖标准库，读取 P5/P2，提取最大连通自由区外轮廓与内部
障碍孔洞，再归一化到仿真坐标。
"""

import json
import math
import os

import world

FREE_PIXEL = 128
MAX_VERTEX_TOLERANCE = 0.13
MIN_HOLE_AREA = 0.5
PADDING = 0.20


class MapImportError(ValueError):
    pass


def _read_pgm(path):
    with open(path, "rb") as handle:
        data = handle.read()
    if len(data) < 8:
        raise MapImportError("PGM 文件为空或损坏")
    cursor = 0
    tokens = []
    magic = None
    while len(tokens) < 3:
        while cursor < len(data) and data[cursor] in b" \t\r\n":
            cursor += 1
        if cursor >= len(data):
            raise MapImportError("PGM 头部不完整")
        if data[cursor : cursor + 1] == b"#":
            while cursor < len(data) and data[cursor] not in b"\r\n":
                cursor += 1
            continue
        start = cursor
        while cursor < len(data) and data[cursor] not in b" \t\r\n":
            cursor += 1
        token = data[start:cursor].decode("ascii", "replace")
        if magic is None:
            magic = token
        else:
            try:
                tokens.append(int(token))
            except ValueError:
                raise MapImportError("PGM 头部数字无效: {}".format(token))
    if magic not in ("P5", "P2"):
        raise MapImportError("仅支持 P5/P2 PGM 地图")
    width, height, max_value = tokens
    if width <= 0 or height <= 0 or max_value <= 0:
        raise MapImportError("PGM 尺寸无效")
    if magic == "P5":
        # 头字段后只允许一个空白分隔符，像素数据可能紧跟其后。
        if cursor < len(data) and data[cursor] in b" \t\r\n":
            cursor += 1
        raw = data[cursor : cursor + width * height]
        if len(raw) != width * height:
            raise MapImportError("PGM 像素数据不足")
        if max_value <= 255:
            return width, height, raw
        values = bytearray(width * height)
        for index in range(width * height):
            values[index] = (
                int.from_bytes(raw[index * 2 : index * 2 + 2], "big") * 255 // max_value
            )
        return width, height, bytes(values)
    values = []
    while len(values) < width * height:
        while cursor < len(data) and data[cursor] in b" \t\r\n":
            cursor += 1
        if cursor >= len(data):
            break
        if data[cursor : cursor + 1] == b"#":
            while cursor < len(data) and data[cursor] not in b"\r\n":
                cursor += 1
            continue
        start = cursor
        while cursor < len(data) and data[cursor] not in b" \t\r\n":
            cursor += 1
        try:
            values.append(int(data[start:cursor]) * 255 // max_value)
        except ValueError:
            raise MapImportError("PGM 像素值无效")
    if len(values) != width * height:
        raise MapImportError("PGM 像素数据不足")
    return width, height, bytes(bytearray(values))


def _metadata(source):
    if os.path.isdir(source):
        root = source
        candidates = ("grid_map.pgm", "room_map.pgm", "default.pgm")
        map_file = None
        for name in candidates:
            candidate = os.path.join(root, name)
            if os.path.isfile(candidate):
                map_file = candidate
                break
        if map_file is None:
            pgms = sorted(
                name
                for name in os.listdir(root)
                if name.lower().endswith(".pgm")
                and not name.startswith("reachable")
            )
            if not pgms:
                raise MapImportError("目录中没有 PGM 地图: {}".format(root))
            map_file = os.path.join(root, pgms[0])
    else:
        map_file = source
        root = os.path.dirname(os.path.abspath(source))
    if not os.path.isfile(map_file):
        raise MapImportError("地图文件不存在: {}".format(map_file))
    if not map_file.lower().endswith(".pgm"):
        raise MapImportError("地图文件必须是 .pgm: {}".format(map_file))

    info = {}
    info_path = os.path.join(root, "map_info.json")
    if os.path.isfile(info_path):
        try:
            with open(info_path, "r", encoding="utf-8") as handle:
                info = json.load(handle)
        except (OSError, ValueError):
            info = {}
    path_info = {}
    default_path = os.path.join(root, "path", "default.json")
    if os.path.isfile(default_path):
        try:
            with open(default_path, "r", encoding="utf-8") as handle:
                path_info = json.load(handle)
        except (OSError, ValueError):
            path_info = {}
    return map_file, root, info, path_info


def _number(mapping, key, default):
    try:
        return float(mapping.get(key, default))
    except (TypeError, ValueError):
        return float(default)


def _resample(width, height, raw, resolution):
    ratio = world.CELL / resolution
    out_width = max(1, int(round(width / ratio)))
    out_height = max(1, int(round(height / ratio)))
    mask = bytearray(out_width * out_height)
    for row in range(out_height):
        sy0 = int(math.floor(row * ratio))
        sy1 = max(sy0 + 1, int(math.floor((row + 1) * ratio)))
        sy1 = min(height, sy1)
        for col in range(out_width):
            sx0 = int(math.floor(col * ratio))
            sx1 = max(sx0 + 1, int(math.floor((col + 1) * ratio)))
            sx1 = min(width, sx1)
            total = 0
            free = 0
            for sy in range(sy0, sy1):
                base = sy * width
                for sx in range(sx0, sx1):
                    total += 1
                    if raw[base + sx] >= FREE_PIXEL:
                        free += 1
            if total and free * 2 >= total:
                mask[row * out_width + col] = 1
    return out_width, out_height, mask


def _largest_component(width, height, mask):
    seen = bytearray(width * height)
    best = []
    for start in range(width * height):
        if not mask[start] or seen[start]:
            continue
        seen[start] = 1
        stack = [start]
        component = []
        while stack:
            index = stack.pop()
            component.append(index)
            col = index % width
            row = index // width
            if col > 0:
                nxt = index - 1
                if mask[nxt] and not seen[nxt]:
                    seen[nxt] = 1
                    stack.append(nxt)
            if col + 1 < width:
                nxt = index + 1
                if mask[nxt] and not seen[nxt]:
                    seen[nxt] = 1
                    stack.append(nxt)
            if row > 0:
                nxt = index - width
                if mask[nxt] and not seen[nxt]:
                    seen[nxt] = 1
                    stack.append(nxt)
            if row + 1 < height:
                nxt = index + width
                if mask[nxt] and not seen[nxt]:
                    seen[nxt] = 1
                    stack.append(nxt)
        if len(component) > len(best):
            best = component
    if not best:
        raise MapImportError("PGM 中没有可通行自由区域")
    result = bytearray(width * height)
    for index in best:
        result[index] = 1
    return result, best


def _boundary_loops(width, height, mask):
    """把自由栅格边界拆成有向边并串成闭合环，内部保持在左侧。"""
    stride = width + 1
    outgoing = {}

    def add(start, end):
        outgoing.setdefault(start, []).append(end)

    for row in range(height):
        for col in range(width):
            index = row * width + col
            if not mask[index]:
                continue
            if row == 0 or not mask[index - width]:
                add(row * stride + col, row * stride + col + 1)
            if col + 1 == width or not mask[index + 1]:
                add(row * stride + col + 1, (row + 1) * stride + col + 1)
            if row + 1 == height or not mask[index + width]:
                add((row + 1) * stride + col + 1, (row + 1) * stride + col)
            if col == 0 or not mask[index - 1]:
                add((row + 1) * stride + col, row * stride + col)

    loops = []
    while outgoing:
        start = next(iter(outgoing))
        current = start
        previous = None
        loop = [current]
        guard = 0
        while True:
            ends = outgoing.get(current)
            if not ends:
                break
            if len(ends) == 1 or previous is None:
                nxt = ends.pop(0)
            else:
                px, py = previous % stride, previous // stride
                cx, cy = current % stride, current // stride
                base = math.atan2(cy - py, cx - px)
                best = None
                for end_index, end in enumerate(ends):
                    ex, ey = end % stride, end // stride
                    turn = (
                        math.atan2(ey - cy, ex - cx) - base + math.pi
                    ) % (2.0 * math.pi) - math.pi
                    if best is None or turn > best[0]:
                        best = (turn, end_index)
                nxt = ends.pop(best[1])
            if not outgoing.get(current):
                outgoing.pop(current, None)
            previous, current = current, nxt
            loop.append(current)
            guard += 1
            if current == start or guard > width * height * 4:
                break
        if len(loop) > 3:
            loops.append(loop)
    return loops


def _polygon_area(points):
    area = 0.0
    for index, point in enumerate(points):
        nxt = points[(index + 1) % len(points)]
        area += point[0] * nxt[1] - nxt[0] * point[1]
    return area * 0.5


def _simplify(points, tolerance=MAX_VERTEX_TOLERANCE):
    if len(points) <= 4:
        return points
    if points[0] == points[-1]:
        points = points[:-1]
    if len(points) <= 4:
        return points

    def rdp(part):
        if len(part) < 3:
            return part
        ax, ay = part[0]
        bx, by = part[-1]
        dx, dy = bx - ax, by - ay
        length_sq = dx * dx + dy * dy
        best = -1.0
        best_index = 0
        for index in range(1, len(part) - 1):
            x, y = part[index]
            if length_sq <= 1e-12:
                distance = math.hypot(x - ax, y - ay)
            else:
                t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / length_sq))
                distance = math.hypot(x - (ax + dx * t), y - (ay + dy * t))
            if distance > best:
                best = distance
                best_index = index
        if best <= tolerance:
            return [part[0], part[-1]]
        left = rdp(part[: best_index + 1])
        right = rdp(part[best_index:])
        return left[:-1] + right

    first = points[0]
    split = max(
        range(1, len(points)),
        key=lambda index: math.hypot(
            points[index][0] - first[0], points[index][1] - first[1]
        ),
    )
    part_a = rdp(points[: split + 1])
    part_b = rdp(points[split:] + [points[0]])
    return part_a[:-1] + part_b[:-1]


def _clearance_map(width, height, mask):
    """每个自由栅格到最近非自由栅格的曼哈顿距离（栅格数）。"""
    from collections import deque

    distance = [0] * (width * height)
    queue = deque()
    for index in range(width * height):
        if not mask[index]:
            queue.append(index)
    while queue:
        index = queue.popleft()
        col = index % width
        row = index // width
        value = distance[index] + 1
        if col > 0:
            nxt = index - 1
            if mask[nxt] and not distance[nxt]:
                distance[nxt] = value
                queue.append(nxt)
        if col + 1 < width:
            nxt = index + 1
            if mask[nxt] and not distance[nxt]:
                distance[nxt] = value
                queue.append(nxt)
        if row > 0:
            nxt = index - width
            if mask[nxt] and not distance[nxt]:
                distance[nxt] = value
                queue.append(nxt)
        if row + 1 < height:
            nxt = index + width
            if mask[nxt] and not distance[nxt]:
                distance[nxt] = value
                queue.append(nxt)
    return distance


def _choose_pose(mask, width, height):
    center_col = width / 2.0
    center_row = height / 2.0
    free = [index for index in range(width * height) if mask[index]]
    if not free:
        raise MapImportError("自由区域为空")
    distance = _clearance_map(width, height, mask)
    clearance = max(3, int(round(0.32 / world.CELL)) + 1)
    candidates = [index for index in free if distance[index] >= clearance]
    pool = candidates or free
    dock_index = min(
        pool,
        key=lambda index: (index % width - center_col) ** 2
        + (index // width - center_row) ** 2,
    )
    dock_col = dock_index % width
    dock_row = dock_index // width

    start_index = None
    start_distance = min(20, max(6, int(min(width, height) * 0.12)))
    for distance_cells in (
        start_distance,
        max(6, start_distance - 4),
        max(5, start_distance - 8),
        6,
        5,
        4,
    ):
        for dc, dr in (
            (distance_cells, 0),
            (-distance_cells, 0),
            (0, distance_cells),
            (0, -distance_cells),
        ):
            col, row = dock_col + dc, dock_row + dr
            if 0 <= col < width and 0 <= row < height:
                index = row * width + col
                if mask[index] and distance[index] >= max(2, clearance - 1):
                    start_index = index
                    break
        if start_index is not None:
            break
    if start_index is None:
        start_index = dock_index
    theta = math.atan2(
        (start_index // width) - dock_row, (start_index % width) - dock_col
    )
    return dock_col, dock_row, start_index % width, start_index // width, theta


def build_map(source, name=None, max_extent=24.0):
    """解析真实地图并返回 world.register_map 可用的定义。"""
    source = os.path.abspath(os.path.expanduser(str(source)))
    map_file, root, map_info, path_info = _metadata(source)
    resolution = _number(path_info, "resolution", 0.05)
    if not (0.005 <= resolution <= 1.0):
        raise MapImportError("地图分辨率无效: {}".format(resolution))
    native_width, native_height, raw = _read_pgm(map_file)
    width, height, mask = _resample(native_width, native_height, raw, resolution)
    component, component_cells = _largest_component(width, height, mask)
    loops = _boundary_loops(width, height, component)
    if not loops:
        raise MapImportError("无法提取地图边界")

    stride = width + 1
    converted = []
    for loop in loops:
        points = [
            (index % stride * world.CELL, index // stride * world.CELL)
            for index in loop
        ]
        points = _simplify(points)
        area = _polygon_area(points)
        if abs(area) > 0.01:
            converted.append((area, points))
    if not converted:
        raise MapImportError("地图边界面积过小")
    converted.sort(key=lambda item: abs(item[0]), reverse=True)
    outer = converted[0][1]
    if converted[0][0] < 0:
        outer = list(reversed(outer))
    holes = [points for area, points in converted[1:] if abs(area) >= MIN_HOLE_AREA]
    dock_col, dock_row, start_col, start_row, theta = _choose_pose(
        component, width, height
    )
    natural_width = width * world.CELL
    natural_height = height * world.CELL
    try:
        max_extent = float(max_extent)
    except (TypeError, ValueError):
        max_extent = 24.0
    if max_extent <= 0.0:
        max_extent = 24.0
    scale = min(1.0, max_extent / max(natural_width, natural_height))

    def point(value):
        return (
            round(value[0] * scale + PADDING, 4),
            round(value[1] * scale + PADDING, 4),
        )

    polygon = [point(item) for item in outer]
    shapes = []
    for hole in holes:
        outline = [point(item) for item in hole]
        area = abs(_polygon_area(outline))
        if area < MIN_HOLE_AREA:
            continue
        shapes.append(
            {
                "kind": "large" if area >= 1.5 else "small",
                "parts": [],
                "outline": outline,
                "area": round(area, 2),
            }
        )
    map_name = name
    if not map_name:
        listed = map_info.get("map_list") or []
        if listed and isinstance(listed[0], dict):
            map_name = listed[0].get("map_name")
    if not map_name:
        map_name = os.path.splitext(os.path.basename(map_file))[0]
    map_name = str(map_name).strip() or "实景地图"
    if not map_name.endswith("·实景"):
        map_name = "{}·实景".format(map_name)

    width_m = round(natural_width * scale + PADDING * 2.0, 3)
    height_m = round(natural_height * scale + PADDING * 2.0, 3)
    dock = {
        "x": round((dock_col + 0.5) * world.CELL * scale + PADDING, 4),
        "y": round((dock_row + 0.5) * world.CELL * scale + PADDING, 4),
        "r": 0.45,
    }
    start = {
        "x": round((start_col + 0.5) * world.CELL * scale + PADDING, 4),
        "y": round((start_row + 0.5) * world.CELL * scale + PADDING, 4),
        "theta": round(theta, 4),
    }
    free_area = len(component_cells) * world.CELL * world.CELL * scale * scale
    return {
        "id": None,
        "name": map_name,
        "native": True,
        "imported": True,
        "source_path": root if os.path.isdir(source) else map_file,
        "map_file": map_file,
        "width": width_m,
        "height": height_m,
        "polygon": polygon,
        "obstacles": shapes,
        "dock": dock,
        "start": start,
        "import_meta": {
            "resolution": resolution,
            "source_width": native_width,
            "source_height": native_height,
            "sim_width": width_m,
            "sim_height": height_m,
            "scale": round(scale, 4),
            "free_area_m2": round(free_area, 2),
            "obstacle_count": len(shapes),
            "map_file": os.path.basename(map_file),
        },
    }


def _point_in_polygon(x, y, polygon):
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
