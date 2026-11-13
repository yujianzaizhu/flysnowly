"""日志回放：把 bag 里的位姿轨迹映射到仿真机器人上并实时评估吻合度。

* 载入日志后按“日志坐标系 → 地图坐标系”做一次平移（保持尺度不变），
  让整条轨迹落在当前地图的自由区里；
* 回放时机器人仍然由自身运动学（四轮后驱差速模型）驱动，控制器
  以日志位姿为目标做前馈 + P 修正，因此动作是"仿真出来的"，不是直接
  改写位姿；
* 每 0.1 s 采样一次日志位姿与仿真位姿的偏差，统计吻合率
  （位置 ≤ 0.20 m 且航向 ≤ 12° 记为吻合）。
"""

import base64
import math

import bag_reader
import world

FIDELITY_POS_TOLERANCE = 0.20
FIDELITY_YAW_TOLERANCE = math.radians(12.0)
FIDELITY_SAMPLE_INTERVAL = 0.1
LOOKAHEAD = 0.35
# 只有日志本身重定位（见 bag_reader.find_relocations）才同步对齐一次；
# 控制跟不上造成的偏差靠参考时间同步收回来，绝不瞬移。
MAX_LINEAR = 0.60
MAX_ANGULAR = 1.80
# 回放允许的角速度上限：日志里原地转向可达 200°/s 以上，
# 若仍用割草限速，仿真必然追不上日志轨迹
REPLAY_MAX_ANGULAR = 4.20
REPLAY_MAX_LINEAR = 0.90
# 参考时间同步：车落在参考点后面就把参考放慢（车头前就加快），
# 让"沿轨迹方向"的偏差始终维持在几厘米，而不是越拉越大直到瞬移。
REPLAY_TIME_RATE_MIN = 0.10
REPLAY_TIME_RATE_MAX = 1.00
REPLAY_TIME_LAG_GAIN = 1.60
REPLAY_GAP_FREEZE = 0.80
# 位姿轨迹预处理：中值去毛刺 / 滑动平均 / 限速
REPLAY_DESPIKE_RADIUS = 2
REPLAY_SMOOTH_RADIUS = 2
REPLAY_TRACK_MAX_V = 1.60
REPLAY_TRACK_MAX_W = 4.50
# 障碍点云按这个尺寸的方格去重 / 刷新（同一格最近一次观测说了算）
CLOUD_CELL = 0.20
# 一个格子要"连续被看清 N 帧且都没测到东西"才清掉：深度相机是稀疏采样，
# 实测同一处障碍只有约 37% 的观测帧会报点，一帧没报就删会闪成一片。
CLOUD_MISS_LIMIT = 3
# 点云观测的留存窗口：最后一次被观测到超过这么久（1 分钟），就从记忆地图里**删掉**
# （不是只隐藏），同一条日志往回拖才会重新生成。这段时间内粒子按年龄逐渐变淡。
CLOUD_FADE_SECONDS = 60.0


class ReplayError(ValueError):
    pass


def _wrap(angle):
    two_pi = math.pi * 2.0
    return ((((angle + math.pi) % two_pi) + two_pi) % two_pi) - math.pi


def _clamp(value, low, high):
    return low if value < low else (high if value > high else value)


def _unwrap_yaws(track):
    yaws = []
    for point in track:
        if not yaws:
            yaws.append(point[3])
        else:
            yaws.append(yaws[-1] + _wrap(point[3] - yaws[-1]))
    return yaws


def _despike_track(track, radius=2):
    """中值去毛刺：单点跳变（定位抖一下又回来）会被整段抹平，真位移保留。"""
    if len(track) < 3 or radius <= 0:
        return track
    xs = [point[1] for point in track]
    ys = [point[2] for point in track]
    yaws = _unwrap_yaws(track)
    count = len(track)
    out = []
    for index in range(count):
        low = max(0, index - radius)
        high = min(count - 1, index + radius)
        out.append(
            (
                track[index][0],
                _median(xs[low : high + 1]),
                _median(ys[low : high + 1]),
                _median(yaws[low : high + 1]),
            )
        )
    return out


def _median(values):
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])


def _limit_track(track):
    """把相邻采样的速度/角速度限制在车能跟上的范围（日志重定位段已在外面切开）。

    日志里偶尔一整段位姿都比车快（掉台阶、被搬动），硬跟会把误差拉大到瞬移；
    这里先把参考轨迹限速成"车能做到的样子"，回放才谈得上映射。
    """
    if len(track) < 2:
        return track
    out = [track[0]]
    for index in range(1, len(track)):
        stamp, x, y, yaw = track[index]
        prev_stamp, prev_x, prev_y, prev_yaw = out[-1]
        span = max(1e-3, stamp - prev_stamp)
        max_step = REPLAY_TRACK_MAX_V * span
        dx = x - prev_x
        dy = y - prev_y
        distance = math.hypot(dx, dy)
        if distance > max_step and distance > 1e-9:
            ratio = max_step / distance
            x = prev_x + dx * ratio
            y = prev_y + dy * ratio
        max_turn = REPLAY_TRACK_MAX_W * span
        delta = _wrap(yaw - prev_yaw)
        if abs(delta) > max_turn:
            yaw = prev_yaw + math.copysign(max_turn, delta)
        out.append((stamp, x, y, yaw))
    return out


def _prepare_track(track, relocations):
    """日志位姿 → 回放参考轨迹。

    先按日志自身的重定位时刻切段，再逐段去毛刺、平滑、限速：不切段的话，
    中值/平均会把"跳到新位置"这件事抹成一段斜线，参考点就会以几米每秒的速度
    从旧位置滑到新位置。返回值里同时给出每段的起止时间（``bridges``），
    回放时在这些时刻同步对齐、并且不让参考点插值穿过重定位区间。
    """
    if len(track) < 3:
        return list(track), []
    cuts = [stamp for stamp in relocations if track[0][0] < stamp < track[-1][0]]
    bounds = list(cuts) + [float("inf")]
    cursor = 0
    bridges = []
    prepared = []
    for limit in bounds:
        segment = []
        while cursor < len(track) and track[cursor][0] < limit:
            segment.append(track[cursor])
            cursor += 1
        if segment:
            if prepared:
                # 上一段末尾 → 这一段开头就是重定位，插值必须跳过这一段
                bridges.append((prepared[-1][0], segment[0][0]))
            prepared.extend(
                _limit_track(
                    _smooth_track(
                        _despike_track(segment, REPLAY_DESPIKE_RADIUS),
                        REPLAY_SMOOTH_RADIUS,
                    )
                )
            )
    if cursor < len(track):
        segment = list(track[cursor:])
        if prepared:
            bridges.append((prepared[-1][0], segment[0][0]))
        prepared.extend(
            _limit_track(
                _smooth_track(
                    _despike_track(segment, REPLAY_DESPIKE_RADIUS),
                    REPLAY_SMOOTH_RADIUS,
                )
            )
        )
    return prepared, bridges


def _smooth_track(track, radius=2):
    """对日志位姿做滑动平均：bag 里的 base_pose 有 1 kHz 时间戳抖动导致的航向噪声，
    直接用会把控制器带成左右摆动，也拉低吻合率。"""
    if len(track) < 3 or radius <= 0:
        return track
    smoothed = []
    count = len(track)
    for index in range(count):
        low = max(0, index - radius)
        high = min(count - 1, index + radius)
        window = track[low : high + 1]
        x = sum(point[1] for point in window) / len(window)
        y = sum(point[2] for point in window) / len(window)
        sin_sum = sum(math.sin(point[3]) for point in window)
        cos_sum = sum(math.cos(point[3]) for point in window)
        yaw = math.atan2(sin_sum, cos_sum)
        smoothed.append((track[index][0], x, y, yaw))
    return smoothed


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


def _apply_cloud_frame(state, frame, scale, dx, dy, cell=CLOUD_CELL):
    """把一帧点云并进"原始点缓冲"，同时记录"这一片看过了但什么都没测到"。

    * ``frame`` = ``(相对时间, [x, y, ...], [vx, vy, ...])``，后两项都在日志地图坐标；
    * ``state`` = ``{"points": [(x, y, 观测时间, 格键)], "miss": {...}, "dead": {...}}``；
    * 点坐标**不做任何格网吸附**：以前按 0.2 m 格子去重、坐标取"亚格平滑值"，
      画出来是一片稀疏散点、像给障碍描了个边；RViz 里那层是原始点的"喷雾"，
      同一处多次观测会叠成一片。0.2 m 格子现在只用来数"没看到"了几次，不影响显示位置。
    * 视锥落地面（凸四边形）里、但这一帧没测到点的格子记一次"空"，
      连续 ``CLOUD_MISS_LIMIT`` 次就把该格**此前**观测到的点判定为过时（只删那之前的点，
      之后再测到的照样保留）——这就是"同一个地方第二次探测没东西了"。
    """
    stamp, points, view = frame
    buffer = state["points"]
    hit = set()
    for index in range(0, len(points) - 1, 2):
        x = points[index] * scale + dx
        y = points[index + 1] * scale + dy
        key = (int(math.floor(x / cell)), int(math.floor(y / cell)))
        buffer.append((round(x, 2), round(y, 2), stamp, key))
        for dc in (-1, 0, 1):
            for dr in (-1, 0, 1):
                hit.add((key[0] + dc, key[1] + dr))
    if len(view) < 6:
        return
    polygon = [
        (view[index] * scale + dx, view[index + 1] * scale + dy)
        for index in range(0, len(view) - 1, 2)
    ]
    cols = [point[0] for point in polygon]
    rows = [point[1] for point in polygon]
    col0 = int(math.floor(min(cols) / cell))
    col1 = int(math.floor(max(cols) / cell))
    row0 = int(math.floor(min(rows) / cell))
    row1 = int(math.floor(max(rows) / cell))
    miss = state["miss"]
    dead = state["dead"]
    for col in range(col0, col1 + 1):
        for row in range(row0, row1 + 1):
            key = (col, row)
            if key in hit:
                continue
            if not _point_in_polygon((col + 0.5) * cell, (row + 0.5) * cell, polygon):
                continue
            count = miss.get(key, 0) + 1
            if count >= CLOUD_MISS_LIMIT:
                # 这一片被连续看清、什么都没测到：落在该格、且观测时间早于此刻的点都算过时
                dead[key] = stamp
                count = 0
            miss[key] = count


def _cloud_window_box(window):
    """代价地图窗口在仿真坐标里的外接矩形 ``(x0, y0, x1, y1)``。"""
    if not window:
        return None
    x0, y0, res, width, height, _data = window
    if res <= 0:
        return None
    return (x0, y0, x0 + width * res, y0 + height * res)


def _prune_cloud_points(state, now, window=None):
    """裁剪原始点：超出留存窗口 / 已被判空 / 跑出代价地图范围的点都丢掉。

    全都"只减不增"（时间只往前走、代价地图窗口跟着车走），所以增量缓存不会失效。
    ``now = inf`` 是"整段归档"用的，此时只按判空裁剪，不按留存窗口裁剪。
    """
    cutoff = now - CLOUD_FADE_SECONDS if math.isfinite(now) else -float("inf")
    dead = state["dead"]
    box = _cloud_window_box(window)
    kept = []
    for point in state["points"]:
        if point[2] < cutoff:
            continue
        if point[2] < dead.get(point[3], -1e18):
            continue
        if box is not None and not (
            box[0] <= point[0] <= box[2] and box[1] <= point[1] <= box[3]
        ):
            continue
        kept.append(point)
    state["points"] = kept


def _interp(track, time):
    """在时间轴上线性插值位姿。"""
    if not track:
        return None
    if time <= track[0][0]:
        return track[0][1], track[0][2], track[0][3]
    if time >= track[-1][0]:
        return track[-1][1], track[-1][2], track[-1][3]
    low, high = 0, len(track) - 1
    while low + 1 < high:
        mid = (low + high) // 2
        if track[mid][0] <= time:
            low = mid
        else:
            high = mid
    t0, x0, y0, a0 = track[low]
    t1, x1, y1, a1 = track[high]
    span = t1 - t0
    ratio = 0.0 if span <= 1e-9 else (time - t0) / span
    yaw = a0 + _wrap(a1 - a0) * ratio
    return x0 + (x1 - x0) * ratio, y0 + (y1 - y0) * ratio, yaw


class BagReplay(object):
    """日志回放控制器。"""

    def __init__(self, robot):
        self.robot = robot
        self.transform = None
        self.whole_map = None
        self.costmap_frames = []
        self.costmap_index = -1
        self._costmap_cache = {}
        self.info = None
        self.track = []
        self.commands = []
        self.name = None
        self.duration = 0.0
        self.time = 0.0
        self.playing = False
        self.active = False
        self.offset = (0.0, 0.0)
        self.map_offset = (0.0, 0.0)
        self.hits = 0
        self.samples = 0
        self.error_sum = 0.0
        self.error_max = 0.0
        self.last_sample = -1.0
        self.finished = False
        self.teleports = 0
        self.image_info = None
        self.image_frames = []
        self._image_cache = {}
        # 每开始一次回放 / 拖动一次进度条就 +1：前端据此断开轨迹，
        # 避免画出一条从上一位置连到起始点的“初始跳线”
        self.restarts = 0
        # 日志自身的重定位时刻（相对时间）与对应的时间空档
        self.relocations = []
        self._bridges = []
        self._lag = 0.0
        self._cut_index = 0
        self.scale = 1.0
        # 日志当时的墙钟时间起点（"世界时间"用它，不是仿真时钟）
        self.wall_start = None

    # ------------------------------------------------------------- 载入
    def load(self, info, transform=None):
        """载入日志信息。

        ``transform`` 由日志自带地图（whole_map）构造场地时给出，
        把日志坐标 (log) 映射到仿真坐标：sim = (log - origin) * scale + offset；
        没有日志地图时退化为"把轨迹平移到当前场地自由区中心"。
        """
        poses = info.get("poses") or []
        if len(poses) < 2:
            raise ReplayError("日志位姿太少，无法回放")
        self.transform = transform
        maps = info.get("maps") or {}
        self.whole_map = maps.get("whole_map")
        self._raw_poses = list(poses)
        self._alignment_kind = "none"
        self.pose_frame = (info or {}).get("pose_frame") or "local"
        if transform and self.whole_map:
            # 静态地图（whole_map）与位姿可能不是同一坐标系：地图按自身 origin
            # 换到仿真坐标，轨迹则整条平移（尺度不变）到日志地图里真正走过的位置。
            origin = transform["origin"]
            scale = float(transform.get("scale", 1.0))
            base = transform["offset"]
            self.scale = scale
            self.map_offset = (
                -origin[0] * scale + base[0],
                -origin[1] * scale + base[1],
            )
            shift = self._pose_shift(info)
            self.offset = (
                self.map_offset[0] + shift[0] * scale,
                self.map_offset[1] + shift[1] * scale,
            )
            raw = [
                (t, x * scale + self.offset[0], y * scale + self.offset[1], yaw)
                for t, x, y, yaw in poses
            ]
            self.alignment = self._alignment_kind
        elif transform:
            # 只有地图没有位姿坐标系信息：地图按自身 origin 换算，轨迹照几何平移
            origin = transform["origin"]
            scale = float(transform.get("scale", 1.0))
            base = transform["offset"]
            self.scale = scale
            self.map_offset = (
                -origin[0] * scale + base[0],
                -origin[1] * scale + base[1],
            )
            self.offset = self.map_offset
            raw = [
                (t, x * scale + self.offset[0], y * scale + self.offset[1], yaw)
                for t, x, y, yaw in poses
            ]
            self.alignment = "map_only"
        else:
            center = (info.get("summary") or {}).get("center") or (0.0, 0.0)
            target = self._map_anchor()
            self.offset = (target[0] - center[0], target[1] - center[1])
            raw = [
                (t, x + self.offset[0], y + self.offset[1], yaw)
                for t, x, y, yaw in poses
            ]
            self.alignment = "open_field"
        self.track, self._bridges = _prepare_track(
            raw, list(info.get("relocations") or [])
        )
        self.relocations = [high for _low, high in self._bridges]
        # 日志当时的墙钟时间起点（"世界时间"用它，而不是仿真时钟）
        self.wall_start = (info or {}).get("wall_start")
        self.commands = list(info.get("commands") or [])
        self.costmap_frames = maps.get(bag_reader.COSTMAP_TOPIC) or []
        self.costmap_index = -1
        self._costmap_cache = {}
        # 障碍点云"记忆地图"：cells 是 {(col, row): [x, y, 未命中次数]}，按事件增量更新
        self._cloud_state = None
        self._cloud_event_cache = None
        self.image_info = info.get("images") or None
        self.image_frames = (self.image_info or {}).get("frames") or []
        self._image_cache = {}
        self.info = info
        self.name = info.get("name")
        self.duration = self.track[-1][0] if self.track else 0.0
        self.time = 0.0
        self.hits = 0
        self.samples = 0
        self.error_sum = 0.0
        self.error_max = 0.0
        self.last_sample = -1.0
        self.finished = False
        self.playing = False
        self.active = False
        self.teleports = 0
        self._lag = 0.0
        self._cut_index = 0
        self._bridges = list(getattr(self, "_bridges", []) or [])
        self.scale = float((transform or {}).get("scale", 1.0) or 1.0)
        return self.state()

    def _pose_shift(self, info):
        """位姿坐标系 → 日志地图坐标系的平移量。

        位姿本身就在地图坐标系里（``pose_frame == "map"``，即取 tf 的
        world→base_link1）时平移量是 0：此时机器人位置和局部代价地图窗口严丝合缝
        （实测窗口中心与位姿只差 0.03 m），窗口自然跟着车走。

        否则位姿在局部坐标系里，优先用局部代价地图反推出来的平移
        （``info["pose_frame_offset"]``）：反推量不可信（离散度大）或摆完之后轨迹
        跑进占据区时，退回"轨迹中心对准自由区中心"的老办法。
        """
        if (info or {}).get("pose_frame") == "map":
            self.pose_frame = "map"
            self._alignment_kind = "map"
            return (0.0, 0.0)
        candidate = info.get("pose_frame_offset") or {}
        if candidate.get("centered") and abs(candidate.get("dx", 0.0)) < 1e3:
            shift = (float(candidate["dx"]), float(candidate["dy"]))
            if self._free_ratio(shift) >= 0.90:
                self._alignment_kind = "costmap"
                return shift
        self._alignment_kind = "free_center"
        free_center = self._free_center_log()
        track_center = (info.get("summary") or {}).get("center") or (0.0, 0.0)
        if free_center is None:
            return (0.0, 0.0)
        return (free_center[0] - track_center[0], free_center[1] - track_center[1])

    def _free_ratio(self, shift):
        """按给定平移把位姿摆进日志地图后，落在自由格上的比例。"""
        grid = self.whole_map or {}
        data = grid.get("data") or b""
        poses = getattr(self, "_raw_poses", None) or []
        if not data or not poses:
            return 0.0
        resolution = grid["resolution"]
        width = grid["width"]
        height = grid["height"]
        ox, oy = grid["origin"]
        step = max(1, len(poses) // 200)
        total = 0
        inside = 0
        for pose in poses[::step]:
            col = int((pose[1] + shift[0] - ox) / resolution)
            row = int((pose[2] + shift[1] - oy) / resolution)
            total += 1
            if 0 <= col < width and 0 <= row < height and data[row * width + col] < 50:
                inside += 1
        return inside / float(total) if total else 0.0

    def _free_center_log(self):
        """日志地图自由区的中心（日志坐标），用来安放轨迹。"""
        grid = self.whole_map
        if not grid:
            return None
        resolution = grid["resolution"]
        width = grid["width"]
        height = grid["height"]
        data = grid["data"]
        step = max(1, int(max(width, height) / 120.0))
        total = 0
        sum_col = 0
        sum_row = 0
        for row in range(0, height, step):
            base = row * width
            for col in range(0, width, step):
                if data[base + col] >= 50:
                    continue
                total += 1
                sum_col += col
                sum_row += row
        if not total:
            return None
        origin = grid["origin"]
        return (
            origin[0] + (sum_col / float(total) + 0.5) * resolution,
            origin[1] + (sum_row / float(total) + 0.5) * resolution,
        )

    def image_at(self, time):
        """取该时刻最近的日志图像（割草分割图等），返回可直接画到 <img> 的 data URL。"""
        frames = self.image_frames
        if not frames:
            return None
        low, high = 0, len(frames) - 1
        while low < high:
            mid = (low + high + 1) // 2
            if frames[mid][0] <= time:
                low = mid
            else:
                high = mid - 1
        if low < len(frames) - 1:
            nxt = low + 1
            if abs(frames[nxt][0] - time) < abs(frames[low][0] - time):
                low = nxt
        cached = self._image_cache.get(low)
        if cached is not None:
            return cached
        info = self.image_info or {}
        mime = "image/png" if "png" in str(info.get("encoding", "")).lower() else "image/jpeg"
        result = {
            "index": low,
            "count": len(frames),
            "time": round(frames[low][0], 2),
            "topic": info.get("topic"),
            "width": info.get("width"),
            "height": info.get("height"),
            "mime": mime,
            "image": "data:{};base64,{}".format(
                mime, base64.b64encode(frames[low][1]).decode("ascii")
            ),
        }
        if len(self._image_cache) > 12:
            self._image_cache.clear()
        self._image_cache[low] = result
        return result

    def _reference(self, time):
        """取 ``time`` 时刻的参考位姿；重定位的那段空档里保持在跳变前的位置。

        重定位前后日志本身是空的（掉台阶后重新定位，中间空了 4.7 s），线性插值
        会把"跳 26 m"铺成 5 m/s 的斜线，车只能一路追着跑 —— 界面上那段"看着在飞"
        的位移就是这么来的。空档里停在原地，到跳变时刻再同步对齐。
        """
        for low, high in self._bridges:
            if low <= time < high:
                time = low
                break
        return _interp(self.track, time)

    def _track_point_at(self, time):
        """按回放相对时间取轨迹位置（线性插值，重定位处直接跳）。"""
        point = self._reference(time)
        if point is None:
            return 0.0, 0.0
        return point[0], point[1]

    def _costmap_index(self, time):
        """按相对时间找最近的代价地图帧（帧间隔并不均匀，不能按索引比例取）。"""
        frames = self.costmap_frames
        low, high = 0, len(frames) - 1
        while low < high:
            mid = (low + high + 1) // 2
            if frames[mid][0] <= time:
                low = mid
            else:
                high = mid - 1
        if low + 1 < len(frames):
            if abs(frames[low + 1][0] - time) < abs(frames[low][0] - time):
                low += 1
        return low

    def costmap_at(self, time, step=2):
        """取当前回放时刻最近的局部代价地图（已换算到仿真坐标），供界面叠加显示。

        代价地图有自己的 ``origin``，而且和静态地图（whole_map）在同一个坐标系里
        （实测逐像素吻合 93%~100%）。原来把窗口硬拽到车身上，窗口内容却一动不动，
        叠加层就和真实地图错开十几米；现在按它自己的 origin 摆放，位置对得上。
        只有拿不到日志地图（没有 transform）时才退化成"以车身为中心"。
        """
        if not self.costmap_frames:
            return None
        best = self._costmap_index(max(0.0, time))
        cached = self._costmap_cache.get(best)
        if cached is not None:
            return cached
        stamp, grid = self.costmap_frames[best]
        scale = self.scale if self.transform else 1.0
        resolution = grid["resolution"] * scale
        if self.transform:
            # 日志坐标 → 仿真坐标：sim = log * scale + map_offset
            x0 = grid["origin"][0] * scale + self.map_offset[0]
            y0 = grid["origin"][1] * scale + self.map_offset[1]
        else:
            ref_x, ref_y = self._track_point_at(stamp)
            x0 = ref_x - grid["width"] * resolution / 2.0
            y0 = ref_y - grid["height"] * resolution / 2.0
        rows = bag_reader.encode_cells(
            grid["data"], grid["width"], grid["height"], step=step, kind="cost"
        )
        result = {
            "time": round(stamp, 2),
            "frame": best,
            "frames": len(self.costmap_frames),
            "resolution": round(resolution * step, 3),
            "width": len(rows[0]) if rows else 0,
            "height": len(rows),
            "origin": [round(x0, 3), round(y0, 3)],
            "cells": rows,
        }
        if len(self._costmap_cache) > 8:
            self._costmap_cache.clear()
        self._costmap_cache[best] = result
        return result

    def cloud_at(self, time, max_points=3000):
        """截至 ``time`` 的障碍物点云（已换算到仿真坐标），供界面叠加。

        和 RViz 里那层点云同一个语义——**近期的原始观测点**：

        * 点是原始观测坐标（不做格网吸附/合并），同一处多次观测会叠成一片"喷雾"；
        * 只留最近 ``CLOUD_FADE_SECONDS``（60 s）内的观测，超时直接丢弃（不是只隐藏）；
        * 只保留在**当前局部代价地图窗口**内的点：地图上看到的就是"车周围这一块"，
          和代价地图那层完全重合，不会有一堆孤零零的点飘在代价地图范围之外；
        * 同一处"看过了但没有东西"（视锥落地面连续 ``CLOUD_MISS_LIMIT`` 帧没测到点）
          会清掉那里的旧点——深度相机的判空只覆盖前方 1.35 m，所以这一条是补充手段，
          主力仍然是上面的留存窗口。

        点云是拿**地图坐标系**的位姿算出来的（只有位姿在 map 帧里才有），所以和静态地图、
        代价地图同坐标系，用 ``map_offset`` 换算；没有日志地图时才退回轨迹的平移量。
        状态按"已应用事件数"增量维护：时间往前走只补新的事件，往回拖才从头重建。
        """
        events = self._cloud_events()
        if not events:
            return None
        scale = self.scale if self.transform else 1.0
        if self.transform:
            dx, dy = self.map_offset
        else:
            dx, dy = self.offset
        upto = 0
        for stamp, _kind, _payload in events:
            if stamp > time:
                break
            upto += 1
        cache = self._cloud_state
        if (
            cache is not None
            and cache["transform"] == (round(scale, 6), dx, dy)
            and cache["applied"] <= upto
            # 状态已经按 cache["time"] 裁剪过（留存窗口 + 代价地图范围），只对"不早于它"的
            # 查询有效：往回拖必须从头重建，否则会拿到已经被裁掉的空点集。
            and time >= cache["time"]
        ):
            state = cache
            start_index = cache["applied"]
        else:
            state = {
                "points": [],
                "miss": {},
                "dead": {},
                "window": None,
                "frames": 0,
                "pruned_at": -1e18,
            }
            start_index = 0
        for stamp, kind, payload in events[start_index:upto]:
            if kind == 0:
                _apply_cloud_frame(state, payload, scale, dx, dy)
                state["frames"] += 1
                # 长日志一次跳很远时别让缓冲无限长：每 10 s 回放时间收一次
                if stamp - state["pruned_at"] >= 10.0:
                    state["pruned_at"] = stamp
                    _prune_cloud_points(state, stamp, state["window"])
            else:
                state["window"] = payload
        _prune_cloud_points(state, time, state["window"])
        state["applied"] = upto
        state["time"] = time
        state["transform"] = (round(scale, 6), dx, dy)
        self._cloud_state = state
        buffer = state["points"]
        total = len(buffer)
        if not total:
            return None
        stride = 1
        if total > max_points:
            stride = int(math.ceil(total / float(max_points)))
        points = []
        ages = []
        for index in range(0, total, stride):
            item = buffer[index]
            points.append(item[0])
            points.append(item[1])
            # 距观测多久（按回放时间算，暂停时不变）：前端据此淡出
            ages.append(round(min(max(0.0, time - item[2]), 1e6), 1))
        return {
            "points": points,
            "ages": ages,
            "fade": CLOUD_FADE_SECONDS,
            "count": len(points) // 2,
            "total": total,
            "frames": state["frames"],
        }

    def _cloud_events(self):
        """点云帧与代价地图帧按时间合成一条事件流（增量回放用）。

        ``kind = 0`` 是点云帧（写点 + 判空），``kind = 1`` 是代价地图帧（清空地）。
        每包只构建一次。
        """
        cached = getattr(self, "_cloud_event_cache", None)
        if cached is not None:
            return cached
        events = []
        for frame in (self.info or {}).get("clouds") or []:
            events.append((frame[0], 0, frame))
        scale = self.scale if self.transform else 1.0
        for stamp, grid in self.costmap_frames or []:
            resolution = float(grid["resolution"]) * scale
            if resolution <= 0:
                continue
            width = grid["width"]
            height = grid["height"]
            if self.transform:
                x0 = grid["origin"][0] * scale + self.map_offset[0]
                y0 = grid["origin"][1] * scale + self.map_offset[1]
            else:
                # 没有日志地图时代价地图窗口以车身为中心（与 costmap_at 一致）
                ref = self._track_point_at(stamp)
                if ref is None:
                    continue
                x0 = ref[0] - width * resolution / 2.0
                y0 = ref[1] - height * resolution / 2.0
            events.append((stamp, 1, (x0, y0, resolution, width, height, grid["data"])))
        events.sort(key=lambda item: (item[0], item[1]))
        self._cloud_event_cache = events
        return events

    def _map_anchor(self):
        """回放轨迹的落点：优先充电桩附近，其次场地中心。"""
        for candidate in (
            (world.DOCK["x"], world.DOCK["y"]),
            (world.WIDTH / 2.0, world.HEIGHT / 2.0),
        ):
            if world.point_in_polygon(candidate[0], candidate[1]):
                return candidate
        return (world.WIDTH / 2.0, world.HEIGHT / 2.0)

    # ------------------------------------------------------------- 控制
    def start(self):
        if not self.track:
            raise ReplayError("请先载入日志")
        self.robot.replay_limits = (REPLAY_MAX_LINEAR, REPLAY_MAX_ANGULAR)
        self.active = True
        self.playing = True
        self.finished = False
        self.seek(0.0)
        return self.state()

    def pause(self):
        self.playing = False
        self.robot.stop()
        return self.state()

    def resume(self):
        if not self.track:
            raise ReplayError("请先载入日志")
        if self.finished or self.time >= self.duration:
            self.seek(0.0)
        self.robot.replay_limits = (REPLAY_MAX_LINEAR, REPLAY_MAX_ANGULAR)
        self.active = True
        self.playing = True
        return self.state()

    def stop(self):
        self.playing = False
        self.active = False
        self.robot.replay_limits = None
        self.robot.stop()
        return self.state()

    def clear(self):
        """清空已载入的日志（复位时用，界面的日志图像窗口也会随之消失）。"""
        self.stop()
        self.info = None
        self.track = []
        self.commands = []
        self.name = None
        self.duration = 0.0
        self.time = 0.0
        self.finished = False
        self.samples = 0
        self.hits = 0
        self.error_sum = 0.0
        self.error_max = 0.0
        self.teleports = 0
        self.relocations = []
        self._bridges = []
        self._lag = 0.0
        self._cut_index = 0
        self.alignment = "none"
        self.pose_frame = "local"
        self.wall_start = None
        self._raw_poses = []
        self.costmap_frames = []
        self.costmap_index = -1
        self._costmap_cache = {}
        self._cloud_state = None
        self._cloud_event_cache = None
        self.image_info = None
        self.image_frames = []
        self._image_cache = {}

    def seek(self, time):
        time = _clamp(float(time), 0.0, self.duration)
        reference = self._reference(time)
        self.restarts += 1
        self.time = time
        self.last_sample = -1.0
        self._lag = 0.0
        self._cut_index = len([cut for cut in self.relocations if cut <= time])
        self.finished = False
        if reference is not None:
            # 拖动进度条时把车体放到该时刻的日志位姿，回放从这里继续
            self.robot.pose = [reference[0], reference[1], reference[2]]
            self.robot.v = 0.0
            self.robot.w = 0.0
            self.robot.target_v = 0.0
            self.robot.target_w = 0.0
        return self.state()

    def _command_at(self, time):
        if not self.commands:
            return None
        best = None
        for stamp, linear, angular in self.commands:
            if stamp > time:
                break
            best = (linear, angular)
        return best

    def tick(self, dt):
        """推进回放，返回 (v, w)；回放结束时返回 None。"""
        if not self.active or not self.track:
            return None
        if not self.playing:
            return 0.0, 0.0

        # 参考时间同步：车落在参考点后面就把参考放慢，让车自己把误差收回来。
        # 原来是固定 1 倍速推进：车跟不上时误差只会越拉越大，最后被瞬移一下 ——
        # 界面上看到的就是"位姿跳变"。另外，离得远时不许加速参考：掉台阶那段
        # 日志航向乱翻，"沿轨迹误差"的正负会反过来，加速只会把参考点送得更远
        # （实测误差一路涨到 5 m）。
        current = self._reference(self.time)
        gap = math.hypot(
            self.robot.pose[0] - current[0], self.robot.pose[1] - current[1]
        )
        rate = _clamp(
            1.0 - REPLAY_TIME_LAG_GAIN * self._lag,
            REPLAY_TIME_RATE_MIN,
            REPLAY_TIME_RATE_MAX,
        )
        if gap > REPLAY_GAP_FREEZE:
            # 离得远的时候绝不加速参考：掉台阶那段日志航向乱翻，"沿轨迹误差"的
            # 正负会反过来，加速参考只会把参考点送得更远（实测误差一路涨到 5 m）。
            rate = min(rate, 1.0)
        if self.time + dt * rate >= self.duration:
            self.time = self.duration
            self.playing = False
            self.finished = True
            self.robot.stop()
            self._sample(force=True)
            return 0.0, 0.0
        self.time += dt * rate

        reference = self._reference(self.time)
        # 只有日志自己重定位的那几处才同步对齐；控制跟不上造成的偏差交给
        # 上面的参考时间同步慢慢收回来，绝不瞬移。
        if (
            self._cut_index < len(self.relocations)
            and self.time >= self.relocations[self._cut_index]
        ):
            self._cut_index += 1
            self.robot.pose = [reference[0], reference[1], reference[2]]
            self.robot.v = 0.0
            self.robot.w = 0.0
            self.robot.target_v = 0.0
            self.robot.target_w = 0.0
            self.teleports += 1
            self._lag = 0.0
            self._sample(force=True)
            return 0.0, 0.0
        x, y, yaw = self.robot.pose
        error_x = reference[0] - x
        error_y = reference[1] - y

        ahead = _interp(self.track, min(self.duration, self.time + LOOKAHEAD))
        span = min(LOOKAHEAD, max(1e-3, self.duration - self.time))
        # 参考量前馈：直接用日志的位移速度与角速度，仿真跟得上才谈得上"映射"
        feed_speed = math.hypot(ahead[0] - reference[0], ahead[1] - reference[1]) / span
        feed_turn = _wrap(ahead[2] - reference[2]) / span
        feed_turn = _clamp(feed_turn, -REPLAY_MAX_ANGULAR, REPLAY_MAX_ANGULAR)
        yaw_error = _wrap(reference[2] - yaw)
        cos_yaw = math.cos(reference[2])
        sin_yaw = math.sin(reference[2])
        along = error_x * cos_yaw + error_y * sin_yaw
        cross = -error_x * sin_yaw + error_y * cos_yaw
        self._lag = along
        speed = feed_speed + 1.2 * along
        # 横向偏差要往"回轨迹"的方向打舵：cross > 0 表示参考点在车左侧，
        # 转向角速度得取正。原来写的 -1.2 * cross 与航向项互相抵消，
        # 车会一直偏在轨迹一侧（实测稳定偏 0.6~0.75 m），偏差累积到阈值就被瞬移。
        turn = feed_turn + 1.6 * yaw_error + 2.5 * cross
        if abs(yaw_error) > 1.2 and feed_speed < 0.05:
            # 日志里大角度原地转向：先转到位，避免边走边转画大圈
            speed = min(speed, 0.05)
        speed = _clamp(speed, -0.20, REPLAY_MAX_LINEAR)
        turn = _clamp(turn, -REPLAY_MAX_ANGULAR, REPLAY_MAX_ANGULAR)
        self.robot.drive(speed, turn)
        self._sample()
        return speed, turn

    def _sample(self, force=False):
        if not self.track:
            return
        if not force and self.time - self.last_sample < FIDELITY_SAMPLE_INTERVAL:
            return
        self.last_sample = self.time
        reference = self._reference(self.time)
        if reference is None:
            return
        x, y, yaw = self.robot.pose
        position_error = math.hypot(reference[0] - x, reference[1] - y)
        yaw_error = abs(_wrap(reference[2] - yaw))
        self.samples += 1
        self.error_sum += position_error
        self.error_max = max(self.error_max, position_error)
        if position_error <= FIDELITY_POS_TOLERANCE and yaw_error <= FIDELITY_YAW_TOLERANCE:
            self.hits += 1

    # ------------------------------------------------------------- 状态
    def fidelity(self):
        if not self.samples:
            return 0.0
        return self.hits / float(self.samples)

    def reference_path(self, max_points=600):
        if not self.track:
            return []
        step = max(1, len(self.track) // max_points)
        return [
            [round(point[1], 2), round(point[2], 2)] for point in self.track[::step]
        ]

    def state(self):
        fidelity = self.fidelity()
        return {
            "loaded": bool(self.track),
            "name": self.name,
            "active": self.active,
            "playing": self.playing,
            "finished": self.finished,
            "time": round(self.time, 2),
            "duration": round(self.duration, 2),
            "progress": round(self.time / self.duration, 4) if self.duration > 0 else 0.0,
            "samples": self.samples,
            "fidelity": round(fidelity, 4),
            "error_avg": round(self.error_sum / self.samples, 3) if self.samples else 0.0,
            "error_max": round(self.error_max, 3),
            "offset": [round(self.offset[0], 2), round(self.offset[1], 2)],
            "teleports": self.teleports,
            "relocations": len(self.relocations),
            "alignment": getattr(self, "alignment", "none"),
            "pose_frame": getattr(self, "pose_frame", "local"),
            # 世界时间：日志当时的墙钟时间，带毫秒
            "wall_time": None
            if not self.wall_start
            else round(self.wall_start + self.time, 3),
            "tf_pose": (self.info or {}).get("tf_pose"),
            "restarts": self.restarts,
            "summary": (self.info or {}).get("summary"),
            "pose_topic": (self.info or {}).get("pose_topic"),
            "image_topic": (self.image_info or {}).get("topic"),
            "image_frames": len(self.image_frames),
        }
