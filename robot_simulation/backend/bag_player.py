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
# 日志位姿跳变超过这个距离视为重定位，仿真侧同步对齐而不是硬追
REPLAY_TELEPORT = 0.80
MAX_LINEAR = 0.60
MAX_ANGULAR = 1.80
# 回放允许的角速度上限：日志里原地转向可达 200°/s 以上，
# 若仍用割草限速，仿真必然追不上日志轨迹
REPLAY_MAX_ANGULAR = 4.20
REPLAY_MAX_LINEAR = 0.80


class ReplayError(ValueError):
    pass


def _wrap(angle):
    two_pi = math.pi * 2.0
    return ((((angle + math.pi) % two_pi) + two_pi) % two_pi) - math.pi


def _clamp(value, low, high):
    return low if value < low else (high if value > high else value)


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
        if transform and self.whole_map:
            # 静态地图（whole_map）与位姿/代价地图可能不是同一坐标系：
            # 地图按自身 origin 换到仿真坐标，轨迹则整条平移到地图自由区中心，
            # 保证机器人开在日志地图的场地内。
            origin = transform["origin"]
            scale = float(transform.get("scale", 1.0))
            base = transform["offset"]
            self.map_offset = (
                -origin[0] * scale + base[0],
                -origin[1] * scale + base[1],
            )
            free_center = self._free_center_log()
            track_center = info["summary"]["center"]
            if free_center is not None:
                self.offset = (
                    free_center[0] + self.map_offset[0] - track_center[0],
                    free_center[1] + self.map_offset[1] - track_center[1],
                )
            else:
                self.offset = self.map_offset
            raw = [
                (t, x + self.offset[0], y + self.offset[1], yaw)
                for t, x, y, yaw in poses
            ]
        elif transform:
            origin = transform["origin"]
            scale = float(transform.get("scale", 1.0))
            base = transform["offset"]
            self.map_offset = self.offset = (
                -origin[0] * scale + base[0],
                -origin[1] * scale + base[1],
            )
            raw = [
                (t, x + self.offset[0], y + self.offset[1], yaw)
                for t, x, y, yaw in poses
            ]
        else:
            center = info["summary"]["center"]
            target = self._map_anchor()
            self.offset = (target[0] - center[0], target[1] - center[1])
            raw = [
                (t, x + self.offset[0], y + self.offset[1], yaw)
                for t, x, y, yaw in poses
            ]
        self.track = _smooth_track(raw)
        self.commands = list(info.get("commands") or [])
        self.costmap_frames = maps.get(bag_reader.COSTMAP_TOPIC) or []
        self.costmap_index = -1
        self._costmap_cache = {}
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
        return self.state()

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

    def _track_point_at(self, time):
        """按回放相对时间取轨迹位置（线性插值）。"""
        if not self.track:
            return 0.0, 0.0
        if time <= self.track[0][0]:
            return self.track[0][1], self.track[0][2]
        if time >= self.track[-1][0]:
            return self.track[-1][1], self.track[-1][2]
        low, high = 0, len(self.track) - 1
        while low + 1 < high:
            mid = (low + high) // 2
            if self.track[mid][0] <= time:
                low = mid
            else:
                high = mid
        t0, x0, y0, _yaw0 = self.track[low]
        t1, x1, y1, _yaw1 = self.track[high]
        span = t1 - t0
        ratio = 0.0 if span <= 1e-9 else (time - t0) / span
        return x0 + (x1 - x0) * ratio, y0 + (y1 - y0) * ratio

    def costmap_at(self, time, step=2):
        """取当前回放时刻最近的局部代价地图（已换算到仿真坐标），供界面叠加显示。"""
        if not self.costmap_frames:
            return None
        # 帧时间戳是日志绝对时间、回放时间是相对时间，按进度比例取最近帧
        ratio = (time / self.duration) if self.duration > 0 else 0.0
        best = min(
            len(self.costmap_frames) - 1,
            max(0, int(round(ratio * (len(self.costmap_frames) - 1)))),
        )
        cached = self._costmap_cache.get(best)
        if cached is not None:
            return cached
        _stamp, grid = self.costmap_frames[best]
        resolution = grid["resolution"]
        # 局部代价地图是随车滚动的窗口：直接以该帧对应的轨迹点为中心叠加，
        # 这样即使在日志里静态地图与位姿不是同一坐标系也能正确对齐。
        ref_x, ref_y = self._track_point_at(
            (best / max(1.0, len(self.costmap_frames) - 1.0)) * self.duration
            if self.duration
            else 0.0
        )
        x0 = ref_x - grid["width"] * resolution / 2.0
        y0 = ref_y - grid["height"] * resolution / 2.0
        scale = 1.0
        rows = bag_reader.encode_cells(
            grid["data"], grid["width"], grid["height"], step=step, kind="cost"
        )
        result = {
            "time": round(best / max(1.0, len(self.costmap_frames) - 1.0) * self.duration, 2)
            if self.duration
            else 0.0,
            "frame": best,
            "frames": len(self.costmap_frames),
            "resolution": round(resolution * scale * step, 3),
            "width": len(rows[0]) if rows else 0,
            "height": len(rows),
            "origin": [round(x0, 3), round(y0, 3)],
            "cells": rows,
        }
        if len(self._costmap_cache) > 8:
            self._costmap_cache.clear()
        self._costmap_cache[best] = result
        return result

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
        self.costmap_frames = []
        self.costmap_index = -1
        self._costmap_cache = {}
        self.image_info = None
        self.image_frames = []
        self._image_cache = {}

    def seek(self, time):
        time = _clamp(float(time), 0.0, self.duration)
        reference = _interp(self.track, time)
        self.restarts += 1
        self.time = time
        self.last_sample = -1.0
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
        self.time += dt
        if self.time >= self.duration:
            self.time = self.duration
            self.playing = False
            self.finished = True
            self.robot.stop()
            self._sample(force=True)
            return 0.0, 0.0

        reference = _interp(self.track, self.time)
        x, y, yaw = self.robot.pose
        error_x = reference[0] - x
        error_y = reference[1] - y
        distance = math.hypot(error_x, error_y)
        if distance > REPLAY_TELEPORT:
            # 日志里发生重定位/跳变（例如重新上桩定位），仿真侧同步对齐
            self.robot.pose = [reference[0], reference[1], reference[2]]
            self.robot.v = 0.0
            self.robot.w = 0.0
            self.robot.target_v = 0.0
            self.robot.target_w = 0.0
            self.teleports += 1
            self._sample(force=True)
            return 0.0, 0.0

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
        speed = feed_speed + 1.2 * along
        turn = feed_turn + 1.6 * yaw_error - 1.2 * cross - 0.20 * self.robot.w
        if abs(yaw_error) > 1.2 and feed_speed < 0.05:
            # 日志里大角度原地转向：先转到位，避免边走边转画大圈
            speed = min(speed, 0.05)
        speed = _clamp(speed, -0.35, REPLAY_MAX_LINEAR)
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
        reference = _interp(self.track, self.time)
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
            "restarts": self.restarts,
            "summary": (self.info or {}).get("summary"),
            "pose_topic": (self.info or {}).get("pose_topic"),
            "image_topic": (self.image_info or {}).get("topic"),
            "image_frames": len(self.image_frames),
        }
