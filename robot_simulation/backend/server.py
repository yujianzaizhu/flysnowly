"""机器人小车 Web 服务：静态界面 + 任务/遥控 JSON 接口。"""

import argparse
import base64
import hashlib
import json
import gzip
import math
import os
import sys
import tempfile
import threading
import time
import traceback
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlsplit

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import bag_player  # noqa: E402
import bag_reader  # noqa: E402
import config  # noqa: E402
import map_editor  # noqa: E402
import map_import
import world  # noqa: E402
from grid_map import GridMap  # noqa: E402
from robot_car import RobotCar  # noqa: E402
from tasks import TaskManager  # noqa: E402

FRONTEND_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, "frontend")
)

# 割草报告目录：默认仓库根的 data/data（可用 ROBOT_REPORT_DIR 覆盖）
def report_dir():
    import mow_report

    return mow_report.output_dir()


# 自建地图落盘目录：data/map/editor_<名字>/editor_map.json，重启后自动读回来
def editor_map_root():
    return os.path.normpath(
        os.path.join(
            os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir, "data", "map"
        )
    )


def persist_editor_map(definition):
    """把编辑器保存的地图写到 data/map/editor_<名字>/editor_map.json。"""
    name = str(definition.get("name") or "自定义地图")
    safe = "".join(ch if ch.isalnum() or ch in "-_·" else "_" for ch in name).strip("_")
    folder = os.path.join(editor_map_root(), "editor_" + (safe or "map"))
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, "editor_map.json")
    payload = dict(definition)
    payload.pop("id", None)  # 编号以运行时注册为准
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False)
    return path


def load_persisted_editor_maps():
    """启动时把自建地图读回来（不然重启一次就全丢了）。"""
    root = editor_map_root()
    if not os.path.isdir(root):
        return 0
    loaded = 0
    for name in sorted(os.listdir(root)):
        path = os.path.join(root, name, "editor_map.json")
        if not os.path.isfile(path):
            continue
        try:
            with open(path, encoding="utf-8") as handle:
                definition = json.load(handle)
        except (OSError, ValueError):
            continue
        definition["id"] = 0
        definition["imported"] = True
        definition["editor_built"] = True
        definition.setdefault("source_path", "editor:" + str(definition.get("name")))
        try:
            world.register_map(definition)
            loaded += 1
        except Exception:  # pragma: no cover - 坏文件跳过
            traceback.print_exc()
    return loaded


def list_reports(limit=50):
    """data/data 下的割草报告（每次任务一个时间命名的文件夹），按时间倒序。

    兼容旧格式：早期直接把 `mow_report_<时间戳>.html/png/json` 平铺在 data/data 里。
    """
    root = report_dir()
    if not os.path.isdir(root):
        return []
    items = []
    # 新格式：<时间>/report.html（+ coverage.png / data.json）
    for name in sorted(os.listdir(root)):
        folder = os.path.join(root, name)
        if not os.path.isdir(folder):
            continue
        html_path = os.path.join(folder, "report.html")
        if not os.path.isfile(html_path):
            continue
        entry = {
            "folder": name,
            "html": "{}/report.html".format(name),
            "url": "/reports/{}/report.html".format(name),
            "png": "{}/coverage.png".format(name),
            "json": "{}/data.json".format(name),
        }
        try:
            entry["created"] = datetime.fromtimestamp(
                os.path.getmtime(html_path)
            ).isoformat(timespec="seconds")
            entry["size"] = os.path.getsize(html_path)
        except OSError:  # pragma: no cover - 文件刚被删掉
            continue
        items.append(entry)
    # 旧格式（平铺文件）
    for name in sorted(os.listdir(root)):
        if not name.endswith(".html") or not name.startswith("mow_report_"):
            continue
        stem = name[: -len(".html")]
        entry = {
            "folder": None,
            "stem": stem,
            "html": name,
            "url": "/reports/" + name,
            "png": stem + ".png",
            "json": stem + ".json",
        }
        path = os.path.join(root, name)
        try:
            entry["created"] = datetime.fromtimestamp(os.path.getmtime(path)).isoformat(
                timespec="seconds"
            )
            entry["size"] = os.path.getsize(path)
        except OSError:  # pragma: no cover
            continue
        items.append(entry)
    items.sort(key=lambda item: item["created"] or "", reverse=True)
    return items[:limit]


# 日志（bag）目录：默认仓库根的 data/bag，可用环境变量 ROBOT_BAG_DIR 覆盖
BAG_DIR = os.environ.get(
    "ROBOT_BAG_DIR",
    os.path.normpath(
        os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            os.pardir,
            os.pardir,
            "data",
            "bag",
        )
    ),
)

# 真实地图目录：默认仓库根的 data/map，可用环境变量 ROBOT_MAP_DIR 覆盖
MAP_DIR = os.environ.get(
    "ROBOT_MAP_DIR",
    os.path.normpath(
        os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            os.pardir,
            os.pardir,
            "data",
            "map",
        )
    ),
)


def _map_candidates(root):
    """列出可导入的地图（含 .pgm 的目录，或直接的 .pgm 文件）。"""
    root = os.path.abspath(os.path.expanduser(str(root or MAP_DIR)))
    if not os.path.isdir(root):
        raise ValueError("地图目录不存在: " + root)
    candidates = []
    for name in sorted(os.listdir(root)):
        full = os.path.join(root, name)
        if os.path.isdir(full):
            pgms = [
                entry
                for entry in sorted(os.listdir(full))
                if entry.lower().endswith(".pgm")
            ]
            if pgms:
                candidates.append(full)
        elif name.lower().endswith(".pgm"):
            candidates.append(full)
    return root, candidates


def _dedupe_map_name(definition, source):
    """data/map 里几张图的元数据常常同名（都叫 map1），重名时补上目录名。

    同一个目录重复导入（编号复用）不算重名，不会反复加后缀。
    """
    name = definition.get("name")
    source_path = definition.get("source_path") or os.path.abspath(str(source))
    taken = set()
    for other in world.MAPS.values():
        if other is definition:
            continue
        if (other.get("source_path") or "") == source_path:
            continue
        taken.add(other.get("name"))
    if name not in taken:
        return
    label = os.path.basename(str(source).rstrip(os.sep))
    if os.path.isfile(str(source)):
        label = os.path.splitext(label)[0]
    candidate = "{} ({})".format(name, label)
    suffix = 2
    while candidate in taken:
        candidate = "{} ({}-{})".format(name, label, suffix)
        suffix += 1
    definition["name"] = candidate


def import_all_maps(httpd, root=None, max_extent=40.0, select_first=True):
    """把 data/map 下所有可用的地图全部导入，返回导入结果摘要。

    已经导入过的（source_path 相同）会复用原编号，不会重复占号。
    """
    root, candidates = _map_candidates(root)
    imported = []
    failures = []
    for source in candidates:
        try:
            definition = map_import.build_map(source, None, max_extent=max_extent)
        except (map_import.MapImportError, ValueError, OSError) as error:
            failures.append({"source": source, "error": str(error)})
            continue
        with httpd.simulator.lock:
            map_id = world.register_map(definition)
        _dedupe_map_name(definition, source)
        imported.append(
            {
                "id": map_id,
                "name": definition.get("name"),
                "source": source,
                "width": round(definition.get("width", 0.0), 2),
                "height": round(definition.get("height", 0.0), 2),
            }
        )
    if select_first and imported:
        map_id = imported[0]["id"]
        with httpd.simulator.lock:
            world.select_map(map_id)
            robot = RobotCar()
            grid = GridMap()
            manager = build_manager(robot, grid)
            httpd.robot = robot
            httpd.grid = grid
            httpd.manager = manager
            httpd.simulator.manager = manager
    return {
        "directory": root,
        "imported": imported,
        "failures": failures,
        "maps": world.world_dict().get("maps", []),
    }


CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".svg": "image/svg+xml",
    ".png": "image/png",
    ".ico": "image/x-icon",
}

MAX_BODY_BYTES = 64 * 1024
MAX_UPLOAD_BODY_BYTES = 40 * 1024 * 1024
MAX_UPLOAD_FILE_BYTES = 16 * 1024 * 1024
MAP_UPLOAD_EXTENSIONS = (".pgm", ".json", ".yaml", ".yml")
SIM_DT = 0.02


def _store_uploaded_map(files):
    """把浏览器上传的文件按相对路径落到稳定临时目录。"""
    if not isinstance(files, list) or not files:
        raise ValueError("没有上传文件")
    normalized = []
    digest = hashlib.sha256()
    total = 0
    for item in files:
        if not isinstance(item, dict):
            raise ValueError("上传文件格式无效")
        relative = str(item.get("path") or item.get("name") or "").replace("\\", "/")
        relative = os.path.normpath(relative).lstrip("/")
        if relative in ("", ".", "..") or relative.startswith("../"):
            raise ValueError("上传文件路径无效")
        name = os.path.basename(relative).lower()
        if not name.endswith(MAP_UPLOAD_EXTENSIONS):
            raise ValueError("不支持的文件类型: {}".format(os.path.basename(relative)))
        try:
            raw = base64.b64decode(str(item.get("data") or ""), validate=True)
        except (ValueError, TypeError):
            raise ValueError("文件 {} 的 base64 数据无效".format(relative))
        if len(raw) > MAX_UPLOAD_FILE_BYTES:
            raise ValueError("文件 {} 超过 16 MB".format(relative))
        total += len(raw)
        if total > 24 * 1024 * 1024:
            raise ValueError("上传文件总大小超过 24 MB")
        normalized.append((relative, raw))
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(raw)

    # 浏览器目录选择会把顶层目录名带进相对路径；统一剥掉，方便 map_import 解析。
    first_parts = {relative.split("/", 1)[0] for relative, _ in normalized}
    has_nested = all("/" in relative for relative, _ in normalized)
    if len(first_parts) == 1 and has_nested:
        prefix = next(iter(first_parts)) + "/"
        normalized = [(relative[len(prefix) :], raw) for relative, raw in normalized]

    root = os.path.join(
        tempfile.gettempdir(), "robot_map_uploads", digest.hexdigest()[:20]
    )
    os.makedirs(root, exist_ok=True)
    for relative, raw in normalized:
        target = os.path.normpath(os.path.join(root, relative))
        if not target.startswith(root + os.sep):
            raise ValueError("上传文件路径越界")
        os.makedirs(os.path.dirname(target), exist_ok=True)
        with open(target, "wb") as handle:
            handle.write(raw)
    return root


def bag_roots(server):
    """当前可用的日志根目录（默认目录 + 用户导入的文件夹）。"""
    roots = getattr(server, "bag_roots", None)
    if roots is None:
        roots = [BAG_DIR]
        server.bag_roots = roots
    return roots


def add_bag_root(server, path):
    """把用户指定的文件夹加入日志根目录（去重）。"""
    path = os.path.abspath(os.path.expanduser(str(path)))
    if not os.path.isdir(path):
        raise ValueError("文件夹不存在: {}".format(path))
    roots = bag_roots(server)
    known = {os.path.realpath(item) for item in roots}
    if os.path.realpath(path) not in known:
        roots.append(path)
    return roots


def list_bag_entries(server):
    """汇总所有根目录下的日志（带分组），按分组 + 录制时间排序。"""
    entries = []
    seen = set()
    for index, root in enumerate(bag_roots(server)):
        for bag in bag_reader.list_bags(root):
            real = os.path.realpath(bag["path"])
            if real in seen:
                continue
            seen.add(real)
            item = dict(bag)
            item["root"] = root
            item["root_index"] = index
            item["group"] = bag["group"] or (
                "根目录" if index == 0 else (os.path.basename(root.rstrip(os.sep)) or root)
            )
            entries.append(item)
    return entries


def resolve_bag(server, name):
    """把日志标识（分组/文件名、文件名或绝对路径）解析成绝对路径。"""
    name = str(name or "").strip()
    if not name:
        raise ValueError("请选择日志文件")
    if os.path.isabs(name):
        path = os.path.abspath(name)
        if not os.path.isfile(path):
            raise ValueError("日志文件不存在: {}".format(os.path.basename(path)))
        if os.path.splitext(path)[1].lower() not in bag_reader.BAG_SUFFIXES:
            raise ValueError("不支持的文件类型: {}".format(os.path.basename(path)))
        return path
    if ".." in name.replace("\\", "/").split("/"):
        raise ValueError("日志名称无效")
    for root in bag_roots(server):
        candidate = os.path.normpath(os.path.join(root, name))
        if os.path.isfile(candidate):
            return candidate
    raise ValueError("找不到日志: {}".format(name))


# 同一个场地的日志（同一个文件夹里的连续几段）共用一个充电桩位置。
# 每段日志的 whole_map 内容略有差别，按"自由区里最靠场地中心"选桩会因为边界
# 差几格就跳到别的角落去，连续播放时界面上充电桩一直在换地方。
BAG_DOCK_CACHE = {}


def _log_map_signature(log_map):
    return (
        round(log_map["resolution"], 4),
        int(log_map["width"]),
        int(log_map["height"]),
        round(log_map["origin"][0], 3),
        round(log_map["origin"][1], 3),
    )


def _log_map_free_cell(log_map, col, row):
    """日志地图的 (col, row) 格是不是可通行（0=自由 / 100=占据）。"""
    width = log_map["width"]
    height = log_map["height"]
    if not (0 <= col < width and 0 <= row < height):
        return False
    return log_map["data"][row * width + col] < 50


def apply_stable_dock(definition, log_map):
    """同一场地复用第一次选出来的充电桩位置（仍落在可通行区域内时）。"""
    signature = _log_map_signature(log_map)
    transform = definition.get("replay_transform") or {}
    scale = float(transform.get("scale", 1.0) or 1.0)
    base = transform.get("offset") or [0.0, 0.0]
    cached = BAG_DOCK_CACHE.get(signature)
    if cached:
        # sim = (log - origin) * scale + base，一格日志栅格 = resolution * scale（米）
        step = max(1e-6, scale * log_map["resolution"])
        col = int((cached["x"] - base[0]) / step)
        row = int((cached["y"] - base[1]) / step)
        if _log_map_free_cell(log_map, col, row):
            definition["dock"] = dict(cached)
            return definition
    BAG_DOCK_CACHE[signature] = dict(definition.get("dock") or {})
    return definition


def _open_field_active():
    """当前场地是不是"无地图"日志造出来的空白场地。"""
    definition = world.MAPS.get(getattr(world, "MAP_ID", None)) or {}
    return bool(definition.get("open_field"))


def _remember_frame(queue, replay, use_log_map):
    """连续播放时记住本包"日志坐标 → 仿真坐标"的换算，下一包沿用它。

    同一批日志（同一次录制）是分段录的，段与段的位姿首尾相接；但没带 whole_map 的
    日志只能把轨迹"居中摆到场地里"，每包各自居中就会在换包时跳到别处。所以把本包
    实际用的换算记下来（统一成 origin=0 的等价形式），同处一段空白场地上的下一包
    直接沿用。
    """
    if not queue or not queue.get("active") or not use_log_map or replay is None:
        return
    dx, dy = replay.offset
    scale = float(getattr(replay, "scale", 1.0) or 1.0)
    queue["frame_transform"] = {
        "origin": [0.0, 0.0],
        "scale": round(scale, 6),
        "offset": [round(float(dx), 6), round(float(dy), 6)],
    }


# 相邻两包"日志坐标 → 仿真坐标"换算差多少算换了场地（m）。同一张地图的几段日志
# 会有几毫米~几厘米的差别，不算换场地；清成空白地图 / 换一张地图才会明显不同。
FRAME_CHANGED_TOLERANCE = 0.05


def _frame_changed(before, after):
    """两包的坐标换算是否变了：变了就等于"场地换了"，旧轨迹/旧点云都作废。"""
    if not before or not after:
        return True
    if abs(float(before.get("scale", 1.0)) - float(after.get("scale", 1.0))) > 1e-3:
        return True
    old = before.get("offset") or [0.0, 0.0]
    new = after.get("offset") or [0.0, 0.0]
    return any(
        abs(float(new[index]) - float(old[index])) > FRAME_CHANGED_TOLERANCE
        for index in (0, 1)
    )


def load_bag(httpd, name, use_log_map=True):
    """解析日志并载入回放；日志自带地图时同时把它建成仿真场地。"""
    path = resolve_bag(httpd, name)
    info = bag_reader.read_bag(path)
    transform = None
    log_map = (info.get("maps") or {}).get("whole_map")
    manager = httpd.manager
    queue = getattr(httpd, "bag_queue", None) or {}
    # 没地图的日志 = 机器把地图删了正在重新建图/本来就无图：场地先清成空白，机器人
    # 放到空白地图中心；已经在空白场地上（同一段建图的下一包）就继续用那块并沿用换算。
    carrying = bool(queue.get("active")) and use_log_map and not log_map and _open_field_active()
    if use_log_map:
        label = os.path.splitext(os.path.basename(path))[0].split("_")[0] or "日志"
        definition = None
        if log_map:
            try:
                definition = map_import.build_map_from_grid(
                    log_map["resolution"],
                    log_map["width"],
                    log_map["height"],
                    log_map["data"],
                    grid_origin=log_map["origin"],
                    name="{}·日志".format(label),
                )
                definition = apply_stable_dock(definition, log_map)
            except Exception as exc:  # 日志地图坏了就退回开阔场地
                print("日志地图构造失败：{}".format(exc))
                definition = None
        if definition is None:
            carried = queue.get("frame_transform") if carrying else None
            if carried:
                # 同一段建图的下一包（机器人还没建出图）：沿用上一包的换算，留在
                # 同一块空白场地上，换包时机器人的绝对位置不会跳。
                transform = dict(carried)
                manager = httpd.manager
            else:
                # 日志里没有地图：给一块没有边界、没有障碍的空白场地（机器人摆到
                # 场地中心），不限制它活动；前一张地图连同其历史点云一起作废。
                bounds = (info.get("summary") or {}).get("bounds") or {}
                span_x = max(0.0, float(bounds.get("x1", 0.0)) - float(bounds.get("x0", 0.0)))
                span_y = max(0.0, float(bounds.get("y1", 0.0)) - float(bounds.get("y0", 0.0)))
                definition = map_import.build_open_field(
                    span_x, span_y, name="{}·无地图".format(label)
                )
        if definition is not None:
            try:
                transform = definition.get("replay_transform")
                manager = rebuild_system(httpd, definition)
            except Exception as exc:  # 场地构造失败时退回当前地图
                print("日志场地构造失败：{}".format(exc))
                transform = None
                manager = httpd.manager
    manager.load_replay(info, transform)
    replay = manager.replay
    if getattr(httpd, "bag_queue", None) is queue:
        # 载入期间可能已经起了新队列：不是同一个队列就不写，免得污染新队列
        before = queue.get("frame_transform")
        _remember_frame(queue, replay, use_log_map)
        if _frame_changed(before, queue.get("frame_transform")):
            # 场地换了（清成空白地图 / 换了一张日志地图）：上一张图上的轨迹与历史
            # 点云都作废——`world.epoch` 变了，前端据此重新起笔
            world.bump_map_epoch()
            clear_cloud_history(httpd)
    if replay is not None and replay.track:
        # 起点/车体跟着日志第一帧走；**充电桩不要跟着动** —— 之前这里把桩直接搬到
        # 回放起点，连续播放时每换一个包桩就跳一次，"充电桩老是换地方"就是这么来的。
        first = replay.track[0]
        pose = (round(first[1], 4), round(first[2], 4), round(first[3], 4))
        world.START.update({"x": pose[0], "y": pose[1], "theta": pose[2]})
        manager.robot.pose = [pose[0], pose[1], pose[2]]
    apply_log_dock(info, manager, use_log_map)
    return manager, info, path


def apply_log_dock(info, manager, use_log_map=True):
    """充电桩摆放：**日志里明确给了桩位才画**，否则回放期间图上不放桩。

    * 日志里有桩位话题（`bag_reader.DOCK_HINTS`）→ 按日志坐标换算到仿真坐标摆放；
    * 没有 → `world.DOCK_VISIBLE = False`（内部 DOCK 仍保留一个锚点给任务用）；
    * 回放跑在仿真自己的场地上（`use_log_map=False`）时，桩是真实的，照常显示。
    """
    replay = getattr(manager, "replay", None)
    dock = (info or {}).get("dock")
    if not use_log_map:
        world.DOCK_VISIBLE = True
        return False
    if not dock:
        world.DOCK_VISIBLE = False
        return False
    scale = replay.scale if replay is not None and replay.transform else 1.0
    if replay is not None and replay.transform:
        dx, dy = replay.map_offset
    elif replay is not None:
        dx, dy = replay.offset
    else:
        dx = dy = 0.0
    world.DOCK.update(
        {
            "x": round(float(dock[0]) * scale + dx, 3),
            "y": round(float(dock[1]) * scale + dy, 3),
            "r": world.DOCK_RADIUS,
        }
    )
    world.DOCK_VISIBLE = True
    return True


# 连续播放时历史感知点云一直留着：换包只换"当前这一段"，前面的点云不清。
# 界面上的「清理地图」才会真正清掉（顺便清掉轨迹）。
CLOUD_HISTORY_POINTS = 24000
CLOUD_MAX_POINTS = 6000
# 历史点云（连续播放时前面几段日志）统一按"半程年龄"淡出：看得见但压不当前段的新点


def cloud_history(server):
    history = getattr(server, "bag_cloud_history", None)
    if history is None:
        history = []
        server.bag_cloud_history = history
    return history


def history_points(chunk):
    """历史条目的点（兼容旧的"裸点列表"写法）。"""
    if isinstance(chunk, dict):
        return chunk.get("points") or []
    return chunk or []


def history_base_wall(chunk):
    """这条历史的"基准墙钟"：``base - ages[i]`` 就是第 i 个点的观测时刻。"""
    if isinstance(chunk, dict):
        return chunk.get("base")
    return None


def history_ages(chunk):
    if isinstance(chunk, dict):
        return chunk.get("ages") or []
    return []


def archive_replay_cloud(server):
    """换包前把当前这段日志的点云整段存进历史。"""
    manager = getattr(server, "manager", None)
    replay = getattr(manager, "replay", None) if manager is not None else None
    if replay is None:
        return 0
    try:
        # 只归档"当前留存窗口内"的点（以前用 inf，会把整包 150 s 的点都存进来，
        # 存的时候又统一盖成"换包那一刻"的年龄，于是这些点到了下一包永远不会过期）
        cloud = replay.cloud_at(float(replay.time), max_points=6000)
    except Exception:
        return 0
    points = (cloud or {}).get("points") or []
    ages = list((cloud or {}).get("ages") or [])
    if not points:
        return 0
    # 每个点的观测墙钟 = base - ages[i]；跨包老化按真实时间算
    wall_start = getattr(replay, "wall_start", None)
    base = round(float(wall_start) + float(replay.time), 3) if wall_start else None
    history = cloud_history(server)
    history.append(
        {
            "points": [float(value) for value in points],
            "ages": [float(value) for value in ages],
            "base": base,
            "name": replay.name,
        }
    )
    total = sum(len(history_points(chunk)) // 2 for chunk in history)
    while len(history) > 1 and total > CLOUD_HISTORY_POINTS:
        total -= len(history_points(history.pop(0))) // 2
    return len(points) // 2


def clear_cloud_history(server):
    server.bag_cloud_history = []


def _downsample_cloud(points, max_points, ages=None):
    total = len(points) // 2
    if total <= max_points or max_points <= 0:
        return points, total, ages if ages is not None else []
    stride = int(math.ceil(total / float(max_points)))
    picked = []
    picked_ages = [] if ages is not None else None
    for index in range(0, total, stride):
        picked.append(points[index * 2])
        picked.append(points[index * 2 + 1])
        if picked_ages is not None:
            picked_ages.append(ages[index] if index < len(ages) else 0.0)
    return picked, total, picked_ages if picked_ages is not None else []


def combined_cloud(server, time_value):
    """历史点云（前面几段日志）+ 当前日志截至 ``time_value`` 的点云。

    历史点云同样受 `CLOUD_FADE_SECONDS` 约束：按**墙钟**算年龄（每段日志录制时刻
    在日志头里），超过留存窗口的直接不返回——以前给它一个固定的"半程年龄"，
    换成下一包以后这些点就永远不会消失了。
    """
    manager = getattr(server, "manager", None)
    replay = getattr(manager, "replay", None) if manager is not None else None
    history = list(cloud_history(server))
    current = None
    wall_now = None
    if replay is not None:
        try:
            current = replay.cloud_at(time_value)
        except Exception:
            current = None
        # 用查询时刻的墙钟（和 cloud_at 的 time_value 对齐），日志没有墙钟就算不出来
        wall_start = getattr(replay, "wall_start", None)
        if wall_start:
            wall_now = float(wall_start) + float(time_value)
    current_points = list((current or {}).get("points") or [])
    current_ages = list((current or {}).get("ages") or [])
    fade = bag_player.CLOUD_FADE_SECONDS
    points = []
    ages = []
    bags = 0
    for chunk in history:
        chunk_points = list(history_points(chunk))
        if not chunk_points:
            continue
        base = history_base_wall(chunk)
        if base is None or wall_now is None:
            # 没有墙钟（老格式/测试里手工塞的裸点）→ 无法判断年龄，按已过期处理
            continue
        chunk_ages = list(history_ages(chunk))
        kept = 0
        for index in range(len(chunk_points) // 2):
            seen_age = chunk_ages[index] if index < len(chunk_ages) else 0.0
            # 这个点是 base - seen_age 时刻观测的，到"现在"多老了
            age = max(0.0, float(wall_now) - (float(base) - float(seen_age)))
            if age >= fade:
                continue
            points.append(chunk_points[index * 2])
            points.append(chunk_points[index * 2 + 1])
            ages.append(round(age, 1))
            kept += 1
        if kept:
            bags += 1
    history_total = len(points) // 2
    if not points and not current_points:
        return None
    points.extend(current_points)
    ages.extend(current_ages[: len(current_points) // 2])
    picked, total, picked_ages = _downsample_cloud(points, CLOUD_MAX_POINTS, ages)
    stride = 1
    if total > CLOUD_MAX_POINTS > 0:
        stride = int(math.ceil(total / float(CLOUD_MAX_POINTS)))
    # 历史点排在最前面，按同样的步长抽稀后还剩多少个
    history_kept = int(math.ceil(history_total / float(stride))) if history_total else 0
    return {
        "points": picked,
        "ages": picked_ages,
        "fade": fade,
        "count": len(picked) // 2,
        "total": total,
        "frames": int((current or {}).get("frames") or 0),
        "history": history_kept,
        "bags": bags,
    }


def bag_queue_snapshot(server):
    queue = getattr(server, "bag_queue", None) or {}
    names = list(queue.get("names") or [])
    index = int(queue.get("index") or 0)
    active = bool(queue.get("active"))
    current = names[index] if 0 <= index < len(names) else None
    return {
        "active": active,
        "names": names,
        "index": index,
        "total": len(names),
        "current": current if active else None,
        "use_log_map": bool(queue.get("use_log_map", True)),
        "loading": bool(queue.get("loading")),
        "failed": list(queue.get("failed") or []),
    }


def _queue_worker(server, owner=None):
    """后台线程：载入队列里的下一条日志并自动播放。

    单条日志载入失败只跳过这一条，不会把整个队列终止掉（以前一条坏包就把
    “连续播放”整条打死，界面看起来就是“有时候播到一半就不动了”）。
    ``owner`` 是发起这次载入的队列对象：期间如果又起了新队列（或按了停止），
    旧线程收尾时不能去改新队列的 ``loading``，否则新队列会被误判成“已在载入”。
    """
    if owner is None:
        owner = getattr(server, "bag_queue", None)
    try:
        while True:
            queue = getattr(server, "bag_queue", None)
            if queue is None or queue is not owner or not queue.get("active"):
                return
            names = queue.get("names") or []
            index = int(queue.get("index") or 0)
            if index >= len(names):
                queue["active"] = False
                return
            try:
                if index > 0:
                    # 换包前把上一段日志的感知点云存进历史：连续播放不清历史点云
                    archive_replay_cloud(server)
                manager, _info, _path = load_bag(
                    server, names[index], queue.get("use_log_map", True)
                )
                manager.start_replay()
                return
            except Exception as exc:
                print("连续播放失败（{}）：{}".format(names[index], exc))
                failed = queue.setdefault("failed", [])
                if names[index] not in failed:
                    failed.append(names[index])
                queue["index"] = index + 1
                if queue["index"] >= len(names):
                    queue["active"] = False
                    return
    finally:
        if getattr(server, "bag_queue", None) is owner:
            owner["loading"] = False


def start_bag_queue(server, names, use_log_map=True):
    """按顺序连续播放多个日志（每个播完自动切下一个）。"""
    cleaned = [str(item) for item in (names or []) if str(item).strip()]
    if not cleaned:
        raise ValueError("请先勾选要连续播放的日志")
    failed = []
    playable = []
    for name in cleaned:
        try:
            resolve_bag(server, name)
            playable.append(name)
        except Exception:
            # 找不到的文件只跳过它，不要让整条队列起不来
            if name not in failed:
                failed.append(name)
    if not playable:
        raise ValueError("勾选的日志都找不到，无法连续播放")
    server.bag_queue = {
        "names": playable,
        "index": 0,
        "active": True,
        "use_log_map": bool(use_log_map),
        "loading": True,
        "failed": failed,
    }
    threading.Thread(
        target=_queue_worker, args=(server, server.bag_queue), daemon=True
    ).start()
    return bag_queue_snapshot(server)


def poll_bag_queue(server):
    """仿真每轮调用：当前日志播完就切队列里的下一条。"""
    queue = getattr(server, "bag_queue", None)
    if not queue or not queue.get("active") or queue.get("loading"):
        return
    manager = server.manager
    replay = getattr(manager, "replay", None)
    if replay is None or not replay.finished:
        return
    queue["index"] = int(queue.get("index") or 0) + 1
    if queue["index"] >= len(queue.get("names") or []):
        queue["active"] = False
        return
    queue["loading"] = True
    threading.Thread(target=_queue_worker, args=(server, queue), daemon=True).start()


class Simulator(threading.Thread):
    """固定步长推进仿真；speed 仅改变每帧执行的步数，保证控制稳定。"""

    def __init__(self, manager, tick=SIM_DT):
        super().__init__(name="robot-simulator", daemon=True)
        self.manager = manager
        self.tick_size = tick
        self.speed = 1
        self.lock = threading.RLock()
        self._stop_event = threading.Event()
        self.on_cycle = None

    def replace_manager(self, manager):
        with self.lock:
            self.manager = manager

    def step_size(self):
        """按倍速自适应积分步长：高倍速时用更大步长，避免每个 20 ms 都跑一遍全量计算。

        一步的算量几乎固定（41 条视觉射线 + 栅格写入），所以倍速能跑到多少
        基本取决于步长。实测步长再放大到 0.04 s 会让窄走廊里频繁撞墙
        （导入地图整场碰撞上千次），0.033 s 是安全上限。
        """
        if self.speed <= 4:
            return self.tick_size
        if self.speed <= 16:
            return max(self.tick_size, 0.025)
        return max(self.tick_size, 0.033)

    def run(self):
        last = time.monotonic()
        accumulator = 0.0
        while not self._stop_event.is_set():
            now = time.monotonic()
            elapsed = min(0.2, max(0.0, now - last))
            last = now
            tick = self.step_size()
            accumulator += elapsed * self.speed
            # 卡顿后不要一次性补太多仿真时间（最多补 4 步），避免"时间突进"
            if accumulator > tick * 4:
                accumulator = tick * 4
            steps = 0
            budget = max(8, int(0.25 / tick))
            try:
                with self.lock:
                    manager = self.manager
                    while accumulator >= tick and steps < budget:
                        manager.tick(tick)
                        accumulator -= tick
                        steps += 1
            except Exception:
                traceback.print_exc()
            if self.on_cycle is not None:
                try:
                    self.on_cycle()
                except Exception:
                    traceback.print_exc()
            # 只在"攒不够下一步的仿真时间"时才睡，且按还差多少墙钟时间精确睡。
            # 原来固定睡 1 ms：光睡眠就让每轮至少 1 ms，一轮又只跑一步 33 ms 仿真，
            # 32 倍速下实测只能跑到十几倍（大实景地图更慢），倍速形同虚设。
            if accumulator < tick:
                wait = (tick - accumulator) / float(max(1, self.speed))
                self._stop_event.wait(min(0.004, max(0.0002, wait)))

    def stop(self):
        self._stop_event.set()


class RobotRequestHandler(BaseHTTPRequestHandler):
    server_version = "RobotCar/2.0"
    protocol_version = "HTTP/1.1"

    # --------------------------------------------------------------- 路由
    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/api/state":
            query = parse_qs(urlsplit(self.path).query)
            with_map = (query.get("map") or ["1"])[0] not in ("0", "false", "")
            # 前端带上自己手上那张位图的指纹：没变就不重发栅格字符画
            known = (query.get("map_key") or [None])[0] or None
            self._send_json(self._state(include_map=with_map, map_key=known))
        elif path == "/api/world":
            self._send_json(world.world_dict())
        elif path == "/api/bag":
            manager = self.server.manager
            entries = list_bag_entries(self.server)
            groups = []
            for entry in entries:
                if entry["group"] not in groups:
                    groups.append(entry["group"])
            self._send_json(
                {
                    "directory": BAG_DIR,
                    "roots": list(bag_roots(self.server)),
                    "groups": groups,
                    "bags": entries,
                    "queue": bag_queue_snapshot(self.server),
                    "state": manager.replay_state(),
                    "topics": (manager.replay.info or {}).get("topics") or [],
                    "path": manager.replay.reference_path(),
                }
            )
        elif path == "/api/bag/frame":
            query = parse_qs(urlsplit(self.path).query)
            try:
                when = float((query.get("t") or ["0"])[0])
            except ValueError:
                when = 0.0
            self._send_json({"image": self.server.manager.replay.image_at(when)})
        elif path == "/api/bag/costmap":
            query = parse_qs(urlsplit(self.path).query)
            time_value = (query.get("t") or ["0"])[0]
            try:
                time_value = float(time_value)
            except ValueError:
                time_value = 0.0
            replay = self.server.manager.replay
            layer = replay.costmap_at(time_value)
            self._send_json(
                {"costmap": layer, "maps": (replay.info or {}).get("map_summary") or {}}
            )
        elif path == "/api/bag/cloud":
            query = parse_qs(urlsplit(self.path).query)
            try:
                time_value = float((query.get("t") or ["0"])[0])
            except ValueError:
                time_value = 0.0
            self._send_json({"cloud": combined_cloud(self.server, time_value)})
        elif path == "/api/reports":
            entries = list_reports()
            self._send_json(
                {
                    "directory": report_dir(),
                    "reports": entries,
                    "latest": entries[0]["url"] if entries else None,
                }
            )
        elif path == "/api/config":
            self._send_json(self._state())
        elif path == "/api/health":
            self._send_json(
                {"status": "ok", "time": round(time.time(), 3), "speed": self.server.simulator.speed}
            )
        elif path.startswith("/reports/"):
            self._serve_report(path[len("/reports/") :])
        elif path.startswith("/api/"):
            self._send_json({"error": "接口不存在"}, status=404)
        else:
            self._serve_static(path)

    def do_POST(self):
        path = urlsplit(self.path).path
        if not path.startswith("/api/"):
            self._send_json({"error": "接口不存在"}, status=404)
            return
        try:
            limit = (
                MAX_UPLOAD_BODY_BYTES
                if path == "/api/map/upload"
                else MAX_BODY_BYTES
            )
            payload = self._read_json(limit)
        except ValueError as exc:
            self._send_json({"error": str(exc)}, status=400)
            return

        manager = self.server.manager
        try:
            if path == "/api/command":
                if manager.task is not None:
                    manager.stop_task("手动接管，当前任务已取消", resumable=False)
                # 手动接管后电量按仿真正常放电（回放时钉在日志值上的锁要解开）
                manager.release_battery_lock()
                self.server.robot.handle_command(
                    payload.get("command"),
                    payload.get("linear"),
                    payload.get("angular"),
                )
            elif path == "/api/task":
                if getattr(self.server, "editor", None) is not None:
                    raise ValueError("地图编辑中：请先保存地图或取消编辑，再开始任务")
                # 开始任务是仿真自己的作业，桩照常显示（回放里"日志没给桩位"的状态到此结束）
                world.DOCK_VISIBLE = True
                task_type = payload.get("task")
                if task_type == "map":
                    manager.start_mapping()
                elif task_type == "mow":
                    manager.start_mowing(payload.get("zones"), payload.get("mode"))
                else:
                    raise ValueError("未知任务类型: {}".format(task_type))
            elif path == "/api/stop":
                manager.stop_task("控制台停止任务")
            elif path == "/api/charge":
                manager.request_charge()
            elif path == "/api/resume":
                manager.request_resume()
            elif path == "/api/reset":
                self.server.bag_queue = None
                clear_cloud_history(self.server)
                world.DOCK_VISIBLE = True
                manager.clear_replay()
                self.server.robot.reset()
                self.server.grid.reset_map()
                manager.reset()
            elif path == "/api/map":
                if getattr(self.server, "editor", None) is not None:
                    raise ValueError("地图编辑中：请先保存或退出编辑，再切换地图")
                map_id = int(payload.get("map", 1))
                if map_id not in world.MAPS:
                    raise ValueError("未知地图: {}".format(map_id))
                clear_cloud_history(self.server)
                with self.server.simulator.lock:
                    world.select_map(map_id)
                    robot = RobotCar()
                    grid = GridMap()
                    manager = build_manager(robot, grid)
                    self.server.robot = robot
                    self.server.grid = grid
                    self.server.manager = manager
                    self.server.simulator.manager = manager
            elif path == "/api/map/edit":
                action = str(payload.get("action") or "")
                kind = str(payload.get("kind") or "")
                manager = self.server.manager
                if not manager.grid.zones:
                    raise ValueError("请先完成自动建图，再编辑地图")
                busy = (
                    manager.task is not None
                    and manager.phase not in ("idle", "done", "stopped")
                )
                if busy:
                    raise ValueError("任务进行中不能编辑地图，请先停止任务")
                if action == "update_dynamic":
                    world.update_dynamic_path(
                        payload.get("id", 0),
                        payload.get("ax", 0.0),
                        payload.get("ay", 0.0),
                        payload.get("bx", 0.0),
                        payload.get("by", 0.0),
                    )
                elif action == "add_static":
                    world.add_static_obstacle(
                        float(payload.get("x", 0.0)),
                        float(payload.get("y", 0.0)),
                        float(payload.get("r", 0.35)),
                    )
                elif action == "add_dynamic":
                    world.add_dynamic_obstacle(
                        float(payload.get("x", 0.0)),
                        float(payload.get("y", 0.0)),
                        float(payload.get("r", 0.30)),
                        float(payload.get("span", 1.6)),
                        float(payload.get("heading", 0.0)),
                    )
                elif action == "add_no_go":
                    if payload.get("x0") is not None and payload.get("y0") is not None:
                        world.add_no_go_zone(
                            float(payload.get("x0", 0.0)),
                            float(payload.get("y0", 0.0)),
                            float(payload.get("x1", 0.0)),
                            float(payload.get("y1", 0.0)),
                        )
                    else:
                        # 兼容旧调用：以圆（中心 + 半径）描述，转成外接正方形
                        radius = float(payload.get("r", 1.0))
                        world.add_no_go_zone(
                            float(payload.get("x", 0.0)) - radius,
                            float(payload.get("y", 0.0)) - radius,
                            float(payload.get("x", 0.0)) + radius,
                            float(payload.get("y", 0.0)) + radius,
                        )
                elif action == "delete":
                    world.remove_edit(kind, payload.get("id", 0))
                elif action == "clear":
                    world.clear_edits(kind or None)
                else:
                    raise ValueError("未知的地图编辑动作: {}".format(action))
                with self.server.simulator.lock:
                    self.server.grid.apply_no_go(world.NO_GO)
                    self.server.grid.version += 1
            elif path == "/api/map/delete":
                map_id = int(payload.get("map", 0))
                world.delete_map(map_id)
                with self.server.simulator.lock:
                    if world.MAP_ID == map_id:
                        world.select_map(1)
                        robot = RobotCar()
                        grid = GridMap()
                        manager = build_manager(robot, grid)
                        self.server.robot = robot
                        self.server.grid = grid
                        self.server.manager = manager
                        self.server.simulator.manager = manager
            elif path.startswith("/api/editor/"):
                action = path[len("/api/editor/"):]
                if action == "blank":
                    if _editor_busy(manager):
                        raise ValueError("任务进行中不能编辑地图，请先停止任务")
                    draft = start_editor(
                        self.server, payload.get("name"), payload.get("map")
                    )
                    self._send_json(
                        {
                            "draft": draft.state(),
                            "maps": world.world_dict().get("maps", []),
                        }
                    )
                    return
                if action == "state":
                    self._send_json({"draft": _editor_snapshot(self.server)})
                    return
                if action == "undo":
                    draft = _editor_require(self.server)
                    left = draft.undo()
                    self._send_json({"draft": draft.state(), "remaining": left})
                    return
                if action == "discard":
                    self._send_json(
                        {
                            "draft": close_editor(self.server),
                            "maps": world.world_dict().get("maps", []),
                        }
                    )
                    return
                if _editor_busy(manager):
                    raise ValueError("任务进行中不能编辑地图，请先停止任务")
                if action == "region":
                    draft = _editor_require(self.server)
                    result = draft.add_region(payload.get("points") or [])
                    self._send_json({"draft": draft.state(), "region": result})
                    return
                if action == "passage":
                    draft = _editor_require(self.server)
                    result = draft.add_passage(payload.get("points") or [])
                    self._send_json({"draft": draft.state(), "passage": result})
                    return
                if action == "dock":
                    draft = _editor_require(self.server)
                    result = draft.add_dock(
                        payload.get("x", 0.0), payload.get("y", 0.0), payload.get("r")
                    )
                    self._send_json({"draft": draft.state(), "dock": result})
                    return
                if action == "no_go":
                    draft = _editor_require(self.server)
                    result = draft.add_no_go(
                        payload.get("x0", 0.0),
                        payload.get("y0", 0.0),
                        payload.get("x1", 0.0),
                        payload.get("y1", 0.0),
                    )
                    self._send_json({"draft": draft.state(), "no_go": result})
                    return
                if action == "obstacle":
                    draft = _editor_require(self.server)
                    kind = str(payload.get("kind") or "static")
                    if kind == "dynamic":
                        added = draft.add_dynamic(
                            payload.get("x", 0.0),
                            payload.get("y", 0.0),
                            payload.get("r"),
                            payload.get("span"),
                            payload.get("heading", 0.0),
                            payload.get("ax"),
                            payload.get("ay"),
                            payload.get("bx"),
                            payload.get("by"),
                            payload.get("speed"),
                        )
                    elif kind == "polygon":
                        added = draft.add_obstacle_polygon(payload.get("points") or [])
                    else:
                        added = draft.add_static(
                            payload.get("points") or [], payload.get("r")
                        )
                    self._send_json({"draft": draft.state(), "added": added})
                    return
                if action == "dynamic":
                    draft = _editor_require(self.server)
                    item = draft.update_dynamic(
                        payload.get("id"),
                        payload.get("ax"),
                        payload.get("ay"),
                        payload.get("bx"),
                        payload.get("by"),
                        payload.get("speed"),
                        payload.get("r"),
                    )
                    self._send_json({"draft": draft.state(), "dynamic": item})
                    return
                if action == "delete":
                    draft = _editor_require(self.server)
                    if payload.get("x") is not None and payload.get("y") is not None:
                        target = draft.hit_test(payload.get("x"), payload.get("y"))
                        if target is None:
                            raise ValueError("这个位置没有可删除的元素")
                    else:
                        target = {
                            "kind": payload.get("kind"),
                            "id": payload.get("id"),
                        }
                    result = draft.delete(target["kind"], target["id"])
                    self._send_json({"draft": draft.state(), "result": result})
                    return
                if action == "save":
                    result = save_editor(self.server, payload.get("name"))
                    result["maps"] = world.world_dict().get("maps", [])
                    self._send_json(result)
                    return
                raise ValueError("未知的地图编辑动作: {}".format(action))
            elif path == "/api/map/scan":
                root = str(payload.get("path") or "").strip() or None
                max_extent = payload.get("max_extent", 40.0)
                try:
                    max_extent = float(max_extent)
                except (TypeError, ValueError):
                    raise ValueError("地图最大边长必须是数字")
                if not (8.0 <= max_extent <= 120.0):
                    raise ValueError("地图最大边长范围 8-120 m")
                self._send_json(
                    import_all_maps(
                        self.server,
                        root,
                        max_extent=max_extent,
                        select_first=bool(payload.get("select_first", True)),
                    )
                )
                return
            elif path == "/api/map/import":
                source = str(payload.get("path") or "").strip()
                if not source:
                    raise ValueError("请填写真实地图目录或 PGM 文件路径")
                max_extent = payload.get("max_extent", 40.0)
                try:
                    max_extent = float(max_extent)
                except (TypeError, ValueError):
                    raise ValueError("地图最大边长必须是数字")
                if not (8.0 <= max_extent <= 120.0):
                    raise ValueError("地图最大边长范围 8-120 m")
                definition = map_import.build_map(
                    source, payload.get("name"), max_extent=max_extent
                )
                with self.server.simulator.lock:
                    map_id = world.register_map(definition)
                    world.select_map(map_id)
                    robot = RobotCar()
                    grid = GridMap()
                    manager = build_manager(robot, grid)
                    self.server.robot = robot
                    self.server.grid = grid
                    self.server.manager = manager
                    self.server.simulator.manager = manager
            elif path == "/api/map/upload":
                max_extent = payload.get("max_extent", 40.0)
                try:
                    max_extent = float(max_extent)
                except (TypeError, ValueError):
                    raise ValueError("地图最大边长必须是数字")
                if not (8.0 <= max_extent <= 120.0):
                    raise ValueError("地图最大边长范围 8-120 m")
                root = _store_uploaded_map(payload.get("files"))
                definition = map_import.build_map(
                    root, payload.get("name"), max_extent=max_extent
                )
                with self.server.simulator.lock:
                    map_id = world.register_map(definition)
                    world.select_map(map_id)
                    robot = RobotCar()
                    grid = GridMap()
                    manager = build_manager(robot, grid)
                    self.server.robot = robot
                    self.server.grid = grid
                    self.server.manager = manager
                    self.server.simulator.manager = manager
            elif path == "/api/config":
                if manager.task is not None and manager.phase not in ("idle", "done", "stopped"):
                    raise ValueError("请先暂停或结束任务，再修改参数")
                payload.pop("reset", None)
                if not payload:
                    raise ValueError("没有可应用的参数")
                config.configure(**payload)
                manager.mowing_mode = config.settings.mowing_mode
            elif path == "/api/config/reset":
                if manager.task is not None and manager.phase not in ("idle", "done", "stopped"):
                    raise ValueError("请先暂停或结束任务，再重置参数")
                config.reset()
                manager.mowing_mode = config.settings.mowing_mode
            elif path.startswith("/api/bag/"):
                action = path[len("/api/bag/"):]
                if action == "load":
                    name = str(payload.get("name") or "").strip()
                    if not name:
                        raise ValueError("请选择日志文件")
                    self.server.bag_queue = None
                    # 手动载入日志：历史点云与轨迹一起重新起笔
                    clear_cloud_history(self.server)
                    manager, info, bag_path = load_bag(
                        self.server, name, payload.get("use_log_map", True)
                    )
                    entries = list_bag_entries(self.server)
                    self._send_json(
                        {
                            "directory": BAG_DIR,
                            "roots": list(bag_roots(self.server)),
                            "groups": sorted({item["group"] for item in entries}),
                            "bags": entries,
                            "queue": bag_queue_snapshot(self.server),
                            "state": manager.replay_state(),
                            "path": manager.replay.reference_path(),
                            "topics": info.get("topics"),
                            "summary": info.get("summary"),
                            "bag_path": bag_path,
                        }
                    )
                    return
                if action == "dir":
                    requested = str(payload.get("path") or "").strip()
                    add_bag_root(self.server, requested)
                    entries = list_bag_entries(self.server)
                    self._send_json(
                        {
                            "roots": list(bag_roots(self.server)),
                            "groups": sorted({item["group"] for item in entries}),
                            "bags": entries,
                            "added": os.path.abspath(os.path.expanduser(requested)),
                        }
                    )
                    return
                if action == "playlist":
                    snapshot = start_bag_queue(
                        self.server,
                        payload.get("names") or payload.get("name"),
                        payload.get("use_log_map", True),
                    )
                    self._send_json({"queue": snapshot, "state": manager.replay_state()})
                    return
                if action == "cloud/clear":
                    # 「清理地图」：把累积的历史点云清掉（轨迹在界面上一起清）
                    cleared = sum(len(chunk) // 2 for chunk in cloud_history(self.server))
                    clear_cloud_history(self.server)
                    self._send_json({"ok": True, "cleared": cleared})
                    return
                if action == "control":
                    command = str(payload.get("command") or "")
                    with self.server.simulator.lock:
                        if command == "play":
                            self.server.bag_queue = None
                            manager.start_replay()
                        elif command == "pause":
                            manager.pause_replay()
                        elif command == "resume":
                            manager.resume_replay()
                        elif command == "stop":
                            self.server.bag_queue = None
                            manager.stop_replay()
                        elif command == "seek":
                            manager.seek_replay(payload.get("time", 0.0))
                        else:
                            raise ValueError("未知的日志播放指令: {}".format(command))
                    self._send_json(
                        {
                            "state": manager.replay_state(),
                            "path": manager.replay.reference_path(),
                        }
                    )
                    return
                raise ValueError("接口不存在")
            elif path == "/api/shutdown":
                manager.stop_task("控制台退出，服务即将关闭", resumable=False)
                self._send_json({"ok": True, "message": "服务已关闭，可以关闭此页面"})
                self.server.stop_service()
                return
            elif path == "/api/speed":
                speed = int(payload.get("speed", 1))
                if speed not in (1, 2, 4, 8, 16, 32):
                    raise ValueError("仿真速度仅支持 1/2/4/8/16/32")
                self.server.simulator.speed = speed
            else:
                self._send_json({"error": "接口不存在"}, status=404)
                return
        except ValueError as exc:
            # 地图编辑的错误带错误码（E01…），单独回给前端方便显示/排查
            payload = {"error": str(exc)}
            if getattr(exc, "code", None):
                payload["code"] = exc.code
                payload["number"] = getattr(exc, "number", None)
            self._send_json(payload, status=400)
            return
        except Exception as exc:  # pragma: no cover - 兜底：内部异常也要给出可读反馈
            # 以前这里会把异常抛给 socketserver，浏览器只看到连接被重置/500 空响应，
            # 界面上就是"点了按钮没反应"。现在回一条 JSON 错误并把堆栈打进服务日志。
            traceback.print_exc()
            self._send_json(
                {"error": "处理 {} 时出错：{}".format(path, exc)}, status=500
            )
            return

        self._send_json(self._state())

    # --------------------------------------------------------------- 工具
    def _state(self, include_map=True, map_key=None):
        # 编辑草稿是一块 100 × 100 m 的空场地（100 万格），栅格字符画没有意义，
        # 也不该每几十毫秒往浏览器传一次，编辑期间直接不取栅格。
        if getattr(self.server, "editor", None) is not None:
            include_map = False
        state = self.server.manager.get_state(
            include_map=include_map, map_key=map_key
        )
        state["speed"] = self.server.simulator.speed
        state["map_id"] = world.MAP_ID
        state["map_name"] = world.MAP_NAME
        # 回放时日志里没有明确桩位 → 界面上不放桩，遥测里的"距桩/回充预留"也留空
        state["dock_visible"] = world.DOCK_VISIBLE
        state["config"] = dict(config.settings.__dict__)
        state["bag"] = self.server.manager.replay_state()
        # 连续播放的状态也放进主状态里：界面的轨迹逻辑要按它判断"是不是在连续播"，
        # 只靠 /api/bag 轮询会慢半拍，切包那一瞬间就以为用户手动换了日志。
        state["queue"] = bag_queue_snapshot(self.server)
        state["draft"] = _editor_snapshot(self.server)
        return state

    def _read_json(self, limit=MAX_BODY_BYTES):
        try:
            length = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            raise ValueError("Content-Length 无效")
        if length <= 0:
            return {}
        if length > limit:
            raise ValueError("请求体过大")
        raw = self.rfile.read(length)
        try:
            data = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            raise ValueError("请求体不是合法的 JSON")
        if not isinstance(data, dict):
            raise ValueError("请求体必须是 JSON 对象")
        return data

    def _send_json(self, payload, status=200):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        encoding = None
        # 大实景地图的 /api/state 有 300+ KB（栅格是 15 万格的字符画），
        # 高倍速时前端几十毫秒轮询一次，光传输和 JSON.parse 就把两边都拖住。
        # 栅格字符画压缩率极高（300 KB → 4 KB），开 gzip 后轮询几乎不占带宽。
        if len(body) > 2048 and "gzip" in (self.headers.get("Accept-Encoding") or ""):
            # 级别 1 就够：栅格字符画重复度极高，1 级的体积和 6 级差不多，
            # 但 CPU 只要十分之一，高倍速轮询时不跟仿真线程抢 GIL。
            # 注意必须是 gzip 流（zlib.compress 带的是 zlib 头，浏览器会解码失败）。
            body = gzip.compress(body, 1)
            encoding = "gzip"
        self.send_response(status)
        self.send_header("Content-Type", CONTENT_TYPES[".json"])
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        if encoding:
            self.send_header("Content-Encoding", encoding)
            self.send_header("Vary", "Accept-Encoding")
        self.end_headers()
        self.wfile.write(body)

    def _serve_static(self, path):
        relative = unquote(path).lstrip("/") or "index.html"
        if ".." in relative.split("/"):
            self._send_text("禁止访问", status=403)
            return
        full_path = os.path.normpath(os.path.join(FRONTEND_DIR, relative))
        if not (full_path == FRONTEND_DIR or full_path.startswith(FRONTEND_DIR + os.sep)):
            self._send_text("禁止访问", status=403)
            return
        if os.path.isdir(full_path):
            full_path = os.path.join(full_path, "index.html")
        if not os.path.isfile(full_path):
            self._send_text("页面不存在", status=404)
            return
        with open(full_path, "rb") as handle:
            body = handle.read()
        extension = os.path.splitext(full_path)[1].lower()
        self.send_response(200)
        self.send_header("Content-Type", CONTENT_TYPES.get(extension, "application/octet-stream"))
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(body)

    def _serve_report(self, relative):
        """把 data/data 下的割草报告发出去；/reports/latest 指向最新一份。"""
        root = report_dir()
        name = unquote(relative or "").strip("/") or "latest"
        if name == "latest":
            entries = list_reports(1)
            if not entries:
                self._send_text("还没有生成过割草报告：先跑一次自动割草", status=404)
                return
            name = entries[0]["html"]
        name = name.strip("/")
        if not name:
            self._send_text("报告名不能为空", status=400)
            return
        if ".." in name.split("/"):
            self._send_text("禁止访问", status=403)
            return
        full_path = os.path.normpath(os.path.join(root, name))
        if not (full_path == root or full_path.startswith(root + os.sep)):
            self._send_text("禁止访问", status=403)
            return
        if not os.path.isfile(full_path):
            self._send_text("报告不存在", status=404)
            return
        with open(full_path, "rb") as handle:
            body = handle.read()
        extension = os.path.splitext(full_path)[1].lower()
        self.send_response(200)
        self.send_header("Content-Type", CONTENT_TYPES.get(extension, "application/octet-stream"))
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(body)

    def _send_text(self, text, status=200):
        body = text.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        if os.environ.get("ROBOT_HTTP_DEBUG") == "1":
            super().log_message(fmt, *args)


class RobotHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def stop_service(self):
        """退出服务：停掉仿真线程并关闭监听端口（可从请求线程调用）。

        ``shutdown()`` 会等 ``serve_forever()`` 退出，所以必须放在独立线程里，
        并稍等片刻让 HTTP 响应先发回浏览器。
        """

        def _shutdown():
            time.sleep(0.15)
            try:
                self.simulator.stop()
            except Exception:
                pass
            self.shutdown()

        threading.Thread(target=_shutdown, name="robot-shutdown", daemon=True).start()


def build_manager(robot, grid):
    """创建任务管理器（纯 Python 规划/控制）。"""
    return TaskManager(robot, grid)


def _editor_busy(manager):
    """任务进行中不能改地图（自动建图 / 割草 / 日志回放 都不行）。"""
    if manager.task is not None and manager.phase not in ("idle", "done", "stopped"):
        return True
    replay = getattr(manager, "replay", None)
    return bool(replay is not None and getattr(replay, "active", False))


def _editor_require(httpd):
    draft = getattr(httpd, "editor", None)
    if draft is None:
        raise ValueError("请先点击“空白地图”开始编辑")
    return draft


def _editor_snapshot(httpd):
    draft = getattr(httpd, "editor", None)
    return draft.state() if draft is not None else {"active": False}


def start_editor(httpd, name=None, map_id=None):
    """进入编辑模式：切到一块空白场地，原地图保持不动。

    传了 ``map_id`` 就把那张已有地图的几何读进草稿，在它基础上改。
    """
    if _editor_busy(httpd.manager):
        raise ValueError("任务进行中不能编辑地图，请先停止任务")
    draft = map_editor.Draft(name or "自定义地图")
    start = None
    if map_id is not None:
        source = world.MAPS.get(int(map_id))
        if source is None:
            raise ValueError("没有找到编号为 {} 的地图".format(map_id))
        try:
            draft.load_from_definition(
                source,
                name=name,
                scale=1.0 if source.get("native") else world.MAP_SCALE,
            )
        except ValueError as error:
            raise ValueError("这张地图读不进编辑器：{}".format(error))
        bounds = draft.content_bounds()
        start = ((bounds[0] + bounds[2]) * 0.5, (bounds[1] + bounds[3]) * 0.5)
    definition = map_editor.blank_field_definition(
        draft.width, draft.height, name=draft.name, start=start
    )
    with httpd.simulator.lock:
        map_id = world.register_map(definition)
        world.select_map(map_id)
        robot = RobotCar()
        grid = GridMap()
        manager = build_manager(robot, grid)
        httpd.robot = robot
        httpd.grid = grid
        httpd.manager = manager
        httpd.simulator.manager = manager
    draft.map_id = map_id
    httpd.editor = draft
    return draft


def close_editor(httpd, fallback_map=1):
    """退出编辑模式：删掉临时空白场地，切回一张正常地图。"""
    draft = getattr(httpd, "editor", None)
    httpd.editor = None
    if draft is not None and draft.map_id is not None:
        try:
            world.delete_map(draft.map_id)
        except ValueError:
            pass
    if world.MAP_ID not in world.MAPS or world.MAPS[int(world.MAP_ID)].get(
        "editor_draft"
    ):
        target = fallback_map
        if target not in world.MAPS:
            target = min(world.MAPS.keys())
        with httpd.simulator.lock:
            world.select_map(target)
            robot = RobotCar()
            grid = GridMap()
            manager = build_manager(robot, grid)
            httpd.robot = robot
            httpd.grid = grid
            httpd.manager = manager
            httpd.simulator.manager = manager
    return _editor_snapshot(httpd)


def save_editor(httpd, name=None):
    draft = _editor_require(httpd)
    definition = draft.to_definition(name)
    with httpd.simulator.lock:
        map_id = world.register_map(definition)
        draft_map_id = draft.map_id
        draft.map_id = None
        world.select_map(map_id)
        robot = RobotCar()
        grid = GridMap()
        manager = build_manager(robot, grid)
        httpd.robot = robot
        httpd.grid = grid
        httpd.manager = manager
        httpd.simulator.manager = manager
    httpd.editor = None
    try:
        persist_editor_map(definition)
    except OSError:  # pragma: no cover - 落盘失败不影响本次保存
        traceback.print_exc()
    if draft_map_id is not None:
        try:
            world.delete_map(draft_map_id)
        except ValueError:
            pass
    return {
        "map_id": map_id,
        "name": definition["name"],
        "state": httpd.manager.get_state(include_map=True),
    }


def rebuild_system(httpd, definition=None):
    """按给定地图定义（或当前地图）重建机器人 / 栅格 / 任务 / 节点。"""
    if definition is not None:
        map_id = world.register_map(definition)
        world.select_map(map_id)
    robot = RobotCar()
    grid = GridMap()
    manager = build_manager(robot, grid)
    with httpd.simulator.lock:
        httpd.robot = robot
        httpd.grid = grid
        httpd.manager = manager
        httpd.simulator.replace_manager(manager)
    return manager


def create_server(host="127.0.0.1", port=8000):
    load_persisted_editor_maps()
    world.select_map(1)
    robot = RobotCar()
    grid = GridMap()
    manager = build_manager(robot, grid)
    httpd = RobotHTTPServer((host, port), RobotRequestHandler)
    httpd.robot = robot
    httpd.grid = grid
    httpd.manager = manager
    httpd.bag_roots = [BAG_DIR]
    httpd.bag_queue = None
    httpd.bag_cloud_history = []
    httpd.editor = None
    httpd.simulator = Simulator(manager)
    httpd.simulator.on_cycle = lambda: poll_bag_queue(httpd)
    httpd.simulator.start()
    return httpd


def main(argv=None):
    parser = argparse.ArgumentParser(description="机器人小车控制台 Web 服务")
    parser.add_argument("--host", default="127.0.0.1", help="监听地址，默认 127.0.0.1")
    parser.add_argument("--port", type=int, default=8000, help="监听端口，默认 8000")
    args = parser.parse_args(argv)

    httpd = create_server(args.host, args.port)
    url = "http://{}:{}/".format(args.host, args.port)
    print("机器人小车控制台已启动: {}".format(url))
    print("按 Ctrl+C 停止。")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n正在关闭服务…")
    finally:
        httpd.simulator.stop()
        httpd.server_close()


if __name__ == "__main__":
    main()
