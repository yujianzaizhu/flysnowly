"""机器人小车 Web 服务：静态界面 + 任务/遥控 JSON 接口。"""

import argparse
import base64
import hashlib
import json
import os
import sys
import tempfile
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlsplit

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config  # noqa: E402
import map_import  # noqa: E402
import nav_bus  # noqa: E402
import nav_bridge  # noqa: E402
import world  # noqa: E402
from grid_map import GridMap  # noqa: E402
from robot_car import RobotCar  # noqa: E402
from tasks import TaskManager  # noqa: E402

FRONTEND_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, "frontend")
)

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


class Simulator(threading.Thread):
    """固定步长推进仿真；speed 仅改变每帧执行的步数，保证控制稳定。"""

    def __init__(self, manager, tick=SIM_DT):
        super().__init__(name="robot-simulator", daemon=True)
        self.manager = manager
        self.tick_size = tick
        self.speed = 1
        self.lock = threading.RLock()
        self._stop_event = threading.Event()

    def replace_manager(self, manager):
        with self.lock:
            self.manager = manager

    def step_size(self):
        """按倍速自适应积分步长：高倍速时用更大步长，避免每个 20 ms 都跑一遍全量计算。"""
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
            # 1 ms 节拍：保证高倍速时每轮累积的仿真时间足以跨过一步，不会因节拍太粗掉速
            self._stop_event.wait(0.001)

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
            self._send_json(self._state(include_map=with_map))
        elif path == "/api/world":
            self._send_json(world.world_dict())
        elif path == "/api/nav/config":
            nav = self.server.manager.nav
            self._send_json(nav.static_summary() if nav is not None else {"loaded": False})
        elif path == "/api/nav":
            nav = self.server.manager.nav
            self._send_json(nav.snapshot() if nav is not None else {"error": "导航节点未启用"})
        elif path == "/api/config":
            self._send_json(self._state())
        elif path == "/api/health":
            self._send_json(
                {"status": "ok", "time": round(time.time(), 3), "speed": self.server.simulator.speed}
            )
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
                self.server.robot.handle_command(payload.get("command"))
            elif path == "/api/task":
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
                self.server.robot.reset()
                self.server.grid.reset_map()
                manager.reset()
            elif path == "/api/map":
                map_id = int(payload.get("map", 1))
                if map_id not in world.MAPS:
                    raise ValueError("未知地图: {}".format(map_id))
                with self.server.simulator.lock:
                    world.select_map(map_id)
                    robot = RobotCar()
                    grid = GridMap()
                    manager, nav = build_manager(robot, grid)
                    nav.set_enabled(getattr(self.server.nav, "enabled", True))
                    self.server.robot = robot
                    self.server.grid = grid
                    self.server.manager = manager
                    self.server.nav = nav
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
                        manager, nav = build_manager(robot, grid)
                        nav.set_enabled(getattr(self.server.nav, "enabled", True))
                        self.server.robot = robot
                        self.server.grid = grid
                        self.server.manager = manager
                        self.server.nav = nav
                        self.server.simulator.manager = manager
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
                    manager, nav = build_manager(robot, grid)
                    nav.set_enabled(getattr(self.server.nav, "enabled", True))
                    self.server.robot = robot
                    self.server.grid = grid
                    self.server.manager = manager
                    self.server.nav = nav
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
                    manager, nav = build_manager(robot, grid)
                    nav.set_enabled(getattr(self.server.nav, "enabled", True))
                    self.server.robot = robot
                    self.server.grid = grid
                    self.server.manager = manager
                    self.server.nav = nav
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
            elif path == "/api/nav/config":
                nav = manager.nav
                if nav is None:
                    raise ValueError("导航节点未启用")
                summary = nav.apply_static_config(payload.get("path") or None)
                self._send_json(summary)
                return
            elif path == "/api/nav/mode":
                nav = manager.nav
                if nav is None:
                    raise ValueError("导航节点未启用")
                enabled = payload.get("enabled")
                if enabled is None:
                    raise ValueError("缺少 enabled 字段")
                nav.set_enabled(bool(enabled))
            elif path == "/api/nav/action":
                nav = manager.nav
                if nav is None:
                    raise ValueError("导航节点未启用")
                action = str(payload.get("action") or "nav_task_action")
                goal = payload.get("goal") or {}
                request = {"cmd": int(payload.get("cmd", nav_bus.ACTION_START)), "goal": goal}
                response = nav.call_action(str(payload.get("node") or "planner"), action, request)
                if response.get("error"):
                    raise ValueError(response["error"])
                self._apply_nav_side_effects(manager, action, request["cmd"], goal, response)
            elif path == "/api/speed":
                speed = int(payload.get("speed", 1))
                if speed not in (1, 2, 4, 8, 16, 32):
                    raise ValueError("仿真速度仅支持 1/2/4/8/16/32")
                self.server.simulator.speed = speed
            else:
                self._send_json({"error": "接口不存在"}, status=404)
                return
        except ValueError as exc:
            self._send_json({"error": str(exc)}, status=400)
            return

        self._send_json(self._state())

    # --------------------------------------------------------------- 工具
    def _apply_nav_side_effects(self, manager, action, cmd, goal, response):
        """把节点动作命令翻译成仿真任务控制（节点 → 仿真的下行链路）。"""
        if response.get("status") != nav_bus.RESULT_SUCCEED:
            return
        if action == "autodock_action" and cmd == nav_bus.ACTION_START:
            manager.request_charge()
        elif action == "leave_dock_action" and cmd == nav_bus.ACTION_START:
            manager.request_resume()
        elif action == "nav_task_action":
            mode = int(goal.get("mode", 0) or 0)
            if cmd == nav_bus.ACTION_START and mode == nav_bus.MODE_COVERAGE:
                zones = goal.get("path_name") or goal.get("zones")
                manager.start_mowing(zones, goal.get("mow_mode"))
            elif cmd == nav_bus.ACTION_PAUSE:
                manager.stop_task("planner 节点请求暂停")
            elif cmd == nav_bus.ACTION_CONTINUE:
                manager.request_resume()
            elif cmd in (nav_bus.ACTION_CANCEL, nav_bus.ACTION_FINISHED):
                manager.stop_task("planner 节点请求结束任务", resumable=False)

    def _state(self, include_map=True):
        state = self.server.manager.get_state(include_map=include_map)
        state["speed"] = self.server.simulator.speed
        state["map_id"] = world.MAP_ID
        state["map_name"] = world.MAP_NAME
        state["config"] = dict(config.settings.__dict__)
        nav = self.server.manager.nav if self.server.manager else None
        state["nav"] = nav.snapshot() if nav is not None else None
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
        self.send_response(status)
        self.send_header("Content-Type", CONTENT_TYPES[".json"])
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
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


def build_manager(robot, grid):
    """创建任务管理器，并接入 planner / dock / region_manager / trajectory_map 四个节点。"""
    nav = nav_bridge.NavBridge(robot, grid)
    try:
        nav.apply_static_config()
    except Exception as exc:  # static_config 缺失时保持默认参数
        print("static_config 未导入：{}".format(exc))
    return TaskManager(robot, grid, nav=nav), nav


def create_server(host="127.0.0.1", port=8000):
    world.select_map(1)
    robot = RobotCar()
    grid = GridMap()
    manager, nav = build_manager(robot, grid)
    httpd = RobotHTTPServer((host, port), RobotRequestHandler)
    httpd.robot = robot
    httpd.grid = grid
    httpd.manager = manager
    httpd.nav = nav
    httpd.simulator = Simulator(manager)
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
