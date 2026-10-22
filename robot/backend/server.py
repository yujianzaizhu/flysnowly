"""机器人小车 Web 服务：静态界面 + 任务/遥控 JSON 接口。"""

import argparse
import json
import os
import sys
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlsplit

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config  # noqa: E402
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
SIM_DT = 0.02


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

    def run(self):
        last = time.monotonic()
        accumulator = 0.0
        while not self._stop_event.is_set():
            now = time.monotonic()
            elapsed = min(0.2, max(0.0, now - last))
            last = now
            accumulator += elapsed * self.speed
            steps = 0
            try:
                with self.lock:
                    manager = self.manager
                    while accumulator >= self.tick_size and steps < 40:
                        manager.tick(self.tick_size)
                        accumulator -= self.tick_size
                        steps += 1
            except Exception:
                traceback.print_exc()
            self._stop_event.wait(0.005)

    def stop(self):
        self._stop_event.set()


class RobotRequestHandler(BaseHTTPRequestHandler):
    server_version = "RobotCar/2.0"
    protocol_version = "HTTP/1.1"

    # --------------------------------------------------------------- 路由
    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/api/state":
            self._send_json(self._state())
        elif path == "/api/world":
            self._send_json(world.world_dict())
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
            payload = self._read_json()
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
                    manager = TaskManager(robot, grid)
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
            elif path == "/api/speed":
                speed = int(payload.get("speed", 1))
                if speed not in (1, 2, 4, 8):
                    raise ValueError("仿真速度仅支持 1/2/4/8")
                self.server.simulator.speed = speed
            else:
                self._send_json({"error": "接口不存在"}, status=404)
                return
        except ValueError as exc:
            self._send_json({"error": str(exc)}, status=400)
            return

        self._send_json(self._state())

    # --------------------------------------------------------------- 工具
    def _state(self):
        state = self.server.manager.get_state()
        state["speed"] = self.server.simulator.speed
        state["map_id"] = world.MAP_ID
        state["map_name"] = world.MAP_NAME
        state["config"] = dict(config.settings.__dict__)
        return state

    def _read_json(self):
        try:
            length = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            raise ValueError("Content-Length 无效")
        if length <= 0:
            return {}
        if length > MAX_BODY_BYTES:
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


def create_server(host="127.0.0.1", port=8000):
    world.select_map(1)
    robot = RobotCar()
    grid = GridMap()
    manager = TaskManager(robot, grid)
    httpd = RobotHTTPServer((host, port), RobotRequestHandler)
    httpd.robot = robot
    httpd.grid = grid
    httpd.manager = manager
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
