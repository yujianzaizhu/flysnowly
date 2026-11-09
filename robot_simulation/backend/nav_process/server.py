"""接入导航进程仿真的独立 HTTP 应用。

不导入内部仿真的 server/world/tasks/bag/config；页面、状态和 API 均独立。
"""

import argparse
import gzip
import json
import os
import sys
import threading
import time
import traceback
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

if __package__:
    from . import config, tea_bridge
    from .robot import NavRobot
else:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from nav_process import config, tea_bridge
    from nav_process.robot import NavRobot

import mode_control

FRONTEND_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir, "frontend", "nav_process")
)
SHARED_DIR = os.path.normpath(os.path.join(FRONTEND_DIR, os.pardir, "shared"))
CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".svg": "image/svg+xml",
}
MAX_BODY_BYTES = 1024 * 1024


class ConflictError(ValueError):
    http_status = 409


class NavSimulator(threading.Thread):
    def __init__(self, robot, tea, tick=0.02):
        super().__init__(name="nav-process-simulator", daemon=True)
        self.robot = robot
        self.tea = tea
        self.tick_size = tick
        self.lock = threading.RLock()
        self._stop_event = threading.Event()

    def run(self):
        last = time.monotonic()
        accumulator = 0.0
        while not self._stop_event.is_set():
            now = time.monotonic()
            accumulator += min(0.2, max(0.0, now - last))
            last = now
            try:
                with self.lock:
                    while accumulator >= self.tick_size:
                        self.robot.step(self.tick_size)
                        accumulator -= self.tick_size
                    self.tea.tick(self.robot)
            except Exception:
                traceback.print_exc()
            self._stop_event.wait(0.005)

    def stop(self):
        self._stop_event.set()


class NavRequestHandler(BaseHTTPRequestHandler):
    server_version = "NavProcess/1.0"
    protocol_version = "HTTP/1.1"

    def do_GET(self):
        if mode_control.handle(self, "GET"):
            return
        path = urllib.parse.urlsplit(self.path).path
        if path == "/api/nav/state":
            self._send_json(self._state())
        elif path == "/api/nav/workarea/state":
            self._send_json(self.server.tea.summary())
        elif path == "/api/nav/workarea/map":
            query = urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query)
            try:
                version = int((query.get("v") or ["-1"])[0])
            except ValueError:
                version = -1
            self._send_json(self.server.tea.map_snapshot(None if version < 0 else version))
        elif path.startswith("/api/"):
            self._send_json({"error": "接口不存在"}, status=404)
        else:
            self._serve_static(path)

    def do_POST(self):
        if mode_control.handle(self, "POST"):
            return
        path = urllib.parse.urlsplit(self.path).path
        try:
            payload = self._read_json()
            if path == "/api/nav/drive":
                self.server.robot.drive(payload.get("v", 0.0), payload.get("w", 0.0))
            elif path == "/api/nav/stop":
                self.server.robot.stop()
            elif path == "/api/nav/activity":
                self._set_activity(str(payload.get("activity") or ""))
            elif path == "/api/nav/config":
                if payload.get("action") == "reset":
                    config.reset()
                else:
                    values = payload.get("values") if isinstance(payload.get("values"), dict) else payload
                    config.configure(**{key: value for key, value in values.items() if key != "action"})
            elif path.startswith("/api/nav/workarea/"):
                result = self._handle_workarea(path[len("/api/nav/workarea/"):], payload)
                state = self._state()
                state["workarea_result"] = result
                self._send_json(state)
                return
            elif path == "/api/nav/shutdown":
                self._send_json({"ok": True})
                self.server.stop_service()
                return
            else:
                self._send_json({"error": "接口不存在"}, status=404)
                return
        except ConflictError as exc:
            self._send_json({"error": str(exc)}, status=409)
            return
        except (TypeError, ValueError) as exc:
            self._send_json({"error": str(exc)}, status=400)
            return
        self._send_json(self._state())

    def _set_activity(self, activity):
        if activity not in ("work", "workarea"):
            raise ValueError("活动状态仅支持 work/workarea")
        if self.server.tea.is_building():
            raise ConflictError("工作区构建中，请先保存或取消")
        if activity == "workarea" and not self.server.tea.summary().get("enabled"):
            raise ValueError("导航进程桥不可用，无法进入工作区构建")
        self.server.robot.stop()
        self.server.activity = activity

    def _handle_workarea(self, action, payload):
        tea = self.server.tea
        if action == "connect":
            return self._workarea_connect(payload)
        if not tea.summary().get("enabled"):
            raise ValueError("导航进程桥未启用: {}".format(tea.summary().get("reason", "")))

        if action == "start":
            type_ = int(payload.get("type", 0) or 0)
            if type_ not in tea_bridge.ELEMENT_NAMES:
                raise ValueError("元素类型必须是 1(区域)/2(通道)/3(禁区)")
            if payload.get("check"):
                resp = tea.record_path(tea_bridge.CMD_ALLOW_START, type_=type_)
                self._log_record(tea_bridge.CMD_ALLOW_START, "检查起点", resp)
                if not resp.get("ok"):
                    return resp
            self.server.robot.stop()
            resp = tea.record_path(tea_bridge.CMD_START, type_=type_)
            self._log_record(tea_bridge.CMD_START, "type={}".format(tea_bridge.ELEMENT_NAMES[type_]), resp)
            if resp.get("ok"):
                tea.begin_session(type_)
                self.server.activity = "workarea"
            resp["session"] = tea.summary()["session"]
            return resp

        if action == "save":
            if payload.get("check"):
                resp = tea.record_path(tea_bridge.CMD_ALLOW_SAVE)
                self._log_record(tea_bridge.CMD_ALLOW_SAVE, "校验可保存", resp)
                if not resp.get("ok"):
                    return resp
            resp = tea.record_path(tea_bridge.CMD_SAVE)
            self._log_record(tea_bridge.CMD_SAVE, "保存轨迹", resp)
            if resp.get("ok"):
                tea.end_session()
                refresh = self._refresh_map("保存")
                resp["map_version"] = refresh.get("map_version")
                resp["map_error"] = refresh.get("map_error")
            resp["session"] = tea.summary()["session"]
            return resp

        if action == "cancel":
            resp = tea.record_path(tea_bridge.CMD_CANCEL)
            self._log_record(tea_bridge.CMD_CANCEL, "取消本次构建", resp)
            if resp.get("ok"):
                tea.end_session()
                refresh = self._refresh_map("取消")
                resp["map_version"] = refresh.get("map_version")
                resp["map_error"] = refresh.get("map_error")
            resp["session"] = tea.summary()["session"]
            return resp

        if action == "clear":
            ids = [int(item) for item in (payload.get("ids") or [])]
            resp = tea.record_path(tea_bridge.CMD_CLEAR, deleted_ids=ids or None)
            self._log_record(tea_bridge.CMD_CLEAR, "删除地图元素", resp)
            if resp.get("ok"):
                refresh = self._refresh_map("删除")
                resp["map_version"] = refresh.get("map_version")
            return resp

        if action == "load":
            resp = tea.record_path(tea_bridge.CMD_LOAD)
            self._log_record(tea_bridge.CMD_LOAD, "从磁盘重载地图", resp)
            if resp.get("ok"):
                refresh = self._refresh_map("加载")
                resp["map_version"] = refresh.get("map_version")
            return resp

        if action == "probe":
            resp = tea.probe()
            self.server.robot.log(
                "event" if resp.get("connected") else "warn",
                "[工作区] 导航进程{}".format("在线" if resp.get("connected") else "离线"),
            )
            return resp

        if action == "raw":
            service = str(payload.get("service") or "record_path")
            if service == "record_path":
                cmd = int(payload.get("cmd", 0) or 0)
                resp = tea.record_path(
                    cmd,
                    type_=payload.get("type"),
                    id_=payload.get("id"),
                    deleted_ids=payload.get("deleted_ids"),
                    path_name=payload.get("path_name"),
                )
                self._log_record(cmd, "raw", resp)
                return resp
            if service == "get_map_info":
                return tea.get_map_info(cmd=int(payload.get("map_cmd", 1) or 1))
            if service == "nav_action":
                return tea.nav_action(int(payload.get("cmd", 0) or 0), payload.get("goal"))
            raise ValueError("未知服务: {}".format(service))

        raise ValueError("工作区接口不存在")

    def _workarea_connect(self, payload):
        current = self.server.tea
        opts = dict(getattr(current, "opts", None) or dict(tea_bridge.DEFAULT_OPTS))
        for key in ("host", "proxy_pub_port", "proxy_sub_port", "record_port", "map_port", "nav_port"):
            if payload.get(key) is not None:
                opts[key] = payload[key]
        if payload.get("run_proxy") is not None:
            opts["run_proxy"] = bool(payload["run_proxy"])
        opts["enabled"] = bool(payload.get("enabled", True))
        current.stop()
        tea = tea_bridge.create(
            opts,
            log_fn=lambda level, message: self.server.robot.log(level, message),
        )
        if opts["enabled"]:
            tea.start()
        self.server.tea = tea
        self.server.simulator.tea = tea
        self.server.robot.stop()
        return tea.summary()

    def _log_record(self, cmd, desc, resp):
        name = tea_bridge.CMD_NAMES.get(cmd, str(cmd))
        if resp.get("ok"):
            message = "[工作区] {}({}) 成功".format(name, desc)
            if resp.get("info"):
                message += " info={}".format(resp["info"])
            self.server.robot.log("event", message)
        else:
            self.server.robot.log(
                "warn",
                "[工作区] {}({}) 失败: {} {}".format(
                    name, desc, tea_bridge.error_text(resp.get("result")), resp.get("info") or ""
                ),
            )

    def _refresh_map(self, context):
        resp = self.server.tea.get_map_info()
        if not resp.get("ok"):
            self.server.robot.log("warn", "[工作区] {}后刷新地图失败".format(context))
        elif resp.get("map_error"):
            self.server.robot.log("warn", "[工作区] {}后地图数据未更新".format(context))
        else:
            self.server.robot.log("event", "[工作区] {}后地图已刷新".format(context))
        return resp

    def _state(self):
        state = self.server.robot.get_state()
        state.update(
            {
                "sim_mode": "nav_process",
                "activity": self.server.activity,
                "config": dict(config.settings.__dict__),
                "workarea": self.server.tea.summary(),
            }
        )
        return state

    def _read_json(self):
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            raise ValueError("Content-Length 无效")
        if length <= 0 or length > MAX_BODY_BYTES:
            raise ValueError("请求体为空或过大")
        try:
            data = json.loads(self.rfile.read(length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            raise ValueError("请求体不是合法 JSON")
        if not isinstance(data, dict):
            raise ValueError("请求体必须是 JSON 对象")
        return data

    def _send_json(self, payload, status=200):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        encoding = None
        if len(body) > 2048 and "gzip" in (self.headers.get("Accept-Encoding") or ""):
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
        relative = urllib.parse.unquote(path).lstrip("/") or "index.html"
        if relative == "index.html" or relative in ("app.js", "styles.css"):
            root = FRONTEND_DIR
        elif relative.startswith("shared/"):
            root = os.path.dirname(SHARED_DIR)
        else:
            self._send_text("页面不存在", status=404)
            return
        if ".." in relative.split("/"):
            self._send_text("禁止访问", status=403)
            return
        full_path = os.path.normpath(os.path.join(root, relative))
        if not full_path.startswith(root + os.sep) or not os.path.isfile(full_path):
            self._send_text("页面不存在", status=404)
            return
        with open(full_path, "rb") as handle:
            body = handle.read()
        self.send_response(200)
        self.send_header("Content-Type", CONTENT_TYPES.get(os.path.splitext(full_path)[1], "application/octet-stream"))
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


class NavHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def stop_application(self):
        self.robot.stop()
        self.simulator.stop()
        self.tea.stop()

    def stop_service(self):
        def _shutdown():
            time.sleep(0.15)
            self.stop_application()
            self.shutdown()

        threading.Thread(target=_shutdown, name="nav-shutdown", daemon=True).start()


def create_server(host="127.0.0.1", port=8000, tea_opts=None):
    robot = NavRobot()
    opts = dict(tea_opts) if tea_opts is not None else {"enabled": True}
    tea = tea_bridge.create(opts, log_fn=lambda level, message: robot.log(level, message))
    httpd = NavHTTPServer((host, port), NavRequestHandler)
    httpd.robot = robot
    httpd.tea = tea
    httpd.activity = "work"
    httpd.simulator = NavSimulator(robot, tea)
    httpd.simulator.start()
    if tea.summary().get("enabled"):
        tea.start()
    return httpd


def tea_opts_from_env(argv=None):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--tea-bridge", action="store_true", default=None)
    parser.add_argument("--tea-host", default=None)
    parser.add_argument("--tea-proxy-pub", type=int, default=None)
    parser.add_argument("--tea-proxy-sub", type=int, default=None)
    parser.add_argument("--tea-record-port", type=int, default=None)
    parser.add_argument("--tea-map-port", type=int, default=None)
    parser.add_argument("--tea-no-proxy", action="store_true", default=None)
    known, _ = parser.parse_known_args(argv)
    opts = {}
    if known.tea_bridge is not None:
        opts["enabled"] = known.tea_bridge
    elif os.environ.get("ROBOT_TEA_BRIDGE") in ("1", "true", "True"):
        opts["enabled"] = True
    if known.tea_host is not None:
        opts["host"] = known.tea_host
    elif os.environ.get("ROBOT_TEA_HOST"):
        opts["host"] = os.environ["ROBOT_TEA_HOST"]
    if known.tea_no_proxy:
        opts["run_proxy"] = False
    for arg_key, env_key, opt_key in (
        ("tea_proxy_pub", "ROBOT_TEA_PROXY_PUB", "proxy_pub_port"),
        ("tea_proxy_sub", "ROBOT_TEA_PROXY_SUB", "proxy_sub_port"),
        ("tea_record_port", "ROBOT_TEA_RECORD_PORT", "record_port"),
        ("tea_map_port", "ROBOT_TEA_MAP_PORT", "map_port"),
    ):
        value = getattr(known, arg_key) or os.environ.get(env_key)
        if value is not None:
            opts[opt_key] = int(value)
    return opts


def main(argv=None):
    parser = argparse.ArgumentParser(description="独立导航进程仿真")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    args, _ = parser.parse_known_args(argv)
    opts = tea_opts_from_env(argv)
    opts.setdefault("enabled", True)
    httpd = create_server(args.host, args.port, opts)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.stop_application()
        httpd.server_close()


if __name__ == "__main__":
    main()
