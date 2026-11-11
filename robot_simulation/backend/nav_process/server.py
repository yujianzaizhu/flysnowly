"""接入导航进程仿真的独立 HTTP 应用。

不导入内部仿真的 server/world/tasks/bag/config；页面、状态和 API 均独立。
"""

import argparse
import gzip
import json
import math
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

# 接入后自动拉图: get_map_info 触发 tea 发布 map_info 回填桥缓存; 失败重试几次
# (tea 可能比仿真后端晚启动), 仍失败则记日志放弃, 可由页面"刷新工作区地图"补拉
AUTO_MAP_FETCH_RETRIES = 3
AUTO_MAP_FETCH_INTERVAL = 2.0  # 重试间隔(s)


def _fetch_map(tea, log, context, retries=0, retry_interval=0.0):
    """统一 get_map_info 调用与结果判定: ok 且无 map_error 才算成功。

    成功返回 resp(调用方追加各自的"已刷新/已获取"日志);
    失败与"读成功但地图数据未更新"(map_error, 桥缓存可能是旧数据)统一在这里记 warn,
    保证 _refresh_map 与接入自动拉图两条路径行为一致。
    """

    resp = {}
    for attempt in range(1, max(retries, 1) + 1):
        resp = tea.get_map_info()
        if resp.get("ok") and not resp.get("map_error"):
            break
        if attempt < max(retries, 1):
            time.sleep(retry_interval)
    if not resp.get("ok"):
        log("warn", "[工作区] {}失败: {}".format(context, resp.get("info") or "无响应"))
    elif resp.get("map_error"):
        log("warn", "[工作区] {}: 地图数据未更新({})".format(context, resp.get("info") or "map_error"))
    return resp


def _fetch_initial_map(tea, log):
    """同步触发一次 get_map_info 并按结果写日志; 供接入流程与测试直接调用。

    map_error(读成功但数据未更新)视为未获取, 不从旧快照得出"已获取"的误导日志。
    """

    resp = _fetch_map(
        tea, log, "接入后自动获取工作区地图",
        retries=AUTO_MAP_FETCH_RETRIES, retry_interval=AUTO_MAP_FETCH_INTERVAL,
    )
    if not (resp.get("ok") and not resp.get("map_error")):
        return resp
    snapshot = tea.map_snapshot()
    data = snapshot.get("map") if isinstance(snapshot, dict) else None
    subs = data.get("sub_maps") if isinstance(data, dict) else None
    if subs:
        log(
            "event",
            "[工作区] 接入后已获取工作区地图: {} 个子图(版本 {})".format(len(subs), resp.get("map_version")),
        )
    else:
        # tea 启动时会自动建空图, 这里即"地图为空"而非"还没拉到"
        log("event", "[工作区] 接入后工作区地图为空(暂无区域/通道/禁区)")
    return resp


def _auto_fetch_map(tea, robot):
    """接入(tea 桥启动)后后台拉一次当前工作区地图, 不阻塞启动/连接响应; 返回线程供测试等待。"""

    thread = threading.Thread(
        target=lambda: _fetch_initial_map(tea, robot.log), name="tea-map-warmup", daemon=True
    )
    thread.start()
    return thread


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
        # 点到点任务状态(dict, 由 server 持有并注入); 线程内只读改 active/paused
        self.nav = None

    def run(self):
        last = time.monotonic()
        accumulator = 0.0
        while not self._stop_event.is_set():
            now = time.monotonic()
            accumulator += min(0.2, max(0.0, now - last))
            last = now
            try:
                with self.lock:
                    self._apply_nav_cmd()
                    while accumulator >= self.tick_size:
                        self.robot.step(self.tick_size)
                        accumulator -= self.tick_size
                    self.tea.tick(self.robot)
            except Exception:
                traceback.print_exc()
            self._stop_event.wait(0.005)

    def _apply_nav_cmd(self):
        """点到点速度闭环: tea cmd_vel -> 机器人导航速度源; 超时清零, 终态自动退出。"""

        nav = self.nav
        if not nav or not nav.get("active"):
            return
        cmd = self.tea.latest_cmd_vel()
        if cmd and cmd["age"] <= tea_bridge.CMD_VEL_TIMEOUT:
            self.robot.drive_nav(cmd["linear"], cmd["angular"])
        else:
            # cmd_vel 超过超时未更新(暂停/断链)即清零, 对齐真机底盘保护
            self.robot.drive_nav(0.0, 0.0)
        result = self.tea.latest_nav_result()
        if result and result.get("at", 0.0) > nav.get("since", 0.0):
            outcome = result.get("outcome", 0)
            nav["active"] = False
            nav["paused"] = False
            self.robot.set_nav_mode(False)
            self.robot.log(
                "event" if outcome == 1 else "warn",
                "[导航] 点到点任务结束: {}({})".format(
                    tea_bridge.NAV_OUTCOME_NAMES.get(outcome, outcome),
                    result.get("text") or "-",
                ),
            )

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
                if self.server.nav.get("active"):
                    raise ConflictError("点到点导航中，手动遥控已锁定")
                self.server.robot.drive(payload.get("v", 0.0), payload.get("w", 0.0))
            elif path == "/api/nav/stop":
                if self.server.nav.get("active"):
                    raise ConflictError("点到点导航中，请先取消导航任务")
                self.server.robot.stop()
            elif path == "/api/nav/goal":
                result = self._nav_goal(payload)
                state = self._state()
                state["nav_action_result"] = result
                self._send_json(state)
                return
            elif path in ("/api/nav/goal/cancel", "/api/nav/goal/pause", "/api/nav/goal/continue"):
                result = self._nav_goal_cmd(path.rsplit("/", 1)[-1])
                state = self._state()
                state["nav_action_result"] = result
                self._send_json(state)
                return
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
        if activity == "workarea" and self.server.nav.get("active"):
            # tea 在 MapEdit 状态会拒绝导航任务, 这里提前拦截保持一致
            raise ConflictError("点到点导航中，请先取消导航任务再进入工作区")
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
            # 老协议约定: 不带 ids = 整图清空(成功码 result=1)
            self._log_record(tea_bridge.CMD_CLEAR, "整图清空" if not ids else "删除地图元素 {}".format(ids), resp)
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

        if action == "refresh_map":
            # 构建中也可调: get_map_info 是纯读命令, 触发 tea 发布当前地图并回填桥缓存
            return self._refresh_map("手动刷新")

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
        for key in (
            "host",
            "proxy_pub_port",
            "proxy_sub_port",
            "record_port",
            "map_port",
            "nav_port",
            "cmd_vel_topic",
            "nav_result_topic",
        ):
            if payload.get(key) is not None:
                opts[key] = payload[key]
        if payload.get("run_proxy") is not None:
            opts["run_proxy"] = bool(payload["run_proxy"])
        opts["enabled"] = bool(payload.get("enabled", True))
        # 幂等连接: 目标参数与当前桥一致且已启用时不重建——重建会拆掉已绑定
        # 10001/10002 的内嵌 proxy, 紧随其后的二次探测会撞上拆解窗口而永久降级
        # (run.sh 开机 connect 曾因此让话题层全断, 构建轨迹采不到位姿)
        if opts == getattr(current, "opts", None) and current.summary().get("enabled"):
            return current.summary()
        current.stop()
        tea = tea_bridge.create(
            opts,
            log_fn=lambda level, message: self.server.robot.log(level, message),
        )
        if opts["enabled"]:
            tea.start()
            _auto_fetch_map(tea, self.server.robot)
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
        resp = _fetch_map(self.server.tea, self.server.robot.log, "{}后刷新地图".format(context))
        if resp.get("ok") and not resp.get("map_error"):
            self.server.robot.log("event", "[工作区] {}后地图已刷新".format(context))
        return resp

    # ------------------------------------------------------------ 点到点导航

    def _nav_goal(self, payload):
        """下发点到点目标: START + NAVIGATION + target_pose。tea 拒绝/离线时返回 ok=False。"""

        tea = self.server.tea
        if not tea.summary().get("enabled"):
            raise ValueError("导航进程桥未启用: {}".format(tea.summary().get("reason", "")))
        if tea.is_building() or self.server.activity == "workarea":
            raise ConflictError("工作区构建/编辑中，tea 会拒绝导航任务，请先退出工作区")
        nav = self.server.nav
        if nav.get("active"):
            raise ConflictError("已有进行中的点到点任务，请先取消")
        try:
            x = float(payload["x"])
            y = float(payload["y"])
        except (KeyError, TypeError, ValueError):
            raise ValueError("目标点坐标缺失或非法")
        yaw = payload.get("yaw")
        if yaw is None:
            # 默认朝向: 当前位姿指向目标点, 与 tea goal 话题入口的默认行为一致
            pose = self.server.robot.pose
            yaw = math.atan2(y - pose[1], x - pose[0])
        yaw = float(yaw)
        resp = tea.nav_action(
            tea_bridge.ACTION_START,
            {"mode": tea_bridge.TASK_MODE_NAVIGATION, "target_pose": {"x": x, "y": y, "yaw": yaw}},
        )
        if not resp.get("ok"):
            self.server.robot.log("warn", "[导航] 目标点下发失败: {}".format(resp.get("info") or resp.get("error")))
            return {"ok": False, "info": resp.get("info") or resp.get("error") or "桥调用失败"}
        if resp.get("status") != tea_bridge.NAV_RESULT_SUCCEED:
            self.server.robot.log("warn", "[导航] 目标点被 tea 拒绝: {}".format(resp.get("msgs") or resp.get("status")))
            return {"ok": False, "status": resp.get("status"), "info": resp.get("msgs") or "tea 拒绝目标点"}
        nav.update(
            {
                "active": True,
                "paused": False,
                "goal": {"x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 3)},
                "since": time.monotonic(),
            }
        )
        self.server.robot.set_nav_mode(True)
        self.server.robot.log("event", "[导航] 点到点任务已下发: ({:.2f}, {:.2f})".format(x, y))
        return {"ok": True, "goal": nav["goal"]}

    def _nav_goal_cmd(self, action):
        """取消/暂停/继续进行中的点到点任务。"""

        tea = self.server.tea
        nav = self.server.nav
        if not nav.get("active"):
            raise ConflictError("当前没有进行中的点到点任务")
        cmd = {
            "cancel": tea_bridge.ACTION_CANCEL,
            "pause": tea_bridge.ACTION_PAUSE,
            "continue": tea_bridge.ACTION_CONTINUE,
        }[action]
        resp = tea.nav_action(cmd)
        if not resp.get("ok"):
            self.server.robot.log("warn", "[导航] {} 失败: {}".format(action, resp.get("info") or resp.get("error")))
            return {"ok": False, "info": resp.get("info") or resp.get("error") or "桥调用失败"}
        if resp.get("status") != tea_bridge.NAV_RESULT_SUCCEED:
            self.server.robot.log("warn", "[导航] {} 被 tea 拒绝: {}".format(action, resp.get("msgs") or resp.get("status")))
            return {"ok": False, "status": resp.get("status"), "info": resp.get("msgs") or "tea 拒绝"}
        if action == "cancel":
            nav.update({"active": False, "paused": False})
            self.server.robot.set_nav_mode(False)
            self.server.robot.stop()
            self.server.robot.log("event", "[导航] 点到点任务已取消")
        elif action == "pause":
            nav["paused"] = True
            self.server.robot.log("event", "[导航] 点到点任务已暂停")
        else:
            nav["paused"] = False
            self.server.robot.log("event", "[导航] 点到点任务继续")
        return {"ok": True, "action": action}

    def _state(self):
        state = self.server.robot.get_state()
        cmd_vel = self.server.tea.latest_cmd_vel()
        result = self.server.tea.latest_nav_result()
        state.update(
            {
                "sim_mode": "nav_process",
                "activity": self.server.activity,
                "config": dict(config.settings.__dict__),
                "workarea": self.server.tea.summary(),
                "nav": {
                    "active": bool(self.server.nav.get("active")),
                    "paused": bool(self.server.nav.get("paused")),
                    "goal": self.server.nav.get("goal"),
                    "cmd_vel_age": round(cmd_vel["age"], 2) if cmd_vel else None,
                    "last_result": (
                        {"outcome": result["outcome"], "text": result["text"]} if result else None
                    ),
                },
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
        self.robot.set_nav_mode(False)
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
    httpd.nav = {"active": False, "paused": False, "goal": None, "since": 0.0}
    httpd.simulator = NavSimulator(robot, tea)
    httpd.simulator.nav = httpd.nav
    httpd.simulator.start()
    if tea.summary().get("enabled"):
        tea.start()
        _auto_fetch_map(tea, robot)
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
