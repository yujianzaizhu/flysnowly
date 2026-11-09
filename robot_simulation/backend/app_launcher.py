"""统一启动入口：同一地址下运行两套相互隔离的仿真应用。"""

import argparse
import os
import socket
import sys
import threading
import time

BACKEND_DIR = os.path.dirname(os.path.abspath(__file__))
if BACKEND_DIR not in sys.path:
    sys.path.insert(0, BACKEND_DIR)

import mode_control
import server as standalone_server
from nav_process import server as nav_server


class SwitchConflict(ValueError):
    http_status = 409


class ConnectionRegistry(object):
    """记录 HTTP 长连接，应用销毁时主动关闭，避免旧应用继续响应。"""

    def __init__(self, httpd):
        self._active = set()
        self._lock = threading.Lock()
        self._process_request = httpd.process_request
        self._shutdown_request = httpd.shutdown_request
        httpd.process_request = self._track_request
        httpd.shutdown_request = self._untrack_request

    def _track_request(self, request, client_address):
        with self._lock:
            self._active.add(request)
        try:
            return self._process_request(request, client_address)
        except Exception:
            self._discard(request)
            raise

    def _untrack_request(self, request):
        self._discard(request)
        return self._shutdown_request(request)

    def _discard(self, request):
        with self._lock:
            self._active.discard(request)

    def close(self):
        with self._lock:
            connections = list(self._active)
            self._active.clear()
        for connection in connections:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                connection.close()
            except OSError:
                pass


class StandaloneRequestHandler(standalone_server.RobotRequestHandler):
    """只给原内部仿真增加模式接口，其余路由完整交给原实现。"""

    def do_GET(self):
        if mode_control.handle(self, "GET"):
            return
        super().do_GET()

    def do_POST(self):
        if mode_control.handle(self, "POST"):
            return
        super().do_POST()


class ApplicationController(object):
    def __init__(self, host="127.0.0.1", port=8000, mode="standalone", tea_opts=None):
        if mode not in ("standalone", "nav_process"):
            raise ValueError("未知仿真类型: {}".format(mode))
        self.host = host
        self.port = port
        self.mode = mode
        self.tea_opts = dict(tea_opts or {})
        self.tea_opts.setdefault("enabled", mode == "nav_process")
        self.httpd = None
        self.switching = False
        self.target_mode = None
        self._next_mode = None
        self._stopped = False
        self._lock = threading.RLock()
        self.ready = threading.Event()

    def snapshot(self):
        with self._lock:
            return {
                "mode": self.mode,
                "switching": self.switching,
                "target": self.target_mode,
            }

    def request_switch(self, httpd, target):
        if target not in ("standalone", "nav_process"):
            raise ValueError("仿真类型仅支持 standalone/nav_process")
        with self._lock:
            if self.switching:
                raise SwitchConflict("正在切换仿真类型，请稍候")
            if target == self.mode:
                return self.snapshot()
            if self.mode == "nav_process" and httpd.tea.is_building():
                raise SwitchConflict("工作区构建中，请先保存或取消，再切换仿真类型")
            self._prepare_stop(httpd)
            self.switching = True
            self.target_mode = target
            self._next_mode = target

        def _shutdown():
            time.sleep(0.15)
            httpd.shutdown()

        threading.Thread(target=_shutdown, name="sim-mode-switch", daemon=True).start()
        return self.snapshot()

    def _prepare_stop(self, httpd):
        """切换响应返回前停止所有界面动作，但不销毁响应所需对象。"""
        if self.mode == "standalone":
            httpd.robot.handle_command("stop")
            manager = httpd.manager
            if manager.task is not None:
                manager.stop_task("切换仿真类型，当前任务已取消", resumable=False)
            if getattr(manager.replay, "active", False):
                manager.stop_replay()
        else:
            httpd.robot.stop()

    def _stop_application(self, httpd, mode):
        if mode == "standalone":
            httpd.robot.handle_command("stop")
            httpd.simulator.stop()
        else:
            httpd.stop_application()

    def _create_application(self):
        if self.mode == "standalone":
            httpd = standalone_server.create_server(self.host, self.port)
            httpd.RequestHandlerClass = StandaloneRequestHandler
        else:
            opts = dict(self.tea_opts)
            opts["enabled"] = True
            httpd = nav_server.create_server(self.host, self.port, opts)
        httpd.mode_controller = self
        httpd.connections = ConnectionRegistry(httpd)
        self.httpd = httpd
        self.port = httpd.server_address[1]
        return httpd

    def run(self):
        try:
            while not self._stopped:
                with self._lock:
                    self.switching = False
                    self.target_mode = None
                self.ready.clear()
                httpd = self._create_application()
                print("仿真控制台已启动: http://{}:{}/ ({})".format(self.host, self.port, self.mode), flush=True)
                self.ready.set()
                try:
                    httpd.serve_forever()
                finally:
                    # 先断开浏览器复用的长连接，防止旧应用在切换间隙继续响应。
                    httpd.connections.close()
                    self._stop_application(httpd, self.mode)
                    httpd.server_close()
                with self._lock:
                    if self._stopped or self._next_mode is None:
                        break
                    self.mode = self._next_mode
                    self._next_mode = None
        except KeyboardInterrupt:
            self._stopped = True
        finally:
            self.ready.set()

    def stop(self):
        with self._lock:
            self._stopped = True
            self._next_mode = None
            httpd = self.httpd
        if httpd is not None:
            threading.Thread(target=httpd.shutdown, daemon=True).start()


def main(argv=None):
    parser = argparse.ArgumentParser(description="机器人控制台双模式启动入口")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument(
        "--mode",
        choices=("standalone", "nav_process"),
        default="standalone",
        help="初始仿真类型，默认内部仿真",
    )
    parser.add_argument("--tea-bridge", action="store_true", help="配置 tea 桥接，不改变默认内部仿真")
    args, _ = parser.parse_known_args(argv)
    mode = args.mode
    controller = ApplicationController(
        args.host,
        args.port,
        mode,
        tea_opts=nav_server.tea_opts_from_env(argv),
    )
    controller.run()


if __name__ == "__main__":
    main()
