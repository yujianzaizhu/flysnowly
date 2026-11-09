"""tea(导航进程)联调桥: 仿真 ↔ 真实 fibo_tea 的 ZMQ+protobuf 通道。

职责(线格式与 aic-common zmq_sdk 源码一致):
- 发布 `tf_dynamic` 话题(tf2_msgs/TFMessage, map → base_link 变换): 仿真位姿
  真值 → tea slam PoseSource_Tf(当前默认)直通, MapManager 构建轨迹即采样该位姿流;
- 发布 `combined` 话题(geometry_msgs/PoseWithCovarianceStamped, 真实系统线格式):
  仿真位姿真值 → tea slam PoseSource_Fusion 直通;
- 订阅 `map_info` 话题: get_map_info 服务成功后 tea 在此发布 MapInfo(服务响应只有 result);
- REQ/REP 直连 `record_path`(20015) / `get_map_info`(20018);
- 维护客户端侧工作区会话镜像(以 tea 响应为唯一权威)与构建轨迹记录。

pyzmq/protobuf 为可选依赖: 缺失或未启用时 create() 返回 DisabledBridge(同接口 no-op),
仿真其余功能不受影响。
"""

import math
import os
import sys
import threading
import time

# ---------------------------------------------------------------------------
# 常量与错误码名称(对齐 tea map_man/legacy_adapter.hh RecordPointsError)
# ---------------------------------------------------------------------------

TRAIL_MIN_STEP = 0.05  # 构建轨迹显示抽稀门限(m), 比 tea 采样门限 0.08 略密
TRAIL_MAX_POINTS = 6000  # 显示轨迹硬上限, 超限对旧点隔点抽稀
PUB_MIN_INTERVAL = 0.05  # 位姿发布的最小仿真时间间隔(s), 倍速下按 sim_time 自适应

# tf 位姿发布(对齐 tea slam_cfg: tf_topic / global_map_frame / robot_base_frame)
TF_TOPIC = "tf_dynamic"
TF_MAP_FRAME = "map"         # tf 目标系(全局地图系)
TF_BASE_FRAME = "base_link"  # tf 源系(机器人基系)
MAP_TOPIC_WAIT = 1.5  # get_map_info 成功后等待 map_info 话题的时长(s)
SUB_RETRY_INTERVAL = 2.0  # SUB 线程断线重连间隔(s)

ELEMENT_NAMES = {1: "区域", 2: "通道", 3: "禁区"}

RECORD_ERROR_NAMES = {
    1: "NO_ENOUGH_WORKAREA_BEFORE_CREATE_TIE",
    2: "NO_WORKAREA_BEFORE_CREATE_FORBIDDEN_ZONE",
    3: "START_POINT_OF_TIE_IS_OUT_OF_MAP",
    4: "START_POINT_OF_FORBIDDEN_IS_OUT_OF_MAP",
    5: "CURRENT_POINT_IS_OUT_OF_MAP",
    6: "TOO_FEW_POINTS_BEFORE_SAVE_MAP",
    7: "NO_MAP_AND_CREATE_NOT_WORKAREA",
    8: "PATH_EMPTY_WHEN_SAVE_MAP",
    9: "START_AND_END_IS_TOO_FAR",
    10: "FORBIDDEN_CROSSING_PATH",
    11: "FORBIDDEN_NEAR_STATION",
    12: "PATH_CROSSING_FORBIDDEN",
    13: "END_POINT_OF_TIE_IS_OUT_OF_MAP",
    14: "END_POINT_OF_FORBIDDEN_IS_OUT_OF_MAP",
    15: "PART_WHITE_LIST_AREA_OUT_OF_MAP",
    16: "WRONG_MAP_DATA_STRUCT",
    17: "CANT_FIND_SPECIFIC_ID",
    18: "DELETE_SINGLE_MAP_FAIL",
    19: "LOAD_MAP_FAIL",
    20: "CHANGE_TYPE_IS_SAME_WITH_CURRENT_TYPE",
    21: "CHANGE_ID_IS_NOT_EXIST",
    22: "CHANGE_TYPE_IS_NOT_WORK_AREA",
    23: "CHOSEN_REGION_IS_NOT_FORBID_AREA",
    24: "FILE_SAVE_PATH_IS_NOT_EXIST",
    25: "TIE_ENDPOINTS_IN_SAME_AREA",
    26: "Unknow_Build_Map_Error",
}

# PathRecordCmdId(usr_define_services.proto)
CMD_ALLOW_START = 0
CMD_START = 1
CMD_ALLOW_SAVE = 2
CMD_SAVE = 3
CMD_CANCEL = 4
CMD_CLEAR = 5
CMD_POINT = 6
CMD_LOAD = 7
CMD_REGION_ATTRIBUTE_CHANGE = 8

CMD_NAMES = {
    CMD_ALLOW_START: "ALLOW_START_MAP",
    CMD_START: "START_MAP",
    CMD_ALLOW_SAVE: "ALLOW_SAVE_MAP",
    CMD_SAVE: "SAVE_MAP",
    CMD_CANCEL: "CANCEL_MAP",
    CMD_CLEAR: "CLEAR_MAP",
    CMD_POINT: "POINT",
    CMD_LOAD: "LOAD_MAP",
    CMD_REGION_ATTRIBUTE_CHANGE: "REGION_ATTRIBUTE_CHANGE",
}

# get_map_info: MapChangeRequest.cmd 语义与 proto 字段名相反
MAP_CMD_LOAD = 1
MAP_CMD_CHANGE = 2

DEFAULT_OPTS = {
    "enabled": False,
    "host": "127.0.0.1",
    "proxy_pub_port": 10001,  # 发布方连接口(proxy XSUB bind)
    "proxy_sub_port": 10002,  # 订阅方连接口(proxy XPUB bind)
    "record_port": 20015,
    "map_port": 20018,
    "nav_port": 25009,  # nav_task_action 动作服务(raw 通道用)
    "run_proxy": True,  # 内嵌 zmq proxy(端口被占时自动降级为仅连接已有 proxy)
    "service_timeout": 2.0,
}

_BACKEND_DIR = os.path.dirname(os.path.abspath(__file__))


def _import_deps():
    """延迟导入 pyzmq/protobuf 与生成的 stub; 失败抛 ImportError。"""

    try:
        import zmq  # noqa: F401
    except ImportError as exc:
        raise ImportError("缺少 pyzmq(pip install pyzmq): %s" % exc)
    try:
        import google.protobuf  # noqa: F401
    except ImportError as exc:
        raise ImportError("缺少 protobuf(pip install protobuf): %s" % exc)

    proto_dir = os.path.join(_BACKEND_DIR, "tea_proto")
    if proto_dir not in sys.path:
        sys.path.insert(0, proto_dir)
    try:
        from proto_msg import geometry_msgs_pb2, tf2_msgs_pb2
        from proto_usr_define import usr_define_msgs_pb2, usr_define_services_pb2
    except Exception as exc:
        raise ImportError(
            "tea_proto stub 不可用(运行 tools/gen_tea_proto.sh 重新生成): %s" % exc
        )
    import zmq

    return zmq, usr_define_services_pb2, usr_define_msgs_pb2, geometry_msgs_pb2, tf2_msgs_pb2


def yaw_to_quat(yaw):
    """平面 yaw -> 四元数 (x, y, z, w)。"""

    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quat_to_yaw(x, y, z, w):
    """四元数 -> 平面 yaw。"""

    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def error_text(result):
    """record_path result 数值 -> 可读文本(未知数值原样)。"""

    name = RECORD_ERROR_NAMES.get(result)
    return "%d(%s)" % (result, name) if name else str(result)


# ---------------------------------------------------------------------------
# 内嵌 zmq proxy(XSUB/XPUB 转发, 对齐 zmq_proxy_pc_node.py)
# ---------------------------------------------------------------------------


class ZmqProxyThread(threading.Thread):
    """XSUB/XPUB 轮询转发(等价 zmq.proxy, 但 stop() 可确定退出)。

    zmq.proxy 在 libzmq 内部阻塞, 外部 term 无法可靠唤醒, 服务停机时会挂住
    shutdown; 这里用 poll 循环手动双向转发(XSUB<->XPUB 的订阅帧按普通消息
    透传即可), stop 只需置位 + 等一个 poll 周期。
    """

    def __init__(self, zmq_mod, host, pub_port, sub_port):
        super().__init__(name="tea-proxy", daemon=True)
        self.zmq_mod = zmq_mod
        self.host = host
        self.pub_port = pub_port
        self.sub_port = sub_port
        self.context = None
        self._stop_event = threading.Event()

    def run(self):
        frontend = None
        backend = None
        try:
            self.context = self.zmq_mod.Context(1)
            frontend = self.context.socket(self.zmq_mod.XSUB)
            frontend.set(self.zmq_mod.LINGER, 0)
            frontend.bind("tcp://%s:%d" % (self.host, self.pub_port))
            backend = self.context.socket(self.zmq_mod.XPUB)
            backend.set(self.zmq_mod.LINGER, 0)
            backend.bind("tcp://%s:%d" % (self.host, self.sub_port))
            poller = self.zmq_mod.Poller()
            poller.register(frontend, self.zmq_mod.POLLIN)
            poller.register(backend, self.zmq_mod.POLLIN)
            while not self._stop_event.is_set():
                for sock, flags in poller.poll(200):
                    if not flags & self.zmq_mod.POLLIN:
                        continue
                    other = backend if sock is frontend else frontend
                    while True:
                        try:
                            frames = sock.recv_multipart(self.zmq_mod.NOBLOCK)
                        except self.zmq_mod.ZMQError:
                            break
                        try:
                            other.send_multipart(frames, self.zmq_mod.NOBLOCK)
                        except self.zmq_mod.ZMQError:
                            pass
        except Exception:
            # stop() 关闭 context 时走到这里, 属正常退出; 其余异常由 start() 侧
            # bind 预检兜底, 这里静默结束线程
            pass
        finally:
            for sock in (frontend, backend):
                if sock is not None:
                    try:
                        sock.close(linger=0)
                    except Exception:
                        pass
            if self.context is not None:
                try:
                    self.context.term()
                except Exception:
                    pass

    def stop(self):
        self._stop_event.set()


# ---------------------------------------------------------------------------
# REQ/REP 服务客户端(单帧 protobuf, 超时重建 socket 防 REQ 僵死)
# ---------------------------------------------------------------------------


class ServiceTimeout(Exception):
    pass


class ZmqServiceClient:
    def __init__(self, zmq_mod, host, port, timeout=2.0):
        self.zmq_mod = zmq_mod
        self.endpoint = "tcp://%s:%d" % (host, port)
        self.timeout = timeout
        self._context = zmq_mod.Context(1)
        self._socket = None

    def _connect(self):
        if self._socket is not None:
            try:
                self._socket.close(linger=0)
            except Exception:
                pass
        sock = self._context.socket(self.zmq_mod.REQ)
        sock.set(self.zmq_mod.LINGER, 0)
        sock.connect(self.endpoint)
        self._socket = sock

    def call(self, request_payload):
        """发送序列化请求, 返回序列化响应; 超时抛 ServiceTimeout。"""

        if self._socket is None:
            self._connect()
        try:
            self._socket.send(request_payload)
            if not self._socket.poll(self.timeout * 1000):
                raise ServiceTimeout("服务无响应: %s" % self.endpoint)
            return self._socket.recv()
        except ServiceTimeout:
            # REQ 进入发送态后必须重建 socket 才能复用
            self._connect()
            raise
        except Exception:
            self._connect()
            raise

    def close(self):
        if self._socket is not None:
            try:
                self._socket.close(linger=0)
            except Exception:
                pass
            self._socket = None
        try:
            self._context.term()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# 桥主体
# ---------------------------------------------------------------------------


class TeaBridge:
    def __init__(self, opts=None, log_fn=None):
        merged = dict(DEFAULT_OPTS)
        for key in DEFAULT_OPTS:
            if opts and key in opts and opts[key] is not None:
                merged[key] = opts[key]
        self.opts = merged
        self.log_fn = log_fn or (lambda level, msg: None)

        self.zmq_mod, self.svc_pb, self.msgs_pb, self.geo_pb, self.tf_pb = _import_deps()
        from proto_usr_define import usr_define_actions_pb2

        self.act_pb = usr_define_actions_pb2

        self.session = {"state": "idle", "type": None, "stale": False, "since": None}
        self.last_error = None
        self.connected = False  # 最近一次服务调用是否收到响应

        self._pub_socket = None
        self._pub_lock = threading.Lock()
        self._last_pub_sim = None
        self._pub_count = 0
        self._tf_pub_count = 0

        self._state_lock = threading.Lock()
        self._trail = []  # [[x, y], ...] 构建轨迹(仅 building 时累积)
        self._trail_len = 0.0

        self._map_lock = threading.Lock()
        self._map_cond = threading.Condition(self._map_lock)
        self._map_cache = None  # 转成 dict 的 MapInfo
        self._map_version = 0

        self._call_lock = threading.Lock()
        self._record_client = None
        self._map_client = None
        self._nav_client = None

        self._sub_stop = threading.Event()
        self._sub_thread = None
        self._proxy = None
        self._started = False

    # ------------------------------------------------------------- 生命周期

    def start(self):
        if self._started:
            return
        if self.opts["run_proxy"]:
            try:
                probe_ctx = self.zmq_mod.Context(1)
                sock = probe_ctx.socket(self.zmq_mod.XSUB)
                sock.bind("tcp://%s:%d" % (self.opts["host"], self.opts["proxy_pub_port"]))
                sock.close(linger=0)
                probe_ctx.term()
                self._proxy = ZmqProxyThread(
                    self.zmq_mod, self.opts["host"], self.opts["proxy_pub_port"], self.opts["proxy_sub_port"]
                )
                self._proxy.start()
                self.log("event", "[工作区] 内嵌 zmq proxy 已启动(10001/10002)")
            except Exception as exc:
                self._proxy = None
                self.log(
                    "warn",
                    "[工作区] proxy 端口绑定失败(%s), 降级为连接已有 proxy" % exc,
                )
        try:
            ctx = self.zmq_mod.Context(1)
            pub = ctx.socket(self.zmq_mod.PUB)
            pub.set(self.zmq_mod.LINGER, 0)
            pub.set(self.zmq_mod.SNDHWM, 100)
            pub.connect("tcp://%s:%d" % (self.opts["host"], self.opts["proxy_pub_port"]))
            self._pub_context = ctx
            self._pub_socket = pub
        except Exception as exc:
            self.last_error = "combined 发布通道创建失败: %s" % exc
            self.log("warn", "[工作区] " + self.last_error)

        self._record_client = ZmqServiceClient(
            self.zmq_mod, self.opts["host"], self.opts["record_port"], self.opts["service_timeout"]
        )
        self._map_client = ZmqServiceClient(
            self.zmq_mod, self.opts["host"], self.opts["map_port"], self.opts["service_timeout"]
        )
        self._nav_client = ZmqServiceClient(
            self.zmq_mod, self.opts["host"], self.opts["nav_port"], self.opts["service_timeout"]
        )

        self._sub_stop.clear()
        self._sub_thread = threading.Thread(target=self._sub_loop, name="tea-sub", daemon=True)
        self._sub_thread.start()
        self._started = True
        self.log(
            "event",
            "[工作区] tea 桥已启动: %s (record_path=%d, get_map_info=%d)"
            % (self.opts["host"], self.opts["record_port"], self.opts["map_port"]),
        )

    def stop(self):
        if not self._started:
            return
        self._started = False
        self._sub_stop.set()
        if self._sub_thread is not None:
            self._sub_thread.join(timeout=3.0)
        for client in (self._record_client, self._map_client, self._nav_client):
            if client is not None:
                client.close()
        with self._pub_lock:
            if self._pub_socket is not None:
                try:
                    self._pub_socket.close(linger=0)
                except Exception:
                    pass
                self._pub_socket = None
            ctx = getattr(self, "_pub_context", None)
            if ctx is not None:
                try:
                    ctx.term()
                except Exception:
                    pass
                self._pub_context = None
        if self._proxy is not None:
            self._proxy.stop()
            self._proxy = None
        self.log("event", "[工作区] tea 桥已停止")

    def log(self, level, message):
        try:
            self.log_fn(level, message)
        except Exception:
            pass

    # ----------------------------------------------------------- 仿真 tick

    def tick(self, robot):
        """Simulator.on_cycle 调用: 发布 tf_dynamic/combined 位姿 + building 时记录轨迹。

        robot 在切图时会被替换, 必须每次现取, 不缓存引用。
        """

        if not self._started or robot is None:
            return
        sim_time = getattr(robot, "sim_time", 0.0)
        pose = getattr(robot, "pose", None)
        if pose is None or len(pose) < 3:
            return
        # 切图后 sim_time 归零: 重置节流与轨迹锚点, 避免跨图连线
        if self._last_pub_sim is not None and sim_time < self._last_pub_sim:
            self._last_pub_sim = None
            with self._state_lock:
                if self._trail:
                    self._trail = []
        if self._last_pub_sim is not None and sim_time - self._last_pub_sim < PUB_MIN_INTERVAL:
            return
        self._last_pub_sim = sim_time
        self._publish_tf(pose, sim_time)
        self._publish_combined(pose)
        with self._state_lock:
            if self.session["state"] == "building":
                self._append_trail(pose[0], pose[1])

    def _publish_tf(self, pose, sim_time):
        """tf_dynamic: TFMessage 携带一条 map → base_link 变换(tea slam PoseSource_Tf)。"""

        msg = self.tf_pb.TFMessage()
        tfs = msg.transforms.add()
        tfs.header.frame_id = TF_MAP_FRAME
        tfs.header.stamp = int(sim_time * 1e6)  # us, 与 tea tic::StampUs 约定一致
        tfs.child_frame_id = TF_BASE_FRAME
        tfs.transform.translation.x = float(pose[0])
        tfs.transform.translation.y = float(pose[1])
        qx, qy, qz, qw = yaw_to_quat(float(pose[2]))
        tfs.transform.rotation.x = qx
        tfs.transform.rotation.y = qy
        tfs.transform.rotation.z = qz
        tfs.transform.rotation.w = qw
        with self._pub_lock:
            if self._pub_socket is None:
                return
            try:
                self._pub_socket.send_multipart([TF_TOPIC.encode(), msg.SerializeToString()])
                self._tf_pub_count += 1
            except Exception as exc:
                self.last_error = "tf_dynamic 发布失败: %s" % exc

    def _publish_combined(self, pose):
        """combined: 真实系统线格式为 PoseWithCovarianceStamped(tea PoseSource_Fusion)。"""

        msg = self.geo_pb.PoseWithCovarianceStamped()
        msg.pose.pose.position.x = float(pose[0])
        msg.pose.pose.position.y = float(pose[1])
        qx, qy, qz, qw = yaw_to_quat(float(pose[2]))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        with self._pub_lock:
            if self._pub_socket is None:
                return
            try:
                self._pub_socket.send_multipart([b"combined", msg.SerializeToString()])
                self._pub_count += 1
            except Exception as exc:
                self.last_error = "combined 发布失败: %s" % exc

    def _append_trail(self, x, y):
        if self._trail:
            last = self._trail[-1]
            if math.hypot(x - last[0], y - last[1]) < TRAIL_MIN_STEP:
                return
            self._trail_len += math.hypot(x - last[0], y - last[1])
        self._trail.append([round(x, 2), round(y, 2)])
        if len(self._trail) > TRAIL_MAX_POINTS:
            keep = self._trail[::2]
            self._trail = keep

    # ----------------------------------------------------------- 服务调用

    def _call_record(self, request):
        payload = request.SerializeToString()
        with self._call_lock:
            if self._record_client is None:
                raise RuntimeError("桥未启动")
            data = self._record_client.call(payload)
        self.connected = True
        response = self.svc_pb.PathRecordResponse.FromString(data)
        result = {
            "ok": response.result == 0,
            "result": int(response.result),
            "info": response.info or "",
            "point_count": len(response.array.point_x),
            "points": [
                [round(x, 2), round(y, 2)]
                for x, y in zip(response.array.point_x, response.array.point_y)
            ],
            "added": list(response.area_change_info.added_area),
            "deleted": list(response.area_change_info.deleted_area),
        }
        return result

    def record_path(self, cmd, type_=None, id_=None, deleted_ids=None, path_name=None):
        """调用 record_path 服务, 返回统一 dict; 传输异常返回 ok=False, error=...。"""

        request = self.svc_pb.PathRecordRequest()
        request.command = cmd
        if type_ is not None:
            request.type = type_
        if id_ is not None:
            request.id = id_
        if deleted_ids:
            request.deleted_ids.extend(int(i) for i in deleted_ids)
        if path_name:
            request.path_name = path_name
        try:
            result = self._call_record(request)
        except Exception as exc:
            self.connected = False
            self.last_error = "%s 调用失败: %s" % (CMD_NAMES.get(cmd, cmd), exc)
            return {"ok": False, "result": None, "info": self.last_error, "error": str(exc)}
        # 老协议特例: CLEAR_MAP 空 ids(整图清空)成功时 result=1
        if cmd == CMD_CLEAR and not deleted_ids and result["result"] in (0, 1):
            result["ok"] = True
        return result

    def get_map_info(self, cmd=MAP_CMD_LOAD, wait=True):
        """调用 get_map_info; 成功后等待 map_info 话题并刷新缓存。

        返回 {ok, result, map_version?, map_error?}。等待失败按用户要求报错:
        写日志并在返回值带 map_error。
        """

        # 必须在服务调用前记录版本：响应快的 tea 可能在 REP 返回前已经发布
        # map_info；调用后再取 baseline 会漏掉本次话题并无谓重试。
        with self._map_cond:
            baseline = self._map_version
        request = self.svc_pb.MapChangeRequest()
        request.cmd = cmd
        try:
            with self._call_lock:
                if self._map_client is None:
                    raise RuntimeError("桥未启动")
                data = self._map_client.call(request.SerializeToString())
            self.connected = True
        except Exception as exc:
            self.connected = False
            self.last_error = "get_map_info 调用失败: %s" % exc
            return {"ok": False, "result": None, "info": str(exc), "map_error": "service_failed"}

        response = self.svc_pb.MapChangeResponse.FromString(data)
        if response.result != 0:
            return {"ok": False, "result": int(response.result), "info": "读图失败"}

        if not wait:
            return {"ok": True, "result": 0}
        # 等待"本次调用之后"的新发布(按版本号增量), 避免命中调用前的旧缓存
        for attempt in (1, 2):
            deadline = time.monotonic() + MAP_TOPIC_WAIT
            with self._map_cond:
                while self._map_version <= baseline and time.monotonic() < deadline:
                    self._map_cond.wait(timeout=max(0.01, deadline - time.monotonic()))
                if self._map_version > baseline:
                    return {"ok": True, "result": 0, "map_version": self._map_version}
            # 话题未到: 重试一次服务调用(tea 每次成功都会再发布一次 map_info)
            if attempt == 1:
                try:
                    with self._call_lock:
                        data = self._map_client.call(request.SerializeToString())
                    if self.svc_pb.MapChangeResponse.FromString(data).result != 0:
                        break
                except Exception:
                    break
        message = "get_map_info 成功但 map_info 话题未收到(等待超时 %.1fs)" % MAP_TOPIC_WAIT
        self.last_error = message
        self.log("warn", "[工作区] " + message)
        return {"ok": True, "result": 0, "map_error": "topic_timeout", "info": message}

    def probe(self):
        """连通性探测: ALLOW_START_MAP(纯校验命令, 收到响应即在线)。"""

        result = self.record_path(CMD_ALLOW_START)
        return {"ok": self.connected, "connected": self.connected, "result": result.get("result")}

    def nav_action(self, cmd, goal=None):
        """调用 nav_task_action 动作服务(raw 通道; 未知 cmd 会被 tea 复位为 Idle)。"""

        request = self.act_pb.NavTaskActionGoal()
        request.cmd = cmd
        if goal:
            if goal.get("mode") is not None:
                request.goal.mode = int(goal["mode"])
            target = goal.get("target_pose") or {}
            if target:
                pose = request.goal.target_pose.pose
                pose.position.x = float(target.get("x", 0.0))
                pose.position.y = float(target.get("y", 0.0))
        try:
            with self._call_lock:
                if self._nav_client is None:
                    raise RuntimeError("桥未启动")
                data = self._nav_client.call(request.SerializeToString())
            self.connected = True
        except Exception as exc:
            self.connected = False
            self.last_error = "nav_task_action 调用失败: %s" % exc
            return {"ok": False, "result": None, "info": str(exc), "error": str(exc)}
        response = self.act_pb.ActionResponse.FromString(data)
        return {"ok": True, "status": int(response.status), "msgs": response.msgs}

    # ------------------------------------------------------------- 会话镜像

    def begin_session(self, type_):
        with self._state_lock:
            self.session = {"state": "building", "type": type_, "stale": False, "since": time.time()}
            self._trail = []
            self._trail_len = 0.0

    def end_session(self):
        with self._state_lock:
            self.session = {"state": "idle", "type": None, "stale": False, "since": None}
            self._trail = []
            self._trail_len = 0.0

    def mark_stale(self):
        with self._state_lock:
            self.session["stale"] = True

    def is_building(self):
        with self._state_lock:
            return self.session["state"] == "building"

    # ------------------------------------------------------------ map 订阅

    def _sub_loop(self):
        sub = None
        ctx = None
        try:
            while not self._sub_stop.is_set():
                try:
                    if sub is None:
                        ctx = self.zmq_mod.Context(1)
                        sub = ctx.socket(self.zmq_mod.SUB)
                        sub.set(self.zmq_mod.LINGER, 0)
                        sub.set(self.zmq_mod.RCVHWM, 10)
                        sub.set(self.zmq_mod.SUBSCRIBE, b"map_info")
                        sub.connect("tcp://%s:%d" % (self.opts["host"], self.opts["proxy_sub_port"]))
                    if not sub.poll(500):
                        continue
                    frames = sub.recv_multipart()
                    if len(frames) < 2 or frames[0] != b"map_info":
                        continue
                    info = self.msgs_pb.MapInfo.FromString(frames[1])
                    parsed = self._map_info_to_dict(info)
                    with self._map_cond:
                        self._map_cache = parsed
                        self._map_version += 1
                        self._map_cond.notify_all()
                except Exception:
                    self.last_error = "map_info 订阅中断, %ds 后重连" % int(SUB_RETRY_INTERVAL)
                    if sub is not None:
                        try:
                            sub.close(linger=0)
                        except Exception:
                            pass
                        sub = None
                    if ctx is not None:
                        try:
                            ctx.term()
                        except Exception:
                            pass
                        ctx = None
                    if self._sub_stop.wait(SUB_RETRY_INTERVAL):
                        break
        finally:
            if sub is not None:
                try:
                    sub.close(linger=0)
                except Exception:
                    pass
            if ctx is not None:
                try:
                    ctx.term()
                except Exception:
                    pass

    def _map_info_to_dict(self, info):
        sub_maps = []
        for sub in info.sub_maps:
            sub_maps.append(
                {
                    "type": int(sub.type),
                    "name": sub.name,
                    "obj_id": int(sub.obj_id),
                    "area": round(sub.area, 2),
                    "pose_count": int(sub.pose_count),
                    "poses": [
                        [round(pose.position.x, 3), round(pose.position.y, 3)]
                        for pose in sub.poses
                    ],
                }
            )
        # origin 仅透传供参考; 绘制按用户决策只使用 poses 绝对坐标
        origin = info.origin
        return {
            "res": round(info.res, 4),
            "area": round(info.area, 2),
            "total_pose_count": int(info.total_pose_count),
            "station_pose": [info.station_pose_x, info.station_pose_y],
            "recharge_pose": [info.recharge_pose_x, info.recharge_pose_y],
            "recharge_yaw": round(
                quat_to_yaw(
                    info.recharge_orientation_x,
                    info.recharge_orientation_y,
                    info.recharge_orientation_z,
                    info.recharge_orientation_w,
                ),
                3,
            ),
            "origin": [origin.position.x, origin.position.y],
            "sub_maps": sub_maps,
        }

    def map_snapshot(self, version=None):
        """版本相同返回 {unchanged}; 否则返回全量地图 dict。"""

        with self._map_lock:
            if self._map_cache is None:
                return {"version": self._map_version, "map": None}
            if version is not None and version == self._map_version:
                return {"version": self._map_version, "unchanged": True}
            return {"version": self._map_version, "map": self._map_cache}

    # ---------------------------------------------------------------- 摘要

    def summary(self):
        with self._state_lock:
            session = dict(self.session)
            trail = list(self._trail)
            trail_len = self._trail_len
        return {
            "enabled": True,
            "connected": self.connected,
            "host": self.opts["host"],
            "record_port": self.opts["record_port"],
            "map_port": self.opts["map_port"],
            "nav_port": self.opts["nav_port"],
            "session": session,
            "trail": trail,
            "trail_len": round(trail_len, 1),
            "map_version": self._map_version,
            "pub_count": self._pub_count,
            "tf_pub_count": self._tf_pub_count,
            "last_error": self.last_error,
        }


class DisabledBridge:
    """未启用/依赖缺失时的同接口 no-op 实现。"""

    def __init__(self, reason="未启用"):
        self.reason = reason

    def start(self):
        pass

    def stop(self):
        pass

    def tick(self, robot):
        pass

    def record_path(self, cmd, type_=None, id_=None, deleted_ids=None, path_name=None):
        return {"ok": False, "result": None, "info": "工作区桥未启用: %s" % self.reason, "error": "disabled"}

    def get_map_info(self, cmd=MAP_CMD_LOAD, wait=True):
        return {"ok": False, "result": None, "info": "工作区桥未启用: %s" % self.reason, "map_error": "disabled"}

    def probe(self):
        return {"ok": False, "connected": False, "result": None}

    def nav_action(self, cmd, goal=None):
        return {"ok": False, "result": None, "info": "工作区桥未启用: %s" % self.reason}

    def begin_session(self, type_):
        pass

    def end_session(self):
        pass

    def mark_stale(self):
        pass

    def is_building(self):
        return False

    def map_snapshot(self, version=None):
        return {"version": 0, "map": None}

    def summary(self):
        return {"enabled": False, "connected": False, "reason": self.reason, "session": {"state": "idle", "type": None, "stale": False, "since": None}}


def create(opts=None, log_fn=None):
    """工厂: 依赖可用且 enabled 时返回 TeaBridge, 否则 DisabledBridge。"""

    merged = dict(DEFAULT_OPTS)
    for key in DEFAULT_OPTS:
        if opts and key in opts and opts[key] is not None:
            merged[key] = opts[key]
    if not merged["enabled"]:
        return DisabledBridge("未启用，请用 --tea-bridge 启动或切换到接入导航进程")
    try:
        return TeaBridge(merged, log_fn=log_fn)
    except ImportError as exc:
        return DisabledBridge(str(exc))
