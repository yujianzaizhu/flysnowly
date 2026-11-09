"""tea_bridge 桥核心测试: 本地起 zmq proxy + 假 tea(REP 服务 + PUB map_info),
不依赖真实 fibo_tea 进程。pyzmq/protobuf 缺失时跳过。
"""

import os
import socket
import sys
import threading
import time
import unittest

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
BACKEND_DIR = os.path.join(os.path.dirname(TEST_DIR), "backend")
sys.path.insert(0, BACKEND_DIR)
sys.path.insert(0, os.path.join(BACKEND_DIR, "nav_process", "tea_proto"))

try:
    import zmq  # noqa: F401

    _ZMQ_OK = True
except Exception:
    _ZMQ_OK = False

try:
    from nav_process import tea_bridge  # noqa: E402

    _BRIDGE_OK = _ZMQ_OK and os.path.isdir(os.path.join(BACKEND_DIR, "nav_process", "tea_proto"))
except Exception:
    _BRIDGE_OK = False


def _free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


class FakeRobot:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.pose = [x, y, theta]
        self.sim_time = 0.0


class FakeTea:
    """假导航端: record_path/get_map_info REP 服务 + map_info 话题发布。"""

    def __init__(self, zmq_mod, pub_port):
        self.zmq_mod = zmq_mod
        self.pub_port = pub_port
        self.context = zmq_mod.Context(1)
        self.record_requests = []
        self.map_requests = []
        self.record_script = {}  # cmd -> callable(req, resp)
        self.map_result = 0
        self.publish_map = True
        self.map_info_builder = self._default_map_info
        self.pub = None
        self.threads = []
        self.sockets = []
        self._stop = threading.Event()

    @staticmethod
    def _default_map_info(msgs_pb):
        info = msgs_pb.MapInfo()
        info.res = 0.05
        info.area = 42.0
        info.total_pose_count = 7
        info.station_pose_x = 1.0
        info.station_pose_y = 2.0
        info.recharge_pose_x = 3.0
        info.recharge_pose_y = 4.0
        info.recharge_orientation_z = 0.0
        info.recharge_orientation_w = 1.0
        sub = info.sub_maps.add()
        sub.type = 1
        sub.obj_id = 1
        sub.pose_count = 3
        for x, y in ((0, 0), (4, 0), (4, 3)):
            pose = sub.poses.add()
            pose.position.x = x
            pose.position.y = y
        return info

    def start(self, record_port, map_port, msgs_pb, svc_pb):
        self.msgs_pb = msgs_pb
        self.svc_pb = svc_pb
        self.pub = self.context.socket(self.zmq_mod.PUB)
        self.pub.set(self.zmq_mod.LINGER, 0)
        self.pub.connect("tcp://127.0.0.1:%d" % self.pub_port)
        for port, handler in ((record_port, self._record_serve), (map_port, self._map_serve)):
            sock = self.context.socket(self.zmq_mod.REP)
            sock.set(self.zmq_mod.LINGER, 0)
            sock.bind("tcp://127.0.0.1:%d" % port)
            self.sockets.append(sock)
            thread = threading.Thread(target=handler, args=(sock,), daemon=True)
            self.threads.append(thread)
            thread.start()

    def _record_serve(self, sock):
        while not self._stop.is_set():
            try:
                if not sock.poll(200):
                    continue
                data = sock.recv()
                req = self.svc_pb.PathRecordRequest.FromString(data)
                self.record_requests.append(req)
                resp = self.svc_pb.PathRecordResponse()
                handler = self.record_script.get(req.command)
                if handler is not None:
                    handler(req, resp)
                else:
                    resp.result = 0
                    resp.info = "ok"
                sock.send(resp.SerializeToString())
            except Exception:
                return

    def _map_serve(self, sock):
        while not self._stop.is_set():
            try:
                if not sock.poll(200):
                    continue
                data = sock.recv()
                req = self.svc_pb.MapChangeRequest.FromString(data)
                self.map_requests.append(req)
                resp = self.svc_pb.MapChangeResponse()
                resp.result = self.map_result
                sock.send(resp.SerializeToString())
                if resp.result == 0 and self.publish_map:
                    self.publish_map_info()
            except Exception:
                return

    def publish_map_info(self):
        info = self.map_info_builder(self.msgs_pb)
        self.pub.send_multipart([b"map_info", info.SerializeToString()])

    def stop(self):
        self._stop.set()
        for thread in self.threads:
            thread.join(timeout=1.0)
        for sock in self.sockets:
            try:
                sock.close(linger=0)
            except Exception:
                pass
        if self.pub is not None:
            try:
                self.pub.close(linger=0)
            except Exception:
                pass
        try:
            self.context.term()
        except Exception:
            pass


@unittest.skipUnless(_BRIDGE_OK, "tea_bridge 依赖(pyzmq/protobuf/tea_proto)不可用")
class TeaBridgeTest(unittest.TestCase):
    def setUp(self):
        self.zmq = zmq
        self.proxy_pub = _free_port()
        self.proxy_sub = _free_port()
        self.record_port = _free_port()
        self.map_port = _free_port()
        self.logs = []

        self.proxy = tea_bridge.ZmqProxyThread(self.zmq, "127.0.0.1", self.proxy_pub, self.proxy_sub)
        self.proxy.start()

        self.fake = FakeTea(self.zmq, self.proxy_pub)
        _, svc_pb, msgs_pb, _, _ = tea_bridge._import_deps()
        self.fake.start(self.record_port, self.map_port, msgs_pb, svc_pb)

        self.bridge = tea_bridge.TeaBridge(
            {
                "enabled": True,
                "host": "127.0.0.1",
                "proxy_pub_port": self.proxy_pub,
                "proxy_sub_port": self.proxy_sub,
                "record_port": self.record_port,
                "map_port": self.map_port,
                "run_proxy": False,
                "service_timeout": 1.0,
            },
            log_fn=lambda level, msg: self.logs.append((level, msg)),
        )
        self.bridge.start()
        time.sleep(0.2)  # PUB/SUB 订阅传播

    def tearDown(self):
        self.bridge.stop()
        self.fake.stop()
        self.proxy.stop()
        self.proxy.join(timeout=1.0)

    def _capture_sub(self, topic):
        sub = self.zmq.Context(1).socket(self.zmq.SUB)
        sub.set(self.zmq.LINGER, 0)
        sub.set(self.zmq.SUBSCRIBE, topic)
        sub.connect("tcp://127.0.0.1:%d" % self.proxy_sub)
        time.sleep(0.2)  # 订阅传播
        return sub

    def test_combined_publish_wire_format(self):
        from proto_msg import geometry_msgs_pb2

        sub = self._capture_sub(b"combined")
        try:
            robot = FakeRobot(1.5, -2.25, 0.7)
            published = False
            for _ in range(50):
                robot.sim_time += 0.1
                self.bridge.tick(robot)
                if not sub.poll(100):
                    continue
                topic, payload = sub.recv_multipart()
                self.assertEqual(topic, b"combined")
                msg = geometry_msgs_pb2.PoseWithCovarianceStamped.FromString(payload)
                self.assertAlmostEqual(msg.pose.pose.position.x, 1.5)
                self.assertAlmostEqual(msg.pose.pose.position.y, -2.25)
                yaw = tea_bridge.quat_to_yaw(
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w,
                )
                self.assertAlmostEqual(yaw, 0.7, places=3)
                published = True
                break
            self.assertTrue(published, "未收到 combined 消息")
        finally:
            sub.close(0)
            sub.context.term()

    def test_tf_publish_wire_format(self):
        """tf_dynamic: TFMessage 应携带 map → base_link 变换, 平移/yaw 即仿真位姿。"""

        from proto_msg import tf2_msgs_pb2

        sub = self._capture_sub(tea_bridge.TF_TOPIC.encode())
        try:
            robot = FakeRobot(2.5, 3.25, -1.2)
            published = False
            for _ in range(50):
                robot.sim_time += 0.1
                self.bridge.tick(robot)
                if not sub.poll(100):
                    continue
                topic, payload = sub.recv_multipart()
                self.assertEqual(topic, tea_bridge.TF_TOPIC.encode())
                msg = tf2_msgs_pb2.TFMessage.FromString(payload)
                self.assertEqual(len(msg.transforms), 1)
                tfs = msg.transforms[0]
                self.assertEqual(tfs.header.frame_id, tea_bridge.TF_MAP_FRAME)
                self.assertEqual(tfs.child_frame_id, tea_bridge.TF_BASE_FRAME)
                self.assertAlmostEqual(tfs.transform.translation.x, 2.5)
                self.assertAlmostEqual(tfs.transform.translation.y, 3.25)
                self.assertAlmostEqual(tfs.transform.translation.z, 0.0)
                yaw = tea_bridge.quat_to_yaw(
                    tfs.transform.rotation.x,
                    tfs.transform.rotation.y,
                    tfs.transform.rotation.z,
                    tfs.transform.rotation.w,
                )
                self.assertAlmostEqual(yaw, -1.2, places=3)
                published = True
                break
            self.assertTrue(published, "未收到 tf_dynamic 消息")
        finally:
            sub.close(0)
            sub.context.term()

    def test_combined_throttled_by_sim_time(self):
        robot = FakeRobot(0.0, 0.0, 0.0)
        before = self.bridge.summary()["pub_count"]
        robot.sim_time = 1.0
        self.bridge.tick(robot)
        after_first = self.bridge.summary()["pub_count"]
        self.bridge.tick(robot)  # sim_time 未推进, 不应再发
        self.assertEqual(self.bridge.summary()["pub_count"], after_first)
        self.assertGreater(after_first, before)

    def test_record_path_request_and_response(self):
        def save_handler(req, resp):
            resp.result = 0
            resp.info = "saved"
            resp.array.point_x.extend([0.0, 1.0, 2.0])
            resp.array.point_y.extend([0.0, 1.0, 0.5])
            resp.array.point_z.extend([0.0, 0.5, 1.0])
            resp.area_change_info.added_area.append(3)

        self.fake.record_script[tea_bridge.CMD_SAVE] = save_handler

        result = self.bridge.record_path(tea_bridge.CMD_START, type_=2)
        self.assertTrue(result["ok"])
        self.assertEqual(result["result"], 0)
        self.assertEqual(len(self.fake.record_requests), 1)
        self.assertEqual(self.fake.record_requests[-1].command, tea_bridge.CMD_START)
        self.assertEqual(self.fake.record_requests[-1].type, 2)

        result = self.bridge.record_path(tea_bridge.CMD_SAVE)
        self.assertTrue(result["ok"])
        self.assertEqual(result["point_count"], 3)
        self.assertEqual(result["points"], [[0.0, 0.0], [1.0, 1.0], [2.0, 0.5]])
        self.assertEqual(result["added"], [3])

    def test_record_path_error_code(self):
        self.fake.record_script[tea_bridge.CMD_SAVE] = lambda req, resp: (
            setattr(resp, "result", 6),
            setattr(resp, "info", "轨迹太短"),
        )
        result = self.bridge.record_path(tea_bridge.CMD_SAVE)
        self.assertFalse(result["ok"])
        self.assertEqual(result["result"], 6)
        self.assertEqual(result["info"], "轨迹太短")

    def test_clear_whole_map_result_one_is_success(self):
        self.fake.record_script[tea_bridge.CMD_CLEAR] = lambda req, resp: setattr(resp, "result", 1)
        result = self.bridge.record_path(tea_bridge.CMD_CLEAR)  # 空 ids = 整图清空
        self.assertTrue(result["ok"])
        self.assertEqual(result["result"], 1)

    def test_get_map_info_waits_for_topic(self):
        result = self.bridge.get_map_info()
        self.assertTrue(result["ok"])
        self.assertEqual(result.get("map_version"), 1)
        self.assertEqual(len(self.fake.map_requests), 1)
        self.assertEqual(self.fake.map_requests[-1].cmd, tea_bridge.MAP_CMD_LOAD)

        snap = self.bridge.map_snapshot()
        self.assertIsNone(snap.get("unchanged"))
        self.assertEqual(snap["version"], 1)
        self.assertEqual(len(snap["map"]["sub_maps"]), 1)
        self.assertEqual(snap["map"]["sub_maps"][0]["type"], 1)
        self.assertEqual(snap["map"]["sub_maps"][0]["poses"][2], [4.0, 3.0])

        # 版本相同 -> unchanged 短路
        self.assertEqual(self.bridge.map_snapshot(version=1), {"version": 1, "unchanged": True})

        # 再次调用: 等待新发布, 版本递增
        result = self.bridge.get_map_info()
        self.assertEqual(result.get("map_version"), 2)
        self.assertEqual(len(self.fake.map_requests), 2, "快速到达的话题不应触发额外重试")

    def test_get_map_info_topic_timeout_reports_error(self):
        old_wait = tea_bridge.MAP_TOPIC_WAIT
        tea_bridge.MAP_TOPIC_WAIT = 0.15
        self.fake.publish_map = False
        try:
            result = self.bridge.get_map_info()
            self.assertEqual(result.get("map_error"), "topic_timeout")
            self.assertTrue(any("map_info 话题未收到" in msg for _, msg in self.logs), self.logs)
        finally:
            tea_bridge.MAP_TOPIC_WAIT = old_wait
            self.fake.publish_map = True

    def test_get_map_info_read_fail(self):
        self.fake.map_result = 1
        result = self.bridge.get_map_info()
        self.assertFalse(result["ok"])
        self.assertEqual(result["result"], 1)

    def test_service_timeout_then_recover(self):
        # 指向不存在的服务端口 -> 超时报错
        dead_client_port = _free_port()
        self.bridge._record_client.close()
        self.bridge._record_client = tea_bridge.ZmqServiceClient(
            self.zmq, "127.0.0.1", dead_client_port, timeout=0.2
        )
        result = self.bridge.record_path(tea_bridge.CMD_START, type_=1)
        self.assertFalse(result["ok"])
        self.assertIn("error", result)

        # 假 tea 起到同端口(或直接指回原端口)后应自动恢复
        self.bridge._record_client.close()
        self.bridge._record_client = tea_bridge.ZmqServiceClient(
            self.zmq, "127.0.0.1", self.record_port, timeout=1.0
        )
        result = self.bridge.record_path(tea_bridge.CMD_START, type_=1)
        self.assertTrue(result["ok"])

    def test_session_and_trail(self):
        self.bridge.begin_session(1)
        robot = FakeRobot(0.0, 0.0, 0.0)
        for i in range(20):
            robot.sim_time = i * 0.1
            robot.pose = [i * 0.07, 0.0, 0.0]
            self.bridge.tick(robot)
        summary = self.bridge.summary()
        self.assertEqual(summary["session"]["state"], "building")
        self.assertEqual(summary["session"]["type"], 1)
        # 0.07m 步进, 0.05m 门限 -> 全部记录
        self.assertEqual(len(summary["trail"]), 20)
        self.assertGreater(summary["trail_len"], 1.0)

        self.bridge.end_session()
        summary = self.bridge.summary()
        self.assertEqual(summary["session"]["state"], "idle")
        self.assertEqual(summary["trail"], [])

    def test_disabled_bridge(self):
        disabled = tea_bridge.DisabledBridge("test")
        disabled.tick(None)
        self.assertFalse(disabled.record_path(1)["ok"])
        self.assertFalse(disabled.is_building())
        self.assertEqual(tea_bridge.create({"enabled": False}).reason, "未启用，请用 --tea-bridge 启动或切换到接入导航进程")


if __name__ == "__main__":
    unittest.main()
