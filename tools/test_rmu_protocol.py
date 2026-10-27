"""RMU 协议层测试：3334 分包/重组/解码、mini broker、客户端链路、Mock 引擎。

运行：python -m unittest tools/test_rmu_protocol.py -v   （在仓库根目录）
或   cd tools && python -m unittest test_rmu_protocol -v
"""
import socket
import struct
import threading
import time
import unittest

import av
from PIL import Image, ImageDraw

from rmu_protocol import custom_video as cv
from rmu_protocol import rm_custom_client_pb2 as pb
from rmu_protocol.client_link import RmuLink, RateLimited, keys_to_mask, INPUT_TTL_S
from rmu_protocol.mini_broker import MiniBroker
from rmu_protocol.mock_engine import MockEngine
from rmu_protocol.video import (HEADER, MAX_PACKET, MAX_PAYLOAD, HevcDecoder, Reassembler, VideoReceiver,
                                hevc_encoder, packetize, parse_header)


def free_port(kind=socket.SOCK_STREAM):
    s = socket.socket(socket.AF_INET, kind)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def wait_until(pred, timeout=5.0, step=0.02):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(step)
    return False


def encoded_frames(n=30, w=320, h=240):
    codec = hevc_encoder(w, h, 30, 800, gop=10)
    out = []
    for i in range(n):
        img = Image.new("RGB", (w, h), "#123")
        ImageDraw.Draw(img).rectangle((i * 5 % 200, 60, i * 5 % 200 + 60, 120), fill="red")
        frame = av.VideoFrame.from_image(img)
        frame.pts = i
        out.extend(bytes(p) for p in codec.encode(frame))
    out.extend(bytes(p) for p in codec.encode(None))
    return out


class PacketFormatTest(unittest.TestCase):
    def test_header_is_big_endian_and_sizes_match_manual(self):
        data = bytes(range(256)) * 20  # 5120 字节 → 4 片
        packets = packetize(0x1234, data)
        self.assertEqual(len(packets), 4)
        self.assertTrue(all(len(p) <= MAX_PACKET for p in packets))
        self.assertEqual(len(packets[0]), 1400)
        self.assertEqual(MAX_PAYLOAD, 1392)
        # 帧号 u16、分片 u16、总长 u32，大端
        self.assertEqual(packets[2][:8], bytes([0x12, 0x34, 0x00, 0x02, 0x00, 0x00, 0x14, 0x00]))
        self.assertEqual(b"".join(p[8:] for p in packets), data)

    def test_frame_id_wraps_at_16_bits(self):
        self.assertEqual(struct.unpack(">H", packetize(65536 + 7, b"x")[0][:2])[0], 7)

    def test_malformed_packets_rejected(self):
        for bad in (b"", b"\x00" * 8, b"\x00" * 1401,
                    HEADER.pack(1, 5, 100) + b"x" * 100,        # 分片号越界
                    HEADER.pack(1, 0, 100) + b"x" * 50,         # 长度不符
                    HEADER.pack(1, 0, 0) + b"x"):               # 总长 0
            with self.assertRaises(ValueError):
                parse_header(bad)


class ReassemblyTest(unittest.TestCase):
    def test_out_of_order_and_duplicate(self):
        data = bytes(5000)
        packets = packetize(9, data)
        r = Reassembler()
        self.assertIsNone(r.push(packets[2], now=0))
        self.assertIsNone(r.push(packets[0], now=0))
        self.assertIsNone(r.push(packets[0], now=0))  # 重复分片
        self.assertIsNone(r.push(packets[3], now=0))
        self.assertEqual(r.push(packets[1], now=0), data)
        self.assertIsNone(r.push(packets[1], now=0.01))  # 已交付帧的迟到重复包
        self.assertEqual(r.duplicates, 2)

    def test_missing_fragment_times_out(self):
        r = Reassembler(timeout_s=0.1)
        p = packetize(1, bytes(3000))
        r.push(p[0], now=0)
        r.push(p[1], now=0)
        r.push(packetize(2, b"ok")[0], now=0.5)  # 触发过期
        self.assertEqual(r.incomplete, 1)
        self.assertEqual(r.pending.keys() - {2}, set())

    def test_wrapped_frame_id_with_new_size_replaces_stale_partial(self):
        r = Reassembler()
        r.push(packetize(5, bytes(3000))[0], now=0)
        self.assertEqual(r.push(packetize(5, b"new")[0], now=0.05), b"new")
        self.assertEqual(r.incomplete, 1)


class HevcTest(unittest.TestCase):
    def test_roundtrip_through_3334_packets(self):
        frames = encoded_frames(30)
        r, d = Reassembler(), HevcDecoder()
        decoded = 0
        for fid, unit in enumerate(frames):
            for p in packetize(fid, unit):
                out = r.push(p)
                if out:
                    decoded += len(d.decode(out))
        decoded += len(d.decode(b""))  # flush 解码器内部缓存（parser 需要看到下一帧起始码）
        self.assertGreaterEqual(decoded, 29)
        self.assertEqual(d.errors, 0)

    def test_recovers_after_lost_fragment_at_next_idr(self):
        frames = encoded_frames(30)
        r, d = Reassembler(), HevcDecoder()
        after = 0
        for fid, unit in enumerate(frames):
            for j, p in enumerate(packetize(fid, unit)):
                if fid == 3 and j == 0:
                    continue  # 丢一片 → 整帧丢弃
                out = r.push(p)
                if out and fid >= 10:
                    after += len(d.decode(out))
                elif out:
                    d.decode(out)
        self.assertGreater(after, 0, "应在下一个 IDR（gop=10）后恢复")


class BrokerTest(unittest.TestCase):
    def test_connect_subscribe_publish_qos1_and_clientid(self):
        import paho.mqtt.client as mqtt
        seen = []
        broker = MiniBroker("127.0.0.1", 0, on_message=lambda cid, t, p: seen.append((cid, t, p))).start()
        got = []
        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="103", protocol=mqtt.MQTTv311)
        c.on_connect = lambda cl, u, f, rc, pr=None: cl.subscribe("GameStatus", 1)
        c.on_message = lambda cl, u, m: got.append(m.payload)
        c.connect("127.0.0.1", broker.port)
        c.loop_start()
        try:
            self.assertTrue(wait_until(lambda: broker.clients() == ["103"]))
            self.assertTrue(wait_until(lambda: any(s.subs for s in broker.sessions)))
            broker.publish("GameStatus", b"\x28\x04")
            broker.publish("Other", b"nope")
            self.assertTrue(wait_until(lambda: got))
            self.assertEqual(got, [b"\x28\x04"])
            c.publish("CommonCommand", b"\x08\x01", qos=1).wait_for_publish(2)
            self.assertTrue(wait_until(lambda: seen))
            self.assertEqual(seen[0], ("103", "CommonCommand", b"\x08\x01"))
        finally:
            c.loop_stop()
            c.disconnect()
            broker.stop()

    def test_rejects_non_311(self):
        broker = MiniBroker("127.0.0.1", 0).start()
        try:
            s = socket.create_connection(("127.0.0.1", broker.port), timeout=2)
            body = b"\x00\x04MQTT\x05\x02\x00\x0a" + b"\x00\x01x"
            s.sendall(bytes([0x10, len(body)]) + body)
            self.assertEqual(s.recv(4), bytes([0x20, 2, 0, 1]))
            s.close()
        finally:
            broker.stop()


def moving_frames(n, w=384, h=288):
    for i in range(n):
        img = Image.new("RGB", (w, h), "#1b2a3a")
        draw = ImageDraw.Draw(img)
        draw.rectangle((i * 7 % (w - 80), 90, i * 7 % (w - 80) + 80, 170), fill="#d04030")
        draw.text((10, 10), f"frame {i}", fill="white")
        yield av.VideoFrame.from_image(img)


class CustomVideoTest(unittest.TestCase):
    """0x0310：300 字节 × 50 Hz 的块格式、预算和丢块恢复。"""

    def stream(self, frames=45, fps=15, loss=None):
        blocks = []
        sender = cv.Sender(blocks.append, fps=fps)
        receiver = cv.Receiver()
        delivered = 0
        for index, frame in enumerate(moving_frames(frames)):
            sender.push_frame(frame)
            for _ in range(cv.BLOCK_HZ // fps):      # 50 Hz 节拍下每帧有 3 个发块时隙
                block = sender.packer.next_block()
                if block is None:
                    continue
                blocks.append(block)
                if loss and loss(len(blocks)):
                    continue
                receiver.push(block)
                delivered += 1
        return sender, receiver, blocks

    def test_blocks_fit_300_bytes_and_50hz_budget(self):
        sender, receiver, blocks = self.stream()
        self.assertTrue(all(len(b) <= cv.BLOCK_SIZE for b in blocks))
        # 3 s 的画面最多 150 个块；积压不能越来越多（否则延迟无界）
        self.assertLessEqual(len(blocks), 3 * cv.BLOCK_HZ)
        self.assertLess(sender.packer.pending(), cv.PAYLOAD * 5)
        self.assertGreaterEqual(receiver.decoded, 40)
        self.assertEqual(receiver.decode_errors, 0)

    def test_header_layout(self):
        sender, _, blocks = self.stream(frames=2)
        magic, seq0, first0 = cv.BLOCK_HEADER.unpack_from(blocks[0])
        self.assertEqual((magic, seq0, first0), (cv.MAGIC, 0, 0))
        self.assertEqual(cv.BLOCK_HEADER.unpack_from(blocks[1])[1], 1)

    def test_lost_block_resyncs_and_heals(self):
        lost = {10, 11}
        _, receiver, _ = self.stream(frames=45, loss=lambda n: n in lost)
        snap = receiver.snapshot()
        self.assertEqual(snap["lost_blocks"], 2)
        # 丢块那一帧丢掉，后面的帧照常解（帧内刷新会把坏区域补回来）
        self.assertGreaterEqual(receiver.decoded, 35)

    def test_garbage_rejected(self):
        receiver = cv.Receiver()
        for bad in (b"", b"V", b"X" + bytes(299), bytes([cv.MAGIC]) + bytes(300)):
            receiver.push(bad)
        self.assertEqual(receiver.decoded, 0)
        self.assertEqual(receiver.unpacker.bad_blocks, 4)

    def test_sender_restart_resets_decoder(self):
        _, _, first = self.stream(frames=20)
        _, _, second = self.stream(frames=20)
        receiver = cv.Receiver()
        for block in first[:len(first) // 2] + second:
            receiver.push(block)
        self.assertEqual(receiver.unpacker.restarts, 1)
        self.assertGreaterEqual(receiver.decoded, 25)

    def test_sender_restart_seen_mid_stream(self):
        """客户端在发送端重启后才重连：看不到 seq 0，序号直接往回跳，也要能恢复。"""
        _, _, first = self.stream(frames=40)
        _, _, second = self.stream(frames=40)
        receiver = cv.Receiver()
        for block in first + second[len(second) // 3:]:
            receiver.push(block)
        self.assertEqual(receiver.unpacker.restarts, 1)
        self.assertGreater(receiver.decoded, 35 + 10, "重启后应继续出帧")


class ClientLinkTest(unittest.TestCase):
    def setUp(self):
        self.mqtt_port = free_port()
        self.video_port = free_port(socket.SOCK_DGRAM)
        self.engine = MockEngine(robot_id=3, mqtt_port=self.mqtt_port, video_to=("127.0.0.1", self.video_port),
                                 width=320, height=240, fps=15, kbps=600, custom_video_size=(192, 144),
                                 custom_video_fps=10, custom_video_kbps=64).start()
        self.link = RmuLink("127.0.0.1", self.mqtt_port, robot_id=3, video_port=self.video_port,
                            video_host="127.0.0.1").start()
        self.assertTrue(wait_until(lambda: self.link.snapshot()["connected"]), "应连上 Mock")

    def tearDown(self):
        self.link.stop()
        self.engine.stop()

    def test_client_id_is_robot_id(self):
        self.assertTrue(wait_until(lambda: "3" in self.engine.broker.clients()))

    def test_rejects_invalid_robot_id(self):
        for bad in (0, 12, 100, 0x0103):
            with self.assertRaises(ValueError):
                RmuLink(robot_id=bad)

    def test_downlink_topics_decoded(self):
        self.assertTrue(wait_until(lambda: {"GameStatus", "RobotDynamicStatus", "GlobalUnitStatus",
                                            "RobotStaticStatus"} <= set(self.link.snapshot()["messages"]), 4))
        snap = self.link.snapshot()["messages"]
        self.assertEqual(len(snap["GlobalUnitStatus"]["data"]["robot_health"]), 10)
        self.assertEqual(snap["GameStatus"]["data"]["current_stage"], 4)

    def test_keyboard_not_sent_until_takeover_then_75hz(self):
        time.sleep(0.3)
        self.assertEqual(self.engine.model.summary()["keyboard_msgs"], 0, "未接管不得发键鼠")
        self.link.set_driving(True)
        stop = threading.Event()

        def heartbeat():
            while not stop.is_set():
                self.link.update_input(dict(keys=["KeyW", "ShiftLeft"], mouse_x=2, buttons=dict(left=False)))
                time.sleep(0.05)
        t = threading.Thread(target=heartbeat)
        t.start()
        time.sleep(1.0)
        stop.set()
        t.join()
        count = self.engine.model.summary()["keyboard_msgs"]
        print(f"\n    KeyboardMouseControl 实测 {count} 帧/秒（目标 75）", end=" ")
        self.assertTrue(60 <= count <= 90, f"约 75Hz，实际 {count}/s")
        self.assertTrue(wait_until(lambda: self.engine.model.summary()["keyboard_value"] in (0b10001, 0)))

    def test_stale_input_becomes_zero_frame(self):
        self.link.set_driving(True)
        self.link.update_input(dict(keys=["KeyW"], buttons=dict(left=True)))
        self.assertTrue(wait_until(lambda: self.engine.model.summary()["keyboard_value"] == 1))
        time.sleep(INPUT_TTL_S + 0.15)  # 不再刷新
        self.assertTrue(wait_until(lambda: self.engine.model.summary()["keyboard_value"] == 0, 1))
        self.assertFalse(self.engine.model.last_keyboard.left_button_down)

    def test_release_sends_zero_and_stops(self):
        self.link.set_driving(True)
        self.link.update_input(dict(keys=["KeyD"]))
        self.assertTrue(wait_until(lambda: self.engine.model.summary()["keyboard_value"] == 8))
        self.link.set_driving(False)
        self.assertTrue(wait_until(lambda: self.engine.model.summary()["keyboard_value"] == 0, 1))
        n = self.engine.model.summary()["keyboard_msgs"]
        time.sleep(0.3)
        self.assertLessEqual(self.engine.model.summary()["keyboard_msgs"] - n, 1, "撤销后不应继续发送")

    def test_update_without_takeover_rejected(self):
        with self.assertRaises(ValueError):
            self.link.update_input(dict(keys=["KeyW"]))

    def test_command_validation_and_rate_limit(self):
        self.assertTrue(wait_until(lambda: "GlobalLogisticsStatus" in self.link.snapshot()["messages"], 3))
        eco = self.engine.model.summary()["economy"]
        self.assertEqual(self.link.send_command("CommonCommand", dict(cmd_type=1, param=10)), "已发送")
        with self.assertRaises(RateLimited):
            self.link.send_command("CommonCommand", dict(cmd_type=1, param=10))
        self.assertTrue(wait_until(lambda: self.engine.model.summary()["economy"] <= eco - 10 + 10))
        for topic, fields in (("CommonCommand", dict(cmd_type=1, param=15)),       # 非 10 的倍数
                              ("CommonCommand", dict(bogus=1)),                   # 未知字段
                              ("CommonCommand", dict(cmd_type=True)),             # bool 冒充整数
                              ("CommonCommand", dict(cmd_type=-1)),               # uint32 越界
                              ("GameStatus", {}),                                 # 下行 topic
                              ("CustomControl", dict(data=list(range(31)))),      # >30 字节
                              ("MapClickCmd", dict(robot_id=[1, 2]))):            # 非 7 字节
            time.sleep(0.11)
            with self.assertRaises(ValueError, msg=f"{topic} {fields}"):
                self.link.send_command(topic, fields)

    def test_map_click_interval_by_role(self):
        fields = dict(is_send_all=1, robot_id=[0] * 7, mode=1, type=1, map_x=10.0, map_y=5.0)
        self.link.send_command("MapClickCmd", fields, role="gimbal")
        with self.assertRaises(RateLimited):
            self.link.send_command("MapClickCmd", fields, role="gimbal")
        time.sleep(0.55)
        self.link.send_command("MapClickCmd", fields, role="gimbal")
        time.sleep(0.55)
        with self.assertRaises(RateLimited):
            self.link.send_command("MapClickCmd", fields, role="semi_auto")
        self.assertTrue(wait_until(lambda: any(e["topic"] == "MapClickInfo" for e in self.link.snapshot()["events"])))

    def test_sentry_result_roundtrip(self):
        self.link.send_command("SentryCtrlCommand", dict(command_id=8))
        self.assertTrue(wait_until(lambda: self.link.snapshot()["messages"].get("SentryCtrlResult", {}).get("data", {}).get("result_code") == 0))

    def test_video_3334_end_to_end(self):
        self.assertTrue(wait_until(lambda: self.link.video.latest_jpeg() is not None, 6), self.link.video.snapshot())
        v = self.link.snapshot()["video"]
        self.assertEqual((v["width"], v["height"]), (320, 240))
        self.assertEqual(v["decode_errors"], 0)

    def test_custom_video_0x0310_end_to_end(self):
        """Mock 经 MQTT CustomByteBlock 发 0x0310 视频块，客户端解出画面。"""
        self.assertTrue(wait_until(lambda: self.link.custom_video.latest_jpeg() is not None, 6),
                        self.link.custom_video.snapshot())
        c = self.link.snapshot()["custom_video"]
        self.assertEqual((c["width"], c["height"]), (192, 144))
        self.assertEqual(c["bad_blocks"], 0)


if __name__ == "__main__":
    unittest.main()
