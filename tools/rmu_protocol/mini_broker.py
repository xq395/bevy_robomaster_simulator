"""最小 MQTT 3.1.1 broker，供 Mock 赛事引擎和测试使用。

只实现自定义客户端协议用得到的部分：
  CONNECT/CONNACK、PUBLISH（QoS 0/1，PUBACK）、SUBSCRIBE/SUBACK（仅精确 topic 和 '#'）、
  UNSUBSCRIBE、PINGREQ、DISCONNECT。不做 QoS 2、retain、will、持久会话。

仅用于本机调试，默认绑定 127.0.0.1，没有认证，这点和官方赛事引擎一致
（手册：无 TLS、无用户名密码）。不要把它暴露到公共网络。
"""
from __future__ import annotations

import socket
import struct
import threading
from typing import Callable

CONNECT, CONNACK, PUBLISH, PUBACK = 1, 2, 3, 4
SUBSCRIBE, SUBACK, UNSUBSCRIBE, UNSUBACK = 8, 9, 10, 11
PINGREQ, PINGRESP, DISCONNECT = 12, 13, 14
MAX_PACKET = 256 * 1024


def encode_length(n: int) -> bytes:
    out = bytearray()
    while True:
        byte, n = n % 128, n // 128
        out.append(byte | (0x80 if n else 0))
        if not n:
            return bytes(out)


def mqtt_string(s: str) -> bytes:
    raw = s.encode()
    return struct.pack(">H", len(raw)) + raw


def build_publish(topic: str, payload: bytes, qos: int = 0, packet_id: int = 0) -> bytes:
    body = mqtt_string(topic)
    if qos:
        body += struct.pack(">H", packet_id)
    body += payload
    return bytes([(PUBLISH << 4) | (qos << 1)]) + encode_length(len(body)) + body


def read_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("closed")
        buf += chunk
    return bytes(buf)


def read_packet(sock: socket.socket) -> tuple[int, int, bytes]:
    first = read_exact(sock, 1)[0]
    mult, length = 1, 0
    for _ in range(4):
        byte = read_exact(sock, 1)[0]
        length += (byte & 0x7F) * mult
        if not byte & 0x80:
            break
        mult *= 128
    else:
        raise ValueError("bad remaining length")
    if length > MAX_PACKET:
        raise ValueError("packet too large")
    return first >> 4, first & 0x0F, read_exact(sock, length)


def _string(data: bytes, pos: int) -> tuple[str, int]:
    (n,) = struct.unpack_from(">H", data, pos)
    return data[pos + 2:pos + 2 + n].decode(), pos + 2 + n


class Session:
    def __init__(self, broker: "MiniBroker", sock: socket.socket, addr):
        self.broker, self.sock, self.addr = broker, sock, addr
        self.client_id = ""
        self.subs: set[str] = set()
        self.send_lock = threading.Lock()
        self.alive = True
        self.next_id = 1

    def send(self, data: bytes):
        try:
            with self.send_lock:
                self.sock.sendall(data)
        except OSError:
            self.alive = False

    def wants(self, topic: str) -> bool:
        return topic in self.subs or "#" in self.subs

    def serve(self):
        try:
            ptype, _, body = read_packet(self.sock)
            if ptype != CONNECT:
                return
            proto, pos = _string(body, 0)
            level = body[pos]
            flags = body[pos + 1]
            keepalive = struct.unpack_from(">H", body, pos + 2)[0]
            self.client_id, pos = _string(body, pos + 4)
            if proto != "MQTT" or level != 4 or flags & 0x01:
                self.send(bytes([CONNACK << 4, 2, 0, 1]))  # unacceptable protocol version
                return
            self.sock.settimeout(keepalive * 1.5 if keepalive else None)
            self.send(bytes([CONNACK << 4, 2, 0, 0]))
            self.broker._on_connect(self)
            while self.alive and self.broker.running:
                ptype, flags, body = read_packet(self.sock)
                if ptype == PUBLISH:
                    qos = (flags >> 1) & 3
                    topic, pos = _string(body, 0)
                    if qos:
                        (pid,) = struct.unpack_from(">H", body, pos)
                        pos += 2
                        self.send(bytes([PUBACK << 4, 2]) + struct.pack(">H", pid))
                    self.broker._route(self, topic, body[pos:])
                elif ptype == SUBSCRIBE:
                    (pid,) = struct.unpack_from(">H", body, 0)
                    pos, codes = 2, bytearray()
                    while pos < len(body):
                        topic, pos = _string(body, pos)
                        pos += 1  # requested QoS
                        self.subs.add(topic)
                        codes.append(1)  # granted QoS 1
                    self.send(bytes([SUBACK << 4]) + encode_length(2 + len(codes)) + struct.pack(">H", pid) + codes)
                elif ptype == UNSUBSCRIBE:
                    (pid,) = struct.unpack_from(">H", body, 0)
                    pos = 2
                    while pos < len(body):
                        topic, pos = _string(body, pos)
                        self.subs.discard(topic)
                    self.send(bytes([UNSUBACK << 4, 2]) + struct.pack(">H", pid))
                elif ptype == PINGREQ:
                    self.send(bytes([PINGRESP << 4, 0]))
                elif ptype == DISCONNECT:
                    break
                elif ptype == PUBACK:
                    pass
        except (OSError, ValueError, ConnectionError, IndexError, struct.error, UnicodeDecodeError):
            pass
        finally:
            self.alive = False
            self.broker._on_disconnect(self)
            try:
                self.sock.close()
            except OSError:
                pass


class MiniBroker:
    """`on_message(client_id, topic, payload)` 在 broker 线程里回调，回调要快。"""

    def __init__(self, host: str = "127.0.0.1", port: int = 3333,
                 on_message: Callable[[str, str, bytes], None] | None = None):
        self.host, self.port = host, port
        self.on_message = on_message
        self.sessions: list[Session] = []
        self.lock = threading.Lock()
        self.running = False
        self._server: socket.socket | None = None

    def start(self):
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind((self.host, self.port))
        self._server.listen(16)
        self.port = self._server.getsockname()[1]
        self.running = True
        threading.Thread(target=self._accept, name="mqtt-accept", daemon=True).start()
        return self

    def stop(self):
        self.running = False
        if self._server:
            self._server.close()
        with self.lock:
            for s in self.sessions:
                try:
                    s.sock.close()
                except OSError:
                    pass

    def _accept(self):
        while self.running:
            try:
                sock, addr = self._server.accept()
            except OSError:
                return
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            session = Session(self, sock, addr)
            threading.Thread(target=session.serve, name="mqtt-session", daemon=True).start()

    def _on_connect(self, session: Session):
        with self.lock:
            # 同一 clientID 重连：踢掉旧连接（MQTT 3.1.1 §3.1.4）
            for old in [s for s in self.sessions if s.client_id == session.client_id]:
                old.alive = False
                try:
                    old.sock.close()
                except OSError:
                    pass
                self.sessions.remove(old)
            self.sessions.append(session)

    def _on_disconnect(self, session: Session):
        with self.lock:
            if session in self.sessions:
                self.sessions.remove(session)

    def _route(self, sender: Session, topic: str, payload: bytes):
        if self.on_message:
            try:
                self.on_message(sender.client_id, topic, payload)
            except Exception:  # noqa: BLE001 - 回调错误不能打断 broker
                pass
        self.publish(topic, payload, exclude=sender)

    def publish(self, topic: str, payload: bytes, client_id: str | None = None, exclude: Session | None = None):
        """发给订阅了该 topic 的会话；client_id 指定时只发给该客户端。"""
        data = build_publish(topic, payload)
        with self.lock:
            targets = [s for s in self.sessions
                       if s is not exclude and s.wants(topic) and (client_id is None or s.client_id == client_id)]
        for s in targets:
            s.send(data)

    def clients(self) -> list[str]:
        with self.lock:
            return [s.client_id for s in self.sessions]
