"""RMU2026 3334 图传：分包、重组、HEVC 解码。

线上格式（通信协议 V2.0.0 第 2 章）：
  - 赛事引擎是 UDP 发送方，自定义客户端监听 3334（客户端是 UDP 服务端）。
  - 码流是标准 HEVC/H.265，1080p60，无重传。
  - 每个 UDP 包最大 1400 字节，前 8 字节大端：
        帧编号 u16（递增，到 65535 后回绕）
        帧内分片序号 u16（从 0 开始）
        当前帧总字节数 u32
    其余最多 1392 字节是码流。

手册没规定码流内部的封装方式，我们按 Annex-B（00 00 00 01 起始码）处理，
这也是 libx265/ffmpeg 裸流的默认格式。手册也没写“一帧”到底是一个访问单元
还是多个 NAL，重组层只按「帧号 + 总字节数」拼包，不关心内部结构。
"""
from __future__ import annotations

import io
import socket
import struct
import threading
import time
from collections import OrderedDict
from fractions import Fraction

import av

from . import avsafe

HEADER = struct.Struct(">HHI")
MAX_PACKET = 1400
MAX_PAYLOAD = MAX_PACKET - HEADER.size  # 1392
MAX_FRAME_BYTES = 8 * 1024 * 1024  # 防御：1080p 单帧远小于这个值
FRAME_ID_MOD = 1 << 16


def packetize(frame_id: int, data: bytes) -> list[bytes]:
    """把一帧码流切成 3334 协议包。"""
    if not data:
        return []
    if len(data) > MAX_FRAME_BYTES:
        raise ValueError("frame too large")
    frame_id %= FRAME_ID_MOD
    total = len(data)
    count = (total + MAX_PAYLOAD - 1) // MAX_PAYLOAD
    if count > 0xFFFF:
        raise ValueError("too many fragments")
    return [
        HEADER.pack(frame_id, index, total) + data[index * MAX_PAYLOAD:(index + 1) * MAX_PAYLOAD]
        for index in range(count)
    ]


def parse_header(packet: bytes) -> tuple[int, int, int, bytes]:
    if len(packet) <= HEADER.size or len(packet) > MAX_PACKET:
        raise ValueError("bad packet size")
    frame_id, index, total = HEADER.unpack_from(packet)
    if total == 0 or total > MAX_FRAME_BYTES:
        raise ValueError("bad frame size")
    payload = packet[HEADER.size:]
    count = (total + MAX_PAYLOAD - 1) // MAX_PAYLOAD
    if index >= count:
        raise ValueError("fragment index out of range")
    expected = MAX_PAYLOAD if index < count - 1 else total - MAX_PAYLOAD * (count - 1)
    if len(payload) != expected:
        raise ValueError("fragment length mismatch")
    return frame_id, index, total, payload


class _Pending:
    __slots__ = ("total", "count", "parts", "received", "first_seen")

    def __init__(self, total: int, now: float):
        self.total = total
        self.count = (total + MAX_PAYLOAD - 1) // MAX_PAYLOAD
        self.parts: list[bytes | None] = [None] * self.count
        self.received = 0
        self.first_seen = now


class Reassembler:
    """按帧号重组分片；允许乱序，丢任意分片则整帧丢弃。

    - 同一帧号在窗口内重复出现、总字节数不同 → 视为回绕后的新帧，重置。
    - 超时（默认 200ms）未收齐 → 丢弃并计入 incomplete。
    - 已交付的帧号若再次出现且落在“最近已交付”集合中 → 视为重复包丢弃。
    """

    def __init__(self, timeout_s: float = 0.2, max_pending: int = 16):
        self.timeout_s = timeout_s
        self.max_pending = max_pending
        self.pending: "OrderedDict[int, _Pending]" = OrderedDict()
        self.recent_done: "OrderedDict[int, float]" = OrderedDict()
        self.frames = 0
        self.incomplete = 0
        self.bad_packets = 0
        self.duplicates = 0

    def push(self, packet: bytes, now: float | None = None) -> bytes | None:
        now = time.monotonic() if now is None else now
        self._expire(now)
        try:
            frame_id, index, total, payload = parse_header(packet)
        except ValueError:
            self.bad_packets += 1
            return None
        done_at = self.recent_done.get(frame_id)
        if done_at is not None and now - done_at < 1.0:
            self.duplicates += 1
            return None
        entry = self.pending.get(frame_id)
        if entry is not None and entry.total != total:
            # 同帧号但长度不同：旧帧残片（回绕或发送端重启），丢旧留新
            self.incomplete += 1
            entry = None
            del self.pending[frame_id]
        if entry is None:
            entry = _Pending(total, now)
            self.pending[frame_id] = entry
            while len(self.pending) > self.max_pending:
                self.pending.popitem(last=False)
                self.incomplete += 1
        if entry.parts[index] is not None:
            self.duplicates += 1
            return None
        entry.parts[index] = payload
        entry.received += 1
        if entry.received < entry.count:
            return None
        del self.pending[frame_id]
        self.recent_done[frame_id] = now
        while len(self.recent_done) > 64:
            self.recent_done.popitem(last=False)
        self.frames += 1
        return b"".join(entry.parts)  # type: ignore[arg-type]

    def _expire(self, now: float):
        stale = [k for k, v in self.pending.items() if now - v.first_seen > self.timeout_s]
        for key in stale:
            del self.pending[key]
            self.incomplete += 1
        old = [k for k, t in self.recent_done.items() if now - t > 1.0]
        for key in old:
            del self.recent_done[key]


class HevcDecoder:
    """Annex-B HEVC 码流 → RGB 帧。解码失败只计数，不抛出。"""

    def __init__(self):
        self.codec = av.CodecContext.create("hevc", "r")
        self.errors = 0

    def decode(self, data: bytes) -> list[av.VideoFrame]:
        try:
            frames = []
            for packet in self.codec.parse(data):
                frames.extend(self.codec.decode(packet))
            return frames
        except av.error.FFmpegError:
            self.errors += 1
            return []


def hevc_encoder(width: int, height: int, fps: int, bitrate_kbps: int, gop: int | None = None):
    """Mock 发送端用的 libx265 编码器：低延迟、无 B 帧、周期 IDR。"""
    codec = av.CodecContext.create("libx265", "w")
    codec.width, codec.height = width, height
    codec.pix_fmt = "yuv420p"
    codec.time_base = Fraction(1, fps)
    codec.framerate = Fraction(fps, 1)
    codec.bit_rate = bitrate_kbps * 1000
    gop = gop or fps
    codec.options = {
        "preset": "ultrafast",
        "tune": "zerolatency",
        "x265-params": f"keyint={gop}:min-keyint={gop}:bframes=0:repeat-headers=1:log-level=error",
    }
    return codec


class VideoReceiver:
    """3334 监听线程：收包 → 重组 → 解码 → 保存最新一帧 JPEG。"""

    def __init__(self, host: str = "0.0.0.0", port: int = 3334, jpeg_width: int = 960, jpeg_quality: int = 70):
        self.host, self.port = host, port
        self.jpeg_width, self.jpeg_quality = jpeg_width, jpeg_quality
        self.lock = threading.Lock()
        self.frame: av.VideoFrame | None = None
        self.jpeg: bytes | None = None
        self.jpeg_index = -1
        self.frame_size = (0, 0)
        self.last_frame_at = 0.0
        self.reassembler = Reassembler()
        self.decoder = HevcDecoder()
        self.packets = 0
        self.bytes = 0
        self.decoded = 0
        self.error = "未启动"
        self._rate = (time.monotonic(), 0, 0)
        self.fps = 0.0
        self.kbps = 0.0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self):
        if self._thread:
            return
        self._thread = threading.Thread(target=self._run, name="rmu-video", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()

    def _run(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
            sock.bind((self.host, self.port))
            sock.settimeout(0.2)
        except OSError as exc:
            with self.lock:
                self.error = f"无法监听 UDP {self.port}：{exc}"
            return
        with self.lock:
            self.error = "等待图传"
        with sock:
            while not self._stop.is_set():
                try:
                    packet, _ = sock.recvfrom(2048)
                except socket.timeout:
                    self._update_rate()
                    continue
                except OSError:
                    # Windows 上对端不可达会让 UDP recv 抛 10054，忽略即可
                    continue
                self.packets += 1
                self.bytes += len(packet)
                frame = self.reassembler.push(packet)
                if frame is not None:
                    for image in self.decoder.decode(frame):
                        self._store(image)
                self._update_rate()

    def _store(self, frame: av.VideoFrame):
        # 只保留解码帧；JPEG 在有人取画面时才生成（1080p60 下逐帧转 JPEG 会吃掉一个核）
        with self.lock:
            self.frame = frame
            self.frame_size = (frame.width, frame.height)
            self.last_frame_at = time.monotonic()
            self.decoded += 1
            self.error = ""

    def _update_rate(self):
        now = time.monotonic()
        start, frames, nbytes = self._rate
        if now - start >= 1.0:
            with self.lock:
                self.fps = round((self.decoded - frames) / (now - start), 1)
                self.kbps = round((self.bytes - nbytes) * 8 / 1000 / (now - start), 1)
            self._rate = (now, self.decoded, self.bytes)

    def snapshot(self) -> dict:
        with self.lock:
            age = None if not self.last_frame_at else round((time.monotonic() - self.last_frame_at) * 1000)
            return dict(
                port=self.port, error=self.error, packets=self.packets, decoded=self.decoded,
                fps=self.fps, kbps=self.kbps, frame_age_ms=age,
                width=self.frame_size[0], height=self.frame_size[1],
                incomplete=self.reassembler.incomplete, bad_packets=self.reassembler.bad_packets,
                decode_errors=self.decoder.errors,
            )

    def latest_jpeg(self, max_age_s: float = 2.0) -> bytes | None:
        with self.lock:
            frame, index = self.frame, self.decoded
            if frame is None or time.monotonic() - self.last_frame_at > max_age_s:
                return None
            if self.jpeg is not None and self.jpeg_index == index:
                return self.jpeg
        image = avsafe.to_image(frame)
        if image.width > self.jpeg_width:
            image = image.resize((self.jpeg_width, round(image.height * self.jpeg_width / image.width)))
        out = io.BytesIO()
        image.save(out, format="JPEG", quality=self.jpeg_quality)
        data = out.getvalue()
        with self.lock:
            self.jpeg, self.jpeg_index = data, index
        return data
