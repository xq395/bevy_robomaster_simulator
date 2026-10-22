"""0x0310 自定义数据流上的低码率图传（机器人 → 图传链路 → 自定义客户端）。

链路约束（通信协议 V2.0.0 §1.4.2、§2.2.20）：
  - 每块最多 300 字节（CustomByteBlock.data，≤2.4 kbit），频率上限 50 Hz，约 15 KB/s；
  - 无重传。客户端 → 机器人方向是 0x0311（CustomControl，≤30 字节，≤75 Hz）。
这条路和 RJ45 上的官方图传（UDP 3334，HEVC 1080p）是两条独立链路，这里只管前者。

编码：H.264 + 帧内刷新（intra-refresh）。不用周期 IDR，关键帧不会突发超预算；
丢块后解码器继续解后续帧，坏区域在一个刷新周期（默认 1 s）内被帧内条带刷掉，
不需要回传请求，也就不占 0x0311 上行。

块格式（自定义，非 ACE/官方格式，版本 1）：
    u8  magic        0x56 ('V')
    u16 seq          块序号，逐块 +1，用来发现丢块
    u16 first        本块内第一个记录的起始偏移；0xFFFF = 本块全是上一个记录的续传
    ... 295 字节记录流（不足 295 字节的块照发，不填充）
记录（可跨块）：
    u16 length       H.264 访问单元字节数
    u8  flags        bit0 = 关键帧/恢复点
    u16 frame_id
    ... Annex-B 访问单元
发送端按 50 Hz 节拍每 20 ms 发一块，队列积压超过上限就跳帧，延迟不随时间累积。
"""
from __future__ import annotations

import struct
import threading
import time
from collections import deque
from fractions import Fraction

import av

BLOCK_SIZE = 300
BLOCK_HZ = 50
MAGIC = 0x56
BLOCK_HEADER = struct.Struct(">BHH")
RECORD_HEADER = struct.Struct(">HBH")
PAYLOAD = BLOCK_SIZE - BLOCK_HEADER.size  # 295
NO_RECORD = 0xFFFF
SEQ_MOD = 1 << 16

FLAG_KEY = 0x01


def encoder_options(fps: int, kbps: int, refresh_s: float = 1.0) -> dict:
    keyint = max(2, round(fps * refresh_s))
    buffer = max(1, kbps * 2 // fps)  # 两帧的 VBV 缓冲：单帧不超过约 2 倍平均大小
    return {
        "preset": "veryfast",
        "tune": "zerolatency",
        "x264-params": (f"intra-refresh=1:keyint={keyint}:min-keyint={keyint}:bframes=0:"
                        f"vbv-maxrate={kbps}:vbv-bufsize={buffer}:nal-hrd=none:repeat-headers=1:"
                        "rc-lookahead=0:sync-lookahead=0:log-level=error"),
    }


class Encoder:
    """RGB/YUV 帧 → H.264 访问单元。"""

    def __init__(self, width: int = 384, height: int = 288, fps: int = 15, kbps: int = 96):
        self.width, self.height, self.fps, self.kbps = width, height, fps, kbps
        self.codec = av.CodecContext.create("libx264", "w")
        self.codec.width, self.codec.height = width, height
        self.codec.pix_fmt = "yuv420p"
        self.codec.time_base = Fraction(1, fps)
        self.codec.framerate = Fraction(fps, 1)
        self.codec.bit_rate = kbps * 1000
        self.codec.options = encoder_options(fps, kbps)
        self.pts = 0

    def encode(self, frame: av.VideoFrame) -> list[tuple[bytes, bool]]:
        if frame.width != self.width or frame.height != self.height or frame.format.name != "yuv420p":
            frame = frame.reformat(width=self.width, height=self.height, format="yuv420p")
        frame.pts = self.pts
        self.pts += 1
        return [(bytes(p), bool(p.is_keyframe)) for p in self.codec.encode(frame)]


class Packer:
    """访问单元 → 300 字节块（记录流跨块拼接，块不填充）。"""

    def __init__(self):
        self.buffer = bytearray()
        self.starts: deque[int] = deque()   # buffer 内各记录的起始偏移
        self.seq = 0
        self.frame_id = 0

    def add(self, unit: bytes, key: bool) -> int:
        if len(unit) > 0xFFFF:
            raise ValueError("access unit too large")
        self.starts.append(len(self.buffer))
        self.buffer += RECORD_HEADER.pack(len(unit), FLAG_KEY if key else 0, self.frame_id) + unit
        self.frame_id = (self.frame_id + 1) % SEQ_MOD
        return self.frame_id

    def pending(self) -> int:
        return len(self.buffer)

    def clear(self):
        self.buffer.clear()
        self.starts.clear()

    def next_block(self) -> bytes | None:
        if not self.buffer:
            return None
        chunk = bytes(self.buffer[:PAYLOAD])
        first = self.starts[0] if self.starts and self.starts[0] < len(chunk) else NO_RECORD
        del self.buffer[:len(chunk)]
        while self.starts and self.starts[0] < len(chunk):
            self.starts.popleft()
        self.starts = deque(s - len(chunk) for s in self.starts)
        block = BLOCK_HEADER.pack(MAGIC, self.seq, first) + chunk
        self.seq = (self.seq + 1) % SEQ_MOD
        return block


class Unpacker:
    """300 字节块 → (frame_id, key, 访问单元)。丢块时丢掉断开的记录，从下一个记录起点重新同步。"""

    def __init__(self):
        self.buffer = bytearray()
        self.synced = False
        self.expected: int | None = None
        self.lost_blocks = 0
        self.bad_blocks = 0
        self.blocks = 0
        self.restarts = 0
        self.gap = False   # 自上次取走以来是否发生过丢块

    def push(self, block: bytes) -> list[tuple[int, bool, bytes]]:
        if len(block) < BLOCK_HEADER.size + 1 or len(block) > BLOCK_SIZE:
            self.bad_blocks += 1
            return []
        magic, seq, first = BLOCK_HEADER.unpack_from(block)
        payload = block[BLOCK_HEADER.size:]
        if magic != MAGIC or (first != NO_RECORD and first >= len(payload)):
            self.bad_blocks += 1
            return []
        self.blocks += 1
        if self.expected is not None and seq != self.expected:
            missed = (seq - self.expected) % SEQ_MOD
            if missed > SEQ_MOD // 2 and seq != 0:   # 迟到/重复的旧块
                return []
            if seq == 0:
                # 发送端重启：序号从 0 重来，解码器也要换新的（参数集可能变了）
                self.restarts += 1
            else:
                self.lost_blocks += missed
            self.gap = True
            self.synced = False
        self.expected = (seq + 1) % SEQ_MOD
        if not self.synced:
            self.buffer.clear()
            if first == NO_RECORD:
                return []
            payload = payload[first:]
            self.synced = True
        self.buffer += payload
        out = []
        while len(self.buffer) >= RECORD_HEADER.size:
            length, flags, frame_id = RECORD_HEADER.unpack_from(self.buffer)
            end = RECORD_HEADER.size + length
            if len(self.buffer) < end:
                break
            out.append((frame_id, bool(flags & FLAG_KEY), bytes(self.buffer[RECORD_HEADER.size:end])))
            del self.buffer[:end]
        return out


class Sender:
    """机器人侧：源帧 → 编码 → 打包 → 按 50 Hz 节拍交给 publish(block)。

    在仿真/Mock 里 publish 直接发 MQTT CustomByteBlock；真机上应改为经串口发 0x0310。
    """

    def __init__(self, publish, width=384, height=288, fps=15, kbps=96, max_queue_s=0.3):
        self.publish = publish
        self.encoder = Encoder(width, height, fps, kbps)
        self.packer = Packer()
        self.fps = fps
        self.max_queue = int(max_queue_s * BLOCK_HZ * PAYLOAD)
        self.lock = threading.Lock()
        self.sent_blocks = 0
        self.sent_bytes = 0
        self.frames = 0
        self.skipped = 0
        self._stop = threading.Event()

    def push_frame(self, frame: av.VideoFrame) -> bool:
        """喂一帧；链路积压时跳过（返回 False），保持端到端延迟有界。"""
        with self.lock:
            if self.packer.pending() > self.max_queue:
                self.skipped += 1
                return False
            for unit, key in self.encoder.encode(frame):
                self.packer.add(unit, key)
            self.frames += 1
            return True

    def run_pacer(self):
        period = 1.0 / BLOCK_HZ
        next_at = time.monotonic()
        while not self._stop.is_set():
            with self.lock:
                block = self.packer.next_block()
            if block is not None:
                try:
                    self.publish(block)
                    self.sent_blocks += 1
                    self.sent_bytes += len(block)
                except Exception:  # noqa: BLE001 - 发送失败等同丢块，接收端会重新同步
                    pass
            next_at += period
            delay = next_at - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_at = time.monotonic()

    def start(self):
        threading.Thread(target=self.run_pacer, name="custom-video-pacer", daemon=True).start()
        return self

    def stop(self):
        self._stop.set()

    def summary(self) -> dict:
        return dict(frames=self.frames, skipped=self.skipped, sent_blocks=self.sent_blocks,
                    queue=self.packer.pending())


class Receiver:
    """客户端侧：CustomByteBlock → 解码最新帧。丢块只丢断开的那一帧，之后照常解码，
    画面上的坏块靠发送端的帧内刷新自愈；recovering 标出这段时间供界面提示。"""

    def __init__(self, refresh_s: float = 1.0, jpeg_width: int = 960, jpeg_quality: int = 75):
        self.unpacker = Unpacker()
        self.codec = av.CodecContext.create("h264", "r")
        self.refresh_s = refresh_s
        self.jpeg_width, self.jpeg_quality = jpeg_width, jpeg_quality
        self.lock = threading.Lock()
        self.frame: av.VideoFrame | None = None
        self.decoded = 0
        self.decode_errors = 0
        self.last_frame_at = 0.0
        self.last_block_at = 0.0
        self.last_gap_at = 0.0
        self.bytes = 0
        self._rate = (time.monotonic(), 0, 0, 0)
        self.fps = self.kbps = self.block_hz = 0.0
        self._jpeg = (None, -1)

    def push(self, block: bytes):
        now = time.monotonic()
        with self.lock:
            self.bytes += len(block)
            self.last_block_at = now
            restarts = self.unpacker.restarts
            units = self.unpacker.push(block)
            if self.unpacker.restarts != restarts:
                self.codec = av.CodecContext.create("h264", "r")
            if self.unpacker.gap:
                self.unpacker.gap = False
                self.last_gap_at = now
            for _frame_id, _key, unit in units:
                try:
                    frames = self.codec.decode(av.Packet(unit))
                except av.error.FFmpegError:
                    self.decode_errors += 1
                    self.last_gap_at = now
                    continue
                for frame in frames:
                    self.frame = frame
                    self.decoded += 1
                    self.last_frame_at = now
            self._update_rate(now)

    def _update_rate(self, now: float):
        start, frames, nbytes, blocks = self._rate
        if now - start >= 1.0:
            span = now - start
            self.fps = round((self.decoded - frames) / span, 1)
            self.kbps = round((self.bytes - nbytes) * 8 / 1000 / span, 1)
            self.block_hz = round((self.unpacker.blocks - blocks) / span, 1)
            self._rate = (now, self.decoded, self.bytes, self.unpacker.blocks)

    def snapshot(self) -> dict:
        with self.lock:
            now = time.monotonic()
            stale = not self.last_block_at or now - self.last_block_at > 2.0
            frame = self.frame
            return dict(
                blocks=self.unpacker.blocks, lost_blocks=self.unpacker.lost_blocks,
                bad_blocks=self.unpacker.bad_blocks, decoded=self.decoded, decode_errors=self.decode_errors,
                fps=0.0 if stale else self.fps, kbps=0.0 if stale else self.kbps,
                block_hz=0.0 if stale else self.block_hz,
                recovering=bool(self.last_gap_at) and now - self.last_gap_at < self.refresh_s,
                width=frame.width if frame else 0, height=frame.height if frame else 0,
                frame_age_ms=round((now - self.last_frame_at) * 1000) if self.last_frame_at else None)

    def latest_jpeg(self, max_age_s: float = 2.0) -> bytes | None:
        import io
        with self.lock:
            frame, index = self.frame, self.decoded
            if frame is None or time.monotonic() - self.last_frame_at > max_age_s:
                return None
            if self._jpeg[1] == index:
                return self._jpeg[0]
        image = frame.to_image()
        if image.width < self.jpeg_width:
            # 低分辨率画面在浏览器里会被拉伸，服务端先做一次平滑放大更好看
            image = image.resize((self.jpeg_width, round(image.height * self.jpeg_width / image.width)))
        out = io.BytesIO()
        image.save(out, format="JPEG", quality=self.jpeg_quality)
        data = out.getvalue()
        with self.lock:
            self._jpeg = (data, index)
        return data
