"""ROS-free H.264 experiment. HTTP input/output surround a simulated 300-byte link.

Own test framing (not ACE wire compatible): !IHHHBB = frame, fragment, count,
payload length, keyframe, version. Packet padding and header count toward budget.
"""
import io
import os
import queue
import random
import struct
import threading
import time
import urllib.request
from fractions import Fraction

import av
from PIL import Image

HEADER = struct.Struct('!IHHHBB')
PACKET_SIZE = 300
PAYLOAD_SIZE = PACKET_SIZE - HEADER.size


def packetize(data, frame_id, keyframe):
    count = (len(data) + PAYLOAD_SIZE - 1) // PAYLOAD_SIZE
    if not 0 < count <= 65535:
        raise ValueError('invalid access unit length')
    return [
        (HEADER.pack(frame_id, i, count, len(part), int(keyframe), 1) + part).ljust(PACKET_SIZE, b'\0')
        for i in range(count)
        for part in [data[i * PAYLOAD_SIZE:(i + 1) * PAYLOAD_SIZE]]
    ]


class Receiver:
    def __init__(self):
        self.decoder = av.CodecContext.create('h264', 'r')
        self.waiting_keyframe = True
        self.current = None
        self.parts = []
        self.last_complete = -1
        self.losses = 0

    def reset(self):
        self.decoder = av.CodecContext.create('h264', 'r')
        self.waiting_keyframe = True
        self.current = None
        self.parts = []
        self.losses += 1

    def push(self, wire):
        if len(wire) != PACKET_SIZE:
            raise ValueError('packet must be 300 bytes')
        frame, index, count, size, key, version = HEADER.unpack_from(wire)
        if version != 1 or key > 1 or not 0 < size <= PAYLOAD_SIZE or not 0 <= index < count:
            raise ValueError('invalid fragment header')
        if frame <= self.last_complete:
            return []
        if self.current is None or self.current[0] != frame:
            if self.current is not None or (self.last_complete >= 0 and frame != self.last_complete + 1):
                self.reset()
            self.current = (frame, count, key)
            self.parts = []
        if self.current != (frame, count, key) or index != len(self.parts):
            self.reset()
            return []
        self.parts.append(wire[HEADER.size:HEADER.size + size])
        if len(self.parts) != count:
            return []
        data = b''.join(self.parts)
        self.current = None
        self.parts = []
        self.last_complete = frame
        if self.waiting_keyframe and not key:
            return []
        try:
            decoded = self.decoder.decode(av.Packet(data))
            if decoded:
                self.waiting_keyframe = False
            return decoded
        except av.FFmpegError:
            self.reset()
            return []


def encoder(width, height, fps, bitrate):
    codec = av.CodecContext.create('libx264', 'w')
    codec.width, codec.height = width, height
    codec.pix_fmt = 'yuv420p'
    codec.time_base = Fraction(1, fps)
    codec.framerate = Fraction(fps, 1)
    codec.bit_rate = bitrate * 1000
    codec.gop_size = fps * 2
    codec.max_b_frames = 0
    codec.options = {'preset': 'veryfast', 'tune': 'zerolatency',
                     'x264-params': f'repeat-headers=1:scenecut=0:vbv-maxrate={bitrate}:vbv-bufsize={max(8, bitrate//4)}'}
    codec.open()
    return codec


class NarrowbandVideo:
    def __init__(self, source=None):
        self.source = source or os.environ.get('DAEDALUS_VIDEO_SOURCE', 'http://127.0.0.1:5000/snapshot/raw')
        self.lock = threading.Lock()
        self.config = dict(width=320, height=240, fps=12, bitrate=80, packet_hz=40, loss=0.0)
        self.generation = 0
        self.jpeg = None
        self.decoded_at = 0
        self.stats = {}
        self.started = False

    def configure(self, data):
        limits = dict(width=(160, 640), height=(120, 480), fps=(2, 20), bitrate=(20, 100), packet_hz=(10, 50))
        with self.lock:
            config = dict(self.config)
            for key, (low, high) in limits.items():
                value = data.get(key, config[key])
                if type(value) is not int or not low <= value <= high:
                    raise ValueError(f'invalid {key}')
                config[key] = value
            loss = data.get('loss', config['loss'])
            if type(loss) not in (float, int) or not 0 <= loss <= 0.2:
                raise ValueError('invalid loss')
            config['loss'] = loss
            if config['width'] % 2 or config['height'] % 2:
                raise ValueError('dimensions must be even')
            if config['bitrate'] * 1000 >= config['packet_hz'] * PAYLOAD_SIZE * 8:
                raise ValueError('video bitrate must leave room inside link payload budget')
            self.config = config
            self.generation += 1
            self.jpeg = None
            self.stats = dict(status='restarting')
        self.start()

    def start(self):
        with self.lock:
            if self.started:
                return
            self.started = True
        threading.Thread(target=self.run, daemon=True).start()

    def snapshot(self):
        with self.lock:
            return dict(config=self.config, **self.stats,
                        decoded_age_ms=round((time.monotonic()-self.decoded_at)*1000) if self.decoded_at else None)

    def run(self):
        while True:
            with self.lock:
                config, generation = dict(self.config), self.generation
            try:
                self.pipeline(config, generation)
            except Exception as exc:
                with self.lock:
                    self.stats['status'] = str(exc)
                time.sleep(1)

    def pipeline(self, config, generation):
        codec = encoder(config['width'], config['height'], config['fps'], config['bitrate'])
        receiver = Receiver()
        pending = queue.Queue(maxsize=1)  # complete access units only; never an unbounded latency queue
        stop = threading.Event()
        started = time.monotonic()
        counters = dict(encoded_bytes=0, wire_bytes=0, packets=0, dropped_packets=0,
                        skipped_inputs=0, encoded_frames=0, decoded_frames=0,
                        fetched_frames=0, changed_snapshots=0)

        def capture():
            previous = None
            frame_id = 0
            while not stop.is_set():
                tick = time.monotonic()
                if pending.full():
                    counters['skipped_inputs'] += 1
                    stop.wait(1/config['fps'])
                    continue
                try:
                    with urllib.request.urlopen(self.source, timeout=1) as response:
                        raw = response.read()
                    counters['fetched_frames'] += 1
                    if raw != previous:
                        counters['changed_snapshots'] += 1
                    previous = raw
                    # Explicit letterbox preserves field of view; no implicit center ROI.
                    image = Image.open(io.BytesIO(raw)).convert('RGB')
                    image.thumbnail((config['width'], config['height']))
                    canvas = Image.new('RGB', (config['width'], config['height']))
                    canvas.paste(image, ((canvas.width-image.width)//2, (canvas.height-image.height)//2))
                    frame = av.VideoFrame.from_image(canvas)
                    frame.pts = frame_id
                    for packet in codec.encode(frame):
                        data = bytes(packet)
                        counters['encoded_bytes'] += len(data)
                        counters['encoded_frames'] += 1
                        pending.put((packetize(data, frame_id, packet.is_keyframe), tick), timeout=1)
                        frame_id += 1
                except (OSError, ValueError, av.FFmpegError, queue.Full) as exc:
                    with self.lock:
                        self.stats['status'] = f'source/encode: {exc}'
                stop.wait(max(0, 1/config['fps'] - (time.monotonic()-tick)))

        worker = threading.Thread(target=capture, daemon=True)
        worker.start()
        next_send = time.monotonic()
        recovery_start = None
        try:
            while True:
                with self.lock:
                    if self.generation != generation:
                        return
                try:
                    packets, captured = pending.get(timeout=0.2)
                except queue.Empty:
                    continue
                for wire in packets:
                    # A full 300B slot every 1/Hz; no catch-up bursts.
                    stop.wait(max(0, next_send-time.monotonic()))
                    next_send = time.monotonic() + 1/config['packet_hz']
                    counters['wire_bytes'] += len(wire)
                    counters['packets'] += 1
                    if random.random() < config['loss']:
                        counters['dropped_packets'] += 1
                        if recovery_start is None:
                            recovery_start = time.monotonic()
                        frames = []
                    else:
                        frames = receiver.push(wire)
                    for frame in frames:
                        out = io.BytesIO()
                        frame.to_image().save(out, format='JPEG', quality=90)
                        now = time.monotonic()
                        counters['decoded_frames'] += 1
                        with self.lock:
                            if self.generation != generation:
                                return
                            self.jpeg, self.decoded_at = out.getvalue(), now
                            self.stats['pipeline_ms'] = round((now-captured)*1000)
                            if recovery_start is not None:
                                self.stats['last_recovery_ms'] = round((now-recovery_start)*1000)
                        recovery_start = None
                    elapsed = max(0.001, time.monotonic()-started)
                    with self.lock:
                        self.stats.update(counters)
                        self.stats.update(status='waiting_idr' if receiver.waiting_keyframe else 'streaming',
                                          encoded_kbps=round(counters['encoded_bytes']*8/elapsed/1000, 2),
                                          wire_kbps=round(counters['wire_bytes']*8/elapsed/1000, 2),
                                          decoded_fps=round(counters['decoded_frames']/elapsed, 2),
                                          decoder_resets=receiver.losses)
        finally:
            stop.set()
            worker.join(timeout=3)