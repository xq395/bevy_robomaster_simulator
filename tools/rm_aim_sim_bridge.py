"""rm_aim_sim <-> Daedalus 操作终端 桥接（Windows / Linux 通用）。

Daedalus 的操作终端（tools/operator_web.py）只认两个本机接口：

  * UDP 127.0.0.1:5001   操作权租约（session / seq / ttl）+ 状态回报
  * HTTP 127.0.0.1:5000  当前画面 /snapshot/raw

本进程把 rm_aim_sim（Godot 自瞄仿真）的共享内存邮箱
（state.json / command.json / request.json / frame-*.raw）映射到上面这两个接口，
于是浏览器端 HUD、接管、自瞄授权、全手动全部不用改一行就能驱动 Godot 仿真。

语义对齐 daedalus 的 src/operator.rs：
  * session 不符 / seq 不递增 / 超过 ttl / 授权了自瞄却没开自瞄 一律拒绝；
  * 全手动模式不允许同时开自瞄、小陀螺；
  * 250ms 没有新指令就撤销接管（rm_aim_sim 自己也有 250ms 看门狗）。

rm_aim_sim 没有底盘，所以 movement 轴与小陀螺是空操作；云台走绝对角指令。
自瞄由本进程用仿真真值（truth）+ 弹道解算 + 目标角速度预测实现，
作用与 Daedalus 内置的真值自瞄一致——它是"接得通"的验收手段，不是视觉算法。
"""

import argparse
import io
import json
import math
import os
import socket
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

try:
    from PIL import Image
except ImportError:  # 画面能跑，只是没有视频
    Image = None


# --------------------------------------------------------------------------- IPC

class Channel:
    """rm_aim_sim 本机邮箱客户端（Linux /dev/shm，其它平台 OS.get_temp_dir）。"""

    def __init__(self, channel, root=None, timeout=8.0):
        self.directory = Path(root or default_root()) / ("rm_aim_sim_" + channel)
        deadline = time.monotonic() + timeout
        while not self.directory.is_dir():
            if time.monotonic() > deadline:
                raise TimeoutError(f"等待仿真启动：{self.directory}")
            time.sleep(0.02)
        self.lock = threading.Lock()          # 本地互斥：一个桥 = 一个客户端
        self.state = None
        self.publication = None
        self.publication_wall = 0.0          # 收到这份状态时的本地单调钟
        self.sequence = 0
        self.sequence_session = None
        self.state_signature = None
        self.token = f"bridge-{os.getpid()}"
        self.push = False                    # True = 仿真连续推流，不再请求-应答
        self.last_frame_file = None          # 推流去重：上一帧的文件名

    # -- 读取 ---------------------------------------------------------------
    def read_json(self, name):
        try:
            with _open_shared(self.directory / name) as handle:
                return json.loads(handle.read())
        except (FileNotFoundError, ValueError, OSError):
            return {}

    def write_json(self, name, value):
        # 临时名带 pid：同机器上可能有第二个客户端（基准脚本）在读同一个邮箱，
        # 共用 ".bridge.tmp" 会撞出 PermissionError。
        temporary = self.directory / f"{name}.{os.getpid()}.tmp"
        data = json.dumps(value, allow_nan=False).encode()
        with open(temporary, "wb") as handle:
            handle.write(data)
        for _ in range(5):
            try:
                os.replace(temporary, self.directory / name)
                return True
            except PermissionError:
                # Windows：目标文件正被仿真 rename 覆盖，稍后重试即可。
                time.sleep(0.001)
        return False

    def read_state(self, timeout=1.0):
        deadline = time.monotonic() + timeout
        while True:
            # 热循环：60 Hz 的控制循环每次 read_state 要轮询等到新状态（~16 ms），
            # 每 2 ms 全量 open+parse 一份 16 KB JSON。Python 侧每次 parse 都占
            # GIL，实测把推理线程从 38 ms 拖到 62 ms（infer 劣化 60%）。
            # 先用 stat 的 (mtime_ns, size) 做签名，没变就直接跳过——
            # 仿真每 60 Hz rename 覆盖一次，签名必然变，语义不变。
            signature = self._file_signature("state.json")
            if signature is not None and signature != self.state_signature:
                value = self.read_json("state.json")
                identity = (value.get("publisher_id"), value.get("publication"))
                fresh = -.1 <= time.time() - value.get("wall_unix_sec", 0) <= .5
                if value.get("protocol") == "rm_aim_sim/2" and identity != self.publication and fresh:
                    self.state_signature = signature
                    self.state = value
                    self.publication = identity
                    self.publication_wall = time.monotonic()
                    return value
            if time.monotonic() > deadline:
                raise TimeoutError("等待仿真状态")
            time.sleep(0.004)

    def _file_signature(self, name):
        """(mtime_ns, size)；仿真 rename 覆盖后签名必变，没变就是同一份内容。"""
        try:
            info = os.stat(self.directory / name)
            return (info.st_mtime_ns, info.st_size)
        except OSError:
            return None

    # -- 发送 ---------------------------------------------------------------
    def send(self, control, fire, yaw, pitch, yaw_vel=0.0, pitch_vel=0.0):
        state = self.state
        if state is None:
            raise RuntimeError("先读状态再发指令")
        session = state["session_id"]
        if self.sequence_session != session:
            self.sequence_session = session
            self.sequence = 0
        self.sequence = max(self.sequence, int(state.get("ipc", {}).get("last_seq", -1)) + 1)
        packet = {"protocol": "rm_aim_sim/2", "type": "command",
                  "session_id": session, "publisher_id": state["publisher_id"],
                  "seq": self.sequence, "control": bool(control), "fire": bool(fire),
                  "yaw": float(yaw), "pitch": float(pitch),
                  "yaw_vel": float(yaw_vel), "pitch_vel": float(pitch_vel)}
        self.sequence += 1
        return self.write_json("command.json", packet), packet["seq"]

    # -- 取图 ---------------------------------------------------------------
    def request_frame(self):
        if self.push:
            # 推流模式：仿真那边看到 push.request 就连续发布最新帧，
            # 不再等 request.json（请求-应答要多等 1~2 个渲染帧）。
            marker = self.directory / "push.request"
            # 每帧重写一次，仿真用它的 mtime 判活；读方退出后 2 秒自动停推
            try:
                marker.write_text(str(os.getpid()))
            except OSError:
                pass
            return "push"
        identity = f"{self.token}-{time.monotonic_ns()}"
        self.write_json("request.json", {"publisher_id": self.state["publisher_id"],
                                         "request_id": identity})
        return identity

    def receive_frame(self, identity, timeout=1.0):
        deadline = time.monotonic() + timeout
        signature = None
        while time.monotonic() < deadline:
            # 同 read_state：mtime 签名没变就不重复 parse，省 GIL 给推理线程
            current = self._file_signature("frame.json")
            if current is not None and current != signature:
                signature = current
                metadata = self.read_json("frame.json")
                if identity == "push" or metadata.get("request_id") == identity:
                    name = metadata.get("frame_file") or ("frame-" + identity + ".raw")
                    # 推流下 frame.json 会被重复写：同一帧不要重复读 6 MB，
                    # 否则桥接把 CPU 全花在重复 I/O 上，推理被拖慢一倍。
                    if identity == "push" and name == self.last_frame_file:
                        time.sleep(0.002)
                        continue
                    self.last_frame_file = name
                    try:
                        with _open_shared(self.directory / name) as handle:
                            pixels = handle.read()
                    except FileNotFoundError:
                        pixels = b""
                    if len(pixels) == metadata.get("image_bytes"):
                        return metadata, pixels
            time.sleep(0.004)
        raise TimeoutError("等待仿真图像")


def default_root():
    return "/dev/shm" if sys.platform.startswith("linux") else os.environ.get("TEMP", ".")


def _open_shared(path):
    """以 FILE_SHARE_DELETE 打开：Windows 上仿真才能 rename 覆盖正在被读的文件。"""
    if not sys.platform.startswith("win"):
        return open(path, "rb")
    import ctypes
    from ctypes import wintypes
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel32.CreateFileW.restype = wintypes.HANDLE
    handle = kernel32.CreateFileW(str(path), 0x80000000, 0x7, None, 3, 0x80, None)
    if handle == wintypes.HANDLE(-1).value:
        raise FileNotFoundError(str(path))
    import msvcrt
    return os.fdopen(msvcrt.open_osfhandle(handle, os.O_RDONLY), "rb", closefd=True)


# ------------------------------------------------------------------------ 弹道

def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def ballistic(speed, gravity, barrel, delta):
    """移植 scripts/aim_assist.gd::_ballistic，返回 (yaw, pitch, flight)。"""
    dx, dy, dz = delta
    horizontal = math.hypot(dx, dy)
    elevation = math.atan2(dz, horizontal)
    for _ in range(8):
        distance = horizontal - barrel * math.cos(elevation)
        height = dz - barrel * math.sin(elevation)
        if distance <= 0.0:
            return None
        if gravity < 1e-9:
            elevation = math.atan2(height, distance)
        else:
            disc = speed ** 4 - gravity * (gravity * distance * distance + 2.0 * height * speed * speed)
            if disc < 0.0:
                return None
            elevation = math.atan((speed * speed - math.sqrt(disc)) / (gravity * distance))
    flight = (horizontal - barrel * math.cos(elevation)) / (speed * math.cos(elevation))
    return math.atan2(dy, dx), -elevation, flight


def _slope(times, values):
    """最小二乘斜率；样本不足或时间不散开时返回 0。"""
    count = len(times)
    if count < 2:
        return 0.0
    mean_t = sum(times) / count
    mean_v = sum(values) / count
    denominator = sum((t - mean_t) ** 2 for t in times)
    if denominator <= 1e-12:
        return 0.0
    return sum((t - mean_t) * (v - mean_v) for t, v in zip(times, values)) / denominator


def _motion_at_now(times, values):
    """以"现在"为原点拟合 y = c0 + c1·u + c2·u²（u≤0），返回 (c1, 2·c2)。

    目标在换向时是 3 m/s² 的加减速，线性外推在 156 ms 飞行里能偏 3.6 cm——
    在 3.8 m 处正好一个装甲板宽。二次项就是补这个。
    """
    count = len(times)
    if count < 4:
        return _slope(times, values), 0.0
    us = [t - times[-1] for t in times]
    # 法方程 (AᵀA)c = Aᵀy，基函数 [1, u, u²]
    s = [sum(u ** k for u in us) for k in range(5)]
    rhs = [sum((u ** k) * y for u, y in zip(us, values)) for k in range(3)]
    matrix = [[s[0], s[1], s[2]],
              [s[1], s[2], s[3]],
              [s[2], s[3], s[4]]]
    solution = _solve3(matrix, rhs)
    if solution is None:
        return _slope(times, values), 0.0
    return solution[1], 2.0 * solution[2]


def _solve3(matrix, rhs):
    """三阶线性方程组：列主元高斯消元，退化时返回 None。"""
    rows = [list(row) + [value] for row, value in zip(matrix, rhs)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda r: abs(rows[r][column]))
        if abs(rows[pivot][column]) < 1e-12:
            return None
        rows[column], rows[pivot] = rows[pivot], rows[column]
        lead = rows[column][column]
        rows[column] = [value / lead for value in rows[column]]
        for r in range(3):
            if r == column:
                continue
            factor = rows[r][column]
            rows[r] = [value - factor * base for value, base in zip(rows[r], rows[column])]
    return [rows[0][3], rows[1][3], rows[2][3]]


def _unwrap(angles):
    """仿真里的 truth.angle 是累计角，正常单调；这里只防偶发跳变。"""
    result = [angles[0]]
    for angle in angles[1:]:
        delta = angle - result[-1]
        while delta > math.pi:
            delta -= 2.0 * math.pi
        while delta < -math.pi:
            delta += 2.0 * math.pi
        result.append(result[-1] + delta)
    return result


class Tracker:
    """真值提前量：平移/自转各估一个速度，再迭代飞行时间。

    两个关键点，都是命中率的瓶颈：
      * 速度用窗口最小二乘而不是最后两点差分，60 Hz 状态下两点差分噪声太大；
      * solve() 的 `ahead` 是"再多预测 ahead 秒"，前后各解一次就得到解本身的角速度，
        喂给伺服做前馈——6 Hz 的二阶伺服跟不上移动目标，调门限治不了这个滞后。
    """

    def __init__(self, window=0.10):
        self.history = deque(maxlen=24)
        self.window = window

    def observe(self, state):
        truth = state.get("truth") or {}
        center = truth.get("center")
        if not center:
            return
        self.history.append((float(state.get("sim_time", 0.0)), list(center),
                             float(truth.get("angle", 0.0))))

    def motion(self):
        """返回 (速度, 加速度, 角速度)。窗口内二次拟合，取"现在"处的导数。"""
        samples = list(self.history)
        if len(samples) < 2:
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0
        newest = samples[-1][0]
        window = [sample for sample in samples if newest - sample[0] <= self.window + 1e-6]
        if len(window) < 2:
            window = samples[-2:]
        times = [sample[0] for sample in window]
        velocity = []
        acceleration = []
        for axis in range(3):
            v, a = _motion_at_now(times, [sample[1][axis] for sample in window])
            velocity.append(v)
            acceleration.append(a)
        # 自转在配置里是恒定角速度，线性拟合比二次稳
        omega = _slope([t - times[0] for t in times], _unwrap([sample[2] for sample in window]))
        if any(abs(c) > 8.0 for c in velocity) or any(abs(c) > 30.0 for c in acceleration) \
                or abs(omega) > 20.0:
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0
        return tuple(velocity), tuple(acceleration), omega

    def solve(self, state, ahead=0.0):
        """返回 {yaw, pitch, armor_id, distance, flight} 或 {}（无可用目标）。"""
        if not state:
            return {}
        health = (state.get("health") or {}).get("current", 0)
        if health <= 0 or state.get("paused"):
            return {}
        truth = state.get("truth") or {}
        armors = truth.get("armors") or []
        center = truth.get("center")
        if not armors or not center:
            return {}
        gimbal = state.get("gimbal") or {}
        weapon = state.get("weapon") or {}
        origin = gimbal.get("origin") or [0.0, 0.0, 0.65]
        speed = float(weapon.get("speed", 23.0))
        gravity = float(weapon.get("gravity", 9.8))
        barrel = float(weapon.get("barrel_length", 0.22))
        lifetime = float(weapon.get("lifetime", 3.0) or 3.0)
        velocity, acceleration, omega = self.motion()
        pitch_min, pitch_max = -0.5236, 0.3491

        best = None
        for armor in armors:
            plate = armor.get("center")
            normal = armor.get("normal") or [0, 0, 0]
            if not plate:
                continue
            to_me = [origin[i] - plate[i] for i in range(3)]
            length = math.sqrt(sum(c * c for c in to_me)) or 1.0
            if sum(to_me[i] * normal[i] for i in range(3)) / length <= 0.2:
                continue                        # 背面装甲不打
            rel = [plate[i] - center[i] for i in range(3)]
            distance = sum((plate[i] - origin[i]) ** 2 for i in range(3)) ** 0.5
            flight = distance / speed
            aim = None
            for _ in range(12):
                t = flight + ahead
                theta = omega * t
                cos_t, sin_t = math.cos(theta), math.sin(theta)
                predicted = [center[0] + velocity[0] * t + 0.5 * acceleration[0] * t * t
                             + rel[0] * cos_t - rel[1] * sin_t,
                             center[1] + velocity[1] * t + 0.5 * acceleration[1] * t * t
                             + rel[0] * sin_t + rel[1] * cos_t,
                             center[2] + velocity[2] * t + 0.5 * acceleration[2] * t * t + rel[2]]
                aim = ballistic(speed, gravity, barrel,
                                [predicted[i] - origin[i] for i in range(3)])
                if aim is None:
                    break
                if abs(aim[2] - flight) < 1e-6:
                    flight = aim[2]
                    break
                flight = aim[2]
            if aim is None:
                continue
            yaw, pitch, flight = aim
            if not (pitch_min <= pitch <= pitch_max) or flight > lifetime:
                continue
            if best is None or distance < best["distance"]:
                best = {"yaw": yaw, "pitch": pitch, "armor_id": armor.get("id"),
                        "distance": distance, "flight": flight}
        return best or {}

    def solve_with_rate(self, state, ahead=0.0, dt=0.016):
        """解 + 解本身的角速度（伺服前馈用）。"""
        aim = self.solve(state, ahead=ahead)
        if not aim:
            return {}, 0.0, 0.0
        later = self.solve(state, ahead=ahead + dt)
        if not later or later.get("armor_id") != aim.get("armor_id"):
            return aim, 0.0, 0.0
        return aim, wrap(later["yaw"] - aim["yaw"]) / dt, (later["pitch"] - aim["pitch"]) / dt


# ----------------------------------------------------------------------- 操作端

class Operator:
    """UDP 租约端。语义照搬 src/operator.rs，另外把指令翻成 command.json。"""

    def __init__(self, channel, port, bind, tolerance, feedforward=True,
                 rate_tolerance=1.2, lead_window=0.10, release_latency=0.0, frames=None,
                 search_amplitude=18.0, search_rate=0.40, coast_seconds=1.5,
                 lead_consume=True, yaw_offset=0.0, pitch_offset=0.0):
        self.channel = channel
        self.port = port
        self.bind = bind
        self.frames = frames                     # 视觉后端时传进来，用它的解而不是真值解
        self.tolerance = tolerance
        self.feedforward = feedforward
        self.rate_tolerance = rate_tolerance
        self.max_fire_gap = 0.08            # 开火要求本帧新鲜检出（秒）；0=不限制
        self.muzzle_lag = 0.02              # 判对齐时看向多少秒后的枪口
        self.min_settle = 0.15              # 换板后稳定多久才允许开火
        self.max_fire_distance = 0.0        # >0 时只在目标近于该距离时开火
        self.truth_log = None               # 60 Hz 真值日志（逐发归因用）
        self.release_latency = release_latency
        self.lock = threading.RLock()
        self.started = time.monotonic()
        self.session = int(time.time() * 1000) & 0xFFFFFFFFFFFF
        self.applied_seq = 0
        self.remote = False
        self.mode = "command"
        self.manual = {"movement": [0.0, 0.0], "gimbal": [0.0, 0.0], "shoot": False}
        self.aim_enabled = False
        self.fire_authorized = False
        self.spin_enabled = False
        self.reason = "idle"
        self.expires = 0.0
        self.last_contact = 0.0
        self.state_age = 999.0
        self.pending = None                 # (seq, control, fire, yaw, pitch) 待确认
        self.flushed_stop = False
        self.search_from = None
        self.search_t0 = 0.0
        self.search_amplitude = search_amplitude
        self.search_rate = search_rate
        self.last_aim = None
        self.last_seen = 0.0
        self.coast_seconds = coast_seconds
        self.lead_consume = lead_consume
        self.calibrate_fire = False            # 标定模式：绕过 aligned 按 10Hz 满打
        self.fire_log = None                   # 开火决策日志路径（JSONL，调试用）
        self.send_log = None                   # 指令发出日志 [(monotonic, fire, yaw)]，测指令时延用
        self.yaw_offset = math.radians(yaw_offset)
        self.pitch_offset = math.radians(pitch_offset)
        self.consume_lead = 0.0                  # 最近一次消费端补的时延（秒），给 HUD/诊断看
        self.lead_ema = 0.04                     # lead 的平滑值（推理线程抖动大，见 vision_command）
        self.rate_ema = (0.0, 0.0)               # 模型角速度的平滑值（远视野解析速率噪声大）
        self.tracker = Tracker(window=lead_window)
        self.stop_requested = False

    # -- 终端状态 ------------------------------------------------------------
    def snapshot(self):
        with self.lock:
            state = self.channel.state or {}
            gimbal = state.get("gimbal") or {}
            origin = gimbal.get("origin") or [0.0, 0.0, 0.0]
            return {
                "session": self.session,
                "applied_seq": max(0, int((state.get("ipc") or {}).get("last_seq", 0))),
                "remote": self.remote,
                "mode": self.mode,
                "manual": self.manual,
                "aim_enabled": self.aim_enabled,
                "fire_authorized": self.fire_authorized,
                "spin_enabled": False,
                "reason": self.reason,
                "shot_count": int((state.get("stats") or {}).get("fired", 0)),
                "position": [origin[0], origin[1]],
                "chassis_heading": 0.0,
                "gimbal_heading": float(gimbal.get("yaw", 0.0)),
            }

    # -- 校验 ---------------------------------------------------------------
    def apply(self, command, now_ms):
        if command.get("session") != self.session:
            return "session_mismatch"
        sequence = command.get("seq")
        if type(sequence) is not int or sequence <= self.applied_seq:
            return "duplicate_or_reordered"
        ttl = command.get("ttl_ms")
        sent = command.get("sent_ms")
        if type(ttl) is not int or not 1 <= ttl <= 500 or type(sent) is not int:
            return "expired_command"
        if sent > now_ms or now_ms - sent >= ttl:
            return "expired_command"
        for key in ("aim_enabled", "fire_authorized", "spin_enabled"):
            if type(command.get(key)) is not bool:
                return "invalid_command"
        if command["fire_authorized"] and not command["aim_enabled"]:
            return "authorization_requires_aim"
        mode = command.get("mode", "command")
        if mode not in ("command", "manual"):
            return "invalid_command"
        manual = command.get("manual") or {}
        axes = list(manual.get("movement") or [0, 0]) + list(manual.get("gimbal") or [0, 0])
        if len(axes) != 4 or any(type(a) not in (int, float) or not math.isfinite(a) or abs(a) > 1.0
                                for a in axes):
            return "invalid_manual_input"
        if type(manual.get("shoot")) is not bool:
            return "invalid_command"
        if mode == "manual" and (command["aim_enabled"] or command["fire_authorized"] or command["spin_enabled"]):
            return "manual_requires_automation_off"
        if mode == "command" and (manual["shoot"] or any(axes)):
            return "manual_input_in_command_mode"
        self.applied_seq = sequence
        self.flushed_stop = False
        self.remote = True
        self.mode = mode
        self.manual = {"movement": list(manual["movement"]), "gimbal": list(manual["gimbal"]),
                       "shoot": manual["shoot"]}
        self.aim_enabled = command["aim_enabled"]
        self.fire_authorized = command["fire_authorized"]
        self.spin_enabled = command["spin_enabled"]
        self.expires = time.monotonic() + max(0.0, (sent + ttl - now_ms) / 1000.0)
        self.last_contact = time.monotonic()
        return "applied"

    def revoke(self, reason):
        self.session += 1
        self.applied_seq = 0
        self.flushed_stop = False
        self.remote = False
        self.mode = "command"
        self.manual = {"movement": [0.0, 0.0], "gimbal": [0.0, 0.0], "shoot": False}
        self.aim_enabled = False
        self.fire_authorized = False
        self.spin_enabled = False
        self.reason = reason

    # -- 控制回路 ------------------------------------------------------------
    def control_loop(self, hz=60.0):
        period = 1.0 / hz
        last = time.monotonic()
        state_misses = 0
        while not self.stop_requested:
            try:
                state = self.channel.read_state(timeout=0.2)
                state_misses = 0
                self.state_age = 0.0
                # 逐发归因：60 Hz 落盘真值装甲位姿，事后精确重建"弹道与装甲平面交会"
                if self.truth_log is not None:
                    truth = state.get("truth") or {}
                    armors = truth.get("armors")
                    if armors:
                        try:
                            self.truth_log.write(json.dumps({
                                "t": state.get("sim_time"),
                                "a": [[int(x["id"])] + [round(float(v), 5) for v in x["center"]] +
                                      [round(float(v), 5) for v in x["normal"]] for x in armors],
                            }) + "\n")
                        except (OSError, KeyError, TypeError):
                            pass
                self.tracker.observe(state)
                with self.lock:
                    if self.remote and time.monotonic() > self.expires:
                        self.revoke("operator_timeout")
                    command = self.compute(state)
                if command is not None:
                    control, fire, yaw, pitch, yaw_vel, pitch_vel = command
                    sent_at = time.monotonic()
                    ok, seq = self.channel.send(control, fire, yaw, pitch, yaw_vel, pitch_vel)
                    if self.send_log is not None:
                        try:
                            self.send_log.append((sent_at, bool(fire), float(yaw)))
                        except Exception:
                            pass
                    with self.lock:
                        if ok and not control:
                            self.pending = None
                        elif ok:
                            self.pending = seq
            except TimeoutError:
                self.state_age += period
                state_misses += 1
                with self.lock:
                    # 仿真偶尔卡一帧（渲染/GC/掉帧）不该把整轮接管撤销：
                    # 一次读超时就 revoke 会让 session+1，终端若不重同步，
                    # 之后每个包都 session_mismatch、整轮数据报废
                    # （bench 曾 355/629 个 mismatch）。连续 3 次
                    # （≈0.6 s 拿不到状态）才认为仿真真的离线。
                    if state_misses >= 3 and (self.remote or self.aim_enabled):
                        self.revoke("simulator_offline")
            except Exception as exc:                      # 桥不该因为一次失败就死
                with self.lock:
                    self.reason = f"bridge_error: {exc}"
            time.sleep(max(0.0, period - (time.monotonic() - last)))
            last = time.monotonic()

    def compute(self, state):
        """把终端意图翻成 (control, fire, yaw, pitch, yaw_vel, pitch_vel)。"""
        gimbal = state.get("gimbal") or {}
        yaw = float(gimbal.get("yaw", 0.0))
        pitch = float(gimbal.get("pitch", 0.0))
        if not self.remote:
            # 撤销后只补一帧停止指令就静默，别在没人接管时每秒写 60 次文件。
            if self.flushed_stop:
                return None
            self.flushed_stop = True
            return (False, False, yaw, pitch, 0.0, 0.0)
        if self.mode == "manual":
            rate = 6.0                                   # gimbal.max_speed，rad/s
            gx, gy = self.manual["gimbal"]
            return (True, bool(self.manual["shoot"]),
                    yaw + gx * rate / 60.0, pitch - gy * rate / 60.0, 0.0, 0.0)
        if not self.aim_enabled:
            return True, False, yaw, pitch, 0.0, 0.0
        if self.frames is not None:
            return self.vision_command(yaw, pitch,
                                       float(gimbal.get("yaw_vel", 0.0)),
                                       float(gimbal.get("pitch_vel", 0.0)))
        # 状态是 60 Hz 的快照，指令要过一会儿才被仿真消费、再飞行才有落点。
        # 这段时延在自转快时直接变成 yaw 误差（4 rad/s 下 16 ms 就是 3.7°），
        # 所以把"状态年龄 + 生效延迟"一起补进预测。
        age = time.monotonic() - (self.channel.publication_wall or time.monotonic())
        ahead = min(max(age, 0.0), 0.08) + self.release_latency
        aim, yaw_vel, pitch_vel = self.tracker.solve_with_rate(state, ahead=ahead)
        if not aim:
            self.reason = "no_target"
            return True, False, yaw, pitch, 0.0, 0.0
        error_yaw = abs(wrap(aim["yaw"] - yaw))
        error_pitch = abs(aim["pitch"] - pitch)
        aligned = error_yaw < self.tolerance and error_pitch < self.tolerance
        if self.feedforward:
            # 云台还没跟上目标角速度时满速开火只是浪费弹，也是低命中率的主因。
            actual_yaw_vel = float(gimbal.get("yaw_vel", 0.0))
            actual_pitch_vel = float(gimbal.get("pitch_vel", 0.0))
            aligned = aligned \
                and abs(actual_yaw_vel - yaw_vel) < self.rate_tolerance \
                and abs(actual_pitch_vel - pitch_vel) < self.rate_tolerance
        fire = bool(self.fire_authorized and aligned)
        self.reason = ("firing" if fire else
                       f"tracking {aim['distance']:.1f}m "
                       f"{aim['flight'] * 1000:.0f}ms err {math.degrees(error_yaw):.2f}°")
        # 小陀螺在这个仿真里没有底盘，只把它当"未支持"回报给终端。
        if self.spin_enabled:
            self.reason += " · 小陀螺无底盘"
        return True, fire, aim["yaw"], aim["pitch"], yaw_vel, pitch_vel

    def vision_command(self, yaw, pitch, gimbal_yaw_vel=0.0, gimbal_pitch_vel=0.0):
        """用视觉后端的解。它比真值解晚一个取帧+推理周期，所以要按时延外推。"""
        aim = self.frames.aim
        fresh = aim and aim[0] is not None and time.monotonic() - aim[3] <= 0.25
        if not fresh:
            info = (aim[4] if aim and len(aim) > 4 else {}) or {}
            now = time.monotonic()
            # 刚刚还看得见：先保持指向、别立刻回去扫，否则搜索和跟踪互相打断
            # （扫到 → 枪还没转过去 → 下一帧丢了 → 又回到扫，永远锁不上）。
            if self.last_aim is not None and now - self.last_seen < self.coast_seconds:
                self.reason = (info.get("reason") or "coast") + " · 惯性保持"
                return True, False, self.last_aim[0], self.last_aim[1], 0.0, 0.0
            self.reason = (info.get("reason") or "no_target") + " · 搜索中"
            # 视觉看不到就是真看不到：真值后端不存在这个问题，所以这里必须有搜索。
            # 以进入搜索时的 yaw 为中心慢扫。
            if self.search_from is None:
                self.search_from = yaw
                self.search_t0 = now
            elapsed_search = now - self.search_t0
            # 单向慢扫而不是绕固定中心来回摆：来回摆会卡在某个偏心的中心上，
            # 目标在视野外就永远找不到（实测云台会锁在 +90° 附近抖）。
            goal = wrap(self.search_from + self.search_rate * elapsed_search)
            return True, False, goal, 0.0, 0.0, 0.0
        self.search_from = None
        self.last_seen = time.monotonic()
        goal_yaw, goal_pitch, _wants_fire, stamp, info, anchor, capture_stamp = aim
        elapsed = time.monotonic() - stamp
        # aim 龄期喂给推理线程算 delay：观测→被消费要等多久，提前量就补多少
        if self.frames is not None:
            self.frames.pipeline_ema = 0.9 * self.frames.pipeline_ema \
                + 0.1 * max(0.0, elapsed)
        rate_yaw, rate_pitch = self.frames.aim_rate
        # 模型解析出来的角速度比"瞄准值的帧间差分"稳：板 id 关联有 θ↔θ+π 的重名等价，
        # 瞄准解会在板之间跳，差分出来的速率是纯噪声，会把前馈门限永久卡住。
        # 不要对它做长时常数平滑：目标横摆的角加速度实测可达 ~3 rad/s²，150 ms 的
        # EMA 会让速率滞后 0.5 rad/s，折算成云台位置误差 2δr/ω ≈ 1.5°，比速率
        # 本身的噪声（白噪声下 0.26°）还大一个量级。
        model_rate = info.get("aim_rate")
        if model_rate:
            rate_yaw, rate_pitch = float(model_rate[0]), float(model_rate[1])
        # 后端把"指令生效→子弹掠甲"的时延（command_latency）折进了弹道提前量，
        # 所以解算值对应的是 anchor + command_latency 处的目标。但这条指令要等
        # 控制循环下一次读到才生效，开火门限还要等云台对齐——从 anchor 到指令
        # 真正发出平均有 20~250 ms 的空档，2 rad/s 自转下 50 ms 就是 5.7°，
        # 比装甲半宽还大。用解算自带的模型角速度从 anchor 外推补上这段。
        # 但补的量必须是**后端 delay 没覆盖的那部分**：后端的
        # delay = 0.020 + pipeline_ema，pipeline_ema ≈ elapsed = 消费时刻 − 推理
        # 完成时刻，已经覆盖了"推理完成→被消费"这段。所以 lead 只能补
        # (推理完成 − 帧内容) + 门限对齐余量 = stamp − anchor + 30ms。
        # 旧写法用 now − anchor（= infer + elapsed），把 elapsed 算了两遍——
        # 旧运行 elapsed 只有 10~20 ms 时被掩盖，infer 劣化到 62 ms 后
        # elapsed 涨到 125 ms，过补偿 85 ms ≈ 9.7°@2rad/s，弹着系统性偏掉。
        # 也不能用瞬时 raw：推理线程被 GIL 挤占时 stamp−anchor 会在 30~130 ms
        # 间跳，乘以角速度就是 ±2° 的目标角抖动。取 ~80 ms 时间常数的 EMA。
        self.consume_lead = 0.0
        raw_aim_yaw, raw_aim_pitch = goal_yaw, goal_pitch
        if self.lead_consume:
            raw = max(0.0, stamp - anchor) + 0.030
            self.lead_ema = 0.8 * self.lead_ema + 0.2 * raw
            self.consume_lead = min(self.lead_ema, 0.35)
            goal_yaw = wrap(goal_yaw + rate_yaw * self.consume_lead)
            goal_pitch = goal_pitch + rate_pitch * self.consume_lead
        self.last_aim = (goal_yaw, goal_pitch)
        # yaml 的 yaw_offset / pitch_offset：系统性的角度偏置。云台会稳态地偏离视觉解
        # 这个量，所以必须在 aligned 判据之前加上——补偿的正是"标定带来的固定偏角"。
        goal_yaw = wrap(goal_yaw + self.yaw_offset)
        goal_pitch = goal_pitch + self.pitch_offset
        # 用"出膛瞬间的预测枪口角"判对齐，而不是等云台此刻对齐：
        # 6 Hz 伺服的稳态滞后 ~1°，等它进 0.45° 窗口要 0.1–0.3 s，那段等待
        # 不可预知，提前量只能猜——实测是剩余 ~0.9° 误差的主因。
        muzzle_lag = float(getattr(self, "muzzle_lag", 0.0))
        muzzle_yaw = yaw + float(gimbal_yaw_vel) * muzzle_lag
        muzzle_pitch = pitch + float(gimbal_pitch_vel) * muzzle_lag
        aligned = abs(wrap(goal_yaw - muzzle_yaw)) < self.tolerance \
            and abs(goal_pitch - muzzle_pitch) < self.tolerance
        # 追踪速度也要跟上：只卡"角度对齐"会在云台大角速度扫过时开火——枪口虽
        # 在窗口内，但云台还在追，出膛瞬间就甩出去了（真值路径靠这条拿到 98%）。
        if self.feedforward:
            rate_ok = abs(float(gimbal_yaw_vel) - rate_yaw) < self.rate_tolerance \
                and abs(float(gimbal_pitch_vel) - rate_pitch) < self.rate_tolerance
            aligned = aligned and rate_ok
        # 后端自带的 FireGate 比的是"解算值 vs 这一帧的云台姿态"，没算取帧+推理解算的
        # 时延；在这个仿真里那一项永远超窗（47 ms × 2 rad/s ≈ 5°）。所以开火判据
        # 用我们自己的"外推后的解 vs 当前云台"，后端闸门只作为跟踪质量的参考。
        # calibrate_fire：标定时绕过 aligned，按 10 Hz 满打出膛，用逐发偏差拟合最优时延。
        fire = bool(self.fire_authorized and info.get("tracking")
                    and (aligned or self.calibrate_fire))
        # 只打"这一帧有新鲜检出"的解：gap 大时是纯外推，实测是重尾误差的主因
        # （p90 6~16°，而中位只有 1.3°）。
        max_gap = float(self.max_fire_gap)
        if max_gap > 0.0 and float(info.get("gap", 0.0)) > max_gap:
            fire = False
        # 换板瞬间瞄准解会跳，等稳定再打（重尾误差的主因之一）
        if self.min_settle > 0.0 and float(info.get("settle", 1.0)) < self.min_settle:
            fire = False
        # 只在近距离打：角尺寸更大、命中率显著更高（分母是开火数，这是合法杠杆）
        if self.max_fire_distance > 0.0 and float(info.get("distance", 0.0)) > self.max_fire_distance:
            fire = False
        # 视觉后端的到达朝向门（AIM_MIN_ARRIVAL_FACING）：子弹到达时板已转成侧向就不打
        if info.get("arrival_block"):
            fire = False
        if self.fire_log:
            try:
                with open(self.fire_log, "a", encoding="utf-8") as handle:
                    handle.write(json.dumps({
                        "t": round(time.monotonic(), 4),
                        "elapsed": round(elapsed, 4), "lead": round(self.consume_lead, 4),
                        "aim_yaw": round(raw_aim_yaw, 4),
                        "aim_rate": round(rate_yaw, 4),
                        "anchor": round(anchor, 4), "capture": round(capture_stamp, 4),
                        "infer_done": round(stamp, 4),
                        "err_yaw": round(math.degrees(wrap(goal_yaw - yaw)), 3),
                        "err_pitch": round(goal_pitch - pitch, 3),
                        "rate_yaw": round(rate_yaw, 3),
                        "gimbal_yaw_vel": round(gimbal_yaw_vel, 3),
                        "tracking": bool(info.get("tracking")), "fired": fire,
                    }) + "\n")
            except OSError:
                pass
        self.reason = (f"vision {'firing' if fire else 'tracking'} "
                       f"{info.get('distance', 0):.1f}m conf {info.get('confidence', 0):.2f} "
                       f"gate {info.get('reason', '')} · {elapsed * 1000:.0f}ms "
                       f"+{self.consume_lead * 1000:.0f}ms · "
                       f"err {math.degrees(abs(wrap(goal_yaw - yaw))):.2f}°")
        return True, fire, goal_yaw, goal_pitch, rate_yaw, rate_pitch

    # -- UDP 服务 ------------------------------------------------------------
    def serve(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.bind, self.port))
        print(f"操作租约：udp://{self.bind}:{self.port}", flush=True)
        while not self.stop_requested:
            try:
                data, peer = sock.recvfrom(4096)
            except OSError:
                continue
            packet = data.strip()
            if packet != b"status":
                with self.lock:
                    try:
                        command = json.loads(packet)
                        if type(command) is not dict:
                            raise ValueError
                        result = self.apply(command, time.time_ns() // 1_000_000)
                        # "applied" 只是终端指令通过校验；reason 仍留给跟踪/开火状态。
                        if result != "applied":
                            self.reason = result
                    except (ValueError, TypeError):
                        self.reason = "invalid_command"
            with self.lock:
                payload = self.snapshot()
            try:
                sock.sendto(json.dumps(payload, ensure_ascii=False).encode(), peer)
            except OSError:
                pass


# ----------------------------------------------------------------------- 画面

class Frames:
    """向仿真取帧，转 JPEG 缓存；终端按 5fps 左右来拉。

    视觉后端模式下（attach 了 backend），它同时是**唯一**的取帧者：
    以固定频率连续取帧、跑检测，并把结果挂到 self.aim 上。
    这样 HUD 看到的就是检测器看到的那一帧，不会两边抢 request.json。
    """

    def __init__(self, channel, port, bind, width=1280, quality=80, min_interval=0.06):
        self.channel = channel
        self.port = port
        self.bind = bind
        self.width = width
        self.quality = quality
        self.min_interval = min_interval
        self.backend = None
        self.state_source = False             # True = 不取图，直接按状态频率跑
        # 指令生效到子弹掠过装甲的时延（秒）：仿真里 = 指令轮询 + 物理步进 + 武器延迟
        # + 出膛时的云台姿态效应。但它只补"下发之后"那段；"观测→下发"由消费端按 anchor
        # 从观测时刻外推，两段各管一段别混。
        # 实测（send 日志对 shot 事件）指令→出膛中位 20 ms；而解算值被消费时平均已经
        # 放了 pipeline_ema 秒（控制循环排队 + 推理）。弹着用的云台指令是"出膛前最后
        # 发出去的那条"，它对应的目标角 = observe + (aim龄期) + 20ms 处的装甲。
        # 所以 delay = aim龄期EMA + 20ms：随流水线快慢自适应，60Hz 状态源 ≈45ms，
        # 30Hz 图像链路自动变长。旧写法把整段时延都塞进 delay、又用 anchor=stamp+delay
        # 做外推，日常把外推量 clamp 成 0，门限等云台对齐的 60~80ms 全丢了。
        self.command_latency = 0.045            # 兜底默认值（60Hz 状态源实测最优）
        self.pipeline_ema = 0.025               # aim 从解算到被消费的平均龄期（EMA）
        self.vision_latency = None              # 非空则覆盖整个 delay（标定用）
        self.sim_wall_offset = None           # 仿真时钟 → unix 墙钟的偏移（EMA）
        self.mono_minus_unix = time.monotonic() - time.time()
        self.hz = 0.0
        self.real_hz = 0.0                   # 取帧线程真实吞吐（目标频率超过串行耗时时会低于目标）
        self.aim_hz = 0.0                    # 解算（aim）真实输出频率
        self.frame_slot = None               # 最新一帧：(metadata, pixels, h, w, c, stamp, id)
        self.frame_event = threading.Event()
        self.capture_ema = 0.0               # 取帧（IPC 往返 + 读原始像素）耗时
        self.convert_ema = 0.0               # RGB→BGR 拷贝耗时
        self.aim = None                      # (yaw, pitch, fire, monotonic, info)
        self.aim_rate = (0.0, 0.0)
        self.infer_ema = 0.04                # 推理耗时估计，用来算弹道提前量里的时延
        self.lock = threading.Lock()
        self.jpeg = None
        self.jpeg_stamp = 0.0
        self.raw_pixels = None
        self.raw_shape = (0, 0, 0)
        self.raw_mode = "RGB"
        self.stamp = 0.0
        self.frames = 0
        self.error = ""

    def grab(self):
        if self.backend is not None:
            # 视觉模式下帧由 _loop 连续在取，这里按需把最新一帧编成 JPEG
            with self.lock:
                if self.raw_pixels is None or time.monotonic() - self.stamp > 1.0:
                    return None
                if self.jpeg is None or self.jpeg_stamp != self.stamp:
                    self.jpeg = self._encode(self.raw_pixels, self.raw_shape, self.raw_mode)
                    self.jpeg_stamp = self.stamp
                return self.jpeg
        with self.lock:
            age = time.monotonic() - self.stamp
            if self.jpeg is not None and age < self.min_interval:
                return self.jpeg
        self._capture()
        with self.lock:
            # 取不到新帧就只给 1 秒内的缓存：控制界面里放陈旧画面比黑屏更误导。
            if time.monotonic() - self.stamp > 1.0:
                return None
            if self.jpeg is None or self.jpeg_stamp != self.stamp:
                self.jpeg = self._encode(self.raw_pixels, self.raw_shape, self.raw_mode)
                self.jpeg_stamp = self.stamp
            return self.jpeg

    def _encode(self, pixels, shape, mode):
        if Image is None:
            self.error = "缺少 Pillow"
            return None
        height, width, _ = shape
        image = Image.frombuffer(mode, (width, height), pixels, "raw", mode, 0, 1)
        if mode == "RGBA":
            image = image.convert("RGB")
        if width > self.width:
            image = image.resize((self.width, max(1, height * self.width // width)), Image.BILINEAR)
        buffer = io.BytesIO()
        image.save(buffer, format="JPEG", quality=self.quality)
        return buffer.getvalue()

    def attach(self, backend, hz):
        """接管取帧：取帧线程与推理线程并行，HUD 与检测共用同一帧。

        原来是"取帧 → 推理"串行：1440×1080 下取帧 33 ms + 推理 38 ms + 颜色拷贝
        11 ms ≈ 82 ms，天花板只有 12 fps。拆成两条线程后单帧周期变成
        max(取帧, 推理) 而不是相加，代价是推理可能吃到稍旧的一帧（丢弃最旧、只认最新）。
        """
        self.backend = backend
        self.hz = hz
        threading.Thread(target=self._capture_loop, daemon=True).start()
        threading.Thread(target=self._infer_loop, daemon=True).start()

    def _capture_loop(self):
        period = 1.0 / self.hz
        previous = 0.0
        while True:
            started = time.monotonic()
            self._capture()
            finished = time.monotonic()
            if previous > 0.0:
                # 真实吞吐：目标频率可能高过"取帧 + 推理"的串行耗时，这时它只会空转
                self.real_hz = 0.9 * self.real_hz + 0.1 / max(1e-3, finished - previous)
            previous = finished
            # 推流模式的节拍由仿真发布决定（receive_frame 会等下一帧唯一帧），
            # 这里再 sleep 会和发布周期叠加成 2 倍周期（实测 capture 16→30 ms）。
            if not self.channel.push:
                time.sleep(max(0.0, period - (finished - started)))

    def _capture(self):
        """取一帧 → JPEG 缓存；挂了后端就再跑一遍视觉，产出云台解。

        state_source=True 时不向仿真要图，直接用 60 Hz 的状态快照——理想检测器的
        角点本来就来自 state["camera"]["armors"]，图像一次都没用到，按状态频率跑
        反而更忠实（也顺带测出"帧率到底还差多少"）。
        """
        import numpy
        capture_start = time.monotonic()
        if self.state_source:
            try:
                state = self.channel.read_state(timeout=0.05)
            except (TimeoutError, OSError):
                return
            if not state:
                return
            metadata = dict(state)
            metadata["type"] = "frame"
            pixels = b""
            width = int((state.get("camera") or {}).get("width") or 1)
            height = int((state.get("camera") or {}).get("height") or 1)
            channels = 3
            mode = "RGB"
            with self.lock:
                self.raw_pixels = None
                self.raw_shape = (height, width, channels)
                self.raw_mode = mode
                self.stamp = time.monotonic()
                self.frames += 1
                self.error = ""
                self.frame_slot = (metadata, pixels, height, width, channels,
                                   self.stamp, self.frames)
                self.frame_event.set()
            capture_done = time.monotonic()
            self.capture_ema = 0.8 * self.capture_ema + 0.2 * (capture_done - capture_start)
            return
        with self.lock:
            if Image is None:
                self.error = "缺少 Pillow"
                return
            try:
                if self.channel.state is None:
                    self.channel.read_state(timeout=0.05)
                identity = self.channel.request_frame()
                metadata, pixels = self.channel.receive_frame(identity, timeout=0.8)
                camera = metadata.get("camera") or {}
                width = int(camera.get("width", 0))
                height = int(camera.get("height", 0))
                channels = 3 if metadata.get("encoding") == "rgb8" else 4
                if width <= 0 or height <= 0 or len(pixels) != width * height * channels:
                    raise ValueError("图像尺寸不符")
                mode = "RGB" if channels == 3 else "RGBA"
                # 不在这里做 JPEG：视觉模式下每一帧都编码是多花的 10~15 ms，
                # 而 HUD 只有 5 fps。原始像素留着，真要图时再编（见 grab）。
                self.raw_pixels = pixels
                self.raw_shape = (height, width, channels)
                self.raw_mode = mode
                self.stamp = time.monotonic()
                self.frames += 1
                self.error = ""
                # 最新一帧交给推理线程；上一帧没消费掉就直接覆盖（宁可丢帧也不要排队，
                # 排队只会把时延越拖越长）。
                self.frame_slot = (metadata, pixels, height, width, channels, self.stamp, self.frames)
                self.frame_event.set()
            except (TimeoutError, ValueError, OSError) as exc:
                self.error = str(exc)
                return
        capture_done = time.monotonic()
        self.capture_ema = 0.8 * self.capture_ema + 0.2 * (capture_done - capture_start)

    def _infer_loop(self):
        """独立线程消费最新一帧：颜色转换 → 检测 → PnP → 跟踪 → 弹道。"""
        import numpy
        seen = 0
        got_at = 0.0
        while True:
            slot = self.frame_slot
            if slot is None or slot[6] == seen:
                self.frame_event.wait(0.05)
                self.frame_event.clear()
                continue
            seen = slot[6]
            metadata, pixels, height, width, channels, stamp, _ = slot
            backend = self.backend
            if backend is None:
                continue
            if pixels:
                array = numpy.frombuffer(pixels, dtype=numpy.uint8).reshape(height, width, channels)
                bgr = numpy.ascontiguousarray(array[:, :, 2::-1])     # RGB(A)→BGR，只拷一次
            else:
                bgr = numpy.zeros((max(1, height), max(1, width), 3), dtype=numpy.uint8)
            call_start = time.monotonic()
            self.convert_ema = 0.8 * self.convert_ema + 0.2 * (call_start - stamp)
            # 帧内容对应的墙钟用**仿真时钟↔墙钟的映射**算，不猜：
            # state.json 里同时带 sim_time 和发布时间，直接读一份最新的做基准
            # （不走阻塞的 read_state——那会等下一个 publication，把时延算糊涂）。
            # anchor 必须是"这一帧内容成立的那个墙钟时刻"，消费端从它外推到指令
            # 真正发出的时刻；旧写法把 anchor 放在 stamp+delay（预测时点）上，
            # 日常 W2 < anchor 时外推量被 clamp 成 0，开火门限等云台对齐的那
            # 0.15~0.25 s 就白白丢了（实测标定值因此迁移不进闭环）。
            frame_sim = float(metadata.get("sim_time") or 0.0)
            fresh = self.channel.read_json("state.json")
            now_sim = float(fresh.get("sim_time") or 0.0)
            wall_unix = float(fresh.get("wall_unix_sec") or 0.0)
            if now_sim > 0.0 and wall_unix > 0.0:
                if self.sim_wall_offset is None:
                    self.sim_wall_offset = wall_unix - now_sim
                else:
                    self.sim_wall_offset = 0.9 * self.sim_wall_offset + 0.1 * (wall_unix - now_sim)
            self.mono_minus_unix = 0.99 * self.mono_minus_unix \
                + 0.01 * (time.monotonic() - time.time())
            if frame_sim > 0.0 and self.sim_wall_offset is not None:
                anchor_wall = frame_sim + self.sim_wall_offset + self.mono_minus_unix
            else:
                anchor_wall = stamp
            if self.vision_latency is not None:
                delay = self.vision_latency
            else:
                # aim 龄期（观测→被消费）+ 实测的指令→出膛 20ms
                delay = 0.020 + self.pipeline_ema
            try:
                # 帧元数据里只有相机；把同一时刻的状态真值/武器补给它。
                # 解算只用 weapon（缺省值与仿真一致），truth 只给 AIM_RECORD 录制用。
                aim_state = dict(metadata)
                for key in ("truth", "weapon", "tick"):
                    if key not in aim_state and fresh.get(key) is not None:
                        aim_state[key] = fresh[key]
                # 状态比这一帧新 pipeline_ema 秒；给录制/诊断标出真值对应的时间
                aim_state["state_sim_time"] = fresh.get("sim_time")
                yaw, pitch, fire, info = backend.step(aim_state, bgr, time.time_ns(),
                                                      delay_s=delay, observe_time_s=frame_sim)
            except Exception as exc:                  # 单帧解算出错不该让整条推理线程死掉
                self.error = f"aim: {exc}"
                self.aim = (None, None, False, time.monotonic(), {"reason": "aim_error"}, time.monotonic(), time.monotonic())
                continue
            finished = time.monotonic()
            with self.lock:
                self.infer_ema = 0.7 * self.infer_ema + 0.3 * max(0.0, finished - call_start)
                if seen > 1 and got_at > 0.0:
                    self.aim_hz = 0.9 * self.aim_hz + 0.1 / max(1e-3, finished - got_at)
            got_at = finished
            now = finished
            if yaw is None:
                self.aim = (None, None, False, now, info, now, now)
                continue
            previous = self.aim
            if previous and previous[0] is not None:
                dt = now - previous[3]
                if 0.0 < dt < 0.5:               # 给云台喂速度前馈，缺了它自转靶会系统性滞后
                    self.aim_rate = (wrap(yaw - previous[0]) / dt, (pitch - previous[1]) / dt)
            # anchor = 这一帧内容对应的墙钟时刻（不是预测时点）。解算值本身已经带了
            # command_latency 的提前量，消费端再从 anchor 外推到"指令真正发出"的
            # 时刻，把推理耗时、控制循环排队、开火门限等云台对齐的时间都补上。
            self.aim = (yaw, pitch, fire, now, info, anchor_wall, stamp)

    def serve(self):
        frames = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                path = urlparse(self.path).path
                if path in ("/snapshot/raw", "/snapshot", "/frame"):
                    image = frames.grab()
                    if image is None:
                        # send_error 的 message 走 latin-1 编码，中文会抛
                        # UnicodeEncodeError 把响应线程打挂，这里只发 ASCII。
                        self.send_error(503, "frame unavailable")
                    else:
                        self.send_response(200)
                        self.send_header("Content-Type", "image/jpeg")
                        self.send_header("Cache-Control", "no-store")
                        self.send_header("Content-Length", str(len(image)))
                        self.end_headers()
                        self.wfile.write(image)
                elif path == "/health":
                    body = json.dumps({"frames": frames.frames, "error": frames.error,
                                       "hz": round(frames.real_hz, 2),
                                       "aim_hz": round(frames.aim_hz, 2),
                                       "infer_ms": round(frames.infer_ema * 1000, 1),
                                       "capture_ms": round(frames.capture_ema * 1000, 1),
                                       "convert_ms": round(frames.convert_ema * 1000, 1),
                                       "size": list(frames.raw_shape),
                                       "state_age": round(frames.channel.state is not None, 1)}).encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                else:
                    self.send_error(404)

            def log_message(self, *_):
                pass

        server = ThreadingHTTPServer((self.bind, self.port), Handler)
        server.daemon_threads = True
        print(f"画面：http://{self.bind}:{self.port}/snapshot/raw", flush=True)
        server.serve_forever()


def main():
    parser = argparse.ArgumentParser(description="rm_aim_sim -> Daedalus 操作终端桥接")
    parser.add_argument("--channel", default="default", help="与仿真 [ipc].channel 一致")
    parser.add_argument("--channel-root", default=None, help="默认 Linux /dev/shm，其它平台 %%TEMP%%")
    parser.add_argument("--operator-port", type=int, default=5001, help="终端连的 UDP 租约端口")
    parser.add_argument("--video-port", type=int, default=5000, help="终端拉的 HTTP 画面端口")
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--tolerance-deg", type=float, default=0.45, help="开火角度门限")
    parser.add_argument("--lead-window", type=float, default=0.10,
                        help="估目标速度的取样窗口（秒），越小越跟手、越大越稳")
    parser.add_argument("--no-feedforward", action="store_true",
                        help="不给云台喂角速度前馈（对照实验用）")
    parser.add_argument("--rate-tol", type=float, default=1.2,
                        help="开火前允许的角速度偏差（rad/s）")
    parser.add_argument("--release-latency", type=float, default=0.0,
                        help="指令生效到出膛的附加延迟（秒），一并补进预测；实测 0 最好")
    # 自瞄后端：truth = 仿真真值几何解（验收脚手架）；vision = 我们自己的视觉管线
    parser.add_argument("--backend", choices=("truth", "vision"), default="truth")
    parser.add_argument("--detector", choices=("rp0526", "yolo26", "ideal"), default="rp0526",
                        help="vision 后端用哪个检测器；ideal=用仿真理想角点，只验链路")
    parser.add_argument("--model", default="", help="onnx 路径，留空按检测器取默认")
    parser.add_argument("--confidence", type=float, default=0.5)
    parser.add_argument("--device", default="CPU")
    parser.add_argument("--vision-hz", type=float, default=20.0,
                        help="vision 后端的取帧+推理频率")
    parser.add_argument("--state-source", action="store_true",
                        help="不向仿真要图，直接按 60 Hz 状态跑（仅 ideal 检测器有意义）")
    parser.add_argument("--vision-latency", type=float, default=None,
                        help="覆盖 command_latency（指令生效→子弹掠甲的时延，秒）；默认 0.10 实测标定")
    parser.add_argument("--no-roi", action="store_true",
                        help="关闭 ROI 裁剪（默认开：按上一帧目标框放大裁一块再检测）")
    parser.add_argument("--roi-margin", type=float, default=2.2, help="ROI 边长相对目标框的倍数")
    parser.add_argument("--corner-offset", type=float, default=0.0,
                        help="角点整体内/外移像素（负=内收）；仿真上 -1 可抹掉 0.18m 的深度偏置")
    parser.add_argument("--use-body", action="store_true",
                        help="走刚体模型（车心+车身角）。默认关：19~57px 小板上 id 指配信噪比不足，"
                             "单板时序跟踪实测更准（见 vision_backend.BodyTracker 注释）")
    parser.add_argument("--motion-window", type=float, default=0.12,
                        help="运动拟合/滤波窗口（秒）。16 Hz 视觉下 0.25 s 只有 ~4 样本，"
                             "二次拟合会被噪声吃掉；实测 0.12 s 的线性拟合跟踪最稳")
    parser.add_argument("--search-amplitude", type=float, default=18.0,
                        help="看不到目标时云台慢扫的半幅（度）")
    parser.add_argument("--search-rate", type=float, default=0.40,
                        help="没目标时云台的搜索角速度（rad/s，0.4≈23°/s）")
    parser.add_argument("--coast-seconds", type=float, default=1.5,
                        help="刚看到目标后又丢了，先保持指向多少秒再回去扫")
    parser.add_argument("--lead-consume", action="store_true", default=True,
                        help="开启消费端时延外推（默认开：早期实测速率噪声大于收益，"
                             "但那是瞬时值外推；现在用 ~80ms 常数的 EMA 平滑，"
                             "且所有拿到 17%%+ 命中率的运行都是开着的）")
    parser.add_argument("--no-lead-consume", action="store_true",
                        help="关掉消费端时延外推（对照组）")
    parser.add_argument("--calibrate-fire", action="store_true",
                        help="标定模式：视觉开火绕过 aligned 门限按 10Hz 满打，用来逐发测偏差")
    parser.add_argument("--fire-log", default="",
                        help="开火决策日志路径（JSONL：每条的 err/速率/是否开火）")
    parser.add_argument("--send-log", action="store_true",
                        help="记录指令发出日志，退出时连同 shot 事件一起算指令→出膛时延")
    parser.add_argument("--yaw-offset", type=float, default=0.0,
                        help="云台 yaw 标定偏置（度，对应 yaml 的 yaw_offset）；系统偏差有多大就补多少")
    parser.add_argument("--pitch-offset", type=float, default=0.0,
                        help="云台 pitch 标定偏置（度，对应 yaml 的 pitch_offset）")
    parser.add_argument("--alpha", type=float, default=0.70, help="跟踪器位置增益")
    parser.add_argument("--beta", type=float, default=0.12, help="跟踪器速度增益")
    parser.add_argument("--max-fire-gap", type=float, default=0.08,
                        help="开火要求瞄准解对应的检出有多新鲜（秒）；超过就只瞄不打。"
                             "丢帧外推是重尾误差的主因")
    parser.add_argument("--muzzle-lag", type=float, default=0.02,
                        help="按多少秒后的预测枪口角判开火对齐；0 = 用当前枪口")
    parser.add_argument("--truth-log", default="",
                        help="把 60 Hz 真值装甲位姿写到该文件（逐发归因用）")
    parser.add_argument("--min-settle", type=float, default=0.15,
                        help="换板后稳定多久才允许开火（秒）；0=不限制")
    parser.add_argument("--max-fire-distance", type=float, default=0.0,
                        help=">0 时只在目标近于该距离（米）时开火")
    parser.add_argument("--no-video", action="store_true")
    parser.add_argument("--push", action="store_true",
                        help="仿真连续推流（放 push.request），不再请求-应答；"
                             "取帧从 ~40 ms 降到 ~10 ms，视觉节拍由推理耗时决定")
    parser.add_argument("--diagnose", action="store_true", help="打印一次状态后退出")
    args = parser.parse_args()

    channel = Channel(args.channel, args.channel_root)
    channel.push = bool(args.push)
    print(f"仿真邮箱：{channel.directory}", flush=True)

    if args.diagnose:
        state = channel.read_state()
        print(json.dumps({key: state[key] for key in
                          ("session_id", "publisher_id", "gimbal", "weapon", "health", "stats", "ipc")
                          if key in state}, indent=2, ensure_ascii=False))
        return 0

    vision_frames = None
    if not args.no_video:
        vision_frames = Frames(channel, args.video_port, args.bind)
        threading.Thread(target=vision_frames.serve, daemon=True).start()

    if args.backend == "vision":
        if vision_frames is None:
            print("视觉后端需要画面，不能和 --no-video 一起用", flush=True)
            return 2
        for candidate in (Path(__file__).resolve().parent / "rm_aim_sim",
                          Path(__file__).resolve().parent):
            sys.path.insert(0, str(candidate))
        try:
            from vision_backend import VisionBackend
        except ImportError as exc:
            print(f"视觉后端不可用（缺少 vision_backend / openvino / cv2）：{exc}", flush=True)
            return 2
        model = args.model or {
            "rp0526": "D:/HNU_RM/HNU_NHS_Vision/assets/models/rp0526.onnx",
            "yolo26": "D:/HNU_RM/HNU_NHS_Vision/assets/models/yolo26.onnx",
            "ideal": "",
        }[args.detector]
        if model and not Path(model).exists():
            print(f"模型不存在：{model}（用 --model 指定）", flush=True)
            return 2
        backend = VisionBackend(model, detector=args.detector, confidence=args.confidence,
                                device=args.device, alpha=args.alpha, beta=args.beta,
                                roi=not args.no_roi, roi_margin=args.roi_margin,
                                corner_offset=args.corner_offset, use_body=args.use_body,
                                motion_window=args.motion_window,
                                gate={"enabled": True, "min_consecutive": 2})
        vision_frames.state_source = bool(args.state_source)
        if args.vision_latency is not None:
            vision_frames.vision_latency = float(args.vision_latency)
        vision_frames.attach(backend, args.vision_hz)
        print(f"视觉后端：{args.detector} · {model} · {args.vision_hz:.0f} Hz", flush=True)
    else:
        vision_frames = None if args.no_video else vision_frames

    operator = Operator(channel, args.operator_port, args.bind, math.radians(args.tolerance_deg),
                        feedforward=not args.no_feedforward,
                        rate_tolerance=args.rate_tol,
                        lead_window=args.lead_window,
                        release_latency=args.release_latency,
                        frames=vision_frames if args.backend == "vision" else None,
                        search_amplitude=args.search_amplitude,
                        search_rate=args.search_rate,
                        coast_seconds=args.coast_seconds,
                        lead_consume=args.lead_consume and not args.no_lead_consume,
                        yaw_offset=args.yaw_offset,
                        pitch_offset=args.pitch_offset)
    operator.calibrate_fire = bool(args.calibrate_fire)
    operator.max_fire_gap = float(args.max_fire_gap)
    operator.muzzle_lag = float(args.muzzle_lag)
    operator.min_settle = float(args.min_settle)
    operator.max_fire_distance = float(args.max_fire_distance)
    if args.truth_log and args.backend == "vision":
        try:
            operator.truth_log = open(args.truth_log, "w", encoding="utf-8")
        except OSError:
            operator.truth_log = None
    if args.fire_log:
        operator.fire_log = args.fire_log
        try:
            Path(args.fire_log).unlink()
        except OSError:
            pass
    if args.send_log:
        operator.send_log = []
        send_path = str(Path(args.channel_root) / "rm_aim_sim_" + args.channel / "sends.jsonl") \
            if False else "F:/bevy_robomaster_simulator/send_log.jsonl"

        def flush_sends():
            while True:
                time.sleep(0.5)
                if operator.send_log:
                    with open(send_path, "a", encoding="utf-8") as handle:
                        for row in operator.send_log:
                            handle.write(json.dumps({"t": round(row[0], 4), "fire": row[1],
                                                     "yaw": round(row[2], 5)}) + "\n")
                    operator.send_log.clear()

        threading.Thread(target=flush_sends, daemon=True).start()
    threads = [threading.Thread(target=operator.serve, daemon=True),
               threading.Thread(target=operator.control_loop, daemon=True)]
    for thread in threads:
        thread.start()
    print("桥接就绪（Ctrl+C 退出）", flush=True)
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    operator.stop_requested = True
    try:
        channel.send(False, False, 0.0, 0.0)
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())

