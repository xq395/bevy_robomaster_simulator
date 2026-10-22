"""把 rm_aim_sim（Godot）接到 RMU 自定义客户端。

`mock_engine.py` 用合成数据扮演"赛场"；这里用同一套协议，但数据源换成仿真：

  * 裁判数据（血量/位置/朝向/开火数）从仿真 `state.json` 读，10 Hz 发 RobotDynamicStatus；
  * 键盘鼠标（KeyboardMouseControl）翻成 `command.json` 驱动仿真云台与开火；
  * 图传两路，共用一条采集线程（直接读仿真 IPC 的原始帧）：
      - 官方图传 UDP 3334：HEVC（有 NVIDIA 显卡用 hevc_nvenc）；
      - 自定义数据流 0x0310：300 B × 50 Hz 的低码率 H.264，经 MQTT CustomByteBlock 下发。

运行前提：仿真在跑（窗口不能最小化）。桥接 `rm_aim_sim_bridge.py` 只在需要模拟器通路
（UDP 5001 / HTTP 5000）时才要，RMU 通路不依赖它。
终端侧：`RMU_MQTT_PORT` 指到本进程的端口，`RMU_VIDEO_PORT` 指到 --video-to 的端口。

用法：
    python -m rmu_protocol.sim_engine --mqtt-port 3343 --video-to 127.0.0.1:3344
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import rm_aim_sim_bridge as bridge  # noqa: E402
from rmu_protocol import rm_custom_client_pb2 as pb  # noqa: E402
import av  # noqa: E402
import numpy  # noqa: E402

from rmu_protocol.mock_engine import (  # noqa: E402
    MAX_HP, FrameSource, GameModel, MockEngine, add_custom_video_args, custom_video_args)

# 鼠标增量 → 云台角：真实的 75 Hz 键鼠包 mouse_x 量级是个位到几十，
# 这里按 0.0012 rad/count 起步；联调时用 --mouse-yaw 调。
MOUSE_YAW = 0.0012
MOUSE_PITCH = 0.0008
PITCH_LIMIT = 0.52


class SimModel(GameModel):
    """把 GameModel 的状态推进换成"从仿真读"，控制换成"写进仿真"。"""

    def __init__(self, robot_id: int, channel: bridge.Channel, mouse_yaw: float = MOUSE_YAW,
                 mouse_pitch: float = MOUSE_PITCH):
        super().__init__(robot_id)
        self.channel = channel
        self.mouse_yaw = mouse_yaw
        self.mouse_pitch = mouse_pitch
        self.last_state = None
        self.state_errors = 0
        self.sent_commands = 0
        self.yaw_cmd = 0.0
        self.pitch_cmd = 0.0
        self.fire_cmd = False
        self.last_input_at = 0.0

    # ---- 状态推进：从仿真读 ----
    def tick(self, dt: float):
        try:
            state = self.channel.read_state(timeout=0.05)
        except (TimeoutError, OSError):
            self.state_errors += 1
            return
        self.last_state = state
        slot = self.slot if self.slot in MAX_HP else 3
        health = state.get("health") or {}
        maximum = max(1, int(health.get("maximum", 1)))
        current = int(health.get("current", 0))
        self.health[("ally", slot)] = 0 if not health.get("alive", True) else max(
            1, int(round(current / maximum * MAX_HP[slot])))
        stats = state.get("stats") or {}
        self.fired = int(stats.get("fired", self.fired))
        gimbal = state.get("gimbal") or {}
        self.yaw = math.degrees(float(gimbal.get("yaw", 0.0))) % 360.0
        origin = gimbal.get("origin") or [0.0, 0.0, 0.0]
        # rm_aim_sim 没有底盘：本机位置固定为云台原点（靶车的位置不是我方位置）
        self.x = float(origin[0]) if len(origin) > 0 else 0.0
        self.y = float(origin[1]) if len(origin) > 1 else 0.0
        # 仿真没有弹量/热量模型：用开火数反推一个可看的余量
        self.ammo_allowed = max(0, 100 - (self.fired % 100))
        self.heat = 0.0
        # 客户端 250ms 没发键鼠就松扳机（和 terminal 的零帧语义一致）
        if self.last_input_at and time.monotonic() - self.last_input_at > 0.25:
            self._send_command(0.0, 0.0, False)

    # ---- 上下行：键鼠 → 仿真云台/开火 ----
    def handle(self, topic: str, payload: bytes):
        if topic != "KeyboardMouseControl":
            super().handle(topic, payload)
            return
        message = pb.KeyboardMouseControl.FromString(payload)
        with self.lock:
            self.last_keyboard, self.last_keyboard_at = message, time.monotonic()
            self.keyboard_count += 1
            self.last_input_at = time.monotonic()
            self.yaw_cmd += message.mouse_x * self.mouse_yaw
            self.pitch_cmd = max(-PITCH_LIMIT, min(PITCH_LIMIT, self.pitch_cmd + message.mouse_y * self.mouse_pitch))
            self._send_command(self.yaw_cmd, self.pitch_cmd, bool(message.left_button_down))

    def _send_command(self, yaw: float, pitch: float, fire: bool):
        """写仿真 command.json：全手动，云台给绝对角 + 角速度前馈。"""
        try:
            self.channel.send(True, fire, yaw, pitch)
            self.sent_commands += 1
            self.fire_cmd = fire
        except (RuntimeError, OSError):
            self.state_errors += 1

    def summary(self) -> dict:
        data = super().summary()
        data["sim_commands"] = self.sent_commands
        data["sim_errors"] = self.state_errors
        return data


class SimFrameSource(FrameSource):
    """直接从仿真 IPC 取原始像素（rgba8 1440×1080），不经桥接的 HTTP/JPEG。

    旧路径是 桥接取帧 34 ms + JPEG 编码 44 ms + 这边 JPEG 解码/缩放，串行下来只有 4~10 fps；
    现在 取帧 / 缩放转 YUV / 编码 三段各在自己的线程，吞吐由最慢的取帧（~34 ms，仿真等下一个
    渲染帧再读回 6 MB）决定，约 25~30 fps。
    仿真窗口被最小化时不渲染，取帧超时会顺手把窗口恢复出来（仅 Windows）。
    """

    def __init__(self, width, height, fps, model, channel_name="default"):
        super().__init__(width, height, fps, None, model)
        self.channel = bridge.Channel(channel_name)
        self.capture_ms = 0.0
        self.convert_ms = 0.0
        self.timeouts = 0
        self.restores = 0
        self._last_restore = 0.0
        self._raw = None            # 取帧线程放进来的最新原始帧，转换线程取走
        self._raw_event = threading.Event()

    def run(self):
        """两级流水线：取帧（IPC 往返 ~34 ms）和缩放转 YUV（~13 ms）并行，吞吐取两者较大值。"""
        threading.Thread(target=self._convert_loop, name="sim-convert", daemon=True).start()
        interval = 1.0 / self.fps
        next_at = time.monotonic()
        while not self._stop.is_set():
            raw = self._fetch()
            if raw is not None:
                self._raw = raw
                self._raw_event.set()
            else:
                with self.lock:
                    self.frame = super().grab(self.index)
                    self.index += 1
                self.event.set()
            next_at += interval
            delay = next_at - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_at = time.monotonic()

    def _convert_loop(self):
        while not self._stop.is_set():
            if not self._raw_event.wait(0.2):
                continue
            self._raw_event.clear()
            raw, self._raw = self._raw, None
            if raw is None:
                continue
            started = time.monotonic()
            frame = self._convert(*raw)
            self.convert_ms = 0.8 * self.convert_ms + 0.2 * (time.monotonic() - started) * 1000
            with self.lock:
                self.frame = frame
                self.index += 1
            self.event.set()

    def _fetch(self):
        started = time.monotonic()
        try:
            if self.channel.state is None:
                self.channel.read_state(timeout=0.5)
            metadata, pixels = self.channel.receive_frame(self.channel.request_frame(), timeout=0.5)
        except (TimeoutError, OSError, ValueError, RuntimeError):
            self.timeouts += 1
            self.source_ok = False
            self._restore_window()
            return None
        camera = metadata.get("camera") or {}
        width, height = int(camera.get("width", 0)), int(camera.get("height", 0))
        channels = 4 if metadata.get("encoding") == "rgba8" else 3
        if width <= 0 or height <= 0 or len(pixels) != width * height * channels:
            self.source_ok = False
            return None
        self.capture_ms = 0.8 * self.capture_ms + 0.2 * (time.monotonic() - started) * 1000
        self.source_ok = True
        return pixels, width, height, channels

    def _convert(self, pixels, width, height, channels):
        array = numpy.frombuffer(pixels, dtype=numpy.uint8).reshape(height, width, channels)
        frame = av.VideoFrame.from_ndarray(array, format="rgba" if channels == 4 else "rgb24")
        return frame.reformat(width=self.width, height=self.height, format="yuv420p",
                              interpolation="FAST_BILINEAR")

    def _restore_window(self):
        now = time.monotonic()
        if sys.platform != "win32" or now - self._last_restore < 3.0:
            return
        self._last_restore = now
        try:
            import ctypes
            user32 = ctypes.windll.user32
            found = []

            @ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.c_void_p, ctypes.c_void_p)
            def visit(hwnd, _):
                buffer = ctypes.create_unicode_buffer(256)
                user32.GetWindowTextW(hwnd, buffer, 256)
                if user32.IsWindowVisible(hwnd) and ("Aim Lab" in buffer.value or "rm_aim_sim" in buffer.value):
                    found.append(hwnd)
                return True

            user32.EnumWindows(visit, 0)
            for hwnd in found:
                if user32.IsIconic(hwnd):
                    user32.ShowWindow(hwnd, 9)          # SW_RESTORE
                    self.restores += 1
                    print("仿真窗口被最小化，已恢复（最小化时不渲染，图传会中断）", flush=True)
        except Exception:  # noqa: BLE001 - 恢复失败只影响画面
            pass


class SimEngine(MockEngine):
    """MockEngine 的接线，只把数据源/控制/图传换成仿真。"""

    def __init__(self, robot_id: int = 3, mqtt_host: str = "127.0.0.1", mqtt_port: int = 3343,
                 video_to=("127.0.0.1", 3344), width: int = 1280, height: int = 720, fps: int = 30,
                 kbps: int = 4000, sim_channel: str = "default",
                 custom_video_size=(384, 288), custom_video_fps: int = 15, custom_video_kbps: int = 96):
        self.model = SimModel(robot_id, bridge.Channel(sim_channel))
        self.broker = bridge_channel_broker(mqtt_host, mqtt_port, self._on_message)
        frames = SimFrameSource(width, height, fps, self.model, sim_channel)
        self._setup_video(frames, video_to, kbps, custom_video_size, custom_video_fps, custom_video_kbps)
        self._stop = threading.Event()

    def start(self):
        return super().start()


def bridge_channel_broker(host: str, port: int, on_message):
    from rmu_protocol.mini_broker import MiniBroker
    return MiniBroker(host, port, on_message=on_message)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-id", type=int, default=3, help="被模拟的机器人 ID（附录二）")
    parser.add_argument("--mqtt-host", default="127.0.0.1")
    parser.add_argument("--mqtt-port", type=int, default=3343, help="终端要连到这里（RMU_MQTT_PORT）")
    parser.add_argument("--video-to", default="127.0.0.1:3344", help="图传发往 host:port；填 none 关闭")
    parser.add_argument("--size", default="960x720", help="仿真相机是 4:3，保持 4:3 不拉伸")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--kbps", type=int, default=4000)
    add_custom_video_args(parser)
    parser.add_argument("--sim-channel", default="default", help="仿真 IPC 通道名")
    parser.add_argument("--mouse-yaw", type=float, default=MOUSE_YAW, help="每单位 mouse_x 的云台 yaw 增量（rad）")
    parser.add_argument("--mouse-pitch", type=float, default=MOUSE_PITCH, help="每单位 mouse_y 的 pitch 增量（rad）")
    args = parser.parse_args()

    video_to = None
    if args.video_to.lower() != "none":
        host, port = args.video_to.rsplit(":", 1)
        video_to = (host, int(port))
    width, height = (int(v) for v in args.size.lower().split("x"))
    engine = SimEngine(args.robot_id, args.mqtt_host, args.mqtt_port, video_to,
                       width, height, args.fps, args.kbps, args.sim_channel, *custom_video_args(args)).start()
    engine.model.mouse_yaw = args.mouse_yaw
    engine.model.mouse_pitch = args.mouse_pitch
    print(f"RMU SIM 引擎：MQTT {args.mqtt_host}:{engine.broker.port}  图传 → {video_to}  "
          f"仿真通道 {args.sim_channel}  机器人 {args.robot_id}", flush=True)
    try:
        while True:
            time.sleep(5)
            s = engine.model.summary()
            v = engine.video
            print(f"clients={engine.broker.clients()} kb={s['keyboard_msgs']} "
                  f"cmds={s['sim_commands']} err={s['sim_errors']} "
                  f"pos=({s['position'][0]:.2f},{s['position'][1]:.2f})"
                  + (f" video={v.sent_frames}f enc={v.encoder_name}/{v.encode_ms:.1f}ms "
                     f"grab={engine.frames.capture_ms:.0f}ms timeouts={engine.frames.timeouts} "
                     f"src={'仿真' if v.source_ok else '测试图'}" if v else "")
                  + (f" custom={engine.custom_video.sender.summary()}" if engine.custom_video else ""), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        engine.stop()


if __name__ == "__main__":
    main()


