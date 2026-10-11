"""RMU2026 Mock 赛事引擎：MQTT 3333 + UDP 3334 HEVC，用于脱离裁判系统开发自定义客户端。

用法：
    python -m rmu_protocol.mock_engine                 # 本机全套：broker + 数据 + 测试图
    python -m rmu_protocol.mock_engine --video-source http://127.0.0.1:5000/snapshot/raw
    python -m rmu_protocol.mock_engine --robot-id 103 --video-to 127.0.0.1:3334

这是开发工具，不模拟真实规则，只保证：
  - topic、字段、频率与手册 V2.0.0 一致；
  - 上行指令有可见反馈（兑换扣经济、复活、部署模式、哨兵回执等）；
  - 图传格式与手册一致（8 字节大端头、≤1400B、HEVC Annex-B）。
"""
from __future__ import annotations

import argparse
import io
import math
import random
import socket
import threading
import time
import urllib.request

import av
from PIL import Image, ImageDraw

from . import rm_custom_client_pb2 as pb
from .mini_broker import MiniBroker
from .video import hevc_encoder, packetize

# 己方 1,2,3,4,7 / 对方 1,2,3,4,7 —— GlobalUnitStatus.robot_health 的顺序
UNIT_ORDER = (1, 2, 3, 4, 7)
MAX_HP = {1: 350, 2: 250, 3: 200, 4: 200, 7: 400}


class GameModel:
    """只维护客户端能观察到的状态；数值为演示用，不代表真实规则。"""

    def __init__(self, robot_id: int):
        self.lock = threading.Lock()
        self.robot_id = robot_id
        self.blue = robot_id >= 100
        self.slot = robot_id % 100
        self.started = time.monotonic()
        self.economy = 400
        self.total_economy = 400
        self.health = {("ally", k): MAX_HP[k] for k in UNIT_ORDER} | {("enemy", k): MAX_HP[k] for k in UNIT_ORDER}
        self.ammo_allowed = 100
        self.fired = 0
        self.heat = 0.0
        self.deploy = 0
        self.rune = 1
        self.sentry_posture = 1
        self.dart_target, self.dart_open = 1, 0
        self.air_support = 0
        self.pending_respawn = False
        self.respawn_progress = 0
        self.perf_shooter, self.perf_chassis = 1, 1
        self.events: list[tuple[str, bytes]] = []
        self.last_keyboard: pb.KeyboardMouseControl | None = None
        self.last_keyboard_at = 0.0
        self.keyboard_count = 0
        self.custom_control_count = 0
        self.map_clicks = 0
        self.x, self.y, self.yaw = 3.0, 7.5, 90.0

    # ---- 上行处理 ----
    def handle(self, topic: str, payload: bytes):
        with self.lock:
            if topic == "KeyboardMouseControl":
                msg = pb.KeyboardMouseControl.FromString(payload)
                self.last_keyboard, self.last_keyboard_at = msg, time.monotonic()
                self.keyboard_count += 1
                keys = msg.keyboard_value
                speed = 0.05 * (2 if keys & (1 << 4) else 1)
                forward = (1 if keys & 1 else 0) - (1 if keys & 2 else 0)
                strafe = (1 if keys & 8 else 0) - (1 if keys & 4 else 0)
                self.yaw = (self.yaw + msg.mouse_x * 0.05) % 360
                rad = math.radians(self.yaw)
                self.x = min(28, max(0, self.x + speed * (forward * math.sin(rad) + strafe * math.cos(rad))))
                self.y = min(15, max(0, self.y + speed * (forward * math.cos(rad) - strafe * math.sin(rad))))
                if msg.left_button_down and self.ammo_allowed > 0 and self.heat < 200:
                    self.ammo_allowed -= 1
                    self.fired += 1
                    self.heat += 10
            elif topic == "CustomControl":
                self.custom_control_count += 1
            elif topic == "CommonCommand":
                msg = pb.CommonCommand.FromString(payload)
                if msg.cmd_type == 1 and msg.param and msg.param % 10 == 0 and self.economy >= msg.param:
                    self.economy -= msg.param
                    self.ammo_allowed += msg.param
                elif msg.cmd_type == 3 and self.pending_respawn and self.respawn_progress >= 10:
                    self._respawn()
                elif msg.cmd_type == 4 and self.pending_respawn and self.economy >= 100:
                    self.economy -= 100
                    self._respawn()
                elif msg.cmd_type == 6 and self.economy >= 50:
                    self.economy -= 50
                    key = ("ally", self.slot if self.slot in MAX_HP else 3)
                    self.health[key] = min(MAX_HP[key[1]], self.health[key] + 50)
            elif topic == "HeroDeployModeEventCommand":
                self.deploy = 1 if pb.HeroDeployModeEventCommand.FromString(payload).mode == 1 else 0
            elif topic == "RuneActivateCommand":
                if pb.RuneActivateCommand.FromString(payload).activate == 1 and self.rune == 1:
                    self.rune = 2
            elif topic == "SentryCtrlCommand":
                cmd = pb.SentryCtrlCommand.FromString(payload).command_id
                ok = cmd in (7, 8, 9)
                if ok:
                    self.sentry_posture = cmd - 6
                self.events.append(("SentryCtrlResult", pb.SentryCtrlResult(command_id=cmd, result_code=0 if ok else 1).SerializeToString()))
            elif topic == "DartCommand":
                msg = pb.DartCommand.FromString(payload)
                if msg.HasField("target_id"):
                    self.dart_target = msg.target_id
                if msg.HasField("open"):
                    self.dart_open = 2 if msg.open else 0
            elif topic == "AirSupportCommand":
                self.air_support = 1 if pb.AirSupportCommand.FromString(payload).command_id in (1, 2) else 0
            elif topic == "RobotPerformanceSelectionCommand":
                msg = pb.RobotPerformanceSelectionCommand.FromString(payload)
                self.perf_shooter = msg.shooter or self.perf_shooter
                self.perf_chassis = msg.chassis or self.perf_chassis
            elif topic == "MapClickCmd":
                msg = pb.MapClickCmd.FromString(payload)
                self.map_clicks += 1
                # 服务器把点击同步回己方客户端
                self.events.append(("MapClickInfo", pb.MapClickInfo(
                    is_send_all=msg.is_send_all, robot_id=msg.robot_id, mode=msg.mode, enemy_id=msg.enemy_id,
                    ascii=msg.ascii, type=msg.type, map_x=msg.map_x, map_y=msg.map_y).SerializeToString()))

    def _respawn(self):
        self.pending_respawn = False
        self.respawn_progress = 0
        key = ("ally", self.slot if self.slot in MAX_HP else 3)
        self.health[key] = MAX_HP[key[1]]

    # ---- 状态推进 ----
    def tick(self, dt: float):
        with self.lock:
            self.heat = max(0.0, self.heat - 40 * dt)
            key = ("ally", self.slot if self.slot in MAX_HP else 3)
            if self.pending_respawn:
                self.respawn_progress = min(10, self.respawn_progress + dt * 2)
            # 演示用随机伤害，便于观察 HUD 变化
            if random.random() < dt * 0.3:
                victim = random.choice(list(self.health))
                if self.health[victim] > 0:
                    self.health[victim] = max(0, self.health[victim] - 10)
                    if victim == key and self.health[victim] == 0:
                        self.pending_respawn = True
                        killer = (101 if not self.blue else 1)
                        self.events.append(("Event", pb.Event(event_id=1, param=f"{self.robot_id},{killer}").SerializeToString()))
            if random.random() < dt * 0.2:
                self.economy += 10
                self.total_economy += 10

    # ---- 下行消息 ----
    def elapsed(self) -> int:
        return int(time.monotonic() - self.started)

    def game_status(self):
        elapsed = self.elapsed() % 420
        return pb.GameStatus(current_round=1, total_rounds=3, red_score=0, blue_score=0, current_stage=4,
                             stage_countdown_sec=420 - elapsed, stage_elapsed_sec=elapsed, is_paused=False,
                             game_result=255, end_reason=255)

    def global_unit(self):
        with self.lock:
            hp = [self.health[("ally", k)] for k in UNIT_ORDER] + [self.health[("enemy", k)] for k in UNIT_ORDER]
            return pb.GlobalUnitStatus(
                base_health=5000, base_status=1, base_shield=500, outpost_health=1500, outpost_status=1,
                enemy_base_health=5000, enemy_base_status=1, enemy_base_shield=500,
                enemy_outpost_health=1500, enemy_outpost_status=1,
                robot_health=hp, robot_bullets=[self.ammo_allowed, 0, 0, 0, 0],
                total_damage_ally=sum(MAX_HP[k] - self.health[("enemy", k)] for k in UNIT_ORDER),
                total_damage_enemy=sum(MAX_HP[k] - self.health[("ally", k)] for k in UNIT_ORDER))

    def dynamic(self):
        with self.lock:
            key = ("ally", self.slot if self.slot in MAX_HP else 3)
            return pb.RobotDynamicStatus(
                current_health=self.health[key], current_heat=self.heat, last_projectile_fire_rate=23.5,
                current_chassis_energy=60000, current_buffer_energy=60, current_experience=0,
                experience_for_upgrade=100, total_projectiles_fired=self.fired, remaining_ammo=self.ammo_allowed,
                is_out_of_combat=False, out_of_combat_countdown=0, can_remote_heal=True, can_remote_ammo=True)

    def static(self):
        slot = self.slot if self.slot in MAX_HP else 3
        with self.lock:
            return pb.RobotStaticStatus(
                connection_state=1, field_state=0, alive_state=2 if self.pending_respawn else 1,
                robot_id=self.robot_id, robot_type=slot, performance_system_shooter=self.perf_shooter,
                performance_system_chassis=self.perf_chassis, level=1, max_health=MAX_HP[slot],
                max_heat=200, heat_cooldown_rate=40.0, max_power=100, max_buffer_energy=60, max_chassis_energy=60000)

    def one_hz(self):
        with self.lock:
            yield "GlobalLogisticsStatus", pb.GlobalLogisticsStatus(remaining_economy=self.economy, total_economy_obtained=self.total_economy, tech_level=1, encryption_level=1)
            yield "GlobalSpecialMechanism", pb.GlobalSpecialMechanism()
            yield "RobotInjuryStat", pb.RobotInjuryStat(total_damage=0)
            yield "RobotRespawnStatus", pb.RobotRespawnStatus(
                is_pending_respawn=self.pending_respawn, total_respawn_progress=10,
                current_respawn_progress=int(self.respawn_progress), can_free_respawn=self.respawn_progress >= 10,
                gold_cost_for_respawn=100, can_pay_for_respawn=self.economy >= 100)
            yield "RobotModuleStatus", pb.RobotModuleStatus(**{f.name: 1 for f in pb.RobotModuleStatus.DESCRIPTOR.fields})
            yield "RobotPosition", pb.RobotPosition(x=self.x, y=self.y, z=0.0, yaw=self.yaw, robot_id=self.robot_id)
            yield "TechCoreMotionStateSync", pb.TechCoreMotionStateSync(maximum_difficulty_level=1, basic_state=1)
            yield "RobotPerformanceSelectionSync", pb.RobotPerformanceSelectionSync(shooter=self.perf_shooter, chassis=self.perf_chassis)
            yield "DeployModeStatusSync", pb.DeployModeStatusSync(status=self.deploy)
            yield "RuneStatusSync", pb.RuneStatusSync(rune_status=self.rune, activated_arms=0, average_rings=0.0)
            yield "SentryStatusSync", pb.SentryStatusSync(posture_id=self.sentry_posture)
            yield "DartSelectTargetStatusSync", pb.DartSelectTargetStatusSync(target_id=self.dart_target, open=self.dart_open)
            yield "AirSupportStatusSync", pb.AirSupportStatusSync(airsupport_status=self.air_support, left_time=30, shooter_status=1)
        yield "RobotStaticStatus", self.static()
        yield "GlobalUnitStatus", self.global_unit()

    def drain_events(self):
        with self.lock:
            events, self.events = self.events, []
        return events

    def summary(self) -> dict:
        with self.lock:
            kb = self.last_keyboard
            return dict(keyboard_msgs=self.keyboard_count, custom_control_msgs=self.custom_control_count,
                        map_clicks=self.map_clicks, economy=self.economy, ammo=self.ammo_allowed,
                        keyboard_value=kb.keyboard_value if kb else None,
                        keyboard_age_ms=round((time.monotonic() - self.last_keyboard_at) * 1000) if kb else None,
                        position=(round(self.x, 2), round(self.y, 2), round(self.yaw, 1)))


class VideoSender:
    """把源画面编码成 HEVC 并按 3334 格式发出。源不可用时发测试图。"""

    def __init__(self, target: tuple[str, int], width: int, height: int, fps: int, kbps: int,
                 source: str | None, model: GameModel):
        self.target, self.width, self.height, self.fps, self.kbps = target, width, height, fps, kbps
        self.source, self.model = source, model
        self.sent_frames = 0
        self.sent_packets = 0
        self.source_ok = False
        self.error = ""
        self._stop = threading.Event()

    def _grab(self, index: int) -> Image.Image:
        if self.source:
            try:
                with urllib.request.urlopen(self.source, timeout=0.3) as r:
                    image = Image.open(io.BytesIO(r.read())).convert("RGB")
                self.source_ok = True
                return image.resize((self.width, self.height))
            except (OSError, ValueError):
                self.source_ok = False
        image = Image.new("RGB", (self.width, self.height), "#10202c")
        draw = ImageDraw.Draw(image)
        span = self.width - 160
        x = int((index * 6) % (2 * span))
        x = x if x < span else 2 * span - x
        draw.rectangle((x, self.height // 2 - 60, x + 160, self.height // 2 + 60), fill="#c0392b")
        summary = self.model.summary()
        draw.text((24, 24), f"RMU MOCK 3334  frame {index}", fill="white")
        draw.text((24, 48), f"keys=0x{(summary['keyboard_value'] or 0):04x} pos={summary['position']}", fill="#9fd")
        return image

    def run(self):
        try:
            self._run()
        except Exception as exc:  # noqa: BLE001 - 线程内异常要可见
            self.error = f"{type(exc).__name__}: {exc}"
            print(f"Mock 图传线程退出：{self.error}", flush=True)

    def _run(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            self._stream(sock)

    def _stream(self, sock: socket.socket):
        codec = hevc_encoder(self.width, self.height, self.fps, self.kbps)
        frame_id = 0
        interval = 1.0 / self.fps
        next_at = time.monotonic()
        index = 0
        while not self._stop.is_set():
            frame = av.VideoFrame.from_image(self._grab(index))
            frame.pts = index
            index += 1
            for packet in codec.encode(frame):
                for chunk in packetize(frame_id, bytes(packet)):
                    try:
                        sock.sendto(chunk, self.target)
                        self.sent_packets += 1
                    except OSError:
                        pass
                frame_id = (frame_id + 1) % 65536
                self.sent_frames += 1
            next_at += interval
            delay = next_at - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_at = time.monotonic()

    def stop(self):
        self._stop.set()


class MockEngine:
    def __init__(self, robot_id: int = 3, mqtt_host: str = "127.0.0.1", mqtt_port: int = 3333,
                 video_to: tuple[str, int] | None = ("127.0.0.1", 3334), video_source: str | None = None,
                 width: int = 1280, height: int = 720, fps: int = 30, kbps: int = 4000):
        self.model = GameModel(robot_id)
        self.broker = MiniBroker(mqtt_host, mqtt_port, on_message=self._on_message)
        self.video = VideoSender(video_to, width, height, fps, kbps, video_source, self.model) if video_to else None
        self._stop = threading.Event()

    def _on_message(self, client_id: str, topic: str, payload: bytes):
        try:
            self.model.handle(topic, payload)
        except Exception:  # noqa: BLE001 - 坏包不应让引擎挂掉
            pass

    def _publish(self, topic: str, message):
        data = message if isinstance(message, bytes) else message.SerializeToString()
        self.broker.publish(topic, data)

    def _loop(self):
        start = time.monotonic()
        tick = 0
        while not self._stop.is_set():
            tick += 1
            self.model.tick(0.1)
            self._publish("RobotDynamicStatus", self.model.dynamic())  # 10Hz
            if tick % 2 == 0:
                self._publish("GameStatus", self.model.game_status())  # 5Hz
            if tick % 10 == 0:
                for topic, msg in self.model.one_hz():  # 1Hz
                    self._publish(topic, msg)
                self._publish("CustomByteBlock", pb.CustomByteBlock(data=b"mock:" + str(tick).encode()))
            for topic, data in self.model.drain_events():
                self._publish(topic, data)
            time.sleep(max(0.0, start + tick * 0.1 - time.monotonic()))

    def start(self):
        self.broker.start()
        threading.Thread(target=self._loop, name="mock-loop", daemon=True).start()
        if self.video:
            threading.Thread(target=self.video.run, name="mock-video", daemon=True).start()
        return self

    def stop(self):
        self._stop.set()
        if self.video:
            self.video.stop()
        self.broker.stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-id", type=int, default=3, help="被模拟的机器人 ID（附录二）")
    parser.add_argument("--mqtt-host", default="127.0.0.1")
    parser.add_argument("--mqtt-port", type=int, default=3333)
    parser.add_argument("--video-to", default="127.0.0.1:3334", help="图传发往 host:port；填 none 关闭")
    parser.add_argument("--video-source", default=None, help="可选 JPEG 快照 URL，例如 http://127.0.0.1:5000/snapshot/raw")
    parser.add_argument("--size", default="1280x720")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--kbps", type=int, default=4000)
    args = parser.parse_args()
    video_to = None
    if args.video_to.lower() != "none":
        host, port = args.video_to.rsplit(":", 1)
        video_to = (host, int(port))
    width, height = (int(v) for v in args.size.lower().split("x"))
    engine = MockEngine(args.robot_id, args.mqtt_host, args.mqtt_port, video_to, args.video_source,
                        width, height, args.fps, args.kbps).start()
    print(f"RMU Mock 引擎：MQTT {args.mqtt_host}:{engine.broker.port}  图传 → {video_to}  机器人 {args.robot_id}", flush=True)
    try:
        while True:
            time.sleep(5)
            s = engine.model.summary()
            v = engine.video
            print(f"clients={engine.broker.clients()} kb={s['keyboard_msgs']} cc={s['custom_control_msgs']} "
                  f"click={s['map_clicks']} eco={s['economy']} ammo={s['ammo']}"
                  + (f" video_frames={v.sent_frames} src={'快照' if v.source_ok else '测试图'}" if v else ""), flush=True)
    except KeyboardInterrupt:
        engine.stop()


if __name__ == "__main__":
    main()
