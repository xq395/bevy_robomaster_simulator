"""RMU2026 自定义客户端链路：MQTT 收下行、发上行，外加 3334 图传接收。

安全约定（和现有模拟器租约一致）：
  - 键鼠输入由浏览器心跳刷新；超过 INPUT_TTL_S 没刷新，立即改发全零帧，
    避免断网/切标签后机器人一直按着 W 或左键。
  - 只有显式“接管”后才发 KeyboardMouseControl；未接管时一帧都不发，
    避免和官方选手端抢键鼠来源（手册 2.2.1：同时只能有一个来源）。
  - 上行指令按手册限频：触发式指令 ≤10Hz；MapClickCmd 间隔 ≥0.5s（云台手）/ ≥3s（半自动操作手）。
"""
from __future__ import annotations

import os
import threading
import time

from google.protobuf.json_format import MessageToDict
from google.protobuf.message import DecodeError

from . import rm_custom_client_pb2 as pb
from .video import VideoReceiver
from .custom_video import MAGIC as _CUSTOM_VIDEO_MAGIC, Receiver as CustomVideoReceiver

CUSTOM_VIDEO_MAGIC = bytes([_CUSTOM_VIDEO_MAGIC])

try:
    import paho.mqtt.client as mqtt
except ImportError:  # 允许无 paho 时仍能导入（例如只跑单元测试的一部分）
    mqtt = None

DOWNLINK = (
    "GameStatus", "GlobalUnitStatus", "GlobalLogisticsStatus", "GlobalSpecialMechanism", "Event",
    "RobotInjuryStat", "RobotRespawnStatus", "RobotStaticStatus", "RobotDynamicStatus", "RobotModuleStatus",
    "RobotPosition", "Buff", "PenaltyInfo", "RobotPathPlanInfo", "MapClickInfo", "RadarInfoToClient",
    "CustomByteBlock", "TechCoreMotionStateSync", "RobotPerformanceSelectionSync", "DeployModeStatusSync",
    "RuneStatusSync", "SentryStatusSync", "DartSelectTargetStatusSync", "SentryCtrlResult", "AirSupportStatusSync",
)
# 触发式、≤10Hz 的上行指令
COMMANDS = {
    "CommonCommand", "AssemblyCommand", "RobotPerformanceSelectionCommand", "HeroDeployModeEventCommand",
    "RuneActivateCommand", "DartCommand", "SentryCtrlCommand", "AirSupportCommand",
}
COMMAND_INTERVAL_S = 0.1
MAP_CLICK_INTERVAL_S = {"gimbal": 0.5, "semi_auto": 3.0}
KEYBOARD_HZ = 75
INPUT_TTL_S = 0.25
EVENT_LOG = 30

# 键盘位掩码（手册 2.2.1）
KEY_BITS = {"KeyW": 0, "KeyS": 1, "KeyA": 2, "KeyD": 3, "ShiftLeft": 4, "ShiftRight": 4,
            "ControlLeft": 5, "ControlRight": 5, "KeyQ": 6, "KeyE": 7, "KeyR": 8, "KeyF": 9,
            "KeyG": 10, "KeyZ": 11, "KeyX": 12, "KeyC": 13, "KeyV": 14, "KeyB": 15}

VALID_ROBOT_IDS = set(range(1, 12)) | set(range(101, 112))


def keys_to_mask(codes) -> int:
    mask = 0
    for code in codes:
        bit = KEY_BITS.get(code)
        if bit is not None:
            mask |= 1 << bit
    return mask


def _clamp_int(value, low, high) -> int:
    if type(value) not in (int, float) or value != value:  # 拒绝 bool/NaN
        raise ValueError("not a number")
    return int(max(low, min(high, value)))


class RmuLink:
    def __init__(self, host: str = "192.168.12.1", port: int = 3333, robot_id: int = 3,
                 video_port: int | None = 3334, video_host: str = "0.0.0.0"):
        if robot_id not in VALID_ROBOT_IDS:
            raise ValueError(f"robot_id 必须是附录二中的编号，收到 {robot_id}")
        self.host, self.port, self.robot_id = host, port, robot_id
        self.lock = threading.Lock()
        self.latest: dict[str, dict] = {}
        self.latest_at: dict[str, float] = {}
        self.counts: dict[str, int] = {}
        self.events: list[dict] = []
        self.connected = False
        self.error = "未连接"
        self.decode_errors = 0
        # 键鼠输入
        self.driving = False
        self.input = dict(mask=0, mouse_x=0, mouse_y=0, mouse_z=0, left=False, right=False, mid=False)
        self.input_at = 0.0
        self.sent_keyboard = 0
        self.zero_frames = 0
        self.last_sent = None
        self.last_command_at: dict[str, float] = {}
        self.command_log: list[dict] = []
        self.client = None
        self.video = VideoReceiver(video_host, video_port) if video_port else None
        # 0x0310 自定义数据流里的低码率画面（机器人端发了才有；见 custom_video.py）
        self.custom_video = CustomVideoReceiver()
        self._stop = threading.Event()

    # ---------------- 生命周期 ----------------
    def start(self):
        if mqtt is None:
            with self.lock:
                self.error = "缺少 paho-mqtt，请 pip install paho-mqtt==2.1.0"
        else:
            # 手册：clientID 填所连接机器人对应的 ID 编号；MQTT 3.1.1，无 TLS 无账号
            # 本机 mock/sim 的 broker 对复用 clientID 的连接会丢消息（实测同 id 只收到
            # 25 条 vs 唯一 id 87 条），所以留一个覆盖口：RMU_CLIENT_ID=3-web。
            client_id = os.environ.get("RMU_CLIENT_ID") or str(self.robot_id)
            self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id,
                                      protocol=mqtt.MQTTv311, clean_session=True)
            self.client.on_connect = self._on_connect
            self.client.on_disconnect = self._on_disconnect
            self.client.on_message = self._on_message
            self.client.reconnect_delay_set(min_delay=1, max_delay=3)
            try:
                self.client.connect_async(self.host, self.port, keepalive=10)
            except (OSError, ValueError) as exc:
                with self.lock:
                    self.error = str(exc)
            self.client.loop_start()
            threading.Thread(target=self._keyboard_loop, name="rmu-keyboard", daemon=True).start()
        if self.video:
            self.video.start()
        return self

    def stop(self):
        self._stop.set()
        if self.client:
            if self.driving:
                self._publish_keyboard(zero=True)
            self.client.loop_stop()
            self.client.disconnect()
        if self.video:
            self.video.stop()

    # ---------------- MQTT 回调 ----------------
    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        ok = not reason_code.is_failure if hasattr(reason_code, "is_failure") else reason_code == 0
        with self.lock:
            self.connected = bool(ok)
            self.error = "" if ok else f"连接被拒绝：{reason_code}"
        if ok:
            client.subscribe([(topic, 1) for topic in DOWNLINK])

    def _on_disconnect(self, client, userdata, flags, reason_code, properties=None):
        with self.lock:
            self.connected = False
            self.error = f"已断开（{reason_code}），自动重连中"
            # 断线即撤销接管：重连后必须重新确认，不自动续发旧输入
            self.driving = False

    def _on_message(self, client, userdata, message):
        topic = message.topic
        cls = getattr(pb, topic, None)
        if topic not in DOWNLINK or cls is None:
            return
        try:
            msg = cls.FromString(message.payload)
        except DecodeError:
            with self.lock:
                self.decode_errors += 1
            return
        data = MessageToDict(msg, preserving_proto_field_name=True, always_print_fields_with_no_presence=True)
        if topic == "CustomByteBlock":
            if msg.data[:1] == CUSTOM_VIDEO_MAGIC:
                self.custom_video.push(msg.data)
            data = dict(size=len(msg.data), hex=msg.data[:64].hex())
        now = time.monotonic()
        with self.lock:
            self.latest[topic] = data
            self.latest_at[topic] = now
            self.counts[topic] = self.counts.get(topic, 0) + 1
            if topic in ("Event", "PenaltyInfo", "SentryCtrlResult", "MapClickInfo", "Buff"):
                self.events.append(dict(topic=topic, data=data, at=time.time()))
                del self.events[:-EVENT_LOG]

    # ---------------- 键鼠 ----------------
    def set_driving(self, on: bool):
        with self.lock:
            if on and not self.connected:
                raise ValueError("未连接赛事服务器")
            self.driving = bool(on)
            self.input = dict(mask=0, mouse_x=0, mouse_y=0, mouse_z=0, left=False, right=False, mid=False)
            self.input_at = time.monotonic() if on else 0.0
        if not on:
            self._publish_keyboard(zero=True)

    def update_input(self, data: dict):
        """浏览器心跳。键用 KeyboardEvent.code 列表；鼠标是本周期累计位移。"""
        keys = data.get("keys", [])
        if not isinstance(keys, list) or len(keys) > 32 or any(not isinstance(k, str) for k in keys):
            raise ValueError("invalid keys")
        buttons = data.get("buttons", {})
        if not isinstance(buttons, dict):
            raise ValueError("invalid buttons")
        for name in ("left", "right", "mid"):
            if type(buttons.get(name, False)) is not bool:
                raise ValueError("invalid button")
        parsed = dict(
            mask=keys_to_mask(keys),
            mouse_x=_clamp_int(data.get("mouse_x", 0), -32768, 32767),
            mouse_y=_clamp_int(data.get("mouse_y", 0), -32768, 32767),
            mouse_z=_clamp_int(data.get("mouse_z", 0), -32768, 32767),
            left=buttons.get("left", False), right=buttons.get("right", False), mid=buttons.get("mid", False),
        )
        with self.lock:
            if not self.driving:
                raise ValueError("未接管")
            # 鼠标位移在发送前累加，发送后清零，避免 75Hz 下重复计数
            parsed["mouse_x"] = _clamp_int(self.input["mouse_x"] + parsed["mouse_x"], -32768, 32767)
            parsed["mouse_y"] = _clamp_int(self.input["mouse_y"] + parsed["mouse_y"], -32768, 32767)
            parsed["mouse_z"] = _clamp_int(self.input["mouse_z"] + parsed["mouse_z"], -32768, 32767)
            self.input = parsed
            self.input_at = time.monotonic()

    def _build_keyboard(self, zero: bool):
        with self.lock:
            stale = time.monotonic() - self.input_at > INPUT_TTL_S
            if zero or stale:
                msg = pb.KeyboardMouseControl(mouse_x=0, mouse_y=0, mouse_z=0, left_button_down=False,
                                              right_button_down=False, keyboard_value=0, mid_button_down=False)
                is_zero = True
            else:
                i = self.input
                msg = pb.KeyboardMouseControl(mouse_x=i["mouse_x"], mouse_y=i["mouse_y"], mouse_z=i["mouse_z"],
                                              left_button_down=i["left"], right_button_down=i["right"],
                                              keyboard_value=i["mask"], mid_button_down=i["mid"])
                i["mouse_x"] = i["mouse_y"] = i["mouse_z"] = 0
                is_zero = not (msg.keyboard_value or msg.left_button_down or msg.right_button_down or msg.mid_button_down)
        return msg, is_zero

    def _publish_keyboard(self, zero: bool = False):
        if not self.client:
            return
        msg, is_zero = self._build_keyboard(zero)
        info = self.client.publish("KeyboardMouseControl", msg.SerializeToString(), qos=0)
        if info.rc == 0:
            with self.lock:
                self.sent_keyboard += 1
                self.zero_frames += 1 if is_zero else 0
                self.last_sent = dict(keyboard_value=msg.keyboard_value, mouse_x=msg.mouse_x, mouse_y=msg.mouse_y,
                                      left=msg.left_button_down, right=msg.right_button_down)

    def _keyboard_loop(self):
        interval = 1.0 / KEYBOARD_HZ
        next_at = time.monotonic()
        while not self._stop.is_set():
            with self.lock:
                driving = self.driving and self.connected
            if driving:
                self._publish_keyboard()
            next_at += interval
            delay = next_at - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_at = time.monotonic()

    # ---------------- 上行指令 ----------------
    def send_command(self, topic: str, fields: dict, role: str = "gimbal") -> str:
        if topic == "MapClickCmd":
            interval = MAP_CLICK_INTERVAL_S.get(role)
            if interval is None:
                raise ValueError("unknown role")
        elif topic == "CustomControl":
            interval = 0.0
        elif topic in COMMANDS:
            interval = COMMAND_INTERVAL_S
        else:
            raise ValueError("该 topic 不是上行指令")
        cls = getattr(pb, topic)
        msg = cls()
        allowed = {f.name: f for f in cls.DESCRIPTOR.fields}
        if not isinstance(fields, dict) or set(fields) - set(allowed):
            raise ValueError("未知字段")
        for name, value in fields.items():
            field = allowed[name]
            kind = field.type
            if kind == field.TYPE_BOOL:
                if type(value) is not bool:
                    raise ValueError(f"{name} 需要 bool")
            elif kind == field.TYPE_FLOAT:
                if type(value) not in (int, float) or value != value:
                    raise ValueError(f"{name} 需要数字")
                value = float(value)
            elif kind == field.TYPE_BYTES:
                if not isinstance(value, list) or any(type(b) is not int or not 0 <= b <= 255 for b in value):
                    raise ValueError(f"{name} 需要字节数组")
                value = bytes(value)
            elif kind in (field.TYPE_UINT32, field.TYPE_INT32):
                if type(value) is not int:
                    raise ValueError(f"{name} 需要整数")
                low = 0 if kind == field.TYPE_UINT32 else -(1 << 31)
                if not low <= value < (1 << 32 if kind == field.TYPE_UINT32 else 1 << 31):
                    raise ValueError(f"{name} 超出范围")
            setattr(msg, name, value)
        # 手册约束
        if topic == "CustomControl" and len(msg.data) > 30:
            raise ValueError("CustomControl 最大 30 字节")
        if topic == "MapClickCmd" and len(msg.robot_id) != 7:
            raise ValueError("MapClickCmd.robot_id 必须是 7 字节")
        if topic == "CommonCommand" and msg.cmd_type == 1 and msg.param % 10:
            raise ValueError("兑换 17mm 发弹量必须是 10 的倍数")
        now = time.monotonic()
        with self.lock:
            if not self.connected or not self.client:
                raise ValueError("未连接赛事服务器")
            last = self.last_command_at.get(topic, 0.0)
            if now - last < interval:
                raise RateLimited(round((interval - (now - last)) * 1000))
            self.last_command_at[topic] = now
        info = self.client.publish(topic, msg.SerializeToString(), qos=1)
        result = "已发送" if info.rc == 0 else f"发送失败 rc={info.rc}"
        with self.lock:
            self.command_log.append(dict(topic=topic, fields=fields, result=result, at=time.time()))
            del self.command_log[:-EVENT_LOG]
        return result

    # ---------------- 状态 ----------------
    def snapshot(self) -> dict:
        now = time.monotonic()
        with self.lock:
            data = dict(
                host=self.host, port=self.port, robot_id=self.robot_id, connected=self.connected, error=self.error,
                driving=self.driving, input_age_ms=round((now - self.input_at) * 1000) if self.input_at else None,
                sent_keyboard=self.sent_keyboard, zero_frames=self.zero_frames, last_sent=self.last_sent,
                decode_errors=self.decode_errors,
                messages={k: dict(data=v, age_ms=round((now - self.latest_at[k]) * 1000), count=self.counts[k])
                          for k, v in self.latest.items()},
                events=list(self.events[-10:]), commands=list(self.command_log[-10:]),
            )
        data["video"] = self.video.snapshot() if self.video else None
        data["custom_video"] = self.custom_video.snapshot()
        return data


class RateLimited(ValueError):
    def __init__(self, wait_ms: int):
        super().__init__(f"限频，{wait_ms}ms 后再试")
        self.wait_ms = wait_ms
