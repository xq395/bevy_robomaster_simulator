#!/usr/bin/env python3
"""Windows-native bridge from Daedalus/Talos SHM to deploy_dev assets.

This is the runnable Windows path when deploy_dev's C++ OpenCV SDK is not
installed. It mirrors deploy_dev's Hebei detector decoding, PnP armor geometry,
iterative ballistic solution, and simulator-boundary safety checks. It does not
replace deploy_dev's C++ EKF/MPC implementation.
"""
from __future__ import annotations

import argparse
import math
import mmap
import os
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml
from openvino import Core


IMAGE_W, IMAGE_H, IMAGE_CHANNELS = 1440, 1080, 3
IMAGE_SIZE = IMAGE_W * IMAGE_H * IMAGE_CHANNELS
IMAGE_POOL_SIZE = IMAGE_SIZE * 3
META_SIZE = 3712
SHM_MAGIC, SHM_VERSION = 0x54414C05, 2
FLAG_NEW, INDEX_MASK = 0x80, 0x03

IMAGE_TB_OFFSET = 64
IMAGE_SLOT_OFFSET = IMAGE_TB_OFFSET + 64
POSE_TB_OFFSETS = (256, 512, 768, 1024)  # gimbal, odom, muzzle, camera
GIMBAL_CMD_TB_OFFSET = 1536
CAMERA_INFO_OFFSET = 1728
RUNTIME_STATE_OFFSET = 3648

# OpenCV camera coordinates (right, down, forward) -> Talos/SP gimbal coordinates
# (forward, left, up). This is the idealized form of deploy_dev R_camera2gimbal.
R_CV_TO_GIMBAL = np.array(
    [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=np.float64
)


@dataclass(frozen=True)
class Pose:
    frame_seq: int
    position: np.ndarray
    quaternion_wxyz: np.ndarray
    timestamp_ns: int


@dataclass(frozen=True)
class CameraInfo:
    timestamp_ns: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: np.ndarray
    width: int
    height: int

    @property
    def matrix(self) -> np.ndarray:
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )


@dataclass(frozen=True)
class FrameBundle:
    image_bgr: np.ndarray
    frame_seq: int
    timestamp_ns: int
    gimbal: Pose
    odom: Pose
    muzzle: Pose
    camera: Pose
    camera_info: CameraInfo


@dataclass(frozen=True)
class PnpResult:
    detection: dict[str, Any]
    world_position: np.ndarray
    distance_m: float
    reprojection_error_px: float


@dataclass(frozen=True)
class AimResult:
    yaw_rad: float
    pitch_rad: float
    distance_m: float
    fly_time_s: float
    world_point: np.ndarray


def _read_pose(meta: mmap.mmap, tb_offset: int, state: int) -> Pose:
    slot_offset = tb_offset + 64 + (state & INDEX_MASK) * 64
    frame_seq = struct.unpack_from("<Q", meta, slot_offset)[0]
    position = np.array(struct.unpack_from("<3f", meta, slot_offset + 8), dtype=np.float64)
    # talos-ipc writes [w, x, y, z], although ROS message APIs commonly expose xyzw.
    quaternion = np.array(struct.unpack_from("<4f", meta, slot_offset + 20), dtype=np.float64)
    timestamp_ns = struct.unpack_from("<Q", meta, slot_offset + 40)[0]
    return Pose(frame_seq, position, quaternion, timestamp_ns)


def quaternion_matrix_wxyz(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    norm = float(np.linalg.norm(q))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid Talos gimbal quaternion")
    w, x, y, z = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def yaw_pitch_from_rotation(rotation: np.ndarray) -> tuple[float, float]:
    forward = rotation[:, 0]
    return math.atan2(float(forward[1]), float(forward[0])), math.atan2(
        float(forward[2]), math.hypot(float(forward[0]), float(forward[1]))
    )


class TalosShm:
    """Single-consumer Talos v2 transport with the actual triple-buffer handshake."""

    def __init__(self, shm_dir: Path):
        self.shm_dir = shm_dir
        meta_path = shm_dir / "talos_ipc_meta"
        image_path = shm_dir / "talos_ipc_image_pool"
        if not meta_path.exists() or not image_path.exists():
            raise FileNotFoundError(
                f"Talos SHM is missing under {shm_dir}. Start daedalus first with "
                f"TALOS_SHM_DIR={shm_dir}"
            )
        self._meta_file = open(meta_path, "r+b", buffering=0)
        self._image_file = open(image_path, "r+b", buffering=0)
        self.meta = mmap.mmap(self._meta_file.fileno(), META_SIZE, access=mmap.ACCESS_WRITE)
        self.image_pool = mmap.mmap(
            self._image_file.fileno(), IMAGE_POOL_SIZE, access=mmap.ACCESS_READ
        )
        magic, version = struct.unpack_from("<II", self.meta, 0)
        width, height = struct.unpack_from("<II", self.meta, 24)
        if magic != SHM_MAGIC:
            raise RuntimeError(f"bad Talos magic {magic:#x}; expected {SHM_MAGIC:#x}")
        if version != SHM_VERSION:
            raise RuntimeError(f"unsupported Talos version {version}; expected {SHM_VERSION}")
        if (width, height) != (IMAGE_W, IMAGE_H):
            raise RuntimeError(f"unsupported Talos image size {width}x{height}")

    def close(self) -> None:
        self.meta.close()
        self.image_pool.close()
        self._meta_file.close()
        self._image_file.close()

    def __enter__(self) -> "TalosShm":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def camera_info(self) -> CameraInfo:
        values = struct.unpack_from("<Q4d5dII24x", self.meta, CAMERA_INFO_OFFSET)
        return CameraInfo(
            values[0], values[1], values[2], values[3], values[4],
            np.array(values[5:10], dtype=np.float64), values[10], values[11]
        )

    def auto_aim_enabled(self) -> bool:
        return self.meta[RUNTIME_STATE_OFFSET + 8] == 1

    def heartbeat_age_s(self) -> float:
        heartbeat_ns = struct.unpack_from("<Q", self.meta, 16)[0]
        return max(0.0, (time.time_ns() - heartbeat_ns) / 1e9)

    def _release(self, tb_offset: int, observed_state: int) -> bool:
        # A synchronized producer waits for all four poses and the image. With one
        # Windows consumer, checking the observed state before the byte-sized swap
        # is sufficient and avoids the old client's deadlock after the first frame.
        if self.meta[tb_offset] != observed_state:
            return False
        ready_idx = observed_state & INDEX_MASK
        previous_read_idx = self.meta[tb_offset + 2] & INDEX_MASK
        self.meta[tb_offset + 2] = ready_idx
        self.meta[tb_offset] = previous_read_idx
        return True

    def read_bundle(self) -> FrameBundle | None:
        image_state = self.meta[IMAGE_TB_OFFSET]
        if (image_state & FLAG_NEW) == 0:
            return None
        pose_states = tuple(self.meta[offset] for offset in POSE_TB_OFFSETS)
        if any((state & FLAG_NEW) == 0 for state in pose_states):
            return None

        image_slot = IMAGE_SLOT_OFFSET + (image_state & INDEX_MASK) * 32
        seq, timestamp_ns, width, height = struct.unpack_from("<QQII", self.meta, image_slot)
        buffer_id, image_format = struct.unpack_from("<BB", self.meta, image_slot + 24)
        if seq == 0 or (width, height) != (IMAGE_W, IMAGE_H) or buffer_id >= 3:
            return None
        if image_format != 0:
            raise RuntimeError(f"unsupported Talos image format {image_format}; expected RGB8=0")

        poses = tuple(
            _read_pose(self.meta, offset, state)
            for offset, state in zip(POSE_TB_OFFSETS, pose_states)
        )
        if any(pose.frame_seq != seq or pose.timestamp_ns != timestamp_ns for pose in poses):
            raise RuntimeError("non-coherent Talos image/pose bundle; refusing to aim from it")

        start = buffer_id * IMAGE_SIZE
        rgb = np.frombuffer(
            self.image_pool[start : start + IMAGE_SIZE], dtype=np.uint8
        ).reshape(IMAGE_H, IMAGE_W, IMAGE_CHANNELS)
        image_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        # Release poses first and image last. image_bgr already owns its pixels.
        for offset, state in zip(POSE_TB_OFFSETS, pose_states):
            if not self._release(offset, state):
                raise RuntimeError("Talos pose state changed during consumption")
        if not self._release(IMAGE_TB_OFFSET, image_state):
            raise RuntimeError("Talos image state changed during consumption")

        info = self.camera_info()
        if (info.width, info.height) != (IMAGE_W, IMAGE_H) or info.fx <= 0 or info.fy <= 0:
            raise RuntimeError("Talos camera_info is not initialized")
        return FrameBundle(image_bgr, seq, timestamp_ns, *poses, info)

    def write_gimbal_command(
        self, yaw_deg: float, pitch_deg: float, distance_m: float, fire: bool
    ) -> None:
        if not all(math.isfinite(v) for v in (yaw_deg, pitch_deg, distance_m)) or distance_m <= 0:
            raise ValueError("refusing a non-finite or non-positive gimbal command")
        tb = GIMBAL_CMD_TB_OFFSET
        write_idx = self.meta[tb + 1] & INDEX_MASK
        if write_idx >= 3:
            raise RuntimeError("invalid Talos gimbal write index")
        slot = tb + 64 + write_idx * 32
        struct.pack_into(
            "<QfffB11x", self.meta, slot, time.time_ns(), float(yaw_deg),
            float(pitch_deg), float(distance_m), int(fire)
        )
        old_state = self.meta[tb]
        self.meta[tb] = write_idx | FLAG_NEW
        self.meta[tb + 1] = old_state & INDEX_MASK

    def gimbal_command_pending(self) -> bool:
        return (self.meta[GIMBAL_CMD_TB_OFFSET] & FLAG_NEW) != 0


class HebeiDetector:
    """Python execution of deploy_dev's Yolo26Detector contract."""

    def __init__(self, model_path: Path, confidence: float, device: str):
        core = Core()
        model = core.read_model(str(model_path))
        input_shape = tuple(int(v) for v in model.input().shape)
        output_shape = tuple(int(v) for v in model.output().shape)
        if input_shape != (1, 3, 640, 640):
            raise RuntimeError(f"Hebei input must be [1,3,640,640], got {input_shape}")
        if output_shape != (1, 30, 18):
            raise RuntimeError(f"Hebei output must be [1,30,18], got {output_shape}")
        self.compiled = core.compile_model(model, device)
        self.output = self.compiled.output(0)
        self.confidence = confidence

    def detect(self, bgr: np.ndarray) -> list[dict[str, Any]]:
        height, width = bgr.shape[:2]
        scale = min(640.0 / width, 640.0 / height)
        resized_width, resized_height = int(width * scale), int(height * scale)
        pad_x, pad_y = (640 - resized_width) // 2, (640 - resized_height) // 2
        resized = cv2.resize(bgr, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((640, 640, 3), 114, dtype=np.uint8)
        canvas[pad_y : pad_y + resized_height, pad_x : pad_x + resized_width] = resized
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        tensor = np.ascontiguousarray(np.transpose(rgb, (2, 0, 1))[None, ...])
        output = np.asarray(self.compiled([tensor])[self.output]).reshape(30, 18)

        detections: list[dict[str, Any]] = []
        for row in output:
            confidence = float(row[4])
            if confidence < self.confidence:
                continue
            class_id = int(np.rint(row[5]))
            color_id = int(np.argmax(row[6:10]))

            def unletterbox(x: float, y: float) -> tuple[float, float]:
                return float(np.clip((x - pad_x) / scale, 0, width - 1)), float(
                    np.clip((y - pad_y) / scale, 0, height - 1)
                )

            x1, y1 = unletterbox(float(row[0]), float(row[1]))
            x2, y2 = unletterbox(float(row[2]), float(row[3]))
            if x2 - x1 <= 1 or y2 - y1 <= 1:
                continue
            raw = [unletterbox(float(row[10 + i * 2]), float(row[11 + i * 2])) for i in range(4)]
            # Model: TL, BL, BR, TR. deploy_dev Solver: TL, TR, BR, BL.
            points = np.array([raw[0], raw[3], raw[2], raw[1]], dtype=np.float64)
            detections.append(
                {"confidence": confidence, "class_id": class_id, "color_id": color_id,
                 "large": class_id >= 6, "box": (x1, y1, x2, y2), "points": points}
            )
        return detections


def solve_pnp(detection: dict[str, Any], bundle: FrameBundle) -> PnpResult | None:
    width, height = (0.230 if detection["large"] else 0.135), 0.056
    object_points = np.array(
        [[0, width / 2, height / 2], [0, -width / 2, height / 2],
         [0, -width / 2, -height / 2], [0, width / 2, -height / 2]], dtype=np.float64
    )
    try:
        success, rvec, tvec = cv2.solvePnP(
            object_points, detection["points"], bundle.camera_info.matrix,
            bundle.camera_info.distortion, flags=cv2.SOLVEPNP_IPPE
        )
    except cv2.error:
        return None
    if not success or not np.all(np.isfinite(tvec)):
        return None
    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, bundle.camera_info.matrix, bundle.camera_info.distortion
    )
    errors = projected.reshape(4, 2) - detection["points"]
    reprojection_error = float(math.sqrt(float(np.mean(np.sum(errors * errors, axis=1)))))
    position_gimbal = R_CV_TO_GIMBAL @ tvec.reshape(3) + bundle.camera.position
    world_from_gimbal = quaternion_matrix_wxyz(bundle.gimbal.quaternion_wxyz)
    position_world = bundle.odom.position + world_from_gimbal @ position_gimbal
    return PnpResult(detection, position_world, float(np.linalg.norm(position_gimbal)), reprojection_error)


class AlphaBetaTracker:
    """Small fallback tracker; deploy_dev's full EKF remains in its C++ target."""

    def __init__(self, max_dt: float, min_hits: int):
        self.max_dt, self.min_hits = max_dt, min_hits
        self.position: np.ndarray | None = None
        self.velocity = np.zeros(3, dtype=np.float64)
        self.timestamp_s = 0.0
        self.class_id: int | None = None
        self.hits = 0

    @property
    def tracking(self) -> bool:
        return self.position is not None and self.hits >= self.min_hits

    def update(self, result: PnpResult, timestamp_ns: int) -> None:
        timestamp_s = timestamp_ns / 1e9
        class_id = int(result.detection["class_id"])
        dt = timestamp_s - self.timestamp_s
        if self.position is None or self.class_id != class_id or dt <= 0 or dt > self.max_dt:
            self.position = result.world_position.copy()
            self.velocity.fill(0)
            self.timestamp_s, self.class_id, self.hits = timestamp_s, class_id, 1
            return
        predicted = self.position + self.velocity * dt
        residual = result.world_position - predicted
        self.position = predicted + 0.70 * residual
        self.velocity += (0.12 / dt) * residual
        self.timestamp_s = timestamp_s
        self.hits += 1


def solve_trajectory(speed: float, distance: float, height: float) -> tuple[float, float] | None:
    gravity = 9.7833
    if speed <= 0 or distance <= 1e-6:
        return None
    a = gravity * distance * distance / (2 * speed * speed)
    discriminant = distance * distance - 4 * a * (a + height)
    if discriminant < 0:
        return None
    candidates = []
    for tangent in ((distance + math.sqrt(discriminant)) / (2 * a),
                    (distance - math.sqrt(discriminant)) / (2 * a)):
        pitch = math.atan(tangent)
        fly_time = distance / (speed * math.cos(pitch))
        if math.isfinite(pitch) and math.isfinite(fly_time):
            candidates.append((fly_time, pitch))
    if not candidates:
        return None
    fly_time, pitch = min(candidates)
    return pitch, fly_time


def calculate_aim(
    tracker: AlphaBetaTracker, bundle: FrameBundle, bullet_speed: float,
    yaw_offset_deg: float, pitch_offset_deg: float, delay_s: float
) -> AimResult | None:
    if tracker.position is None or bullet_speed < 14:
        return None
    fly_time, point_world, pitch = 0.0, tracker.position.copy(), 0.0
    for _ in range(10):
        point_world = tracker.position + tracker.velocity * (delay_s + fly_time)
        relative = point_world - bundle.odom.position
        trajectory = solve_trajectory(
            bullet_speed, math.hypot(float(relative[0]), float(relative[1])), float(relative[2])
        )
        if trajectory is None:
            return None
        pitch, next_fly_time = trajectory
        if abs(next_fly_time - fly_time) < 0.001:
            fly_time = next_fly_time
            break
        fly_time = next_fly_time
    relative = point_world - bundle.odom.position
    yaw = math.atan2(float(relative[1]), float(relative[0])) + math.radians(yaw_offset_deg)
    pitch += math.radians(pitch_offset_deg)
    return AimResult(yaw, pitch, float(np.linalg.norm(relative)), fly_time, point_world)


class FireGate:
    def __init__(self, config: dict[str, Any], explicitly_allowed: bool):
        self.enabled = bool(config.get("enabled", False)) and explicitly_allowed
        self.max_age = float(config.get("max_measurement_age", 0.05))
        self.max_reprojection = float(config.get("max_reprojection", 5.0))
        self.min_consecutive = int(config.get("min_consecutive", 3))
        self.min_yaw = math.radians(float(config.get("min_yaw_tolerance_deg", 0.2)))
        self.min_pitch = math.radians(float(config.get("min_pitch_tolerance_deg", 0.2)))
        self.ratio = float(config.get("armor_tolerance_ratio", 0.25))
        self.consecutive = 0

    def evaluate(
        self, aim: AimResult, pnp: PnpResult, bundle: FrameBundle,
        tracking: bool, measurement_age_s: float
    ) -> tuple[bool, str]:
        reason = None
        if not self.enabled:
            reason = "disabled"
        elif not tracking:
            reason = "not_tracking"
        elif measurement_age_s < 0 or measurement_age_s > self.max_age:
            reason = "stale_pose"
        elif pnp.reprojection_error_px > self.max_reprojection:
            reason = "bad_pnp"
        if reason:
            self.consecutive = 0
            return False, reason

        feedback_yaw, feedback_pitch = yaw_pitch_from_rotation(
            quaternion_matrix_wxyz(bundle.gimbal.quaternion_wxyz)
        )
        width = 0.230 if pnp.detection["large"] else 0.135
        yaw_tolerance = max(self.min_yaw, math.atan2(width * self.ratio, aim.distance_m))
        pitch_tolerance = max(self.min_pitch, math.atan2(0.056 * self.ratio, aim.distance_m))
        yaw_error = math.remainder(aim.yaw_rad - feedback_yaw, 2 * math.pi)
        pitch_error = aim.pitch_rad - feedback_pitch
        if abs(yaw_error) > yaw_tolerance or abs(pitch_error) > pitch_tolerance:
            self.consecutive = 0
            return False, "out_of_window"
        self.consecutive += 1
        if self.consecutive < self.min_consecutive:
            return False, "not_stable"
        return True, "allowed"


def load_settings(config_path: Path) -> dict[str, Any]:
    with open(config_path, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    model_path = Path(config.get("detector", {}).get("model_path", ""))
    if not model_path.is_absolute():
        model_path = (config_path.parent / model_path).resolve()
    config["_model_path"] = model_path
    return config


def select_target(
    detections: list[dict[str, Any]], bundle: FrameBundle,
    enemy_color: int, max_reprojection: float
) -> PnpResult | None:
    candidates = []
    for detection in detections:
        if detection["color_id"] != enemy_color:
            continue
        result = solve_pnp(detection, bundle)
        if result is not None and result.reprojection_error_px <= max_reprojection:
            candidates.append(result)
    return max(candidates, key=lambda r: float(r.detection["confidence"])) if candidates else None


def draw_overlay(image: np.ndarray, detections: list[dict[str, Any]], selected: PnpResult | None) -> None:
    for detection in detections:
        x1, y1, x2, y2 = (int(v) for v in detection["box"])
        chosen = selected is not None and detection is selected.detection
        color = (0, 255, 0) if chosen else (100, 100, 100)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cv2.putText(image, f"{detection['confidence']:.2f} c{detection['color_id']}",
                    (x1, max(15, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    color, 1, cv2.LINE_AA)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--deploy-dir", type=Path, default=Path(r"D:\HNU_RM\代码栈\deploy_dev"))
    parser.add_argument("--config", type=Path)
    parser.add_argument("--shm-dir", type=Path,
                        default=Path(os.environ.get("TALOS_SHM_DIR", r"F:\talos_shm")))
    parser.add_argument("--device", default="CPU")
    parser.add_argument("--send", action="store_true", help="write aim commands to the simulator")
    parser.add_argument("--allow-fire", action="store_true",
                        help="also requires fire_gate.enabled=true in deploy_dev config")
    parser.add_argument("--probe-hold", action="store_true",
                        help="with --send, send one fire=false command holding the current pose")
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--wait-seconds", type=float, default=0.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config_path = (args.config or args.deploy_dir / "config" / "sp_standard.yaml").resolve()
    config = load_settings(config_path)
    detector_config = config.get("detector", {})
    model_path: Path = config["_model_path"]
    detector = HebeiDetector(
        model_path, float(detector_config.get("conf_threshold", 0.25)), args.device
    )
    enemy_color = 1 if str(config.get("enemy_color", "blue")).lower() == "red" else 0
    max_reprojection = float(config.get("max_reprojection_error_px", 5.0))
    tracker = AlphaBetaTracker(
        float(config.get("max_tracker_dt", 0.1)), int(config.get("min_detect_count", 5))
    )
    fire_gate = FireGate(config.get("fire_gate", {}), args.allow_fire)

    print(f"[bridge] config={config_path}")
    print(f"[bridge] model={model_path}")
    print(f"[safety] send={args.send} allow_fire={args.allow_fire} "
          f"config_fire_gate={bool(config.get('fire_gate', {}).get('enabled', False))}")

    frames, last_seq = 0, 0
    command_sent = False
    start = wait_started = time.perf_counter()
    warned_auto_aim = False
    with TalosShm(args.shm_dir) as talos:
        print(f"[talos] v{SHM_VERSION} {IMAGE_W}x{IMAGE_H} dir={args.shm_dir} "
              f"heartbeat_age={talos.heartbeat_age_s():.3f}s "
              f"auto_aim={talos.auto_aim_enabled()}")
        try:
            while True:
                bundle = talos.read_bundle()
                if bundle is None or bundle.frame_seq == last_seq:
                    if args.wait_seconds > 0 and time.perf_counter() - wait_started >= args.wait_seconds:
                        print("[bridge] timed out waiting for a synchronized frame", file=sys.stderr)
                        return 3
                    time.sleep(0.001)
                    continue
                wait_started = time.perf_counter()
                last_seq, frames = bundle.frame_seq, frames + 1
                detections = detector.detect(bundle.image_bgr)
                selected = select_target(detections, bundle, enemy_color, max_reprojection)
                aim, fire, fire_reason = None, False, "no_target"
                if selected is not None:
                    tracker.update(selected, bundle.timestamp_ns)
                    aim = calculate_aim(
                        tracker, bundle, float(config.get("bullet_speed", 23.0)),
                        float(config.get("yaw_offset", 0.0)), float(config.get("pitch_offset", 0.0)),
                        float(config.get("low_speed_delay_time", 0.015))
                    )
                    if aim is not None:
                        age_s = max(0.0, (time.time_ns() - bundle.timestamp_ns) / 1e9)
                        fire, fire_reason = fire_gate.evaluate(
                            aim, selected, bundle, tracker.tracking, age_s
                        )
                        if args.send:
                            if not talos.auto_aim_enabled() and not warned_auto_aim:
                                print("[safety] simulator auto-aim is OFF; press F5 to accept commands")
                                warned_auto_aim = True
                            talos.write_gimbal_command(
                                math.degrees(aim.yaw_rad), math.degrees(aim.pitch_rad),
                                aim.distance_m, fire
                            )
                            command_sent = True

                if args.send and args.probe_hold and not command_sent:
                    if not talos.auto_aim_enabled() and not warned_auto_aim:
                        print("[safety] simulator auto-aim is OFF; press F5 to accept commands")
                        warned_auto_aim = True
                    hold_yaw, hold_pitch = yaw_pitch_from_rotation(
                        quaternion_matrix_wxyz(bundle.gimbal.quaternion_wxyz)
                    )
                    talos.write_gimbal_command(
                        math.degrees(hold_yaw), math.degrees(hold_pitch), 1.0, False
                    )
                    command_sent = True
                    print(f"[probe] hold yaw={math.degrees(hold_yaw):.2f} "
                          f"pitch={math.degrees(hold_pitch):.2f} fire=false")

                if frames == 1 or frames % 15 == 0:
                    fps = frames / max(time.perf_counter() - start, 1e-6)
                    message = f"seq={bundle.frame_seq} fps={fps:.1f} det={len(detections)} " \
                              f"track={tracker.hits}/{tracker.min_hits}"
                    if selected is not None:
                        message += f" conf={selected.detection['confidence']:.2f} " \
                                   f"pnp={selected.reprojection_error_px:.2f}px d={selected.distance_m:.2f}m"
                    if aim is not None:
                        message += f" yaw={math.degrees(aim.yaw_rad):.2f} " \
                                   f"pitch={math.degrees(aim.pitch_rad):.2f} fire={fire}({fire_reason})"
                    print(message, flush=True)

                if args.show:
                    view = bundle.image_bgr.copy()
                    draw_overlay(view, detections, selected)
                    cv2.imshow("deploy_dev <-> Talos", view)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                if args.max_frames > 0 and frames >= args.max_frames:
                    break
            if command_sent:
                deadline = time.perf_counter() + 0.5
                while talos.gimbal_command_pending() and time.perf_counter() < deadline:
                    time.sleep(0.005)
                print(f"[probe] command_consumed={not talos.gimbal_command_pending()}")
        except KeyboardInterrupt:
            pass
        finally:
            if args.show:
                cv2.destroyAllWindows()
    print(f"[bridge] stopped after {frames} frames")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
