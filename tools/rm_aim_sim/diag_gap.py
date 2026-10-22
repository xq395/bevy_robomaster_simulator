"""诊断 no_target 空档：逐帧打印原始检测数、PnP 候选数、槽位、真值 yaw vs 云台 yaw。

用法：python tools/rm_aim_sim/diag_gap.py --seconds 12
"""
import argparse
import math
import sys
import time
from pathlib import Path

for candidate in (Path(__file__).resolve().parent, Path(__file__).resolve().parent.parent):
    sys.path.insert(0, str(candidate))

import numpy as np
import rm_aim_sim_bridge as bridge
import deploy_dev_talos_bridge as vision
from vision_backend import VisionBackend


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=12.0)
    parser.add_argument("--hz", type=float, default=25.0)
    parser.add_argument("--confidence", type=float, default=0.5)
    args = parser.parse_args()

    channel = bridge.Channel("default", timeout=5.0)
    channel.read_state()
    backend = VisionBackend("D:/HNU_RM/HNU_NHS_Vision/assets/models/rp0526.onnx",
                            detector="rp0526", confidence=args.confidence,
                            roi=False, use_body=False,
                            gate={"enabled": True, "min_consecutive": 2})

    period = 1.0 / args.hz
    stop_at = time.monotonic() + args.seconds
    rows = []
    while time.monotonic() < stop_at:
        started = time.monotonic()
        try:
            identity = channel.request_frame()
            metadata, pixels = channel.receive_frame(identity, timeout=0.8)
        except (TimeoutError, OSError, ValueError):
            time.sleep(0.05)
            continue
        camera = metadata.get("camera") or {}
        width = int(camera.get("width", 0)); height = int(camera.get("height", 0))
        channels = 3 if metadata.get("encoding") == "rgb8" else 4
        array = np.frombuffer(pixels, dtype=np.uint8).reshape(height, width, channels)
        bgr = np.ascontiguousarray(array[:, :, 2::-1])
        observe = float(metadata.get("sim_time") or 0.0)
        state = channel.read_state(timeout=0.05)
        try:
            yaw, pitch, fire, info = backend.step(metadata, bgr, time.time_ns(),
                                                  delay_s=0.045, observe_time_s=observe)
        except Exception as exc:
            print(f"step 异常: {exc!r}", flush=True)
            continue
        truth = state.get("truth") or {}
        center = truth.get("center") or [0.0, 0.0, 0.0]
        truth_yaw = math.degrees(math.atan2(center[1], center[0]))
        gimbal = state.get("gimbal") or {}
        gq = gimbal.get("quaternion_wxyz") or [1.0, 0.0, 0.0, 0.0]
        gyaw, _ = vision.yaw_pitch_from_rotation(vision.quaternion_matrix_wxyz(gq))
        rows.append({
            "t": observe, "det": info.get("detections", 0), "sel": info.get("selected"),
            "slots": info.get("slots"), "po": info.get("predicted_only"),
            "reason": info.get("reason"),
            "ages": [round(s.age(observe), 2) for s in backend.slots],
            "hist": [len(s.motion.history) for s in backend.slots],
            "truth_yaw": truth_yaw,
            "gimbal_yaw": math.degrees(gyaw),
            "yaw": None if yaw is None else math.degrees(yaw),
            "conf": info.get("confidence"),
        })
        elapsed = time.monotonic() - started
        time.sleep(max(0.0, period - elapsed))

    print(f"{'t':>9} {'det':>3} {'sel':>3} {'slot':>4} {'po':>2} {'reason':>14} "
          f"{'ages':>18} {'hist':>10} {'true°':>7} {'gim°':>7} {'aim°':>7} {'conf':>5}")
    for row in rows:
        sel = "Y" if row["sel"] else "."
        yaw_text = "" if row["yaw"] is None else format(row["yaw"], "7.2f")
        conf_text = "" if row["conf"] is None else format(row["conf"], "5.2f")
        print(f"{row['t']:9.3f} {row['det']:>3} {sel:>3} {row['slots']:>4} "
              f"{'Y' if row['po'] else '.':>2} {str(row['reason']):>14} "
              f"{str(row['ages']):>18} {str(row['hist']):>10} "
              f"{row['truth_yaw']:7.2f} {row['gimbal_yaw']:7.2f} "
              f"{yaw_text} {conf_text}")


if __name__ == "__main__":
    sys.exit(main())
