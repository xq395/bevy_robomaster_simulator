"""逐帧诊断单板运动估计：世界位置、拟合 (v,a)、aim、aim_rate。

用法：python tools/rm_aim_sim/diag_motion.py --seconds 8
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
from vision_backend import VisionBackend


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--motion-window", type=float, default=0.25)
    parser.add_argument("--confidence", type=float, default=0.5)
    args = parser.parse_args()

    channel = bridge.Channel("default", timeout=5.0)
    channel.read_state()
    backend = VisionBackend("D:/HNU_RM/HNU_NHS_Vision/assets/models/rp0526.onnx",
                            detector="rp0526", confidence=args.confidence,
                            roi=False, use_body=False,
                            motion_window=args.motion_window,
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
        try:
            yaw, pitch, fire, info = backend.step(metadata, bgr, time.time_ns(),
                                                  delay_s=0.045, observe_time_s=observe)
        except Exception as exc:
            print(f"step 异常: {exc!r}", flush=True)
            continue
        motion = backend.motion
        velocity, acceleration = motion.estimate() if motion.history else (None, None)
        rows.append({
            "t": observe,
            "yaw": yaw, "aim_rate": backend.aim_rate,
            "v": None if velocity is None else [round(float(x), 3) for x in velocity],
            "a": None if acceleration is None else [round(float(x), 3) for x in acceleration],
            "world": info.get("world"),
            "hist": len(motion.history),
            "slots": info.get("slots"),
            "predicted_only": info.get("predicted_only"),
            "reason": info.get("reason"),
            "omega": round(motion.omega, 3),
        })
        elapsed = time.monotonic() - started
        time.sleep(max(0.0, period - elapsed))

    print(f"{'t':>9} {'yaw':>8} {'rate':>7} {'v':>26} {'a':>26} {'hist':>4} {'slot':>4} {'po':>2} {'reason'}")
    for row in rows:
        v = "" if row["v"] is None else f"({row['v'][0]:+.2f},{row['v'][1]:+.2f},{row['v'][2]:+.2f})"
        a = "" if row["a"] is None else f"({row['a'][0]:+.2f},{row['a'][1]:+.2f},{row['a'][2]:+.2f})"
        yaw = "" if row["yaw"] is None else f"{math.degrees(row['yaw']):+8.2f}"
        rate = "" if row["aim_rate"] is None else f"{row['aim_rate'][0]:+7.3f}"
        print(f"{row['t']:9.3f} {yaw} {rate} {v:>26} {a:>26} {row['hist']:>4} "
              f"{row['slots']:>4} {'Y' if row['predicted_only'] else '.':>2} {row['reason']}")
    solved = [r for r in rows if r["yaw"] is not None]
    rates = [r["aim_rate"][0] for r in solved if r["aim_rate"]]
    if rates:
        import statistics
        print(f"\n有解 {len(solved)}/{len(rows)}，aim_rate p50={statistics.median(rates):+.3f} "
              f"std={statistics.pstdev(rates):.3f}")


if __name__ == "__main__":
    sys.exit(main())
