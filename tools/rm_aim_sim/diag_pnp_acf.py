"""PnP 残差的时间结构：对同一块真值板，逐帧打印 (PnP − truth) 的 y 误差。

平滑缓慢变化 → 检测偏差（窗平均无效，但可作为信号的一部分被跟踪）；
逐帧抖动 → 白噪声（窗平均有效，σ/√N）。
另打印自相关 lag1/lag4/lag8 判别。

用法：python tools/rm_aim_sim/diag_pnp_acf.py --seconds 8 --plate 0
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
    parser.add_argument("--hz", type=float, default=25.0)
    parser.add_argument("--plate", type=int, default=0)
    args = parser.parse_args()

    channel = bridge.Channel("default", timeout=5.0)
    channel.read_state()
    backend = VisionBackend("D:/HNU_RM/HNU_NHS_Vision/assets/models/rp0526.onnx",
                            detector="rp0526", confidence=0.5,
                            roi=False, use_body=False,
                            gate={"enabled": False})

    period = 1.0 / args.hz
    stop_at = time.monotonic() + args.seconds
    series = []          # (t, ex, ey, ez)
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
        truth_list = {a["id"]: np.asarray(a["center"], dtype=np.float64)
                      for a in (state.get("truth") or {}).get("armors") or []}
        if args.plate not in truth_list:
            continue
        bundle = backend._bundle(state, bgr, int(state.get("tick", 0)), time.time_ns())
        if bundle is None:
            continue
        detections, _ = backend._detect(bgr)
        import deploy_dev_talos_bridge as vision
        for detection in detections:
            result = vision.solve_pnp(detection, bundle)
            if result is None:
                continue
            pos = np.asarray(result.world_position, dtype=np.float64)
            # 只留离目标板最近的候选（同一块板的重复检测会有多个）
            d = float(np.linalg.norm(pos - truth_list[args.plate]))
            if d < 0.3:
                series.append((observe, *(pos - truth_list[args.plate])))
                break
        elapsed = time.monotonic() - started
        time.sleep(max(0.0, period - elapsed))

    print(f"板 {args.plate}：{len(series)} 帧")
    for t, ex, ey, ez in series:
        print(f"{t:9.3f}  ex {ex:+.4f}  ey {ey:+.4f}  ez {ez:+.4f}")
    if len(series) > 20:
        arr = np.asarray([s[1:] for s in series])
        for axis, name in enumerate("xyz"):
            column = arr[:, axis] - arr[:, axis].mean()
            denominator = float(np.dot(column, column))
            if denominator <= 0:
                continue
            acf = [float(np.dot(column[:-lag], column[lag:]) / denominator)
                   for lag in (1, 4, 8)]
            print(f"{name}: σ {column.std():.4f}  acf1 {acf[0]:+.2f}  "
                  f"acf4 {acf[1]:+.2f}  acf8 {acf[2]:+.2f}")


if __name__ == "__main__":
    sys.exit(main())
