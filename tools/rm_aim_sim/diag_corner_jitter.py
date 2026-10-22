"""量检测器 2D 角点抖动：对同一块真值板，逐帧打印图像四边形的质心/宽/高/角度。

若 2D 稳定而 3D 跳 → PnP/位姿变换问题；若 2D 本身跳 → 检测器问题。

用法：python tools/rm_aim_sim/diag_corner_jitter.py --seconds 8 --plate 0
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
        truth_list = {a["id"]: np.asarray(a["center"], dtype=np.float64)
                      for a in (state.get("truth") or {}).get("armors") or []}
        if args.plate not in truth_list:
            continue
        detections, _ = backend._detect(bgr)
        # 用"真值板投影到图像的近似位置"选检测：直接取置信度最高且宽高合理的
        best = None
        for detection in detections:
            points = np.asarray(detection["points"], dtype=np.float64)
            centre = points.mean(axis=0)
            width_px = float(np.linalg.norm(points[1] - points[0]))
            height_px = float(np.linalg.norm(points[3] - points[0]))
            angle = math.degrees(math.atan2(points[1][1] - points[0][1],
                                            points[1][0] - points[0][0]))
            if best is None or detection["confidence"] > best[0]:
                best = (detection["confidence"], centre, width_px, height_px, angle,
                        detection["class_id"])
        if best is not None:
            conf, centre, w, h, angle, cls = best
            rows.append((observe, centre[0], centre[1], w, h, angle, conf, cls))
        elapsed = time.monotonic() - started
        time.sleep(max(0.0, period - elapsed))

    print(f"{'t':>9} {'cx':>8} {'cy':>8} {'w':>6} {'h':>6} {'ang':>7} {'conf':>5} {'cls':>3}")
    for t, cx, cy, w, h, angle, conf, cls in rows:
        print(f"{t:9.3f} {cx:8.2f} {cy:8.2f} {w:6.2f} {h:6.2f} {angle:7.2f} {conf:5.2f} {cls:3d}")
    if len(rows) > 10:
        arr = np.asarray([[r[1], r[2], r[3], r[4], r[5]] for r in rows])
        for index, name in enumerate(("cx", "cy", "w", "h", "ang")):
            column = arr[:, index]
            print(f"{name}: 均值 {column.mean():8.2f}  σ {column.std():6.2f}")


if __name__ == "__main__":
    sys.exit(main())
