"""rp0526 真检测器链路诊断：抓真实帧 → 检测 → PnP，逐帧统计漏点。

用法（仿真跑着的时候）：
    python tools/rm_aim_sim/diag_rp0526.py --seconds 12 --confidence 0.5
输出：
  * 每 30 帧一行的汇总（raw 检出 / NMS 后 / 过重投影门限 / 真值板数）
  * 结束时的总体统计：检出率、漏检原因分布、置信度/重投影/板子像素尺寸分位
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
from deploy_dev_talos_bridge import Rp0526Detector, solve_pnp, CameraInfo, Pose, FrameBundle


def percentile(values, q):
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(q * (len(ordered) - 1))))
    return ordered[index]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--channel", default="default")
    parser.add_argument("--seconds", type=float, default=12.0)
    parser.add_argument("--confidence", type=float, default=0.5)
    parser.add_argument("--model", default="D:/HNU_RM/HNU_NHS_Vision/assets/models/rp0526.onnx")
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--roi", action="store_true", help="同时测 ROI 裁剪路径")
    args = parser.parse_args()

    channel = bridge.Channel(args.channel, timeout=5.0)
    state = channel.read_state()
    print(f"仿真就绪：camera {(state.get('camera') or {}).get('width')}x"
          f"{(state.get('camera') or {}).get('height')}", flush=True)

    detector = Rp0526Detector(Path(args.model), args.confidence, "CPU")
    print(f"rp0526 加载完成，conf>={args.confidence}", flush=True)

    period = 1.0 / args.hz
    stop_at = time.monotonic() + args.seconds
    rows = []
    raw_counts, conf_list, repro_list, size_list = [], [], [], []
    frames = 0
    missed_no_detect = 0
    missed_repro = 0
    plate_frames = 0
    detected_plate_frames = 0

    while time.monotonic() < stop_at:
        started = time.monotonic()
        try:
            identity = channel.request_frame()
            metadata, pixels = channel.receive_frame(identity, timeout=0.8)
        except (TimeoutError, OSError, ValueError) as exc:
            print("取帧失败:", exc, flush=True)
            time.sleep(0.1)
            continue
        camera = metadata.get("camera") or {}
        width = int(camera.get("width", 0))
        height = int(camera.get("height", 0))
        channels = 3 if metadata.get("encoding") == "rgb8" else 4
        array = np.frombuffer(pixels, dtype=np.uint8).reshape(height, width, channels)
        bgr = np.ascontiguousarray(array[:, :, 2::-1])
        truth = camera.get("armors") or []

        image = bgr
        if args.roi:
            # 复刻 vision_backend._detect 的 ROI 路径：按上一帧 box 裁
            if rows and rows[-1]["boxes"]:
                boxes = rows[-1]["boxes"]
                xs1 = min(b[0] for b in boxes); ys1 = min(b[1] for b in boxes)
                xs2 = max(b[2] for b in boxes); ys2 = max(b[3] for b in boxes)
                side = max(xs2 - xs1, ys2 - ys1) * 2.2
                side = max(side, 224)
                side = min(side, min(width, height))
                left = int(max(0, min(round((xs1 + xs2) * 0.5 - side * 0.5), width - int(side))))
                top = int(max(0, min(round((ys1 + ys2) * 0.5 - side * 0.5), height - int(side))))
                image = bgr[top:top + int(side), left:left + int(side)]

        detections = detector.detect(image)
        raw_counts.append(len(detections))
        boxes = []
        kept = []
        for detection in detections:
            conf = float(detection["confidence"])
            conf_list.append(conf)
            x1, y1, x2, y2 = detection["box"]
            boxes.append((x1, y1, x2, y2))
            size_list.append(max(x2 - x1, y2 - y1))
            # PnP（用整幅图的内参；ROI 模式下检测在裁图上，坐标需还原——这里只测检测）
            K = camera.get("K") or []
            if len(K) != 9:
                continue
            info = CameraInfo(0, K[0], K[4], K[2], K[5], np.zeros(5), width, height)
            gimbal = metadata.get("gimbal") or {}
            quat = np.asarray(gimbal.get("quaternion_wxyz") or [1.0, 0.0, 0.0, 0.0], dtype=np.float64)
            origin = np.asarray(gimbal.get("origin") or [0.0, 0.0, 0.0], dtype=np.float64)
            offset = np.asarray(camera.get("t_camera2gimbal") or [0.0, 0.0, 0.0], dtype=np.float64)
            bundle = FrameBundle(bgr, 0, 0,
                                 Pose(0, origin, quat, 0), Pose(0, origin.copy(), np.array([1.0, 0.0, 0.0, 0.0]), 0),
                                 Pose(0, origin, quat, 0), Pose(0, offset, quat, 0), info)
            result = solve_pnp(detection, bundle)
            if result is None:
                missed_repro += 1
                continue
            repro_list.append(result.reprojection_error_px)
            if result.reprojection_error_px <= 5.0:
                kept.append(result)
            else:
                missed_repro += 1

        frames += 1
        if truth:
            plate_frames += 1
            if detections:
                detected_plate_frames += 1
            else:
                missed_no_detect += 1
        rows.append({"boxes": boxes, "kept": len(kept), "truth": len(truth), "raw": len(detections)})
        if frames % 30 == 0:
            print(f"[{frames:4d}] raw={len(detections)} kept={len(kept)} truth={len(truth)} "
                  f"sim={metadata.get('sim_time'):.2f}", flush=True)
        elapsed = time.monotonic() - started
        time.sleep(max(0.0, period - elapsed))

    print("\n===== 汇总 =====", flush=True)
    print(f"帧数 {frames}，其中有真值板的帧 {plate_frames}")
    if plate_frames:
        print(f"检出率（raw>0）: {detected_plate_frames / plate_frames:.1%}，"
              f"完全漏检 {missed_no_detect} 帧 ({missed_no_detect / plate_frames:.1%})")
    if plate_frames:
        one = sum(1 for r in rows if r["raw"] == 1 and r["truth"] > 0)
        two = sum(1 for r in rows if r["raw"] >= 2 and r["truth"] > 0)
        print(f"有真值板的帧里：单检出 {one} ({one / plate_frames:.1%})，"
              f"双检出 {two} ({two / plate_frames:.1%})")
    print(f"raw detections/帧: p50={percentile(raw_counts, .5):.0f} "
          f"p90={percentile(raw_counts, .9):.0f} max={max(raw_counts) if raw_counts else 0}")
    print(f"confidence: p10={percentile(conf_list, .1):.3f} p50={percentile(conf_list, .5):.3f} "
          f"max={max(conf_list) if conf_list else 0:.3f}")
    print(f"reprojection_px: p50={percentile(repro_list, .5):.2f} "
          f"p90={percentile(repro_list, .9):.2f} max={max(repro_list) if repro_list else 0:.2f}")
    print(f"过 5px 门限被滤: {missed_repro} 个候选")
    print(f"板子最大边 px: p10={percentile(size_list, .1):.0f} p50={percentile(size_list, .5):.0f} "
          f"p90={percentile(size_list, .9):.0f}")
    kept_per_frame = [row["kept"] for row in rows]
    print(f"每帧可用候选(kept): p50={percentile(kept_per_frame, .5):.0f} "
          f"p90={percentile(kept_per_frame, .9):.0f} max={max(kept_per_frame) if kept_per_frame else 0}")
    truth_per_frame = [row["truth"] for row in rows]
    print(f"每帧真值板数: p50={percentile(truth_per_frame, .5):.0f} max={max(truth_per_frame) if truth_per_frame else 0}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
