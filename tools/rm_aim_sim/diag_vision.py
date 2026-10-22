"""视觉链路体检：检测 → PnP → 世界坐标，和仿真真值逐帧比对。

先起仿真，再：
    python tools/rm_aim_sim/diag_vision.py --frames 40

看点：
  * 检出率：每帧检出几块装甲、置信度多少；
  * **距离误差**：PnP 算出的目标距离 vs 真值距离。差 ~1.7 倍就是 large/small 判错了；
  * 角度误差：PnP 出的世界坐标相对真值的偏差；
  * color_id：仿真靶车是蓝方，确认敌我颜色索引。
"""

import argparse
import math
import statistics
import sys
import time
from pathlib import Path

for _candidate in (Path(__file__).resolve().parent, Path(__file__).resolve().parent.parent):
    sys.path.insert(0, str(_candidate))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

import rm_aim_sim_bridge as bridge  # noqa: E402
from vision_backend import VisionBackend  # noqa: E402


def truth_nearest(state, world):
    """真值里离 PnP 结果最近的那块装甲，返回 (armor_id, 距离误差, 横向误差)。"""
    origin = state["gimbal"]["origin"]
    best = None
    for armor in state["truth"]["armors"]:
        center = armor["center"]
        error = math.dist(world, center)
        if best is None or error < best[0]:
            best = (error, armor["id"], center)
    error, armor_id, center = best
    true_distance = math.dist(origin, center)
    return armor_id, error, true_distance


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--channel", default="default")
    parser.add_argument("--detector", choices=("rp0526", "yolo26"), default="rp0526")
    parser.add_argument("--model", default="")
    parser.add_argument("--frames", type=int, default=40)
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--enemy-color", type=int, default=1)
    parser.add_argument("--min-hits", type=int, default=1)
    args = parser.parse_args()

    model = args.model or {
        "rp0526": "D:/HNU_RM/HNU_NHS_Vision/assets/models/rp0526.onnx",
        "yolo26": "D:/HNU_RM/HNU_NHS_Vision/assets/models/yolo26.onnx",
    }[args.detector]
    channel = bridge.Channel(args.channel, timeout=8.0)
    backend = VisionBackend(model, confidence=args.confidence, detector=args.detector,
                            enemy_color=args.enemy_color, min_hits=args.min_hits)
    print(f"检测器 {args.detector} · 模型 {model}")

    rows = []
    started = time.monotonic()
    for _ in range(args.frames):
        state = channel.read_state()
        identity = channel.request_frame()
        metadata, pixels = channel.receive_frame(identity, timeout=2.0)
        width, height = int(metadata["camera"]["width"]), int(metadata["camera"]["height"])
        channels = 3 if metadata["encoding"] == "rgb8" else 4
        image = np.frombuffer(pixels, np.uint8).reshape(height, width, channels)
        bgr = cv2.cvtColor(image[:, :, :3], cv2.COLOR_RGB2BGR)

        yaw, pitch, fire, info = backend.step(state, bgr, time.time_ns())
        if info.get("world"):
            armor_id, error, true_distance = truth_nearest(state, info["world"])
            pnp_distance = info.get("distance") or true_distance
            rows.append({"error": error, "true": true_distance, "pnp": pnp_distance,
                         "armor": armor_id, "class": info["class_id"], "large": info["large"],
                         "color": info["color_id"], "conf": info["confidence"],
                         "reproj": info["reprojection"], "yaw_error": info.get("yaw_error")})
        if len(rows) <= 3:
            print("  帧", len(rows), {k: info.get(k) for k in
                                      ("detections", "class_id", "large", "color_id", "confidence",
                                       "distance", "reprojection", "reason")})

    elapsed = time.monotonic() - started
    print(f"\n{args.frames} 帧用时 {elapsed:.1f}s（{args.frames / elapsed:.1f} fps，含取图与推理）")
    print(f"检出并解出目标：{len(rows)}/{args.frames}")
    if not rows:
        print("没有可用目标——先看检出率，必要时把曝光/亮度调到能看见灯条")
        return 1

    errors = [row["error"] for row in rows]
    relative = [row["pnp"] / row["true"] for row in rows if row["true"] > 0.1]
    print(f"位置误差  中位 {statistics.median(errors) * 100:.1f} cm  最大 {max(errors) * 100:.1f} cm")
    print(f"距离比 pnp/truth  中位 {statistics.median(relative):.3f}（≈1.0 才对；≈1.7 说明按大装甲算了）")
    print(f"class_id 分布 {sorted({row['class'] for row in rows})}  "
          f"large={sorted({row['large'] for row in rows})}  "
          f"color_id={sorted({row['color'] for row in rows})}")
    print(f"置信度 中位 {statistics.median(row['conf'] for row in rows):.2f}  "
          f"重投影 中位 {statistics.median(row['reproj'] for row in rows):.2f} px")
    yaw_errors = [abs(row["yaw_error"]) for row in rows if row["yaw_error"] is not None]
    if yaw_errors:
        print(f"云台角度偏差 中位 {statistics.median(yaw_errors):.2f}°")
    return 0


if __name__ == "__main__":
    sys.exit(main())
