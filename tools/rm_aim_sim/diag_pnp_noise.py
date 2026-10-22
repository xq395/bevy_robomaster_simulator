"""PnP 误差细查：按类别拆分，并把 y 误差对真值速度回归（分离"运动滞后假象"与真实噪声）。

err_y ≈ −vy·lag → 滞后假象；回归斜率≈0 → 真实 PnP 噪声。
另测深度偏差随假定板宽的标定关系（换 --width-scale 扫一遍找零点）。

用法：python tools/rm_aim_sim/diag_pnp_noise.py --seconds 10
     python tools/rm_aim_sim/diag_pnp_noise.py --width-scale 0.965
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

_original_solve = vision.solve_pnp


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--hz", type=float, default=25.0)
    parser.add_argument("--confidence", type=float, default=0.5)
    parser.add_argument("--width-scale", type=float, default=1.0,
                        help="缩放 solve_pnp 用的板宽（标定深度偏差）")
    args = parser.parse_args()

    if args.width_scale != 1.0:
        scale = args.width_scale
        original = vision.solve_pnp

        def scaled_solve_pnp(detection, bundle):
            saved = detection.get("points")
            detection = dict(detection)
            # 改 detection 的物理尺寸：solve_pnp 内部按 detection["large"] 选宽高，
            # 这里临时改 large 不管用，所以直接换函数太重——简单办法是缩放 points
            # 会同比例缩放解出的深度。1/scale 缩放角点 → 深度 ×1/scale。
            detection["points"] = np.asarray(saved, dtype=np.float64) / scale
            return original(detection, bundle)

        vision.solve_pnp = scaled_solve_pnp
        import vision_backend
        vision_backend.vision.solve_pnp = scaled_solve_pnp

    channel = bridge.Channel("default", timeout=5.0)
    channel.read_state()
    backend = VisionBackend("D:/HNU_RM/HNU_NHS_Vision/assets/models/rp0526.onnx",
                            detector="rp0526", confidence=args.confidence,
                            roi=False, use_body=False,
                            gate={"enabled": False})

    period = 1.0 / args.hz
    stop_at = time.monotonic() + args.seconds
    errors = []          # (dx, dy, dz, class_large, plate_id)
    reproj = []
    confs = []
    widths = []
    prev_truth = None     # (t, center)
    lags = []             # (err_y, vy)
    n_frames = 0
    n_det = 0
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
        truth_list = (state.get("truth") or {}).get("armors") or []
        if not truth_list:
            continue
        bundle = backend._bundle(state, bgr, int(state.get("tick", 0)), time.time_ns())
        if bundle is None:
            continue
        detections, _ = backend._detect(bgr)
        n_frames += 1
        n_det += len(detections)
        # 真值车心速度（相邻状态差分）
        truth_center = np.mean([np.asarray(a["center"], dtype=np.float64)
                                for a in truth_list], axis=0)
        vy = None
        if prev_truth is not None:
            dt = observe - prev_truth[0]
            if 0.001 < dt < 0.2:
                vy = (truth_center[1] - prev_truth[1][1]) / dt
        prev_truth = (observe, truth_center)
        for detection in detections:
            result = vision.solve_pnp(detection, bundle)
            if result is None:
                continue
            pos = np.asarray(result.world_position, dtype=np.float64)
            best, best_d, best_id = None, None, None
            for armor in truth_list:
                center = np.asarray(armor["center"], dtype=np.float64)
                d = float(np.linalg.norm(pos - center))
                if best_d is None or d < best_d:
                    best, best_d, best_id = center, d, armor.get("id")
            if best is not None and best_d < 0.5:
                err = pos - best
                errors.append((err[0], err[1], err[2],
                               1 if detection.get("large") else 0, best_id))
                reproj.append(float(result.reprojection_error_px))
                confs.append(float(detection["confidence"]))
                pts = detection["points"]
                widths.append(float(np.linalg.norm(pts[1] - pts[0])))
                if vy is not None:
                    lags.append((err[1], vy))
        elapsed = time.monotonic() - started
        time.sleep(max(0.0, period - elapsed))

    print(f"width_scale={args.width_scale}  帧 {n_frames}，检测 {n_det}，匹配候选 {len(errors)}")
    if not errors:
        return
    arr = np.asarray([e[:3] for e in errors])
    classes = np.asarray([e[3] for e in errors])
    for label, mask in (("all", np.ones(len(arr), bool)),
                        ("small", classes == 0), ("large", classes == 1)):
        if not mask.any():
            continue
        sub = arr[mask]
        print(f"[{label:>5}] n={len(sub):3d}  "
              f"dx {sub[:, 0].mean():+.4f}±{sub[:, 0].std():.4f}  "
              f"dy {sub[:, 1].mean():+.4f}±{sub[:, 1].std():.4f}  "
              f"dz {sub[:, 2].mean():+.4f}±{sub[:, 2].std():.4f}")
    if lags:
        lag_arr = np.asarray(lags)
        vy, ey = lag_arr[:, 1], lag_arr[:, 0]
        slope = float(np.polyfit(vy, ey, 1)[0]) if len(vy) > 10 else float("nan")
        resid = ey - slope * vy
        print(f"err_y ~ vy 回归: 斜率 {slope:+.4f} s（=滞后），残差 σ {resid.std():.4f} m")
    norm = np.linalg.norm(arr, axis=1)
    print(f"|e| p50 {np.percentile(norm, 50):.4f} p90 {np.percentile(norm, 90):.4f} m  "
          f"重投影 p50 {np.percentile(reproj, 50):.2f} px  宽 p50 {np.percentile(widths, 50):.1f} px")


if __name__ == "__main__":
    sys.exit(main())
