"""验证 rp0526 链路里 BodyTracker.associate 的两板配对在 19px 板上的精度。

抓真实帧 → 检测 → PnP，对每帧双检出的候选复刻 associate 的穷举配对，
统计：PnP 位置误差、配对 gap 分布、解出 theta 与真值车身角的差、
以及单板时用 PnP 法向解 theta 的可行性。
"""
import argparse
import math
import sys
import time
from pathlib import Path

for candidate in (Path(__file__).resolve().parent, Path(__file__).resolve().parent.parent):
    sys.path.insert(0, str(candidate))

import cv2
import numpy as np
import rm_aim_sim_bridge as bridge
from deploy_dev_talos_bridge import (Rp0526Detector, solve_pnp, CameraInfo, Pose,
                                     FrameBundle, quaternion_matrix_wxyz)
from vision_backend import BodyTracker

R_CV_TO_GIMBAL = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])


def percentile(values, q):
    if not values:
        return float("nan")
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, max(0, int(q * (len(ordered) - 1))))]


def pnp_with_normal(detection, bundle):
    """solve_pnp 的同款实现，但额外返回板面法向（世界系）。"""
    width, height = (0.230 if detection["large"] else 0.135), 0.056
    object_points = np.array(
        [[0, width / 2, height / 2], [0, -width / 2, height / 2],
         [0, -width / 2, -height / 2], [0, width / 2, -height / 2]], dtype=np.float64)
    success, rvec, tvec = cv2.solvePnP(
        object_points, detection["points"], bundle.camera_info.matrix,
        bundle.camera_info.distortion, flags=cv2.SOLVEPNP_IPPE)
    if not success:
        return None
    rotation, _ = cv2.Rodrigues(rvec)
    normal_cv = rotation @ np.array([1.0, 0.0, 0.0])
    normal_gimbal = R_CV_TO_GIMBAL @ normal_cv
    world_from_gimbal = quaternion_matrix_wxyz(bundle.gimbal.quaternion_wxyz)
    return world_from_gimbal @ normal_gimbal


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--channel", default="default")
    parser.add_argument("--seconds", type=float, default=15.0)
    parser.add_argument("--confidence", type=float, default=0.5)
    parser.add_argument("--hz", type=float, default=30.0)
    args = parser.parse_args()

    channel = bridge.Channel(args.channel, timeout=5.0)
    state = channel.read_state()
    print(f"仿真就绪：camera {(state.get('camera') or {}).get('width')}x"
          f"{(state.get('camera') or {}).get('height')}", flush=True)
    detector = Rp0526Detector(
        Path("D:/HNU_RM/HNU_NHS_Vision/assets/models/rp0526.onnx"), args.confidence, "CPU")

    period = 1.0 / args.hz
    stop_at = time.monotonic() + args.seconds
    tracker = BodyTracker()
    pair_gaps, pair_theta_err = [], []
    good_theta_err, bad_theta_err = [], []
    pos_err = []
    radial_err, tangential_err = [], []
    pair_frames = single_frames = 0
    normal_theta_err = []
    theta_established_at = None
    frames = 0
    state_truth = {}

    def truth_at(sim_time):
        """取 sim_time 之前最新的 state.json 真值。

        truth.angle 与 armor normal 的变化率不一致，不能当车身角基准；
        用板 0/2 的 center 差解 theta，和 BodyTracker._solve_angle 同源但输入是真值。
        """
        nonlocal state_truth
        fresh = channel.read_json("state.json")
        if fresh.get("truth") and fresh.get("sim_time"):
            if not state_truth or abs(fresh["sim_time"] - sim_time) <= \
                    abs(state_truth.get("_sim", -1e9) - sim_time):
                truth = dict(fresh["truth"])
                truth["_sim"] = fresh["sim_time"]
                armors = {a["id"]: a for a in truth.get("armors") or []}
                if 0 in armors and 2 in armors:
                    c0, c2 = armors[0]["center"], armors[2]["center"]
                    truth["theta"] = math.atan2(c0[1] - c2[1], c0[0] - c2[0])
                state_truth = truth
        return state_truth

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
        truth = truth_at(observe)
        truth_theta = truth.get("theta")

        detections = detector.detect(bgr)
        K = camera.get("K") or []
        gimbal = metadata.get("gimbal") or {}
        quat = np.asarray(gimbal.get("quaternion_wxyz") or [1, 0, 0, 0], dtype=np.float64)
        origin = np.asarray(gimbal.get("origin") or [0, 0, 0], dtype=np.float64)
        offset = np.asarray(camera.get("t_camera2gimbal") or [0, 0, 0], dtype=np.float64)
        info = CameraInfo(0, K[0], K[4], K[2], K[5], np.zeros(5), width, height)
        bundle = FrameBundle(bgr, 0, 0, Pose(0, origin, quat, 0),
                             Pose(0, origin.copy(), np.array([1.0, 0, 0, 0]), 0),
                             Pose(0, origin, quat, 0), Pose(0, offset, quat, 0), info)
        candidates, normals = [], []
        truth_armors = [a["center"] for a in (truth.get("armors") or [])]
        truth_normals = [a["normal"] for a in (truth.get("armors") or [])]
        for detection in detections:
            result = solve_pnp(detection, bundle)
            if result is None or result.reprojection_error_px > 5.0:
                continue
            position = [float(v) for v in result.world_position]
            normal = pnp_with_normal(detection, bundle)
            candidates.append(position)
            normals.append(normal)
            if truth_armors:
                nearest = min(range(len(truth_armors)),
                              key=lambda k: sum((position[m] - truth_armors[k][m]) ** 2 for m in range(3)))
                error = [position[k] - truth_armors[nearest][k] for k in range(3)]
                # 径向 = 沿真值板法向（距离方向），切向 = 板面内
                axis = truth_normals[nearest] if nearest < len(truth_normals) else None
                if axis is not None:
                    length = math.sqrt(sum(a * a for a in axis)) or 1.0
                    axis = [a / length for a in axis]
                    radial = sum(error[k] * axis[k] for k in range(3))
                    radial_err.append(abs(radial))
                    tangential_err.append(math.sqrt(max(0.0, sum(e * e for e in error) - radial * radial)))
                pos_err.append(math.sqrt(sum(e * e for e in error)))

        if len(candidates) >= 2:
            pair_frames += 1
            best = None
            count = len(candidates)
            for i in range(count):
                for j in range(count):
                    if i == j:
                        continue
                    for id_i in BodyTracker.PLATES:
                        for id_j in BodyTracker.PLATES:
                            if id_i == id_j:
                                continue
                            theta = tracker._theta_from_pair(candidates[i], candidates[j], id_i, id_j)
                            if theta is None:
                                continue
                            first = tracker._center_of(candidates[i], id_i, theta)
                            second = tracker._center_of(candidates[j], id_j, theta)
                            if first is None or second is None:
                                continue
                            gap = math.sqrt(sum((first[k] - second[k]) ** 2 for k in range(3)))
                            if best is None or gap < best[0]:
                                best = (gap, theta)
            if best is not None:
                pair_gaps.append(best[0])
                if truth_theta is not None:
                    error = abs(math.degrees(math.remainder(best[1] - truth_theta, 2 * math.pi)))
                    if best[0] < 0.06:
                        good_theta_err.append(error)
                    else:
                        bad_theta_err.append(error)
            plates, theta = tracker.associate(candidates)
            if plates:
                tracker.observe(observe, plates)
            if tracker.theta is not None and theta_established_at is None:
                theta_established_at = frames
        elif candidates:
            single_frames += 1
            position = candidates[0]
            normal = normals[0]
            to_me = [origin[k] - position[k] for k in range(3)]
            facing = sum(to_me[k] * normal[k] for k in range(3)) > 0
            if not facing:
                normal = [-v for v in normal]
            normal_yaw = math.atan2(normal[1], normal[0])
            for plate_id in BodyTracker.PLATES:
                nx, ny, _ = BodyTracker.PLATE_NORMALS[plate_id]
                id_yaw = math.atan2(ny, nx)
                theta = math.remainder(normal_yaw - id_yaw, 2 * math.pi)
                px, py, pz = BodyTracker.PLATES[plate_id]
                c, s = math.cos(theta), math.sin(theta)
                rel = (c * px - s * py, s * px + c * py, pz)
                center = [position[k] - rel[k] for k in range(3)]
                if truth_theta is not None and \
                        abs(center[2] - (truth.get("center") or [0, 0, 0])[2]) < 0.06:
                    normal_theta_err.append(abs(math.degrees(
                        math.remainder(theta - truth_theta, 2 * math.pi))))
                    break

        frames += 1
        if frames % 30 == 0:
            print(f"[{frames:4d}] cand={len(candidates)} "
                  f"truth_theta={math.degrees(truth_theta) if truth_theta is not None else 0:7.2f} "
                  f"tracker_theta={math.degrees(tracker.theta) if tracker.theta is not None else 0:7.2f}",
                  flush=True)
        elapsed = time.monotonic() - started
        time.sleep(max(0.0, period - elapsed))

    print("\n===== 汇总 =====", flush=True)
    print(f"帧数 {frames}，双检出帧 {pair_frames}，单检出帧 {single_frames}")
    print(f"PnP 位置误差(m): n={len(pos_err)} p50={percentile(pos_err, .5):.3f} "
          f"p90={percentile(pos_err, .9):.3f} max={max(pos_err) if pos_err else 0:.3f}")
    print(f"  径向(距离向): p50={percentile(radial_err, .5):.3f} "
          f"p90={percentile(radial_err, .9):.3f}  |  切向(板面内): "
          f"p50={percentile(tangential_err, .5):.3f} p90={percentile(tangential_err, .9):.3f}")
    print(f"配对 gap(m): p50={percentile(pair_gaps, .5):.4f} p90={percentile(pair_gaps, .9):.4f} "
          f"max={max(pair_gaps) if pair_gaps else 0:.4f}  (门限 0.06)")
    print(f"配对 gap<0.06 比例: {sum(1 for g in pair_gaps if g < 0.06) / max(1, len(pair_gaps)):.1%}")
    if good_theta_err:
        print(f"  gap<0.06 帧 theta 误差(deg): n={len(good_theta_err)} "
              f"p50={percentile(good_theta_err, .5):.2f} p90={percentile(good_theta_err, .9):.2f} "
              f"max={max(good_theta_err):.2f}")
    if bad_theta_err:
        print(f"  gap>=0.06 帧 theta 误差(deg): n={len(bad_theta_err)} "
              f"p50={percentile(bad_theta_err, .5):.2f}")
    if normal_theta_err:
        print(f"单板法向 theta 误差(deg): n={len(normal_theta_err)} "
              f"p50={percentile(normal_theta_err, .5):.2f} "
              f"p90={percentile(normal_theta_err, .9):.2f} max={max(normal_theta_err):.2f}")
    print(f"theta 建立于第 {theta_established_at} 帧" if theta_established_at is not None
          else "theta 始终未建立")
    return 0


if __name__ == "__main__":
    sys.exit(main())
