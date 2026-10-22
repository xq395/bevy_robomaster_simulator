"""量"法向朝向相机"约束对相邻板错配的分辨力。

每帧双检出时枚举 24 种配对，对每种算 (gap, theta, facing_ok)，
与真值 theta 比对，分成 正确(<10°)/相邻错配(70~110°)/其他，
统计 facing 约束把错配滤掉的比例。
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
from vision_backend import BodyTracker

R_CV_TO_GIMBAL = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])


def pnp_normal(detection, bundle):
    import cv2
    from deploy_dev_talos_bridge import quaternion_matrix_wxyz
    width, height = (0.230 if detection["large"] else 0.135), 0.056
    object_points = np.array(
        [[0, width / 2, height / 2], [0, -width / 2, height / 2],
         [0, -width / 2, -height / 2], [0, width / 2, -height / 2]], dtype=np.float64)
    success, rvec, _ = cv2.solvePnP(object_points, detection["points"],
                                    bundle.camera_info.matrix, bundle.camera_info.distortion,
                                    flags=cv2.SOLVEPNP_IPPE)
    if not success:
        return None
    rotation, _ = cv2.Rodrigues(rvec)
    return quaternion_matrix_wxyz(bundle.gimbal.quaternion_wxyz) @ (R_CV_TO_GIMBAL @ (rotation @ np.array([1.0, 0.0, 0.0])))


def facing_ok(theta, assignment, positions, origin):
    """assignment: {plate_id: position index}。解出 theta 后两块板的应有法向是否都朝向相机。"""
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    for plate_id, index in assignment.items():
        nx, ny, _ = BodyTracker.PLATE_NORMALS[plate_id]
        normal = (cos_t * nx - sin_t * ny, sin_t * nx + cos_t * ny, 0.0)
        position = positions[index]
        to_me = [origin[k] - position[k] for k in range(3)]
        if sum(to_me[k] * normal[k] for k in range(3)) <= 0.05:
            return False
    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=15.0)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--confidence", type=float, default=0.5)
    args = parser.parse_args()

    channel = bridge.Channel("default", timeout=5.0)
    state = channel.read_state()
    print(f"仿真就绪：{(state.get('camera') or {}).get('width')}x"
          f"{(state.get('camera') or {}).get('height')}", flush=True)
    detector = Rp0526Detector(Path("D:/HNU_RM/HNU_NHS_Vision/assets/models/rp0526.onnx"),
                              args.confidence, "CPU")
    tracker = BodyTracker()

    period = 1.0 / args.hz
    stop_at = time.monotonic() + args.seconds
    frames = 0
    stats = {"correct": 0, "adjacent": 0, "other": 0}
    correct_gap, adjacent_gap = [], []
    facing_correct = facing_adjacent = 0
    frames_with_both = 0
    state_truth = {}

    def truth_at(sim_time):
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
        positions = []
        for detection in detections:
            result = solve_pnp(detection, bundle)
            if result is not None and result.reprojection_error_px <= 5.0:
                positions.append([float(v) for v in result.world_position])

        if len(positions) >= 2 and truth_theta is not None:
            frames_with_both += 1
            count = len(positions)
            for i in range(count):
                for j in range(count):
                    if i == j:
                        continue
                    for id_i in BodyTracker.PLATES:
                        for id_j in BodyTracker.PLATES:
                            if id_i == id_j:
                                continue
                            theta = tracker._theta_from_pair(positions[i], positions[j], id_i, id_j)
                            if theta is None:
                                continue
                            first = tracker._center_of(positions[i], id_i, theta)
                            second = tracker._center_of(positions[j], id_j, theta)
                            if first is None or second is None:
                                continue
                            gap = math.sqrt(sum((first[k] - second[k]) ** 2 for k in range(3)))
                            if gap > 0.15:
                                continue
                            error = abs(math.degrees(math.remainder(theta - truth_theta, 2 * math.pi)))
                            ok = facing_ok(theta, {id_i: i, id_j: j}, positions, origin)
                            if error < 10:
                                stats["correct"] += 1
                                correct_gap.append(gap)
                                facing_correct += 1 if ok else 0
                            elif 70 < error < 110:
                                stats["adjacent"] += 1
                                adjacent_gap.append(gap)
                                facing_adjacent += 1 if ok else 0
                            else:
                                stats["other"] += 1

        frames += 1
        elapsed = time.monotonic() - started
        time.sleep(max(0.0, period - elapsed))

    print(f"\n帧数 {frames}，双检出 {frames_with_both}")
    print(f"gap<0.15 的配对里：正确 {stats['correct']}，相邻错配 {stats['adjacent']}，"
          f"其他 {stats['other']}")
    if correct_gap:
        correct_gap.sort()
        print(f"正确配对 gap: p50={correct_gap[len(correct_gap)//2]:.4f} "
              f"p90={correct_gap[int(len(correct_gap)*0.9)]:.4f}")
    if adjacent_gap:
        adjacent_gap.sort()
        print(f"相邻错配 gap: p50={adjacent_gap[len(adjacent_gap)//2]:.4f} "
              f"p90={adjacent_gap[int(len(adjacent_gap)*0.9)]:.4f}")
    print(f"facing 约束：正确配对通过率 {facing_correct}/{stats['correct']}，"
          f"相邻错配通过率 {facing_adjacent}/{stats['adjacent']}")


if __name__ == "__main__":
    sys.exit(main())
