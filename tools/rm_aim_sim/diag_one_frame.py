"""最小复现：抓一帧，打印 associate 的每个候选配对的 (gap, theta, facing)。"""
import math
import sys
import time
from pathlib import Path

for candidate in (Path(__file__).resolve().parent, Path(__file__).resolve().parent.parent):
    sys.path.insert(0, str(candidate))

import numpy as np
import rm_aim_sim_bridge as bridge
from deploy_dev_talos_bridge import Rp0526Detector, solve_pnp, CameraInfo, Pose, FrameBundle
from vision_backend import BodyTracker, VisionBackend

channel = bridge.Channel("default", timeout=5.0)
state = channel.read_state()
detector = Rp0526Detector(Path("D:/HNU_RM/HNU_NHS_Vision/assets/models/rp0526.onnx"), 0.5, "CPU")
tracker = BodyTracker()

for attempt in range(80):
    try:
        identity = channel.request_frame()
        metadata, pixels = channel.receive_frame(identity, timeout=0.8)
    except (TimeoutError, OSError, ValueError):
        time.sleep(0.1)
        continue
    camera = metadata.get("camera") or {}
    width = int(camera.get("width", 0)); height = int(camera.get("height", 0))
    channels = 3 if metadata.get("encoding") == "rgb8" else 4
    array = np.frombuffer(pixels, dtype=np.uint8).reshape(height, width, channels)
    bgr = np.ascontiguousarray(array[:, :, 2::-1])
    observe = float(metadata.get("sim_time") or 0.0)
    truth = channel.read_json("state.json").get("truth") or {}
    armors = {a["id"]: a for a in truth.get("armors") or []}
    truth_theta = math.atan2(armors[0]["center"][1] - armors[2]["center"][1],
                             armors[0]["center"][0] - armors[2]["center"][0]) if 0 in armors and 2 in armors else None

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
    print(f"\n帧 sim={observe:.3f} det={len(detections)} cand={len(positions)} "
          f"truth_theta={math.degrees(truth_theta) if truth_theta is not None else float('nan'):.2f} "
          f"tracker_theta={math.degrees(tracker.theta) if tracker.theta is not None else float('nan')}",
          flush=True)
    if len(positions) < 2:
        time.sleep(0.03)
        continue

    from deploy_dev_talos_bridge import quaternion_matrix_wxyz
    world_from_gimbal = quaternion_matrix_wxyz(bundle.gimbal.quaternion_wxyz)
    camera_world = np.asarray(bundle.odom.position, dtype=np.float64) \
        + world_from_gimbal @ np.asarray(bundle.camera.position, dtype=np.float64)
    print(f"camera_world={np.round(camera_world, 4).tolist()} "
          f"shape={world_from_gimbal.shape} gimbal_yaw={math.degrees(math.atan2(2*(quat[0]*quat[3]+quat[1]*quat[2]), 1-2*(quat[2]**2+quat[3]**2))):.1f}", flush=True)

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
                    facing = tracker._facing(theta, {id_i: positions[i], id_j: positions[j]},
                                             camera_world)
                    err = abs(math.degrees(math.remainder(theta - truth_theta, 2 * math.pi))) \
                        if truth_theta is not None else -1
                    if err > 90:
                        err = 180 - err
                    tag = "正确" if err < 15 else ("错配90" if 70 < err < 110 else "其他")
                    prior = None
                    if tracker.theta is not None and tracker.last_time is not None:
                        prior = tracker.theta + tracker.omega * max(0.0, observe - tracker.last_time)
                    consistent = prior is not None and \
                        abs(math.remainder(2.0 * (theta - prior), 2 * math.pi)) < math.radians(40.0)
                    print(f"  ({i}->{id_i}, {j}->{id_j}) gap={gap:.4f} "
                          f"theta={math.degrees(theta):8.2f} facing={facing} "
                          f"cons={consistent} err={err:6.2f} {tag}",
                          flush=True)
    plates, theta = tracker.associate(positions, now=observe, origin=camera_world)
    print(f"  => associate: plates={plates} theta={math.degrees(theta) if theta is not None else None}", flush=True)
    if plates:
        tracker.observe(observe, plates, origin=camera_world)
        print(f"  => observe 后 tracker.theta={math.degrees(tracker.theta) if tracker.theta is not None else None} "
              f"solved={tracker.solved_frames}", flush=True)
    time.sleep(0.03)
