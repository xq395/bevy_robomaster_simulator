"""直连 VisionBackend 的诊断：统计 step() 的 reason 分布和刚体模型状态。

绕过 bridge/bench，直接测 vision_backend 本身在真实帧上的行为：
  * reason 分布（no_target / no_motion / no_plate / ok）
  * body.theta 建立时机、solved_frames、center_history 长度
  * 每帧 detections / candidates / selected / tracking
  * aim 输出频率与 fire 率
"""
import argparse
import math
import sys
import time
from collections import Counter
from pathlib import Path

for candidate in (Path(__file__).resolve().parent, Path(__file__).resolve().parent.parent):
    sys.path.insert(0, str(candidate))

import numpy as np
import rm_aim_sim_bridge as bridge
from vision_backend import VisionBackend


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--confidence", type=float, default=0.5)
    parser.add_argument("--roi", action="store_true")
    parser.add_argument("--trace", type=int, default=0, help="打印前 N 帧的配对候选明细")
    parser.add_argument("--corner-offset", type=float, default=0.0)
    parser.add_argument("--theta-truth", action="store_true",
                        help="用真值 theta 覆盖估计（分离误差来源用）")
    args = parser.parse_args()

    channel = bridge.Channel("default", timeout=5.0)
    state = channel.read_state()
    print(f"仿真就绪：{(state.get('camera') or {}).get('width')}x"
          f"{(state.get('camera') or {}).get('height')}", flush=True)
    backend = VisionBackend("D:/HNU_RM/HNU_NHS_Vision/assets/models/rp0526.onnx",
                            detector="rp0526", confidence=args.confidence,
                            roi=args.roi, corner_offset=args.corner_offset,
                            use_body=False,
                            gate={"enabled": True, "min_consecutive": 2})

    period = 1.0 / args.hz
    stop_at = time.monotonic() + args.seconds
    reasons = Counter()
    frames = 0
    fire_count = 0
    solve_count = 0
    theta_established_at = None
    detections_hist = []
    info_samples = []
    center_err = []
    theta_err = []
    class_counter = Counter()
    state_truth = {}

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
            time.sleep(0.1)
            continue
        if args.theta_truth:
            fresh = channel.read_json("state.json")
            truth = fresh.get("truth") or {}
            armors = {a["id"]: a for a in truth.get("armors") or []}
            if 0 in armors and 2 in armors:
                c0, c2 = armors[0]["center"], armors[2]["center"]
                backend.body.theta = math.atan2(c0[1] - c2[1], c0[0] - c2[0])
        frames += 1
        reasons[info.get("reason") or "ok"] += 1
        if fire:
            fire_count += 1
        if yaw is not None:
            solve_count += 1
        detections_hist.append(info.get("detections", 0))
        if info.get("class_id") is not None:
            class_counter[info["class_id"]] += 1
        if args.trace and frames <= args.trace:
            print(f"--- 帧 {frames} sim={observe:.3f} det={info.get('detections')} "
                  f"theta={math.degrees(backend.body.theta) if backend.body.theta is not None else float('nan'):8.2f} "
                  f"omega={backend.body.omega:6.2f} solved={backend.body.solved_frames} "
                  f"hist={len(backend.body.angle_history)} "
                  f"dec={backend.body.last_decision} reason={info.get('reason')}", flush=True)
            if info.get("world"):
                print(f"    selected world={info['world']} conf={info.get('confidence')} "
                      f"reproj={info.get('reprojection')}", flush=True)
            fresh = channel.read_json("state.json")
            truth = fresh.get("truth") or {}
            armors = {a["id"]: a for a in truth.get("armors") or []}
            if 0 in armors and 2 in armors:
                c0, c2 = armors[0]["center"], armors[2]["center"]
                print(f"    truth theta={math.degrees(math.atan2(c0[1]-c2[1], c0[0]-c2[0])):.2f} "
                      f"center={[round(v,3) for v in truth.get('center') or []]}", flush=True)
        # 真值（state.json 按 sim_time 取最近）
        fresh = channel.read_json("state.json")
        truth = fresh.get("truth") or {}
        if truth.get("armors"):
            armors = {a["id"]: a for a in truth["armors"]}
            if 0 in armors and 2 in armors:
                c0, c2 = armors[0]["center"], armors[2]["center"]
                theta_truth = math.atan2(c0[1] - c2[1], c0[0] - c2[0])
                if backend.body.theta is not None:
                    # θ↔θ+π 等价表示（0↔2、1↔3 互换），误差取两种表示的最小值
                    raw = abs(math.degrees(math.remainder(
                        backend.body.theta - theta_truth, 2 * math.pi)))
                    theta_err.append(min(raw, 180.0 - raw))
            if backend.body.center_history:
                center = backend.body.center_history[-1][1]
                truth_center = truth.get("center")
                if truth_center:
                    center_err.append(math.sqrt(sum((center[k] - truth_center[k]) ** 2
                                                    for k in range(3))))
        if frames % 40 == 0:
            info_samples.append(dict(info))
        if backend.body.theta is not None and theta_established_at is None:
            theta_established_at = frames
        elapsed = time.monotonic() - started
        time.sleep(max(0.0, period - elapsed))

    print("\n===== 汇总 =====", flush=True)
    print(f"帧数 {frames}（目标 {args.hz:.0f} Hz），有解 {solve_count}，fire {fire_count}")
    print(f"reason 分布: {dict(reasons.most_common())}")
    print(f"body.theta 建立于第 {theta_established_at} 帧" if theta_established_at is not None
          else "body.theta 始终未建立")
    print(f"body.solved_frames={backend.body.solved_frames} "
          f"center_history={len(backend.body.center_history)} "
          f"angle_history={len(backend.body.angle_history)}")
    print(f"detections/帧: p50={sorted(detections_hist)[len(detections_hist)//2]:.0f} "
          f"max={max(detections_hist) if detections_hist else 0}")
    print(f"class_id 分布: {dict(class_counter.most_common())}")
    if center_err:
        center_err.sort()
        print(f"车心误差(m): n={len(center_err)} p50={center_err[len(center_err)//2]:.3f} "
              f"p90={center_err[int(len(center_err)*0.9)]:.3f} max={center_err[-1]:.3f}")
        jumps = sum(1 for a, b in zip(center_err, center_err[1:]) if b - a > 0.25)
        print(f"  中心跳变(相邻帧差>0.25m): {jumps} 次")
    if theta_err:
        theta_raw = list(theta_err)
        theta_err.sort()
        print(f"theta 误差(deg): n={len(theta_err)} p50={theta_err[len(theta_err)//2]:.2f} "
              f"p90={theta_err[int(len(theta_err)*0.9)]:.2f} max={theta_err[-1]:.2f}")
        # 时间演化：每 20 帧一段的中位，看 bootstrap 瞬态 vs 稳态
        for start in range(0, len(theta_raw), 20):
            chunk = sorted(theta_raw[start:start + 20])
            if chunk:
                print(f"    帧 {start:4d}-{start + len(chunk):4d}: 中位 {chunk[len(chunk)//2]:6.2f} "
                      f"max {chunk[-1]:6.2f}")
    if info_samples:
        last = info_samples[-1]
        print(f"最后一帧 info: {({k: last[k] for k in last if k not in ('reason',)})}")


if __name__ == "__main__":
    sys.exit(main())
