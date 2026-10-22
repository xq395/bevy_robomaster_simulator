"""离线消融：拿 solve() 的预测和"真实未来位姿"比，看命中误差来自哪一项。

先起仿真（无界面即可），再：
    python tools/rm_aim_sim/diag_lead.py --seconds 4 --window 0.10

输出的是"按当前解算打出去，落点相对装甲中心的角度偏差"——小于半个装甲板
（3.8 m 处约 1.0°）才算能打中。
"""

import argparse
import math
import statistics
import sys
import time
from pathlib import Path

# 桥接可能在上一级（源码树 tools/）或同级（部署包 components/aim/）
for _candidate in (Path(__file__).resolve().parent,
                   Path(__file__).resolve().parent.parent):
    sys.path.insert(0, str(_candidate))
import rm_aim_sim_bridge as bridge  # noqa: E402


def collect(client, seconds):
    samples = []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        samples.append(client.read_state())
        time.sleep(0.016)
    return samples


class Variant(bridge.Tracker):
    """在基础 Tracker 上强制改写入参，做单项消融。"""

    def __init__(self, window, omega=None, no_translate=False):
        super().__init__(window=window)
        self.forced_omega = omega
        self.no_translate = no_translate

    def motion(self):
        velocity, acceleration, omega = super().motion()
        if self.no_translate:
            velocity, acceleration = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        if self.forced_omega is not None:
            omega = self.forced_omega
        return velocity, acceleration, omega


def true_omega(samples):
    """用整段样本估 ω：自转是恒定的，长基线基本等于真值。"""
    times = [sample["sim_time"] for sample in samples]
    angles = bridge._unwrap([sample["truth"]["angle"] for sample in samples])
    return bridge._slope([t - times[0] for t in times], angles)


def measure(samples, tracker, index):
    state = samples[index]
    aim = tracker.solve(state)
    if not aim:
        return None
    target_time = state["sim_time"] + aim["flight"]
    future = min(samples[index + 1:], key=lambda s: abs(s["sim_time"] - target_time), default=None)
    if future is None or abs(future["sim_time"] - target_time) > 0.03:
        return None
    plate = next((a["center"] for a in future["truth"]["armors"] if a["id"] == aim["armor_id"]), None)
    if plate is None:
        return None
    origin = state["gimbal"]["origin"]
    weapon = state["weapon"]
    ballistic = bridge.ballistic(weapon["speed"], weapon["gravity"], weapon["barrel_length"],
                                 [plate[i] - origin[i] for i in range(3)])
    if ballistic is None:
        return None
    return (math.degrees(bridge.wrap(aim["yaw"] - ballistic[0])),
            math.degrees(aim["pitch"] - ballistic[1]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--channel", default="default")
    parser.add_argument("--seconds", type=float, default=4.0)
    parser.add_argument("--window", type=float, default=0.10)
    args = parser.parse_args()

    client = bridge.Channel(args.channel, timeout=5.0)
    samples = collect(client, args.seconds)
    omega = true_omega(samples)
    print(f"采到 {len(samples)} 帧，长基线 ω={omega:.4f} rad/s")

    variants = {
        "现状（窗口拟合）": Variant(args.window),
        "ω 用真值": Variant(args.window, omega=omega),
        "去掉平移预测": Variant(args.window, no_translate=True),
        "去掉自转预测": Variant(args.window, omega=0.0),
    }
    for name, tracker in variants.items():
        errors = []
        for index, state in enumerate(samples):
            tracker.observe(state)
            if index < 12:
                continue
            result = measure(samples, tracker, index)
            if result:
                errors.append(result)
        if not errors:
            print(f"{name}：无样本")
            continue
        yaw = [abs(value[0]) for value in errors]
        pitch = [abs(value[1]) for value in errors]
        print(f"{name:18s} n={len(errors):4d}  "
              f"yaw 中位 {statistics.median(yaw):5.2f}° 90分位 {sorted(yaw)[int(len(yaw) * 0.9)]:5.2f}°  "
              f"pitch 中位 {statistics.median(pitch):5.2f}°")
    return 0


if __name__ == "__main__":
    sys.exit(main())
