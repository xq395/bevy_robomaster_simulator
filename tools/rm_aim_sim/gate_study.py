"""开火门离线扫参：一轮数据比较所有门限，不用每个值各打一轮。

每发子弹都用真值日志算出"到达板平面时的 facing"和"仿真是否判命中"，再离线问：
"只在到达 facing ≥ g 时开火"会剩下哪些发、命中率多少、每秒还能打几发。
真值 facing 是上限口径（门用的是真值）；实际门要靠预测，所以选出的 g 还得实跑确认。

用法：
    python tools/rm_aim_sim/bench_hit_rate.py ... --truth-log truth.jsonl
    python tools/rm_aim_sim/gate_study.py truth.jsonl 25
"""

import json
import math
import sys
from pathlib import Path

import numpy as np

for candidate in (Path(__file__).resolve().parent, Path(__file__).resolve().parent.parent):
    sys.path.insert(0, str(candidate))

import rm_aim_sim_bridge as bridge  # noqa: E402
from attribute_flight import BARREL, GRAVITY, SPEED, armor_at, load_truth  # noqa: E402


def arrival_facing(truth, start, velocity, born):
    """子弹飞行期间，离弹道最近的那块板在最近时刻的 facing（-v̂·n）。"""
    best = (1e9, None)
    unit = velocity / np.linalg.norm(velocity)
    for armor_id in truth:
        for step in range(1, 160):
            age = step * 0.002
            pose = armor_at(truth, armor_id, born + age)
            if pose is None:
                break
            center, normal = pose
            point = start + velocity * age + np.array([0.0, 0.0, -0.5 * GRAVITY * age * age])
            distance = float(np.linalg.norm(point - center))
            if distance < best[0]:
                best = (distance, float(-unit @ normal))
    return best[1]


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "truth_log.jsonl"
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 25.0
    truth = load_truth(path)
    state = bridge.Channel("default", timeout=5.0).read_state()
    origin = np.asarray(state["gimbal"]["origin"], dtype=np.float64)
    events = state.get("events") or []
    impacts = {e.get("id"): e for e in events if e.get("type") == "impact"}
    rows = []
    for shot in (e for e in events if e.get("type") == "shot"):
        impact = impacts.get(shot.get("id"))
        if impact is None:
            continue
        yaw, pitch = float(shot["yaw"]), float(shot["pitch"])
        direction = np.array([math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw),
                              -math.sin(pitch)])
        facing = arrival_facing(truth, origin + BARREL * direction, direction * SPEED, float(shot["time"]))
        if facing is not None:
            rows.append((facing, impact.get("kind") == "hit"))
    if not rows:
        print("没有可分析的弹")
        return 1
    print(f"样本 {len(rows)} 发（事件缓冲只保留最近一段，按 {seconds:.0f} s 折算射速）")
    print(" 门限 g  剩余发数  命中率   有效命中/s")
    for gate in (0.0, 0.5, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9):
        kept = [hit for facing, hit in rows if facing >= gate]
        if not kept:
            continue
        rate = sum(kept) / len(kept)
        print(f"  {gate:4.2f}   {len(kept):5d}    {rate:6.1%}   {sum(kept) / seconds * len(rows) / max(1, len(rows)):5.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
