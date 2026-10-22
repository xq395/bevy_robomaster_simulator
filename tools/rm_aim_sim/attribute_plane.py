"""逐发归因（板平面版）：子弹穿过装甲板平面时落在板上的哪里、当时板朝向多正。

attribute_flight.py 量的是"离板心最近距离"，对侧向板会误判——子弹擦过板边
最近距离只有 2 cm，可它根本没穿过板面。仿真判定命中的方式是：子弹穿越板平面的
那一刻，交点在 armor_width × armor_height 矩形内。这里照同样口径算，并把未命中
按原因分成三类：
  * plane_miss  —— 穿过了板平面，但交点在矩形外（横/纵偏多少）
  * edge_on     —— 板几乎侧对（facing 低），子弹在交点前已经从板前方掠过
  * no_cross    —— 飞行时间窗内没穿过任何板平面（提前量/方向大错）

用法：
    python tools/rm_aim_sim/bench_hit_rate.py ... --truth-log truth.jsonl
    python tools/rm_aim_sim/attribute_plane.py truth.jsonl
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

HALF_W, HALF_H = 0.0675, 0.0625


def crossing(truth, armor_id, start, velocity, born):
    """子弹从板前方穿到板后方的时刻（二分），返回 (age, local_lat, local_vert, facing) 或 None。"""
    def side(age):
        pose = armor_at(truth, armor_id, born + age)
        if pose is None:
            return None
        center, normal = pose
        point = start + velocity * age + np.array([0.0, 0.0, -0.5 * GRAVITY * age * age])
        return float((point - center) @ normal)

    previous_age, previous = 0.002, side(0.002)
    for step in range(2, 200):
        age = step * 0.002
        value = side(age)
        if value is None or previous is None:
            return None
        if previous > 0.0 >= value:
            lo, hi = previous_age, age
            for _ in range(20):
                mid = (lo + hi) * 0.5
                if side(mid) > 0.0:
                    lo = mid
                else:
                    hi = mid
            at = (lo + hi) * 0.5
            center, normal = armor_at(truth, armor_id, born + at)
            point = start + velocity * at + np.array([0.0, 0.0, -0.5 * GRAVITY * at * at])
            tangent = np.array([-normal[1], normal[0], 0.0])
            tangent /= np.linalg.norm(tangent) + 1e-9
            up = np.cross(normal, tangent)
            local = point - center
            facing = float(-(velocity / np.linalg.norm(velocity)) @ normal)
            return at, float(local @ tangent), float(local @ up), facing
        previous_age, previous = age, value
    return None


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "truth_log.jsonl"
    truth = load_truth(path)
    state = bridge.Channel("default", timeout=5.0).read_state()
    origin = np.asarray(state["gimbal"]["origin"], dtype=np.float64)
    events = state.get("events") or []
    shots = [event for event in events if event.get("type") == "shot"]
    impacts = {event.get("id"): event for event in events if event.get("type") == "impact"}

    rows = []
    for shot in shots:
        impact = impacts.get(shot.get("id"))
        if impact is None:
            continue
        yaw, pitch = float(shot["yaw"]), float(shot["pitch"])
        direction = np.array([math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw),
                              -math.sin(pitch)])
        start = origin + BARREL * direction
        velocity = direction * SPEED
        born = float(shot["time"])
        best = None
        for armor_id in truth:
            hit = crossing(truth, armor_id, start, velocity, born)
            if hit is None:
                continue
            # 取"最像要打的那块"：交点离板心最近的平面穿越
            score = math.hypot(hit[1], hit[2])
            if best is None or score < best[0]:
                best = (score, armor_id) + hit
        rows.append((impact.get("kind"), best))

    if not rows:
        print("没有可归因的弹（真值日志与 shot 事件时间不重叠？）")
        return 1
    hits = sum(1 for kind, _ in rows if kind == "hit")
    print(f"可归因 {len(rows)} 发，命中 {hits}（{hits / len(rows):.1%}）")
    categories = {"plane_miss_lat": [], "plane_miss_vert": [], "edge_on": [], "no_cross": []}
    facings_hit, facings_miss = [], []
    for kind, best in rows:
        if best is None:
            if kind != "hit":
                categories["no_cross"].append(None)
            continue
        _, _, _, lat, vert, facing = best
        (facings_hit if kind == "hit" else facings_miss).append(facing)
        if kind == "hit":
            continue
        if abs(lat) <= HALF_W and abs(vert) <= HALF_H:
            categories["edge_on"].append(facing)
        elif abs(lat) / HALF_W >= abs(vert) / HALF_H:
            categories["plane_miss_lat"].append(lat)
        else:
            categories["plane_miss_vert"].append(vert)
    misses = len(rows) - hits
    print(f"未命中 {misses} 发：")
    for name, values in categories.items():
        extra = ""
        numeric = [v for v in values if v is not None]
        if numeric and name.startswith("plane"):
            extra = f"  偏差 p50 {np.median(np.abs(numeric)) * 100:.1f} cm，带符号均值 {np.mean(numeric) * 100:+.1f} cm"
        if numeric and name == "edge_on":
            extra = f"  facing p50 {np.median(numeric):.2f}"
        print(f"  {name:16s} {len(values):3d}{extra}")
    if facings_hit and facings_miss:
        print(f"facing（命中）p50 {np.median(facings_hit):.2f}  （未命中）p50 {np.median(facings_miss):.2f}")
    all_lat = [b[3] for k, b in rows if b is not None]
    if all_lat:
        print(f"全部弹横向交点带符号均值 {np.mean(all_lat) * 100:+.1f} cm（系统性提前/滞后）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
