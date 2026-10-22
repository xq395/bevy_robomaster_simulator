"""逐发归因（弹道重建版）：把每发的实际弹道和 60 Hz 真值装甲求最小接近距离。

为什么不用 attribute_misses：ground 弹的 `impact.time` 是**落地**时刻，不是子弹
掠过装甲的时刻。拿它去查真值装甲会参考到一块已经跑远的板，误差被严重低估
（实测报 0.5°，真实最近接近是 1.6°）。

用法：
    python tools/rm_aim_sim/bench_hit_rate.py ... --truth-log truth_log.jsonl
    python tools/rm_aim_sim/attribute_flight.py truth_log.jsonl

输出：最近接近距离的中位/p90（米/度）、命中弹 vs 未命中弹、以及每发最近的板 id。
"""

import json
import math
import sys
from pathlib import Path

import numpy as np

for candidate in (Path(__file__).resolve().parent, Path(__file__).resolve().parent.parent):
    sys.path.insert(0, str(candidate))

import rm_aim_sim_bridge as bridge  # noqa: E402

BARREL = 0.22
SPEED = 23.0
GRAVITY = 9.7833


def load_truth(path):
    truth = {}
    for line in open(path, encoding="utf-8"):
        line = line.strip()
        if not line:
            continue
        row = json.loads(line)
        for armor in row["a"]:
            truth.setdefault(int(armor[0]), []).append(
                (row["t"], np.asarray(armor[1:4]), np.asarray(armor[4:7])))
    for armor_id in truth:
        truth[armor_id].sort(key=lambda item: item[0])
    return truth


def armor_at(truth, armor_id, when):
    rows = truth[armor_id]
    times = [row[0] for row in rows]
    if when < times[0] or when > times[-1]:
        return None
    index = min(max(int(np.searchsorted(times, when)), 1), len(rows) - 1)
    t0, c0, n0 = rows[index - 1]
    t1, c1, n1 = rows[index]
    weight = 0.0 if t1 == t0 else (when - t0) / (t1 - t0)
    center = c0 + (c1 - c0) * weight
    normal = n0 + (n1 - n0) * weight
    return center, normal / (np.linalg.norm(normal) + 1e-9)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "truth_log.jsonl"
    truth = load_truth(path)
    channel = bridge.Channel("default", timeout=5.0)
    state = channel.read_state()
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
        direction = np.array([math.cos(pitch) * math.cos(yaw),
                              math.cos(pitch) * math.sin(yaw),
                              -math.sin(pitch)])
        start = origin + BARREL * direction
        velocity = direction * SPEED
        born = float(shot["time"])
        best = (1e9, None, None)
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
                    local = point - center
                    tangent = np.array([-normal[1], normal[0], 0.0])
                    tangent /= np.linalg.norm(tangent) + 1e-9
                    up = np.cross(normal, tangent)
                    up /= np.linalg.norm(up) + 1e-9
                    best = (distance, armor_id, age,
                            float(local @ tangent), float(local @ up))
        rows.append({"id": shot.get("id"), "kind": impact.get("kind"),
                     "closest": best[0], "armor": best[1], "age": best[2],
                     "lat": best[3] if len(best) > 3 else None,
                     "vert": best[4] if len(best) > 4 else None})

    usable = [row for row in rows if row["armor"] is not None]
    if not usable:
        print("没有可归因的弹（真值日志与 shot 事件时间不重叠？）")
        return 1
    distances = np.array([row["closest"] for row in usable])
    angles = np.degrees(np.arctan2(distances, 4.0))
    print(f"可归因 {len(usable)}/{len(rows)} 发")
    print(f"最近接近：p50 {np.median(distances):.3f} m ({np.median(angles):.2f}°)  "
          f"p90 {np.percentile(distances, 90):.3f} m ({np.percentile(angles, 90):.2f}°)")
    hits = [row["closest"] for row in usable if row["kind"] == "hit"]
    misses = [row["closest"] for row in usable if row["kind"] != "hit"]
    print(f"命中 {len(hits)} 发" + (f"，p50 {np.median(hits):.3f} m" if hits else ""))
    print(f"未中 {len(misses)} 发" + (f"，p50 {np.median(misses):.3f} m" if misses else ""))
    print(f"命中率 {len(hits) / len(usable):.1%}")
    lats = np.array([abs(row["lat"]) for row in usable if row["lat"] is not None])
    verts = np.array([abs(row["vert"]) for row in usable if row["vert"] is not None])
    if len(lats) and len(verts):
        print(f"横向偏 |lat|：p50 {np.median(lats) * 100:.1f} cm  p90 {np.percentile(lats, 90) * 100:.1f} cm")
        print(f"纵向偏 |vert|：p50 {np.median(verts) * 100:.1f} cm  p90 {np.percentile(verts, 90) * 100:.1f} cm")
        inside = int(((lats <= 0.0675) & (verts <= 0.0625)).sum())
        print(f"落在板矩形内（按最近接近时刻）：{inside}/{len(usable)} = {inside / len(usable):.1%}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
