"""离线比较预测器：用录制的真实 PnP 序列 + 仿真真值，算各模型在 +T 秒的误差。

录制见 vision_backend.step() 里的 AIM_RECORD：
    set AIM_RECORD=aim_record.jsonl
    python tools/rm_aim_sim/bench_hit_rate.py --seconds 30 --backend vision --detector rp0526

用法：
    python tools/rm_aim_sim/study_predictor.py aim_record.jsonl --horizon 0.20

比较：常数保持 / 线性 / 二次 / 刚体圆周（ω 自由搜索或固定真值）。
输出中位、p90 的米误差，以及 4.3 m 处折算的角度（°）。
"""

import argparse
import json
import math

import numpy as np


def load(path):
    rows = []
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    rows.sort(key=lambda row: row["t"])
    return rows


def truth_at(rows, when):
    """按真值自己的时间轴（ts）插值装甲中心；返回 {id: np.array} 或 None。"""
    timed = [(row.get("ts", row["t"]), row) for row in rows]
    if not timed or when < timed[0][0] or when > timed[-1][0]:
        return None
    low, high = 0, len(timed) - 1
    while high - low > 1:
        mid = (low + high) // 2
        if timed[mid][0] <= when:
            low = mid
        else:
            high = mid
    (a_time, a), (b_time, b) = timed[low], timed[high]
    span = b_time - a_time
    weight = 0.0 if span <= 1e-9 else (when - a_time) / span
    out = {}
    left = {int(armor[0]): armor[1:] for armor in a["truth"]}
    right = {int(armor[0]): armor[1:] for armor in b["truth"]}
    for armor_id, center in left.items():
        other = right.get(armor_id)
        if other is None:
            continue
        out[armor_id] = np.asarray(center) + (np.asarray(other) - np.asarray(center)) * weight
    return out


def window(hist, when, span):
    """hist 是 [(t, [x,y,z])]；取 when 前 span 秒内的样本。"""
    rows = [(t, np.asarray(p, dtype=np.float64)) for t, p in hist if when - t <= span + 1e-9]
    rows = [(t, p) for t, p in rows if t <= when + 1e-9]
    return rows


def fit_poly(rows, order, horizon):
    times = np.asarray([t for t, _ in rows])
    if len(rows) < 2:
        return None
    t_ref = times[-1]
    us = times - t_ref
    out = []
    for axis in range(3):
        values = np.asarray([p[axis] for _, p in rows])
        if order == 1 or len(rows) < 3:
            design = np.stack([np.ones_like(us), us], axis=1)
        else:
            design = np.stack([np.ones_like(us), us, 0.5 * us * us], axis=1)
        coef, *_ = np.linalg.lstsq(design, values, rcond=None)
        value = coef[0] + coef[1] * horizon
        if order == 2 and len(rows) >= 3:
            value += coef[2] * horizon * horizon
        out.append(value)
    return np.asarray(out)


def fit_orbit(rows, horizon, omega=None):
    """p(t)=c0+v·u+½a·u²+R(ωu)(A,B)；给定 ω 线性最小二乘。"""
    count = len(rows)
    if count < 5:
        return None
    times = np.asarray([t for t, _ in rows])
    if times[-1] - times[0] < 0.25:
        return None
    t_ref = times[-1]
    us = times - t_ref
    points = np.asarray([p for _, p in rows])
    basis_z = np.stack([np.ones_like(us), us, 0.5 * us * us], axis=1)
    z_coef, *_ = np.linalg.lstsq(basis_z, points[:, 2], rcond=None)
    rhs = np.concatenate([points[:, 0], points[:, 1]])

    def solve(rate):
        wu = rate * us
        cw, sw = np.cos(wu), np.sin(wu)
        design = np.zeros((2 * count, 8))
        design[:count, 0] = 1.0
        design[:count, 1] = us
        design[:count, 2] = 0.5 * us * us
        design[:count, 6] = cw
        design[:count, 7] = -sw
        design[count:, 3] = 1.0
        design[count:, 4] = us
        design[count:, 5] = 0.5 * us * us
        design[count:, 6] = sw
        design[count:, 7] = cw
        coef, *_ = np.linalg.lstsq(design, rhs, rcond=None)
        residual = design @ coef - rhs
        return float(residual @ residual), coef

    if omega is None:
        best = None
        for rate in np.arange(-8.0, 8.0001, 0.2):
            rss, coef = solve(float(rate))
            if best is None or rss < best[0]:
                best = (rss, float(rate), coef)
        center = best[1]
        for rate in np.arange(center - 0.2, center + 0.2001, 0.02):
            rss, coef = solve(float(rate))
            if rss < best[0]:
                best = (rss, float(rate), coef)
        _, rate, coef = best
    else:
        _, coef = solve(float(omega))
        rate = float(omega)
    c0x, vx, ax, c0y, vy, ay, rot_a, rot_b = coef
    c0z, vz, az = z_coef
    u = horizon
    wu = rate * u
    cw, sw = math.cos(wu), math.sin(wu)
    return np.asarray([
        c0x + vx * u + 0.5 * ax * u * u + rot_a * cw - rot_b * sw,
        c0y + vy * u + 0.5 * ay * u * u + rot_a * sw + rot_b * cw,
        c0z + vz * u + 0.5 * az * u * u,
    ])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("path")
    parser.add_argument("--horizon", type=float, default=0.20)
    parser.add_argument("--min-samples", type=int, default=5)
    parser.add_argument("--max-match", type=float, default=0.25,
                        help="PnP 与真值最近板的距离门限（超过说明这帧是外推/错配）")
    parser.add_argument("--min-span", type=float, default=0.15,
                        help="历史窗口至少覆盖的时间（秒）")
    args = parser.parse_args()

    rows = load(args.path)
    print(f"录制 {len(rows)} 帧，{rows[0]['t']:.1f}–{rows[-1]['t']:.1f} s")

    methods = {
        "hold 0": ("hold", None),
        "lin .25": ("poly", (1, 0.25)),
        "lin .45": ("poly", (1, 0.45)),
        "quad .25": ("poly", (2, 0.25)),
        "quad .45": ("poly", (2, 0.45)),
        "orbit .50": ("orbit", 0.50),
        "orbit .70": ("orbit", 0.70),
        "orbit .70 ω=2": ("orbit_fixed", 0.70),
        "truth(noise floor)": ("truth", None),
    }
    errors = {name: [] for name in methods}
    distances = []
    used = 0
    for row in rows:
        when = row["t"]
        future = truth_at(rows, when + args.horizon)
        current = truth_at(rows, when)
        if not future or not current:
            continue
        position = np.asarray(row["p"])
        armor_id = min(current, key=lambda key: float(np.linalg.norm(position - current[key])))
        if float(np.linalg.norm(position - current[armor_id])) > args.max_match:
            continue
        if armor_id not in future:
            continue
        target = future[armor_id]
        used += 1
        distances.append(float(np.linalg.norm(target)))
        for name, (kind, param) in methods.items():
            if kind == "hold":
                samples = window(row["hist"], when, 0.5)
                if not samples:
                    continue
                estimate = samples[-1][1]
            elif kind == "truth":
                estimate = current[armor_id]  # 原地保持，误差 = 目标在这段时间的位移
            elif kind == "poly":
                order, span = param
                samples = window(row["hist"], when, span)
                if len(samples) < args.min_samples or \
                        samples[-1][0] - samples[0][0] < args.min_span:
                    continue
                estimate = fit_poly(samples, order, args.horizon)
            elif kind == "orbit":
                samples = window(row["hist"], when, param)
                if len(samples) < 6 or samples[-1][0] - samples[0][0] < args.min_span:
                    continue
                estimate = fit_orbit(samples, args.horizon)
            else:
                samples = window(row["hist"], when, param)
                if len(samples) < 6 or samples[-1][0] - samples[0][0] < args.min_span:
                    continue
                estimate = fit_orbit(samples, args.horizon, omega=2.0)
            if estimate is None:
                continue
            errors[name].append(float(np.linalg.norm(estimate - target)))

    print(f"可评估 {used} 帧，平均距离 {np.mean(distances):.2f} m")
    print(f"{'方法':>20} {'n':>5} {'中位 m':>8} {'p90 m':>8} {'中位°':>7} {'p90°':>7}")
    mean_distance = float(np.mean(distances))
    for name, values in errors.items():
        if not values:
            continue
        values = np.asarray(values)
        angle = np.degrees(np.arctan2(values, mean_distance))
        print(f"{name:>20} {len(values):>5} {np.median(values):8.3f} "
              f"{np.percentile(values, 90):8.3f} {np.median(angle):7.2f} "
              f"{np.percentile(angle, 90):7.2f}")


if __name__ == "__main__":
    main()
