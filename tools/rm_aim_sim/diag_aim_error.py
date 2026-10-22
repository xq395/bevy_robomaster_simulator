"""按帧测"云台指令角 vs 命中时刻真实需要的角"——把瞄准误差从命中率里抽出来。

命中率一发只有 0/1，12 秒才十几发，噪声太大；真正决定命中率的是**指令角误差**。
本脚本边开火边以 60 Hz 采 (仿真真值, 指令角)，跑完用**同一段录制里 t+飞行时间 的
真值**算"要打中此刻该瞄哪"，逐帧给 yaw/pitch 误差。这是 diag_lead.py 的真值版本，
但不需要离线模型：直接查未来帧，连加速度/换向都天然正确。

用法（先起仿真）：
    python tools/rm_aim_sim/diag_aim_error.py --seconds 8 ^
        --bridge-args "--backend vision --detector rp0526 --motion-window 0.25"

输出：命中率、逐帧 |yaw| 中位/p90、带符号中位（系统偏差）、按"目标是否在平移"分组，
并把每帧明细写到 --out（JSONL）。
"""

import argparse
import json
import math
import shlex
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

for candidate in (Path(__file__).resolve().parent, Path(__file__).resolve().parent.parent):
    sys.path.insert(0, str(candidate))

import rm_aim_sim_bridge as bridge  # noqa: E402


def drive(sock, peer, stop, box):
    """20 Hz 心跳（和 bench 一致：每次回复重同步 session）。"""
    sequence = 1
    while not stop.is_set():
        packet = {"session": box.get("session"), "seq": sequence,
                  "sent_ms": time.time_ns() // 1_000_000, "ttl_ms": 250,
                  "aim_enabled": True, "fire_authorized": True, "spin_enabled": False,
                  "mode": "command",
                  "manual": {"movement": [0.0, 0.0], "gimbal": [0.0, 0.0], "shoot": False}}
        sock.sendto(json.dumps(packet).encode(), peer)
        sequence += 1
        try:
            reply = json.loads(sock.recvfrom(4096)[0])
            if reply.get("session") is not None:
                box["session"] = reply["session"]
            box["reply"] = reply
            box.setdefault("replies", []).append(reply)
            box["reply_count"] = box.get("reply_count", 0) + 1
        except (TimeoutError, socket.timeout, ConnectionResetError, OSError):
            pass
        time.sleep(0.02)


def interpolate(samples, when):
    """在录制里按仿真时间线性插值真值装甲中心；返回 {id: center} 或 None。"""
    if not samples or when < samples[0]["t"] or when > samples[-1]["t"]:
        return None
    # 时间单调，二分
    low, high = 0, len(samples) - 1
    while high - low > 1:
        mid = (low + high) // 2
        if samples[mid]["t"] <= when:
            low = mid
        else:
            high = mid
    a, b = samples[low], samples[high]
    span = b["t"] - a["t"]
    weight = 0.0 if span <= 1e-9 else (when - a["t"]) / span
    out = {}
    for armor_id, center in a["armors"].items():
        other = b["armors"].get(armor_id)
        if other is None:
            continue
        out[armor_id] = [center[k] + (other[k] - center[k]) * weight for k in range(3)]
    return out


def needed_for(sample, when, armors):
    """origin 在这一帧，装甲中心取 t+flight 的值；迭代飞行时间。"""
    weapon = sample["weapon"]
    origin = sample["origin"]
    flight = sample["distance"] / float(weapon.get("speed", 23.0))
    result = None
    for _ in range(4):
        target = armors.get(sample["aim_id"])
        if target is None:
            return None
        result = bridge.ballistic(float(weapon["speed"]), float(weapon["gravity"]),
                                  float(weapon["barrel_length"]),
                                  [target[k] - origin[k] for k in range(3)])
        if result is None:
            return None
        flight = result[2]
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--channel", default="default")
    parser.add_argument("--port", type=int, default=5103)
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--warmup", type=float, default=1.5)
    parser.add_argument("--bridge-args", default="")
    parser.add_argument("--python", default="")
    parser.add_argument("--out", default="aim_error.jsonl")
    parser.add_argument("--log", default="aim_error_bridge.log")
    args = parser.parse_args()

    client = bridge.Channel(args.channel, timeout=5.0)
    here = Path(__file__).resolve().parent
    script = here / "rm_aim_sim_bridge.py"
    if not script.exists():
        script = here.parent / "rm_aim_sim_bridge.py"
    command = [args.python or sys.executable, str(script), "--channel", args.channel,
               "--operator-port", str(args.port), "--bind", "127.0.0.1"]
    if args.bridge_args:
        command += shlex.split(args.bridge_args)
    else:
        command.append("--no-video")
    log = open(args.log, "w", encoding="utf-8", errors="replace")
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)

    before = client.read_state().get("stats") or {}
    samples = []          # 真值序列（插值用）
    frames = []           # 指令样本
    stop = threading.Event()
    box = {}
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        peer = ("127.0.0.1", args.port)
        sock.settimeout(0.3)
        for _ in range(80):
            try:
                sock.sendto(b"status", peer)
                box["session"] = json.loads(sock.recvfrom(4096)[0])["session"]
                break
            except (TimeoutError, socket.timeout, ConnectionResetError, OSError):
                time.sleep(0.1)
        if box.get("session") is None:
            print("桥接没回应，见", args.log)
            return 1
        threading.Thread(target=drive, args=(sock, peer, stop, box), daemon=True).start()
        began = time.monotonic()
        armed_at = began + args.warmup
        while time.monotonic() - began < args.warmup + args.seconds:
            state = client.read_state()
            now = time.monotonic()
            truth = state.get("truth") or {}
            armors = {int(a["id"]): [float(v) for v in a["center"]]
                      for a in truth.get("armors") or []}
            if not armors:
                time.sleep(0.01)
                continue
            t = float(state.get("sim_time") or 0.0)
            gimbal = state.get("gimbal") or {}
            origin = [float(v) for v in gimbal.get("origin") or (0, 0, 0)]
            center = truth.get("center") or [0.0, 0.0, 0.0]
            record = {"t": t, "armors": armors, "origin": origin,
                      "weapon": state.get("weapon") or {}}
            samples.append(record)
            if now < armed_at:
                time.sleep(0.012)
                continue
            command = client.read_json("command.json")
            reply = box.get("reply") or {}
            reason = str(reply.get("reason") or "")
            if not command.get("control"):
                time.sleep(0.012)
                continue
            # 只统计"正在跟踪目标"的帧：搜索/惯性保持阶段的指令不是瞄准解
            if not (reason.startswith("vision") or reason.startswith("tracking")):
                time.sleep(0.012)
                continue
            aim_id = None
            best = None
            for armor_id, armor_center in armors.items():
                distance = math.dist(origin, armor_center)
                candidate = bridge.ballistic(
                    float(record["weapon"]["speed"]), float(record["weapon"]["gravity"]),
                    float(record["weapon"]["barrel_length"]),
                    [armor_center[k] - origin[k] for k in range(3)])
                if candidate is None:
                    continue
                error = abs(bridge.wrap(float(command["yaw"]) - candidate[0]))
                if best is None or error < best:
                    best, aim_id = error, armor_id
            if aim_id is None:
                time.sleep(0.012)
                continue
            frames.append({"t": t, "yaw": float(command["yaw"]),
                           "pitch": float(command["pitch"]), "aim_id": aim_id,
                           "origin": origin, "weapon": record["weapon"],
                           "distance": math.dist(origin, armors[aim_id]),
                           "reason": reason})
            time.sleep(0.012)
        stop.set()
        time.sleep(0.3)
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
        log.close()

    after = client.read_state().get("stats") or {}
    fired = int(after.get("fired", 0)) - int(before.get("fired", 0))
    hit = int(after.get("hit", 0)) - int(before.get("hit", 0))
    print(f"心跳回复 {box.get('reply_count', 0)} 条；指令帧 {len(frames)}")
    for reply in box.get("replies", [])[:2]:
        print("  reply:", json.dumps(reply, ensure_ascii=False)[:220])
        print("  reason:", reply.get("reason"))
    print(f"出膛 {fired}，命中 {hit}，命中率 {(hit / fired if fired else 0.0):.1%}"
          f"（{args.seconds:.0f}s）")

    rows = []
    for frame in frames:
        armors = interpolate(samples, frame["t"] + frame["distance"]
                             / float(frame["weapon"].get("speed", 23.0)))
        if armors is None:
            continue
        solution = needed_for(frame, frame["t"], armors)
        if solution is None:
            continue
        yaw_error = math.degrees(bridge.wrap(frame["yaw"] - solution[0]))
        pitch_error = math.degrees(frame["pitch"] - solution[1])
        rows.append({"t": round(frame["t"], 3), "id": frame["aim_id"],
                     "yaw": round(yaw_error, 3), "pitch": round(pitch_error, 3),
                     "distance": round(frame["distance"], 2)})
    if not rows:
        print("没有可比较的指令帧")
        return 1
    with open(args.out, "w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row) + "\n")
    yaw = sorted(abs(row["yaw"]) for row in rows)
    pitch = sorted(abs(row["pitch"]) for row in rows)
    signed = sorted(row["yaw"] for row in rows)
    print(f"可比较 {len(rows)} 帧，距离 {rows[0]['distance']:.2f} m")
    print(f"|yaw|  中位 {yaw[len(yaw) // 2]:.3f}°  p90 {yaw[int(len(yaw) * 0.9)]:.3f}°  "
          f"带符号中位 {signed[len(signed) // 2]:+.3f}°  std "
          f"{statistics_std(signed):.3f}°")
    print(f"|pitch| 中位 {pitch[len(pitch) // 2]:.3f}°  p90 {pitch[int(len(pitch) * 0.9)]:.3f}°")
    negative = sum(1 for value in signed if value < 0)
    print(f"yaw 偏负 {negative} / 偏正 {len(signed) - negative}")
    return 0


def statistics_std(values):
    if len(values) < 2:
        return 0.0
    mean = sum(values) / len(values)
    return math.sqrt(sum((value - mean) ** 2 for value in values) / len(values))


if __name__ == "__main__":
    sys.exit(main())
