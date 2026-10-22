"""逐发归因：边开火边采真值，然后拿仿真 events 里每发的出膛角，和"命中时刻真正需要的角"比。

先起仿真（无界面即可），再：
    python tools/rm_aim_sim/attribute_misses.py --seconds 6 --tolerance 0.45 --window 0.10

它自己起一个桥接子进程并持续发送接管+授权开火，采完就退。
"""

import argparse
import json
import math
import socket
import statistics
import shlex
import subprocess
import sys
import threading
import time
from pathlib import Path

# 桥接可能在上一级（源码树 tools/）或同级（部署包 components/aim/）
for _candidate in (Path(__file__).resolve().parent,
                   Path(__file__).resolve().parent.parent):
    sys.path.insert(0, str(_candidate))
import rm_aim_sim_bridge as bridge  # noqa: E402


def drive(sock, peer, session, stop_event, warmup_done):
    """20 Hz 心跳：先跟踪，warmup 之后再授权开火。"""
    sequence = 1
    while not stop_event.is_set():
        packet = {"session": session, "seq": sequence,
                  "sent_ms": time.time_ns() // 1_000_000, "ttl_ms": 250,
                  "aim_enabled": True, "fire_authorized": warmup_done.is_set(),
                  "spin_enabled": False, "mode": "command",
                  "manual": {"movement": [0.0, 0.0], "gimbal": [0.0, 0.0], "shoot": False}}
        sock.sendto(json.dumps(packet).encode(), peer)
        sequence += 1
        try:
            sock.recvfrom(4096)
        except (TimeoutError, socket.timeout, ConnectionResetError, OSError):
            pass
        time.sleep(0.02)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--channel", default="default")
    parser.add_argument("--port", type=int, default=5102)
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--warmup", type=float, default=1.5)
    parser.add_argument("--tolerance", type=float, default=0.45)
    parser.add_argument("--window", type=float, default=0.10)
    parser.add_argument("--python", default="")
    parser.add_argument("--bridge-args", default="",
                        help="额外透传给桥接的参数，例如 \"--backend vision --detector rp0526\"")
    args = parser.parse_args()

    client = bridge.Channel(args.channel, timeout=5.0)
    here = Path(__file__).resolve().parent
    script = here / "rm_aim_sim_bridge.py"
    if not script.exists():
        script = here.parent / "rm_aim_sim_bridge.py"
    command = [args.python or sys.executable, str(script), "--channel", args.channel,
               "--operator-port", str(args.port), "--bind", "127.0.0.1",
               "--tolerance-deg", str(args.tolerance), "--lead-window", str(args.window),
               ]
    if not args.bridge_args:
        command.append("--no-video")
    command += shlex.split(args.bridge_args)
    start_stats = client.read_state().get("stats") or {}
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    samples = []
    stop_event = threading.Event()
    warmup_done = threading.Event()
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        peer = ("127.0.0.1", args.port)
        sock.settimeout(0.3)
        session = None
        for _ in range(60):
            try:
                sock.sendto(b"status", peer)
                session = json.loads(sock.recvfrom(4096)[0])["session"]
                break
            except (TimeoutError, socket.timeout, ConnectionResetError, OSError):
                time.sleep(0.1)
        if session is None:
            print("桥接没回应")
            return 1

        thread = threading.Thread(target=drive, args=(sock, peer, session, stop_event, warmup_done),
                                  daemon=True)
        thread.start()
        began = time.monotonic()
        while time.monotonic() - began < args.warmup + args.seconds:
            state = client.read_state()
            samples.append({"sim_time": state["sim_time"], "truth": state["truth"],
                            "gimbal": state["gimbal"], "weapon": state["weapon"]})
            if not warmup_done.is_set() and time.monotonic() - began >= args.warmup:
                warmup_done.set()
            time.sleep(0.016)
        stop_event.set()
        time.sleep(0.4)
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()

    final = client.read_state()
    stats = final.get("stats") or {}
    fired = stats.get("fired", 0) - start_stats.get("fired", 0)
    hit = stats.get("hit", 0) - start_stats.get("hit", 0)
    print(f"出膛 {fired}，命中 {hit}，命中率 {(hit / fired if fired else 0):.1%}")
    print(f"采样 {len(samples)} 帧")

    events = final.get("events") or []
    shots = [e for e in events if e.get("type") == "shot"]
    impacts = {e.get("id"): e for e in events if e.get("type") == "impact"}
    rows = []
    for shot in shots:
        impact = impacts.get(shot.get("id"))
        if not impact:
            continue
        when = min(samples, key=lambda s: abs(s["sim_time"] - impact["time"]), default=None)
        if when is None or abs(when["sim_time"] - impact["time"]) > 0.05:
            continue
        origin = when["gimbal"]["origin"]
        weapon = when["weapon"]
        best = None
        for armor in when["truth"]["armors"]:
            needed = bridge.ballistic(weapon["speed"], weapon["gravity"], weapon["barrel_length"],
                                      [armor["center"][i] - origin[i] for i in range(3)])
            if needed is None:
                continue
            yaw_error = math.degrees(bridge.wrap(shot["yaw"] - needed[0]))
            pitch_error = math.degrees(shot["pitch"] - needed[1])
            span = math.hypot(yaw_error, pitch_error)
            if best is None or span < best[0]:
                best = (span, yaw_error, pitch_error, armor["id"])
        if best is None:
            continue
        rows.append({"id": shot.get("id"), "kind": impact.get("kind"),
                     "armor": impact.get("armor_id"), "nearest": best[3],
                     "yaw": best[1], "pitch": best[2]})

    misses = [row for row in rows if row["kind"] != "hit"]
    print(f"可归因 {len(rows)} 发，未命中 {len(misses)} 发")
    for row in misses[:20]:
        print(f"  id={row['id']:3d} {row['kind']:7s} 命中装甲={row['armor']:2d} "
              f"最近装甲={row['nearest']:2d} 出膛角偏差 yaw {row['yaw']:+6.2f}° pitch {row['pitch']:+6.2f}°")
    if rows:
        yaw = sorted(abs(row["yaw"]) for row in rows)
        pitch = sorted(abs(row["pitch"]) for row in rows)
        print(f"全体 |yaw| 中位 {statistics.median(yaw):.2f}° 90分位 {yaw[int(len(yaw) * 0.9)]:.2f}°")
        print(f"全体 |pitch| 中位 {statistics.median(pitch):.2f}° 90分位 {pitch[int(len(pitch) * 0.9)]:.2f}°")
        # 带符号的中位数才是"系统偏差"（对应 yaml 的 yaw_offset / pitch_offset）。
        # 必须只统计未命中的发：命中的发本身是按"误差小"筛出来的，混进来会把中位
        # 系统性拉向 0（实测过：混入后 -1.8° 会伪装成 -0.17°）。
        if misses:
            miss_yaw = sorted(row["yaw"] for row in misses)
            miss_pitch = sorted(row["pitch"] for row in misses)
            print(f"未命中带符号 yaw 中位 {statistics.median(miss_yaw):+.2f}° "
                  f"pitch 中位 {statistics.median(miss_pitch):+.2f}°  ← 可用 offset 补偿的量")
            negative = sum(1 for row in misses if row["yaw"] < 0)
            print(f"未命中 {len(misses)} 发里 yaw 偏负 {negative} 发 / 偏正 {len(misses) - negative} 发")
    return 0


if __name__ == "__main__":
    sys.exit(main())
