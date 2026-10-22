"""命中率基准：给仿真喂一段固定时长的"接管 + 自瞄 + 授权开火"，统计命中率。

用法（先让仿真跑起来，见 tools/rm_aim_sim/README.md）：

    python tools/rm_aim_sim/bench_hit_rate.py --seconds 20
    python tools/rm_aim_sim/bench_hit_rate.py --tolerance 0.2 --window 0.05 --no-feedforward

它自己起一个桥接子进程（用 5101 端口，不占终端的 5001），跑完就退，输出一行 JSON。
仿真的 stats 是累计值，所以用前后差值算本轮命中率——目标血量在 bench.cfg 里设成很大，
不会因为被打死而停下来变成静止靶。
"""

import argparse
import json
import math
import socket
import subprocess
import sys
import time
from pathlib import Path

# 桥接可能在上一级（源码树 tools/）或同级（部署包 components/aim/）
for _candidate in (Path(__file__).resolve().parent,
                   Path(__file__).resolve().parent.parent):
    sys.path.insert(0, str(_candidate))
import rm_aim_sim_bridge as bridge  # noqa: E402


def wait_reply(sock, peer, deadline):
    """等桥接回第一包。未连通的 UDP 不会因为端口没人监听而拿到 ECONNRESET。"""
    while time.monotonic() < deadline:
        try:
            sock.sendto(b"status", peer)
            data, _ = sock.recvfrom(4096)
            return json.loads(data)
        except (TimeoutError, socket.timeout, ConnectionResetError, OSError):
            continue
    raise TimeoutError("桥接没有回应租约请求")


def run_round(args, client, port):
    python = args.python or sys.executable
    here = Path(__file__).resolve().parent
    script = here / "rm_aim_sim_bridge.py"
    if not script.exists():
        script = here.parent / "rm_aim_sim_bridge.py"
    command = [python, str(script), "--channel", args.channel,
               "--operator-port", str(port), "--bind", "127.0.0.1",
               "--tolerance-deg", str(args.tolerance), "--lead-window", str(args.window),
               "--rate-tol", str(args.rate_tol),
               "--search-rate", str(args.search_rate)]
    if args.backend == "truth":
        command.append("--no-video")
    if args.no_feedforward:
        command.append("--no-feedforward")
    if args.no_lead_consume:
        command.append("--no-lead-consume")
    if args.yaw_offset:
        command += ["--yaw-offset", str(args.yaw_offset)]
    if args.pitch_offset:
        command += ["--pitch-offset", str(args.pitch_offset)]
    if args.release_latency is not None:
        command += ["--release-latency", str(args.release_latency)]
    if args.backend == "vision":
        command += ["--backend", "vision", "--detector", args.detector,
                    "--device", args.device,
                    "--confidence", str(args.confidence), "--vision-hz", str(args.vision_hz),
                    "--alpha", str(args.alpha), "--beta", str(args.beta),
                    "--roi-margin", str(args.roi_margin),
                    "--corner-offset", str(args.corner_offset)]
        if args.no_roi:
            command.append("--no-roi")
        if args.use_body:
            command.append("--use-body")
        if args.calibrate_fire:
            command.append("--calibrate-fire")
        if args.push:
            command.append("--push")
        command += ["--max-fire-gap", str(args.max_fire_gap)]
        command += ["--muzzle-lag", str(args.muzzle_lag)]
        if args.truth_log:
            command += ["--truth-log", args.truth_log]
        command += ["--min-settle", str(args.min_settle)]
        command += ["--max-fire-distance", str(args.max_fire_distance)]
        if args.motion_window is not None:
            command += ["--motion-window", str(args.motion_window)]
        if args.state_source:
            command.append("--state-source")
        if args.vision_latency:
            command += ["--vision-latency", str(args.vision_latency)]
        if args.fire_log:
            command += ["--fire-log", args.fire_log]
        if args.send_log:
            command.append("--send-log")
        if args.model:
            command += ["--model", args.model]
    before = client.read_state().get("stats") or {}
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        peer = ("127.0.0.1", port)
        sock.settimeout(0.3)
        reply = wait_reply(sock, peer, time.monotonic() + 8.0)
        session = reply["session"]
        sequence = 1
        stop_at = time.monotonic() + args.warmup + args.seconds
        firing_at = time.monotonic() + args.warmup
        fired_window = False
        last_reason = ""
        reasons = {}
        replies = 0
        resyncs = 0
        while time.monotonic() < stop_at:
            aim = time.monotonic() >= firing_at
            packet = {"session": session, "seq": sequence,
                      "sent_ms": time.time_ns() // 1_000_000, "ttl_ms": 250,
                      "aim_enabled": True, "fire_authorized": aim, "spin_enabled": False,
                      "mode": "command",
                      "manual": {"movement": [0.0, 0.0], "gimbal": [0.0, 0.0], "shoot": False}}
            sock.sendto(json.dumps(packet).encode(), peer)
            sequence += 1
            fired_window = fired_window or aim
            try:
                reply = json.loads(sock.recvfrom(4096)[0])
                replies += 1
                # bridge 每次撤销（operator_timeout / simulator_offline）都会 session+1，
                # 而 snapshot 里带的就是当前 session。只认启动时那一版的 bench 会被
                # 之后所有包打成 session_mismatch、整轮报废——每次回复都重同步。
                if reply.get("session") != session:
                    session = reply["session"]
                    resyncs += 1
                if reply.get("reason"):
                    last_reason = reply["reason"]
                    # bridge 会给原始 reason 加"· 惯性保持/搜索中"后缀，剥掉再统计
                    head = last_reason.split("·")[0].strip()
                    head = head.split()[0] if head else last_reason
                    reasons[head] = reasons.get(head, 0) + 1
            except (TimeoutError, socket.timeout, ConnectionResetError, OSError):
                pass
            time.sleep(0.02)
        sock.sendto(json.dumps(dict(packet, seq=sequence, aim_enabled=False,
                                    fire_authorized=False, ttl_ms=250,
                                    sent_ms=time.time_ns() // 1_000_000)).encode(), peer)
        time.sleep(0.3)
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()

    after = client.read_state().get("stats") or {}
    fired = int(after.get("fired", 0)) - int(before.get("fired", 0))
    hit = int(after.get("hit", 0)) - int(before.get("hit", 0))
    ground = int(after.get("ground", 0)) - int(before.get("ground", 0))
    damage = float(after.get("damage", 0)) - float(before.get("damage", 0))
    result = {"fired": fired, "hit": hit, "ground": ground, "reason": last_reason,
              "hit_rate": round(hit / fired, 4) if fired else 0.0,
              "damage": round(damage, 1), "replies": replies, "resyncs": resyncs,
              "reasons": dict(sorted(reasons.items(), key=lambda kv: -kv[1]))}
    return result


def main():
    parser = argparse.ArgumentParser(description="rm_aim_sim 命中率基准")
    parser.add_argument("--channel", default="default")
    parser.add_argument("--port", type=int, default=5101, help="桥接用的租约端口（避开终端的 5001）")
    parser.add_argument("--seconds", type=float, default=20.0, help="开火时长")
    parser.add_argument("--warmup", type=float, default=1.5, help="先只跟踪不开火")
    parser.add_argument("--search-rate", type=float, default=0.40,
                        help="丢目标时的搜索角速度（rad/s）；0.4 扫满一圈要 16 s，"
                             "预热不够时整轮都在找目标")
    parser.add_argument("--tolerance", type=float, default=0.45, help="开火角度门限（度）")
    parser.add_argument("--window", type=float, default=0.10, help="测速窗口（秒）")
    parser.add_argument("--rate-tol", type=float, default=1.2, help="角速度偏差门限（rad/s）")
    parser.add_argument("--no-feedforward", action="store_true")
    parser.add_argument("--release-latency", type=float, default=None,
                        help="覆盖桥接的 --release-latency（秒）")
    parser.add_argument("--no-lead-consume", action="store_true",
                        help="关掉消费端时延补偿（对照组）")
    parser.add_argument("--state-source", action="store_true",
                        help="按 60 Hz 状态直跑（仅 ideal 检测器）")
    parser.add_argument("--vision-latency", type=float, default=0.0,
                        help="视觉链路附加时延标定量（秒）")
    parser.add_argument("--fire-log", default="",
                        help="开火决策日志路径（JSONL）")
    parser.add_argument("--send-log", action="store_true",
                        help="记录指令发出日志，测指令→出膛时延")
    parser.add_argument("--yaw-offset", type=float, default=0.0)
    parser.add_argument("--pitch-offset", type=float, default=0.0)
    parser.add_argument("--backend", choices=("truth", "vision"), default="truth")
    parser.add_argument("--detector", choices=("rp0526", "yolo26", "ideal"), default="rp0526")
    parser.add_argument("--model", default="")
    parser.add_argument("--device", default="CPU",
                        help="检测器后端：CPU / GPU / cuda（cuda 走 onnxruntime CUDA，"
                             "实测 rp0526 40→9 ms）")
    parser.add_argument("--confidence", type=float, default=0.5)
    parser.add_argument("--vision-hz", type=float, default=20.0)
    parser.add_argument("--no-roi", action="store_true")
    parser.add_argument("--use-body", action="store_true",
                        help="走刚体模型（默认关：小板上单板时序跟踪更准）")
    parser.add_argument("--push", action="store_true",
                        help="让仿真连续推流（省掉请求-应答的 1~2 个渲染帧）")
    parser.add_argument("--max-fire-gap", type=float, default=0.08,
                        help="开火要求瞄准解对应的检出有多新鲜（秒）")
    parser.add_argument("--muzzle-lag", type=float, default=0.02,
                        help="按多少秒后的预测枪口角判开火对齐")
    parser.add_argument("--truth-log", default="",
                        help="60 Hz 真值装甲位姿日志（逐发归因用）")
    parser.add_argument("--min-settle", type=float, default=0.15,
                        help="换板后稳定多久才允许开火（秒）")
    parser.add_argument("--max-fire-distance", type=float, default=0.0,
                        help=">0 时只在近于该距离时开火")
    parser.add_argument("--calibrate-fire", action="store_true",
                        help="绕过开火对齐门限按 10 Hz 满打：命中率直接反映解算精度，"
                             "样本量从十几发涨到几十发，适合 A/B")
    parser.add_argument("--motion-window", type=float, default=None,
                        help="单板运动拟合窗口（秒），透传给桥接；不传用后端默认")
    parser.add_argument("--roi-margin", type=float, default=2.2)
    parser.add_argument("--corner-offset", type=float, default=0.0)
    parser.add_argument("--alpha", type=float, default=0.70)
    parser.add_argument("--beta", type=float, default=0.12)
    parser.add_argument("--python", default="", help="桥接用的解释器，默认与本文同一份")
    parser.add_argument("--label", default="")
    args = parser.parse_args()

    channel = bridge.Channel(args.channel, timeout=5.0)
    result = run_round(args, channel, args.port)
    result.update(backend=args.backend, alpha=args.alpha, beta=args.beta,
                  tolerance=args.tolerance, window=args.window,
                  feedforward=not args.no_feedforward, rate_tol=args.rate_tol,
                  latency=args.release_latency, seconds=args.seconds, label=args.label,
                  yaw_offset=args.yaw_offset, pitch_offset=args.pitch_offset)
    print(json.dumps(result, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())


