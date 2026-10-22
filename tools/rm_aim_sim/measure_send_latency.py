"""量"指令发出 → 子弹出膛"的真实时延：send 日志（墙钟）对上仿真 shot 事件（仿真钟）。

    python tools/rm_aim_sim/measure_send_latency.py send_log.jsonl

仿真事件从 state.json 的 events 里拿（脚本会再读一次最终状态，所以要在仿真还活着时跑，
或把 events 先存下来）。这里直接在跑完基准后、仿真还没退场时调用。
"""

import json
import statistics
import sys
from pathlib import Path

for candidate in (Path(__file__).resolve().parent,
                  Path(__file__).resolve().parent.parent):
    sys.path.insert(0, str(candidate))
import rm_aim_sim_bridge as bridge  # noqa: E402


def main():
    send_path = sys.argv[1] if len(sys.argv) > 1 else "F:/bevy_robomaster_simulator/send_log.jsonl"
    sends = [json.loads(line) for line in open(send_path, encoding="utf-8")]
    channel = bridge.Channel("default", timeout=5.0)
    state = channel.read_state()
    events = state.get("events") or []
    shots = [e for e in events if e.get("type") == "shot"]

    # 仿真钟 → 墙钟：用任意一份带 wall_unix_sec 的状态
    sim_time = float(state.get("sim_time") or 0.0)
    wall_unix = float(state.get("wall_unix_sec") or 0.0)
    offset = wall_unix - sim_time
    # 墙钟(unix) → 单调钟：send 日志用的是单调钟，近似用当前差值（同一台机器漂移可忽略）
    import time
    mono_minus_unix = time.monotonic() - time.time()

    print(f"send 记录 {len(sends)} 条，shot 事件 {len(shots)} 条")
    lags = []
    for shot in shots:
        spawn_wall = float(shot["time"]) + offset + mono_minus_unix
        # 找这条弹对应的 fire 指令：出膛时刻之前最近的一条 fire=True
        prior = [s for s in sends if s["t"] <= spawn_wall and s["fire"]]
        if not prior:
            continue
        last = prior[-1]
        lag = spawn_wall - last["t"]
        if -0.05 < lag < 0.3:
            lags.append(lag)
    if not lags:
        print("没有配上的 shot")
        return 1
    lags.sort()
    print(f"指令→出膛时延: 中位 {statistics.median(lags)*1000:.0f} ms  "
          f"p10 {lags[len(lags)//10]*1000:.0f}  p90 {lags[len(lags)*9//10]*1000:.0f}  n={len(lags)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
