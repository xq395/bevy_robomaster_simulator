#!/usr/bin/env bash
# 时延标定：强制开火（--calibrate-fire）拿到大量逐发样本，用"出膛角 vs 命中时刻
# 真正需要的角"的带符号中位，扫出视觉链路缺的那段指令生效时延。
#
#   bash latency_calibrate.sh            # 扫默认档位
#   EXTRA="0.008 0.016" bash latency_calibrate.sh
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY=${PYTHON:-/c/Users/34741/AppData/Local/Programs/Python/Python311/python.exe}
[ -x "$PY" ] || { echo "找不到 python：$PY" >&2; exit 2; }

GODOT=${GODOT_BIN:-/f/rm_aim_sim_ws/godot/Godot_v4.7.2-stable_win64.exe}
PROJ=${RM_AIM_SIM_DIR:-/f/rm_aim_sim_ws/rm_aim_sim}
INBOX="${TEMP:-/tmp}/rm_aim_sim_default"

start_sim() {
  [ -f "$INBOX/state.json" ] && mv "$INBOX/state.json" "$INBOX/state.stale"
  "$GODOT" --path "$PROJ" -- --remote --config="$CFG" > /tmp/sim_cal.log 2>&1 &
  SIM_PID=$!
  for _ in $(seq 1 80); do
    [ -f "$INBOX/state.json" ] && return 0
    sleep 0.25
  done
  echo "仿真没起来，看 /tmp/sim_cal.log" >&2
  return 1
}

stop_sim() {
  [ -n "${SIM_PID:-}" ] && kill "$SIM_PID" 2>/dev/null
  wait "${SIM_PID:-}" 2>/dev/null
  SIM_PID=""
  sleep 0.6
}

CFG=${CFG:-$HERE/bench.cfg}
EXTRAS=${EXTRA:-"0.0 0.008 0.016 0.024"}
SECS=${SECS:-14}
WARM=${WARM:-4}

start_sim || exit 1
sleep 1.5
for extra in $EXTRAS; do
  echo "== vision-latency=$extra"
  "$PY" "$HERE/attribute_misses.py" --seconds "$SECS" --warmup "$WARM" \
    --tolerance 0.45 --window 0.06 \
    --bridge-args "--backend vision --detector ideal --state-source --vision-hz 60 \
--calibrate-fire --no-lead-consume --vision-latency $extra" 2>&1 \
    | grep -E "^出膛|^可归因|^全体|^未命中"
  sleep 3
done
stop_sim
