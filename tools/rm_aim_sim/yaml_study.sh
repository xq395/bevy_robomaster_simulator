#!/usr/bin/env bash
# yaml 参数对命中率的实际贡献：先把"系统偏差"量出来（对应 yaw_offset / pitch_offset），
# 再扫一遍看闭环命中率动不动。
#
#   bash yaml_study.sh bias    # 只测系统偏差（ideal + rp0526 两条链）
#   bash yaml_study.sh sweep   # 扫 yaw_offset 看闭环命中率
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE=${1:-bias}

PY=${PYTHON:-/c/Users/34741/AppData/Local/Programs/Python/Python311/python.exe}
[ -x "$PY" ] || { echo "找不到 python：$PY" >&2; exit 2; }

GODOT=${GODOT_BIN:-/f/rm_aim_sim_ws/godot/Godot_v4.7.2-stable_win64.exe}
PROJ=${RM_AIM_SIM_DIR:-/f/rm_aim_sim_ws/rm_aim_sim}
INBOX="${TEMP:-/tmp}/rm_aim_sim_default"

start_sim() {
  [ -f "$INBOX/state.json" ] && mv "$INBOX/state.json" "$INBOX/state.stale"
  "$GODOT" --path "$PROJ" -- --remote --config="$CFG" > /tmp/sim_yaml.log 2>&1 &
  SIM_PID=$!
  for _ in $(seq 1 80); do
    [ -f "$INBOX/state.json" ] && return 0
    sleep 0.25
  done
  echo "仿真没起来，看 /tmp/sim_yaml.log" >&2
  return 1
}

stop_sim() {
  [ -n "${SIM_PID:-}" ] && kill "$SIM_PID" 2>/dev/null
  wait "${SIM_PID:-}" 2>/dev/null
  SIM_PID=""
  sleep 0.6
}

study_bias() {
  echo "== 系统偏差：出膛角 vs 命中时刻真正需要的角（带符号中位）=="
  for det in ideal rp0526; do
    echo "-- $det"
    "$PY" "$HERE/attribute_misses.py" --seconds 20 --warmup 8 \
      --bridge-args "--backend vision --detector $det --vision-hz 30" 2>&1 | grep -E "出膛|可归因|全体|带符号"
  done
}

study_sweep() {
  echo "== 扫 yaw_offset（ideal 检测器，出弹量够做统计）=="
  for off in 0.0 -0.6 0.6 -1.2 1.2; do
    "$PY" "$HERE/bench_hit_rate.py" --backend vision --detector ideal \
      --vision-hz 30 --seconds 18 --warmup 6 --python "$PY" \
      --yaw-offset "$off" --label "yaw_offset=$off" 2>&1 | tail -1
  done
}

CFG=${CFG:-$HERE/bench.cfg}
start_sim || exit 1
sleep 1.5
case "$MODE" in
  bias)  study_bias ;;
  sweep) study_sweep ;;
  both)  study_bias; study_sweep ;;
  *)     echo "未知模式 $MODE" >&2 ;;
esac
stop_sim
