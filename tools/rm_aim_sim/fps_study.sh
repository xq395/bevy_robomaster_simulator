#!/usr/bin/env bash
# 帧率研究：仿真必须开着窗口跑（vision 后端要真实渲染），所以把"起仿真→测→关仿真"
# 放进同一个脚本里，避免仿真进程被工具调用回收。
#
#   bash fps_study.sh fps     # 测不同目标频率下的真实吞吐
#   bash fps_study.sh ab      # 闭环 A/B：消费端时延补偿 开 / 关
#   bash fps_study.sh both
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
MODE=${1:-fps}

PY=${PYTHON:-/c/Users/34741/AppData/Local/Programs/Python/Python311/python.exe}
[ -x "$PY" ] || { echo "找不到 python：$PY" >&2; exit 2; }

GODOT=${GODOT_BIN:-/f/rm_aim_sim_ws/godot/Godot_v4.7.2-stable_win64.exe}
PROJ=${RM_AIM_SIM_DIR:-/f/rm_aim_sim_ws/rm_aim_sim}
CFG=${CFG:-$HERE/bench.cfg}
INBOX="${TEMP:-/tmp}/rm_aim_sim_default"

start_sim() {
  [ -f "$INBOX/state.json" ] && mv "$INBOX/state.json" "$INBOX/state.stale"
  "$GODOT" --path "$PROJ" -- --remote --config="$CFG" > /tmp/sim_fps.log 2>&1 &
  SIM_PID=$!
  for _ in $(seq 1 80); do
    [ -f "$INBOX/state.json" ] && return 0
    sleep 0.25
  done
  echo "仿真没起来，看 /tmp/sim_fps.log" >&2
  return 1
}

stop_sim() {
  [ -n "${SIM_PID:-}" ] && kill "$SIM_PID" 2>/dev/null
  wait "${SIM_PID:-}" 2>/dev/null
  SIM_PID=""
  sleep 0.6
}

study_fps() {
  echo "== 帧率上限（1440×1080，bench.cfg）=="
  for hz in 10 20 40 200; do
    "$PY" "$HERE/measure_fps.py" --vision-hz "$hz" --seconds 6 --warmup 10 \
      --python "$PY" --label "vision-hz=$hz"
  done
  echo "== 同场景降到 640×480（bench_640.cfg）=="
  stop_sim
  CFG="$HERE/bench_640.cfg"
  start_sim || return 1
  sleep 1.5
  for hz in 20 40 200; do
    "$PY" "$HERE/measure_fps.py" --vision-hz "$hz" --seconds 6 --warmup 10 \
      --python "$PY" --label "640 · vision-hz=$hz"
  done
}

study_ab() {
  echo "== 闭环 A/B =="
  for pass in 1 2; do
    # 对照组：把消费端时延补偿关掉，其余完全相同
    "$PY" "$HERE/bench_hit_rate.py" --backend vision --detector rp0526 \
      --vision-hz 20 --seconds 15 --warmup 6 --no-lead-consume \
      --python "$PY" --label "无补偿 #$pass"
    "$PY" "$HERE/bench_hit_rate.py" --backend vision --detector rp0526 \
      --vision-hz 20 --seconds 15 --warmup 6 \
      --python "$PY" --label "补时延 #$pass"
  done
}

start_sim || exit 1
sleep 1.5
case "$MODE" in
  fps)  study_fps ;;
  ab)   study_ab ;;
  both) study_fps; study_ab ;;
  *)    echo "未知模式 $MODE" >&2 ;;
esac
stop_sim
