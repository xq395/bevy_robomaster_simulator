#!/usr/bin/env bash
# 命中率参数扫描：每组配置起一个干净的仿真（无界面），跑固定时长，打印一行结果。
#
#   bash sweep.sh [秒数] [场景cfg]
#
# 源码树（tools/rm_aim_sim/）和部署包（components/aim/）两种布局都能跑：
# 引擎依次找 $GODOT_BIN、包内 third_party、本机克隆目录；工程找 $RM_AIM_SIM_DIR 再找包内。
set -u

SECONDS_PER_RUN=${1:-12}
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"

CFG=${2:-$HERE/bench.cfg}

# 解释器
if [ -z "${PYTHON:-}" ]; then
  if [ -x "$ROOT/.venv/Scripts/python.exe" ]; then
    PYTHON="$ROOT/.venv/Scripts/python.exe"
  elif command -v python >/dev/null 2>&1; then
    PYTHON=python
  else
    echo "找不到 python" >&2; exit 2
  fi
fi

# 引擎
if [ -z "${GODOT_BIN:-}" ]; then
  for candidate in "$ROOT/third_party"/Godot*win64*.exe \
                   /f/rm_aim_sim_ws/godot/Godot*win64*_console.exe \
                   /f/rm_aim_sim_ws/godot/Godot*win64*.exe; do
    case "$candidate" in *console*) GODOT_BIN="$candidate"; break;; esac
    [ -f "$candidate" ] && GODOT_BIN="$candidate"
  done
fi
[ -x "${GODOT_BIN:-}" ] || { echo "找不到 Godot（设 GODOT_BIN）" >&2; exit 2; }

# 工程
if [ -z "${RM_AIM_SIM_DIR:-}" ]; then
  for candidate in "$ROOT/components/sim/rm_aim_sim" /f/rm_aim_sim_ws/rm_aim_sim; do
    [ -f "$candidate/project.godot" ] && RM_AIM_SIM_DIR="$candidate" && break
  done
fi
[ -f "${RM_AIM_SIM_DIR:-}/project.godot" ] || { echo "找不到 rm_aim_sim 工程（设 RM_AIM_SIM_DIR）" >&2; exit 2; }

INBOX="${TEMP:-/tmp}/rm_aim_sim_default"
echo "引擎 $GODOT_BIN"
echo "工程 $RM_AIM_SIM_DIR"
echo "场景 $CFG"

start_sim() {
  # 上一轮被 kill 时仿真来不及清理，state.json 会留在原处；先挪开，
  # 否则下面的等待会立刻返回，基准就跑在已经死掉的仿真上（统计变成负数）。
  if [ -f "$INBOX/state.json" ]; then mv "$INBOX/state.json" "$INBOX/state.stale"; fi
  "$GODOT_BIN" --headless --path "$RM_AIM_SIM_DIR" -- --remote --config="$CFG" \
    --quit-after-seconds=120 > /tmp/sim_sweep.log 2>&1 &
  SIM_PID=$!
  for _ in $(seq 1 80); do
    [ -f "$INBOX/state.json" ] && return 0
    sleep 0.25
  done
  echo "仿真没起来，看 /tmp/sim_sweep.log" >&2
  return 1
}

stop_sim() {
  [ -n "${SIM_PID:-}" ] && kill "$SIM_PID" 2>/dev/null
  wait "${SIM_PID:-}" 2>/dev/null
  SIM_PID=""
  sleep 0.4
}

run() {
  local label="$1"; shift
  start_sim || return 1
  sleep 1.2
  "$PYTHON" "$HERE/bench_hit_rate.py" --seconds "$SECONDS_PER_RUN" --label "$label" "$@"
  stop_sim
}

for i in 1 2 3; do
  run "默认参数 #$i"
done
run "门限 0.35"   --tolerance 0.35
run "窗口 0.16"   --window 0.16
run "无前馈"      --no-feedforward
