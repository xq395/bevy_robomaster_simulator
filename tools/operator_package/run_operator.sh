#!/usr/bin/env bash
# NHS 操作手终端 · Linux 启动脚本（比赛用：RM2026 制作规范 S87 要求开源操作系统）
#
#   ./run_operator.sh                 # 连官方裁判系统（MQTT 192.168.12.1:3333，图传 UDP 3334）
#   ./run_operator.sh --mock          # 本机 Mock 裁判系统，自测用
#   ./run_operator.sh --check         # 只检查依赖
#
# 参数取自 operator_config.json 的 rmu 段；环境变量 RMU_* 可覆盖。
# Linux 下不提供模拟器模式（图像桥接为 Windows 程序）。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP="$ROOT/app"
PYTHON="${PYTHON:-python3}"
MODE=rmu
CHECK=0
for arg in "$@"; do
    case "$arg" in
        --mock) MODE=rmu-mock ;;
        --check) CHECK=1 ;;
        -h|--help) sed -n '2,9p' "$0"; exit 0 ;;
        *) echo "错误：未知参数 $arg" >&2; exit 2 ;;
    esac
done

command -v "$PYTHON" >/dev/null || { echo "错误：未找到 $PYTHON（可用 PYTHON=/path/to/python3 指定）" >&2; exit 2; }

# 读取配置（用 Python 解析 JSON，避免依赖 jq）。
cfg() {
    "$PYTHON" - "$ROOT/operator_config.json" "$1" <<'EOF'
import json, sys
value = json.load(open(sys.argv[1], encoding="utf-8"))
for key in sys.argv[2].split("."):
    value = value[key]
print(value)
EOF
}

export RMU_ENABLED=1
export RMU_ROBOT_ID="${RMU_ROBOT_ID:-$(cfg rmu.robot_id)}"
export RMU_MQTT_PORT="${RMU_MQTT_PORT:-$(cfg rmu.mqtt_port)}"
export RMU_VIDEO_PORT="${RMU_VIDEO_PORT:-$(cfg rmu.video_port)}"
if [ "$MODE" = rmu-mock ]; then
    export RMU_MQTT_HOST=127.0.0.1 RMU_VIDEO_HOST=127.0.0.1
else
    export RMU_MQTT_HOST="${RMU_MQTT_HOST:-$(cfg rmu.mqtt_host)}"
    export RMU_VIDEO_HOST="${RMU_VIDEO_HOST:-$(cfg rmu.video_host)}"
fi
export DAEDALUS_OPERATOR_PORT="${DAEDALUS_OPERATOR_PORT:-$(cfg operator_port)}"

echo "NHS 操作手终端（$MODE）"
echo "  裁判系统 : MQTT $RMU_MQTT_HOST:$RMU_MQTT_PORT  clientID=$RMU_ROBOT_ID"
echo "  图传     : UDP $RMU_VIDEO_HOST:$RMU_VIDEO_PORT"

if ! (cd "$APP" && "$PYTHON" -c "import flask, PIL, av, paho.mqtt.client, google.protobuf, rmu_protocol.rm_custom_client_pb2") ; then
    echo "错误：缺少 Python 依赖（或 protobuf 低于 6.32.1）。请运行：" >&2
    echo "  $PYTHON -m pip install -r \"$ROOT/requirements.txt\"" >&2
    exit 3
fi
if [ "$CHECK" = 1 ]; then echo "依赖检查通过（未启动服务）。"; exit 0; fi

PIDS=()
cleanup() {
    for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
    wait 2>/dev/null || true
    echo "服务已停止。"
}
trap cleanup EXIT INT TERM

cd "$APP"
if [ "$MODE" = rmu-mock ]; then
    "$PYTHON" -m rmu_protocol.mock_engine --robot-id "$RMU_ROBOT_ID" --mqtt-host 127.0.0.1 \
        --mqtt-port "$RMU_MQTT_PORT" --video-to "127.0.0.1:$RMU_VIDEO_PORT" &
    PIDS+=($!)
    echo "Mock 裁判系统已启动 (PID $!)"
    sleep 1
fi
"$PYTHON" operator_web.py &
PIDS+=($!)
echo "操作手服务已启动 (PID $!)"
echo ""
echo "操作手页面： http://127.0.0.1:$DAEDALUS_OPERATOR_PORT/   （视频源选“裁判系统图传”）"
echo "按 Ctrl+C 结束全部服务。"

# 任一进程退出即整体退出。
wait -n "${PIDS[@]}"
echo "有进程已退出，正在停止其余服务。" >&2
