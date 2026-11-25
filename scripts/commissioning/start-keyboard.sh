#!/usr/bin/env bash
# Run the Group 5 Pi's chassis and lift keyboard in a single SSH terminal.
set -eo pipefail
if [[ ! -t 0 || ! -t 1 ]]; then
    echo "请在交互终端启动遥控；使用远程命令时可加 ssh -t。" >&2
    exit 1
fi
source "${GROUP5_SESSION_ENV:-$HOME/dase7503-commissioning/session.env}"
nodes=$(ros2 node list --no-daemon --spin-time 2)
has_node() { grep -Fxq "$1" <<<"$nodes"; }
if has_node /robot_keyboard || has_node /lift_keyboard || has_node /keyboard_deadman_relay; then
    echo "已有键盘遥控运行，请先在原遥控终端退出。" >&2
    exit 1
fi
if has_node /ec_serial_bridge && has_node /ec_adapter; then
    echo "连接已有 EC 节点。若已有适配器仅允许升降，可在其启动终端退出后重新运行本命令。"
    exec ros2 run robot_bringup robot_keyboard
elif has_node /ec_serial_bridge || has_node /ec_adapter; then
    echo "EC 串口桥和适配器只发现一个，请先检查已有 EC 会话。" >&2
    exit 1
fi

log=$(mktemp -t group5-keyboard-ec.XXXXXX)
echo "启动 EC 串口桥与适配器。底盘由 A 键使能。日志：$log"
setsid env --default-signal=INT ros2 launch robot_bringup ec_bridge.launch.py \
    start_bridge:=true port:="${EC_PORT:?未配置 EC_PORT}" enable_motion:=true enable_lift:=true \
    </dev/null >"$log" 2>&1 &
ec_pid=$!
cleanup() {
    kill -INT -- "-$ec_pid" 2>/dev/null || true
    wait "$ec_pid" 2>/dev/null || true
}
trap cleanup EXIT
ros2 run robot_bringup robot_keyboard
