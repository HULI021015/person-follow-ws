#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"
set -u
source "${WS_DIR}/scripts/netsdk_env.sh"

# 网页后台无法可靠显示 sudo 密码提示，因此在控制台启动前完成 Piper
# SocketCAN 配置。接口尚未连接时只提示，不影响单独使用小车跟随。
PIPER_CAN_PORT="${PIPER_CAN_PORT:-can0}"
PIPER_CAN_BITRATE="${PIPER_CAN_BITRATE:-1000000}"
if [[ "${AUTO_SETUP_PIPER_CAN:-1}" != "0" ]]; then
  if ip link show "${PIPER_CAN_PORT}" >/dev/null 2>&1; then
    echo "正在检查并配置 Piper CAN：${PIPER_CAN_PORT}"
    "${WS_DIR}/scripts/can_activate.sh" \
      "${PIPER_CAN_PORT}" "${PIPER_CAN_BITRATE}"
  else
    echo "提示：未发现 ${PIPER_CAN_PORT}，网页仍会启动；机械臂按钮暂不可用。" >&2
    echo "连接 USB-CAN 后请重新运行 start_dashboard.sh。" >&2
  fi
fi

exec ros2 run robot_dashboard dashboard_node --ros-args \
  -p workspace_root:="${WS_DIR}" "$@"
