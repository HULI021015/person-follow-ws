#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"
set -u
source "${WS_DIR}/scripts/netsdk_env.sh"

if [[ -z "${DAHUA_IP:-}" ]]; then
  if ! timeout 1 bash -c '</dev/tcp/192.168.1.21/37777' 2>/dev/null; then
    WSL_GATEWAY="$(awk '/^nameserver / {print $2; exit}' /etc/resolv.conf)"
    if [[ -n "${WSL_GATEWAY}" ]] && timeout 1 bash -c "</dev/tcp/${WSL_GATEWAY}/37778" 2>/dev/null; then
      export DAHUA_IP="${WSL_GATEWAY}"
      export DAHUA_PORT="37778"
      echo "相机经 Windows 转发连接：${DAHUA_IP}:${DAHUA_PORT}"
    else
      echo "警告：WSL 不能直连相机，且未检测到 Windows 37778 转发。" >&2
      echo "请先在 Windows PowerShell 运行 scripts\\start_camera_relay_windows.ps1。" >&2
    fi
    unset WSL_GATEWAY
  fi
fi

exec ros2 launch person_follower follow.launch.py "$@"
