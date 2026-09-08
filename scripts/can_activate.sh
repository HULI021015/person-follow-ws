#!/usr/bin/env bash
set -euo pipefail

CAN_PORT="${1:-can0}"
BITRATE="${2:-1000000}"

if ! [[ "${CAN_PORT}" =~ ^can[0-9]+$ ]]; then
  echo "CAN 名称无效：${CAN_PORT}（示例：can0）" >&2
  exit 2
fi
if ! [[ "${BITRATE}" =~ ^[0-9]+$ ]]; then
  echo "CAN 波特率无效：${BITRATE}" >&2
  exit 2
fi
if ! command -v ip >/dev/null || ! command -v ethtool >/dev/null; then
  echo "缺少 iproute2/ethtool，请先运行 scripts/install_dependencies.sh" >&2
  exit 3
fi

mapfile -t CANDIDATES < <(ip -o link show | awk -F': ' '$2 ~ /^can[0-9]+$/ {print $2}')
if ! ip link show "${CAN_PORT}" >/dev/null 2>&1; then
  echo "未发现 ${CAN_PORT}。当前 CAN 接口：${CANDIDATES[*]:-无}" >&2
  echo "请检查 USB-CAN 设备是否已连接到 Ubuntu/WSL。" >&2
  exit 4
fi

CAN_INFO="$(ip -details link show "${CAN_PORT}")"
if printf '%s\n' "${CAN_INFO}" | head -n 1 | grep -q 'UP' \
  && printf '%s\n' "${CAN_INFO}" | grep -q "bitrate ${BITRATE}"; then
  echo "${CAN_PORT} 已是 UP，bitrate=${BITRATE}，无需重新配置。"
  exit 0
fi

sudo ip link set "${CAN_PORT}" down 2>/dev/null || true
sudo ip link set "${CAN_PORT}" type can bitrate "${BITRATE}"
sudo ip link set "${CAN_PORT}" txqueuelen 1000
sudo ip link set "${CAN_PORT}" up

ip -details link show "${CAN_PORT}"
echo "${CAN_PORT} 已启用，bitrate=${BITRATE}"
