#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ "${EUID}" -eq 0 ]]; then
  echo "请使用普通用户执行，本脚本会在需要时调用 sudo。" >&2
  exit 1
fi

sudo apt update
sudo apt install -y \
  build-essential cmake python3-pip python3-venv python3-opencv \
  python3-numpy python3-pytest python3-scipy can-utils ethtool \
  libeigen3-dev libopencv-dev libpcl-dev libgflags-dev \
  libgoogle-glog-dev nlohmann-json3-dev libdw-dev libusb-1.0-0-dev \
  ros-humble-geometry-msgs ros-humble-nav-msgs ros-humble-sensor-msgs \
  ros-humble-tf2-ros ros-humble-tf2-eigen ros-humble-tf2-sensor-msgs \
  ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-image-transport-plugins ros-humble-compressed-image-transport \
  ros-humble-image-publisher ros-humble-camera-info-manager \
  ros-humble-diagnostic-updater ros-humble-diagnostic-msgs \
  ros-humble-statistics-msgs ros-humble-backward-ros \
  ros-humble-pcl-conversions ros-humble-pcl-ros \
  ros-humble-robot-state-publisher ros-humble-rviz2 \
  ros-humble-teleop-twist-keyboard

if ! python3 -c 'import torch' >/dev/null 2>&1; then
  python3 -m pip install --user \
    --index-url https://download.pytorch.org/whl/cpu torch torchvision
fi
python3 -m pip install --user --upgrade "ultralytics>=8.2,<9"

if ! python3 -c 'import piper_sdk' >/dev/null 2>&1; then
  python3 -m pip install --user piper_sdk
fi

NETSDK_WHEEL="${WS_DIR}/vendor/NetSDK-2.0.0.1-py3-none-linux_x86_64.whl"
if [[ ! -f "${NETSDK_WHEEL}" ]]; then
  echo "缺少大华 Linux NetSDK wheel：${NETSDK_WHEEL}" >&2
  exit 2
fi
python3 -m pip install --user --force-reinstall "${NETSDK_WHEEL}"
source "${WS_DIR}/scripts/netsdk_env.sh"
python3 -c 'from NetSDK.NetSDK import NetClient; sdk=NetClient(); assert hasattr(sdk, "sdk") and hasattr(sdk, "play_sdk"); assert sdk.InitEx() == 1; sdk.Cleanup(); print("NetSDK/PlaySDK 加载正常")'

echo "依赖安装完成。已有 PyTorch 不会被替换；新安装默认为 CPU 版。"
