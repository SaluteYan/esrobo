#!/usr/bin/env bash
# Write the two physical glove serial numbers used by senseglove_bringup.
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
[[ $# -eq 2 ]] || { echo "用法：$0 <左手套序列号> <右手套序列号>" >&2; exit 2; }
[[ "$1" =~ ^[A-Za-z0-9._-]+$ && "$2" =~ ^[A-Za-z0-9._-]+$ ]] || { echo "序列号格式无效。" >&2; exit 2; }
SENSEGLOVE_WS="${SENSEGLOVE_WS:-${LAPTOP_ROOT}/external/senseglove_ros_ws}"
CFG="${SENSEGLOVE_WS}/src/senseglove_ros/senseglove/senseglove_bringup/config/gloves.yaml"
[[ -f "${CFG}" ]] || { echo "未找到 ${CFG}，请先安装 SenseGlove ROS。" >&2; exit 1; }
cp -n "${CFG}" "${CFG}.original" || true
cat >"${CFG}" <<EOF
gloves:
  - type: nova2
    side: left
    serial: "$1"
    finger_distance: false
  - type: nova2
    side: right
    serial: "$2"
    finger_distance: false
EOF
echo "已写入 ${CFG}"
