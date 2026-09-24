#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
SENSEGLOVE_WS="${SENSEGLOVE_WS:-${LAPTOP_ROOT}/external/senseglove_ros_ws}"
BIN="${SENSEGLOVE_WS}/src/senseglove_ros/senseglove_com/SenseCom/Linux/SenseCom_Linux_Latest/SenseCom.x86_64"
[[ -x "${BIN}" ]] || { echo "未找到 SenseCom：${BIN}" >&2; exit 1; }
cd "$(dirname "${BIN}")"
exec ./SenseCom.x86_64 "$@"
