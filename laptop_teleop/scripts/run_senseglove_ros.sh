#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
SENSEGLOVE_WS="${SENSEGLOVE_WS:-${LAPTOP_ROOT}/external/senseglove_ros_ws}"
[[ -f "${SENSEGLOVE_WS}/install/setup.bash" ]] || { echo "请先运行 install_senseglove.sh" >&2; exit 1; }
set +u
source /opt/ros/humble/setup.bash
source "${SENSEGLOVE_WS}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION="${SENSEGLOVE_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
exec ros2 launch senseglove_bringup senseglove.launch.py run_rviz:=false run_sensecom:=false "$@"
