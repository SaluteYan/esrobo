#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
SENSEGLOVE_WS="${SENSEGLOVE_WS:-${LAPTOP_ROOT}/external/senseglove_ros_ws}"
# The SenseGlove ROS driver and SenseCom run on this laptop; no LinkerHand/CAN launcher is used.
if [[ ! -f "${SENSEGLOVE_WS}/install/setup.bash" ]]; then
    echo "SenseGlove ROS 工作空间尚未构建，请先运行 ${LAPTOP_ROOT}/scripts/install_senseglove.sh" >&2
    exit 1
fi
set +u
source /opt/ros/humble/setup.bash
source "${SENSEGLOVE_WS}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION="${SENSEGLOVE_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export PYTHONPATH="${LAPTOP_ROOT}/src:${LAPTOP_ROOT}/../teleoperation/src:${LAPTOP_ROOT}/../robot_link${PYTHONPATH:+:${PYTHONPATH}}"
exec "${SENSEGLOVE_PYTHON:-/usr/bin/python3}" -m esrobo_laptop.acquisition senseglove \
    --host 127.0.0.1 --port "${INPUT_PORT:-15050}" --max-state-age 0.1 \
    --calibration-file "${LAPTOP_ROOT}/config/senseglove_calibration.json" "$@"
