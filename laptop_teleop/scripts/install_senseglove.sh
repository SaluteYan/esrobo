#!/usr/bin/env bash
# Install dependencies and build the official SenseGlove ROS 2 Humble workspace.
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
SENSEGLOVE_WS="${SENSEGLOVE_WS:-${LAPTOP_ROOT}/external/senseglove_ros_ws}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "未找到 /opt/ros/humble/setup.bash。请先按 docs/01_ENVIRONMENT_SETUP.md 安装 ROS 2 Humble。" >&2
    exit 1
fi

if [[ "${SENSEGLOVE_SKIP_APT:-0}" != "1" ]]; then
    sudo apt-get update
    sudo apt-get install -y \
        python3-colcon-common-extensions python3-rosdep python3-pyqt5 \
        ros-humble-ros2-control ros-humble-ros2-controllers \
        ros-humble-xacro ros-humble-robot-state-publisher \
        ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller \
        ros-humble-rmw-cyclonedds-cpp
fi
SENSEGLOVE_SKIP_APT=1 SENSEGLOVE_WS="${SENSEGLOVE_WS}" \
    "${LAPTOP_ROOT}/../teleoperation/scripts/install_senseglove_ros_humble.sh"

set +u
source /opt/ros/humble/setup.bash
source "${SENSEGLOVE_WS}/install/setup.bash"
set -u
ros2 pkg prefix senseglove_msgs >/dev/null
ros2 pkg prefix senseglove_bringup >/dev/null
echo "SenseGlove ROS 工作空间安装完成：${SENSEGLOVE_WS}"
