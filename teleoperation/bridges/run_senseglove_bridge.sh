#!/usr/bin/env bash
# Stream SenseGlove Nova 2 hand joints + IMU (from SenseGlove ROS2 topics) to
# the ESROBO teleop core over UDP port 15050.
#
# REQUIRES the SenseGlove ROS2 hardware driver running (senseglove_bringup),
# publishing /senseglove/glove<serial>/{lh|rh}/senseglove_states.
#
# Workspace pin: senseglove_ros on branch `humble-dev` @ commit a14a468
# ("hardware interface: humble port fixes").  Its prebuilt x86-64 libsgcore.so
# only requires glibc 2.34, so it builds/runs natively on Ubuntu 22.04 (glibc 2.35).
#
#   ./bridges/run_senseglove_bridge.sh \
#       --left-serial <serial> --right-serial <serial>
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SENSEGLOVE_WS="${ROOT_DIR}/external/senseglove_ros"
UDP_HOST="${ESROBO_BODY_UDP_HOST:-127.0.0.1}"
UDP_PORT="${ESROBO_BODY_UDP_PORT:-15050}"

source /opt/ros/humble/setup.bash
if [ -f "${SENSEGLOVE_WS}/install/setup.bash" ]; then
    # shellcheck source=/dev/null
    source "${SENSEGLOVE_WS}/install/setup.bash"
fi

export PYTHONPATH="${ROOT_DIR}/bridges:${PYTHONPATH:-}"
echo "==> SenseGlove bridge -> ${UDP_HOST}:${UDP_PORT}"
echo "==> (run the hardware driver first: ros2 launch senseglove_bringup senseglove.launch.py)"
exec python3 -u "${ROOT_DIR}/bridges/senseglove_ros_to_esrobo_hand_bridge.py" \
    --host "${UDP_HOST}" --port "${UDP_PORT}" "$@"
