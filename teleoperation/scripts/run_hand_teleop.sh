#!/usr/bin/env bash
# SenseGlove hand-only teleoperation (fingers + IMU) for the LinkerHand.
#
# This runs the teleop core in hand-only mode: it listens on UDP 15050 for the
# SenseGlove bridge output (hand_joints), maps radians -> 0..255 with safety
# clamps, and forwards to UDP 15051 where `hand_ros_bridge.py` (system ROS2)
# publishes to the LinkerHand topics.
#
#   ./scripts/run_hand_teleop.sh                # press 'h' to enable hands
#   ./scripts/run_hand_teleop.sh --enable       # enable hands immediately
set -eu

ENV_NAME="${ENV_NAME:-teleop_esrobo}"
MINICONDA_DIR="${MINICONDA_DIR:-/home/esrobo/miniconda3}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PYTHON="${MINICONDA_DIR}/envs/${ENV_NAME}/bin/python"

export PYTHONPATH="${ROOT}/src:${PYTHONPATH:-}"
export ESROBO_TELEOP_ROOT="${ROOT}"

if [ ! -x "${PYTHON}" ]; then
    echo "ERROR: ${PYTHON} not found. Run scripts/install_env.sh first."
    exit 1
fi

cd "${ROOT}"
echo "==> hand-only teleop (SenseGlove -> LinkerHand)"
exec "${PYTHON}" -m esrobo_teleop.teleop_node \
    --hand-only --config "${ROOT}/config/teleop_config.yaml" "$@"
