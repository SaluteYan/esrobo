#!/usr/bin/env bash
# Stream PICO full-body tracking (XRoboToolkit) to the ESROBO teleop core over UDP.
#
# Prereq: xrobotoolkit_sdk built (see README) and PICO PC Service + PICO app running.
#   ./bridges/run_xrobotoolkit_bridge.sh [--rate-hz 90]
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_NAME="${ENV_NAME:-teleop_esrobo}"
MINICONDA_DIR="${MINICONDA_DIR:-/home/esrobo/miniconda3}"
UDP_HOST="${ESROBO_BODY_UDP_HOST:-127.0.0.1}"
UDP_PORT="${ESROBO_BODY_UDP_PORT:-15050}"
RATE_HZ="${XROBOTOOLKIT_BODY_RATE_HZ:-90}"
VIEWER_HOST="${PICO_SKELETON_VIEWER_HOST:-127.0.0.1}"
VIEWER_PORT="${PICO_SKELETON_VIEWER_PORT:-8765}"
PYBIND_DIR="${ROOT_DIR}/external/XRoboToolkit-PC-Service-Pybind"
PYTHON="${MINICONDA_DIR}/envs/${ENV_NAME}/bin/python"

if [ ! -x "${PYTHON}" ]; then
    echo "ERROR: ${PYTHON} not found. Build the conda env first (scripts/install_env.sh)."
    exit 1
fi

export PYTHONPATH="${ROOT_DIR}/src:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="${PYBIND_DIR}/lib:${MINICONDA_DIR}/envs/${ENV_NAME}/lib:${LD_LIBRARY_PATH:-}"

echo "==> streaming PICO body -> ${UDP_HOST}:${UDP_PORT} (rate ${RATE_HZ} Hz)"
echo "==> read-only skeleton viewer -> http://${VIEWER_HOST}:${VIEWER_PORT}"
exec "${PYTHON}" -u "${ROOT_DIR}/bridges/xrobotoolkit_body_udp_bridge.py" \
    --host "${UDP_HOST}" --port "${UDP_PORT}" --rate-hz "${RATE_HZ}" \
    --viewer-host "${VIEWER_HOST}" --viewer-port "${VIEWER_PORT}" "$@"
