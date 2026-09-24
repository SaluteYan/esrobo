#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
PICO_PYTHON="${PICO_PYTHON:-$(require_laptop_python)}"
PYBIND_LIB="${LAPTOP_ROOT}/external/XRoboToolkit-PC-Service-Pybind/lib"
export LD_LIBRARY_PATH="${PYBIND_LIB}:${CONDA_ENV_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export PYTHONPATH="${LAPTOP_ROOT}/src:${LAPTOP_ROOT}/../teleoperation/src:${LAPTOP_ROOT}/../robot_link${PYTHONPATH:+:${PYTHONPATH}}"
exec "${PICO_PYTHON}" -m esrobo_laptop.acquisition pico --host 127.0.0.1 --port "${INPUT_PORT:-15050}" --viewer-host 127.0.0.1 "$@"
