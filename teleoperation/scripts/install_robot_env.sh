#!/usr/bin/env bash
# Install only the robot-side environment and local packages; never start hardware.
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
ROBOT_ENV_NAME="${ROBOT_ENV_NAME:-teleop_esrobo}"
ROBOT_CONDA="${CONDA_EXE:-${MINIFORGE_DIR}/bin/conda}"
if [[ ! -x "${ROBOT_CONDA}" ]]; then
    echo "未找到 Conda；请通过 CONDA_EXE 指定机器人端已有的 conda。" >&2
    exit 1
fi
if "${ROBOT_CONDA}" env list | awk '{print $1}' | grep -Fxq "${ROBOT_ENV_NAME}"; then
    "${ROBOT_CONDA}" env update -n "${ROBOT_ENV_NAME}" -f "${LAPTOP_ROOT}/environment.robot.yml"
else
    "${ROBOT_CONDA}" env create -n "${ROBOT_ENV_NAME}" -f "${LAPTOP_ROOT}/environment.robot.yml"
fi
"${ROBOT_CONDA}" run -n "${ROBOT_ENV_NAME}" python -m pip install --no-deps \
    -e "${LAPTOP_ROOT}" -e "${LAPTOP_ROOT}/../robot_link"
"${ROBOT_CONDA}" run -n "${ROBOT_ENV_NAME}" python -m pip install \
    "${PYAGXARM_PATH:-${LAPTOP_ROOT}/../src/pyAgxArm-master}"
"${ROBOT_CONDA}" run -n "${ROBOT_ENV_NAME}" python -c \
    'import numpy, scipy, can, yaml, pinocchio, pyAgxArm, esrobo_teleop, esrobo_link; print("robot environment OK")'
echo "机器人环境安装完成；ROS 驱动和网关仍需按 robot_link/README.md 显式启动。"
