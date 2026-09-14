#!/usr/bin/env bash
# Create the conda environment for ESROBO real-machine teleoperation and
# install the required packages (including pyAgxArm from the ESROBO workspace).
#
# Usage:  ./scripts/install_env.sh [--env-name teleop_esrobo]
set -eu

ENV_NAME="${1:-teleop_esrobo}"
MINICONDA_DIR="${MINICONDA_DIR:-/home/esrobo/miniconda3}"
CONDA_BIN="${MINICONDA_DIR}/bin/conda"
PYAGXARM_PATH="${PYAGXARM_PATH:-/home/esrobo/Projects/esrobo/src/pyAgxArm-master}"

echo "==> conda: ${CONDA_BIN}  env: ${ENV_NAME}"

if [ ! -x "${CONDA_BIN}" ]; then
    echo "ERROR: conda not found at ${CONDA_BIN}. Install Miniconda first or set MINICONDA_DIR."
    exit 1
fi

# Accept Anaconda channel ToS so conda-forge installs don't block.
"${CONDA_BIN}" tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main >/dev/null 2>&1 || true
"${CONDA_BIN}" tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r >/dev/null 2>&1 || true

if ! "${CONDA_BIN}" env list | grep -q "${ENV_NAME}"; then
    echo "==> creating env ${ENV_NAME}"
    "${CONDA_BIN}" create -n "${ENV_NAME}" -c conda-forge python=3.10 numpy scipy pyyaml python-can pinocchio -y
fi

PYTHON="${MINICONDA_DIR}/envs/${ENV_NAME}/bin/python"
PIP="${MINICONDA_DIR}/envs/${ENV_NAME}/bin/pip"

echo "==> pip installing pink / qpsolvers / proxqp"
"${PIP}" install --upgrade pip >/dev/null 2>&1
"${PIP}" install pink qpsolvers proxsuite proxqp

echo "==> installing pyAgxArm SDK from ${PYAGXARM_PATH}"
if [ -d "${PYAGXARM_PATH}" ]; then
    "${PIP}" install "${PYAGXARM_PATH}"
else
    echo "WARN: pyAgxArm not found at ${PYAGXARM_PATH}; set PYAGXARM_PATH."
fi

echo "==> verifying imports"
export PYTHONPATH=""
"${PYTHON}" -c "import numpy, scipy, can, yaml, pinocchio, pink; from pink import solve_ik; import pyAgxArm; print('env OK')"

echo
echo "Activate with:  source ${MINICONDA_DIR}/etc/profile.d/conda.sh && conda activate ${ENV_NAME}"
