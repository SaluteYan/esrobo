#!/usr/bin/env bash

LAPTOP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
USER_HOME_DIR="$(getent passwd "$(id -u)" | cut -d: -f6)"
MINIFORGE_DIR="${MINIFORGE_DIR:-${USER_HOME_DIR}/miniforge3}"
CONDA_ENV_NAME="${CONDA_ENV_NAME:-esrobo_laptop}"
CONDA_ENV_PREFIX="${CONDA_ENV_PREFIX:-${MINIFORGE_DIR}/envs/${CONDA_ENV_NAME}}"
# Avoid silently importing similarly named packages from ~/.local into the Conda environment.
export PYTHONNOUSERSITE=1

if [[ -x "${CONDA_ENV_PREFIX}/bin/python" ]]; then
    DEFAULT_LAPTOP_PYTHON="${CONDA_ENV_PREFIX}/bin/python"
else
    DEFAULT_LAPTOP_PYTHON=""
fi

require_laptop_python() {
    local candidate="${LAPTOP_PYTHON:-${DEFAULT_LAPTOP_PYTHON}}"
    if [[ -z "${candidate}" || ! -x "${candidate}" ]]; then
        echo "未找到 Conda 环境 ${CONDA_ENV_NAME}，请先运行 ${LAPTOP_ROOT}/scripts/install_env.sh" >&2
        return 1
    fi
    printf '%s\n' "${candidate}"
}
