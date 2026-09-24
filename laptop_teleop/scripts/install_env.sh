#!/usr/bin/env bash
# Install Miniforge when needed, create the laptop environment, and install local packages.
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"

CONDA_BIN="${CONDA_EXE:-${MINIFORGE_DIR}/bin/conda}"
if [[ ! -x "${CONDA_BIN}" ]]; then
    case "$(uname -m)" in
        x86_64) installer_arch=x86_64 ;;
        aarch64|arm64) installer_arch=aarch64 ;;
        *) echo "不支持的 CPU 架构：$(uname -m)" >&2; exit 1 ;;
    esac
    installer="$(mktemp --suffix=.sh)"
    trap 'rm -f "${installer:-}"' EXIT
    url="${MINIFORGE_URL:-https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-${installer_arch}.sh}"
    echo "下载 Miniforge：${url}"
    curl -fL --retry 3 -o "${installer}" "${url}"
    bash "${installer}" -b -p "${MINIFORGE_DIR}"
    CONDA_BIN="${MINIFORGE_DIR}/bin/conda"
fi

if "${CONDA_BIN}" env list | awk '{print $1}' | grep -Fxq "${CONDA_ENV_NAME}"; then
    echo "更新 Conda 环境：${CONDA_ENV_NAME}"
    "${CONDA_BIN}" env update -n "${CONDA_ENV_NAME}" -f "${LAPTOP_ROOT}/environment.yml" --prune
else
    echo "创建 Conda 环境：${CONDA_ENV_NAME}"
    "${CONDA_BIN}" env create -n "${CONDA_ENV_NAME}" -f "${LAPTOP_ROOT}/environment.yml"
fi

LAPTOP_PYTHON="${CONDA_ENV_PREFIX}/bin/python"
"${LAPTOP_PYTHON}" -m pip install -e "${LAPTOP_ROOT}[test]"
# Only reuse computation/communication modules; skip teleoperation's unrelated PyPI pink dependency.
"${LAPTOP_PYTHON}" -m pip install --no-deps \
    -e "${LAPTOP_ROOT}/../teleoperation" -e "${LAPTOP_ROOT}/../robot_link"

echo "环境安装完成。激活命令："
echo "  source ${MINIFORGE_DIR}/etc/profile.d/conda.sh && conda activate ${CONDA_ENV_NAME}"
"${LAPTOP_ROOT}/scripts/run_laptop.sh" check
