#!/usr/bin/env bash
# Install XRoboToolkit PC Service and its Python binding for the laptop environment.
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"

PACKAGE="/tmp/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb"
PACKAGE_URL="https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb"
PACKAGE_SHA256="61961067eb4b41f81ed7cae35f4690dbb0ddfefb329a12b24e0b90ebc46ada91"
SERVICE_REPO="${LAPTOP_ROOT}/external/XRoboToolkit-PC-Service"
PYBIND_REPO="${LAPTOP_ROOT}/external/XRoboToolkit-PC-Service-Pybind"
PYTHON="$(require_laptop_python)"

if [[ "$(dpkg --print-architecture)" != amd64 ]]; then
    echo "官方 PICO PC Service v1.0.0 安装包仅支持 Ubuntu amd64。" >&2
    exit 1
fi
if ! dpkg-query -W -f='${Status}' roboticsservice 2>/dev/null | grep -q 'install ok installed'; then
    [[ -f "${PACKAGE}" ]] || curl -fL --retry 3 -o "${PACKAGE}" "${PACKAGE_URL}"
    echo "${PACKAGE_SHA256}  ${PACKAGE}" | sha256sum -c -
    sudo dpkg -i "${PACKAGE}"
fi

mkdir -p "${LAPTOP_ROOT}/external"
if [[ ! -d "${SERVICE_REPO}/.git" ]]; then
    git clone --depth 1 https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git "${SERVICE_REPO}"
fi
if [[ ! -d "${PYBIND_REPO}/.git" ]]; then
    git clone --depth 1 https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind.git "${PYBIND_REPO}"
fi

SDK_SOURCE="${SERVICE_REPO}/RoboticsService/PXREARobotSDK"
"${PYTHON}" "${LAPTOP_ROOT}/scripts/patch_pico_sdk.py" "${PYBIND_REPO}"
PATH="${CONDA_ENV_PREFIX}/bin:${PATH}" cmake -S "${SDK_SOURCE}" -B "${SDK_SOURCE}/build" -DCMAKE_BUILD_TYPE=Release
PATH="${CONDA_ENV_PREFIX}/bin:${PATH}" cmake --build "${SDK_SOURCE}/build" --parallel
mkdir -p "${PYBIND_REPO}/lib"
cp "${SDK_SOURCE}/build/libPXREARobotSDK.so" "${PYBIND_REPO}/lib/"

(cd "${PYBIND_REPO}" && \
    CMAKE_PREFIX_PATH="${CONDA_ENV_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}" \
    "${PYTHON}" -m pip install --no-build-isolation .)

env LD_LIBRARY_PATH="${PYBIND_REPO}/lib:${CONDA_ENV_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
    "${PYTHON}" -c 'import xrobotoolkit_sdk; print("xrobotoolkit_sdk: READY")'
echo "PICO 软件安装完成。服务启动：${LAPTOP_ROOT}/scripts/xrobotoolkit_service.sh start"
