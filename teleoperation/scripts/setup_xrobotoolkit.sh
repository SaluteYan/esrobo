#!/usr/bin/env bash
# Idempotently install XRoboToolkit PC Service and its Python client SDK.

set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
MINICONDA_DIR="${MINICONDA_DIR:-/home/esrobo/miniconda3}"
ENV_NAME="${ENV_NAME:-teleop_esrobo}"
PYTHON="${MINICONDA_DIR}/envs/${ENV_NAME}/bin/python"
PACKAGE=/tmp/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
PACKAGE_URL=https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
PACKAGE_SHA256=61961067eb4b41f81ed7cae35f4690dbb0ddfefb329a12b24e0b90ebc46ada91

if [[ "$(dpkg --print-architecture)" != "amd64" ]]; then
  echo "ERROR: this setup script is pinned to the official Ubuntu 22.04 amd64 package." >&2
  exit 1
fi

if ! dpkg-query -W -f='${Status}' roboticsservice 2>/dev/null | grep -q 'install ok installed'; then
  echo "==> Downloading official XRoboToolkit PC Service v1.0.0 for Ubuntu 22.04 amd64"
  curl -fL --retry 2 --connect-timeout 15 -o "${PACKAGE}" "${PACKAGE_URL}"
  echo "${PACKAGE_SHA256}  ${PACKAGE}" | sha256sum -c -
  sudo dpkg -i "${PACKAGE}"
else
  echo "==> XRoboToolkit PC Service already installed: $(dpkg-query -W -f='${Version}' roboticsservice)"
fi

if [[ ! -x "${PYTHON}" ]]; then
  echo "==> Creating ${ENV_NAME} conda environment"
  "${ROOT_DIR}/scripts/install_env.sh" "${ENV_NAME}"
fi

PYBIND_DIR="${ROOT_DIR}/external/XRoboToolkit-PC-Service-Pybind"
SDK_LIB="${PYBIND_DIR}/lib/libPXREARobotSDK.so"
if ! env LD_LIBRARY_PATH="${PYBIND_DIR}/lib:${MINICONDA_DIR}/envs/${ENV_NAME}/lib:${LD_LIBRARY_PATH:-}" \
  "${PYTHON}" -c 'import xrobotoolkit_sdk' 2>/dev/null; then
  echo "==> Building and installing xrobotoolkit_sdk"
  SDK_SOURCE="${ROOT_DIR}/external/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK"
  (cd "${SDK_SOURCE}" && bash build.sh)
  mkdir -p "${PYBIND_DIR}/lib"
  cp "${SDK_SOURCE}/build/libPXREARobotSDK.so" "${SDK_LIB}"
  "${PYTHON}" -m pip install pybind11
  (cd "${PYBIND_DIR}" && "${PYTHON}" -m pip install .)
else
  echo "==> xrobotoolkit_sdk already imports successfully"
fi

"${ROOT_DIR}/scripts/xrobotoolkit_pc_service.sh" start
env LD_LIBRARY_PATH="${PYBIND_DIR}/lib:${MINICONDA_DIR}/envs/${ENV_NAME}/lib:${LD_LIBRARY_PATH:-}" \
  "${PYTHON}" -c 'import xrobotoolkit_sdk; print("xrobotoolkit_sdk READY")'
echo "==> XRoboToolkit environment is ready. PICO body data is checked separately after the headset connects."
