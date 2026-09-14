#!/usr/bin/env bash
# Build the external teleoperation SDKs inside this teleoperation folder.
#   * XRoboToolkit-PC-Service -> xrobotoolkit_sdk (PICO 全身骨骼)
#   * senseglove_ros (humble-dev @ a14a468) -> full workspace, native on Ubuntu 22.04
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_NAME="${ENV_NAME:-teleop_esrobo}"
MINICONDA_DIR="${MINICONDA_DIR:-/home/esrobo/miniconda3}"
CONDA_PYTHON="${MINICONDA_DIR}/envs/${ENV_NAME}/bin/python"
SENSEGLOVE_WS="${ROOT_DIR}/external/senseglove_ros"
SENSEGLOVE_COMMIT="a14a468"   # humble-dev "hardware interface: humble port fixes"

echo "==> [1/3] XRoboToolkit-PC-Service -> libPXREARobotSDK.so"
(
  cd "${ROOT_DIR}/external/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK"
  bash build.sh
)

echo "==> [2/3] xrobotoolkit_sdk (pybind11) into conda env"
PYBIND_DIR="${ROOT_DIR}/external/XRoboToolkit-PC-Service-Pybind"
mkdir -p "${PYBIND_DIR}/lib"
cp "${ROOT_DIR}/external/XRoboToolkit-PC-Service/RoboticsService/SDK/linux/64/libPXREARobotSDK.so" "${PYBIND_DIR}/lib/"
export PYTHONPATH=""
"${CONDA_PYTHON}" -m pip install -q pybind11
( cd "${PYBIND_DIR}" && "${CONDA_PYTHON}" -m pip install . )

echo "==> [3/3] senseglove_ros (humble-dev @ ${SENSEGLOVE_COMMIT}) full build"
(
  cd "${SENSEGLOVE_WS}"
  git fetch origin humble-dev >/dev/null 2>&1 || true
  git checkout "${SENSEGLOVE_COMMIT}" >/dev/null 2>&1 || true
  rm -rf build install log
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install
)

echo
echo "==> verify xrobotoolkit_sdk"
"${CONDA_PYTHON}" -c "import xrobotoolkit_sdk; print('xrobotoolkit_sdk OK')"

echo "==> verify senseglove_msgs"
source /opt/ros/humble/setup.bash
source "${SENSEGLOVE_WS}/install/setup.bash"
python3 -c "from senseglove_msgs.msg import SenseGloveState; print('senseglove_msgs OK')"

echo
echo "NOTE: senseglove_ros is pinned to humble-dev@${SENSEGLOVE_COMMIT} whose x86-64"
echo "      libsgcore.so only requires glibc 2.34 -- native on Ubuntu 22.04 (glibc 2.35)."
