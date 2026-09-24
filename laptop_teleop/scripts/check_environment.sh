#!/usr/bin/env bash
set -u
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
failed=0
check() { if "$@" >/dev/null 2>&1; then printf '[OK]   %s\n' "$*"; else printf '[MISS] %s\n' "$*"; failed=1; fi; }
check test -x "${MINIFORGE_DIR}/bin/conda"
check test -x "${CONDA_ENV_PREFIX}/bin/python"
check test -f /opt/ros/humble/setup.bash
check dpkg-query -W roboticsservice
check test -x /opt/apps/roboticsservice/RoboticsServiceProcess
if [[ -x "${CONDA_ENV_PREFIX}/bin/python" ]]; then
    LD_LIBRARY_PATH="${LAPTOP_ROOT}/external/XRoboToolkit-PC-Service-Pybind/lib:${CONDA_ENV_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
        check "${CONDA_ENV_PREFIX}/bin/python" -c 'import numpy, scipy, yaml, pinocchio, xrobotoolkit_sdk'
fi
check test -f "${LAPTOP_ROOT}/external/senseglove_ros_ws/install/setup.bash"
check test -x "${LAPTOP_ROOT}/external/senseglove_ros_ws/src/senseglove_ros/senseglove_com/SenseCom/Linux/SenseCom_Linux_Latest/SenseCom.x86_64"
exit "${failed}"
