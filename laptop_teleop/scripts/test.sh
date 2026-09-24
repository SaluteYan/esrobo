#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
LAPTOP_PYTHON="${LAPTOP_PYTHON:-$(require_laptop_python)}"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH="${LAPTOP_ROOT}/src:${LAPTOP_ROOT}/../teleoperation/src:${LAPTOP_ROOT}/../robot_link:${LAPTOP_ROOT}/../teleoperation:${LAPTOP_ROOT}/../teleoperation/tests"
exec "${LAPTOP_PYTHON}" -m pytest -q \
    "${LAPTOP_ROOT}/tests" "${LAPTOP_ROOT}/../robot_link/tests" \
    "${LAPTOP_ROOT}/../teleoperation/tests/test_partitioned_position_ik.py" \
    -k 'not trajectory_collision and not expanded_j4_session' "$@"
