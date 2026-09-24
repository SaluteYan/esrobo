#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
LAPTOP_PYTHON="$(require_laptop_python)"
export PYTHONPATH="${LAPTOP_ROOT}/src:${LAPTOP_ROOT}/../teleoperation/src:${LAPTOP_ROOT}/../robot_link${PYTHONPATH:+:${PYTHONPATH}}"
exec "${LAPTOP_PYTHON}" -m esrobo_laptop.app "$@"
