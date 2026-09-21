#!/usr/bin/env bash
# Dedicated launcher for linked PICO right arm + SenseGlove/LinkerHand control.

set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
exec "${ROOT_DIR}/scripts/run_pico_right_arm_teleop.sh" --with-right-imu "$@"
