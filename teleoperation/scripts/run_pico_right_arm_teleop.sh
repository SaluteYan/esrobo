#!/usr/bin/env bash
# Right-arm entry point for the shared safe single-arm PICO launcher.

set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
exec "${ROOT_DIR}/scripts/run_pico_left_arm_teleop.sh" --arm-side right "$@"
