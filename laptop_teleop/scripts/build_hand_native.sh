#!/usr/bin/env bash
# Build the left-hand numerical kernel outside all running control loops.
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
HAND_BUILD_PYTHON="${PICO_PYTHON:-$(require_laptop_python)}"
PYTHONPATH="${LAPTOP_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}" \
    "${HAND_BUILD_PYTHON}" -m esrobo_laptop.hand_native
