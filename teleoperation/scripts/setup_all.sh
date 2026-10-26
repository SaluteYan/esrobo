#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"${SCRIPT_DIR}/install_env.sh"
"${SCRIPT_DIR}/install_pico.sh"
"${SCRIPT_DIR}/install_senseglove.sh"
"${SCRIPT_DIR}/check_environment.sh"
