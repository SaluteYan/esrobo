#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
"${LAPTOP_ROOT}/scripts/run_laptop.sh" check "$@"
"${LAPTOP_ROOT}/scripts/xrobotoolkit_service.sh" status
