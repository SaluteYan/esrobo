#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
exec /usr/bin/python3 "$ROOT/teleoperation/scripts/head_camera_web.py" "$@"
