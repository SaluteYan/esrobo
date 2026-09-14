#!/usr/bin/env bash
# Stream official SenseGlove ROS 2 states to IsaacLab ESROBO hand-joint UDP targets.

set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
SENSEGLOVE_WS="${SENSEGLOVE_WS:-${ROOT_DIR}/external/senseglove_ros_ws}"
UDP_HOST="${ESROBO_BODY_UDP_HOST:-127.0.0.1}"
UDP_PORT="${ESROBO_BODY_UDP_PORT:-15050}"
export ESROBO_HAND_RETARGETING_MODE="${ESROBO_HAND_RETARGETING_MODE:-vector}"

clean_path_var() {
    local raw="${1:-}"
    local cleaned=""
    local entry
    IFS=':' read -r -a entries <<< "${raw}"
    for entry in "${entries[@]}"; do
        [[ -z "${entry}" ]] && continue
        if [[ -n "${CONDA_PREFIX:-}" && "${entry}" == "${CONDA_PREFIX}"* ]]; then
            continue
        fi
        if [[ "${entry}" == "${HOME}/anaconda3"* || "${entry}" == "${HOME}/miniconda3"* ]]; then
            continue
        fi
        if [[ -z "${cleaned}" ]]; then
            cleaned="${entry}"
        else
            cleaned="${cleaned}:${entry}"
        fi
    done
    printf '%s' "${cleaned}"
}

if [[ -n "${CONDA_PREFIX:-}" ]]; then
    echo "[senseglove_bridge] Active conda environment detected: ${CONDA_PREFIX}"
    echo "[senseglove_bridge] ROS 2 Humble bridge will use system /usr/bin/python3 and clean conda library paths."
    export LD_LIBRARY_PATH="$(clean_path_var "${LD_LIBRARY_PATH:-}")"
    export PYTHONPATH="$(clean_path_var "${PYTHONPATH:-}")"
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "[senseglove_bridge] ROS 2 Humble was not found at /opt/ros/humble/setup.bash." >&2
    exit 1
fi

set +u
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
set -u

if [[ -f "${SENSEGLOVE_WS}/install/setup.bash" ]]; then
    set +u
    # shellcheck source=/dev/null
    source "${SENSEGLOVE_WS}/install/setup.bash"
    set -u
else
    echo "[senseglove_bridge] Missing SenseGlove ROS 2 overlay: ${SENSEGLOVE_WS}/install/setup.bash" >&2
    echo "[senseglove_bridge] Build it first: ./scripts/install_senseglove_ros_humble.sh" >&2
    exit 1
fi

export PYTHONUNBUFFERED=1

exec /usr/bin/python3 -u "${ROOT_DIR}/scripts/senseglove_ros_to_esrobo_hand_bridge.py" \
    --host "${UDP_HOST}" \
    --port "${UDP_PORT}" \
    "$@"
