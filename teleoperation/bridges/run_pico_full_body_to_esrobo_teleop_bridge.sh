#!/usr/bin/env bash
# Stream PICO full-body shoulder/elbow/wrist tracking to IsaacLab ESROBO bodytracking_udp teleop.

set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
ENV_NAME="${PICO_FULL_BODY_ENV_NAME:-env_isaaclab}"
UDP_HOST="${ESROBO_BODY_UDP_HOST:-127.0.0.1}"
PORT="${ESROBO_BODY_UDP_PORT:-15050}"
RATE_HZ="${PICO_FULL_BODY_RATE_HZ:-60}"

if [[ -f "${HOME}/anaconda3/etc/profile.d/conda.sh" ]]; then
    # shellcheck source=/dev/null
    source "${HOME}/anaconda3/etc/profile.d/conda.sh"
elif [[ -f "${HOME}/miniconda3/etc/profile.d/conda.sh" ]]; then
    # shellcheck source=/dev/null
    source "${HOME}/miniconda3/etc/profile.d/conda.sh"
fi

conda activate "${ENV_NAME}"
if [[ "${TERM:-}" == "dumb" || -z "${TERM:-}" ]]; then
    export TERM=xterm-256color
fi
export PYTHONUNBUFFERED=1

EXTRA_ARGS=("$@")
HAS_RATE_HZ=0
for arg in "${EXTRA_ARGS[@]}"; do
    if [[ "${arg}" == "--rate-hz" || "${arg}" == --rate-hz=* ]]; then
        HAS_RATE_HZ=1
        break
    fi
done
if [[ "${HAS_RATE_HZ}" -eq 0 ]]; then
    EXTRA_ARGS=(--rate-hz "${RATE_HZ}" "${EXTRA_ARGS[@]}")
fi

exec python -u "${ROOT_DIR}/scripts/pico_full_body_udp_bridge.py" \
    --host "${UDP_HOST}" \
    --port "${PORT}" \
    "${EXTRA_ARGS[@]}"
