#!/usr/bin/env bash
# Launch ESROBO real-machine teleoperation in the conda env.
#
#   ./scripts/run_teleop.sh                  # uses config/teleop_config.yaml
#   ./scripts/run_teleop.sh --no-robot       # offline: IK/hand only, no arm command
#   ./scripts/run_teleop.sh --enable         # arm the robot immediately (careful)
set -eu

ENV_NAME="${ENV_NAME:-teleop_esrobo}"
MINICONDA_DIR="${MINICONDA_DIR:-/home/esrobo/miniconda3}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PYTHON="${MINICONDA_DIR}/envs/${ENV_NAME}/bin/python"

export PYTHONPATH="${ROOT}/src:${PYTHONPATH:-}"
export ESROBO_TELEOP_ROOT="${ROOT}"

if [ ! -x "${PYTHON}" ]; then
    echo "ERROR: ${PYTHON} not found. Run scripts/install_env.sh first."
    exit 1
fi

cd "${ROOT}"
echo "==> teleop root: ${ROOT}"
exec "${PYTHON}" -m esrobo_teleop.teleop_node \
    --config "${ROOT}/config/teleop_config.yaml" "$@"
