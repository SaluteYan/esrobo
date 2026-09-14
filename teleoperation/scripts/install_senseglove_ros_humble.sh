#!/usr/bin/env bash
# Install and build the official SenseGlove ROS 2 Humble workspace.

set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
SENSEGLOVE_WS="${SENSEGLOVE_WS:-${ROOT_DIR}/external/senseglove_ros_ws}"
SENSEGLOVE_REPO="${SENSEGLOVE_REPO:-https://github.com/Adjuvo/senseglove_ros.git}"
SENSEGLOVE_BRANCH="${SENSEGLOVE_BRANCH:-humble-dev}"
REPO_DIR="${SENSEGLOVE_WS}/src/senseglove_ros"

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

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "[senseglove_install] ROS 2 Humble was not found at /opt/ros/humble/setup.bash." >&2
    exit 1
fi

if [[ -n "${CONDA_PREFIX:-}" ]]; then
    echo "[senseglove_install] Active conda environment detected: ${CONDA_PREFIX}"
    echo "[senseglove_install] Building ROS 2 Humble packages with system /usr/bin/python3."
fi
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:$(clean_path_var "${PATH:-}")"
export LD_LIBRARY_PATH="$(clean_path_var "${LD_LIBRARY_PATH:-}")"
export PYTHONPATH="$(clean_path_var "${PYTHONPATH:-}")"

echo "[senseglove_install] Workspace: ${SENSEGLOVE_WS}"
echo "[senseglove_install] Repository: ${SENSEGLOVE_REPO} (${SENSEGLOVE_BRANCH})"

if [[ "${SENSEGLOVE_SKIP_APT:-0}" != "1" ]]; then
    echo "[senseglove_install] Installing common ROS 2/SenseGlove build dependencies."
    sudo apt-get update
    sudo apt-get install -y \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-pyqt5 \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-xacro \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-broadcaster \
        ros-humble-joint-trajectory-controller
fi

mkdir -p "${SENSEGLOVE_WS}/src"
if [[ -d "${REPO_DIR}/.git" ]]; then
    current_branch=$(git -C "${REPO_DIR}" branch --show-current)
    echo "[senseglove_install] Using existing checkout: ${REPO_DIR} (${current_branch})."
    if [[ "${current_branch}" != "${SENSEGLOVE_BRANCH}" ]]; then
        echo "[senseglove_install] Checking out local branch ${SENSEGLOVE_BRANCH}."
        git -C "${REPO_DIR}" checkout "${SENSEGLOVE_BRANCH}"
    fi
    if [[ "${SENSEGLOVE_UPDATE_REPO:-0}" == "1" ]]; then
        echo "[senseglove_install] Updating existing checkout from origin."
        git -C "${REPO_DIR}" -c http.version=HTTP/1.1 fetch --all --prune
        git -C "${REPO_DIR}" pull --ff-only
    else
        echo "[senseglove_install] Skipping git update. Set SENSEGLOVE_UPDATE_REPO=1 to fetch latest origin/${SENSEGLOVE_BRANCH}."
    fi
elif [[ -e "${REPO_DIR}" ]]; then
    echo "[senseglove_install] ${REPO_DIR} exists but is not a git checkout." >&2
    exit 1
else
    echo "[senseglove_install] Cloning official SenseGlove ROS repository."
    git -c http.version=HTTP/1.1 clone --depth 1 --single-branch -b "${SENSEGLOVE_BRANCH}" "${SENSEGLOVE_REPO}" "${REPO_DIR}"
fi

set +u
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
set -u

if [[ "${SENSEGLOVE_SKIP_ROSDEP:-0}" != "1" ]]; then
    if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
        sudo rosdep init || true
    fi
    rosdep update
    rosdep install --from-paths "${SENSEGLOVE_WS}/src" --ignore-src -r -y
fi

cd "${SENSEGLOVE_WS}"
colcon build --symlink-install --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3

echo
echo "[senseglove_install] Done."
echo "[senseglove_install] Source this overlay before launching:"
echo "source ${SENSEGLOVE_WS}/install/setup.bash"
