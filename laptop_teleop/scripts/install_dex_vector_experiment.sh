#!/usr/bin/env bash
# Separate optional environment. Does not modify runtime dependencies or services.
set -euo pipefail
TASK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TASK_BASE="${TASK_ROOT}/laptop_teleop/.venv/bin/python"
TASK_ENV="${TASK_ROOT}/laptop_teleop/external/dex-vector-venv"
TASK_SOURCE="${TASK_ROOT}/laptop_teleop/external/dex-retargeting"
TASK_REV=3f56141bc8bd2760d5e452e382937269554ebb21
if [[ ! -x "${TASK_BASE}" ]]; then echo 'Create the normal laptop environment first.' >&2; exit 1; fi
if [[ ! -d "${TASK_SOURCE}/.git" ]]; then
    git clone https://github.com/dexsuite/dex-retargeting.git "${TASK_SOURCE}"
fi
if [[ -n "$(git -C "${TASK_SOURCE}" status --porcelain)" ]]; then
    echo 'External checkout has changes; preserve them before installing.' >&2; exit 1
fi
git -C "${TASK_SOURCE}" fetch origin "${TASK_REV}"
git -C "${TASK_SOURCE}" checkout --detach "${TASK_REV}"
"${TASK_BASE}" -m venv "${TASK_ENV}"
"${TASK_BASE}" - "${TASK_ENV}" <<'PY'
from pathlib import Path
import site,sys
folder=Path(sys.argv[1])/f'lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages'
# Process the baseline cmeel .pth files too; plain sys.path addition is insufficient.
(folder/'esrobo_baseline.pth').write_text('import site; site.addsitedir('+repr(site.getsitepackages()[0])+')\n')
PY
"${TASK_ENV}/bin/python" -m pip install 'torch==2.14.1+cpu' --index-url https://download.pytorch.org/whl/cpu
"${TASK_ENV}/bin/python" -m pip install "${TASK_SOURCE}" 'nlopt==2.11.0'
echo "Offline interpreter: ${TASK_ENV}/bin/python"
