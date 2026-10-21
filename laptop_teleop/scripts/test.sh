#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
usage() {
    cat <<'EOF'
用法：test.sh [--smoke|--hand|--handover|--transport|--web|--arm|--full|--performance] [pytest参数]
      test.sh 测试文件或文件::用例 [pytest参数]
无参数只运行快速冒烟，不代表完整回归。显式路径只运行所选测试，不追加全套。
--full 为现行功能回归；依赖机器负载的时序门槛单独用 --performance 验收。
修改采集、并发、日志或计算性能时仍须运行性能验收；不要在实机遥操作期间压测。
EOF
}
profile="${1:---smoke}"
case "$profile" in
    --help|-h) usage; exit 0 ;;
    --smoke|--hand|--handover|--transport|--web|--arm|--full|--performance)
        if [[ $# -gt 0 ]]; then shift; fi ;;
    -*) usage >&2; echo "请先指定测试范围。" >&2; exit 2 ;;
    *) profile=selected ;;
esac
L="${LAPTOP_ROOT}/tests"
R="${LAPTOP_ROOT}/../robot_link/tests"
T="${LAPTOP_ROOT}/../teleoperation/tests"
selection=()
marker='not performance'
case "$profile" in
    --smoke) selection=("$L/test_hand_handover.py" "$R/test_joint_trajectory.py" "$L/test_recent_rate.py") ;;
    --hand) selection=("$L/test_hand_geometry.py" "$L/test_hand_contact.py" "$L/test_hand_participation.py" "$L/test_hand_episode.py" "$L/test_hand_hierarchy.py" "$L/test_hand_shape_filter.py" "$L/test_pico_hand.py") ;;
    --handover) selection=("$L/test_hand_handover.py" "$L/test_wrist_handover.py" "$L/test_entry_pipeline.py" "$L/test_entry_keepalive.py" "$L/test_input_recovery.py" "$L/test_contact_following.py" "$R/test_handover.py" "$R/test_return_hand_open.py" "$R/test_hand_return_proof.py") ;;
    --transport) selection=("$L/test_pico_transport.py" "$L/test_pico_component_samples.py" "$R/test_hand_transport.py" "$R/test_link.py") ;;
    --web) selection=("$L/test_dashboard.py" "$L/test_dashboard_web.py") ;;
    --arm) selection=("$L/test_wrist_smoothing.py" "$L/test_continuous_deadband.py" "$L/test_wrist_joint_filter.py" "$L/test_wrist_jacobian.py" "$R/test_joint_trajectory.py" "$T/test_partitioned_position_ik.py") ;;
    --full) selection=("$L" "$R" "$T/test_partitioned_position_ik.py") ;;
    --performance) selection=("$L/test_pico_transport.py"); marker=performance ;;
esac
LAPTOP_PYTHON="${LAPTOP_PYTHON:-$(require_laptop_python)}"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH="${LAPTOP_ROOT}/src:${LAPTOP_ROOT}/../teleoperation/src:${LAPTOP_ROOT}/../robot_link:${LAPTOP_ROOT}/../teleoperation:${LAPTOP_ROOT}/../teleoperation/tests"
echo "测试范围：${profile}；标记：${marker}（完整验收需另外选择 --full / --performance）"
PICO_PYTHON="${LAPTOP_PYTHON}" "${LAPTOP_ROOT}/scripts/build_hand_native.sh"
exec "${LAPTOP_PYTHON}" -m pytest -q \
    -o 'markers=performance: wall-clock performance acceptance; run separately on controlled load' \
    -m "$marker" "${selection[@]}" "$@"
