#!/usr/bin/env bash
# Start/stop the official XRoboToolkit service without tying it to a terminal.
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
APP_DIR=/opt/apps/roboticsservice
SERVICE_BIN="${APP_DIR}/RoboticsServiceProcess"
LOG_FILE="${LAPTOP_ROOT}/log/xrobotoolkit_pc_service.log"
ACTION="${1:-status}"

service_pids() { pgrep -f '^/opt/apps/roboticsservice/RoboticsServiceProcess$' || true; }
status() {
    local pids
    pids="$(service_pids)"
    [[ -n "${pids}" ]] || { echo "XRoboToolkit PC Service: STOPPED"; return 1; }
    echo "XRoboToolkit PC Service: RUNNING (PID ${pids//$'\n'/,})"
    ss -lnt 2>/dev/null | grep -q ':60061 ' \
        && echo "gRPC: 127.0.0.1:60061 LISTENING" \
        || { echo "进程存在，但 60061 端口尚未监听。" >&2; return 1; }
}
start() {
    status >/dev/null 2>&1 && { status; return; }
    [[ -x "${SERVICE_BIN}" ]] || { echo "未安装 ${SERVICE_BIN}，请先运行 install_pico.sh" >&2; return 1; }
    mkdir -p "${LAPTOP_ROOT}/log"
    nohup setsid env \
        LD_LIBRARY_PATH="${APP_DIR}:${APP_DIR}/lib:${APP_DIR}/SDK/x64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
        QT_PLUGIN_PATH="${APP_DIR}/plugins${QT_PLUGIN_PATH:+:${QT_PLUGIN_PATH}}" \
        QT_QML_PATH="${APP_DIR}/qml${QT_QML_PATH:+:${QT_QML_PATH}}" \
        "${SERVICE_BIN}" >"${LOG_FILE}" 2>&1 </dev/null &
    for _ in $(seq 1 100); do status >/dev/null 2>&1 && { status; echo "日志：${LOG_FILE}"; return; }; sleep .1; done
    tail -n 40 "${LOG_FILE}" >&2 || true
    return 1
}
stop() {
    local pids
    pids="$(service_pids)"
    [[ -n "${pids}" ]] || { echo "XRoboToolkit PC Service: STOPPED"; return; }
    kill -TERM ${pids}
    for _ in $(seq 1 50); do [[ -z "$(service_pids)" ]] && { echo "XRoboToolkit PC Service: STOPPED"; return; }; sleep .1; done
    echo "服务未能在 5 秒内停止。" >&2; return 1
}
case "${ACTION}" in
    start) start ;; stop) stop ;; restart) stop; start ;; status) status ;;
    *) echo "用法：$0 {start|stop|restart|status}" >&2; exit 2 ;;
esac
