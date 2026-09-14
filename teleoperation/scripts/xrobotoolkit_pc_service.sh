#!/usr/bin/env bash
# Manage the official XRoboToolkit PC Service in a headless SSH session.

set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
APP_DIR=/opt/apps/roboticsservice
SERVICE_BIN="${APP_DIR}/RoboticsServiceProcess"
LOG_FILE="${ROOT_DIR}/log/xrobotoolkit_pc_service.log"
ACTION="${1:-status}"

service_pids() {
  pgrep -f '^/opt/apps/roboticsservice/RoboticsServiceProcess$' || true
}

status() {
  local pids
  pids=$(service_pids)
  if [[ -z "${pids}" ]]; then
    echo "XRoboToolkit PC Service: STOPPED"
    return 1
  fi
  echo "XRoboToolkit PC Service: RUNNING (PID ${pids//$'\n'/,})"
  if ss -lntp 2>/dev/null | grep -q ':60061'; then
    echo "gRPC client endpoint: 127.0.0.1:60061 LISTENING"
    return 0
  fi
  echo "ERROR: process exists but gRPC port 60061 is not listening." >&2
  return 1
}

start() {
  if status >/dev/null 2>&1; then
    status
    return 0
  fi
  if [[ ! -x "${SERVICE_BIN}" ]]; then
    echo "ERROR: ${SERVICE_BIN} is not installed." >&2
    echo "Run: ${ROOT_DIR}/scripts/setup_xrobotoolkit.sh" >&2
    return 1
  fi
  mkdir -p "${ROOT_DIR}/log"
  nohup setsid env \
    LD_LIBRARY_PATH="${APP_DIR}:${APP_DIR}/lib:${APP_DIR}/SDK/x64:${LD_LIBRARY_PATH:-}" \
    QT_PLUGIN_PATH="${APP_DIR}/plugins:${QT_PLUGIN_PATH:-}" \
    QT_QML_PATH="${APP_DIR}/qml:${QT_QML_PATH:-}" \
    "${SERVICE_BIN}" > "${LOG_FILE}" 2>&1 < /dev/null &
  for _ in $(seq 1 50); do
    if status >/dev/null 2>&1; then
      status
      echo "Log: ${LOG_FILE}"
      return 0
    fi
    sleep 0.1
  done
  echo "ERROR: XRoboToolkit PC Service failed to start." >&2
  tail -n 40 "${LOG_FILE}" >&2 || true
  return 1
}

stop() {
  local pids
  pids=$(service_pids)
  if [[ -z "${pids}" ]]; then
    echo "XRoboToolkit PC Service: already stopped"
    return 0
  fi
  kill -TERM ${pids}
  for _ in $(seq 1 30); do
    [[ -z "$(service_pids)" ]] && {
      echo "XRoboToolkit PC Service: STOPPED"
      return 0
    }
    sleep 0.1
  done
  echo "ERROR: service did not stop after SIGTERM; inspect PID(s): ${pids//$'\n'/,}" >&2
  return 1
}

case "${ACTION}" in
  start) start ;;
  stop) stop ;;
  restart) stop; start ;;
  status) status ;;
  *) echo "Usage: $0 {start|stop|restart|status}" >&2; exit 2 ;;
esac
