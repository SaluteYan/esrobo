#!/usr/bin/env bash
# Safely launch PICO tracking for one NERO arm, optionally with its glove/hand.

set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
CONDA_PYTHON="${MINICONDA_DIR:-/home/esrobo/miniconda3}/envs/${ENV_NAME:-teleop_esrobo}/bin/python"
LEFT_SERIAL="${SENSEGLOVE_LEFT_SERIAL:-00885}"
RIGHT_SERIAL="${SENSEGLOVE_RIGHT_SERIAL:-00892}"
WITH_LEFT_IMU=0
WITH_RIGHT_IMU=0
ARM_SIDE="left"
FORWARD_ARGS=()
BRIDGE_PID=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-left-imu) WITH_LEFT_IMU=1; shift;;
    --with-right-imu) WITH_RIGHT_IMU=1; shift;;
    -l|--left-serial) LEFT_SERIAL="$2"; shift 2;;
    -r|--right-serial) RIGHT_SERIAL="$2"; shift 2;;
    --recalibrate|--no-hardware) FORWARD_ARGS+=("$1"); shift;;
    --imu-calibration-mode) FORWARD_ARGS+=("$1" "$2"); shift 2;;
    --arm-side) ARM_SIDE="$2"; shift 2;;
    *) echo "ERROR: unknown arg: $1" >&2; exit 2;;
  esac
done

if [[ "${ARM_SIDE}" != "left" && "${ARM_SIDE}" != "right" ]]; then
  echo "ERROR: --arm-side must be left or right." >&2
  exit 2
fi

if [[ "${ARM_SIDE}" == "left" ]]; then
  ARM_CN="左"
  OTHER_ARM_CN="右"
  ARM_CAN="can_piper1"
  ARM_USB_PATH="5.3:1.0"
else
  ARM_CN="右"
  OTHER_ARM_CN="左"
  ARM_CAN="can_piper2"
  ARM_USB_PATH="5.4:1.0"
fi

if [[ "${WITH_LEFT_IMU}" == "1" && "${WITH_RIGHT_IMU}" == "1" ]]; then
  echo "ERROR: choose only one of --with-left-imu or --with-right-imu." >&2
  exit 2
fi

if [[ "${WITH_LEFT_IMU}" == "1" ]]; then
  [[ "${ARM_SIDE}" == "left" ]] || {
    echo "ERROR: --with-left-imu is only valid with --arm-side left." >&2
    exit 2
  }
  exec "${ROOT_DIR}/scripts/run_senseglove_hand_teleop.sh" \
    --left-serial "${LEFT_SERIAL}" --pico-left-arm-hand "${FORWARD_ARGS[@]}"
fi

if [[ "${WITH_RIGHT_IMU}" == "1" ]]; then
  [[ "${ARM_SIDE}" == "right" ]] || {
    echo "ERROR: --with-right-imu is only valid with --arm-side right." >&2
    exit 2
  }
  exec "${ROOT_DIR}/scripts/run_senseglove_hand_teleop.sh" \
    --right-serial "${RIGHT_SERIAL}" --pico-right-arm-hand "${FORWARD_ARGS[@]}"
fi

if [[ "${#FORWARD_ARGS[@]}" -gt 0 ]]; then
  echo "ERROR: SenseGlove options require --with-left-imu or --with-right-imu." >&2
  exit 2
fi

cleanup() {
  if [[ -n "${BRIDGE_PID}" ]] && kill -0 "${BRIDGE_PID}" 2>/dev/null; then
    kill -TERM -- "-${BRIDGE_PID}" 2>/dev/null || kill -TERM "${BRIDGE_PID}" 2>/dev/null || true
    wait "${BRIDGE_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ ! -x "${CONDA_PYTHON}" ]]; then
  echo "ERROR: ${CONDA_PYTHON} 不存在，请先运行 ./scripts/install_env.sh。" >&2
  exit 1
fi

ensure_arm_can() {
  local target="${ARM_CAN}" iface="" candidate bus_info port_path
  if ip link show "${target}" >/dev/null 2>&1; then
    iface="${target}"
  else
    command -v ethtool >/dev/null 2>&1 || {
      echo "ERROR: 缺少 ethtool，无法识别${ARM_CN}机械臂 USB-CAN。" >&2
      return 1
    }
    while read -r candidate _; do
      [[ -n "${candidate}" ]] || continue
      bus_info=$(ethtool -i "${candidate}" 2>/dev/null | awk '/bus-info:/ {print $2}')
      port_path="${bus_info#*-}"
      if [[ "${bus_info}" == "${ARM_USB_PATH}" || "${port_path}" == "${ARM_USB_PATH}" ]]; then
        iface="${candidate}"
        break
      fi
    done < <(ip -br link show type can)
  fi
  if [[ -z "${iface}" ]]; then
    echo "ERROR: 未找到${ARM_CN}机械臂 USB-CAN（预期 USB 端口 *-${ARM_USB_PATH}）。" >&2
    return 1
  fi

  echo "==> 配置${ARM_CN}机械臂 CAN：${iface} -> ${target}，1 Mbps"
  echo "    此步骤只初始化通信接口，不发送运动指令；如提示，请输入 sudo 密码。"
  sudo modprobe can
  sudo modprobe can_raw
  sudo ip link set "${iface}" down
  if [[ "${iface}" != "${target}" ]]; then
    sudo ip link set "${iface}" name "${target}"
  fi
  sudo ip link set "${target}" type can bitrate 1000000
  sudo ip link set "${target}" up
  ip -details link show "${target}" | grep -q 'state ERROR-ACTIVE' || {
    echo "ERROR: ${target} 未进入 ERROR-ACTIVE，停止启动。" >&2
    return 1
  }
}

echo "============================================================"
echo " PICO -> ${ARM_CN}机械臂 J1-J4 遥操作（无 SenseGlove）"
echo " PICO ${ARM_CN}肩/${ARM_CN}肘/${ARM_CN}腕位置只控制 J1-J4。"
echo " J5/J6/J7 始终保持各自实时反馈位置，不使用 PICO 腕姿态。"
echo " ${OTHER_ARM_CN}机械臂和双灵巧手不会初始化；按 e 前${ARM_CN}臂保持失能。"
echo "============================================================"

echo "==> 启动并检查 XRoboToolkit PC Service"
"${ROOT_DIR}/scripts/xrobotoolkit_pc_service.sh" start
echo "==> 请先在 PICO 中启动 XRoboToolkit 应用并开启全身追踪。"
echo "==> 等待 PICO ${ARM_CN}肩、${ARM_CN}肘、${ARM_CN}腕数据（最多 20 秒）；此时尚未配置机械臂 CAN。"
env LD_LIBRARY_PATH="${ROOT_DIR}/external/XRoboToolkit-PC-Service-Pybind/lib:$("${CONDA_PYTHON}" -c 'import sys; print(sys.prefix)')/lib:${LD_LIBRARY_PATH:-}" \
  "${CONDA_PYTHON}" "${ROOT_DIR}/scripts/check_xrobotoolkit_body.py" --side "${ARM_SIDE}" --wait-seconds 20

mkdir -p "${ROOT_DIR}/log"
setsid "${ROOT_DIR}/bridges/run_xrobotoolkit_bridge.sh" \
  > "${ROOT_DIR}/log/pico_${ARM_SIDE}_arm_bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 1
if ! kill -0 "${BRIDGE_PID}" 2>/dev/null; then
  echo "ERROR: PICO 数据桥启动失败：" >&2
  tail -n 30 "${ROOT_DIR}/log/pico_${ARM_SIDE}_arm_bridge.log" >&2 || true
  exit 1
fi
echo "==> PICO 数据桥和标定前骨架预览已启动。"
echo "==> 请先在本机打开 http://${PICO_SKELETON_VIEWER_HOST:-127.0.0.1}:${PICO_SKELETON_VIEWER_PORT:-8765} 检查 PICO 原始数据。"
echo "==> 当前只接收和显示数据，不发送机械臂控制命令；SSH 端口转发见操作文档 12.5 节。"

ensure_arm_can
echo
echo "==> ${ARM_CN}臂可能没有 CAN 主动上报；此处不发送使能、模式或位置命令。"
echo "==> 标定结束并按 e 后，主程序才以自然下垂零位限速唤醒并验证反馈。"
echo
echo "==> 标定姿态：站直，${ARM_CN}臂舒适地下垂并保持在 PICO 捕捉范围内，允许自然微屈，不要锁直肘部。"
echo "==> 按 Enter 后先提供 5 秒动作准备和 1 秒稳定缓冲，再采集约 3 秒。"
read -r -p "确认机器人${ARM_CN}臂自然下垂且有人托稳后，按 Enter 进入标定流程："

echo "==> 即将进入 PICO 标定；骨架调试页会继续显示实时数据。"
echo "==> 标定完成后仍不会运动；托稳${ARM_CN}臂并按 e 才开始 J1-J4 跟随。"
if [[ "${ARM_SIDE}" == "left" ]]; then
  echo "==> 可选联合模式：添加 --with-left-imu --left-serial 00885。"
else
  echo "==> 可选联合模式：添加 --with-right-imu --right-serial 00892。"
fi
"${ROOT_DIR}/scripts/run_teleop.sh" --arm-only --arm-side "${ARM_SIDE}"
