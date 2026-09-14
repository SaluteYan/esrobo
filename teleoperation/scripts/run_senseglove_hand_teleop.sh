#!/usr/bin/env bash
# ============================================================================
# SenseGlove 手指 + IMU 实机遥操作 —— 一键启动
#
# 一条命令把整个链路跑起来（含可选标定）：
#   LinkerHand 驱动 + hand_ros_bridge + SenseGlove 驱动 + SenseGlove 桥接
#   + 手部遥操作核心（前台，可按 'h' 使能灵巧手 / 'q' 退出）。
#
# 用法示例：
#   ./scripts/run_senseglove_hand_teleop.sh --left-serial 01001 --right-serial 01002
#   ./scripts/run_senseglove_hand_teleop.sh -l 01001 -r 01002 --recalibrate   # 强制重新标定
#   ./scripts/run_senseglove_hand_teleop.sh -l 01001 -r 01002 --enable        # 立即使能灵巧手
#   ./scripts/run_senseglove_hand_teleop.sh -l 01001 -r 01002 --no-hardware   # 仅测试链路(用仿真输入)
#
# 参数：
#   -l/--left-serial   <s>   左手套序列号
#   -r/--right-serial  <s>   右手套序列号
#   --recalibrate             强制重新做手指端点与 IMU 多姿态坐标基标定
#   --enable                  启动后立即使能灵巧手（默认需按 'h'）
#   --no-hardware             跳过 LinkerHand/SenseGlove 硬件驱动（配合仿真输入测链路）
#   --calib-only              只做标定后退出
#   --left-only               只测试左手套和左灵巧手（右手不启动、不发命令）
#   --right-only              只测试右手套和右灵巧手（左手不启动、不发命令）
#   --left-wrist-imu          左手模式下增加左机械臂末端第 5/6/7 轴 IMU 跟随
#   --right-wrist-imu         右手模式下增加右机械臂末端第 5/6/7 轴 IMU 跟随
#   --pico-left-arm           PICO 控制左臂 J1-J4，左手套 IMU 控制 J5-J7
#   --pico-left-arm-hand      在上述七轴控制基础上同时控制左灵巧手
#   --pico-right-arm          PICO 控制右臂 J1-J4，右手套 IMU 控制 J5-J7
#   --pico-right-arm-hand     在上述七轴控制基础上同时控制右灵巧手
#   --imu-calibration-mode <static-orthogonal|static-linear>
#                              静态纯旋转（默认）或静态线性对照
# ============================================================================
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_NAME="${ENV_NAME:-teleop_esrobo}"
MINICONDA_DIR="${MINICONDA_DIR:-/home/esrobo/miniconda3}"
CONDA_PYTHON="${MINICONDA_DIR}/envs/${ENV_NAME}/bin/python"
SENSEGLOVE_WS="${SENSEGLOVE_WS:-${ROOT_DIR}/external/senseglove_ros}"
SENSECOM_DIR="${SENSEGLOVE_WS}/senseglove_com/SenseCom/Linux/SenseCom_Linux_Latest"
SENSECOM_BIN="${SENSECOM_DIR}/SenseCom.x86_64"
SENSECOM_VERSION_FILE="${SENSECOM_DIR}/versionInfo.txt"
ESROBO_WS="${ESROBO_WS:-/home/esrobo/Projects/esrobo}"
CALIB_FILE="${ROOT_DIR}/config/senseglove_esrobo_calibration.json"
CALIB_DURATION="${CALIB_DURATION:-10.0}"
CALIB_SAMPLE_START="${CALIB_SAMPLE_START:-6.0}"

LEFT_SERIAL=""; RIGHT_SERIAL=""; RECAL=0; AUTO_ENABLE=0; NO_HW=0; CALIB_ONLY=0
LEFT_ONLY=0; RIGHT_ONLY=0; LEFT_WRIST_IMU=0; RIGHT_WRIST_IMU=0
PICO_ARM=0; PICO_ARM_HAND=0; PICO_ARM_SIDE=""
IMU_CALIBRATION_MODE="${IMU_CALIBRATION_MODE:-static-orthogonal}"
IMU_BRIDGE_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    -l|--left-serial)  LEFT_SERIAL="$2"; shift 2;;
    -r|--right-serial) RIGHT_SERIAL="$2"; shift 2;;
    --recalibrate)     RECAL=1; shift;;
    --enable)          AUTO_ENABLE=1; shift;;
    --no-hardware)     NO_HW=1; shift;;
    --calib-only)      CALIB_ONLY=1; shift;;
    --left-only)       LEFT_ONLY=1; shift;;
    --right-only)      RIGHT_ONLY=1; shift;;
    --left-wrist-imu)  LEFT_ONLY=1; LEFT_WRIST_IMU=1; RECAL=1; shift;;
    --right-wrist-imu) RIGHT_ONLY=1; RIGHT_WRIST_IMU=1; RECAL=1; shift;;
    --pico-left-arm)
      LEFT_ONLY=1; PICO_ARM=1; PICO_ARM_SIDE="left"; RECAL=1; shift;;
    --pico-left-arm-hand)
      LEFT_ONLY=1; PICO_ARM=1; PICO_ARM_HAND=1; PICO_ARM_SIDE="left"; RECAL=1; shift;;
    --pico-right-arm)
      RIGHT_ONLY=1; PICO_ARM=1; PICO_ARM_SIDE="right"; RECAL=1; shift;;
    --pico-right-arm-hand)
      RIGHT_ONLY=1; PICO_ARM=1; PICO_ARM_HAND=1; PICO_ARM_SIDE="right"; RECAL=1; shift;;
    --imu-calibration-mode) IMU_CALIBRATION_MODE="$2"; shift 2;;
    *) echo "unknown arg: $1"; exit 2;;
  esac
done

if [ "${IMU_CALIBRATION_MODE}" != "static-orthogonal" ] && \
   [ "${IMU_CALIBRATION_MODE}" != "static-linear" ]; then
  echo "ERROR: invalid --imu-calibration-mode." >&2
  exit 2
fi

if [ "${LEFT_ONLY}" = "1" ] && [ "${RIGHT_ONLY}" = "1" ]; then
  echo "ERROR: --left-only and --right-only cannot be used together." >&2
  exit 2
fi

if [ "${LEFT_ONLY}" = "1" ]; then
  if [ -z "${LEFT_SERIAL}" ]; then
    echo "ERROR: --left-only requires --left-serial." >&2
    exit 2
  fi
  CALIB_FILE="${ROOT_DIR}/config/senseglove_esrobo_calibration_left.json"
elif [ "${RIGHT_ONLY}" = "1" ]; then
  if [ -z "${RIGHT_SERIAL}" ]; then
    echo "ERROR: --right-only requires --right-serial." >&2
    exit 2
  fi
  CALIB_FILE="${ROOT_DIR}/config/senseglove_esrobo_calibration_right.json"
elif [ "${CALIB_ONLY}" = "0" ]; then
  if [ -z "${LEFT_SERIAL}" ] || [ -z "${RIGHT_SERIAL}" ]; then
    echo "ERROR: need --left-serial and --right-serial (or use --calib-only)." >&2
    exit 2
  fi
fi

# Keep the optional linear calibration artifact separate for repeatable tests.
if [ "${IMU_CALIBRATION_MODE}" = "static-linear" ]; then
  CALIB_FILE="${CALIB_FILE%.json}_linear.json"
fi

if { [ "${LEFT_WRIST_IMU}" = "1" ] || [ "${RIGHT_WRIST_IMU}" = "1" ]; } && [ "${AUTO_ENABLE}" = "1" ]; then
  echo "ERROR: --*-wrist-imu 禁止与 --enable 同时使用；机械臂必须在标定后手动按 e 使能。" >&2
  exit 2
fi

if [ "${PICO_ARM}" = "1" ] && [ "${AUTO_ENABLE}" = "1" ]; then
  echo "ERROR: PICO 全臂模式禁止 --enable；完成两套标定并托稳机械臂后必须手动按 e。" >&2
  exit 2
fi

# Pure hand tests do not consume wrist orientation. Do not force users through
# an unrelated three-axis IMU calibration or publish an untrusted orientation.
if [ "${LEFT_WRIST_IMU}" = "0" ] && [ "${RIGHT_WRIST_IMU}" = "0" ] && \
   [ "${PICO_ARM}" = "0" ]; then
  IMU_BRIDGE_ARGS+=(--disable-imu-orientation)
fi

if [ "${PICO_ARM_SIDE}" = "left" ]; then
  PICO_ARM_CN="左"; PICO_OTHER_CN="右"
  PICO_ARM_CAN="can_piper1"; PICO_ARM_USB_PATH="5.3:1.0"
elif [ "${PICO_ARM_SIDE}" = "right" ]; then
  PICO_ARM_CN="右"; PICO_OTHER_CN="左"
  PICO_ARM_CAN="can_piper2"; PICO_ARM_USB_PATH="5.4:1.0"
else
  PICO_ARM_CN=""; PICO_OTHER_CN=""
  PICO_ARM_CAN=""; PICO_ARM_USB_PATH=""
fi

if [ "${NO_HW}" = "0" ]; then
  if [ ! -r "${SENSECOM_VERSION_FILE}" ]; then
    echo "ERROR: 找不到 SenseCom 版本文件：${SENSECOM_VERSION_FILE}" >&2
    exit 2
  fi
  sensecom_version="$(sed -n 's/.*"senseComVersion"[[:space:]]*:[[:space:]]*"\([0-9.]*\)".*/\1/p' "${SENSECOM_VERSION_FILE}" | head -n 1)"
  if [ -z "${sensecom_version}" ] || [ "$(printf '%s\n' 1.8.0 "${sensecom_version}" | sort -V | head -n 1)" != "1.8.0" ]; then
    echo "ERROR: SenseCom ${sensecom_version:-未知} 不支持 Nova 2 BLE；要求 1.8.0 或更新版本。" >&2
    exit 2
  fi
  echo "==> SenseCom 版本 ${sensecom_version}：支持 Nova 2 BLE。"
  echo "    SenseCom 没有关闭手套硬件自动休眠的设置；长时间静置时手套仍可能自行关机。"
fi

mkdir -p "${ROOT_DIR}/log"
PIDS=()

cleanup() {
  local status=$? pid
  # Prevent Ctrl-C/TERM received during cleanup from recursively entering this
  # function. In particular, a second Ctrl-C must not resume startup afterward.
  trap - EXIT INT TERM
  echo; echo "==> stopping background processes..."
  # Each launcher runs in its own session. Stop the complete process group so
  # ROS launch children cannot keep hardware drivers alive after an error.
  for pid in "${PIDS[@]:-}"; do kill -TERM -- "-${pid}" 2>/dev/null || true; done
  sleep 1
  for pid in "${PIDS[@]:-}"; do kill -KILL -- "-${pid}" 2>/dev/null || true; done
  # Wait only for processes registered by start_bg(). A bare `wait` also waits
  # for the intentionally persistent headless SenseCom launcher and can hang.
  for pid in "${PIDS[@]:-}"; do wait "${pid}" 2>/dev/null || true; done
  return "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# ---------------------------------------------------------------- ROS2 env
set +u
source /opt/ros/humble/setup.bash
if [ -f "${ESROBO_WS}/install/setup.bash" ]; then source "${ESROBO_WS}/install/setup.bash"; fi
if [ -f "${SENSEGLOVE_WS}/install/setup.bash" ]; then source "${SENSEGLOVE_WS}/install/setup.bash"; fi
set -u

start_bg() {  # name  cmd...
  local name="$1"; shift
  echo "==> 启动 [${name}]（日志: ${ROOT_DIR}/log/${name}.log）"
  setsid "$@" > "${ROOT_DIR}/log/${name}.log" 2>&1 &
  PIDS+=("$!")
}

start_bg_with_enter() {  # name  cmd...
  local name="$1"; shift
  echo "==> 启动 [${name}]（SenseCom/手套数据已验证；日志: ${ROOT_DIR}/log/${name}.log）"
  setsid "$@" < <(printf '\n') > "${ROOT_DIR}/log/${name}.log" 2>&1 &
  PIDS+=("$!")
}

sensecom_pids() {
  pgrep -f "^${SENSECOM_BIN}([[:space:]]|$)" || true
}

stop_sensecom() {
  local pids wrappers
  pids="$(sensecom_pids)"
  if [ -n "${pids}" ]; then
    kill -TERM ${pids} 2>/dev/null || true
    sleep 2
    pids="$(sensecom_pids)"
    [ -z "${pids}" ] || kill -KILL ${pids} 2>/dev/null || true
  fi
  wrappers="$(pgrep -f "^dbus-run-session -- ${SENSECOM_BIN}([[:space:]]|$)" || true)"
  [ -z "${wrappers}" ] || kill -TERM ${wrappers} 2>/dev/null || true
  sleep 1
}

ensure_sensecom_headless() {
  local display="${SENSECOM_DISPLAY:-:99}" waited pids process_count

  pids="$(sensecom_pids)"
  process_count="$(wc -w <<< "${pids}")"
  if [ "${process_count}" -eq 1 ]; then
    if timeout 2 ros2 run senseglove_api sg_tester >/dev/null 2>&1; then
      echo "==> 检测到 SenseCom IPC 正常，复用当前进程。"
      return 0
    fi
    echo "==> 检测到无效的 SenseCom 残留进程，正在清理后重启。"
    stop_sensecom
  elif [ "${process_count}" -gt 1 ]; then
    echo "==> 检测到 ${process_count} 个 SenseCom 实例，正在清理重复进程后重启。"
    stop_sensecom
  fi
  [ -x "${SENSECOM_BIN}" ] || {
    echo "ERROR: SenseCom 不可执行：${SENSECOM_BIN}" >&2
    return 1
  }

  if ! DISPLAY="${display}" xdpyinfo >/dev/null 2>&1; then
    command -v Xvfb >/dev/null 2>&1 || {
      echo "ERROR: SSH 无桌面且未安装 Xvfb，无法启动 SenseCom。" >&2
      return 1
    }
    echo "==> 启动 SenseCom 专用虚拟显示 ${display}"
    nohup setsid Xvfb "${display}" -screen 0 1280x720x24 -nolisten tcp \
      > "${ROOT_DIR}/log/xvfb-sensecom.log" 2>&1 < /dev/null &
    for waited in $(seq 1 20); do
      DISPLAY="${display}" xdpyinfo >/dev/null 2>&1 && break
      sleep 0.25
    done
    DISPLAY="${display}" xdpyinfo >/dev/null 2>&1 || {
      echo "ERROR: 虚拟显示 ${display} 启动失败。" >&2
      return 1
    }
  fi

  echo "==> 以 SSH 无图形模式启动 SenseCom（避免 Unity/Xvfb 窗口崩溃）"
  nohup setsid env DISPLAY="${display}" LIBGL_ALWAYS_SOFTWARE=1 \
    dbus-run-session -- "${SENSECOM_BIN}" -batchmode -nographics \
    -logFile "${ROOT_DIR}/log/sensecom.log" \
    > "${ROOT_DIR}/log/sensecom-launch.log" 2>&1 < /dev/null &

  for waited in $(seq 1 20); do
    if [ -n "$(sensecom_pids)" ] && \
       timeout 2 ros2 run senseglove_api sg_tester >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.5
  done
  echo "ERROR: SenseCom 进程或 SGCore IPC 启动失败；日志：${ROOT_DIR}/log/sensecom.log" >&2
  tail -n 40 "${ROOT_DIR}/log/sensecom.log" >&2 || true
  return 1
}

restart_sensecom_headless() {
  echo "==> 清理 SenseCom 的失效 BLE 缓存，并重新扫描手套。"
  stop_sensecom
  ensure_sensecom_headless
}

wait_for_senseglove() {  # serial  side-letter  label
  local serial="$1" side_letter="$2" label="$3" expected status waited
  expected="Nova 2-${serial}-${side_letter}"
  echo "==> 等待${label} ${expected} 的实时数据（最多 20 秒）..."
  for waited in $(seq 1 20); do
    status="$(ros2 run senseglove_api sg_tester 2>&1 || true)"
    if awk -v expected="${expected}" '
      index($0, "ID: " expected " ") { target=1; next }
      target && /connected:/ {
        if ($0 ~ /connected: true/ && match($0, /rx: [0-9]+/)) {
          rate=substr($0, RSTART + 4, RLENGTH - 4) + 0
          if (rate > 0) ok=1
        }
        exit
      }
      END { exit !ok }
    ' <<< "${status}"; then
      echo "==> ${label}已连接：${expected}，SGCore 正在接收实时数据。"
      return 0
    fi
    sleep 1
  done
  echo "ERROR: 未检测到${label} ${expected} 的有效数据，机器人硬件尚未初始化。" >&2
  echo "${status}" >&2
  return 1
}

verify_requested_sensegloves() {
  if [ "${LEFT_ONLY}" = "1" ]; then
    wait_for_senseglove "${LEFT_SERIAL}" L "左手套"
  elif [ "${RIGHT_ONLY}" = "1" ]; then
    wait_for_senseglove "${RIGHT_SERIAL}" R "右手套"
  else
    wait_for_senseglove "${LEFT_SERIAL}" L "左手套" &&
      wait_for_senseglove "${RIGHT_SERIAL}" R "右手套"
  fi
}

ensure_can() {  # interface  stable USB port path  device label
  local target="$1" usb_port="$2" device_label="$3" iface="" candidate bus_info port_path

  if ip link show "${target}" >/dev/null 2>&1; then
    iface="${target}"
  else
    command -v ethtool >/dev/null 2>&1 || {
      echo "ERROR: 缺少 ethtool，无法识别 ${device_label} 对应的 USB-CAN。" >&2
      return 1
    }
    while read -r candidate _; do
      [ -n "${candidate}" ] || continue
      bus_info="$(ethtool -i "${candidate}" 2>/dev/null | awk '/bus-info:/ {print $2}')"
      port_path="${bus_info#*-}"
      if [ "${bus_info}" = "${usb_port}" ] || [ "${port_path}" = "${usb_port}" ]; then
        iface="${candidate}"
        break
      fi
    done < <(ip -br link show type can)
  fi

  if [ -z "${iface}" ]; then
    echo "ERROR: 未找到 ${target}（预期 USB 端口 *-${usb_port}）。" >&2
    echo "请检查${device_label}电源、USB-CAN 接线，然后重新运行本命令。" >&2
    return 1
  fi

  echo "==> 配置${device_label} CAN：${iface} -> ${target}，1 Mbps"
  echo "    此步骤只初始化通信接口，不发送运动指令；如提示，请输入 sudo 密码。"
  sudo modprobe can can_raw
  sudo ip link set "${iface}" down
  if [ "${iface}" != "${target}" ]; then
    sudo ip link set "${iface}" name "${target}"
  fi
  sudo ip link set "${target}" type can bitrate 1000000
  sudo ip link set "${target}" up
  ip -details link show "${target}" | grep -q 'state ERROR-ACTIVE' || {
    echo "ERROR: ${target} 未进入 ERROR-ACTIVE，停止启动。" >&2
    return 1
  }
}

wait_for_hand_feedback() {  # side
  local side="$1" topic="/cb_${1}_hand_state" sample
  echo "==> 等待 ${side} 灵巧手首次状态反馈（最多 10 秒）..."
  if ! sample="$(timeout 10 ros2 topic echo --once "${topic}" 2>/dev/null)"; then
    echo "ERROR: 未收到 ${topic}，禁止进入遥操作。驱动日志如下：" >&2
    tail -n 30 "${ROOT_DIR}/log/linker_hand_driver.log" >&2 || true
    return 1
  fi
  if ! awk '
    /^position:$/ { in_position=1; next }
    /^velocity:$/ { in_position=0 }
    in_position && /^- / {
      value=$2 + 0
      count++
      if (value < 0 || value > 255) invalid=1
    }
    END { exit !(count == 10 && !invalid) }
  ' <<< "${sample}"; then
    echo "ERROR: ${topic} 不是 10 个有效的 0..255 关节反馈，禁止进入遥操作。" >&2
    return 1
  fi
  echo "==> ${side} 灵巧手状态反馈正常；遥操作仍保持未使能。"
}

echo
echo "============================================================"
if [ "${PICO_ARM}" = "1" ]; then
  if [ "${PICO_ARM_HAND}" = "1" ]; then
    echo " PICO + SenseGlove -> ${PICO_ARM_CN}机械臂 + ${PICO_ARM_CN}灵巧手联动遥操作"
  else
    echo " PICO + SenseGlove -> ${PICO_ARM_CN}机械臂七自由度遥操作"
  fi
else
  echo " SenseGlove -> 双灵巧手遥操作"
fi
echo " 左手套序列号: ${LEFT_SERIAL:-未指定}"
echo " 右手套序列号: ${RIGHT_SERIAL:-未指定}"
if [ "${PICO_ARM}" = "1" ]; then
  echo " 安全状态: PICO 仅控制${PICO_ARM_CN}臂 J1-J4；${PICO_ARM_CN}手套 IMU 仅控制 J5/J6/J7"
  if [ "${PICO_ARM_HAND}" = "1" ]; then
    echo " ${PICO_ARM_CN}手套手指控制${PICO_ARM_CN}灵巧手；${PICO_OTHER_CN}臂和${PICO_OTHER_CN}灵巧手不初始化；标定后必须手动按 e"
  else
    echo " ${PICO_OTHER_CN}臂和双灵巧手不初始化；标定后必须手动按 e"
  fi
elif [ "${LEFT_WRIST_IMU}" = "1" ]; then
  echo " 安全状态: 左灵巧手 + 左机械臂末端第 5/6/7 轴；右臂不连接"
  echo " 机械臂默认未使能，标定完成后必须手动按 e"
elif [ "${RIGHT_WRIST_IMU}" = "1" ]; then
  echo " 安全状态: 右灵巧手 + 右机械臂末端第 5/6/7 轴；左臂不连接"
  echo " 机械臂默认未使能，标定完成后必须手动按 e"
else
  echo " 安全状态: 仅控制灵巧手，不启动双臂机械臂"
fi
if [ "${PICO_ARM}" = "1" ]; then
  if [ "${PICO_ARM_HAND}" = "1" ]; then
    echo " 输入模式: PICO ${PICO_ARM_CN}臂骨架 + ${PICO_ARM_CN}手套 IMU/手指；初始化${PICO_ARM_CN}臂和${PICO_ARM_CN}灵巧手"
  else
    echo " 输入模式: ${PICO_ARM_CN}手套仅提供 IMU，不初始化${PICO_ARM_CN}灵巧手"
  fi
elif [ "${LEFT_ONLY}" = "1" ]; then
  echo " 单手模式: 只启动并控制左灵巧手；右灵巧手不发送命令"
elif [ "${RIGHT_ONLY}" = "1" ]; then
  echo " 单手模式: 只启动并控制右灵巧手；左灵巧手不发送命令"
  echo " 骨架诊断: 标定完成后启动 http://192.168.10.100:8766 实时 3D 页面"
fi
echo "============================================================"

# Verify SenseCom and the requested glove before touching either robot CAN bus.
if [ "${NO_HW}" = "0" ]; then
  ensure_sensecom_headless
  if ! verify_requested_sensegloves; then
    echo "==> 首次未检测到目标手套；机器人硬件仍未初始化，自动重启 SenseCom 后重试一次。"
    restart_sensecom_headless
    verify_requested_sensegloves
  fi
fi

# ---------------------------------------------------- LinkerHand 硬件驱动
if [ "${NO_HW}" = "0" ] && \
   { [ "${PICO_ARM}" = "0" ] || [ "${PICO_ARM_HAND}" = "1" ]; }; then
  if [ "${LEFT_ONLY}" = "1" ]; then
    ensure_can can_hand2 "5.1.1:1.0" "左灵巧手"
    if [ "${LEFT_WRIST_IMU}" = "1" ]; then
      ensure_can can_piper1 "5.3:1.0" "左机械臂"
      echo
      echo "WARNING: 左机械臂将退出 leader/follower 模式后全关节失能，可能在重力下下落。"
      read -r -p "请人工托稳左机械臂，并确认周围无夹点后按 Enter："
      PYTHONPATH="${ROOT_DIR}/src" "${CONDA_PYTHON}" \
        "${ROOT_DIR}/scripts/disable_right_arm.py" \
        --config "${ROOT_DIR}/config/teleop_config.yaml" --side left \
        --release-leader-mode
    fi
    start_bg linker_hand_driver \
      ros2 run linker_hand_ros2_sdk linker_hand_sdk --ros-args \
        -r __node:=linker_hand_sdk_left \
        -p hand_type:=left -p hand_joint:=L10 -p is_touch:=false \
        -p can:=can_hand2 -p move_on_start:=false -p modbus:=None
  elif [ "${RIGHT_ONLY}" = "1" ]; then
    ensure_can can_hand1 "5.1.2:1.0" "右灵巧手"
    if [ "${RIGHT_WRIST_IMU}" = "1" ]; then
      ensure_can can_piper2 "5.4:1.0" "右机械臂"
      echo
      echo "WARNING: 右机械臂将退出 leader/follower 模式后全关节失能，可能在重力下下落。"
      read -r -p "请人工托稳右机械臂，并确认周围无夹点后按 Enter："
      PYTHONPATH="${ROOT_DIR}/src" "${CONDA_PYTHON}" \
        "${ROOT_DIR}/scripts/disable_right_arm.py" \
        --config "${ROOT_DIR}/config/teleop_config.yaml" --side right \
        --release-leader-mode
    fi
    start_bg linker_hand_driver \
      ros2 launch linker_hand_ros2_sdk linker_hand.launch.py
  else
    start_bg linker_hand_driver \
      ros2 launch linker_hand_ros2_sdk linker_hand_double.launch.py
  fi
fi

# --------------------------------------------------------- hand_ros_bridge
if [ "${PICO_ARM}" = "1" ] && [ "${PICO_ARM_HAND}" = "0" ]; then
  : # Full-arm mode does not initialize either LinkerHand ROS bridge.
elif [ "${LEFT_ONLY}" = "1" ]; then
  start_bg hand_ros_bridge \
    python3 -u "${ROOT_DIR}/bridges/hand_ros_bridge.py" --port 15051 --side left
elif [ "${RIGHT_ONLY}" = "1" ]; then
  start_bg hand_ros_bridge \
    python3 -u "${ROOT_DIR}/bridges/hand_ros_bridge.py" --port 15051 --side right
else
  start_bg hand_ros_bridge \
    python3 -u "${ROOT_DIR}/bridges/hand_ros_bridge.py" --port 15051
fi

# ------------------------------------------------- SenseGlove 硬件驱动
if [ "${NO_HW}" = "0" ]; then
  # Upstream launch file always calls input() once, even when SenseCom is already
  # running. Feed only that confirmation; calibration prompts remain foreground.
  if [ "${LEFT_ONLY}" = "1" ]; then
    start_bg_with_enter senseglove_driver \
      ros2 launch senseglove_bringup senseglove.launch.py glove_side:=left
  elif [ "${RIGHT_ONLY}" = "1" ]; then
    start_bg_with_enter senseglove_driver \
      ros2 launch senseglove_bringup senseglove.launch.py glove_side:=right
  else
    start_bg_with_enter senseglove_driver \
      ros2 launch senseglove_bringup senseglove.launch.py
  fi
fi

# 等硬件驱动起来（避免桥接订阅不到 topic）
echo "==> 等待驱动初始化并发布手套数据..."
sleep 4

if [ "${NO_HW}" = "0" ] && \
   { [ "${PICO_ARM}" = "0" ] || [ "${PICO_ARM_HAND}" = "1" ]; }; then
  if [ "${LEFT_ONLY}" = "1" ]; then
    wait_for_hand_feedback left
  elif [ "${RIGHT_ONLY}" = "1" ]; then
    wait_for_hand_feedback right
  else
    wait_for_hand_feedback left
    wait_for_hand_feedback right
  fi
fi

# -------------------------------------------------- SenseGlove 数据标定
needs_calib=0
[ "${RECAL}" = "1" ] && needs_calib=1
[ ! -f "${CALIB_FILE}" ] && needs_calib=1

if [ "${needs_calib}" = "1" ]; then
  echo; echo "============================================================"
  calibration_hand="双手"
  [ "${LEFT_ONLY}" = "1" ] && calibration_hand="左手"
  [ "${RIGHT_ONLY}" = "1" ] && calibration_hand="右手"
  if [ "${#IMU_BRIDGE_ARGS[@]}" -gt 0 ]; then
    echo " SenseGlove 三步手指标定（纯灵巧手模式，不使用 IMU）"
    echo " 第 1 步：手臂和手自然下垂，手指伸直朝地面，掌心朝向身体，手腕保持平直"
    echo " 第 2 步：${calibration_hand}拇指指腹与食指指腹轻触，其余手指放松并尽量伸直"
    echo " 第 3 步：${calibration_hand}握拳，拇指主动弯曲并收向掌心"
    echo " 中指和无名指使用张手到握拳的连续映射；小指跟随无名指。"
  else
    echo " SenseGlove 六步标定（食指对指端点 + 本次开机 IMU 坐标基）"
    echo " IMU 标定模式：${IMU_CALIBRATION_MODE}"
    echo " 第 1 步：手臂和手自然下垂，手指伸直朝地面，掌心朝向身体，手腕保持平直"
    echo "          此姿态同时规定手套 IMU 零点与当前侧灵巧手/机械臂末端零点方向"
    echo " 第 2 步：回到自然下垂姿态，手指保持朝下，把掌心转向身体正前方约 25-35 度"
    echo " 第 3 步：回到自然下垂姿态，前臂不要扭转，只弯手腕把指尖向身体正前方抬约 25-35 度"
    echo " 第 4 步：回到自然下垂姿态，手离腿部留出距离，把指尖向身体内侧弯约 25-35 度"
    echo " 第 5 步：${calibration_hand}拇指指腹与食指指腹轻触，其余手指放松并尽量伸直"
    echo " 第 6 步：${calibration_hand}握拳，拇指主动弯曲并收向掌心"
    echo " 中指和无名指使用张手到握拳的连续映射；小指跟随无名指。"
    echo " 三个 IMU 动作用于求本次手套上电后的坐标基修正，不改变程序的 XYZ 定义。"
    echo " 若三个轴不够独立，程序会保留第 1 步零点，只要求重新采集第 2-4 步。"
  fi
  echo " 程序会先确认${calibration_hand}手套数据；每一步准备好后按 Enter 开始。"
  echo " 每次采集 ${CALIB_DURATION} 秒（前 ${CALIB_SAMPLE_START} 秒稳定姿态），请按终端倒计时操作。"
  echo "============================================================"
  if [ "${NO_HW}" = "0" ]; then
    if [ "${LEFT_ONLY}" = "1" ]; then
      python3 -u "${ROOT_DIR}/bridges/senseglove_ros_to_esrobo_hand_bridge.py" \
        --left-serial "${LEFT_SERIAL}" --single-side left \
        --calibration-file "${CALIB_FILE}" --recalibrate --calibrate-only \
        --imu-calibration-mode "${IMU_CALIBRATION_MODE}" \
        "${IMU_BRIDGE_ARGS[@]}" \
        --calibration-method average \
        --calibration-duration "${CALIB_DURATION}" \
        --calibration-sample-start "${CALIB_SAMPLE_START}"
    elif [ "${RIGHT_ONLY}" = "1" ]; then
      python3 -u "${ROOT_DIR}/bridges/senseglove_ros_to_esrobo_hand_bridge.py" \
        --right-serial "${RIGHT_SERIAL}" --single-side right \
        --calibration-file "${CALIB_FILE}" --recalibrate --calibrate-only \
        --imu-calibration-mode "${IMU_CALIBRATION_MODE}" \
        "${IMU_BRIDGE_ARGS[@]}" \
        --calibration-method average \
        --calibration-duration "${CALIB_DURATION}" \
        --calibration-sample-start "${CALIB_SAMPLE_START}"
    else
      python3 -u "${ROOT_DIR}/bridges/senseglove_ros_to_esrobo_hand_bridge.py" \
        --left-serial "${LEFT_SERIAL}" --right-serial "${RIGHT_SERIAL}" \
        --calibration-file "${CALIB_FILE}" --recalibrate --calibrate-only \
        --imu-calibration-mode "${IMU_CALIBRATION_MODE}" \
        "${IMU_BRIDGE_ARGS[@]}" \
        --calibration-method average \
        --calibration-duration "${CALIB_DURATION}" \
        --calibration-sample-start "${CALIB_SAMPLE_START}"
    fi
  else
    echo "[launcher] --no-hardware: skip real calibration; creating a placeholder."
    echo '{"open":{"left":{}},"closed":{"left":{}},"imu_neutral":{"left":[1,0,0,0],"right":[1,0,0,0]},"imu_frame_correction":{"left":[[1,0,0],[0,1,0],[0,0,1]],"right":[[1,0,0],[0,1,0],[0,0,1]]}}' \
      > "${CALIB_FILE}"
  fi
  if [ ! -s "${CALIB_FILE}" ]; then
    echo "ERROR: 标定未完成，未生成 ${CALIB_FILE}；不会启动遥操作或初始化后续控制。" >&2
    exit 1
  fi
else
  echo
  echo "==> 检测到已有标定文件，本次直接加载，不再等待 Enter："
  echo "    ${CALIB_FILE}"
  echo "    需要重新标定时，在原命令末尾添加 --recalibrate"
fi

# ---------------------------------------------------- PICO + single-arm preparation
# Complete the glove calibration before even configuring the robot CAN bus.
if [ "${PICO_ARM}" = "1" ] && [ "${CALIB_ONLY}" = "0" ]; then
  echo
  echo "==> 启动并检查 XRoboToolkit PC Service"
  "${ROOT_DIR}/scripts/xrobotoolkit_pc_service.sh" start
  echo "==> 请在 PICO 中启动 XRoboToolkit，并开启全身追踪。"
  echo "==> 等待${PICO_ARM_CN}肩、${PICO_ARM_CN}肘、${PICO_ARM_CN}腕数据；此时尚未配置机械臂 CAN。"
  env LD_LIBRARY_PATH="${ROOT_DIR}/external/XRoboToolkit-PC-Service-Pybind/lib:$(${CONDA_PYTHON} -c 'import sys; print(sys.prefix)')/lib:${LD_LIBRARY_PATH:-}" \
    "${CONDA_PYTHON}" "${ROOT_DIR}/scripts/check_xrobotoolkit_body.py" \
      --side "${PICO_ARM_SIDE}" --wait-seconds 20

  ensure_can "${PICO_ARM_CAN}" "${PICO_ARM_USB_PATH}" "${PICO_ARM_CN}机械臂"
  echo
  echo "==> 数据职责已固定：PICO ${PICO_ARM_CN}肩/${PICO_ARM_CN}肘/${PICO_ARM_CN}腕位置 -> J1-J4；${PICO_ARM_CN}手套 IMU -> J5-J7。"
  echo "==> ${PICO_ARM_CN}臂保持失能，不发送位置命令；按 e 后才执行限速唤醒。"
  read -r -p "请托稳自然下垂的${PICO_ARM_CN}臂、确认周围无夹点，按 Enter 进入 PICO 标定："
  start_bg "pico_${PICO_ARM_SIDE}_arm_bridge" \
    "${ROOT_DIR}/bridges/run_xrobotoolkit_bridge.sh" --print-poses
  echo "==> 骨架调试页已启动：http://${PICO_SKELETON_VIEWER_HOST:-127.0.0.1}:${PICO_SKELETON_VIEWER_PORT:-8765}（SSH 本机端口转发见操作文档 12.5 节）。"
fi

# ------------------------------------------------------- SenseGlove 桥接(流式)
if [ "${CALIB_ONLY}" = "0" ]; then
  if [ "${NO_HW}" = "0" ]; then
    if [ "${LEFT_ONLY}" = "1" ]; then
      start_bg senseglove_bridge \
        python3 -u "${ROOT_DIR}/bridges/senseglove_ros_to_esrobo_hand_bridge.py" \
          --left-serial "${LEFT_SERIAL}" --single-side left \
          --calibration-file "${CALIB_FILE}" \
          --imu-calibration-mode "${IMU_CALIBRATION_MODE}" \
          "${IMU_BRIDGE_ARGS[@]}" --print-raw
    elif [ "${RIGHT_ONLY}" = "1" ]; then
      start_bg senseglove_bridge \
        python3 -u "${ROOT_DIR}/bridges/senseglove_ros_to_esrobo_hand_bridge.py" \
          --right-serial "${RIGHT_SERIAL}" --single-side right \
          --calibration-file "${CALIB_FILE}" \
          --imu-calibration-mode "${IMU_CALIBRATION_MODE}" \
          "${IMU_BRIDGE_ARGS[@]}" --visualize-right-hand
    else
      start_bg senseglove_bridge \
        python3 -u "${ROOT_DIR}/bridges/senseglove_ros_to_esrobo_hand_bridge.py" \
          --left-serial "${LEFT_SERIAL}" --right-serial "${RIGHT_SERIAL}" \
          --calibration-file "${CALIB_FILE}" \
          --imu-calibration-mode "${IMU_CALIBRATION_MODE}" \
          "${IMU_BRIDGE_ARGS[@]}"
    fi
  else
    # 离线测试：用仿真手套输入代替真实桥接
    start_bg sim_senseglove \
      "${CONDA_PYTHON}" "${ROOT_DIR}/scripts/sim_body_input.py" --hz 40
  fi

  # ------------------------------------------------------ 手部遥操作核心(前台)
  cd "${ROOT_DIR}"
  echo
  echo "============================================================"
  teleop_hand="双手"
  [ "${LEFT_ONLY}" = "1" ] && teleop_hand="左手"
  [ "${RIGHT_ONLY}" = "1" ] && teleop_hand="右手"
  if [ "${PICO_ARM}" = "1" ]; then
    if [ "${PICO_ARM_HAND}" = "1" ]; then
      echo " 标定/加载完成，即将进入${PICO_ARM_CN}机械臂 + ${PICO_ARM_CN}灵巧手联动遥操作"
    else
      echo " 标定/加载完成，即将进入${PICO_ARM_CN}机械臂遥操作"
    fi
    echo " 进入按键控制前：核对${PICO_ARM_CN}机械臂 J1-J7 自然下垂零位"
    if [ "${PICO_ARM_HAND}" = "1" ]; then
      echo " 同时核对${PICO_ARM_CN}灵巧手十个物理关节的自然张开零位"
    fi
  else
    echo " 标定/加载完成，即将进入${teleop_hand}遥操作"
    echo " 进入按键控制前：读取并核对机械臂 J5/J6/J7 与灵巧手全部物理关节的初始零位"
    echo " 如存在超差，程序会再次要求人工确认，再限速对齐并复核；复核失败将终止启动"
    echo " 按 h：使能或停止灵巧手跟随"
  fi
  if [ "${PICO_ARM}" = "1" ]; then
    if [ "${PICO_ARM_HAND}" = "1" ]; then
      echo " 按 e：确认 PICO、${PICO_ARM_CN}手套 IMU/手指及${PICO_ARM_CN}灵巧手反馈后，同时开始联动"
      echo " 按 s：停止手指跟随，${PICO_ARM_CN}臂分阶段回零失能，再让${PICO_ARM_CN}灵巧手缓慢张开"
      echo " 按 h：仅切换${PICO_ARM_CN}灵巧手跟随；按 x：${PICO_ARM_CN}臂立即急停并冻结手指目标"
    else
      echo " 按 e：确认 PICO 与${PICO_ARM_CN}手套 IMU 均为实时数据后，使能${PICO_ARM_CN}臂开始跟随"
      echo " 按 s：${PICO_ARM_CN}臂分阶段回自然下垂零位并失能；按 x：立即电子急停"
    fi
  elif [ "${LEFT_WRIST_IMU}" = "1" ]; then
    echo " 按 i：开启/关闭 IMU 坐标诊断（默认关闭，不影响控制）"
    echo " 按 e：读取当前 IMU 角度，限速将左臂 J5/J6/J7 回零并确认后开始跟随"
    echo " 按 s：停止左腕跟随，J5/J6/J7 限速回零并确认后使左臂失能；按 x：左机械臂立即电子急停"
  elif [ "${RIGHT_WRIST_IMU}" = "1" ]; then
    echo " 按 i：开启/关闭 IMU 坐标诊断（默认关闭，不影响控制）"
    echo " 按 e：读取当前 IMU 角度，限速将右臂 J5/J6/J7 回零并确认后开始跟随"
    echo " 按 s：停止右腕跟随，J5/J6/J7 限速回零并确认后使右臂失能；按 x：右机械臂立即电子急停"
  fi
  if [ "${PICO_ARM}" = "1" ]; then
    echo " 按 q：${PICO_ARM_CN}臂分阶段回自然下垂零位并失能"
    [ "${PICO_ARM_HAND}" = "0" ] || echo "       随后${PICO_ARM_CN}灵巧手缓慢张开并退出"
  else
    echo " 按 q：末端 J5/J6/J7 限速回零、机械臂失能、灵巧手自然张开后退出"
  fi
  if [ "${RIGHT_ONLY}" = "1" ]; then
    echo " 右手骨架页面：http://192.168.10.100:8766（本地浏览器直接打开）"
  fi
  if [ "${PICO_ARM}" = "1" ]; then
    echo " 启动后默认不使能；请托稳${PICO_ARM_CN}臂，先做小幅单自由度测试，再按 e。"
  else
    echo " 启动后默认不使能；请先让双手远离夹点，再按 h。"
  fi
  echo "============================================================"
  teleop_args=(--config "${ROOT_DIR}/config/teleop_config.yaml")
  if [ "${PICO_ARM}" = "1" ]; then
    teleop_args+=(--arm-only --arm-side "${PICO_ARM_SIDE}" --require-hand-imu)
    [ "${PICO_ARM_HAND}" = "0" ] || teleop_args+=(--with-hand)
    [ "${NO_HW}" = "1" ] && teleop_args+=(--no-robot)
  elif [ "${LEFT_WRIST_IMU}" = "1" ]; then
    teleop_args+=(--left-wrist-imu)
    [ "${NO_HW}" = "1" ] && teleop_args+=(--no-robot)
  elif [ "${RIGHT_WRIST_IMU}" = "1" ]; then
    teleop_args+=(--right-wrist-imu)
    [ "${NO_HW}" = "1" ] && teleop_args+=(--no-robot)
  else
    teleop_args+=(--hand-only)
  fi
  [ "${LEFT_ONLY}" = "1" ] && teleop_args+=(--hand-side left)
  [ "${RIGHT_ONLY}" = "1" ] && teleop_args+=(--hand-side right)
  [ "${AUTO_ENABLE}" = "1" ] && teleop_args+=(--enable)
  PYTHONPATH="${ROOT_DIR}/src" "${CONDA_PYTHON}" -m esrobo_teleop.teleop_node \
    "${teleop_args[@]}"
fi

echo "==> done."
