#!/usr/bin/env bash
# ============================================================================
# SenseGlove Nova 2 序列号识别 + 自动配置
#
# 前提：两只手套已通过蓝牙连接，且电脑上有 SenseCom。
# 本脚本自动：
#   1) 确保 SenseCom 运行（未运行则启动）
#   2) 用 senseglove_api 的 sg_tester 枚举已连接手套，识别左右序列号
#   3) 把左右序列号写入 senseglove_bringup/config/gloves.yaml
#   4) 打印可直接运行的遥操作命令
#
# 用法：
#   ./scripts/senseglove_auto_config.sh
#   ./scripts/senseglove_auto_config.sh --no-write   # 只识别并打印，不改 gloves.yaml
# ============================================================================
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SENSEGLOVE_WS="${SENSEGLOVE_WS:-${ROOT_DIR}/external/senseglove_ros}"
GLOVES_YAML="${SENSEGLOVE_WS}/senseglove/senseglove_bringup/config/gloves.yaml"
# 真实 SenseCom（Unity 程序，需 SenseCom_Data 和 UnityPlayer.so 在旁）
SENSECOM_BIN="${SENSEGLOVE_WS}/senseglove_com/SenseCom/Linux/SenseCom_Linux_Latest/SenseCom.x86_64"
SG_TESTER="${SENSEGLOVE_WS}/install/senseglove_api/lib/senseglove_api/sg_tester"

WRITE=1
[[ "${1:-}" == "--no-write" ]] && WRITE=0

# ---------------------------------------------------------------- ROS2 env
set +u
source /opt/ros/humble/setup.bash
if [ -f "${SENSEGLOVE_WS}/install/setup.bash" ]; then source "${SENSEGLOVE_WS}/install/setup.bash"; fi
set -u

# SenseCom 是 Unity GUI，必须要有 X 显示。探测可用 DISPLAY（默认本机 :1）。
if [ -z "${DISPLAY:-}" ]; then
  for d in :1 :0 :2; do
    if [ -e "/tmp/.X11-unix/X${d#:}" ]; then DISPLAY="$d"; break; fi
  done
fi
export DISPLAY
echo "==> DISPLAY=${DISPLAY:-<none>}  (SenseCom 需要图形界面)"

[ -x "${SG_TESTER}" ] || { echo "ERROR: sg_tester not built. Run scripts/build_external_sdks.sh"; exit 1; }

echo "==> 1) 确保 SenseCom 运行"
if pgrep -f "SenseCom.x86_64" >/dev/null 2>&1; then
  echo "   SenseCom 已在运行."
else
  if [ -x "${SENSECOM_BIN}" ]; then
    echo "   启动 SenseCom..."
    "${SENSECOM_BIN}" >/dev/null 2>&1 &
    echo "   等待 SenseCom 检测手套..."
    sleep 8
  else
    echo "   WARN: 未找到 SenseCom (${SENSECOM_BIN})。请手动启动 SenseCom。"
  fi
fi

echo "==> 2) 枚举已连接手套（识别序列号）"
OUT="$("${SG_TESTER}" 2>&1)" || true
echo "$OUT"

SERIALS="$(echo "$OUT" | python3 -c '
import sys, re
left=""; right=""
for line in sys.stdin:
    m=re.search(r"ID:\s*(\S+)", line)
    r=re.search(r"isRight:\s*(true|false)", line)
    if m:
        serial=m.group(1)
        is_right = bool(r and r.group(1)=="true")
        if is_right: right=serial
        else: left=serial
print("LEFT=" + (left or "NONE"))
print("RIGHT=" + (right or "NONE"))
')"
LEFT_SERIAL="$(echo "$SERIALS" | sed -n 's/^LEFT=//p')"
RIGHT_SERIAL="$(echo "$SERIALS" | sed -n 's/^RIGHT=//p')"

if [ -z "${LEFT_SERIAL}" ] || [ -z "${RIGHT_SERIAL}" ]; then
  echo "ERROR: 未识别到左右手套序列号。请确认 SenseCom 已检测到两只手套。" >&2
  exit 1
fi
echo "==> 识别结果：LEFT=${LEFT_SERIAL}  RIGHT=${RIGHT_SERIAL}"

# ------------------------------------------------------- 写入 gloves.yaml
if [ "${WRITE}" = "1" ]; then
  echo "==> 3) 写入 ${GLOVES_YAML}"
  python3 - "$LEFT_SERIAL" "$RIGHT_SERIAL" "$GLOVES_YAML" <<'PY'
import sys, re
left, right, path = sys.argv[1], sys.argv[2], sys.argv[3]
out = []
cur_side = None
serials = {"left": left, "right": right}
with open(path, "r", encoding="utf-8") as f:
    for ln in f:
        m_side = re.search(r"side:\s*(\w+)", ln)
        if m_side:
            cur_side = m_side.group(1)
        m_ser = re.match(r"(\s*serial:).*", ln)
        if m_ser and cur_side in serials:
            out.append(f"{m_ser.group(1)} \"{serials[cur_side]}\"\n")
            continue
        out.append(ln)
with open(path, "w", encoding="utf-8") as f:
    f.writelines(out)
print("   gloves.yaml 已更新: left->"+left+"  right->"+right)
PY
else
  echo "==> 3) (--no-write) 未修改 gloves.yaml"
fi

# ------------------------------------------------------- 打印运行命令
echo
echo "================================================================"
echo " 配置完成！运行一键遥操作："
echo "  cd ${ROOT_DIR}"
echo "  ./scripts/run_senseglove_hand_teleop.sh \\"
echo "      --left-serial ${LEFT_SERIAL} --right-serial ${RIGHT_SERIAL}"
echo "================================================================"
