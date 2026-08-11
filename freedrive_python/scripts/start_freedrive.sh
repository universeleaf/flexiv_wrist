#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOT_SN="${ROBOT_SN:-Rizon4s-123456}"
ROBOT_INTERFACE_IP="${ROBOT_INTERFACE_IP:-127.0.0.1}"
PYTHON_BIN="${PYTHON_BIN:-$PROJECT_ROOT/.venv/bin/python}"

if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "错误：找不到项目 Python：$PYTHON_BIN" >&2
  echo "请先运行：python3 -m venv .venv && .venv/bin/python -m pip install -r requirements.txt" >&2
  exit 2
fi

usage() {
  cat <<'EOF'
用法：
  ./scripts/start_freedrive.sh [其他 run_freedrive.py 参数]

默认机器人：
  Rizon4s-123456
默认机器人网口：
  127.0.0.1（PC LAN1）

默认使用中间脚踏板切换：
  第一次踩下：开启 FloatingCartesian TCP 拖动
  第二次踩下：Stop()，回到锁定/IDLE
  程序启动时保持锁定，不执行 Home。
  启动时自动执行 ZeroFTSensor；此时不要触碰机器人。
  通过 X11 XInput 只监听 PCsensor FootSwitch，不需要 /dev/input 权限。

也可以先设置环境变量：
  ROBOT_SN=Rizon4s-其他序列号 ./scripts/start_freedrive.sh

仅使用 Flexiv 使能按钮（按住开启、松开停止）：
  ./scripts/start_freedrive.sh --no-foot-pedal

紧急情况使用实体急停；Ctrl+C 结束整个程序。
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ -n "${1:-}" && "${1:0:1}" != "-" ]]; then
  ROBOT_SN="$1"
  shift
fi

if [[ -z "$ROBOT_SN" ]]; then
  echo "错误：必须提供 Rizon 4S 序列号，例如 Rizon4s-123456。" >&2
  usage >&2
  exit 2
fi

if [[ "${ROBOT_SN,,}" != rizon4s-* || "$ROBOT_SN" == *" "* ]]; then
  echo "错误：序列号必须以 Rizon4s- 开头且不能包含空格。" >&2
  exit 2
fi

USE_FOOT_PEDAL=1
PASS_ARGS=()
for arg in "$@"; do
  if [[ "$arg" == "--no-foot-pedal" ]]; then
    USE_FOOT_PEDAL=0
  else
    PASS_ARGS+=("$arg")
  fi
done

if [[ "$USE_FOOT_PEDAL" -eq 1 ]]; then
  cat <<EOF
机器人：$ROBOT_SN
模式：FloatingCartesian TCP Freedrive（X/Y/Z/Rx/Ry/Rz）
关节独立拖动：关闭（enableElbowMotion=0，不使用 FloatingJoint）
踏板后端：XInput 物理设备识别（不使用 /dev/input，不把键盘 B 当踏板）
RDK 网卡白名单：$ROBOT_INTERFACE_IP（隔离 Wi-Fi DDS/RTPS）

启动后不会执行 Home，机器人先进入 Stop/锁定状态。
程序会自动执行 ZeroFTSensor；完成前不要触碰法兰、工具或机器人。
第一次踩中间踏板 = 解锁笛卡尔拖动；再踩一次 = 停止并锁定。
异常运动立即再次踩中间踏板；必要时按实体急停。
EOF
  "$PYTHON_BIN" "$PROJECT_ROOT/scripts/run_freedrive.py" \
    "$ROBOT_SN" \
    --network-interface-ip "$ROBOT_INTERFACE_IP" \
    --confirm-motion \
    --foot-pedal \
    --foot-pedal-backend xinput \
    "${PASS_ARGS[@]}"
else
  cat <<EOF
机器人：$ROBOT_SN
模式：FloatingCartesian（Flexiv 使能按钮按住运行）

启动后不会执行 Home；终端会要求输入 EXECUTE FREEDRIVE。
EOF
  "$PYTHON_BIN" "$PROJECT_ROOT/scripts/run_freedrive.py" \
    "$ROBOT_SN" \
    --network-interface-ip "$ROBOT_INTERFACE_IP" \
    --confirm-motion \
    "${PASS_ARGS[@]}"
fi
