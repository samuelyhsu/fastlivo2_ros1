#!/usr/bin/env bash
# ============================================================
#  实时数据采集 一键启动脚本
#  启动: rosbridge + LiDAR + 相机 + rosbag 录制，随后启动 FAST-LIVO2 建图
#  用法（已软链到 ~/.local/bin/collect，任意目录可直接调用）:
#     collect                  # rosbridge + 采集录包 + 建图
#     collect --no-record      # 只起传感器，不录包
#     collect --no-mapping     # 只采集，不跑建图
#     collect --no-rosbridge   # 不起 rosbridge websocket
#     BAG_DIR=/data/bags collect
#     MAPPING_DELAY=10 collect   # 传感器就绪等待秒数（默认 5）
#  停止: Ctrl+C （会等待 roslaunch 干净退出，bag 正常收尾）
# ============================================================
set -euo pipefail

# 工作区根目录（脚本所在目录的上一级，解析软链接以支持 ~/.local/bin/collect 调用）
SELF="$(readlink -f "${BASH_SOURCE[0]}")"
WS="$(cd "$(dirname "$SELF")/.." && pwd)"

# ROS 环境
ROS_SETUP="/opt/ros/noetic/setup.bash"
RECORD="true"
MAPPING="true"
ROSBRIDGE="true"
BAG_DIR="${BAG_DIR:-$WS/dataset/rec}"
# 传感器 launch 起来后，等待多少秒再启动建图
MAPPING_DELAY="${MAPPING_DELAY:-5}"

for arg in "$@"; do
  case "$arg" in
    --no-record)    RECORD="false" ;;
    --no-mapping)   MAPPING="false" ;;
    --no-rosbridge) ROSBRIDGE="false" ;;
    -h|--help)   sed -n '2,${/^#/!q; s/^# \{0,1\}//p;}' "$0"; exit 0 ;;
    *) echo "Unknown argument: $arg"; exit 1 ;;
  esac
done

# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1091
source "$WS/devel/setup.bash"

mkdir -p "$BAG_DIR"

echo "==============================================="
echo " workspace: $WS"
echo " record   : $RECORD"
echo " mapping  : $MAPPING"
echo " rosbridge: $ROSBRIDGE"
echo " bag dir  : $BAG_DIR"
echo " free disk: $(df -h "$BAG_DIR" | awk 'NR==2{print $4}')"
echo "==============================================="

PIDS=()
ROSCORE_PID=""

cleanup() {
  trap - INT TERM EXIT
  echo
  echo ">>> Stopping ..."
  # 先停建图，再停传感器/录包，保证 bag 正常收尾
  for ((i = ${#PIDS[@]} - 1; i >= 0; i--)); do
    kill -INT "${PIDS[i]}" 2>/dev/null || true
  done
  for pid in "${PIDS[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
  # 最后关闭本脚本自己启动的 roscore
  if [[ -n "$ROSCORE_PID" ]]; then
    kill -INT "$ROSCORE_PID" 2>/dev/null || true
    wait "$ROSCORE_PID" 2>/dev/null || true
  fi
  echo ">>> Exited"
}
trap cleanup INT TERM EXIT

# 0) 先起 roscore 并等就绪
#    否则多个 roslaunch 并发时会各自 auto-start master，抢 11311 端口，
#    导致 "Address already in use" / "run_id does not match" 而整体启动失败
if rostopic list &>/dev/null; then
  echo ">>> roscore already running, reuse it"
else
  echo ">>> Starting roscore ..."
  roscore &
  ROSCORE_PID=$!
  for _ in $(seq 1 30); do
    rostopic list &>/dev/null && break
    if ! kill -0 "$ROSCORE_PID" 2>/dev/null; then
      echo "!!! roscore failed to start"
      exit 1
    fi
    sleep 1
  done
  if ! rostopic list &>/dev/null; then
    echo "!!! roscore start timeout"
    exit 1
  fi
fi

# 1) rosbridge websocket（供外部前端订阅话题）
if [[ "$ROSBRIDGE" == "true" ]]; then
  roslaunch rosbridge_server rosbridge_websocket.launch &
  PIDS+=($!)
fi

# 2) 传感器 + 录包
roslaunch "$WS/collect/record_data.launch" \
  record:="$RECORD" \
  bag_dir:="$BAG_DIR" &
SENSOR_PID=$!
PIDS+=("$SENSOR_PID")

# 3) 等传感器起来后再启动 FAST-LIVO2 建图
if [[ "$MAPPING" == "true" ]]; then
  echo ">>> Waiting ${MAPPING_DELAY}s for sensors before starting mapping ..."
  sleep "$MAPPING_DELAY"
  # 传感器 launch 若已挂掉就不必再起建图
  if ! kill -0 "$SENSOR_PID" 2>/dev/null; then
    echo "!!! Sensor launch already exited, skip mapping"
    exit 1
  fi
  roslaunch fast_livo mapping_mid360.launch &
  PIDS+=($!)
fi

# 任一进程退出即整体收尾
wait -n "${PIDS[@]}"
