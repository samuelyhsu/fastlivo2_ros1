#!/usr/bin/env bash
# ============================================================
#  FAST-LIVO2 一键回归测试
#
#  FAST-LIVO2 的输出不可复现（vio.cpp 的 OpenMP 浮点归约影响迭代收敛判断），
#  同倍速重跑两次末帧就差约 5cm。所以判据不是"位姿完全不变"，
#  而是"偏差落在噪声基线内"。
#
#  用法（可软链到 ~/.local/bin/ 后任意目录调用）:
#     # 改代码前：跑 3 轮 1x，生成基线轨迹 + 噪声阈值
#     regress.sh baseline dataset/livo_2026-07-25-22-50-36.bag
#
#     # 改代码后：跑 1 轮 5x 与基线对比
#     regress.sh check
#
#     regress.sh list                       # 列出已有基线
#     regress.sh baseline <bag> -n 名字 -r 5  # 指定基线名与轮数
#     regress.sh check -n 名字 --rate 1       # 指定基线与回放倍速
#     regress.sh baseline <bag> --cfg avia.yaml --cam-cfg camera_pinhole.yaml
#
#  退出码: 0=PASS  1=FAIL  2=环境/运行错误  3=结果不可信(检测到积压)
# ============================================================
set -euo pipefail

SELF="$(readlink -f "${BASH_SOURCE[0]}")"
TEST_DIR="$(dirname "$SELF")"
WS="$(cd "$TEST_DIR/.." && pwd)"
WORK="$TEST_DIR/.work"
BASELINE_ROOT="$TEST_DIR/baseline"
PY="$TEST_DIR/compare_traj.py"
ROS_SETUP="/opt/ros/noetic/setup.bash"
TRAJ_OUT="$WS/src/FAST-LIVO2/Log/result" # 节点硬编码的轨迹输出目录

# 积压判据。DRAIN_LIMIT 用绝对值而非"排空/回放"比值：正常排空近乎瞬时，
# 落后时随积压量线性增长；比值阈值会贴着 5x 的正常值导致误报。
DRAIN_LIMIT=3.0
POLL=0.2      # 排空轮询间隔
STABLE_NEED=3 # 连续多少次行数不变才算排空完成

CFG="mid360.yaml"
CAM_CFG="camera_pinhole_mid360.yaml"
SAFETY=2.0
RUNS=3
RATE=5
NAME=""

EXIT_FAIL=1
EXIT_ERROR=2
EXIT_UNRELIABLE=3

log() { printf '>>> %s\n' "$*"; }
warn() { printf '!!! %s\n' "$*" >&2; }
err() {
  printf '!!! %s\n' "$*" >&2
  exit "$EXIT_ERROR"
}

usage() {
  sed -n '2,${/^#/!q; s/^# \{0,1\}//p;}' "$SELF"
}

# ---------- 清理 ----------
ROSCORE_PID=""
LAUNCH_PID=""
RSS_PID=""

# set -e 静默退出很难排查，明确报出失败位置
trap 'rc=$?; warn "第 $LINENO 行失败（退出码 $rc）: ${BASH_COMMAND}"' ERR

cleanup() {
  trap - INT TERM EXIT ERR
  [[ -n "$RSS_PID" ]] && kill "$RSS_PID" 2>/dev/null || true
  if [[ -n "$LAUNCH_PID" ]]; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  pkill -f fastlivo_mapping 2>/dev/null || true
  pkill -f image_transport/republish 2>/dev/null || true
  if [[ -n "$ROSCORE_PID" ]]; then
    kill -INT "$ROSCORE_PID" 2>/dev/null || true
    wait "$ROSCORE_PID" 2>/dev/null || true
  fi
}
trap cleanup INT TERM EXIT

ensure_roscore() {
  if rostopic list &>/dev/null; then
    log "复用已有 roscore"
    return
  fi
  log "启动 roscore ..."
  roscore >"$WORK/roscore.log" 2>&1 &
  ROSCORE_PID=$!
  for _ in $(seq 1 30); do
    rostopic list &>/dev/null && return
    kill -0 "$ROSCORE_PID" 2>/dev/null || err "roscore 启动失败，见 $WORK/roscore.log"
    sleep 1
  done
  err "roscore 启动超时"
}

bag_topics_json() {
  python3 - "$1" <<'PYEOF'
import json, sys, rosbag
with rosbag.Bag(sys.argv[1]) as b:
    info = b.get_type_and_topic_info().topics
print(json.dumps({t: i.message_count for t, i in info.items()}, sort_keys=True))
PYEOF
}

# ---------- 单轮运行 ----------
# run_once <倍速> <标签> <bag>
# 结果写入全局: R_PLAY R_DRAIN R_RSS_PEAK R_RSS_SLOPE R_LIDAR R_IMAGES R_POINTS R_TRAJ
run_once() {
  local rate="$1" label="$2" bag="$3"
  local seq="regress_${label}_$$"
  local node_log="$WORK/node_$label.log"
  local rss_csv="$WORK/rss_$label.csv"
  local traj_src="$TRAJ_OUT/$seq.txt"

  mkdir -p "$TRAJ_OUT"
  rm -f "$traj_src" "$node_log" "$rss_csv"

  roslaunch "$TEST_DIR/regress.launch" \
    seq:="$seq" cfg:="$CFG" cam_cfg:="$CAM_CFG" >"$node_log" 2>&1 &
  LAUNCH_PID=$!

  # 等节点起来并真正订阅上，否则开头会丢消息导致帧数对不上
  local up=false
  for _ in $(seq 1 120); do
    if rosnode list 2>/dev/null | grep -qx '/laserMapping'; then
      up=true
      break
    fi
    kill -0 "$LAUNCH_PID" 2>/dev/null || break
    sleep 0.5
  done
  [[ "$up" == true ]] || err "建图节点未启动，见 $node_log"

  local lid_topic
  lid_topic=$(rosparam get /common/lid_topic 2>/dev/null | tr -d "'\"" || true)
  if [[ -n "$lid_topic" ]]; then
    for _ in $(seq 1 60); do
      rostopic info "$lid_topic" 2>/dev/null | grep -q laserMapping && break
      sleep 0.5
    done
  fi
  sleep 2

  # 用 pgrep -o 取最早的匹配进程，避免 `pgrep | head -1` 在 pipefail 下的 SIGPIPE
  local node_pid
  node_pid=$(pgrep -of fastlivo_mapping || true)
  [[ -n "$node_pid" ]] || err "找不到 fastlivo_mapping 进程"

  # 子 shell 里不能用 local，这里全部用普通变量
  (
    echo "t_s,rss_kb"
    t0=$(date +%s.%N)
    while kill -0 "$node_pid" 2>/dev/null; do
      rss=$(grep VmRSS "/proc/$node_pid/status" 2>/dev/null | tr -dc '0-9')
      [[ -n "$rss" ]] && echo "$(echo "$(date +%s.%N) - $t0" | bc),$rss"
      sleep 0.5
    done
  ) >"$rss_csv" 2>/dev/null &
  RSS_PID=$!

  log "回放 ${rate}x ..."
  local t_play_start t_play_end
  t_play_start=$(date +%s.%N)
  rosbag play -r "$rate" --queue 10000 "$bag" >"$WORK/play_$label.log" 2>&1
  t_play_end=$(date +%s.%N)
  R_PLAY=$(echo "$t_play_end - $t_play_start" | bc)

  # 排空：轮询轨迹行数直到连续 STABLE_NEED 次不变
  local prev=-1 cur stable=0
  while :; do
    cur=$(wc -l <"$traj_src" 2>/dev/null || echo 0)
    if [[ "$cur" == "$prev" ]]; then
      stable=$((stable + 1))
      [[ $stable -ge $STABLE_NEED ]] && break
    else
      stable=0
      prev=$cur
    fi
    kill -0 "$node_pid" 2>/dev/null || break
    sleep "$POLL"
  done
  R_DRAIN=$(echo "$(date +%s.%N) - $t_play_end - $STABLE_NEED * $POLL" | bc)
  # 减去稳定判定本身的开销，负数归零
  [[ $(echo "$R_DRAIN < 0" | bc) == 1 ]] && R_DRAIN=0

  kill "$RSS_PID" 2>/dev/null || true
  RSS_PID=""

  R_RSS_PEAK=$(sort -t, -k2 -n "$rss_csv" 2>/dev/null | tail -1 | cut -d, -f2 || echo 0)
  R_RSS_PEAK=$(echo "scale=0; ${R_RSS_PEAK:-0} / 1024" | bc)
  R_RSS_SLOPE=$(python3 "$PY" rss-slope "$rss_csv")

  # 帧数统计。有效图像 = header 时间晚于首帧 LiDAR 的图像；
  # 开头那批 header 卡死的垃圾帧从不进入解算，且 republish(队列深度1)
  # 在加速回放时丢的正是它们，计入会造成假失败。
  # 注意 pipefail：`grep | head -1` 会让 grep 被 SIGPIPE 杀掉(141)，
  # 进而使整条管道返回 141 并被 set -e 终止。用 grep -m1 自行截断，不接管道。
  R_LIDAR=$(grep -c 'Get LiDAR' "$node_log" || true)
  local first_lidar
  first_lidar=$(grep -m1 -oP 'Get LiDAR, its header time: \K[0-9.]+' "$node_log" || true)
  if [[ -n "$first_lidar" ]]; then
    R_IMAGES=$({ grep -oP 'Get image, its header time: \K[0-9.]+' "$node_log" || true; } |
      awk -v t="$first_lidar" '$1 + 0 > t + 0' | wc -l)
  else
    R_IMAGES=0
  fi

  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  LAUNCH_PID=""

  [[ -s "$traj_src" ]] || err "未产出轨迹，见 $node_log"
  R_TRAJ="$WORK/traj_$label.txt"
  mv "$traj_src" "$R_TRAJ"
  R_POINTS=$(wc -l <"$R_TRAJ")

  log "play ${R_PLAY}s  排空 ${R_DRAIN}s  RSS峰值 ${R_RSS_PEAK}MB  斜率 ${R_RSS_SLOPE}MB/s"
  log "LiDAR $R_LIDAR  有效图像 $R_IMAGES  轨迹点 $R_POINTS"
}

# 只用排空耗时判积压。RSS 斜率仅作参考打印，不作判据：
# FAST-LIVO2 建图时体素地图持续增长，1x 正常运行实测就有约 15MB/s 的上升，
# 无法把"地图增长"和"buffer 积压"区分开，用作判据必然误报。
backlog_check() {
  if [[ $(echo "$R_DRAIN > $DRAIN_LIMIT" | bc) == 1 ]]; then
    warn "排空耗时 ${R_DRAIN}s 超过 ${DRAIN_LIMIT}s，算法跟不上 ${RATE}x —— 消息在 buffer 里积压"
    warn "结果不可信 —— 既不能证明变了、也不能证明没变。请用 --rate 1 重跑。"
    exit "$EXIT_UNRELIABLE"
  fi
}

resolve_baseline() {
  if [[ -n "$NAME" ]]; then
    echo "$BASELINE_ROOT/$NAME"
    return
  fi
  local found=()
  if [[ -d "$BASELINE_ROOT" ]]; then
    local d
    for d in "$BASELINE_ROOT"/*/; do
      [[ -f "$d/meta.json" ]] && found+=("$(basename "$d")")
    done
  fi
  case ${#found[@]} in
  0) err "没有任何基线。先跑: $(basename "$SELF") baseline <bag>" ;;
  1) echo "$BASELINE_ROOT/${found[0]}" ;;
  *) err "有多套基线，请用 -n 指定: ${found[*]}" ;;
  esac
}

# ---------- 子命令 ----------
cmd_baseline() {
  local bag="${1:-}"
  [[ -n "$bag" ]] || err "用法: $(basename "$SELF") baseline <bag> [-n 名字] [-r 轮数]"
  shift
  parse_flags "$@"
  [[ -f "$bag" ]] || err "bag 不存在: $bag"
  bag="$(readlink -f "$bag")"
  [[ -n "$NAME" ]] || NAME="$(basename "$bag" .bag)"

  ensure_roscore
  log "基线 '$NAME'：$RUNS 轮 1x，$(basename "$bag")"

  local trajs=() i
  local lidar="" images="" points=""
  for i in $(seq 1 "$RUNS"); do
    log "--- 第 $i/$RUNS 轮 ---"
    run_once 1 "base$i" "$bag"
    trajs+=("$R_TRAJ")
    if [[ -z "$lidar" ]]; then
      lidar="$R_LIDAR" images="$R_IMAGES" points="$R_POINTS"
    elif [[ "$lidar" != "$R_LIDAR" || "$images" != "$R_IMAGES" || "$points" != "$R_POINTS" ]]; then
      err "第 $i 轮帧数与前几轮不一致（LiDAR $R_LIDAR/$lidar 图像 $R_IMAGES/$images 轨迹 $R_POINTS/$points）——
    环境本身不稳定，基线不可信。检查是否有别的进程抢 CPU，或改用 1x 重跑。"
    fi
  done

  python3 "$PY" build-baseline \
    --dir "$BASELINE_ROOT/$NAME" \
    --traj "${trajs[@]}" \
    --safety "$SAFETY" \
    --bag-path "$(realpath --relative-to="$WS" "$bag")" \
    --bag-size "$(stat -c %s "$bag")" \
    --bag-topics "$(bag_topics_json "$bag")" \
    --cfg "$CFG" --cam-cfg "$CAM_CFG" \
    --commit "$(git -C "$WS" rev-parse --short HEAD 2>/dev/null || echo '')" \
    --submodule-commit "$(git -C "$WS/src/FAST-LIVO2" rev-parse --short HEAD 2>/dev/null || echo '')" \
    --host "$(hostname)" --rate 1 \
    --lidar-frames "$lidar" --valid-images "$images" --traj-points "$points"
}

cmd_check() {
  parse_flags "$@"
  local dir
  dir="$(resolve_baseline)"
  [[ -f "$dir/meta.json" ]] || err "基线不存在: $dir"

  local bag
  bag="$WS/$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["bag"]["path"])' "$dir/meta.json")"
  [[ -f "$bag" ]] || err "基线记录的 bag 不存在: $bag"

  ensure_roscore

  # FAIL 需要复现才算数。FAST-LIVO2 的不确定性是重尾的：实测 13 轮 check 中
  # 有 1 轮偏差达中位数的 18 倍（单轮误报率约 8%），且 1x/5x 都会出现。
  # 放松阈值会让工具测不出真回归，所以改为首轮 FAIL 时复测一次，
  # 两轮都 FAIL 才判定回归；连续两次抽到长尾的概率约 0.6%。
  local attempt rc=0
  for attempt in 1 2; do
    run_once "$RATE" "check$attempt" "$bag"
    backlog_check
    rc=0
    python3 "$PY" check \
      --dir "$dir" --traj "$R_TRAJ" \
      --lidar-frames "$R_LIDAR" --valid-images "$R_IMAGES" --traj-points "$R_POINTS" \
      --bag-size "$(stat -c %s "$bag")" \
      --bag-topics "$(bag_topics_json "$bag")" \
      --host "$(hostname)" --cfg "$CFG" --cam-cfg "$CAM_CFG" || rc=$?

    # 只有"指标越界"才复测；环境错误(2)直接返回
    [[ $rc -eq $EXIT_FAIL ]] || break
    if [[ $attempt -eq 1 ]]; then
      echo
      warn "首轮 FAIL。算法存在重尾不确定性，单轮可能误报，复测一次确认 ..."
    fi
  done

  if [[ $rc -eq $EXIT_FAIL ]]; then
    echo
    warn "两轮均 FAIL —— 判定为真实回归。"
  fi
  exit "$rc"
}

parse_flags() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
    -n | --name)
      NAME="$2"
      shift 2
      ;;
    -r | --runs)
      RUNS="$2"
      shift 2
      ;;
    --rate)
      RATE="$2"
      shift 2
      ;;
    --safety)
      SAFETY="$2"
      shift 2
      ;;
    --cfg)
      CFG="$2"
      shift 2
      ;;
    --cam-cfg)
      CAM_CFG="$2"
      shift 2
      ;;
    -h | --help)
      usage
      exit 0
      ;;
    *) err "未知参数: $1" ;;
    esac
  done
}

# ---------- 入口 ----------
[[ $# -gt 0 ]] || {
  usage
  exit "$EXIT_ERROR"
}
SUB="$1"
shift

mkdir -p "$WORK"
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1091
source "$WS/devel/setup.bash"

case "$SUB" in
baseline) cmd_baseline "$@" ;;
check) cmd_check "$@" ;;
list) python3 "$PY" list --root "$BASELINE_ROOT" ;;
-h | --help)
  usage
  exit 0
  ;;
*) err "未知子命令: $SUB（可用: baseline / check / list）" ;;
esac
