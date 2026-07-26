#!/usr/bin/env bash
# ============================================================
#  FAST-LIVO2 one-shot regression test
#
#  Repeated runs on the same data are byte-identical, so the verdict really is
#  "the pose did not move": the noise measured while building a baseline is 0,
#  hence so are the thresholds. (That holds only since the two undefined
#  behaviours in vio.cpp were fixed on 2026-07-26 -- see readme.md.)
#
#  Usage (symlink into ~/.local/bin/ to call it from anywhere):
#     # before changing code: 3 runs at 1x -> baseline trajectory + thresholds
#     regress.sh baseline dataset/livo_2026-07-25-22-50-36.bag
#
#     # after changing code: 1 run at 5x, compared against the baseline
#     regress.sh check
#
#     regress.sh list                        # list existing baselines
#     regress.sh baseline <bag> -n NAME -r 5 # baseline name and run count
#     regress.sh check -n NAME --rate 1      # baseline name and replay rate
#     regress.sh baseline <bag> --rate 5     # build the baseline at 5x too
#     regress.sh baseline <bag> --cfg avia.yaml --cam-cfg camera_pinhole.yaml
#
#  --rate defaults to 1 for baseline and 5 for check. A sped-up baseline is
#  much faster to build and, on the datasets tried so far, produces the same
#  frames and the same trajectory; but it is the replay rate that decides which
#  frames republish (queue depth 1) drops, so if a rate ever does drop frames
#  the hard metrics stored in the baseline are the ones from that rate and a
#  check at another rate will fail on them. Backlog is checked on every run, so
#  a rate the algorithm cannot keep up with exits 3 instead of writing a
#  worthless baseline.
#
#  One dataset, one baseline: every -n NAME is a directory baseline/NAME/ and
#  its meta.json records the bag path and the config files. So check only needs
#  the baseline name: dataset and config follow it (pass --cfg/--cam-cfg to
#  override). With more than one baseline, check requires -n.
#
#  Exit codes: 0=PASS  1=FAIL  2=environment/run error  3=unreliable (backlog)
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
CFG_SET=false     # 命令行显式指定过 --cfg
CAM_CFG_SET=false # 命令行显式指定过 --cam-cfg
SAFETY=2.0
RUNS=3
# 留空表示"命令行没给"，由各子命令填自己的默认值：baseline 求稳用 1x，
# check 求快用 5x。共用一个常量默认值会让 baseline 悄悄变成 5x。
RATE=""
RATE_BASELINE_DEFAULT=1
RATE_CHECK_DEFAULT=5
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

# ---------- 阶段计时 ----------
# 加速回放只压缩"replay"这一段，其余都是与倍速无关的固定开销，
# 所以按阶段记账，才看得出继续加倍速还有没有意义。
# 用法: t0=$(now); ...; mark "阶段名" "$t0"    同名多次调用自动累加（跑 N 轮时）
SCRIPT_T0=$(date +%s.%N)
STAGE_ORDER=()
declare -A STAGE_SUM=()

now() { date +%s.%N; }

mark() {
  local name="$1" dt
  dt=$(echo "$(now) - $2" | bc)
  if [[ -z "${STAGE_SUM[$name]:-}" ]]; then
    STAGE_ORDER+=("$name")
    STAGE_SUM["$name"]="$dt"
  else
    STAGE_SUM["$name"]=$(echo "${STAGE_SUM[$name]} + $dt" | bc)
  fi
}

print_stages() {
  [[ ${#STAGE_ORDER[@]} -gt 0 ]] || return 0
  local wall acc=0 name
  wall=$(echo "$(now) - $SCRIPT_T0" | bc)
  for name in "${STAGE_ORDER[@]}"; do
    acc=$(echo "$acc + ${STAGE_SUM[$name]}" | bc)
  done
  echo
  log "time breakdown"
  for name in "${STAGE_ORDER[@]}"; do
    printf '      %-26s %7.2f s  %5.1f%%\n' \
      "$name" "${STAGE_SUM[$name]}" "$(echo "scale=4; 100 * ${STAGE_SUM[$name]} / $wall" | bc)"
  done
  # 未计入的零碎：source 环境、参数解析、进程间的空隙
  printf '      %-26s %7.2f s  %5.1f%%\n' \
    "unaccounted" "$(echo "$wall - $acc" | bc)" "$(echo "scale=4; 100 * ($wall - $acc) / $wall" | bc)"
  printf '      %-26s %7.2f s\n' "TOTAL (wall clock)" "$wall"
}

# ---------- 清理 ----------
ROSCORE_PID=""
LAUNCH_PID=""
RSS_PID=""

# set -e 静默退出很难排查，明确报出失败位置
trap 'rc=$?; warn "line $LINENO failed (exit $rc): ${BASH_COMMAND}"' ERR

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
    log "reusing the running roscore"
    return
  fi
  log "starting roscore ..."
  roscore >"$WORK/roscore.log" 2>&1 &
  ROSCORE_PID=$!
  for _ in $(seq 1 30); do
    rostopic list &>/dev/null && return
    kill -0 "$ROSCORE_PID" 2>/dev/null || err "roscore failed to start, see $WORK/roscore.log"
    sleep 1
  done
  err "roscore start timed out"
}

# SIGKILL 掉的节点不会自己从 master 注销，残留项会让下一轮的"等节点起来/
# 等订阅上"直接假阳性通过、提前开播丢帧。这里按名字定点清掉它们的注册。
# 必须在 roslaunch 退出后调用，否则会把还活着的节点从 master 上摘掉。
purge_dead_nodes() {
  python3 - <<'PYEOF' >/dev/null 2>&1 || true
import rosgraph, rosnode
rosnode.cleanup_master_blacklist(rosgraph.Master("/regress_purge"),
                                 ["/laserMapping", "/republish"])
PYEOF
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

  # 上一次异常中断（或外部 kill -9）会在 master 里留下残留注册，
  # 那会让下面"等节点起来"瞬间假阳性通过，于是没人订阅就开播，整轮空跑。
  purge_dead_nodes

  local t_stage
  t_stage=$(now)
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
  [[ "$up" == true ]] || err "mapping node did not start, see $node_log"
  mark "node startup" "$t_stage"

  t_stage=$(now)
  local lid_topic
  lid_topic=$(rosparam get /common/lid_topic 2>/dev/null | tr -d "'\"" || true)
  if [[ -n "$lid_topic" ]]; then
    for _ in $(seq 1 60); do
      rostopic info "$lid_topic" 2>/dev/null | grep -q laserMapping && break
      sleep 0.5
    done
  fi
  sleep 2
  mark "wait for subscription" "$t_stage"

  # 用 pgrep -o 取最早的匹配进程，避免 `pgrep | head -1` 在 pipefail 下的 SIGPIPE
  local node_pid
  node_pid=$(pgrep -of fastlivo_mapping || true)
  [[ -n "$node_pid" ]] || err "cannot find the fastlivo_mapping process"

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

  log "replaying at ${rate}x ..."
  local t_play_start t_play_end
  t_play_start=$(date +%s.%N)
  rosbag play -r "$rate" --queue 10000 "$bag" >"$WORK/play_$label.log" 2>&1
  t_play_end=$(date +%s.%N)
  R_PLAY=$(echo "$t_play_end - $t_play_start" | bc)
  mark "bag replay (${rate}x)" "$t_play_start"

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
  mark "queue drain" "$t_play_end"
  R_DRAIN=$(echo "$(date +%s.%N) - $t_play_end - $STABLE_NEED * $POLL" | bc)
  # 减去稳定判定本身的开销，负数归零
  [[ $(echo "$R_DRAIN < 0" | bc) == 1 ]] && R_DRAIN=0

  t_stage=$(now)
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
  mark "parse node log" "$t_stage"

  # 建图节点自己退出要 ~17s：析构那张 1.4GB 的体素地图。而此刻轨迹已经逐帧
  # open/append/endl/close 落盘、日志也已解析完，它的退出过程再没有我们要的
  # 东西，所以直接 SIGKILL。实测把单轮收尾从 15.8s 压到 0.3s。
  t_stage=$(now)
  kill -9 "$node_pid" 2>/dev/null || true
  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  LAUNCH_PID=""
  purge_dead_nodes
  mark "node shutdown" "$t_stage"

  [[ -s "$traj_src" ]] || err "no trajectory produced, see $node_log"
  R_TRAJ="$WORK/traj_$label.txt"
  mv "$traj_src" "$R_TRAJ"
  R_POINTS=$(wc -l <"$R_TRAJ")

  log "replay ${R_PLAY} s  drain ${R_DRAIN} s  RSS peak ${R_RSS_PEAK} MB  slope ${R_RSS_SLOPE} MB/s"
  log "LiDAR ${R_LIDAR} frames  valid images ${R_IMAGES}  trajectory points ${R_POINTS}"
}

# 只用排空耗时判积压。RSS 斜率仅作参考打印，不作判据：
# FAST-LIVO2 建图时体素地图持续增长，1x 正常运行实测就有约 15MB/s 的上升，
# 无法把"地图增长"和"buffer 积压"区分开，用作判据必然误报。
backlog_check() {
  if [[ $(echo "$R_DRAIN > $DRAIN_LIMIT" | bc) == 1 ]]; then
    warn "drain took ${R_DRAIN} s, over the ${DRAIN_LIMIT} s limit: the algorithm cannot keep up with ${RATE}x, messages piled up in the buffer"
    warn "result is unreliable, it proves neither a change nor the absence of one. Re-run with --rate 1."
    print_stages
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
  0) err "no baseline yet. Run: $(basename "$SELF") baseline <bag>" ;;
  1) echo "$BASELINE_ROOT/${found[0]}" ;;
  *) err "several baselines, pick one with -n: ${found[*]}" ;;
  esac
}

# ---------- 子命令 ----------
cmd_baseline() {
  local bag="${1:-}"
  [[ -n "$bag" ]] || err "usage: $(basename "$SELF") baseline <bag> [-n NAME] [-r RUNS]"
  shift
  parse_flags "$@"
  RATE="${RATE:-$RATE_BASELINE_DEFAULT}"
  [[ -f "$bag" ]] || err "no such bag: $bag"
  bag="$(readlink -f "$bag")"
  [[ -n "$NAME" ]] || NAME="$(basename "$bag" .bag)"

  local t_stage
  t_stage=$(now)
  ensure_roscore
  mark "roscore" "$t_stage"
  log "baseline '$NAME': $RUNS runs at ${RATE}x on $(basename "$bag")"

  local trajs=() i
  local lidar="" images="" points=""
  for i in $(seq 1 "$RUNS"); do
    log "--- run $i/$RUNS ---"
    run_once "$RATE" "base$i" "$bag"
    # 积压跑出来的基线是废的：阈值由丢帧噪声主导，之后再也测不出回归
    backlog_check
    trajs+=("$R_TRAJ")
    if [[ -z "$lidar" ]]; then
      lidar="$R_LIDAR" images="$R_IMAGES" points="$R_POINTS"
    elif [[ "$lidar" != "$R_LIDAR" || "$images" != "$R_IMAGES" || "$points" != "$R_POINTS" ]]; then
      err "run $i disagrees with the earlier runs (LiDAR $R_LIDAR/$lidar frames, images $R_IMAGES/$images, trajectory $R_POINTS/$points points) --
    the machine itself is unstable, so the baseline would be worthless. Check for other processes eating CPU, or re-run with --rate 1."
    fi
  done

  t_stage=$(now)
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
    --host "$(hostname)" --rate "$RATE" \
    --lidar-frames "$lidar" --valid-images "$images" --traj-points "$points"
  mark "bag scan + compare" "$t_stage"

  print_stages
}

cmd_check() {
  parse_flags "$@"
  RATE="${RATE:-$RATE_CHECK_DEFAULT}"
  local dir
  dir="$(resolve_baseline)"
  [[ -f "$dir/meta.json" ]] || err "no such baseline: $dir"

  # 数据源与配置都从基线里取：换数据集只需换 -n，不必每次重写 --cfg。
  # 命令行显式给过的以命令行为准（用于"就是要拿另一套配置比一比"的场景，
  # compare_traj.py 会因配置名不符直接报错，这正是期望行为）。
  local meta_vals=()
  mapfile -t meta_vals < <(python3 -c '
import json, sys
m = json.load(open(sys.argv[1]))
print(m["bag"]["path"])
print(m.get("cfg", ""))
print(m.get("cam_cfg", ""))' "$dir/meta.json")
  [[ ${#meta_vals[@]} -ge 3 ]] || err "cannot parse baseline meta.json: $dir/meta.json"

  local bag="$WS/${meta_vals[0]}"
  [[ -f "$bag" ]] || err "the bag recorded in the baseline is missing: $bag"
  if [[ "$CFG_SET" != true && -n "${meta_vals[1]}" ]]; then CFG="${meta_vals[1]}"; fi
  if [[ "$CAM_CFG_SET" != true && -n "${meta_vals[2]}" ]]; then CAM_CFG="${meta_vals[2]}"; fi
  log "dataset $(basename "$bag")  config $CFG + $CAM_CFG"

  local t_stage
  t_stage=$(now)
  ensure_roscore
  mark "roscore" "$t_stage"

  # 跑一轮就够：同一份数据重复运行的输出逐字节一致（见 readme），
  # FAIL 不会是抽样噪声。历史上这里会在 FAIL 时复测一次，理由是"算法的
  # 不确定性是重尾的、单轮误报率约 8%"——那个不确定性来自两个未定义行为，
  # 已于 2026-07-26 修掉，复测只是白跑一轮。
  local rc=0
  run_once "$RATE" "check" "$bag"
  backlog_check
  t_stage=$(now)
  python3 "$PY" check \
    --dir "$dir" --traj "$R_TRAJ" \
    --lidar-frames "$R_LIDAR" --valid-images "$R_IMAGES" --traj-points "$R_POINTS" \
    --bag-size "$(stat -c %s "$bag")" \
    --bag-topics "$(bag_topics_json "$bag")" \
    --host "$(hostname)" --cfg "$CFG" --cam-cfg "$CAM_CFG" || rc=$?
  mark "bag scan + compare" "$t_stage"

  print_stages
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
      CFG_SET=true
      shift 2
      ;;
    --cam-cfg)
      CAM_CFG="$2"
      CAM_CFG_SET=true
      shift 2
      ;;
    -h | --help)
      usage
      exit 0
      ;;
    *) err "unknown option: $1" ;;
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
*) err "unknown subcommand: $SUB (available: baseline / check / list)" ;;
esac
