#!/usr/bin/env python3
"""FAST-LIVO2 regression test: trajectory comparison, thresholds and verdict.

Called by regress.sh, also usable standalone.
Positions are in meters, rotations in degrees.

Subcommands:
  build-baseline  N trajectories -> thresholds + medoid trajectory + meta.json
  check           baseline + candidate trajectory -> verdict table, PASS/FAIL
  list            list all baselines
  rss-slope       RSS sample csv -> slope of second half, linear fit (MB/s)

Exit codes: 0=PASS  1=FAIL  2=ERROR
"""

import argparse
import json
import math
import os
import sys
import unicodedata
from datetime import datetime, timezone

# (key, 显示名, 列表短名, 单位, 是否参与判定)，顺序即打印顺序
#
# 末帧偏差（位置与姿态）只打印不判定：它恒 <= 对应的最大偏差（同一组逐帧
# 距离的末值 vs 最大值），能报的问题最大偏差都能报；但它由"最后一次分叉
# 恰好发生在何处"主导，实测跨度约 10 倍（0.03~0.28），最大偏差只有
# 2.6 倍（0.11~0.29）。用它做判据只会在无任何改动时误报。
METRICS = [
    ("pos_rmse", "Position RMSE", "rmse", "m", True),
    ("final", "Final drift", "final", "m", False),
    ("max", "Max drift", "max", "m", True),
    ("final_rot_deg", "Final rotation drift", "final rot", "deg", False),
    ("max_rot_deg", "Max rotation drift", "rot", "deg", True),
]

# 判定通过但已占到阈值这么高的比例时提示：说明标定余量不足
NEAR_THRESHOLD = 0.70

EXIT_PASS, EXIT_FAIL, EXIT_ERROR = 0, 1, 2


def die(msg):
    print(f"!! {msg}", file=sys.stderr)
    sys.exit(EXIT_ERROR)


def load_traj(path):
    """读 TUM 轨迹：timestamp x y z qx qy qz qw。

    时间戳保留原始字符串作为 key —— FAST-LIVO2 以固定 6 位小数写出，
    跨运行逐字节一致，比浮点比较更可靠。
    """
    traj = {}
    with open(path) as fh:
        for lineno, line in enumerate(fh, 1):
            parts = line.split()
            if not parts:
                continue
            if len(parts) != 8:
                die(f"{path}:{lineno} expected 8 TUM columns, got {len(parts)}")
            try:
                traj[parts[0]] = tuple(float(v) for v in parts[1:])
            except ValueError:
                die(f"{path}:{lineno} cannot parse numbers: {line.strip()}")
    if not traj:
        die(f"{path} is empty, the node may not have written a trajectory "
            f"(check evo/pose_output_en)")
    return traj


def rot_angle_deg(qa, qb):
    """两个四元数 (qx,qy,qz,qw) 之间的相对旋转角。

    两处都必须讲究，否则轨迹完全相同也报不出 0：

    - 先归一化。轨迹文件只写 6 位小数，四元数落盘后不再是单位长度（实测模平方
      约 0.999998），不归一化会把这点量化误差当成旋转差，自己跟自己比就有 0.18°。
    - 用相对四元数 + atan2，而不是点积 + acos。acos 在自变量趋近 1 时条件数极差，
      即便归一化过，同一条轨迹自比仍会残留约 2e-06 度；而两条轨迹相同时相对四元数
      的虚部逐项精确抵消为 0，atan2(0, |w|) 恰好给出 0。
    """
    ax, ay, az, aw = qa
    bx, by, bz, bw = qb
    na = math.sqrt(ax * ax + ay * ay + az * az + aw * aw)
    nb = math.sqrt(bx * bx + by * by + bz * bz + bw * bw)
    if na == 0.0 or nb == 0.0:
        return 0.0
    ax, ay, az, aw = ax / na, ay / na, az / na, aw / na
    bx, by, bz, bw = bx / nb, by / nb, bz / nb, bw / nb

    # q_rel = conj(qa) * qb；取 |w| 是因为 q 与 -q 表示同一旋转
    rw = aw * bw + ax * bx + ay * by + az * bz
    rx = aw * bx - ax * bw - ay * bz + az * by
    ry = aw * by - ay * bw - az * bx + ax * bz
    rz = aw * bz - az * bw - ax * by + ay * bx
    return 2.0 * math.degrees(
        math.atan2(math.sqrt(rx * rx + ry * ry + rz * rz), abs(rw)))


def closure_error(traj):
    """末点相对世界原点的位置/姿态偏差，返回 (米, 度)。

    测试数据通常绕一圈回到起点，而 FAST-LIVO2 的世界系锚定在首帧，
    所以"末点离原点多远、姿态偏了多少"直接就是这一趟的累计漂移。

    只打印不判定：它衡量的是算法精度，与本工具要回答的"改动有没有改变行为"
    是两个问题。判定该指标会把"算法本来就有的漂移"误报成回归。
    """
    last = traj[max(traj)]
    return (math.dist(last[:3], (0.0, 0.0, 0.0)),
            rot_angle_deg(last[3:], (0.0, 0.0, 0.0, 1.0)))


def start_offset(traj):
    """首点离原点的距离。不接近 0 说明这条轨迹压根不是从原点起步，
    此时 closure_error 量的就不是闭环误差，得提醒一句。"""
    return math.dist(traj[min(traj)][:3], (0.0, 0.0, 0.0))


def print_closure(rows):
    """rows: [(标签, traj), ...]，同一张表里并排打印各条轨迹的闭环误差。"""
    print("\nLoop closure   (not judged, this is algorithm accuracy)")
    table = [["metric"] + [label for label, _ in rows]]
    table.append(["Distance from origin [m]"]
                 + [fmt(closure_error(t)[0]) for _, t in rows])
    table.append(["Rotation from origin [deg]"]
                 + [fmt(closure_error(t)[1]) for _, t in rows])
    print_table(table, ["<"] + [">"] * len(rows))
    for label, traj in rows:
        off = start_offset(traj)
        if off > 0.05:
            print(f"  ! {label} starts {fmt(off)} m from the origin, "
                  f"so the numbers above are not a loop-closure error")


def compare(a, b):
    """两条轨迹在共同时间戳上的指标。返回 (metrics, n_common)。"""
    keys = sorted(set(a) & set(b))
    if not keys:
        die("the two trajectories share no timestamp, cannot compare")
    pos = [math.dist(a[k][:3], b[k][:3]) for k in keys]
    rot = [rot_angle_deg(a[k][3:], b[k][3:]) for k in keys]
    return {
        "pos_rmse": math.sqrt(sum(d * d for d in pos) / len(pos)),
        "final": pos[-1],
        "max": max(pos),
        "final_rot_deg": rot[-1],
        "max_rot_deg": max(rot),
    }, len(keys)


def fmt(value):
    return f"{value:.4f}"


def label_of(label, unit):
    return f"{label} [{unit}]"


def width(text):
    """终端显示宽度：CJK 字符占两列。基线名可能带中文，不能用 len()。"""
    return sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in text)


def lpad(text, n):
    return " " * max(0, n - width(text)) + text


def rpad(text, n):
    return text + " " * max(0, n - width(text))


def print_table(rows, aligns, indent="  ", gap=3):
    """按内容自适应列宽打印。aligns 逐列给 '<'(左) 或 '>'(右)。

    列宽取该列最宽单元格，所以加长指标名或多跑几位小数都不会错位。
    """
    widths = [max(width(row[i]) for row in rows) for i in range(len(aligns))]
    for row in rows:
        cells = [(lpad if a == ">" else rpad)(row[i], widths[i])
                 for i, a in enumerate(aligns)]
        print((indent + (" " * gap).join(cells)).rstrip())


def cmd_build_baseline(args):
    if len(args.traj) < 2:
        die("a baseline needs at least 2 trajectories to estimate noise, 3+ recommended")

    trajs = [load_traj(p) for p in args.traj]

    # 两两对比，每个指标取全组最大值作为观测噪声
    n = len(trajs)
    observed = {key: 0.0 for key, *_ in METRICS}
    rmse = [[0.0] * n for _ in range(n)]  # 两两 pos_rmse 矩阵
    rmse_sum = [0.0] * n
    for i in range(n):
        for j in range(i + 1, n):
            metrics, _ = compare(trajs[i], trajs[j])
            for key in observed:
                observed[key] = max(observed[key], metrics[key])
            rmse[i][j] = rmse[j][i] = metrics["pos_rmse"]
            rmse_sum[i] += metrics["pos_rmse"]
            rmse_sum[j] += metrics["pos_rmse"]

    # 基线里混进一轮发散会把阈值抬高一个数量级，工具就再也测不出回归。
    #
    # 不能用"两两偏差的 max/median"来识别：n=3 时混入 1 条发散轨迹会污染
    # 3 组对比中的 2 组，中位数本身就被抬高，比值接近 1 而漏报。
    # 改用最近邻距离——正常轨迹至少有一个正常邻居（距离小），
    # 发散的那条对所有邻居都远。
    contaminated = []
    if n >= 3:
        nearest = [min(rmse[i][j] for j in range(n) if j != i) for i in range(n)]
        ref = sorted(nearest)[len(nearest) // 2]
        if ref > 0:
            for i, d in enumerate(nearest):
                if d / ref > 3.0:
                    contaminated.append(
                        f"run {i + 1}: {fmt(d)} m from its closest neighbour, "
                        f"{d / ref:.1f}x the typical {fmt(ref)} m of the other runs")

    threshold = {k: v * args.safety for k, v in observed.items()}

    # 中位轨迹：到其余轨迹位置 RMSE 之和最小的那条
    medoid = min(range(len(trajs)), key=lambda i: rmse_sum[i])

    os.makedirs(args.dir, exist_ok=True)
    with open(args.traj[medoid]) as src, open(os.path.join(args.dir, "traj.txt"), "w") as dst:
        dst.write(src.read())

    meta = {
        "name": os.path.basename(os.path.normpath(args.dir)),
        "created": datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds"),
        "bag": {
            "path": args.bag_path,
            "size_bytes": args.bag_size,
            "topics": json.loads(args.bag_topics),
        },
        "cfg": args.cfg,
        "cam_cfg": args.cam_cfg,
        "commit": args.commit,
        "submodule_commit": args.submodule_commit,
        "host": args.host,
        "runs": len(trajs),
        "rate": args.rate,
        "hard": {
            "lidar_frames": args.lidar_frames,
            "valid_images": args.valid_images,
            "traj_points": args.traj_points,
        },
        "safety_factor": args.safety,
        "observed": {k: round(v, 4) for k, v in observed.items()},
        "threshold": {k: round(v, 4) for k, v in threshold.items()},
        # 中位轨迹的闭环误差，仅供参考，不参与判定
        "closure": dict(zip(("pos_m", "rot_deg"),
                            (round(v, 4) for v in closure_error(trajs[medoid])))),
    }
    with open(os.path.join(args.dir, "meta.json"), "w") as fh:
        json.dump(meta, fh, indent=2, ensure_ascii=False)
        fh.write("\n")

    print(f"\nBaseline written: {args.dir}")
    print(f"  {len(trajs)} runs @ {args.rate}x, medoid = run {medoid + 1}")
    print(f"  hard metrics  {args.lidar_frames} LiDAR frames, "
          f"{args.valid_images} valid images, {args.traj_points} trajectory points")

    print()
    rows = [["metric", "observed noise", f"threshold (x{args.safety})", ""]]
    for key, label, _short, unit, judged in METRICS:
        rows.append([label_of(label, unit), fmt(observed[key]), fmt(threshold[key]),
                     "" if judged else "(not judged)"])
    print_table(rows, ["<", ">", ">", "<"])

    print_closure([(f"run {i + 1}", t) for i, t in enumerate(trajs)])

    if len(trajs) == 2:
        print("\n  ! only 2 runs = 1 pair, the noise estimate is optimistic; use -r 3 or more")
    if contaminated:
        print("\n  !! one run may have diverged: thresholds are inflated "
              "and real regressions will be missed")
        for c in contaminated:
            print(f"     - {c}")
        print("     rebuild the baseline; if it keeps happening the algorithm "
              "is unstable on this dataset.")
    return EXIT_PASS


def cmd_check(args):
    meta_path = os.path.join(args.dir, "meta.json")
    if not os.path.isfile(meta_path):
        die(f"no such baseline: {meta_path}")
    with open(meta_path) as fh:
        meta = json.load(fh)

    base = load_traj(os.path.join(args.dir, "traj.txt"))
    cand = load_traj(args.traj)
    metrics, n_common = compare(base, cand)

    print(f"\nBaseline  {meta['name']}   "
          f"({meta['runs']} runs @ {meta['rate']}x, {meta['created'][:10]})")
    if meta.get("host") and args.host and meta["host"] != args.host:
        print(f"  ! baseline was built on {meta['host']}, now on {args.host}; "
              f"a different core count changes the OpenMP split, rebuild it")
    # 配置文件"内容"变了正是要检测的对象；但"文件名"都不同就是拿两套配置在比，无意义
    for field, got in (("cfg", args.cfg), ("cam_cfg", args.cam_cfg)):
        if got and meta.get(field) and meta[field] != got:
            die(f"config mismatch: baseline uses {meta[field]}, this run uses {got}. "
                f"Build a separate baseline for a different config file.")

    # bag 身份
    if args.bag_size is not None and args.bag_size != meta["bag"]["size_bytes"]:
        die(f"dataset mismatch: baseline bag is {meta['bag']['size_bytes']} bytes, "
            f"this one is {args.bag_size} bytes")
    if args.bag_topics and json.loads(args.bag_topics) != meta["bag"]["topics"]:
        die("dataset mismatch: per-topic message counts differ from the baseline")

    failures = []

    print("\nHard metrics")
    rows = [["metric", "measured", "baseline", "", ""]]
    for label, got, want in (
        ("LiDAR frames", args.lidar_frames, meta["hard"]["lidar_frames"]),
        ("Valid images", args.valid_images, meta["hard"]["valid_images"]),
        ("Trajectory points", args.traj_points, meta["hard"]["traj_points"]),
    ):
        ok = got == want
        if not ok:
            failures.append(f"{label} {got} != baseline {want}")
        rows.append([label, str(got), str(want), "OK" if ok else "FAIL", ""])

    ratio = n_common / meta["hard"]["traj_points"]
    ok = ratio >= 0.95
    if not ok:
        failures.append(f"only {ratio:.1%} of timestamps are shared, "
                        f"the update schedule has changed")
    rows.append(["Common timestamps", str(n_common), str(meta["hard"]["traj_points"]),
                 "OK" if ok else "FAIL", f"({ratio:.1%})"])
    print_table(rows, ["<", ">", ">", "<", "<"])

    print("\nTrajectory metrics")
    rows = [["metric", "measured", "threshold", "baseline noise", ""]]
    marginal = []
    stale = []
    for key, label, _short, unit, judged in METRICS:
        got = metrics[key]
        # 新增的指标在老基线里没有阈值，只报测量值，重建基线后才纳入
        if key not in meta["threshold"]:
            stale.append(label_of(label, unit))
            rows.append([label_of(label, unit), fmt(got), "-", "-", "(not in baseline)"])
            continue
        limit = meta["threshold"][key]
        if not judged:
            verdict = "(not judged)"
        elif got <= limit:
            verdict = "OK"
            if limit > 0 and got / limit >= NEAR_THRESHOLD:
                marginal.append(f"{label_of(label, unit)} is at {got / limit:.0%} of threshold")
        else:
            verdict = "FAIL"
            failures.append(f"{label_of(label, unit)} {fmt(got)} > threshold {fmt(limit)}")
        rows.append([label_of(label, unit), fmt(got), fmt(limit),
                     fmt(meta["observed"][key]), verdict])
    print_table(rows, ["<", ">", ">", ">", "<"])
    if stale:
        print(f"  ! this baseline predates {', '.join(stale)}; "
              f"rebuild it to get the threshold for those")

    print_closure([("measured", cand), ("baseline", base)])

    if failures:
        print("\n=> FAIL")
        for f in failures:
            print(f"   - {f}")
        return EXIT_FAIL
    print("\n=> PASS   no change in behaviour (every judged metric is within its threshold)")
    if marginal:
        print("   ! little margin left, the baseline noise may be underestimated; "
              "rebuild with -r 5")
        for m in marginal:
            print(f"     - {m}")
    return EXIT_PASS


def cmd_list(args):
    if not os.path.isdir(args.root):
        die(f"no baseline directory: {args.root}")
    names = sorted(
        d for d in os.listdir(args.root)
        if os.path.isfile(os.path.join(args.root, d, "meta.json"))
    )
    if not names:
        print(f"no baseline under {args.root}. Run: regress.sh baseline <bag>")
        return EXIT_PASS

    # 阈值直接做成列，一行一套基线，长名字也不会把后面的列顶歪
    judged = [(key, short, unit) for key, _label, short, unit, ok in METRICS if ok]
    rows = [["name", "created", "runs", "rate"]
            + [f"{short} [{unit}]" for _key, short, unit in judged] + ["bag"]]
    for name in names:
        with open(os.path.join(args.root, name, "meta.json")) as fh:
            meta = json.load(fh)
        rows.append([name, meta["created"][:10], str(meta["runs"]), f"{meta['rate']}x"]
                    + [fmt(meta["threshold"][key]) for key, _short, _unit in judged]
                    + [os.path.basename(meta["bag"]["path"])])
    print_table(rows, ["<", "<", ">", ">"] + [">"] * len(judged) + ["<"], indent="")
    return EXIT_PASS


def cmd_rss_slope(args):
    """后半段线性拟合斜率 (MB/s)，用于判断内存是否单调增长。"""
    rows = []
    with open(args.csv) as fh:
        next(fh, None)  # 表头
        for line in fh:
            parts = line.strip().split(",")
            if len(parts) == 2:
                try:
                    rows.append((float(parts[0]), float(parts[1]) / 1024.0))
                except ValueError:
                    continue
    if len(rows) < 8:
        print("0.0")  # 样本太少，不判定
        return EXIT_PASS
    rows = rows[len(rows) // 2:]
    n = len(rows)
    mt = sum(t for t, _ in rows) / n
    mv = sum(v for _, v in rows) / n
    den = sum((t - mt) ** 2 for t, _ in rows)
    print(f"{0.0 if den == 0 else sum((t - mt) * (v - mv) for t, v in rows) / den:.2f}")
    return EXIT_PASS


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("build-baseline")
    p.add_argument("--dir", required=True)
    p.add_argument("--traj", nargs="+", required=True)
    p.add_argument("--safety", type=float, default=2.0)
    p.add_argument("--bag-path", required=True)
    p.add_argument("--bag-size", type=int, required=True)
    p.add_argument("--bag-topics", required=True, help="JSON: {topic: count}")
    p.add_argument("--cfg", required=True)
    p.add_argument("--cam-cfg", required=True)
    p.add_argument("--commit", default="")
    p.add_argument("--submodule-commit", default="")
    p.add_argument("--host", default="")
    p.add_argument("--rate", type=float, default=1)
    p.add_argument("--lidar-frames", type=int, required=True)
    p.add_argument("--valid-images", type=int, required=True)
    p.add_argument("--traj-points", type=int, required=True)
    p.set_defaults(func=cmd_build_baseline)

    p = sub.add_parser("check")
    p.add_argument("--dir", required=True)
    p.add_argument("--traj", required=True)
    p.add_argument("--lidar-frames", type=int, required=True)
    p.add_argument("--valid-images", type=int, required=True)
    p.add_argument("--traj-points", type=int, required=True)
    p.add_argument("--bag-size", type=int)
    p.add_argument("--bag-topics")
    p.add_argument("--host", default="")
    p.add_argument("--cfg", default="")
    p.add_argument("--cam-cfg", default="")
    p.set_defaults(func=cmd_check)

    p = sub.add_parser("list")
    p.add_argument("--root", required=True)
    p.set_defaults(func=cmd_list)

    p = sub.add_parser("rss-slope")
    p.add_argument("csv")
    p.set_defaults(func=cmd_rss_slope)

    args = ap.parse_args()
    sys.exit(args.func(args))


if __name__ == "__main__":
    main()
