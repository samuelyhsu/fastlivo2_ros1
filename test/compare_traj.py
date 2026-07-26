#!/usr/bin/env python3
"""FAST-LIVO2 回归测试：轨迹对比、阈值计算与判定。

由 regress.sh 调用，也可单独使用。位置单位 m，姿态单位 deg。

子命令：
  build-baseline  N 条轨迹 -> 阈值 + 中位轨迹 + meta.json
  check           基线 + 候选轨迹 -> 判定表，PASS/FAIL
  list            列出所有基线
  rss-slope       RSS 采样 csv -> 后半段线性拟合斜率 (MB/s)

退出码：0=PASS  1=FAIL  2=错误
"""

import argparse
import json
import math
import os
import sys
import unicodedata
from datetime import datetime, timezone

# (key, 显示名, 单位, 是否参与判定)，顺序即打印顺序
#
# 末帧偏差只打印不判定：它恒 <= 最大偏差（同一组逐帧距离的末值 vs 最大值），
# 能报的问题最大偏差都能报；但它由"最后一次分叉恰好发生在何处"主导，
# 实测跨度约 10 倍（0.03~0.28），最大偏差只有 2.6 倍（0.11~0.29）。
# 用它做判据只会在无任何改动时误报。
METRICS = [
    ("pos_rmse", "位置 RMSE", "m", True),
    ("final", "末帧偏差", "m", False),
    ("max", "最大偏差", "m", True),
    ("max_rot_deg", "最大姿态偏差", "deg", True),
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
                die(f"{path}:{lineno} 期望 8 列 TUM 格式，实际 {len(parts)} 列")
            try:
                traj[parts[0]] = tuple(float(v) for v in parts[1:])
            except ValueError:
                die(f"{path}:{lineno} 数值解析失败: {line.strip()}")
    if not traj:
        die(f"{path} 为空，节点可能未写出轨迹（检查 evo/pose_output_en）")
    return traj


def rot_angle_deg(qa, qb):
    """两个四元数 (qx,qy,qz,qw) 之间的相对旋转角。"""
    dot = abs(sum(a * b for a, b in zip(qa, qb)))
    return 2.0 * math.degrees(math.acos(min(1.0, dot)))


def compare(a, b):
    """两条轨迹在共同时间戳上的指标。返回 (metrics, n_common)。"""
    keys = sorted(set(a) & set(b))
    if not keys:
        die("两条轨迹没有共同时间戳，无法对比")
    pos = [math.dist(a[k][:3], b[k][:3]) for k in keys]
    rot = [rot_angle_deg(a[k][3:], b[k][3:]) for k in keys]
    return {
        "pos_rmse": math.sqrt(sum(d * d for d in pos) / len(pos)),
        "final": pos[-1],
        "max": max(pos),
        "max_rot_deg": max(rot),
    }, len(keys)


def fmt(value, _unit):
    return f"{value:.4f}"


def width(text):
    """终端显示宽度：CJK 字符占两列。"""
    return sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in text)


def lpad(text, n):
    return " " * max(0, n - width(text)) + text


def rpad(text, n):
    return text + " " * max(0, n - width(text))


def cmd_build_baseline(args):
    if len(args.traj) < 2:
        die("基线至少需要 2 条轨迹才能估计噪声，建议 3 条以上")

    trajs = [load_traj(p) for p in args.traj]

    # 两两对比，每个指标取全组最大值作为观测噪声
    n = len(trajs)
    observed = {key: 0.0 for key, _, _, _ in METRICS}
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
                        f"第 {i + 1} 轮：与最接近的另一轮相差 {fmt(d, 'm')}，"
                        f"是其余轮次典型值 {fmt(ref, 'm')} 的 {d / ref:.1f} 倍")

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
    }
    with open(os.path.join(args.dir, "meta.json"), "w") as fh:
        json.dump(meta, fh, indent=2, ensure_ascii=False)
        fh.write("\n")

    print(f"\n基线已生成: {args.dir}")
    print(f"  轮数 {len(trajs)} @ {args.rate}x    中位轨迹 = 第 {medoid + 1} 轮")
    print(f"  硬指标  LiDAR {args.lidar_frames}  有效图像 {args.valid_images}  轨迹点 {args.traj_points}")
    print(f"\n  {rpad('指标', 18)}{lpad('实测噪声', 12)}{lpad(f'阈值(×{args.safety})', 16)}")
    for key, label, unit, judged in METRICS:
        note = "" if judged else "   (仅参考)"
        print(f"  {rpad(label, 18)}{lpad(fmt(observed[key], unit), 12)}"
              f"{lpad(fmt(threshold[key], unit), 16)}{note}")
    if len(trajs) == 2:
        print("\n  ! 只有 2 轮 = 1 组对比，噪声估计偏乐观，建议 -r 3 以上")
    if contaminated:
        print("\n  !! 基线里可能混进了一轮发散运行，阈值被抬高、将测不出真实回归：")
        for c in contaminated:
            print(f"     - {c}")
        print("     建议重新生成基线；若反复出现，说明该数据源上算法本身不稳定。")
    return EXIT_PASS


def cmd_check(args):
    meta_path = os.path.join(args.dir, "meta.json")
    if not os.path.isfile(meta_path):
        die(f"基线不存在: {meta_path}")
    with open(meta_path) as fh:
        meta = json.load(fh)

    base = load_traj(os.path.join(args.dir, "traj.txt"))
    cand = load_traj(args.traj)
    metrics, n_common = compare(base, cand)

    print(f"\n基线  {meta['name']}   ({meta['runs']} 轮 {meta['rate']}x, {meta['created'][:10]})")
    if meta.get("host") and args.host and meta["host"] != args.host:
        print(f"  ! 基线生成于 {meta['host']}，当前 {args.host}；CPU 核数不同会改变 OpenMP 划分，建议重建基线")
    # 配置文件"内容"变了正是要检测的对象；但"文件名"都不同就是拿两套配置在比，无意义
    for field, got in (("cfg", args.cfg), ("cam_cfg", args.cam_cfg)):
        if got and meta.get(field) and meta[field] != got:
            die(f"配置不一致：基线用 {meta[field]}，当前用 {got}。"
                f"换配置文件请另建一套基线。")

    # bag 身份
    if args.bag_size is not None and args.bag_size != meta["bag"]["size_bytes"]:
        die(f"数据源不一致：基线 bag {meta['bag']['size_bytes']} 字节，当前 {args.bag_size} 字节")
    if args.bag_topics and json.loads(args.bag_topics) != meta["bag"]["topics"]:
        die("数据源不一致：话题消息数与基线不符")

    failures = []

    print("\n硬指标")
    hard = [
        ("LiDAR 帧数", args.lidar_frames, meta["hard"]["lidar_frames"]),
        ("有效图像帧数", args.valid_images, meta["hard"]["valid_images"]),
        ("轨迹点数", args.traj_points, meta["hard"]["traj_points"]),
    ]
    for label, got, want in hard:
        ok = got == want
        if not ok:
            failures.append(f"{label} {got} != 基线 {want}")
        print(f"  {rpad(label, 18)}{lpad(str(got), 8)}  基线 {rpad(str(want), 8)}"
              f"{'OK' if ok else 'FAIL'}")

    ratio = n_common / meta["hard"]["traj_points"]
    ok = ratio >= 0.95
    if not ok:
        failures.append(f"共同时间戳仅 {ratio:.1%}，更新调度已改变")
    print(f"  {rpad('共同时间戳', 18)}{lpad(str(n_common), 8)}  "
          f"基线 {rpad(str(meta['hard']['traj_points']), 8)}"
          f"{'OK' if ok else 'FAIL'}  ({ratio:.1%})")

    print(f"\n轨迹指标{lpad('实测', 12)}{lpad('阈值', 10)}{lpad('基线噪声', 12)}")
    marginal = []
    for key, label, unit, judged in METRICS:
        got, limit = metrics[key], meta["threshold"][key]
        if not judged:
            verdict = "参考"
        elif got <= limit:
            verdict = "OK"
            if limit > 0 and got / limit >= NEAR_THRESHOLD:
                marginal.append(f"{label} 已占阈值 {got / limit:.0%}")
        else:
            verdict = "FAIL"
            failures.append(f"{label} {fmt(got, unit)} > 阈值 {fmt(limit, unit)}")
        print(f"  {rpad(label, 18)}{lpad(fmt(got, unit), 10)}{lpad(fmt(limit, unit), 10)}"
              f"{lpad(fmt(meta['observed'][key], unit), 12)}   {verdict}")

    if failures:
        print("\n=> FAIL")
        for f in failures:
            print(f"   - {f}")
        return EXIT_FAIL
    print("\n=> PASS   效果无变化（偏差落在噪声基线内）")
    if marginal:
        print("   ! 余量不足，基线噪声可能估计偏低，建议 -r 5 重建基线：")
        for m in marginal:
            print(f"     - {m}")
    return EXIT_PASS


def cmd_list(args):
    if not os.path.isdir(args.root):
        die(f"基线目录不存在: {args.root}")
    names = sorted(
        d for d in os.listdir(args.root)
        if os.path.isfile(os.path.join(args.root, d, "meta.json"))
    )
    if not names:
        print(f"{args.root} 下没有基线。先跑: regress.sh baseline <bag>")
        return EXIT_PASS
    print(f"{rpad('名字', 34)}{rpad('生成日期', 12)}{lpad('轮数', 6)}  bag")
    for name in names:
        with open(os.path.join(args.root, name, "meta.json")) as fh:
            meta = json.load(fh)
        print(f"{rpad(name, 34)}{rpad(meta['created'][:10], 12)}{lpad(str(meta['runs']), 6)}  "
              f"{os.path.basename(meta['bag']['path'])}")
        thr = meta["threshold"]
        print("      阈值  " + "  ".join(
            f"{label} {fmt(thr[key], unit)}" for key, label, unit, judged in METRICS if judged))
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
