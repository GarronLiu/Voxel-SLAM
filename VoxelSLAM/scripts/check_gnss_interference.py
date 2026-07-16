#!/usr/bin/env python3
"""Detect suspected GNSS interference in a ROS1 bag."""

import argparse
import math
import os
import statistics

import rosbag


def percentile(values, ratio):
    values = sorted(values)
    return values[int((len(values) - 1) * ratio)]


def epoch_metrics(msg, stamp, min_cn0):
    sat_cn0, lli, status = [], [], []
    for obs in msg.meas:
        cn0 = [x for x in obs.CN0 if math.isfinite(x) and x > 0]
        if cn0:
            sat_cn0.append(max(cn0))
        lli.extend(obs.LLI)
        status.extend(obs.status)
    if not sat_cn0:
        return None
    return {
        "t": stamp,
        "cn0": statistics.median(sat_cn0),
        "sats": sum(x >= min_cn0 for x in sat_cn0),
        "lli": sum(bool(x) for x in lli) / max(1, len(lli)),
        "invalid": sum(not (x & 1) for x in status) / max(1, len(status)),
    }


def merge_intervals(rows, min_duration):
    intervals, start = [], None
    gaps = [b["t"] - a["t"] for a, b in zip(rows, rows[1:]) if b["t"] > a["t"]]
    step = statistics.median(gaps) if gaps else 0.0
    for i, row in enumerate(rows):
        if start is not None and step and row["t"] - rows[i - 1]["t"] > 2.5 * step:
            if rows[i - 1]["t"] - rows[start]["t"] + step >= min_duration:
                intervals.append((rows[start]["t"], rows[i - 1]["t"] + step))
            start = None
        if row["bad"] and start is None:
            start = i
        if start is not None and (not row["bad"] or i == len(rows) - 1):
            end = i if row["bad"] else i - 1
            if rows[end]["t"] - rows[start]["t"] + step >= min_duration:
                intervals.append((rows[start]["t"], rows[end]["t"] + step))
            start = None
    return intervals


def plot_metrics(rows, intervals, base_cn0, base_sats, args):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t0 = rows[0]["t"]
    times = [x["t"] - t0 for x in rows]
    fig, axes = plt.subplots(3, 1, figsize=(11, 7), sharex=True)
    axes[0].plot(times, [x["cn0"] for x in rows], lw=1)
    axes[0].axhline(base_cn0 - args.cn0_drop, color="r", ls="--")
    axes[0].set_ylabel("Median C/N0 (dB-Hz)")
    axes[1].plot(times, [x["sats"] for x in rows], lw=1)
    axes[1].axhline(base_sats * args.sat_ratio, color="r", ls="--")
    axes[1].set_ylabel("Usable satellites")
    axes[2].plot(times, [100 * x["lli"] for x in rows], label="Lost lock")
    axes[2].plot(times, [100 * x["invalid"] for x in rows], label="Invalid")
    axes[2].set_ylabel("Ratio (%)")
    axes[2].set_xlabel("Time from bag start (s)")
    axes[2].legend()
    for axis in axes:
        axis.grid(alpha=0.3)
        for begin, end in intervals:
            axis.axvspan(begin - t0, end - t0, color="r", alpha=0.15)
    fig.tight_layout()
    path = args.plot or os.path.splitext(args.bag)[0] + "_gnss_interference.png"
    fig.savefig(path, dpi=150)
    print("曲线已保存:", path)


def main():
    parser = argparse.ArgumentParser(description="检测 rosbag 中疑似 GNSS 射频干扰")
    parser.add_argument("bag")
    parser.add_argument("--topic", default="/ublox_driver/range_meas")
    parser.add_argument("--cn0-drop", type=float, default=6.0, help="C/N0 下降阈值(dB-Hz)")
    parser.add_argument("--sat-ratio", type=float, default=0.65, help="卫星数相对基线阈值")
    parser.add_argument("--min-cn0", type=float, default=20.0, help="可用卫星最低 C/N0")
    parser.add_argument("--min-duration", type=float, default=2.0, help="最短异常持续时间(s)")
    parser.add_argument("--plot", nargs="?", const="", metavar="PNG",
                        help="绘制曲线，可选输出文件名")
    args = parser.parse_args()

    rows = []
    with rosbag.Bag(args.bag) as bag:
        for _, msg, t in bag.read_messages(topics=[args.topic]):
            row = epoch_metrics(msg, t.to_sec(), args.min_cn0)
            if row:
                rows.append(row)

    if len(rows) < 10:
        raise SystemExit("有效 GNSS 历元不足 10 个，无法判断。请检查 bag 和话题名。")

    base_cn0 = percentile([x["cn0"] for x in rows], 0.8)
    base_sats = max(1, percentile([x["sats"] for x in rows], 0.8))
    lli_limit = max(0.15, percentile([x["lli"] for x in rows], 0.5) + 0.10)

    for row in rows:
        cn0_bad = base_cn0 - row["cn0"] >= args.cn0_drop
        sat_bad = row["sats"] <= base_sats * args.sat_ratio
        lli_bad = row["lli"] >= lli_limit
        row["bad"] = 2 * cn0_bad + sat_bad + lli_bad + (row["invalid"] >= 0.30) >= 2

    intervals = merge_intervals(rows, args.min_duration)
    abnormal = sum(x["bad"] for x in rows) / len(rows)
    print("基线: C/N0={:.1f} dB-Hz, 可用卫星={}；异常历元={:.1%}".format(
        base_cn0, base_sats, abnormal))
    if args.plot is not None:
        plot_metrics(rows, intervals, base_cn0, base_sats, args)
    if not intervals:
        print("结论: 未发现明显的持续 GNSS 干扰。")
        return

    t0 = rows[0]["t"]
    print("结论: 存在疑似 GNSS 干扰，共 {} 段：".format(len(intervals)))
    for begin, end in intervals:
        print("  bag 起始后 {:.1f}s ~ {:.1f}s（持续 {:.1f}s）".format(
            begin - t0, end - t0, end - begin))
    print("注意: 仅凭 GNSS 数据不能区分工业相机电磁干扰、遮挡和强多径；建议结合相机开关实验复核。")


if __name__ == "__main__":
    main()
