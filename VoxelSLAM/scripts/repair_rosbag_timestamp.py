#!/usr/bin/env python3
"""
逐帧修复 rosbag 中 Livox LiDAR/IMU 的整数秒跳变和压缩图像时间戳异常。

修复依据:
  1. GNSS header.stamp - gnss_local_time_diff 是本地时钟参考；
  2. 用 GNSS 与 bag record time 的关系插值得到每个传感器帧的参考时刻；
  3. 对每帧枚举少量整数秒修正候选，通过动态规划同时最小化
     GNSS 参考误差、相邻帧间隔误差和不必要的修正量切换；
  4. 图像用正常帧估计周期和亚秒相位，修复连续出现的 0/200ms 间隔；
  5. LiDAR header.stamp 与 timebase 使用完全相同的整数纳秒增量。

这种方法不会将低频 GNSS 样本直接配给某一帧，因而不会在采样相位的
半秒边界产生错误的 round() 结果，也能保留各传感器原有的亚秒精度。

用法:
  python3 repair_rosbag_timestamp.py input.bag [output.bag]
"""

import argparse
import math
import os
import sys

import numpy as np
import rosbag
import rospy

NS_PER_SEC = 1_000_000_000


def extract_all_timestamps(input_path, lidar_topic, imu_topic, gnss_topic,
                           image_topic=None):
    """单次遍历 bag，返回各 topic 的 (record_time, header_stamp)。"""
    topics = [lidar_topic, imu_topic, gnss_topic]
    if image_topic:
        topics.append(image_topic)
    data = {topic: [] for topic in topics}
    print("[PASS 1] 提取时间戳: {}".format(input_path))

    with rosbag.Bag(input_path) as bag:
        for topic, msg, record_time in bag.read_messages(topics=topics):
            if not hasattr(msg, "header"):
                continue
            data[topic].append(
                (record_time.to_sec(), msg.header.stamp.to_sec()))

    lidar_data = data[lidar_topic]
    imu_data = data[imu_topic]
    gnss_data = data[gnss_topic]
    print("  LiDAR: {} 帧".format(len(lidar_data)))
    print("  IMU:   {} 帧".format(len(imu_data)))
    print("  GNSS:  {} 帧".format(len(gnss_data)))
    if image_topic:
        image_data = data[image_topic]
        print("  Image: {} 帧".format(len(image_data)))
        return lidar_data, imu_data, gnss_data, image_data
    return lidar_data, imu_data, gnss_data


def _running_median(values, radius):
    """短窗口中值滤波，抑制 GNSS 消息到达时间的偶发抖动。"""
    values = np.asarray(values, dtype=float)
    if radius <= 0 or len(values) < 3:
        return values.copy()
    result = np.empty_like(values)
    for i in range(len(values)):
        left = max(0, i - radius)
        right = min(len(values), i + radius + 1)
        result[i] = np.median(values[left:right])
    return result


class GnssClockReference:
    """
    根据 bag record time 估计本地传感器时钟。

    插值对象是 clock_offset = gnss_local_stamp - record_time，而不是直接
    对绝对 epoch 时间拟合；数值更稳定，且可自然表达缓慢时钟漂移。
    """

    def __init__(self, gnss_data, gnss_local_time_diff, median_radius=2):
        valid = [
            (float(rt), float(st) - gnss_local_time_diff)
            for rt, st in gnss_data
            if math.isfinite(rt) and math.isfinite(st)
        ]
        if not valid:
            raise ValueError("未找到有效 GNSS 时间戳，无法确定绝对秒修正量")

        valid.sort(key=lambda item: item[0])
        # record time 重复时仅保留最后一个，保证 np.interp 的横坐标严格递增。
        unique = []
        for item in valid:
            if unique and item[0] == unique[-1][0]:
                unique[-1] = item
            else:
                unique.append(item)

        self.record_times = np.asarray([item[0] for item in unique])
        raw_offsets = np.asarray([item[1] - item[0] for item in unique])
        self.clock_offsets = _running_median(raw_offsets, median_radius)

        median_offset = float(np.median(raw_offsets))
        mad = float(np.median(np.abs(raw_offsets - median_offset)))
        print("\n[PASS 2] GNSS 时钟参考:")
        print("  有效样本: {}, record->local 偏移中位数: {:.6f}s, MAD: {:.3f}ms"
              .format(len(unique), median_offset, mad * 1000.0))

    def at(self, record_time):
        # np.interp 在两端使用端点值，相当于端点外保持 clock offset 不变，
        # 比对 epoch 时间作远距离线性外推更安全。
        offset = float(np.interp(
            record_time, self.record_times, self.clock_offsets))
        return record_time + offset


def estimate_frame_offsets(data, clock, name, jump_threshold=0.5,
                           candidate_radius=2, switch_penalty=0.02):
    """
    用动态规划为每帧选择整数秒修正量。

    状态是当前帧的整数 offset。观测项约束到 GNSS 插值得到的参考时刻；
    转移项约束修复后的帧间隔与 bag record time 的帧间隔一致。后者使
    位于 round() 边界附近的帧仍能选择与前后帧连续的结果。
    """
    if not data:
        return []

    references = [clock.at(rt) for rt, _ in data]
    candidate_sets = []
    for (_, stamp), reference in zip(data, references):
        center = int(round(reference - stamp))
        candidate_sets.append(
            list(range(center - candidate_radius, center + candidate_radius + 1)))

    # 每个元素: offset -> (累计代价, 上一帧 offset)
    layers = []
    first_layer = {}
    first_stamp = data[0][1]
    for offset in candidate_sets[0]:
        reference_error = abs(first_stamp + offset - references[0])
        first_layer[offset] = (reference_error, None)
    layers.append(first_layer)

    if jump_threshold <= 0.0:
        raise ValueError("jump_threshold 必须大于 0")
    continuity_scale = jump_threshold
    for i in range(1, len(data)):
        record_dt = data[i][0] - data[i - 1][0]
        current_stamp = data[i][1]
        previous_stamp = data[i - 1][1]
        layer = {}

        for offset in candidate_sets[i]:
            corrected = current_stamp + offset
            reference_error = abs(corrected - references[i])
            best = None
            for previous_offset, (previous_cost, _) in layers[-1].items():
                corrected_dt = corrected - (previous_stamp + previous_offset)
                continuity_error = abs(corrected_dt - record_dt)

                # Huber 型连续性代价：小误差强约束，bag 自身有大间隙时
                # 不让单个转移项无限支配 GNSS 观测。
                normalized = continuity_error / continuity_scale
                continuity_cost = (
                    2.5 * normalized * normalized
                    if normalized <= 1.0
                    else 2.5 * (2.0 * normalized - 1.0)
                )
                transition_cost = switch_penalty * abs(
                    offset - previous_offset)
                cost = (previous_cost + reference_error +
                        continuity_cost + transition_cost)
                if best is None or cost < best[0]:
                    best = (cost, previous_offset)
            layer[offset] = best
        layers.append(layer)

    offsets = [0] * len(data)
    offsets[-1] = min(layers[-1], key=lambda key: layers[-1][key][0])
    for i in range(len(data) - 1, 0, -1):
        offsets[i - 1] = layers[i][offsets[i]][1]

    corrected = np.asarray([
        stamp + offset for (_, stamp), offset in zip(data, offsets)
    ])
    record_times = np.asarray([rt for rt, _ in data])
    continuity_residual = (
        np.diff(corrected) - np.diff(record_times)
        if len(data) > 1 else np.asarray([])
    )
    reference_residual = corrected - np.asarray(references)
    switches = sum(a != b for a, b in zip(offsets, offsets[1:]))

    print("\n  {} 逐帧估计:".format(name))
    print("    参考误差: median={:.3f}ms, max={:.3f}ms"
          .format(float(np.median(np.abs(reference_residual))) * 1000.0,
                  float(np.max(np.abs(reference_residual))) * 1000.0))
    if len(continuity_residual):
        print("    连续性残差: median={:.3f}ms, max={:.3f}ms, offset切换={}次"
              .format(float(np.median(np.abs(continuity_residual))) * 1000.0,
                      float(np.max(np.abs(continuity_residual))) * 1000.0,
                      switches))

    unique, counts = np.unique(offsets, return_counts=True)
    for offset, count in zip(unique, counts):
        print("    {:+d}s: {} 帧".format(int(offset), int(count)))
    if name == "LiDAR":
        _print_lidar_anomaly_sources(
            data, references, offsets, jump_threshold)
    return offsets


def _print_lidar_anomaly_sources(data, references, offsets, jump_threshold,
                                 max_items=20):
    """打印导致 LiDAR 秒修正切换或帧间不连续的原始消息位置。"""
    anomalies = []
    for i in range(1, len(data)):
        previous_record, previous_stamp = data[i - 1]
        current_record, current_stamp = data[i]
        record_dt = current_record - previous_record
        raw_dt = current_stamp - previous_stamp
        residual = raw_dt - record_dt
        offset_changed = offsets[i] != offsets[i - 1]
        discontinuous = raw_dt <= 0.0 or abs(residual) > jump_threshold
        if offset_changed or discontinuous:
            anomalies.append((
                i - 1, previous_record, current_record,
                previous_stamp, current_stamp, record_dt, raw_dt, residual,
                offsets[i - 1], offsets[i], references[i]))

    print("\n    LiDAR 原始时间戳异常出处: {} 处".format(len(anomalies)))
    for item in anomalies[:max_items]:
        (index, previous_record, current_record,
         previous_stamp, current_stamp, record_dt, raw_dt, residual,
         previous_offset, current_offset, current_reference) = item
        print("      topic帧 #{} -> #{}:".format(index, index + 1))
        print("        bag record: {:.9f} -> {:.9f}, dt={:.3f}ms"
              .format(previous_record, current_record, record_dt * 1000.0))
        print("        raw header: {:.9f} -> {:.9f}, dt={:.3f}ms"
              .format(previous_stamp, current_stamp, raw_dt * 1000.0))
        print("        dt残差={:+.3f}ms, offset={:+d}s -> {:+d}s, "
              "当前GNSS参考={:.9f}"
              .format(residual * 1000.0, int(previous_offset),
                      int(current_offset), current_reference))
    if len(anomalies) > max_items:
        print("      ... 其余 {} 处省略".format(len(anomalies) - max_items))


def reconstruct_image_timestamps(data, clock):
    """
    根据图像帧序号、bag record time 和 GNSS 参考重建连续时间轴。

    STM32 GPZDA 竞态会产生一帧丢失、一帧重复，表现为相邻 0/200ms，
    此时只修整数秒无法恢复亚秒部分。这里从未受影响的多数帧估计周期，
    用 record time 识别真正的缺帧，再以所有原始帧的中位相位确定起点。
    """
    if not data:
        return []

    references = np.asarray([clock.at(rt) for rt, _ in data], dtype=float)
    record_times = np.asarray([rt for rt, _ in data], dtype=float)
    raw_stamps = np.asarray([stamp for _, stamp in data], dtype=float)

    if len(data) == 1:
        corrected = raw_stamps[0] + round(references[0] - raw_stamps[0])
        return [int(round(corrected * NS_PER_SEC))]

    record_dt = np.diff(record_times)
    positive_record_dt = record_dt[
        np.isfinite(record_dt) & (record_dt > 0.0)]
    if not len(positive_record_dt):
        raise ValueError("图像 bag record time 非递增，无法重建时间轴")

    raw_dt = np.diff(raw_stamps)
    positive_raw_dt = raw_dt[np.isfinite(raw_dt) & (raw_dt > 0.0)]
    # 以 1ms 分箱寻找最常见周期。相比直接取中位数，这在短数据中也不会
    # 把一次 100ms 正常间隔和一次 200ms 缺帧错误平均成 150ms。
    period_candidates = np.concatenate(
        (positive_record_dt, positive_raw_dt))
    period_bins_ms = np.rint(period_candidates * 1000.0).astype(np.int64)
    valid_bins = period_bins_ms > 0
    period_candidates = period_candidates[valid_bins]
    period_bins_ms = period_bins_ms[valid_bins]
    unique_bins, bin_counts = np.unique(period_bins_ms, return_counts=True)
    max_bin_count = int(np.max(bin_counts))
    nominal_bin_ms = int(np.min(unique_bins[bin_counts == max_bin_count]))
    modal_candidates = period_candidates[period_bins_ms == nominal_bin_ms]
    record_period = float(np.median(modal_candidates))

    # 0ms 和 200ms 是异常；优先从接近 record 周期的正常 header 间隔估计。
    valid_raw_dt = raw_dt[
        np.isfinite(raw_dt) &
        (raw_dt > 0.5 * record_period) &
        (raw_dt < 1.5 * record_period)]
    nominal_period = (float(np.median(valid_raw_dt))
                      if len(valid_raw_dt) else record_period)
    if not math.isfinite(nominal_period) or nominal_period <= 0.0:
        raise ValueError("无法估计有效的图像帧周期")

    # 只有 record 和原始 header 都显示跨过多个周期时才认定确实缺帧。
    # 单独出现的 record 到达抖动，或 header 的伪 200ms 跳变，均按一帧推进。
    record_steps = np.maximum(
        1, np.rint(record_dt / nominal_period).astype(np.int64))
    raw_steps = np.maximum(
        1, np.rint(np.maximum(raw_dt, 0.0) / nominal_period).astype(np.int64))
    frame_steps = np.where(
        (record_steps > 1) & (raw_steps > 1),
        np.minimum(record_steps, raw_steps), 1)
    elapsed = np.empty(len(data), dtype=float)
    elapsed[0] = 0.0
    elapsed[1:] = np.cumsum(frame_steps, dtype=float) * nominal_period

    # 先逐帧借助 GNSS 只对齐整数秒，再对 aligned_stamp-elapsed 取中位数。
    # 重复/漏发 GPZDA 只会形成少量 ±一个周期的离群相位，不会改变中位数。
    integer_offsets = np.rint(references - raw_stamps)
    aligned_stamps = raw_stamps + integer_offsets
    origin = float(np.median(aligned_stamps - elapsed))
    corrected = origin + elapsed
    # 防止整个重建序列落在相邻的错误整数秒。
    origin += round(float(np.median(references - corrected)))
    corrected = origin + elapsed

    timestamps_ns = [
        int(round(value * NS_PER_SEC)) for value in corrected
    ]
    corrected_float = np.asarray(timestamps_ns, dtype=float) / NS_PER_SEC
    corrected_dt = np.diff(corrected_float)
    raw_bad = np.where(
        (raw_dt <= 0.0) |
        (np.abs(raw_dt - nominal_period) > 0.25 * nominal_period))[0]
    true_gaps = int(np.count_nonzero(frame_steps > 1))
    reference_error = corrected_float - references

    print("\n  Image 连续时间轴重建:")
    print("    标称周期={:.3f}ms ({:.3f}Hz), 原始异常={}处, "
          "record缺帧间隔={}处"
          .format(nominal_period * 1000.0, 1.0 / nominal_period,
                  len(raw_bad), true_gaps))
    print("    重建后dt: median={:.3f}ms, min={:.3f}ms, max={:.3f}ms"
          .format(float(np.median(corrected_dt)) * 1000.0,
                  float(np.min(corrected_dt)) * 1000.0,
                  float(np.max(corrected_dt)) * 1000.0))
    print("    GNSS参考误差: median={:.3f}ms, max={:.3f}ms"
          .format(float(np.median(np.abs(reference_error))) * 1000.0,
                  float(np.max(np.abs(reference_error))) * 1000.0))
    for i in raw_bad[:10]:
        print("    原始异常 #{} -> #{}: header dt={:.3f}ms, "
              "record dt={:.3f}ms, 重建 dt={:.3f}ms"
              .format(i, i + 1, raw_dt[i] * 1000.0,
                      record_dt[i] * 1000.0, corrected_dt[i] * 1000.0))
    if len(raw_bad) > 10:
        print("    ... 其余 {} 处省略".format(len(raw_bad) - 10))
    return timestamps_ns


def write_repaired_bag(input_path, output_path, lidar_topic, imu_topic,
                       lidar_offsets, imu_offsets, image_topic=None,
                       image_timestamps_ns=None):
    """按 topic 内的帧序号应用结果，避免用浮点 record_time 作字典键。"""
    indices = {lidar_topic: 0, imu_topic: 0}
    offsets = {lidar_topic: lidar_offsets, imu_topic: imu_offsets}
    counts = {lidar_topic: 0, imu_topic: 0}
    if image_topic:
        indices[image_topic] = 0
        counts[image_topic] = 0

    print("\n[PASS 3] 写入修复后 rosbag: {}".format(output_path))
    with rosbag.Bag(input_path) as inbag, rosbag.Bag(output_path, "w") as outbag:
        for topic, msg, record_time in inbag.read_messages():
            if (image_topic and topic == image_topic and
                    indices[topic] < len(image_timestamps_ns or [])):
                timestamp_ns = int(image_timestamps_ns[indices[topic]])
                indices[topic] += 1
                msg.header.stamp = rospy.Time(
                    timestamp_ns // NS_PER_SEC,
                    timestamp_ns % NS_PER_SEC)
                counts[topic] += 1
            elif topic in offsets and indices[topic] < len(offsets[topic]):
                offset_sec = int(offsets[topic][indices[topic]])
                indices[topic] += 1

                # 修正量是整数秒，直接改 secs 可无损保留 nsecs。
                msg.header.stamp = rospy.Time(
                    msg.header.stamp.secs + offset_sec,
                    msg.header.stamp.nsecs)

                if topic == lidar_topic and hasattr(msg, "timebase"):
                    corrected_timebase = int(msg.timebase) + offset_sec * NS_PER_SEC
                    if corrected_timebase < 0:
                        raise ValueError(
                            "LiDAR timebase 修复后为负数: {}".format(
                                corrected_timebase))
                    msg.timebase = corrected_timebase
                counts[topic] += 1

            outbag.write(topic, msg, record_time)

    summary = "  LiDAR: {} 帧, IMU: {} 帧".format(
        counts[lidar_topic], counts[imu_topic])
    if image_topic:
        summary += ", Image: {} 帧".format(counts[image_topic])
    print(summary)


def validate_repaired_bag(output_path, lidar_topic, imu_topic,
                          max_deviation=0.02, image_topic=None):
    """检查非单调、帧间隔突变，以及 LiDAR header/timebase 一致性。"""
    stamps = {lidar_topic: [], imu_topic: []}
    records = {lidar_topic: [], imu_topic: []}
    if image_topic:
        stamps[image_topic] = []
        records[image_topic] = []
    timebase_errors = []
    lidar_timebases = []
    topics = [lidar_topic, imu_topic]
    if image_topic:
        topics.append(image_topic)

    print("\n[VALIDATE] 检查修复后时间戳: {}".format(output_path))
    with rosbag.Bag(output_path) as bag:
        for topic, msg, record_time in bag.read_messages(
                topics=topics):
            stamps[topic].append(msg.header.stamp.to_sec())
            records[topic].append(record_time.to_sec())
            if topic == lidar_topic and hasattr(msg, "timebase"):
                header_ns = (int(msg.header.stamp.secs) * NS_PER_SEC +
                             int(msg.header.stamp.nsecs))
                timebase_errors.append(header_ns - int(msg.timebase))
                lidar_timebases.append(int(msg.timebase))
            elif topic == lidar_topic:
                lidar_timebases.append(None)

    total_anomalies = 0
    named_topics = [("LiDAR", lidar_topic), ("IMU", imu_topic)]
    if image_topic:
        named_topics.append(("Image", image_topic))
    for name, topic in named_topics:
        values = np.asarray(stamps[topic])
        if len(values) < 2:
            print("  {}: 帧数不足，跳过".format(name))
            continue
        dt = np.diff(values)
        positive_dt = dt[dt > 0.0]
        median_dt = (float(np.median(positive_dt))
                     if len(positive_dt) else 0.0)
        bad = np.where(
            (dt <= 0.0) | (np.abs(dt - median_dt) > max_deviation))[0]
        total_anomalies += len(bad)
        print("  {}: {} 帧, dt中位数={:.3f}ms, 异常={}处"
              .format(name, len(values), median_dt * 1000.0, len(bad)))
        for i in bad[:5]:
            print("    #{}: {:.9f} -> {:.9f}, dt={:.3f}ms"
                  .format(i, values[i], values[i + 1], dt[i] * 1000.0))
            if topic == lidar_topic:
                record_dt = records[topic][i + 1] - records[topic][i]
                print("      出处: topic帧 #{} -> #{}, bag record "
                      "{:.9f} -> {:.9f}, dt={:.3f}ms"
                      .format(i, i + 1, records[topic][i],
                              records[topic][i + 1], record_dt * 1000.0))
                print("      header-record偏差: {:+.3f}ms -> {:+.3f}ms"
                      .format((values[i] - records[topic][i]) * 1000.0,
                              (values[i + 1] - records[topic][i + 1]) *
                              1000.0))
                if (lidar_timebases[i] is not None and
                        lidar_timebases[i + 1] is not None):
                    timebase_dt = (lidar_timebases[i + 1] -
                                   lidar_timebases[i]) / float(NS_PER_SEC)
                    print("      timebase: {} -> {}, dt={:.3f}ms"
                          .format(lidar_timebases[i], lidar_timebases[i + 1],
                                  timebase_dt * 1000.0))

    if timebase_errors:
        spread = max(timebase_errors) - min(timebase_errors)
        print("  LiDAR header-timebase 差值变化范围: {}ns".format(spread))
        # 驱动在 header 与 timebase 的换算中可能本来就有亚微秒舍入；
        # 只有差值随帧变化超过 1 us 才视为不同步。
        if spread > 1000:
            total_anomalies += 1

    if total_anomalies == 0:
        print("  ✓ 时间戳连续，LiDAR header/timebase 同步")
    else:
        print("  ⚠ 共检测到 {} 处异常".format(total_anomalies))
    return total_anomalies


def repair_bag(input_path, output_path, lidar_topic, imu_topic, gnss_topic,
               gnss_local_time_diff, jump_threshold, image_topic=None):
    extracted = extract_all_timestamps(
        input_path, lidar_topic, imu_topic, gnss_topic, image_topic)
    lidar_data, imu_data, gnss_data = extracted[:3]
    image_data = extracted[3] if image_topic else []
    if not lidar_data:
        raise ValueError("未找到 LiDAR 数据: {}".format(lidar_topic))

    clock = GnssClockReference(gnss_data, gnss_local_time_diff)
    lidar_offsets = estimate_frame_offsets(
        lidar_data, clock, "LiDAR", jump_threshold)
    imu_offsets = estimate_frame_offsets(
        imu_data, clock, "IMU", jump_threshold)
    image_timestamps_ns = reconstruct_image_timestamps(image_data, clock)

    write_repaired_bag(
        input_path, output_path, lidar_topic, imu_topic,
        lidar_offsets, imu_offsets, image_topic, image_timestamps_ns)
    return validate_repaired_bag(
        output_path, lidar_topic, imu_topic,
        image_topic=image_topic)


def main():
    parser = argparse.ArgumentParser(
        description=("基于 GNSS 时钟参考和帧间连续性逐帧修复 "
                     "LiDAR/IMU/CompressedImage 时间戳"))
    parser.add_argument("input", help="输入 rosbag 路径")
    parser.add_argument("output", nargs="?", help="输出 rosbag 路径")
    parser.add_argument("--lidar-topic", default="/livox/lidar")
    parser.add_argument("--imu-topic", default="/livox/imu")
    parser.add_argument("--image-topic", default="/mvs/image/compressed",
                        help="sensor_msgs/CompressedImage 话题，默认 %(default)s")
    parser.add_argument("--gnss-topic", default="/ublox_driver/receiver_lla")
    parser.add_argument("--gnss-time-diff", type=float, default=18.0,
                        help="GNSS 与本地时间差（秒），默认 18.0")
    parser.add_argument("--jump-threshold", type=float, default=0.5,
                        help="连续性代价尺度上限（秒），默认 0.5")
    # 兼容旧命令行；新算法不再按固定时长切段。
    parser.add_argument("--segment-duration", type=float, default=None,
                        help=argparse.SUPPRESS)
    parser.add_argument("--validate-only", action="store_true",
                        help="仅验证 input bag，不执行修复")
    args = parser.parse_args()

    if args.validate_only:
        validate_repaired_bag(
            args.input, args.lidar_topic, args.imu_topic,
            image_topic=args.image_topic)
        return

    output = args.output
    if output is None:
        root, extension = os.path.splitext(args.input)
        output = root + "_fixed" + (extension or ".bag")
    if os.path.abspath(output) == os.path.abspath(args.input):
        parser.error("输出路径不能与输入路径相同")

    try:
        repair_bag(
            args.input, output, args.lidar_topic, args.imu_topic,
            args.gnss_topic, args.gnss_time_diff, args.jump_threshold,
            args.image_topic)
    except (ValueError, rosbag.ROSBagException) as error:
        print("[ERROR] {}".format(error), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
