#!/usr/bin/env python3
"""Stationary sensor acquisition monitor for RealSense D455 and RPLIDAR S2.

Subscribes strictly in read-only mode to sensor streams.
Collects message counts, reception rates, inter-arrival intervals,
header stamp intervals, stamp anomalies, and concurrent tegrastats telemetry.
"""

import argparse
import csv
import json
import math
import os
import re
import subprocess
import threading
import time
from collections import defaultdict
from datetime import datetime, timezone

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CameraInfo, Image, LaserScan


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def calc_stats(values):
    if not values:
        return None
    arr = np.array(values, dtype=np.float64)
    return {
        "count": int(len(arr)),
        "min": float(np.min(arr)),
        "p25": float(np.percentile(arr, 25)),
        "p50": float(np.percentile(arr, 50)),
        "p75": float(np.percentile(arr, 75)),
        "p95": float(np.percentile(arr, 95)),
        "p99": float(np.percentile(arr, 99)),
        "max": float(np.max(arr)),
        "mean": float(np.mean(arr)),
        "std": float(np.std(arr)),
    }


def scale_stats_to_ms(stats):
    if not stats:
        return None
    res = {}
    for k, v in stats.items():
        if k == "count":
            res[k] = int(v)
        else:
            res[k] = round(v * 1000.0, 3)
    return res


class TegrastatsCollector(threading.Thread):
    def __init__(self, interval_ms=1000):
        super().__init__(daemon=True)
        self.interval_ms = interval_ms
        self.samples = []
        self._stop_event = threading.Event()
        self.process = None

    def run(self):
        cmd = ["tegrastats", "--interval", str(self.interval_ms)]
        try:
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                universal_newlines=True,
                bufsize=1,
            )
            for line in self.process.stdout:
                if self._stop_event.is_set():
                    break
                line = line.strip()
                if not line:
                    continue
                parsed = self._parse_line(line)
                if parsed:
                    self.samples.append(parsed)
        except Exception as e:
            print(f"[TegrastatsCollector] Exception: {e}")

    def _parse_line(self, line):
        record = {
            "wall_time": time.time(),
            "monotonic_time": time.monotonic(),
            "raw": line,
        }
        # RAM x/yMB
        ram_m = re.search(r"RAM\s+(\d+)/(\d+)MB", line)
        if ram_m:
            record["ram_used_mb"] = int(ram_m.group(1))
            record["ram_total_mb"] = int(ram_m.group(2))

        # SWAP x/yMB
        swap_m = re.search(r"SWAP\s+(\d+)/(\d+)MB", line)
        if swap_m:
            record["swap_used_mb"] = int(swap_m.group(1))
            record["swap_total_mb"] = int(swap_m.group(2))

        # Power rails (mW)
        vdd_in = re.search(r"VDD_IN\s+(\d+)mW/(\d+)mW", line)
        if vdd_in:
            record["vdd_in_curr_mw"] = int(vdd_in.group(1))
            record["vdd_in_avg_mw"] = int(vdd_in.group(2))

        vdd_cpu_gpu = re.search(r"VDD_CPU_GPU_CV\s+(\d+)mW/(\d+)mW", line)
        if vdd_cpu_gpu:
            record["vdd_cpu_gpu_curr_mw"] = int(vdd_cpu_gpu.group(1))
            record["vdd_cpu_gpu_avg_mw"] = int(vdd_cpu_gpu.group(2))

        vdd_soc = re.search(r"VDD_SOC\s+(\d+)mW/(\d+)mW", line)
        if vdd_soc:
            record["vdd_soc_curr_mw"] = int(vdd_soc.group(1))
            record["vdd_soc_avg_mw"] = int(vdd_soc.group(2))

        # Temperatures
        for zone in ["cpu", "gpu", "soc0", "soc1", "soc2", "tj"]:
            t_m = re.search(rf"{zone}@([\d\.]+)C", line)
            if t_m:
                record[f"temp_{zone}_c"] = float(t_m.group(1))

        # CPU utilizations and clocks: CPU [10%@1036,2%@1036,8%@1036,3%@1036,0%@729,2%@729]
        cpu_m = re.search(r"CPU\s+\[(.*?)\]", line)
        if cpu_m:
            core_strs = cpu_m.group(1).split(",")
            core_utils = []
            core_freqs = []
            for cs in core_strs:
                m_core = re.search(r"(\d+)%@(\d+)", cs.strip())
                if m_core:
                    core_utils.append(int(m_core.group(1)))
                    core_freqs.append(int(m_core.group(2)))
            if core_utils:
                record["cpu_util_avg_pct"] = float(np.mean(core_utils))
                record["cpu_util_max_pct"] = int(np.max(core_utils))
            if core_freqs:
                record["cpu_freq_avg_mhz"] = float(np.mean(core_freqs))
                record["cpu_freq_max_mhz"] = int(np.max(core_freqs))

        # GR3D (GPU) %
        gr3d_m = re.search(r"GR3D_FREQ\s+(\d+)%", line)
        if gr3d_m:
            record["gpu_util_pct"] = int(gr3d_m.group(1))

        return record

    def stop(self):
        self._stop_event.set()
        if self.process and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.process.kill()


class SensorAcquisitionMonitor(Node):
    def __init__(self, target_topics):
        super().__init__("sensor_acquisition_monitor")
        self.target_topics = target_topics
        self.subscriptions_list = []
        # Per topic record list of dicts: {mono_time, wall_time, header_stamp, frame_id}
        self.data = defaultdict(list)
        self.sub_qos = {}

        type_map = {
            "/camera/camera/color/image_raw": Image,
            "/camera/camera/aligned_depth_to_color/image_raw": Image,
            "/camera/camera/depth/image_rect_raw": Image,
            "/camera/camera/color/camera_info": CameraInfo,
            "/scan": LaserScan,
        }

        # Explicit compatible QoS: Reliable, Volatile, Depth 10
        # Matches both RPLIDAR (Reliable, Volatile) and RealSense (Reliable, Transient Local)
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        for topic in target_topics:
            msg_type = type_map.get(topic)
            if not msg_type:
                # Default fallback
                if "scan" in topic:
                    msg_type = LaserScan
                else:
                    msg_type = Image

            self.subscriptions_list.append(
                self.create_subscription(
                    msg_type,
                    topic,
                    lambda msg, t=topic: self.on_message(t, msg),
                    qos,
                )
            )

    def on_message(self, topic, message):
        t_mono = time.monotonic()
        t_wall = time.time()
        stamp_sec = 0.0
        frame_id = ""
        if hasattr(message, "header"):
            stamp_sec = stamp_to_sec(message.header.stamp)
            frame_id = message.header.frame_id

        self.data[topic].append({
            "monotonic_time": t_mono,
            "wall_time": t_wall,
            "header_stamp": stamp_sec,
            "frame_id": frame_id,
        })

    def analyze(self, nominal_rates=None):
        if nominal_rates is None:
            nominal_rates = {
                "/camera/camera/color/image_raw": 30.0,
                "/camera/camera/aligned_depth_to_color/image_raw": 30.0,
                "/camera/camera/depth/image_rect_raw": 30.0,
                "/scan": 10.0,
            }

        results = {}
        for topic, msgs in self.data.items():
            if not msgs:
                results[topic] = {"message_count": 0}
                continue

            mono_times = [m["monotonic_time"] for m in msgs]
            wall_times = [m["wall_time"] for m in msgs]
            stamps = [m["header_stamp"] for m in msgs]
            frames = sorted(list(set(m["frame_id"] for m in msgs)))

            count = len(msgs)
            total_duration_s = mono_times[-1] - mono_times[0] if count > 1 else 0.0
            recv_rate_hz = (count - 1) / total_duration_s if total_duration_s > 0 else 0.0

            # Inter-arrival intervals (monotonic clock reception deltas)
            inter_arrivals = np.diff(mono_times).tolist() if count > 1 else []
            inter_arrival_stats = calc_stats(inter_arrivals)

            # Header stamp deltas
            valid_stamps = [s for s in stamps if s > 0.0]
            stamp_deltas = np.diff(stamps).tolist() if count > 1 else []
            stamp_delta_stats = calc_stats(stamp_deltas)

            # Anomaly checks on timestamps
            zero_count = sum(1 for s in stamps if s == 0.0)
            repeated_count = sum(1 for i in range(1, count) if stamps[i] == stamps[i - 1])
            nonmonotonic_count = sum(1 for i in range(1, count) if stamps[i] < stamps[i - 1])

            # Future timestamps: check if header_stamp > wall_time + 0.1s
            # Note: only valid if header stamp is UTC clock domain, not hardware clock domain
            clock_domain_note = "Valid"
            future_count = 0
            stamp_ages = []
            if valid_stamps:
                # If stamps are near current epoch (> 1.7e9), device uses system time
                # If stamps are small (< 1e8), device uses hardware timestamp domain (e.g. RealSense HW clock)
                median_stamp = float(np.median(valid_stamps))
                if median_stamp < 1.0e8:
                    clock_domain_note = "Hardware timestamp domain (starts at 0 / boot); direct age relative to host wall time is invalid"
                else:
                    stamp_ages = [wall_times[i] - stamps[i] for i in range(count)]
                    future_count = sum(1 for age in stamp_ages if age < -0.05)

            stamp_age_stats = calc_stats(stamp_ages) if stamp_ages else None

            # Likely missing messages based on nominal rate
            nom_rate = nominal_rates.get(topic, 0.0)
            missing_count = 0
            if nom_rate > 0.0 and count > 1:
                nom_period = 1.0 / nom_rate
                # A jump > 1.8 * nominal_period indicates at least one missed cycle
                for delta in stamp_deltas:
                    if delta > 1.8 * nom_period:
                        missing_count += int(round(delta / nom_period)) - 1

            results[topic] = {
                "message_count": count,
                "duration_seconds": round(total_duration_s, 4),
                "received_rate_hz": round(recv_rate_hz, 3),
                "nominal_rate_hz": nom_rate,
                "frames": frames,
                "inter_arrival_ms": scale_stats_to_ms(inter_arrival_stats),
                "stamp_interval_ms": scale_stats_to_ms(stamp_delta_stats),
                "stamp_anomalies": {
                    "zero_timestamps": zero_count,
                    "repeated_timestamps": repeated_count,
                    "nonmonotonic_timestamps": nonmonotonic_count,
                    "future_timestamps": future_count,
                    "clock_domain_note": clock_domain_note,
                },
                "header_stamp_age_ms": scale_stats_to_ms(stamp_age_stats),
                "estimated_missing_messages": missing_count,
            }
        return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--warmup-seconds", type=float, default=30.0, help="Warmup duration before recording statistics")
    parser.add_argument("--run-seconds", type=float, default=120.0, help="Active measurement duration in seconds")
    parser.add_argument("--topics", default="/camera/camera/color/image_raw,/camera/camera/aligned_depth_to_color/image_raw,/scan", help="Comma-separated topics to subscribe")
    parser.add_argument("--output-json", required=True, help="Destination JSON file for summary")
    parser.add_argument("--output-raw-csv", default=None, help="Optional CSV file for individual message arrival timestamps")
    parser.add_argument("--output-tegrastats-csv", default=None, help="Optional CSV file for tegrastats time series")
    parser.add_argument("--condition-name", default="condition", help="Name of condition (e.g. D455_alone, RPLIDAR_alone, Both_concurrent)")

    args = parser.parse_args(remove_ros_args()[1:])
    target_topics = [t.strip() for t in args.topics.split(",") if t.strip()]

    print(f"[{args.condition_name}] Initializing acquisition monitor...")
    print(f"[{args.condition_name}] Target topics: {target_topics}")
    print(f"[{args.condition_name}] Warm-up: {args.warmup_seconds}s | Measurement: {args.run_seconds}s")

    # Start tegrastats collector
    tegra_collector = TegrastatsCollector(interval_ms=1000)
    tegra_collector.start()

    rclpy.init()
    node = SensorAcquisitionMonitor(target_topics)

    # Warm-up phase
    if args.warmup_seconds > 0:
        print(f"[{args.condition_name}] Warming up for {args.warmup_seconds} seconds...")
        warmup_end = time.monotonic() + args.warmup_seconds
        while rclpy.ok() and time.monotonic() < warmup_end:
            rclpy.spin_once(node, timeout_sec=0.1)
        # Clear data accumulated during warmup
        node.data.clear()
        print(f"[{args.condition_name}] Warmup complete. Reset buffers. Starting {args.run_seconds}s measurement run...")

    # Active measurement phase
    start_monotonic = time.monotonic()
    start_utc = datetime.now(timezone.utc).isoformat()
    measurement_end = start_monotonic + args.run_seconds

    last_print = start_monotonic
    while rclpy.ok() and time.monotonic() < measurement_end:
        rem = measurement_end - time.monotonic()
        rclpy.spin_once(node, timeout_sec=min(0.2, max(0.01, rem)))
        now = time.monotonic()
        if now - last_print >= 10.0:
            elapsed = now - start_monotonic
            counts = {t: len(node.data[t]) for t in target_topics}
            print(f"[{args.condition_name}] Progress: {elapsed:.1f}/{args.run_seconds:.1f}s | Counts: {counts}")
            last_print = now

    end_utc = datetime.now(timezone.utc).isoformat()
    end_monotonic = time.monotonic()

    # Stop tegrastats
    tegra_collector.stop()

    print(f"[{args.condition_name}] Run finished. Analyzing data...")
    topic_results = node.analyze()

    node.destroy_node()
    rclpy.shutdown()

    # Aggregate tegrastats
    tegra_samples = tegra_collector.samples
    tegra_stats = {}
    if tegra_samples:
        for key in ["ram_used_mb", "swap_used_mb", "vdd_in_curr_mw", "vdd_cpu_gpu_curr_mw", "vdd_soc_curr_mw",
                    "temp_cpu_c", "temp_gpu_c", "temp_soc0_c", "temp_tj_c", "gpu_util_pct",
                    "cpu_util_avg_pct", "cpu_util_max_pct", "cpu_freq_avg_mhz", "cpu_freq_max_mhz"]:
            vals = [s[key] for s in tegra_samples if key in s]
            if vals:
                tegra_stats[key] = {
                    "min": float(np.min(vals)),
                    "p50": float(np.median(vals)),
                    "p95": float(np.percentile(vals, 95)),
                    "max": float(np.max(vals)),
                    "mean": float(np.mean(vals)),
                }

    output_data = {
        "condition": args.condition_name,
        "start_utc": start_utc,
        "end_utc": end_utc,
        "target_duration_s": args.run_seconds,
        "actual_duration_s": round(end_monotonic - start_monotonic, 4),
        "topics": topic_results,
        "tegrastats_summary": tegra_stats,
    }

    os.makedirs(os.path.dirname(os.path.abspath(args.output_json)), exist_ok=True)
    with open(args.output_json, "w") as f:
        json.dump(output_data, f, indent=2)
    print(f"[{args.condition_name}] Saved summary JSON to {args.output_json}")

    # Save raw message CSV if requested
    if args.output_raw_csv:
        os.makedirs(os.path.dirname(os.path.abspath(args.output_raw_csv)), exist_ok=True)
        with open(args.output_raw_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["topic", "msg_index", "monotonic_time", "wall_time", "header_stamp", "frame_id"])
            for topic, msgs in node.data.items():
                for idx, m in enumerate(msgs):
                    writer.writerow([topic, idx, f"{m['monotonic_time']:.6f}", f"{m['wall_time']:.6f}", f"{m['header_stamp']:.9f}", m["frame_id"]])
        print(f"[{args.condition_name}] Saved raw CSV to {args.output_raw_csv}")

    # Save tegrastats CSV if requested
    if args.output_tegrastats_csv and tegra_samples:
        os.makedirs(os.path.dirname(os.path.abspath(args.output_tegrastats_csv)), exist_ok=True)
        with open(args.output_tegrastats_csv, "w", newline="") as f:
            fieldnames = sorted(list(set().union(*(s.keys() for s in tegra_samples))))
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for s in tegra_samples:
                writer.writerow(s)
        print(f"[{args.condition_name}] Saved tegrastats CSV to {args.output_tegrastats_csv}")


if __name__ == "__main__":
    main()
