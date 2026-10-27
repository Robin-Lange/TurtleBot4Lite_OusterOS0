#!/usr/bin/env python3
"""Compact Encoder/GRU Proxy Benchmark for Stationary Sensor Feasibility.

Architecture:
- Input: 128 x 128 x 4 (RGB-D)
- Conv1: 4 -> 16, kernel 3x3, stride 2, padding 1 -> (64, 64, 16)
- Conv2: 16 -> 32, kernel 3x3, stride 2, padding 1 -> (32, 32, 32)
- Conv3: 32 -> 64, kernel 3x3, stride 2, padding 1 -> (16, 16, 64)
- Global Average Pooling -> 64
- GRU: 64 input, 64 hidden
- Output Heads:
  - Planar State: Linear(64 -> 4)
  - Lower-triangular Covariance Factor: Linear(64 -> 10)

Implemented in pure NumPy with explicit weights to benchmark execution
independently of PyTorch on Jetson Orin Nano CPU cores.
"""

import argparse
import json
import os
import resource
import time
import numpy as np


class NumPyConv2d:
    def __init__(self, in_channels, out_channels, kernel_size=3, stride=2, padding=1, dtype=np.float32):
        self.in_channels = in_channels
        self.out_channels = out_channels
        self.kernel_size = kernel_size
        self.stride = stride
        self.padding = padding
        self.dtype = dtype

        # Kaiming normal init
        scale = np.sqrt(2.0 / (in_channels * kernel_size * kernel_size))
        self.weight = (np.random.randn(out_channels, in_channels, kernel_size, kernel_size) * scale).astype(dtype)
        self.bias = np.zeros((out_channels,), dtype=dtype)

        self.num_params = self.weight.size + self.bias.size

    def __call__(self, x):
        # x shape: (in_channels, H, W)
        C, H, W = x.shape
        pad = self.padding
        if pad > 0:
            x_padded = np.pad(x, ((0, 0), (pad, pad), (pad, pad)), mode='constant')
        else:
            x_padded = x

        out_h = (H + 2 * pad - self.kernel_size) // self.stride + 1
        out_w = (W + 2 * pad - self.kernel_size) // self.stride + 1

        # Vectorized im2col implementation for performance
        K = self.kernel_size
        S = self.stride
        shape = (C, K, K, out_h, out_w)
        strides = (
            x_padded.strides[0],
            x_padded.strides[1],
            x_padded.strides[2],
            x_padded.strides[1] * S,
            x_padded.strides[2] * S
        )
        cols = np.lib.stride_tricks.as_strided(x_padded, shape=shape, strides=strides)
        cols = cols.reshape(C * K * K, out_h * out_w)

        w_row = self.weight.reshape(self.out_channels, C * K * K)
        out = np.dot(w_row, cols) + self.bias[:, None]
        out = out.reshape(self.out_channels, out_h, out_w)
        # ReLU activation
        return np.maximum(out, 0.0, out)


class NumPyGRUCell:
    def __init__(self, input_size=64, hidden_size=64, dtype=np.float32):
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.dtype = dtype

        scale_i = np.sqrt(1.0 / input_size)
        scale_h = np.sqrt(1.0 / hidden_size)

        # Gates: r (reset), z (update), n (candidate)
        self.W_ih = (np.random.randn(3 * hidden_size, input_size) * scale_i).astype(dtype)
        self.W_hh = (np.random.randn(3 * hidden_size, hidden_size) * scale_h).astype(dtype)
        self.b_ih = np.zeros((3 * hidden_size,), dtype=dtype)
        self.b_hh = np.zeros((3 * hidden_size,), dtype=dtype)

        self.num_params = self.W_ih.size + self.W_hh.size + self.b_ih.size + self.b_hh.size

    def __call__(self, x, h):
        # x: (input_size,), h: (hidden_size,)
        gi = np.dot(self.W_ih, x) + self.b_ih
        gh = np.dot(self.W_hh, h) + self.b_hh

        H = self.hidden_size
        i_r, i_z, i_n = gi[:H], gi[H:2*H], gi[2*H:]
        h_r, h_z, h_n = gh[:H], gh[H:2*H], gh[2*H:]

        # Sigmoid activations
        reset_gate = 1.0 / (1.0 + np.exp(-(i_r + h_r)))
        update_gate = 1.0 / (1.0 + np.exp(-(i_z + h_z)))
        new_gate = np.tanh(i_n + reset_gate * h_n)

        h_next = (1.0 - update_gate) * new_gate + update_gate * h
        return h_next


class CompactTrackerProxy:
    def __init__(self, dtype=np.float32):
        self.dtype = dtype
        self.conv1 = NumPyConv2d(4, 16, kernel_size=3, stride=2, padding=1, dtype=dtype)
        self.conv2 = NumPyConv2d(16, 32, kernel_size=3, stride=2, padding=1, dtype=dtype)
        self.conv3 = NumPyConv2d(32, 64, kernel_size=3, stride=2, padding=1, dtype=dtype)
        self.gru = NumPyGRUCell(64, 64, dtype=dtype)

        # Output heads
        self.head_state_w = np.random.randn(4, 64).astype(dtype) * 0.1
        self.head_state_b = np.zeros((4,), dtype=dtype)

        self.head_cov_w = np.random.randn(10, 64).astype(dtype) * 0.1
        self.head_cov_b = np.zeros((10,), dtype=dtype)

        self.num_params = (
            self.conv1.num_params
            + self.conv2.num_params
            + self.conv3.num_params
            + self.gru.num_params
            + self.head_state_w.size + self.head_state_b.size
            + self.head_cov_w.size + self.head_cov_b.size
        )

        self.h = np.zeros((64,), dtype=dtype)

    def reset_state(self):
        self.h.fill(0.0)

    def forward(self, rgbd):
        # rgbd: (4, 128, 128)
        c1 = self.conv1(rgbd)       # (16, 64, 64)
        c2 = self.conv2(c1)         # (32, 32, 32)
        c3 = self.conv3(c2)         # (64, 16, 16)
        # Global Average Pooling
        gap = np.mean(c3, axis=(1, 2))  # (64,)
        # GRU step
        self.h = self.gru(gap, self.h)
        # Output heads
        state = np.dot(self.head_state_w, self.h) + self.head_state_b      # (4,)
        cov_factor = np.dot(self.head_cov_w, self.h) + self.head_cov_b    # (10,)
        return state, cov_factor


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--warmup-iters", type=int, default=50, help="Number of warmup iterations")
    parser.add_argument("--benchmark-iters", type=int, default=500, help="Number of benchmark iterations")
    parser.add_argument("--paced-hz", type=float, default=10.0, help="Target paced execution rate in Hz")
    parser.add_argument("--output-json", required=True, help="Path to write JSON benchmark summary")
    args = parser.parse_args()

    np.random.seed(42)
    model = CompactTrackerProxy(dtype=np.float32)

    param_breakdown = {
        "conv1_4_to_16": model.conv1.num_params,
        "conv2_16_to_32": model.conv2.num_params,
        "conv3_32_to_64": model.conv3.num_params,
        "gru_64_to_64": model.gru.num_params,
        "head_state_4": model.head_state_w.size + model.head_state_b.size,
        "head_cov_factor_10": model.head_cov_w.size + model.head_cov_b.size,
        "total_parameters": model.num_params,
    }

    print(f"Proxy Model Initialized. Total Parameters: {model.num_params}")
    for k, v in param_breakdown.items():
        print(f"  - {k}: {v:,}")

    # Generate synthetic RGB-D frames (128x128x4)
    # 4 channels: R, G, B, Depth in meters
    input_shape = (4, 128, 128)
    dummy_input = np.random.uniform(0.0, 1.0, size=input_shape).astype(np.float32)

    # 1. Warm-up
    print(f"Running {args.warmup_iters} warm-up iterations...")
    for _ in range(args.warmup_iters):
        model.forward(dummy_input)

    # 2. Maximum throughput benchmark (unpaced)
    print(f"Running {args.benchmark_iters} maximum throughput iterations...")
    model.reset_state()
    latencies_max_ms = []
    t_start_total = time.monotonic()
    for _ in range(args.benchmark_iters):
        t0 = time.monotonic()
        state, cov = model.forward(dummy_input)
        t1 = time.monotonic()
        latencies_max_ms.append((t1 - t0) * 1000.0)
    t_end_total = time.monotonic()
    total_elapsed = t_end_total - t_start_total
    achieved_max_hz = args.benchmark_iters / total_elapsed

    arr_lat = np.array(latencies_max_ms)
    max_throughput_stats = {
        "iterations": args.benchmark_iters,
        "total_elapsed_s": round(total_elapsed, 4),
        "achieved_throughput_hz": round(achieved_max_hz, 2),
        "p50_latency_ms": round(float(np.percentile(arr_lat, 50)), 3),
        "p95_latency_ms": round(float(np.percentile(arr_lat, 95)), 3),
        "p99_latency_ms": round(float(np.percentile(arr_lat, 99)), 3),
        "min_latency_ms": round(float(np.min(arr_lat)), 3),
        "max_latency_ms": round(float(np.max(arr_lat)), 3),
        "mean_latency_ms": round(float(np.mean(arr_lat)), 3),
        "std_latency_ms": round(float(np.std(arr_lat)), 3),
    }

    # 3. Paced benchmark (target 10 Hz)
    print(f"Running paced benchmark at {args.paced_hz} Hz for 100 cycles...")
    target_period = 1.0 / args.paced_hz
    latencies_paced_ms = []
    cycle_intervals_ms = []
    model.reset_state()

    t_next = time.monotonic()
    t_prev_cycle = None
    for _ in range(100):
        t0 = time.monotonic()
        if t_prev_cycle is not None:
            cycle_intervals_ms.append((t0 - t_prev_cycle) * 1000.0)
        t_prev_cycle = t0

        state, cov = model.forward(dummy_input)
        t1 = time.monotonic()
        latencies_paced_ms.append((t1 - t0) * 1000.0)

        t_next += target_period
        sleep_dur = t_next - time.monotonic()
        if sleep_dur > 0:
            time.sleep(sleep_dur)
        else:
            t_next = time.monotonic()

    arr_paced_lat = np.array(latencies_paced_ms)
    arr_cycles = np.array(cycle_intervals_ms)
    paced_stats = {
        "target_hz": args.paced_hz,
        "target_period_ms": round(target_period * 1000.0, 2),
        "p50_model_latency_ms": round(float(np.percentile(arr_paced_lat, 50)), 3),
        "p95_model_latency_ms": round(float(np.percentile(arr_paced_lat, 95)), 3),
        "p99_model_latency_ms": round(float(np.percentile(arr_paced_lat, 99)), 3),
        "p50_cycle_interval_ms": round(float(np.percentile(arr_cycles, 50)), 3) if len(arr_cycles) else None,
        "p95_cycle_interval_ms": round(float(np.percentile(arr_cycles, 95)), 3) if len(arr_cycles) else None,
    }

    # Memory utilization
    usage = resource.getrusage(resource.RUSAGE_SELF)
    max_rss_mb = usage.ru_maxrss / 1024.0  # Linux ru_maxrss is in KiB

    result_data = {
        "benchmark_timestamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "input_specification": {
            "spatial_dimensions": [128, 128],
            "channels": 4,
            "channel_format": "RGB-D (Red, Green, Blue, Depth)",
            "batch_size": 1,
            "data_type": "FP32 (NumPy reference)",
            "synthetic_input": True,
            "note": "Untrained representative proxy architecture; does not validate full tracker accuracy"
        },
        "parameter_breakdown": param_breakdown,
        "max_throughput_benchmark": max_throughput_stats,
        "paced_10hz_benchmark": paced_stats,
        "process_memory": {
            "peak_rss_mb": round(max_rss_mb, 2),
            "budget_mb": 4096.0,
            "budget_utilization_pct": round((max_rss_mb / 4096.0) * 100.0, 3)
        },
        "hardware_context": {
            "device": "NVIDIA Jetson Orin Nano Developer Kit Super",
            "cpu_architecture": "6-core ARM Cortex-A78AE",
            "power_mode": "MAXN_SUPER (ID 2)",
            "gpu_acceleration_status": "Blocked: PyTorch (torch module) and CUDA SDK not installed on host Python"
        }
    }

    os.makedirs(os.path.dirname(os.path.abspath(args.output_json)), exist_ok=True)
    with open(args.output_json, "w") as f:
        json.dump(result_data, f, indent=2)

    print(f"Results saved to {args.output_json}")


if __name__ == "__main__":
    main()
