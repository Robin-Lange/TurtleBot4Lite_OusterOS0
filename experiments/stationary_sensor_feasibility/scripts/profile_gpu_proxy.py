#!/usr/bin/env python3
"""PyTorch GPU Inference Proxy Benchmark for Jetson Orin Nano Super.

Benchmarking compact untrained multimodal tracking proxy:
- 128x128x4 RGB-D input
- 3 stride-2 convs (4->16->32->64) + ReLU
- Global average pooling -> 64
- GRU(64, 64)
- 4-dim planar state head
- 10-dim lower-triangular covariance factor head with softplus + eps positive diagonal
"""

import argparse
import json
import os
import time
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F


class TrackerBackboneGRU(nn.Module):
    def __init__(self, eps=1e-4):
        super().__init__()
        self.eps = eps
        self.conv1 = nn.Conv2d(4, 16, kernel_size=3, stride=2, padding=1)
        self.conv2 = nn.Conv2d(16, 32, kernel_size=3, stride=2, padding=1)
        self.conv3 = nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1)
        self.relu = nn.ReLU(inplace=True)
        self.gap = nn.AdaptiveAvgPool2d((1, 1))

        self.gru = nn.GRU(input_size=64, hidden_size=64, batch_first=True)

        self.head_state = nn.Linear(64, 4)
        self.head_cov = nn.Linear(64, 10)

        # Indices of diagonal elements in 10-element lower triangular representation:
        # Row 0: L[0,0] -> idx 0
        # Row 1: L[1,0], L[1,1] -> idx 1, 2
        # Row 2: L[2,0], L[2,1], L[2,2] -> idx 3, 4, 5
        # Row 3: L[3,0], L[3,1], L[3,2], L[3,3] -> idx 6, 7, 8, 9
        self.diag_indices = [0, 2, 5, 9]

    def forward(self, x, h):
        # x: (B, 4, 128, 128), h: (1, B, 64)
        c1 = self.relu(self.conv1(x))
        c2 = self.relu(self.conv2(c1))
        c3 = self.relu(self.conv3(c2))
        pooled = self.gap(c3).view(x.size(0), 1, 64)  # (B, 1, 64)

        out, h_next = self.gru(pooled, h)
        feat = out.squeeze(1)  # (B, 64)

        state = self.head_state(feat)
        cov_raw = self.head_cov(feat)

        # Enforce positive diagonal
        cov_factor = cov_raw.clone()
        for idx in self.diag_indices:
            cov_factor[:, idx] = F.softplus(cov_raw[:, idx]) + self.eps

        return state, cov_factor, h_next


def benchmark_mode(model, device, dtype, batch_size=1, warmup=50, iters=1000, paced_hz=10.0):
    model.eval()
    model.to(device=device, dtype=dtype)

    # 1. Warm-up
    dummy_input = torch.randn(batch_size, 4, 128, 128, device=device, dtype=dtype)
    h = torch.zeros(1, batch_size, 64, device=device, dtype=dtype)

    with torch.no_grad():
        for _ in range(warmup):
            _, _, h = model(dummy_input, h)
        if device.type == "cuda":
            torch.cuda.synchronize()

    # 2. Maximum throughput benchmark
    latencies_model_ms = []
    latencies_transfer_ms = []
    h = torch.zeros(1, batch_size, 64, device=device, dtype=dtype)

    # Pre-generate host data to measure transfer time accurately
    host_inputs = [torch.randn(batch_size, 4, 128, 128, dtype=dtype) for _ in range(iters)]

    if device.type == "cuda":
        torch.cuda.synchronize()

    t_start_total = time.monotonic()
    with torch.no_grad():
        for i in range(iters):
            # Measure host to device transfer
            t0 = time.monotonic()
            dev_x = host_inputs[i].to(device=device, non_blocking=False)
            if device.type == "cuda":
                torch.cuda.synchronize()
            t1 = time.monotonic()
            latencies_transfer_ms.append((t1 - t0) * 1000.0)

            # Measure model forward pass
            t2 = time.monotonic()
            state, cov, h = model(dev_x, h)
            if device.type == "cuda":
                torch.cuda.synchronize()
            t3 = time.monotonic()
            latencies_model_ms.append((t3 - t2) * 1000.0)

    t_end_total = time.monotonic()
    total_elapsed_s = t_end_total - t_start_total
    throughput_hz = iters / total_elapsed_s

    arr_model = np.array(latencies_model_ms)
    arr_xfer = np.array(latencies_transfer_ms)
    arr_total = arr_model + arr_xfer

    max_stats = {
        "iterations": iters,
        "total_elapsed_s": round(total_elapsed_s, 4),
        "achieved_throughput_hz": round(throughput_hz, 2),
        "model_only_latency_ms": {
            "p50": round(float(np.percentile(arr_model, 50)), 3),
            "p95": round(float(np.percentile(arr_model, 95)), 3),
            "p99": round(float(np.percentile(arr_model, 99)), 3),
            "mean": round(float(np.mean(arr_model)), 3),
            "min": round(float(np.min(arr_model)), 3),
            "max": round(float(np.max(arr_model)), 3),
        },
        "host_to_device_transfer_ms": {
            "p50": round(float(np.percentile(arr_xfer, 50)), 3),
            "p95": round(float(np.percentile(arr_xfer, 95)), 3),
            "p99": round(float(np.percentile(arr_xfer, 99)), 3),
            "mean": round(float(np.mean(arr_xfer)), 3),
        },
        "total_step_latency_ms": {
            "p50": round(float(np.percentile(arr_total, 50)), 3),
            "p95": round(float(np.percentile(arr_total, 95)), 3),
            "p99": round(float(np.percentile(arr_total, 99)), 3),
            "mean": round(float(np.mean(arr_total)), 3),
        }
    }

    # 3. Paced benchmark at target Hz (default 10 Hz)
    target_period = 1.0 / paced_hz
    paced_steps = min(200, iters)
    latencies_paced_ms = []
    cycle_intervals_ms = []
    h = torch.zeros(1, batch_size, 64, device=device, dtype=dtype)

    t_next = time.monotonic()
    t_prev_cycle = None
    with torch.no_grad():
        for i in range(paced_steps):
            t_cycle_start = time.monotonic()
            if t_prev_cycle is not None:
                cycle_intervals_ms.append((t_cycle_start - t_prev_cycle) * 1000.0)
            t_prev_cycle = t_cycle_start

            dev_x = host_inputs[i % len(host_inputs)].to(device=device)
            state, cov, h = model(dev_x, h)
            if device.type == "cuda":
                torch.cuda.synchronize()
            t_cycle_work_done = time.monotonic()
            latencies_paced_ms.append((t_cycle_work_done - t_cycle_start) * 1000.0)

            t_next += target_period
            sleep_sec = t_next - time.monotonic()
            if sleep_sec > 0:
                time.sleep(sleep_sec)
            else:
                t_next = time.monotonic()

    arr_paced = np.array(latencies_paced_ms)
    arr_intervals = np.array(cycle_intervals_ms)

    paced_stats = {
        "target_hz": paced_hz,
        "target_period_ms": round(target_period * 1000.0, 2),
        "steps": paced_steps,
        "step_latency_ms": {
            "p50": round(float(np.percentile(arr_paced, 50)), 3),
            "p95": round(float(np.percentile(arr_paced, 95)), 3),
            "p99": round(float(np.percentile(arr_paced, 99)), 3),
            "mean": round(float(np.mean(arr_paced)), 3),
        },
        "cycle_interval_ms": {
            "p50": round(float(np.percentile(arr_intervals, 50)), 3) if len(arr_intervals) else None,
            "p95": round(float(np.percentile(arr_intervals, 95)), 3) if len(arr_intervals) else None,
            "p99": round(float(np.percentile(arr_intervals, 99)), 3) if len(arr_intervals) else None,
        }
    }

    # GPU memory usage
    cuda_mem = {}
    if device.type == "cuda":
        cuda_mem = {
            "allocated_mb": round(torch.cuda.memory_allocated() / (1024 * 1024), 2),
            "max_allocated_mb": round(torch.cuda.max_memory_allocated() / (1024 * 1024), 2),
            "reserved_mb": round(torch.cuda.memory_reserved() / (1024 * 1024), 2),
            "max_reserved_mb": round(torch.cuda.max_memory_reserved() / (1024 * 1024), 2),
        }

    return {
        "dtype": str(dtype).split(".")[-1],
        "device": str(device),
        "max_throughput": max_stats,
        "paced_benchmark": paced_stats,
        "gpu_memory_telemetry": cuda_mem,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--iters", type=int, default=1000)
    parser.add_argument("--warmup", type=int, default=100)
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--power-mode-label", default="MAXN_SUPER")
    args = parser.parse_args()

    torch.manual_seed(42)
    device = torch.device(args.device)

    model = TrackerBackboneGRU()

    # Exact parameter count calculation
    param_counts = {name: param.numel() for name, param in model.named_parameters()}
    total_params = sum(param_counts.values())

    print(f"[{args.power_mode_label}] Initialized TrackerBackboneGRU on {device}")
    print(f"Total Parameters: {total_params:,}")
    for name, cnt in param_counts.items():
        print(f"  - {name}: {cnt:,}")

    results = {
        "timestamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "power_mode_label": args.power_mode_label,
        "total_parameters": total_params,
        "parameter_breakdown": param_counts,
        "input_shape": [1, 4, 128, 128],
        "benchmarks": {}
    }

    # 1. FP32 benchmark
    print(f"[{args.power_mode_label}] Running FP32 Benchmark ({args.iters} iters)...")
    results["benchmarks"]["fp32"] = benchmark_mode(
        model, device, torch.float32, batch_size=1, warmup=args.warmup, iters=args.iters
    )

    # 2. FP16 benchmark (if CUDA available)
    if device.type == "cuda":
        print(f"[{args.power_mode_label}] Running FP16 Benchmark ({args.iters} iters)...")
        results["benchmarks"]["fp16"] = benchmark_mode(
            model, device, torch.float16, batch_size=1, warmup=args.warmup, iters=args.iters
        )

    os.makedirs(os.path.dirname(os.path.abspath(args.output_json)), exist_ok=True)
    with open(args.output_json, "w") as f:
        json.dump(results, f, indent=2)

    print(f"Benchmark results saved to {args.output_json}")


if __name__ == "__main__":
    main()
