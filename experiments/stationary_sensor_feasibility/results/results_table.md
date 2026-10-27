# Stationary Sensor Feasibility & Compute Feasibility Results Table

## 1. Sustained Sensor Acquisition Across Trials

| Condition | Stream / Topic | Runs | Nominal Rate (Hz) | Measured Rate (Hz, mean ± std) | Inter-Arrival p50 / p95 / p99 (ms) | Header Stamp Interval p50 / p95 / p99 (ms) | Header Stamp Age p50 / p95 (ms) | Missing Msgs (total / run) | Module VDD_IN Power (mW, p50 / p95) | Module Temp CPU / GPU (°C, mean) |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| D455_alone | `/camera/camera/depth/image_rect_raw` | 3 | 30.0 | 29.98 ± 0.01 | 33.4 / 37.9 / 41.9 | 33.3 / 33.3 / 33.4 | 48.3 / 52.9 | 4 (1.3/run) | 6315 / 6977 | 54.3 / 53.1 |
| D455_alone | `/camera/camera/aligned_depth_to_color/image_raw` | 3 | 30.0 | 29.93 ± 0.04 | 33.4 / 36.4 / 41.5 | 33.3 / 33.3 / 33.4 | 42.0 / 44.4 | 22 (7.3/run) | 6315 / 6977 | 54.3 / 53.1 |
| D455_alone | `/camera/camera/color/image_raw` | 3 | 30.0 | 29.81 ± 0.12 | 33.4 / 38.3 / 46.8 | 33.3 / 33.3 / 33.4 | 45.9 / 50.3 | 65 (21.7/run) | 6315 / 6977 | 54.3 / 53.1 |
| RPLIDAR_alone | `/scan` | 3 | 10.0 | 10.01 ± 0.00 | 100.0 / 100.4 / 101.2 | 100.0 / 100.3 / 101.1 | 100.5 / 101.0 | 0 (0.0/run) | 5373 / 5376 | 52.5 / 51.7 |
| Both_concurrent | `/camera/camera/color/image_raw` | 3 | 30.0 | 29.89 ± 0.02 | 33.3 / 38.8 / 45.3 | 33.3 / 33.3 / 33.7 | 46.2 / 51.3 | 28 (9.3/run) | 6027 / 6054 | 53.4 / 52.4 |
| Both_concurrent | `/camera/camera/depth/image_rect_raw` | 3 | 30.0 | 29.97 ± 0.02 | 33.3 / 38.6 / 43.3 | 33.3 / 33.3 / 33.7 | 49.0 / 53.8 | 1 (0.3/run) | 6027 / 6054 | 53.4 / 52.4 |
| Both_concurrent | `/scan` | 3 | 10.0 | 10.01 ± 0.00 | 100.0 / 102.8 / 105.7 | 100.0 / 100.5 / 101.4 | 100.7 / 107.4 | 0 (0.0/run) | 6027 / 6054 | 53.4 / 52.4 |
| Both_concurrent | `/camera/camera/aligned_depth_to_color/image_raw` | 3 | 30.0 | 29.95 ± 0.02 | 33.3 / 36.9 / 42.8 | 33.3 / 33.3 / 33.7 | 42.1 / 45.3 | 9 (3.0/run) | 6027 / 6054 | 53.4 / 52.4 |
| Both_concurrent_15W | `/camera/camera/color/image_raw` | 1 | 30.0 | 29.89 ± 0.00 | 33.3 / 36.3 / 44.3 | 33.3 / 33.3 / 33.4 | 43.6 / 46.4 | 12 (12.0/run) | 6120 / 6439 | 53.6 / 52.6 |
| Both_concurrent_15W | `/camera/camera/depth/image_rect_raw` | 1 | 30.0 | 29.99 ± 0.00 | 33.3 / 37.2 / 42.6 | 33.3 / 33.3 / 33.4 | 46.6 / 49.9 | 0 (0.0/run) | 6120 / 6439 | 53.6 / 52.6 |
| Both_concurrent_15W | `/camera/camera/aligned_depth_to_color/image_raw` | 1 | 30.0 | 29.95 ± 0.00 | 33.4 / 35.3 / 41.9 | 33.3 / 33.3 / 33.4 | 40.1 / 41.8 | 5 (5.0/run) | 6120 / 6439 | 53.6 / 52.6 |
| Both_concurrent_15W | `/scan` | 1 | 10.0 | 10.01 ± 0.00 | 100.0 / 101.8 / 103.7 | 100.0 / 101.4 / 102.6 | 100.6 / 104.9 | 0 (0.0/run) | 6120 / 6439 | 53.6 / 52.6 |

## 2. Neural Inference Proxy Benchmark (Batch Size = 1, 128×128×4 RGB-D)

| Implementation | Execution Device | Power Mode | Data Type | Parameters | Max Throughput (Hz) | Step Latency p50 / p95 / p99 (ms) | Model Forward Only p50 / p95 (ms) | Host-to-Device Transfer p50 (ms) | Paced 10 Hz Work Latency p50 / p95 (ms) | Peak Process / GPU Mem (MB) |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| NumPy Reference | Cortex-A78AE CPU (Single-thread) | MAXN_SUPER | FP32 | 49,598 | 444.17 | 1.908 / 4.079 / 8.792 | 1.908 / 4.079 | 0.0 (Host RAM) | 3.078 / 3.768 | 119.61 MB (RSS) |
| PyTorch CUDA | Orin Nano GPU (Ampere) | MAXN_SUPER | FP32 | 49,598 | 418.77 | 2.121 / 3.627 / 4.459 | 1.929 / 3.424 | 0.193 | 5.177 / 7.001 | 17.36 MB (VRAM) |
| PyTorch CUDA | Orin Nano GPU (Ampere) | MAXN_SUPER | FP16 | 49,598 | 447.76 | 2.007 / 3.389 / 4.482 | 1.852 / 3.227 | 0.153 | 5.238 / 5.719 | 17.36 MB (VRAM) |
| PyTorch CUDA (15W) | Orin Nano GPU (Ampere) | 15W_MODE | FP32 | 49,598 | 381.67 | 2.392 / 3.689 / 4.531 | 2.187 / 3.483 | 0.206 | 5.197 / 5.957 | 17.36 MB (VRAM) |
| PyTorch CUDA (15W) | Orin Nano GPU (Ampere) | 15W_MODE | FP16 | 49,598 | 405.14 | 2.258 / 3.761 / 4.493 | 2.093 / 3.573 | 0.165 | 5.092 / 6.994 | 17.36 MB (VRAM) |
