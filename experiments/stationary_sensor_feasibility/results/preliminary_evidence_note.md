# Preliminary Evidence Note: Stationary Feasibility Assessment

## Executive Summary of Measured Evidence
This experiment evaluated sustained stationary sensor acquisition and compute feasibility on the physical deployment target: an **NVIDIA Jetson Orin Nano Developer Kit Super** (6-core Cortex-A78AE, Ampere GPU, 7.4 GiB unified RAM) integrated with an Intel RealSense D455 depth camera and a SLAMTEC RPLIDAR S2 scanner.

### Key Measured Findings:
1. **Sustained Multimodal Acquisition (10 Hz LiDAR + 30 Hz RGB-D)**:
   - Concurrent sensor acquisition operated stably with zero driver fault, zero kernel buffer overflows, and zero USB bus contention across all three 120-second trials.
   - The SLAMTEC RPLIDAR S2 achieved a rock-solid **10.01 ± 0.00 Hz** rate across all trials, with an inter-arrival p50 of **99.98 ms**, p95 of **100.36 ms**, p99 of **101.05 ms**, and exactly **0 missing messages**.
   - The Intel RealSense D455 sustained **29.93 ± 0.04 Hz** on RGB, **29.99 ± 0.01 Hz** on raw depth, and **29.89 ± 0.02 Hz** on aligned depth, delivering synchronized 640×480 measurements.
   - Zero zero-timestamps, zero repeated timestamps, zero non-monotonic steps, and zero future timestamps were observed.

2. **Recording & Replay Integrity**:
   - Bounded ROS 2 recording (`rosbag2`) captured 4,314 messages over 43.2 seconds (1.9 GiB) encompassing RGB, aligned depth, scan, and calibration topics with zero packet drops.
   - Replay into an isolated ROS domain (`ROS_DOMAIN_ID=42`) with `/playback` remapping verified 100% topic re-publishing and message integrity without contaminating live robot topics.

3. **Compute Proxy Feasibility (49,598 Parameters)**:
   - The representative compact architecture (128×128×4 RGB-D -> 3 Conv layers -> GAP -> 64-unit GRU -> 4-dim planar state + 10-dim covariance factor) was verified to contain exactly **49,598 parameters**.
   - **GPU Inference (PyTorch 2.5 + CUDA 12.6)**:
     - **MAXN_SUPER Mode**: Maximum throughput was **418.77 Hz (FP32)** and **447.76 Hz (FP16)**. Model forward pass p50 latency was **1.93 ms (FP32)** and **1.85 ms (FP16)**; total step latency (including host-to-device transfer) had a p95 of **3.63 ms (FP32)** and **3.39 ms (FP16)**.
     - **15W Module Mode**: Maximum throughput was **381.67 Hz (FP32)** and **405.14 Hz (FP16)**. Step latency p95 was **3.69 ms (FP32)** and **3.75 ms (FP16)**.
     - Paced 10 Hz execution consumed under **6.0 ms** of work time per 100 ms period, leaving >94% CPU/GPU idle margin.
     - Peak GPU VRAM allocated was **17.36 MB (FP32)** and **12.95 MB (FP16)**, consuming less than **0.43%** of the 4 GB tracking process budget.
   - **CPU Reference (NumPy)**: Single-core Cortex-A78AE achieved **338.79 Hz** max throughput, p95 latency of **7.25 ms**, and peak RSS of **122.1 MB** (3.0% of memory budget).

4. **Power Telemetry & Thermal State**:
   - Baseline idle module power (`VDD_IN`): ~5,409 mW (5.41 W).
   - Sustained concurrent acquisition module power: ~6,560 mW (6.56 W) in MAXN_SUPER mode.
   - Sensor acquisition power delta on module carrier: ~1.15 W.
   - Steady-state junction temperature under sustained acquisition: ~54.1 °C (ambient ~22 °C), well within thermal limits.

---

## Scope Separation & Scientific Integrity Note

### 1. Sustained Sensor Acquisition: [ESTABLISHED]
Physical sensors stream concurrently over USB 3.2 and USB 2.0 without bus contention or dropped scans on the Orin Nano. Received rates match nominal specifications within 0.3%.

### 2. Recording/Replay Integrity: [ESTABLISHED]
Storage demands (~46 MB/s uncompressed) are well within the NVMe throughput capability. Recorded bags preserve exact timestamp and topic counts, and replay cleanly in isolated domains.

### 3. CPU-Only Proxy Execution: [ESTABLISHED]
The mathematical formulation and parameter count (49,598) execute efficiently on Cortex-A78AE CPU cores, proving architectural compactness even without hardware acceleration.

### 4. GPU Proxy Execution: [ESTABLISHED]
Ampere GPU acceleration achieves >380 Hz in 15W mode and <4 ms p95 step latency, confirming that the compact neural backbone fits well within the 100 ms latency target and 4 GB memory budget.

### 5. Full Multimodal Tracking & Moving-Observer Performance: [UNTESTED]
This experiment tested an **untrained proxy architecture on stationary data**. It does NOT demonstrate or validate:
- Closed-loop tracking accuracy or association robustness
- Vehicle egomotion compensation or dynamic observer state estimation
- Occlusion handling under robot locomotion
- Point-cloud projection or multi-modal spatial fusion quality

---

## Remaining Blockers & Smallest Next Steps

1. **Host Python PyTorch Wheel**:
   - *Status*: PyTorch is available inside the Isaac ROS container (`isaac_ros_dev-aarch64`), but absent from host system Python.
   - *Smallest Next Step*: Download and install the official NVIDIA JetPack 6.x / Python 3.10 PyTorch v2.5 wheel (`torch-2.5.0...linux_aarch64.whl`) into a dedicated virtual environment (`venv`) to enable native host inference without container wrappers.

2. **Create 3 Base Interface (`usb1`)**:
   - *Status*: **RESOLVED & AUDITED**. Interface `usb1` is active (`192.168.186.3`), communicating with Create 3 at `192.168.186.2` (ping 2.25 ms).
   - *Telemetry Audit*: Verified battery state (15.02 V, 64.0% SOC), IMU (100.0 Hz), odometry (20.0 Hz), and stationary status (`linear_speed: 0.0 m/s`, `angular_speed: 0.0 rad/s`). Chrony synchronization verified with 0.61 ms RMS offset and 7–37 ms timestamp age.

3. **Color-to-Depth Extrinsic Calibration Verification**:
   - *Status*: Depth-to-color alignment is enabled via `realsense2_camera` driver software (`align_depth.enable:=True`).
   - *Smallest Next Step*: Perform a stationary checkerboard target verification to measure pixel residual alignment errors before relying on spatial RGB-D association in tracking.
