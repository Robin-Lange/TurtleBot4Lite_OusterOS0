# Preliminary Evidence Note: Stationary Feasibility Assessment

## Executive Summary of Measured Evidence

This experiment evaluated sustained stationary sensor acquisition and compute feasibility on the physical deployment target: an **NVIDIA Jetson Orin Nano Developer Kit Super** (6-core Cortex-A78AE, Ampere GPU, 7.4 GiB unified RAM) integrated with an Intel RealSense D455 depth camera, a SLAMTEC RPLIDAR S2 scanner, and an iRobot Create 3 mobile base.

---

### Key Measured Findings

#### 1. Sustained Sensor Acquisition (10 Hz LiDAR + 30 Hz RGB-D)
- **RPLIDAR S2**: Maintained a steady rate of **10.01 ± 0.00 Hz** across all three 120-second trials alone and in concurrent operation. The inter-arrival distribution was tightly bounded (p50: **100.0 ms**, p95: **100.4–102.8 ms**, p99: **101.2–105.7 ms**; standard deviation < 1.9 ms) with **0 missing scans** and 0 timestamp anomalies (zero, repeated, backward, or future timestamps).
- **Intel RealSense D455**: Sustained concurrent acquisition across 120-second runs with observed received rates of **29.89 ± 0.02 Hz** on RGB (`image_raw`), **29.95 ± 0.02 Hz** on aligned depth (`aligned_depth_to_color/image_raw`), and **29.97 ± 0.02 Hz** on raw depth (`depth/image_rect_raw`).
- **Frame Drop Tracking**: Rather than being strictly lossless, minor frame drops were observed in the camera pipeline based on header delta intervals > 1.8× nominal period:
  - In concurrent MAXN mode: RGB color dropped an average of 9.3 frames/run (~0.26%), aligned depth dropped 3.0 frames/run (~0.08%), and raw depth dropped 0.3 frames/run (~0.01%) out of ~3,600 frames per 120s run.
  - In D455 standalone mode: RGB color dropped an average of 21.7 frames/run (~0.60%), aligned depth dropped 7.3 frames/run (~0.20%), and raw depth dropped 1.3 frames/run (~0.04%).
- **Timestamp Integrity**: All published ROS header timestamps were monotonically increasing with 0 zero-stamps, 0 repeated stamps, and 0 future timestamps across all runs.

#### 2. Recording & Replay Integrity
- Bounded ROS 2 recording (`rosbag2`) captured **4,314 messages** over **43.21 seconds** (1.9 GiB across 5 SQLite3 splits) encompassing RGB, aligned depth, scan, and calibration topics.
- Replay into an isolated ROS domain (`ROS_DOMAIN_ID=42`) with topic remapping to `/playback/...` verified 100% topic re-emission, schema readability, and message integrity without contaminating default domain namespaces.

#### 3. Compute Proxy Feasibility (49,598 Parameters)
- **Model Architecture**: Input: 128×128×4 RGB-D → Conv1 (4→16, stride 2, pad 1) → Conv2 (16→32, stride 2, pad 1) → Conv3 (32→64, stride 2, pad 1) → Global Average Pooling (64) → 64-unit GRU → Planar State Head (Linear 64→4) + Covariance Factor Head (Linear 64→10 with softplus diagonal).
- **Exact Parameter Count**: Evaluated at **49,598 parameters** (Conv layers: 23,728; GRU: 24,960; Heads: 910).
- **Execution Separation**: GPU benchmarks were executed in an **isolated container process** (`isaac_ros_dev-aarch64:latest`), *not concurrently* with active ROS sensor ingestion pipelines.
- **Timing Scope**: Measured step latencies include explicit **host-to-device tensor transfer** and **model forward pass**, each bounded by `torch.cuda.synchronize()`. Timings do *not* include upstream ROS message deserialization or 640×480 to 128×128 RGB-D cropping/resizing.
- **Measured Latency & Throughput**:
  - **`MAXN_SUPER` Mode (GPU)**: Max throughput was **418.77 Hz (FP32)** and **447.76 Hz (FP16)**. Model-only forward latency p50 was **1.93 ms (FP32)** and **1.85 ms (FP16)**. Total step latency p95 was **3.63 ms (FP32)** and **3.39 ms (FP16)**.
  - **`15W` Mode (GPU, ID 0)**: Max throughput was **381.67 Hz (FP32)** and **405.14 Hz (FP16)**. Model-only forward latency p50 was **2.19 ms (FP32)** and **2.09 ms (FP16)**. Total step latency p95 was **3.69 ms (FP32)** and **3.76 ms (FP16)**.
  - **Paced 10 Hz Execution**: Work step latency per 100 ms period had a p50 of **5.20 ms** and p95 of **5.96 ms** in 15W mode, demonstrating over 94% execution headroom.
  - **CPU Reference (NumPy)**: Single-core Cortex-A78AE achieved **444.17 Hz** max throughput and p95 latency of **4.08 ms**.
- **Memory Metrics**:
  - GPU metric: Recorded **17.36 MB** peak active CUDA memory allocated (`torch.cuda.max_memory_allocated()`; 24.0 MB reserved). This constitutes **0.43%** of the 4 GB tracking process budget.
  - CPU metric: Recorded **119.61 MB** peak process Resident Set Size (RSS via `getrusage`).

#### 4. Power Telemetry & Thermal Behavior
- **Baseline Idle Power**: `VDD_IN` averaged **5,409 mW** (~5.41 W).
- **Acquisition Power Draw**:
  - RPLIDAR S2 alone: `VDD_IN` p50 was **5,373 mW** (statistically negligible change over idle).
  - D455 alone: `VDD_IN` p50 was **6,315 mW** (mean **6,560 mW**), indicating ~1.15 W delta for camera streaming on the carrier.
  - Both concurrent (MAXN): `VDD_IN` p50 was **6,027 mW**.
  - Both concurrent (15W mode): `VDD_IN` p50 was **6,120 mW** (mean **6,088 mW**).
- **Measurement Boundary**: `VDD_IN` measures the module carrier input supply (SoC, LPDDR5, on-module PMIC, and carrier USB host controller logic). It does not measure peripheral DC self-consumption independently. The configured "15W" mode is an nvpmodel clock and power-gating ceiling, not an actual measured power draw.
- **Thermals**: Under sustained concurrent acquisition, junction temperature (`tj`) stabilized at **53.8–54.1 °C** (ambient ~22 °C), well below hardware thermal throttling limits.

#### 5. Robot Base Telemetry & Host-Base Observations
- **Interface State**: Interface `usb1` was active at `192.168.186.3/24` with 2.25 ms RTT ping to the Create 3 base (`192.168.186.2`).
- **Stationary Status**: Audited `/odom` twist confirmed `linear.x = 0.0 m/s` and `angular.z = 0.0 rad/s` (vehicle strictly stationary).
- **Timing Observation**: The Jetson's local Chrony service tracked upstream NTP with 0.61 ms RMS offset. Observed Create 3 message arrival ages relative to Jetson reception were: `/imu` (7.0 ms), `/hazard_detection` (12.0 ms), `/odom` (37.0 ms), and `/battery_state` (9.0 ms). These latencies reflect combined base clock offset, message assembly, and RNDIS network transport; a formal bidirectional NTP/PTP cross-audit between base OS and Jetson OS remains to be completed.

---

## Scope Separation & Factual Distinction

### 1. Sustained Sensor Acquisition: [ESTABLISHED]
Physical sensors stream concurrently over USB 3.2 Gen 1 (RealSense D455) and USB 2.0 (RPLIDAR S2) without driver faults, bus collisions, or dropped scans on the Orin Nano. Camera streams achieve 29.89–29.97 Hz with 0.01%–0.26% frame drop rates, and LiDAR achieves 10.01 Hz with zero dropped scans.

### 2. Recording/Replay Integrity: [ESTABLISHED]
Storage demands (~45 MB/s uncompressed) operate cleanly within NVMe write capabilities. Recorded bags preserve message counts and schema structures, replaying without data corruption in an isolated ROS domain.

### 3. Proxy Compute Feasibility: [ESTABLISHED]
The exact 49,598-parameter proxy architecture executes on the Orin Nano Ampere GPU at >380 Hz in the verified 15W mode (`nvpmodel -m 0`), with p95 step latency under 3.8 ms (well below the < 100 ms target) and peak allocated VRAM of 17.36 MB (well below the 4 GB budget).

### 4. Full Multimodal Tracking & Moving-Observer Performance: [UNTESTED]
This experiment tested an **untrained proxy on stationary data**. It does NOT demonstrate or validate:
- Closed-loop multi-object tracking accuracy or data association robustness.
- Vehicle ego-motion compensation or dynamic observer state estimation.
- Occlusion recovery or tracking survival under robot locomotion.
- Multi-modal sensor calibration accuracy or optical alignment under vibration.

---

## Remaining Platform Observations & Next Steps

1. **Working Execution Environment**:
   - *Status*: The official container (`isaac_ros_dev-aarch64:latest`, ID `6bb2d422f1be`) provides a functional PyTorch 2.5 + CUDA 12.6 stack without modifying host system packages.
   - *Next Step*: Retain this container environment as the standard execution container for GPU-accelerated node testing.
2. **Sensor Geometric Alignment**:
   - *Status*: Software depth-to-color alignment is active (`align_depth.enable:=True`).
   - *Next Step*: Perform a stationary checkerboard target verification to measure pixel residual alignment errors before relying on spatial RGB-D association in tracking.
3. **End-to-End Pipeline Latency Under Concurrent Ingestion**:
   - *Status*: Compute proxy was profiled in isolation; sensor acquisition was profiled in isolation.
   - *Next Step*: Profile the combined pipeline where camera images and scans are actively ingested, preprocessed, and passed to the model on the GPU concurrently.
