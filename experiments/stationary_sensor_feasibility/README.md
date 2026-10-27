# Stationary Sensor-Acquisition and Compute-Feasibility Experiment

Reproducible preliminary evidence for compact, self-supervised multimodal object tracking on NVIDIA Jetson Orin Nano Super.

## Target Platform & Hardware Configuration
- **Host Device**: NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super (`ivlaborin2-desktop`)
- **Tegra SoC**: Orin / Tegra234 (Chip ID: `0x23`, Module: `3767-300-0005-T.1-1-1-jetson-orin-nano-devkit-super-`)
- **Compute Architecture**: 6-core ARM Cortex-A78AE CPU @ 1.728 GHz max, NVIDIA Ampere GPU (1024 CUDA cores, 32 Tensor Cores)
- **Memory**: 7,620 MB (~7.4 GiB) unified LPDDR5 memory, 23.7 GiB swap (20 GB NVMe file + 3.7 GB compressed zram)
- **OS & Software Stack**: Ubuntu 22.04.5 LTS, L4T R36.4.7 (Kernel `5.15.148-tegra`), ROS 2 Humble (`/opt/ros/humble`), `rmw_fastrtps_cpp`
- **Sensors**:
  - **Camera**: Intel RealSense D455 (`SN: 313522300833`, FW `5.13.0.55`) on USB 3.2 Gen 1 (5000 Mbps)
  - **LiDAR**: SLAMTEC RPLIDAR S2 (`SN: 89C6E386C8E69FC2B8EB9CF409194B71`, FW `1.02`, HW Rev 18) via CP2102N UART bridge (`/dev/rplidar`, 1,000,000 baud)
  - **Robot Base**: iRobot Create 3 (gadget interface `usb1` unpowered/disconnected, vehicle stationary)

## Repository & Git Baseline
- **Repository Checkout**: `/home/ivlaborin2/TurtleBot4Lite_OusterOS0`
- **Branch**: `feature/realsense-rplidar-navigation`
- **HEAD Commit**: `ace9366` ("Merge pull request #3 from Robin-Lange/fix/github-actions")
- **Preserved Modifications**: Local changes in `perception.launch.py`, `components.py`, `fastdds_create3.xml`, and map files are strictly preserved.

---

## Experiment Structure & Workflow

```
stationary_sensor_feasibility/
├── manifest.json                  # Complete machine, hardware, software and QoS manifest
├── README.md                      # Reproduction steps, methodology, and command references
├── scripts/
│   ├── monitor_acquisition.py     # Non-intrusive ROS 2 acquisition & tegrastats monitor
│   ├── run_acquisition_trials.sh  # Per-condition trial orchestrator (warmup + timed run + teardown)
│   ├── run_all_acquisition.sh     # Campaign runner for Step 2 (3 conditions x 3 runs)
│   ├── record_and_replay_bag.sh   # Step 3 bounded recording & domain-isolated replay verification
│   ├── profile_compute_proxy.py   # Step 4 CPU (NumPy) reference proxy benchmark
│   ├── profile_gpu_proxy.py       # Step 4 GPU (PyTorch CUDA FP32/FP16) benchmark
│   └── analyze_all_results.py     # Results aggregation and table generator
├── logs/                          # Raw JSON summary, CSV message intervals, and tegrastats timeseries
├── bags/                          # Bounded ROS 2 bag files
└── results/
    ├── results_table.md           # Aggregated statistics and metric comparison table
    └── preliminary_evidence_note.md# Factual preliminary evidence note
```

---

## Reproduction Commands

### 1. Source Environment
```bash
source /home/ivlaborin2/TurtleBot4Lite_OusterOS0/setup_env.sh
```

### 2. Sustained Sensor Acquisition (Step 2)
To run the full 9-trial acquisition campaign across all 3 conditions:
```bash
/home/ivlaborin2/TurtleBot4Lite_OusterOS0/experiments/stationary_sensor_feasibility/scripts/run_all_acquisition.sh 30 120
```

To run an individual trial (e.g., condition `Both_concurrent`, run 1):
```bash
/home/ivlaborin2/TurtleBot4Lite_OusterOS0/experiments/stationary_sensor_feasibility/scripts/run_acquisition_trials.sh Both_concurrent 1 30 120
```

### 3. Bounded Recording and Isolated Replay (Step 3)
```bash
/home/ivlaborin2/TurtleBot4Lite_OusterOS0/experiments/stationary_sensor_feasibility/scripts/record_and_replay_bag.sh 10
```

### 4. Neural Inference Proxy Benchmarking (Step 4)
#### CPU Implementation Check (NumPy Reference):
```bash
python3 /home/ivlaborin2/TurtleBot4Lite_OusterOS0/experiments/stationary_sensor_feasibility/scripts/profile_compute_proxy.py \
    --warmup-iters 50 \
    --benchmark-iters 500 \
    --paced-hz 10.0 \
    --output-json /home/ivlaborin2/TurtleBot4Lite_OusterOS0/experiments/stationary_sensor_feasibility/results/proxy_cpu_benchmark.json
```

#### GPU Benchmark (Containerized PyTorch 2.5 + CUDA 12.6):
```bash
docker run --rm --runtime nvidia \
    -v /home/ivlaborin2/TurtleBot4Lite_OusterOS0/experiments/stationary_sensor_feasibility:/workspace \
    isaac_ros_dev-aarch64:latest \
    python3 /workspace/scripts/profile_gpu_proxy.py \
    --device cuda \
    --iters 1000 \
    --warmup 100 \
    --power-mode-label MAXN_SUPER \
    --output-json /workspace/results/proxy_gpu_benchmark_maxn_super.json
```

### 5. 15W Power Mode Comparison (Step 5)
```bash
# Switch to verified 15W mode (ID 0)
docker run --rm --privileged \
    -v /usr/sbin/nvpmodel:/usr/sbin/nvpmodel \
    -v /etc/nvpmodel.conf:/etc/nvpmodel.conf \
    -v /var/lib/nvpmodel:/var/lib/nvpmodel \
    -v /sys:/sys \
    isaac_ros_dev-aarch64:latest \
    /usr/sbin/nvpmodel -m 0

# Run concurrent sensor acquisition in 15W mode
/home/ivlaborin2/TurtleBot4Lite_OusterOS0/experiments/stationary_sensor_feasibility/scripts/run_acquisition_trials.sh Both_concurrent 15W 30 120

# Run GPU inference benchmark in 15W mode
docker run --rm --runtime nvidia \
    -v /home/ivlaborin2/TurtleBot4Lite_OusterOS0/experiments/stationary_sensor_feasibility:/workspace \
    isaac_ros_dev-aarch64:latest \
    python3 /workspace/scripts/profile_gpu_proxy.py \
    --device cuda \
    --iters 1000 \
    --warmup 100 \
    --power-mode-label 15W_MODE \
    --output-json /workspace/results/proxy_gpu_benchmark_15w.json

# Restore original MAXN_SUPER mode (ID 2)
docker run --rm --privileged \
    -v /usr/sbin/nvpmodel:/usr/sbin/nvpmodel \
    -v /etc/nvpmodel.conf:/etc/nvpmodel.conf \
    -v /var/lib/nvpmodel:/var/lib/nvpmodel \
    -v /sys:/sys \
    isaac_ros_dev-aarch64:latest \
    /usr/sbin/nvpmodel -m 2
```

### 6. Results Aggregation
```bash
python3 /home/ivlaborin2/TurtleBot4Lite_OusterOS0/experiments/stationary_sensor_feasibility/scripts/analyze_all_results.py
```
