# TurtleBot 4 Lite + NVIDIA Jetson Orin Nano + Ouster OS0-128 Setup Guide

This guide provides step-by-step instructions to configure the **NVIDIA Jetson Orin Nano** with the **Ouster OS0-128 3D LiDAR** on the **TurtleBot 4 Lite (iRobot Create® 3)** base.

---

## 1. Hardware & Network Architecture (Dual-Ethernet Setup)

By using your **Dell USB 3.0-to-Ethernet Adapter (YX2FJ / DBJBCBC064)**, you achieve a dedicated dual-Ethernet architecture that keeps the high-bandwidth LiDAR traffic isolated from your campus/lab internet network.

```
                                +----------------------------------------------+
                                |   Zeee 4S 14.8V 10,000mAh (148Wh) LiPo       |
                                +----------------------+-----------------------+
                                                       | (XT60 Splitter)
                           14.8V DC Barrel             |            14.8V DC Barrel
                           +---------------------------+---------------------------+
                           |                                                       |
                           v                                                       v
            +------------------------------+                        +------------------------------+
            |   NVIDIA Jetson Orin Nano    |                        |   Ouster OS0 Interface Box   |
            |   (Ubuntu 22.04 / Humble)    |                        |   (Accepts 9V - 58V DC)      |
            +-------+--------------+-------+                        +--------------+---------------+
                    |              |                                               |
  Campus/Lab LAN    |              | Dell USB 3.0 to RJ45 Adapter (YX2FJ)          |
  for Internet/SSH  |              | Dedicated Gigabit Link (1000BASE-T)           |
  (eth0 - DHCP)     |              +<==============================================+
                    |
                    | USB-C to USB-C Cable (Jetson in Device Mode)
                    v
            +------------------------------+
            |     iRobot Create® 3 Base    |
            |   (Runs ROS 2 Base Driver)   |
            +------------------------------+
```

### Network Interface Roles
1. **Onboard RJ45 (`enP8p1s0`):** Connected to your campus/lab network wall jack or router. Handles **Internet access, `apt update`, `git clone`, and remote SSH**.
2. **Dell USB 3.0 to Ethernet (`enx0c379604b359`):** Connected directly to the Ouster Interface Box. Handles **only the Ouster 3D point cloud & IMU UDP stream (15–35 MB/s)** on `169.254.0.0/16`.
3. **USB-C Device Link (`usb1`):** Connected to the Create 3 base via the delivered USB-C to USB-C cable. Handles **motor velocity (`/cmd_vel`), odometry (`/odom`), battery telemetry (`/battery_state`), and hazards** on `192.168.186.0/24`.

---

## 2. Configuring the Dell USB Ethernet Adapter on Jetson

The Dell YX2FJ uses the Realtek RTL8153 Gigabit controller, which is **natively supported by the Linux kernel (`r8152` driver)** with zero third-party drivers needed.

### Step A: Identify the Adapter Name
Plug the Dell adapter into one of the blue USB 3.0 ports on the Jetson, then run:

```bash
# Check USB device detection
lsusb | grep -i realtek
# Output: Bus 002 Device 004: ID 0bda:8153 Realtek Semiconductor Corp. RTL8153 Gigabit Ethernet Adapter

# Find the network interface name
ip link show
```
On this Jetson, the interface name assigned by the kernel/udev is:
* `enP8p1s0`: Onboard RJ45 port (connected to lab/campus LAN).
* `enx0c379604b359`: The Dell USB Gigabit adapter (MAC `0c:37:96:04:b3:59`).

### Step B: Configure Link-Local IP on the Dell Adapter
Set it up permanently using `nmcli` (NetworkManager) with `link-local` mode so both the Jetson and Ouster negotiate `169.254.x.x` addresses automatically:

```bash
sudo nmcli con add type ethernet con-name "ouster-net" ifname enx0c379604b359 ipv4.method link-local
sudo nmcli con up "ouster-net"
```
*(On this setup, the Jetson was automatically assigned `169.254.9.153/16`).*

### Step C: Verify Sensor Discovery
With the Ouster powered on and connected to the Dell adapter:
```bash
# Discover sensors using the official CLI
ouster-cli discover
```
* **Discovered Output:**
  * Model: `OS-0-128` (Serial: `122313000402`, Part: `860-105000-07`)
  * Firmware: `v3.1.0` (`ousteros-image-prod-bootes-v3.1.0+20240426041747`)
  * Sensor IP: `169.254.97.211`
  * Hostname: `os-122313000402.local.`

Verify HTTP communication:
```bash
curl -s http://169.254.97.211/api/v1/sensor/metadata | grep prod_line
# Output returns: "prod_line": "OS-0-128"
```

### Step D: Access & Configure Sensor via HTTP (e.g. Set to 20 Hz)

The Ouster OS0 runs an embedded HTTP REST API on port 80. You can query its parameters, switch operating modes (e.g., from 10 Hz to 20 Hz), and persist the changes across power cycles.

#### 1. Available 20 Hz Modes
* **`1024x20` (Recommended):** 1024 horizontal columns × 128 vertical channels @ 20 Hz (maximum spatial resolution at high frame rate).
* **`512x20`:** 512 horizontal columns × 128 vertical channels @ 20 Hz (lower network and CPU/GPU processing overhead).

#### 2. Configure via HTTP (`curl` commands)

* **Read current configuration:**
  ```bash
  curl -s http://169.254.97.211/api/v1/sensor/config
  ```

* **Set mode to 20 Hz (`1024x20`):**
  ```bash
  curl -X POST http://169.254.97.211/api/v1/sensor/config \
       -H "Content-Type: application/json" \
       -d '{"lidar_mode": "1024x20"}'
  ```

* **Reinitialize the sensor (applies the new mode immediately):**
  ```bash
  curl -X POST http://169.254.97.211/api/v1/sensor/cmd/reinitialize
  ```

* **Save configuration permanently (persists across sensor power cycles):**
  ```bash
  curl -X POST http://169.254.97.211/api/v1/sensor/cmd/save_config_params
  ```

#### 3. Alternative: Configure via `ouster-cli`
You can also use the Ouster CLI utility directly:
```bash
# Apply temporarily (until sensor reboot)
ouster-cli source 169.254.97.211 config lidar_mode 1024x20

# Apply permanently (-p flag saves to sensor EEPROM)
ouster-cli source 169.254.97.211 config -p lidar_mode 1024x20
```

#### 4. Optimize Linux UDP Receive Buffers (Prevents Packet Drop at 20 Hz)
At 20 Hz, the 128-channel LiDAR generates over 2,500 UDP packets per second (~25–35 MB/s). If the Linux socket receive buffer is too small, you may see warnings like `Failed to set desired SO_RCVBUF size`. Run:
```bash
# Temporarily increase UDP socket receive buffers to 25 MB
sudo sysctl -w net.core.rmem_max=26214400
sudo sysctl -w net.core.rmem_default=26214400

# Make persistent across Jetson reboots:
sudo bash -c 'cat << EOF > /etc/sysctl.d/60-ouster-buffers.conf
net.core.rmem_max=26214400
net.core.rmem_default=26214400
EOF'
sudo sysctl --system
```

---

## 3. Verify Ouster 3D Point Clouds in Python & CLI

Install the official Ouster Python SDK on the Jetson:

```bash
pip3 install ouster-sdk numpy requests
```

### Script 1: Point Cloud Stream Verification (`test_ouster_jetson.py`)
Run the Python streaming test:
```bash
python3 test_ouster_jetson.py 169.254.97.211
```
* **What it does:** Fetches metadata, auto-routes sensor UDP to the Jetson IP (`169.254.9.153`), streams live 3D frames, filters returns, and computes point statistics (~56,000–72,000 valid returns per frame at 20 Hz).
* **SDK Compatibility Note:** In modern Ouster SDK releases (v0.16.2+), `open_source()` yields `FrameSet` objects rather than raw `LidarScan` objects. The script extracts the primary frame using `frame = scan[0]` before accessing `frame.field("RANGE")` to prevent `Field 'RANGE' not found in FrameSet` runtime errors.

### Script 2: Diagnostics & Mode Re-Arming (`test_ouster_os0.py`)
Inspect alerts and reconfigure UDP target/frequency via HTTP:
```bash
# Query status and set 20 Hz mode:
python3 test_ouster_os0.py 169.254.97.211 169.254.9.153 1024x20

# Add --persist to write settings to sensor EEPROM:
python3 test_ouster_os0.py 169.254.97.211 169.254.9.153 1024x20 --persist
```

### Launch Interactive 3D Visualizer (GUI)
If you have a monitor connected to the Jetson (or X11 forwarding enabled):
```bash
ouster-cli source 169.254.97.211 viz
```
This pops up an interactive 3D point cloud visualizer rendered using the Tegra Orin GPU (nvgpu), showing 360° depth, calibrated reflectivity, and near-IR ambient camera channels.

---

## 4. ROS 2 Humble Setup & Ouster Driver

### A. Install ROS 2 Packages
> **Note:** If you see `Package 'ouster_ros' not found`, ensure the ROS 2 apt repository packages are installed:
```bash
# If apt fails with "404 Not Found" (due to a recent upstream ROS 2 package sync), clear the cached list:
sudo rm -f /var/lib/apt/lists/*ros.org*

sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-ouster-ros \
    ros-humble-ouster-sensor-msgs \
    ros-humble-irobot-create-msgs \
    ros-humble-teleop-twist-keyboard \
    ros-humble-pointcloud-to-laserscan
```

### B. Launch the Ouster ROS 2 Driver
> **Important for Firmware 3.1.0:** The latest `ouster_ros` (0.15.1+) defaults the profile to expect a `WINDOW` field (window blockage detection introduced in FW 3.2+). For sensors running **Firmware 3.1.0**, you **must** supply `udp_profile_lidar:=LEGACY` to avoid the `Field 'WINDOW' not found in LidarScan` crash.
>
> **Startup Notice:** During initial launch, the sensor takes approximately 10–12 seconds to negotiate phase-locking and network calibration. RViz will automatically open 5 seconds after launch.

```bash
source /opt/ros/humble/setup.bash

# Shortest command with 3D RViz visualization ON (recommended):
ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=169.254.97.211 udp_profile_lidar:=LEGACY

# Headless mode (no RViz GUI window):
ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=169.254.97.211 udp_profile_lidar:=LEGACY viz:=false
```

### C. Verify Published Topics
In a second terminal:
```bash
ros2 topic list
```
You will see:
* `/ouster/points` (`sensor_msgs/msg/PointCloud2`) — Full 128-channel 3D point cloud
* `/ouster/imu` (`sensor_msgs/msg/Imu`) — Built-in 6-axis IMU (gyro + accel)
* `/ouster/nearir_image` (`sensor_msgs/msg/Image`) — Ambient 2D infrared image (works in pitch black)

---

## 5. Connecting the Jetson to the Create 3 Base

The Create® 3 mobile base runs ROS 2 Humble natively and exposes low-level hardware control (`/cmd_vel`), odometry (`/odom`), safety sensors (`/hazard_detection`), and battery telemetry (`/battery_state`).

---

### A. Critical Hardware Architecture: USB Host vs. Device

> [!CAUTION]
> **Do NOT use a USB-C to USB-A cable into the Jetson's USB-A ports!**  
> * The **Create® 3 adapter board is a USB 2.0 Host**.
> * The **Jetson's rectangular USB-A ports are ALSO USB Hosts**.
> * Connecting Host to Host violates USB protocol; neither device will enumerate or communicate.
> * The **front USB-C port** on the Jetson Orin Nano (labeled `USB C Host, Device, USB Recovery`) is the **only upstream Device/Gadget port** on the board.

#### Physical Hookup Steps:
1. **Toggle Switch:** Inside the Create 3 cargo bay, verify the tiny black slider switch on the adapter board is toggled toward the **USB trident icon** (not BLE).
2. **Board Seating:** Press the adapter board firmly straight down into the robot's cargo bay until it clicks flush into the expansion slot.
3. **Cable:** Use the delivered **USB-C to USB-C cable**:
   * One end plugs into the Create 3 adapter board.
   * The other end plugs into the **USB-C port** on the front of the Jetson Orin Nano.
4. **Verify USB Role:**
   ```bash
   cat /sys/class/usb_role/*/role
   # Must return: device
   ```

---

### B. Network Configuration (`usb1` at `192.168.186.3/24`)

When the USB-C to USB-C cable is connected, the Jetson's USB device controller (`tegra-xudc`) enumerates a CDC-NCM Ethernet gadget interface named **`usb1`**.

1. **Configure NetworkManager:**
   Assign the Jetson static IP **`192.168.186.3/24`** and attach the multicast route required for ROS 2 DDS discovery:
   ```bash
   sudo nmcli con add type ethernet con-name "create3-usb1" ifname usb1 ip4 192.168.186.3/24
   sudo nmcli con mod create3-usb1 +ipv4.routes "224.0.0.0/4"
   sudo nmcli con up "create3-usb1"
   ```

2. **Verify Network Link & Ping:**
   The Create 3 base sits at static IP **`192.168.186.2`**:
   ```bash
   ping -c 3 192.168.186.2
   # Success: 3 packets transmitted, 3 received, 0% packet loss, ~2 ms latency
   ```

3. **Access the Create 3 Web Dashboard:**
   Open a browser or curl the robot's embedded web server:
   ```bash
   curl -s http://192.168.186.2/home | grep "VERSION"
   # Output: VERSION: H.2.3
   ```
   *The web GUI at `http://192.168.186.2` provides firmware updates, logs, application restart, and ROS 2 namespace/domain configuration.*

---

### C. FastDDS Transport Profile (`maxMessageSize: 8192`)

> [!IMPORTANT]
> The Create 3 base runs an internal RMW XML override (`/opt/irobot/persistent/opt/irobot/data/bbk/user_rmw_profile.xml`) with:
> * `maxMessageSize`: **`8192` bytes** (standard FastDDS default is 65,500 bytes).
> * `sendBufferSize` / `receiveBufferSize`: **`32768` bytes**.
>
> If the Jetson runs default FastDDS settings, discovery and topic messages exceeding 8192 bytes will be rejected. Furthermore, on multi-NIC systems (Ethernet + WiFi + USB Gadget), FastDDS must whitelist the USB subnet.

1. **FastDDS Profile File (`fastdds_create3.xml`):**
   Save the following to `/home/ivlaborin2/TurtleBot4Lite_OusterOS0/fastdds_create3.xml`:
   ```xml
   <?xml version="1.0" encoding="UTF-8" ?>
   <dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
       <profiles>
           <transport_descriptors>
               <transport_descriptor>
                   <transport_id>udp_transport</transport_id>
                   <type>UDPv4</type>
                   <sendBufferSize>32768</sendBufferSize>
                   <receiveBufferSize>32768</receiveBufferSize>
                   <maxMessageSize>8192</maxMessageSize>
                   <interfaceWhiteList>
                       <address>192.168.186.3</address>
                   </interfaceWhiteList>
               </transport_descriptor>
           </transport_descriptors>
           <participant profile_name="domainparticipant_profile_name" is_default_profile="true">
               <rtps>
                   <userTransports>
                       <transport_id>udp_transport</transport_id>
                   </userTransports>
                   <useBuiltinTransports>false</useBuiltinTransports>
                   <builtin>
                       <initialPeersList>
                           <locator>
                               <udpv4>
                                   <address>192.168.186.2</address>
                               </udpv4>
                           </locator>
                       </initialPeersList>
                   </builtin>
               </rtps>
           </participant>
       </profiles>
   </dds>
   ```

2. **Environment Variables in `~/.bashrc`:**
   Add these lines to `~/.bashrc` to configure FastDDS and Domain ID 0 automatically:
   ```bash
   source /opt/ros/humble/setup.bash
   export ROS_DOMAIN_ID=0
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ivlaborin2/TurtleBot4Lite_OusterOS0/fastdds_create3.xml
   ```

---

### D. Install Create 3 & TurtleBot 4 ROS 2 Packages

Install the required message interfaces and teleoperation utilities:
```bash
sudo apt update
sudo apt install -y \
    ros-humble-irobot-create-msgs \
    ros-humble-turtlebot4-msgs \
    ros-humble-turtlebot4-description \
    ros-humble-turtlebot4-navigation \
    ros-humble-turtlebot4-node \
    ros-humble-teleop-twist-keyboard
```

---

### E. Verify Communication & Telemetry

1. **List All Robot Topics:**
   ```bash
   ros2 topic list
   ```
   You will see the full suite of Create 3 topics alongside Ouster LiDAR topics:
   * `/battery_state` (`sensor_msgs/msg/BatteryState`)
   * `/dock_status` (`irobot_create_msgs/msg/DockStatus`)
   * `/odom` (`nav_msgs/msg/Odometry`)
   * `/cmd_vel` (`geometry_msgs/msg/Twist`)
   * `/hazard_detection` (`irobot_create_msgs/msg/HazardDetectionVector`)
   * `/imu` (`sensor_msgs/msg/Imu`)
   * `/wheel_status`, `/wheel_ticks`, `/wheel_vels`

2. **Echo Live Battery State:**
   ```bash
   ros2 topic echo /battery_state sensor_msgs/msg/BatteryState --once
   ```
   *Returns real-time voltage (~14.2 V), current (+1.05 A charging), and percentage.*

3. **Echo Odometry:**
   ```bash
   ros2 topic echo /odom nav_msgs/msg/Odometry --once
   ```

4. **Echo Docking Status:**
   ```bash
   ros2 topic echo /dock_status irobot_create_msgs/msg/DockStatus --once
   # Returns: is_docked: true
   ```

5. **Drive the Robot:**
   Ensure the robot has at least 1 meter of clear space in front:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   Controls:
   * `i`: Drive forward
   * `,`: Drive backward
   * `j`: Turn left
   * `l`: Turn right
   * `k`: Stop

---

## 6. Real-Time 3D SLAM (KISS-ICP)

Because the Jetson Orin Nano has 1024 CUDA cores, you can run modern GPU-accelerated 3D LiDAR odometry and mapping without CPU bottlenecking.

### Install and Launch KISS-ICP
```bash
sudo apt install -y ros-humble-kiss-icp

# Launch 3D Odometry using the Ouster point cloud
ros2 launch kiss_icp odometry.launch.py \
    pointcloud_topic:=/ouster/points \
    base_frame:=base_link
```
KISS-ICP registers consecutive 3D scans, computes high-precision odometry (`/kiss/odometry`), and builds a real-time 3D voxel map.

---

## 7. Troubleshooting & Hard-Learned Lessons

### A. ROS 2 Driver Crash: `Field 'WINDOW' not found in LidarScan`
* **Symptom:**
  ```text
  [os_driver-1] terminate called after throwing an instance of 'std::out_of_range'
  [os_driver-1]   what():  Field 'WINDOW' not found in LidarScan.
  [ERROR] [os_driver-1]: process has died [exit code -6]
  ```
* **Root Cause:**
  The sensor runs **Firmware 3.1.0** (`ousteros-image-prod-bootes-v3.1.0`). In `ouster_ros` version 0.15.1+, the default profile definition for `RNG19_RFL8_SIG16_NIR16` was updated to require a 6th field: `WINDOW` (window blockage monitoring introduced in FW 3.2+). Because FW 3.1.0 does not transmit this channel, the driver throws `std::out_of_range` when accessing it.
* **Resolution:**
  Always specify `udp_profile_lidar:=LEGACY` in your launch arguments when connecting to sensors with Firmware 3.1.0:
  ```bash
  ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=169.254.97.211 udp_profile_lidar:=LEGACY
  ```
  The `LEGACY` profile reads the 5 standard channels (`RANGE`, `SIGNAL`, `NEAR_IR`, `REFLECTIVITY`, `FLAGS`) and runs stably at 20 Hz without requiring `WINDOW`.

---

### B. Linux Kernel Socket Buffer Constraints (`SO_RCVBUF` Warning)
* **Symptom:**
  ```text
  [warning] Failed to set desired SO_RCVBUF size to 1048576. Actual was 425984. You may experience packet drop unless you allow a larger receive buffer.
  ```
* **Root Cause:**
  At 20 Hz, the 128-beam LiDAR generates ~25–35 MB/s of UDP traffic. Linux kernels cap socket receive buffers by default (`rmem_max = ~212 KB`), which can lead to dropped LiDAR packets during peak loads.
* **Resolution:**
  Expand socket receive buffers to 25 MB:
  ```bash
  sudo sysctl -w net.core.rmem_max=26214400
  sudo sysctl -w net.core.rmem_default=26214400
  ```
  To persist across reboots, add to `/etc/sysctl.d/60-ouster-buffers.conf`:
  ```bash
  sudo bash -c 'cat << EOF > /etc/sysctl.d/60-ouster-buffers.conf
  net.core.rmem_max=26214400
  net.core.rmem_default=26214400
  EOF'
  sudo sysctl --system
  ```

---

### C. `apt install` 404 Not Found During ROS Package Installation
* **Symptom:**
  ```text
  Err:1 http://packages.ros.org/ros2/ubuntu jammy/main arm64 ros-humble-ouster-ros ... 404 Not Found
  ```
* **Root Cause:**
  The ROS buildfarm recently synced `ouster-ros` (bumping from `0.14.2` to `0.15.1`). When repositories sync, older deb binaries are deleted from the pool. If local `apt` has cached the old `InRelease` file, it attempts to download the deleted debs.
* **Resolution:**
  Purge the cached list files so `apt update` re-downloads the fresh package manifest:
  ```bash
  sudo rm -f /var/lib/apt/lists/*ros.org*
  sudo apt update
  sudo apt install -y ros-humble-ouster-ros ros-humble-ouster-sensor-msgs
  ```

---

### D. Python Ouster SDK: `Field 'RANGE' not found in FrameSet`
* **Symptom:**
  ```text
  [ERROR] Scan streaming error: Field 'RANGE' not found in FrameSet.
  ```
* **Root Cause:**
  Modern Ouster SDK releases (v0.16.2+ / v1.0+) return `FrameSet` objects when iterating over `open_source(sensor_ip)`. `FrameSet` contains multi-sensor frames and cannot be queried directly with `.field("RANGE")`.
* **Resolution:**
  Index the primary `LidarFrame` from the `FrameSet` before accessing fields:
  ```python
  for scan in it:
      frame = scan[0] if (hasattr(scan, "__getitem__") and hasattr(scan, "valid_scans")) else scan
      range_field = frame.field("RANGE")
  ```

---

### E. Sensor Network Settings & Discovery Summary
* **Jetson USB Adapter:** Realtek RTL8153 Gigabit controller (`enx0c379604b359`)
* **Jetson Host Link-Local IP:** `169.254.9.153/16`
* **Sensor Model:** Ouster `OS-0-128` (128 beams, Rev 7/8, SN: `122313000402`)
* **Sensor Firmware:** `v3.1.0` (`ousteros-image-prod-bootes-v3.1.0`)
* **Sensor Link-Local IP / Hostname:** `169.254.97.211` / `os-122313000402.local.`
* **Ports Used:** UDP LiDAR `7502`, UDP IMU `7503` (or ephemeral ports when using `sensor.launch.xml`).

---

### F. Create 3 USB Detection Failure: Host-to-Host Collision on Jetson USB-A Ports
* **Symptom:**
  `lsusb` shows no new devices when the robot is plugged into the Jetson, and `dmesg` contains zero USB events.
* **Root Cause:**
  * The Create® 3 cargo bay adapter board is a **USB 2.0 Host** providing 5 V @ 3 A.
  * The Jetson Orin Nano carrier board's 4 rectangular **USB-A ports are ALSO USB Hosts**.
  * Plugging a USB-C to USB-A cable between two USB hosts results in a dead link (neither side negotiates or detects the other).
* **Resolution:**
  * Always use the delivered **USB-C to USB-C cable**.
  * Plug one end into the Create 3 adapter board inside the cargo bay.
  * Plug the other end into the **small oval USB-C port** on the front of the Jetson (the port labeled `USB C Host, Device, USB Recovery`). This is the only port backed by the Tegra USB Device Controller (`tegra-xudc`).
  * Verify `cat /sys/class/usb_role/*/role` switches to `device`.

---

### G. ROS 2 Discovery Failure: FastDDS `maxMessageSize: 8192` & Multi-NIC Mismatch
* **Symptom:**
  `ping 192.168.186.2` succeeds with 0% packet loss and the web GUI at `http://192.168.186.2` works, but `ros2 topic list` returns empty or only shows `/parameter_events` and `/rosout`.
* **Root Cause:**
  1. The Create 3 base runs an internal RMW XML override (`user_rmw_profile.xml`) that clamps `maxMessageSize` to **8192 bytes** (default FastDDS is 65,500 bytes).
  2. The Jetson has multiple active network interfaces (`enP8p1s0` for LAN, `enx...` for LiDAR, and `usb1` for Create 3). By default, FastDDS binds to the default gateway (`enP8p1s0`) and ignores `usb1`.
* **Resolution:**
  Create `/home/ivlaborin2/TurtleBot4Lite_OusterOS0/fastdds_create3.xml` specifying `maxMessageSize: 8192`, an interface whitelist for `192.168.186.3`, and initial unicast peer `192.168.186.2`:
  ```bash
  export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ivlaborin2/TurtleBot4Lite_OusterOS0/fastdds_create3.xml
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_DOMAIN_ID=0
  ```
  Also ensure NetworkManager assigns the multicast route `224.0.0.0/4` to `usb1`:
  ```bash
  sudo nmcli con mod create3-usb1 +ipv4.routes "224.0.0.0/4"
  sudo nmcli con up "create3-usb1"
  ```

---

### H. Create 3 Battery Diagnostics & Charging State Machine
* **Battery Specs:** 4S Lithium-Ion smart pack (14.4 V nominal, ~26 Wh):
  * **100% Charge:** ~16.8 V
  * **0% Charge:** ~12.0 V
  * **BMS Cutoff / Self-Protection:** $\le 10.8\text{ V}$ (the internal BMS disconnects to prevent permanent cell damage).
* **Light Ring Behaviors on the Charging Dock:**
  * **Pulsing Green:** Dock connection established, charger active.
  * **Pulsing Red:** Battery is critically low (<10% state of charge) and actively fast-charging. The robot intentionally keeps motors and USB offline during this phase.
  * **Spinning White:** The onboard Linux computer inside the Create 3 is booting up (takes ~1.5 to 2.5 minutes). **Do not press any buttons during this phase.**
  * **4-Note Ascending Chime:** Boot complete! The base OS and ROS 2 system are fully operational.
  * **Pulsing Red Immediately After Chime:** Normal behavior! The robot finished booting, checked its battery capacity, determined it is still under 10% SoC, and resumed fast charging on the dock while running the ROS 2 driver in the background.
  * **Solid / Dim White:** Battery charged above 10% and ready for off-dock operation.
* **BMS Hard Reset Procedure:**
  If the robot sat discharged for months and refuses to charge:
  1. Unscrew the bottom cover screws and lift out the battery pack.
  2. Leave the battery completely disconnected for at least **15 minutes** to drain BMS residual charge.
  3. Reinstall the battery and place the robot directly onto the charging dock contacts.

---

### I. Power Safety: Why the External 10,000 mAh LiPo Cannot Power the Robot Base
* **Do NOT back-feed the Cargo Bay Port:**
  The JST-XH port on the Create 3 adapter board is strictly a **power OUTPUT** (raw battery voltage, max 2 A limited by a PTC fuse). iRobot documentation explicitly warns: *"Do not back-feed power into the robot through this port."* Doing so will blow the PTC fuse or destroy the charging circuitry.
* **Proprietary Smart Battery Interface:**
  The robot's internal battery bay uses a proprietary blade connector communicating over SMBus/thermistor lines with the base's power manager. Raw RC LiPo batteries (like the Zeee 10,000 mAh pack) lack this interface and cannot be plugged into the internal battery bay.
* **Correct Architecture:**
  The Zeee 4S 10,000 mAh LiPo is dedicated exclusively to powering the **NVIDIA Jetson Orin Nano** (via its DC barrel jack) and the **Ouster OS0-128 LiDAR** (via the interface box barrel jack), while the Create 3 base operates self-contained on its internal 4S pack charged by the Home Base dock.

---

## 8. Official Documentation & References

1. **iRobot Create® 3 Documentation:**
   * [Create 3 Official Documentation Portal](https://iroboteducation.github.io/create3_docs/)
   * [Add a Compute Board: NVIDIA® Jetson™](https://iroboteducation.github.io/create3_docs/hw/add-compute/)
   * [Create 3 ROS 2 Message Definitions (`irobot_create_msgs`)](https://github.com/iRobotEducation/irobot_create_msgs)
2. **Ouster OS0 Documentation:**
   * [Ouster SDK Documentation & Tutorials](https://docs.ouster.com/sdk-docs/index.html)
   * [Ouster OS0 Rev 8 Hardware Datasheet](https://data.ouster.io/downloads/datasheets/datasheet-rev8-v4p0-os0.pdf)
   * [Ouster ROS 2 Official Driver GitHub (`ouster-ros`)](https://github.com/ouster-lidar/ouster-ros)
   * [Webinar: Unleashing the Power of ROS 2 with Ouster](https://ouster.com/insights/webinars/unleashing-the-power-of-ros2/watch)
3. **TurtleBot 4 Documentation:**
   * [TurtleBot 4 Official User Manual](https://turtlebot.github.io/turtlebot4-user-manual/)
   * [TurtleBot 4 Lite Mechanical Details](https://turtlebot.github.io/turtlebot4-user-manual/mechanical/turtlebot4_lite.html)
   * [Clearpath TurtleBot 4 Setup GitHub](https://github.com/turtlebot/turtlebot4_setup)
4. **NVIDIA Isaac & Jetson Resources:**
   * [NVIDIA Jetson Orin Nano Developer Guide](https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit)
   * [NVIDIA Isaac ROS Documentation Portal](https://nvidia-isaac-ros.github.io/)
   * [KISS-ICP 3D LiDAR Odometry & SLAM](https://github.com/PRB-Robotics/kiss-icp)
