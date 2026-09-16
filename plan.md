# Plan: Connecting & Verifying TurtleBot 4 Lite (Create 3) on NVIDIA Jetson

**Robot Model:** iRobot Create® 3 / TurtleBot 4 Lite (`TB4L-0100 Rev 1.0`)  
**Host Computer:** NVIDIA Jetson Orin Nano (Ubuntu 22.04 LTS / ROS 2 Humble)  
**Status:** Connected via USB-C to USB-C cable (Adapter Board set to USB mode; Jetson in Device Mode)

---

## Overview

This guide documents the verified procedure to connect the **NVIDIA Jetson Orin Nano** to the **TurtleBot 4 Lite (Create 3)** mobile base over USB and command it via ROS 2 Humble:
1. **Physical Architecture:** The Create 3 base operates as a **USB 2.0 Host**, while the Jetson Orin Nano's front USB-C port operates as a **USB Device/Gadget**. Connecting via the delivered **USB-C to USB-C cable** establishes a virtual CDC-NCM Ethernet link (`usb1` on the Jetson).
2. **Network Link:** The Jetson is assigned `192.168.186.3/24` on `usb1`. The Create 3 base responds at `192.168.186.2`.
3. **FastDDS Middleware:** Both sides communicate over Domain ID `0` using matching FastDDS profiles with `maxMessageSize: 8192`.

---

## Phase 1: Hardware & Network Link Verification

### Step 1.1: Physical Connections
1. Set the slider switch on the Create 3 cargo bay adapter board to **USB** (trident icon).
2. Plug the **USB-C to USB-C** cable into the Create 3 adapter board.
3. Plug the other end into the **USB-C port** (labeled *USB C Host, Device, USB Recovery*) on the Jetson Orin Nano.
4. Verify the Jetson USB role switches to device:
```bash
cat /sys/class/usb_role/*/role
# Returns: device
```

---

### Step 1.2: Check Network Interfaces
Run:
```bash
ip link show dev usb1
```
*You will see `usb1: <BROADCAST,MULTICAST,UP,LOWER_UP>` indicating the virtual Ethernet link is UP.*

---

### Step 1.3: Configure the USB Network on Jetson
Assign the Jetson static IP **`192.168.186.3/24`** on `usb1`:

```bash
sudo nmcli con add type ethernet con-name "create3-usb1" ifname usb1 ip4 192.168.186.3/24
sudo nmcli con mod create3-usb1 +ipv4.routes "224.0.0.0/4"
sudo nmcli con up "create3-usb1"
```

---

### Step 1.4: Ping the Robot Base
Test network communication with the Create 3 base (`192.168.186.2`):
```bash
ping -c 3 192.168.186.2
```
* **Success:** 3 packets transmitted, 3 received, 0% packet loss.

---

### Step 1.5: Access the Create 3 Web Dashboard
Open your browser to `http://192.168.186.2` or test with `curl`:
```bash
curl -s http://192.168.186.2/home | grep "VERSION"
# Returns: VERSION: H.2.3
```

---

## Phase 2: Install ROS 2 Humble on the Jetson

If ROS 2 Humble is not yet installed on your Jetson, execute the standard ROS 2 setup:

### Step 2.1: Enable Ubuntu Universe & Add ROS 2 Repositories
```bash
sudo apt update && sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe

# Add the ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to sources.list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 2.2: Install ROS 2 Humble Base & Build Tools
```bash
sudo apt update
sudo apt install -y \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip
```

---

## Phase 3: Install Create 3 Base Packages

Install the message definitions and teleop tools needed to talk to the robot:

```bash
sudo apt update
sudo apt install -y \
    ros-humble-irobot-create-msgs \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rmw-cyclonedds-cpp
```

---

## Phase 4: Configure FastDDS XML Profile for Create 3

The Create 3 base uses **FastDDS** with a customized transport buffer (`maxMessageSize=8192`) on **Domain ID 0**. On multi-NIC systems like the Jetson, FastDDS requires specifying an interface whitelist and initial peer.

### Step 4.1: FastDDS Configuration File
Create `/home/ivlaborin2/TurtleBot4Lite_OusterOS0/fastdds_create3.xml`:
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

### Step 4.2: Update `~/.bashrc`
Add these lines to the bottom of your Jetson's `~/.bashrc`:
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ivlaborin2/TurtleBot4Lite_OusterOS0/fastdds_create3.xml
```

---

## Phase 5: Discover & Control the Robot

### Step 5.1: List Robot Topics
```bash
ros2 topic list
```
**Discovered topics from Create 3:**
* `/battery_state` (`sensor_msgs/msg/BatteryState`)
* `/dock_status` (`irobot_create_msgs/msg/DockStatus`)
* `/odom` (`nav_msgs/msg/Odometry`)
* `/cmd_vel` (`geometry_msgs/msg/Twist`)
* `/hazard_detection` (`irobot_create_msgs/msg/HazardDetectionVector`)
* `/imu` (`sensor_msgs/msg/Imu`)

### Step 5.2: Check Battery State
```bash
ros2 topic echo /battery_state sensor_msgs/msg/BatteryState --once
```

### Step 5.3: Drive the Robot
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Phase 6: Diagnostic & Troubleshooting Matrix

| Symptom | Cause | Solution |
| :--- | :--- | :--- |
| `cat /sys/class/usb_role/*/role` returns `none` | Plugged into USB-A port instead of USB-C | Use the USB-C to USB-C cable plugged into the Jetson's front USB-C port. |
| `usb1` interface does not show up in `ip link` | Switch set to BLE or unseated adapter | 1. Toggle switch toward USB trident icon.<br>2. Firmly seat adapter board into cargo bay. |
| `ping 192.168.186.2` fails | IP not configured on `usb1` | Run `sudo nmcli con up create3-usb1`. |
| `ros2 topic list` empty | FastDDS buffer size or peer mismatch | Ensure `FASTRTPS_DEFAULT_PROFILES_FILE` points to `fastdds_create3.xml` with `maxMessageSize: 8192`. |

---

## Summary Checklist
- [x] Step 1: USB-C to USB-C cable connected to Jetson USB-C port (`role: device`).
- [x] Step 2: `usb1` assigned `192.168.186.3/24`.
- [x] Step 3: `ping 192.168.186.2` returns 0% packet loss.
- [x] Step 4: Web dashboard responsive at `http://192.168.186.2`.
- [x] Step 5: `ros2 topic list` displays `/odom`, `/battery_state`, and `/cmd_vel`.
- [x] Step 6: Live battery and odometry telemetry verified.
