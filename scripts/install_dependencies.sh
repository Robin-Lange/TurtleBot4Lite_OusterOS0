#!/usr/bin/env bash
# Automated Dependency Installer & System Setup for TurtleBot 4 Lite (Jetson Orin Nano)
# Run this script on a fresh NVIDIA Jetson running Ubuntu 22.04 LTS and ROS 2 Humble.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo "=========================================================="
echo " Starting TurtleBot 4 Lite Setup on NVIDIA Jetson"
echo " Repository: ${SCRIPT_DIR}"
echo "=========================================================="

# Check if ROS 2 Humble is installed
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "ERROR: ROS 2 Humble not found at /opt/ros/humble/setup.bash."
    echo "Please install ROS 2 Humble Hawksbill first: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

echo "[1/6] Updating APT repositories and installing ROS 2 & System Packages..."
sudo apt update
sudo apt install -y \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-rplidar-ros \
    ros-humble-laser-filters \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-teleop-twist-keyboard \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-compressed-image-transport \
    ros-humble-foxglove-bridge \
    python3-colcon-common-extensions \
    python3-rosdep \
    chrony \
    curl \
    net-tools

echo "[2/6] Installing Python dependencies..."
if [ -f "${SCRIPT_DIR}/requirements.txt" ]; then
    pip3 install --upgrade pip || true
    pip3 install -r "${SCRIPT_DIR}/requirements.txt" || true
fi

echo "[3/6] Setting up RPLIDAR S2 udev rules..."
cat << 'RULE' | sudo tee /etc/udev/rules.d/99-rplidar.rules > /dev/null
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="rplidar"
RULE
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout "$USER" || true
echo "  -> Created /etc/udev/rules.d/99-rplidar.rules (/dev/rplidar symlink)"

echo "[4/6] Configuring Create 3 USB-C Network (usb1 @ 192.168.186.3/24)..."
# Remove conflicting l4tbr0 profile if created by NetworkManager
if nmcli connection show "l4tbr0" &>/dev/null; then
    sudo nmcli connection delete "l4tbr0" || true
fi

if ! nmcli connection show "create3-usb1" &>/dev/null; then
    sudo nmcli connection add type ethernet con-name create3-usb1 ifname usb1 ip4 192.168.186.3/24
    sudo nmcli connection modify create3-usb1 ipv4.routes "224.0.0.0/4 192.168.186.3 50"
    sudo nmcli connection modify create3-usb1 ipv4.route-metric 50
    sudo nmcli connection modify create3-usb1 connection.autoconnect yes
    echo "  -> Created NetworkManager profile 'create3-usb1'"
else
    echo "  -> Profile 'create3-usb1' already exists"
fi

echo "[5/6] Configuring Chrony NTP time server for Create 3..."
if ! grep -q "192.168.186.0/24" /etc/chrony/chrony.conf 2>/dev/null; then
    echo "allow 192.168.186.0/24" | sudo tee -a /etc/chrony/chrony.conf
    sudo systemctl restart chrony
    echo "  -> Added 'allow 192.168.186.0/24' to /etc/chrony/chrony.conf and restarted chrony"
else
    echo "  -> Chrony already configured for 192.168.186.0/24"
fi

echo "[6/6] Building ROS 2 Workspace..."
source /opt/ros/humble/setup.bash
cd "${SCRIPT_DIR}/ros2_ws"
colcon build --symlink-install

# Make helper scripts executable
chmod +x "${SCRIPT_DIR}/setup_env.sh"
chmod +x "${SCRIPT_DIR}/scripts/"*.sh
chmod +x "${SCRIPT_DIR}/scripts/"*.py

echo "=========================================================="
echo " Setup Complete! All packages, udev rules, and configs set."
echo " To use the robot, simply run in any terminal:"
echo "   source setup_env.sh"
echo "=========================================================="
