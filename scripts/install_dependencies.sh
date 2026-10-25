#!/usr/bin/env bash
# Run on Ubuntu 22.04 / ROS 2 Humble. Network changes require --configure-network.
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo 'Install ROS 2 Humble on Ubuntu 22.04 before running this script.' >&2
    exit 1
fi
sudo apt-get update
sudo apt-get install -y python3-colcon-common-extensions python3-rosdep python3-numpy \
    python3-pytest chrony curl ros-humble-image-transport-plugins ros-humble-foxglove-bridge
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths "${SCRIPT_DIR}/ros2_ws/src" --ignore-src -y --rosdistro humble

# CP210x ID identifies an adapter family, not a particular LiDAR. Select by serial if several exist.
cat <<'RULE' | sudo tee /etc/udev/rules.d/99-rplidar.rules >/dev/null
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0660", GROUP="dialout", SYMLINK+="rplidar"
RULE
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout "$USER"
echo 'Log out and back in for dialout membership. Verify /dev/rplidar identifies the S2 adapter.'

if [ "${1:-}" = '--configure-network' ]; then
    CREATE3_IF="${CREATE3_IF:-usb1}"
    ip link show "$CREATE3_IF" >/dev/null
    # Never delete NVIDIA gadget bridge profiles automatically.
    if ! nmcli connection show create3-usb1 >/dev/null 2>&1; then
        sudo nmcli connection add type ethernet con-name create3-usb1 ifname "$CREATE3_IF" \
            ipv4.method manual ipv4.addresses 192.168.186.3/24
    fi
    sudo nmcli connection modify create3-usb1 connection.interface-name "$CREATE3_IF" \
        ipv4.never-default yes ipv4.route-metric 50 connection.autoconnect yes
    sudo nmcli connection up create3-usb1
    sudo ip route replace 224.0.0.0/4 dev "$CREATE3_IF" metric 50
    if ! grep -q '^allow 192\.168\.186\.0/24$' /etc/chrony/chrony.conf 2>/dev/null; then
        echo 'allow 192.168.186.0/24' | sudo tee -a /etc/chrony/chrony.conf
    fi
    sudo systemctl restart chrony
    echo 'Verify chronyc tracking and Create 3 time; see docs/HARDWARE_AUDIT.md.'
fi
cd "${SCRIPT_DIR}/ros2_ws"
colcon build --symlink-install
