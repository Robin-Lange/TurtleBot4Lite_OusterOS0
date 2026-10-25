#!/usr/bin/env bash
# Automated Dual-Network Configuration for Jetson AGX Orin
# Sets up USB-to-Ethernet adapter for the private robot router while preserving institutional internet.

set -e

echo "=========================================================="
echo " Jetson AGX Orin: Private Robot Link Network Setup"
echo "=========================================================="

# Check for nmcli
if ! command -v nmcli &>/dev/null; then
    echo "ERROR: NetworkManager (nmcli) is required but not installed."
    exit 1
fi

TARGET_IF="${1:-}"
if [ -z "$TARGET_IF" ]; then
    nmcli -t -f DEVICE,TYPE,STATE device
    echo "Pass the private robot-network interface explicitly: $0 <interface>" >&2
    exit 1
fi
ip link show "$TARGET_IF" >/dev/null

echo "Configuring interface: ${TARGET_IF} -> IP 192.168.0.10/24"

# Update the named profile in place; do not delete a potentially active connection.
if ! nmcli connection show "robot-link" >/dev/null 2>&1; then
    sudo nmcli connection add type ethernet con-name "robot-link" ifname "$TARGET_IF"
fi
sudo nmcli connection modify "robot-link" connection.interface-name "$TARGET_IF" \
    ipv4.method manual ipv4.addresses 192.168.0.10/24 ipv4.never-default yes \
    ipv6.never-default yes connection.autoconnect yes

# Bring up connection
sudo nmcli connection up "robot-link"

echo "=========================================================="
echo " Successfully configured 'robot-link' on ${TARGET_IF}!"
echo " IP Address: 192.168.0.10/24"
echo " Default Internet Gateway: PRESERVED (never-default=yes)"
echo "=========================================================="
echo "Active Routing Table:"
ip route
echo "=========================================================="
echo "Review config/fastdds_agx_orin.xml, then source setup_env.sh."
echo "These profiles use peer discovery; no Discovery Server is configured."
