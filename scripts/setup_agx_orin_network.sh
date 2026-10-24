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

TARGET_IF="$1"

if [ -z "$TARGET_IF" ]; then
    echo "Scanning available network interfaces..."
    nmcli -t -f DEVICE,TYPE,STATE device
    echo "----------------------------------------------------------"
    
    # Try to identify USB ethernet adapter automatically
    # USB adapters usually have 'enx' prefix or are secondary ethernet
    CANDIDATES=$(nmcli -t -f DEVICE,TYPE device | grep ":ethernet$" | cut -d: -f1)
    
    # Exclude standard primary interfaces if obvious
    USB_CANDIDATE=""
    for iface in $CANDIDATES; do
        # Check if interface is a USB device
        DEVPATH=$(readlink -f "/sys/class/net/${iface}" 2>/dev/null || true)
        if [[ "$DEVPATH" == *"usb"* ]]; then
            USB_CANDIDATE="$iface"
            break
        fi
    done
    
    if [ -n "$USB_CANDIDATE" ]; then
        echo "Detected USB Ethernet adapter: ${USB_CANDIDATE}"
        TARGET_IF="$USB_CANDIDATE"
    else
        echo "Could not auto-detect a distinct USB Ethernet adapter."
        echo "Please specify the interface name as an argument:"
        echo "  ./scripts/setup_agx_orin_network.sh <INTERFACE_NAME>"
        echo "Example: ./scripts/setup_agx_orin_network.sh eth1"
        exit 1
    fi
fi

echo "Configuring interface: ${TARGET_IF} -> IP 192.168.0.10/24"

# Remove existing profile if present
if nmcli connection show "robot-link" &>/dev/null; then
    echo "Updating existing 'robot-link' profile..."
    sudo nmcli connection delete "robot-link" || true
fi

# Create new connection profile
sudo nmcli connection add type ethernet con-name "robot-link" ifname "${TARGET_IF}" ip4 192.168.0.10/24
sudo nmcli connection modify "robot-link" ipv4.never-default yes
sudo nmcli connection modify "robot-link" connection.autoconnect yes

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
echo "Next step: Start FastDDS Discovery Server on this AGX Orin:"
echo "  fastdds discovery --server-id 0 -i 192.168.0.10 -p 11811"
echo "=========================================================="
