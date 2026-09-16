#!/usr/bin/env python3
"""
Ouster OS0-128 Verification Script for NVIDIA Jetson Orin Nano
--------------------------------------------------------------
Tests HTTP connectivity, queries sensor telemetry, and streams live 3D
scans using the Ouster Python SDK on Linux (Ubuntu 22.04 / JetPack 6).

Usage:
    python3 test_ouster_jetson.py [SENSOR_IP_OR_HOSTNAME]
    Example: python3 test_ouster_jetson.py 169.254.97.211
"""

import sys
import time
import requests
import numpy as np

# Sensor details discovered on your network
DEFAULT_SENSOR_IP = "169.254.97.211"

def print_header(text):
    print("\n" + "=" * 60)
    print(f"  {text}")
    print("=" * 60)

def verify_sensor_http(sensor_ip):
    print_header(f"1. Querying Sensor via HTTP (http://{sensor_ip})")
    try:
        r = requests.get(f"http://{sensor_ip}/api/v1/sensor/metadata", timeout=3.0)
        if r.status_code != 200:
            print(f"[ERROR] HTTP status code {r.status_code}")
            return False
            
        data = r.json()
        info = data.get("sensor_info", {})
        config = data.get("config_params", {})
        
        print(f"[SUCCESS] Sensor Responded!")
        print(f"  - Model:          {info.get('prod_line')}")
        print(f"  - Serial Number:  {info.get('prod_sn')}")
        print(f"  - Firmware:       {info.get('build_rev')}")
        print(f"  - Status:         {info.get('status')}")
        print(f"  - Operating Mode: {config.get('operating_mode')}")
        print(f"  - Lidar Mode:     {config.get('lidar_mode')}")
        print(f"  - Beams/Channels: {data.get('lidar_data_format', {}).get('pixels_per_column')} beams")
        print(f"  - UDP Dest IP:    {config.get('udp_dest')}")
        print(f"  - Lidar Port:     {config.get('udp_port_lidar')}")
        print(f"  - IMU Port:       {config.get('udp_port_imu')}")
        return True
    except Exception as e:
        print(f"[ERROR] Failed to reach sensor at {sensor_ip}: {e}")
        print("Troubleshooting Tips:")
        print("  1. Ensure Ethernet cable is connected to the Dell USB adapter (eth1).")
        print("  2. Check your adapter IP with: ip addr show eth1")
        print("  3. Ensure the adapter has a link-local address: sudo ip addr add 169.254.121.10/16 dev eth1")
        return False

def stream_scans(sensor_ip, num_frames=10):
    print_header("2. Streaming 3D Point Cloud Scans via Ouster SDK")
    try:
        from ouster.sdk import open_source
    except ImportError:
        print("[ERROR] ouster-sdk is not installed.")
        print("Run: pip3 install ouster-sdk")
        return False

    print(f"Connecting to {sensor_ip} ...")
    try:
        src = open_source(sensor_ip, timeout=5.0)
        print("[SUCCESS] Source opened! Receiving live frames...")
        
        it = iter(src)
        frame_idx = 0
        
        for scan in it:
            frame_idx += 1
            # If scan is a FrameSet (multi-sensor container), extract the primary lidar frame
            frame = scan[0] if (hasattr(scan, "__getitem__") and hasattr(scan, "valid_scans")) else scan
            if hasattr(scan, "has_field") and not scan.has_field("RANGE") and hasattr(scan, "__getitem__"):
                frame = scan[0]

            # Retrieve range and reflectivity channels
            range_field = frame.field("RANGE")  # uint32 distances in mm
            reflectivity = frame.field("REFLECTIVITY")
            
            # Filter non-zero ranges
            valid_mask = range_field > 50
            num_valid_points = np.count_nonzero(valid_mask)
            
            if num_valid_points > 0:
                valid_ranges_m = range_field[valid_mask] / 1000.0
                min_m = np.min(valid_ranges_m)
                max_m = np.max(valid_ranges_m)
                mean_m = np.mean(valid_ranges_m)
                print(f"Frame {frame_idx:02d}/{num_frames:02d} | "
                      f"Valid Points: {num_valid_points:6d} | "
                      f"Range: [{min_m:.2f} m - {max_m:.2f} m] | Mean: {mean_m:.2f} m")
            else:
                print(f"Frame {frame_idx:02d}/{num_frames:02d} | Zero returns (check lens cap)")
                
            if frame_idx >= num_frames:
                break
                
        print(f"\n[SUCCESS] Successfully acquired {frame_idx} 3D scans from Ouster OS0!")
        print("Point cloud streaming is fully functional on your Jetson.")
        return True
        
    except Exception as e:
        print(f"[ERROR] Scan streaming error: {e}")
        return False

def main():
    sensor_ip = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_SENSOR_IP
    print("=" * 60)
    print("  Ouster OS0-128 Jetson Orin Nano Bringup Test")
    print("=" * 60)
    print(f"Target Sensor IP: {sensor_ip}")
    
    if verify_sensor_http(sensor_ip):
        stream_scans(sensor_ip, num_frames=10)

if __name__ == "__main__":
    main()
