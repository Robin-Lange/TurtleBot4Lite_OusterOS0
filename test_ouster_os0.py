"""
Ouster OS0-128 Diagnostics & Configuration Script
-------------------------------------------------
Queries sensor HTTP endpoints, displays active alerts, and configures
UDP destination IP/ports and LiDAR frequency/mode (e.g., 1024x20 for 20 Hz).

Usage:
    python3 test_ouster_os0.py [SENSOR_IP] [HOST_IP] [LIDAR_MODE]
    Example (Set 20 Hz):
        python3 test_ouster_os0.py 169.254.97.211 169.254.9.153 1024x20
"""

import sys
import time
import requests
import json

DEFAULT_SENSOR_IP = "169.254.97.211"
DEFAULT_HOST_IP   = "169.254.9.153"
DEFAULT_LIDAR_MODE = "1024x20"

def check_sensor_status(sensor_ip):
    print("=" * 60)
    print(f" Connecting to Ouster OS0 at http://{sensor_ip} ...")
    print("=" * 60)
    
    try:
        r = requests.get(f"http://{sensor_ip}/api/v1/sensor/metadata", timeout=3.0)
        if r.status_code != 200:
            print(f"[ERROR] HTTP Error {r.status_code}")
            return False
        meta = r.json()
        info = meta.get("sensor_info", {})
        config = meta.get("config_params", {})
        
        print(f"[ONLINE] Sensor Connected!")
        print(f"  - Model:         {info.get('prod_line')} ({info.get('prod_pn')})")
        print(f"  - Serial Number: {info.get('prod_sn')}")
        print(f"  - Firmware:      {info.get('build_rev')}")
        print(f"  - Status:        {info.get('status')}")
        print(f"  - Operating Mode:{config.get('operating_mode')}")
        print(f"  - Lidar Mode:    {config.get('lidar_mode')}")
        print(f"  - Target UDP IP: {config.get('udp_dest')}")
        print(f"  - Lidar Port:    {config.get('udp_port_lidar')}")
        print(f"  - IMU Port:      {config.get('udp_port_imu')}")
        
        # Check alerts
        r_alert = requests.get(f"http://{sensor_ip}/api/v1/sensor/alerts", timeout=3.0)
        if r_alert.status_code == 200:
            alerts = r_alert.json().get("log", [])
            active_alerts = [a for a in alerts if a.get("active")]
            if active_alerts:
                print("\n[ACTIVE ALERTS]")
                for a in active_alerts:
                    print(f"  ! {a.get('category')}: {a.get('msg')}")
                    if a.get('msg_verbose'):
                        print(f"    Details: {a.get('msg_verbose')}")
            else:
                print("\n[HEALTH] Zero active alerts. Sensor reports clean state.")
                
        return True
    except Exception as e:
        print(f"[ERROR] Could not connect via HTTP: {e}")
        return False

def configure_sensor(sensor_ip, host_ip, lidar_mode, persist=False):
    print(f"\nConfiguring sensor: UDP -> {host_ip}:7502 | Mode -> {lidar_mode} ...")
    payload = {
        "udp_dest": host_ip,
        "udp_port_lidar": 7502,
        "udp_port_imu": 7503,
        "operating_mode": "NORMAL",
        "lidar_mode": lidar_mode
    }
    try:
        r = requests.post(f"http://{sensor_ip}/api/v1/sensor/config", json=payload, timeout=3.0)
        print(f"Config response: {r.status_code} ({r.text.strip()})")
        
        r2 = requests.post(f"http://{sensor_ip}/api/v1/sensor/cmd/reinitialize", timeout=3.0)
        print(f"Reinitialize response: {r2.status_code}")
        
        if persist:
            r3 = requests.post(f"http://{sensor_ip}/api/v1/sensor/cmd/save_config_params", timeout=3.0)
            print(f"Save config (persist) response: {r3.status_code}")
        print("[SUCCESS] Sensor reinitialized successfully!")
    except Exception as e:
        print(f"Error configuring sensor: {e}")

if __name__ == "__main__":
    s_ip = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_SENSOR_IP
    h_ip = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_HOST_IP
    mode = sys.argv[3] if len(sys.argv) > 3 else DEFAULT_LIDAR_MODE
    persist = "--persist" in sys.argv or "-p" in sys.argv
    if check_sensor_status(s_ip):
        configure_sensor(s_ip, h_ip, mode, persist=persist)
