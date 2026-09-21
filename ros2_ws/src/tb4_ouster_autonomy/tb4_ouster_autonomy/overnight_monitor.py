#!/usr/bin/env python3
"""Overnight stationary test monitor for Create 3 + Ouster OS0.

Continuously logs system health, battery, odometry drift, Ouster LiDAR rates,
and temperature every 60 seconds without publishing any motion commands.
Appends records to overnight_test.log and updates overnight_summary.json.
"""

import json
import math
import os
import shutil
import time
from datetime import datetime, timezone
from urllib.request import urlopen

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, LaserScan, PointCloud2
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import DockStatus, HazardDetectionVector
from tf2_msgs.msg import TFMessage


def get_jetson_metrics():
    # Load average
    load1, load5, load15 = os.getloadavg()

    # Memory
    mem_total, mem_used, mem_free = 0, 0, 0
    try:
        with open("/proc/meminfo", "r") as f:
            lines = f.readlines()
        mem_info = {}
        for line in lines:
            parts = line.split(":")
            if len(parts) == 2:
                mem_info[parts[0].strip()] = int(parts[1].split()[0])
        mem_total = mem_info.get("MemTotal", 0) // 1024  # MB
        mem_free = (mem_info.get("MemFree", 0) + mem_info.get("Buffers", 0) + mem_info.get("Cached", 0)) // 1024
        mem_used = mem_total - mem_free
    except Exception:
        pass

    # Disk
    disk = shutil.disk_usage("/")
    disk_free_gb = round(disk.free / (1024 ** 3), 2)

    # Temperatures
    temps = {}
    try:
        thermal_dir = "/sys/devices/virtual/thermal"
        for entry in os.listdir(thermal_dir):
            if entry.startswith("thermal_zone"):
                t_type_path = os.path.join(thermal_dir, entry, "type")
                t_temp_path = os.path.join(thermal_dir, entry, "temp")
                if os.path.exists(t_type_path) and os.path.exists(t_temp_path):
                    with open(t_type_path, "r") as tf:
                        z_type = tf.read().strip()
                    with open(t_temp_path, "r") as tf:
                        z_temp = float(tf.read().strip()) / 1000.0
                    temps[z_type] = round(z_temp, 1)
    except Exception:
        pass

    return {
        "load_1m": round(load1, 2),
        "mem_used_mb": mem_used,
        "mem_total_mb": mem_total,
        "disk_free_gb": disk_free_gb,
        "temps_c": temps,
    }


def get_ouster_http_status(sensor_ip="169.254.97.211"):
    try:
        with urlopen(f"http://{sensor_ip}/api/v1/sensor/metadata/sensor_info", timeout=2) as resp:
            info = json.load(resp)
        return {
            "status": info.get("status"),
            "prod_line": info.get("prod_line"),
            "firmware": info.get("image_rev", "")[:40],
        }
    except Exception as exc:
        return {"error": str(exc)}


class OvernightMonitor(Node):
    def __init__(self, log_path, summary_path):
        super().__init__("overnight_stationary_monitor")
        self.log_path = log_path
        self.summary_path = summary_path

        self.start_time = time.time()
        self.sample_count = 0

        # Message counters and stats
        self.count_points = 0
        self.count_scan = 0
        self.count_odom = 0
        self.count_tf = 0

        self.last_battery: dict = {}
        self.is_docked = False
        self.active_hazards = []

        self.initial_pos = None
        self.max_drift_xy_m = 0.0
        self.max_drift_yaw_deg = 0.0

        # Subscriptions
        qos_best_effort = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, "/ouster/points", self.cb_points, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/ouster/scan", self.cb_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.cb_odom, qos_profile_sensor_data)
        self.create_subscription(TFMessage, "/tf", self.cb_tf, qos_profile_sensor_data)
        self.create_subscription(BatteryState, "/battery_state", self.cb_battery, qos_best_effort)
        self.create_subscription(DockStatus, "/dock_status", self.cb_dock, qos_profile_sensor_data)
        self.create_subscription(HazardDetectionVector, "/hazard_detection", self.cb_hazard, qos_profile_sensor_data)

        # 60-second logging timer
        self.timer = self.create_timer(60.0, self.log_snapshot)
        self.get_logger().info(f"OvernightMonitor initialized. Logging to {log_path} every 60s.")

    def cb_points(self, _):
        self.count_points += 1

    def cb_scan(self, _):
        self.count_scan += 1

    def cb_tf(self, _):
        self.count_tf += 1

    def cb_dock(self, msg: DockStatus):
        self.is_docked = msg.is_docked

    def cb_hazard(self, msg: HazardDetectionVector):
        self.active_hazards = [d.type for d in msg.detections if d.type in (1, 2, 3)]

    def cb_battery(self, msg: BatteryState):
        self.last_battery = {
            "percentage": round(msg.percentage * 100, 1),
            "voltage": round(msg.voltage, 2),
            "current_a": round(msg.current, 2),
            "temp_c": round(msg.temperature, 1),
        }

    def cb_odom(self, msg: Odometry):
        self.count_odom += 1
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        if self.initial_pos is None:
            self.initial_pos = (pos.x, pos.y, yaw)
        else:
            dx = pos.x - self.initial_pos[0]
            dy = pos.y - self.initial_pos[1]
            dist = math.hypot(dx, dy)
            self.max_drift_xy_m = max(self.max_drift_xy_m, dist)

            dyaw = abs(yaw - self.initial_pos[2])
            if dyaw > math.pi:
                dyaw = 2.0 * math.pi - dyaw
            self.max_drift_yaw_deg = max(self.max_drift_yaw_deg, math.degrees(dyaw))

    def log_snapshot(self):
        self.sample_count += 1
        now_utc = datetime.now(timezone.utc).isoformat()
        elapsed_s = time.time() - self.start_time

        # Calculate rates over the 60s period
        rate_points = round(self.count_points / 60.0, 2)
        rate_scan = round(self.count_scan / 60.0, 2)
        rate_odom = round(self.count_odom / 60.0, 2)

        # Reset period counters
        self.count_points = 0
        self.count_scan = 0
        self.count_odom = 0
        self.count_tf = 0

        # Host and sensor metrics
        host_metrics = get_jetson_metrics()
        ouster_metrics = get_ouster_http_status()

        record = {
            "sample": self.sample_count,
            "timestamp": now_utc,
            "elapsed_hours": round(elapsed_s / 3600.0, 2),
            "rates_hz": {
                "ouster_points": rate_points,
                "ouster_scan": rate_scan,
                "base_odom": rate_odom,
            },
            "battery": self.last_battery,
            "is_docked": self.is_docked,
            "active_hazards": self.active_hazards,
            "stationary_drift": {
                "max_xy_m": round(self.max_drift_xy_m, 4),
                "max_yaw_deg": round(self.max_drift_yaw_deg, 2),
            },
            "host": host_metrics,
            "ouster_sensor": ouster_metrics,
        }

        # Append to log
        try:
            with open(self.log_path, "a") as f:
                f.write(json.dumps(record) + "\n")
        except Exception as e:
            self.get_logger().error(f"Failed to write log: {e}")

        # Update summary
        try:
            summary = {
                "status": "RUNNING",
                "start_utc": datetime.fromtimestamp(self.start_time, timezone.utc).isoformat(),
                "last_update_utc": now_utc,
                "elapsed_hours": round(elapsed_s / 3600.0, 2),
                "samples_logged": self.sample_count,
                "latest_battery": self.last_battery,
                "latest_rates_hz": record["rates_hz"],
                "max_drift_xy_m": round(self.max_drift_xy_m, 4),
                "max_drift_yaw_deg": round(self.max_drift_yaw_deg, 2),
                "host_summary": host_metrics,
                "ouster_status": ouster_metrics.get("status", "UNKNOWN"),
            }
            with open(self.summary_path, "w") as f:
                json.dump(summary, f, indent=2)
        except Exception as e:
            self.get_logger().error(f"Failed to write summary: {e}")

        self.get_logger().info(
            f"[Overnight Sample {self.sample_count}] "
            f"Ouster pts: {rate_points}Hz, scan: {rate_scan}Hz, odom: {rate_odom}Hz | "
            f"Bat: {self.last_battery.get('percentage', '?')}% | "
            f"CPU Load: {host_metrics['load_1m']}"
        )


def main():
    rclpy.init()
    base_dir = os.getcwd()
    log_path = os.path.join(base_dir, "overnight_test.log")
    summary_path = os.path.join(base_dir, "overnight_summary.json")

    node = OvernightMonitor(log_path, summary_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
