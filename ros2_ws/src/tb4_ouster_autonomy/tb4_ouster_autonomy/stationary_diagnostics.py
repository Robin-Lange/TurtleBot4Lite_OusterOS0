#!/usr/bin/env python3
"""Read-only stationary ROS and Ouster timing/rate diagnostic.

This program subscribes only. It never configures the sensor or publishes ROS data.
Source the ROS 2 underlay and this repository's workspace overlay before use.
"""

import argparse
import json
import math
import time
from collections import defaultdict
from datetime import datetime, timezone
from urllib.request import urlopen

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, LaserScan, Imu
from tf2_msgs.msg import TFMessage
from irobot_create_msgs.msg import DockStatus, HazardDetectionVector


TOPICS = {
    "/odom": Odometry,
    "/tf": TFMessage,
    "/tf_static": TFMessage,
    "/ouster/points": PointCloud2,
    "/ouster/imu": Imu,
    "/ouster/scan": LaserScan,
    "/scan": LaserScan,
    "/dock_status": DockStatus,
    "/hazard_detection": HazardDetectionVector,
}


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1_000_000_000


class StationaryObserver(Node):
    def __init__(self, topics):
        super().__init__("stationary_observer")
        self.topics = topics
        self.samples = defaultdict(list)
        self.frames = defaultdict(set)
        self.topic_subscriptions = []
        for topic in topics:
            message_type = TOPICS[topic]
            qos = qos_profile_sensor_data
            if topic == "/tf_static":
                qos = QoSProfile(
                    depth=1,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=QoSReliabilityPolicy.RELIABLE,
                )
            self.topic_subscriptions.append(self.create_subscription(
                message_type, topic,
                lambda message, name=topic: self.observe(name, message),
                qos,
            ))

    def observe(self, topic, message):
        received = time.time()
        stamps = []
        if hasattr(message, "header"):
            stamps.append(stamp_seconds(message.header.stamp))
            self.frames[topic].add(message.header.frame_id)
        elif topic in ("/tf", "/tf_static"):
            for transform in message.transforms:
                stamps.append(stamp_seconds(transform.header.stamp))
                self.frames[topic].add(
                    f"{transform.header.frame_id}->{transform.child_frame_id}"
                )
        self.samples[topic].append((received, stamps))

    def report(self):
        result = {}
        for topic in self.topics:
            samples = self.samples[topic]
            receive_times = [sample[0] for sample in samples]
            stamps = [stamp for _, group in samples for stamp in group if stamp > 0]
            result[topic] = {
                "messages": len(samples),
                "receive_rate_hz": round(
                    (len(samples) - 1) / (receive_times[-1] - receive_times[0]), 2
                ) if len(samples) > 1 and receive_times[-1] > receive_times[0] else None,
                "latest_stamp_utc": datetime.fromtimestamp(
                    max(stamps), timezone.utc
                ).isoformat() if stamps else None,
                "latest_stamp_age_s": round(receive_times[-1] - max(stamps), 3)
                if stamps else None,
                "frames": sorted(self.frames[topic]),
            }
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--sensor-url", default="http://169.254.97.211")
    parser.add_argument(
        "--topics", default=",".join(TOPICS),
        help="comma-separated topics to observe; omit high-rate topics for focused rate tests",
    )
    args = parser.parse_args(remove_ros_args()[1:])
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds must be finite and positive")
    topics = [topic.strip() for topic in args.topics.split(",") if topic.strip()]
    if not topics or any(topic not in TOPICS for topic in topics):
        parser.error("--topics must name one or more supported topics")

    report = {"host_utc_start": datetime.now(timezone.utc).isoformat()}
    for name, path in (
        ("sensor_info", "/api/v1/sensor/metadata/sensor_info"),
        ("sensor_config", "/api/v1/sensor/config"),
    ):
        try:
            with urlopen(args.sensor_url + path, timeout=3) as response:
                report[name] = json.load(response)
        except (OSError, ValueError) as exc:
            report[name] = {"error": str(exc)}

    rclpy.init()
    observer = StationaryObserver(topics)
    deadline = time.monotonic() + args.seconds
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            remaining_s = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(observer, timeout_sec=min(0.2, remaining_s))
        report["topics"] = observer.report()
    finally:
        observer.destroy_node()
        rclpy.shutdown()
    report["host_utc_end"] = datetime.now(timezone.utc).isoformat()
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
