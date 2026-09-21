#!/usr/bin/env python3
"""Bumper contact cloud: marks costmap obstacles at physical collision points.

Subscribes to /hazard_detection (irobot_create_msgs/HazardDetectionVector) and
on any BUMP event publishes a PointCloud2 of synthetic obstacle points in the
base_link frame at the bumper arc radius. Points decay after point_decay_s seconds
so the costmap clears as the robot moves away.

This covers the Ouster OS0's 30 cm minimum-range blind spot: objects that are too
close for the LiDAR to see but close enough to trigger physical contact.

Published topic:
    /safety/bumper_contact_cloud  (sensor_msgs/PointCloud2, base_link frame)
"""

import math
import time

from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


# Bumper arc radius (m) — approximately the Create 3 body radius
BUMPER_RADIUS_M = 0.175


class BumperContactCloud(Node):
    """Converts BUMP hazard events into a short-lived PointCloud2 for Nav2 costmaps."""

    def __init__(self):
        super().__init__('bumper_contact_cloud')

        self.declare_parameter('point_decay_s', 5.0)
        self.declare_parameter('publish_rate_hz', 5.0)

        self.point_decay_s = self.get_parameter('point_decay_s').value
        publish_rate_hz = self.get_parameter('publish_rate_hz').value

        # Each entry: (x, y, z_low, z_high, expiry_monotonic)
        self._active_events: list = []

        self._sub_hazard = self.create_subscription(
            HazardDetectionVector,
            '/hazard_detection',
            self._hazard_callback,
            qos_profile_sensor_data,
        )

        self._pub_cloud = self.create_publisher(
            PointCloud2, '/safety/bumper_contact_cloud', 10
        )

        self._timer = self.create_timer(1.0 / publish_rate_hz, self._publish_callback)

    def _bumper_angles(self, bump_frames: list) -> list:
        """Return arc angles (rad) matching which bumper zones were hit."""
        has_right = any('right' in fid for fid in bump_frames)
        has_left = any('left' in fid for fid in bump_frames)

        if has_right and not has_left:
            # Right side hit — mark right arc
            return [-0.8, -0.5, -0.2]
        elif has_left and not has_right:
            # Left side hit — mark left arc
            return [0.2, 0.5, 0.8]
        else:
            # Center or both sides — mark full front arc
            return [-0.4, -0.2, 0.0, 0.2, 0.4]

    def _hazard_callback(self, msg: HazardDetectionVector) -> None:
        bump_frames = [
            d.header.frame_id
            for d in msg.detections
            if d.type == HazardDetection.BUMP
        ]
        if not bump_frames:
            return

        expiry = time.monotonic() + self.point_decay_s
        angles = self._bumper_angles(bump_frames)

        for a in angles:
            x = BUMPER_RADIUS_M * math.cos(a)
            y = BUMPER_RADIUS_M * math.sin(a)
            self._active_events.append((x, y, expiry))

        self.get_logger().info(
            f'Bumper contact: frames={bump_frames} → '
            f'{len(angles)} arc points, decay in {self.point_decay_s:.1f}s'
        )

    def _publish_callback(self) -> None:
        now = time.monotonic()
        # Expire stale events
        self._active_events = [(x, y, t) for x, y, t in self._active_events if t > now]

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'

        # Two height layers per point: low (ankle) and mid (shin) — within costmap height window
        points = []
        for x, y, _ in self._active_events:
            points.append((x, y, 0.04))
            points.append((x, y, 0.12))

        cloud = pc2.create_cloud_xyz32(header, points)
        self._pub_cloud.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = BumperContactCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
