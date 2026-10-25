"""Publish short-lived contact observations anchored in odom, without broadcasting TF."""

import math
import time

from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from .systems import contact_points_system


BUMPER_RADIUS_M = 0.175


class BumperContactCloud(Node):
    def __init__(self):
        super().__init__("bumper_contact_cloud")
        self.declare_parameter("point_decay_s", 5.0)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.point_decay_s = self.get_parameter("point_decay_s").value
        rate = self.get_parameter("publish_rate_hz").value
        if rate <= 0 or self.point_decay_s <= 0:
            raise ValueError("Contact lifetime and publish rate must be positive")
        self.events = []
        self.in_contact = False
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.sub_hazard = self.create_subscription(
            HazardDetectionVector, "/hazard_detection", self.hazard_callback, qos_profile_sensor_data,
        )
        self.pub_cloud = self.create_publisher(PointCloud2, "/safety/bumper_contact_cloud", 1)
        self.timer = self.create_timer(1.0 / rate, self.publish_callback)

    def hazard_callback(self, msg):
        frames = [d.header.frame_id for d in msg.detections if d.type == HazardDetection.BUMP]
        rising = bool(frames) and not self.in_contact
        self.in_contact = bool(frames)
        if not rising:
            return
        try:
            tf = self.buffer.lookup_transform("odom", "base_link", Time.from_msg(msg.header.stamp)).transform
        except TransformException:
            return
        q = tf.rotation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        has_left = any("left" in frame for frame in frames)
        has_right = any("right" in frame for frame in frames)
        if has_left and not has_right:
            angles = (0.2, 0.5, 0.8)
        elif has_right and not has_left:
            angles = (-0.8, -0.5, -0.2)
        else:
            angles = (-0.4, -0.2, 0.0, 0.2, 0.4)
        points = contact_points_system(angles, BUMPER_RADIUS_M, (tf.translation.x, tf.translation.y, yaw))
        self.events.append((time.monotonic() + self.point_decay_s, points))
        self.events = self.events[-100:]

    def publish_callback(self):
        self.events = [(expiry, points) for expiry, points in self.events if expiry > time.monotonic()]
        header = Header()
        header.frame_id = "odom"
        header.stamp = self.get_clock().now().to_msg()
        points = [point for _, group in self.events for point in group]
        self.pub_cloud.publish(pc2.create_cloud_xyz32(header, points))


def main(args=None):
    rclpy.init(args=args)
    node = BumperContactCloud()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
