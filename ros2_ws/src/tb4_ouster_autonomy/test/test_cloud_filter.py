#!/usr/bin/env python3
"""Unit tests for CloudFilter node.

Tests self-envelope cropping, ground plane rejection, ceiling rejection,
and minimum range thresholding.
"""

import os
import time
import unittest
import numpy as np

import rclpy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from tb4_ouster_autonomy.cloud_filter import CloudFilter


class TestCloudFilter(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        os.environ["ROS_DOMAIN_ID"] = "89"
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.filter_node = CloudFilter()
        self.test_node = rclpy.create_node("test_cloud_filter_harness")

        self.pub_raw = self.test_node.create_publisher(PointCloud2, "/ouster/points", 10)
        self.received_cloud = None
        self.sub_filtered = self.test_node.create_subscription(
            PointCloud2, "/ouster/cloud_filtered", self.cloud_cb, 10
        )

        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.filter_node)
        self.executor.add_node(self.test_node)

    def tearDown(self):
        self.executor.remove_node(self.filter_node)
        self.executor.remove_node(self.test_node)
        self.filter_node.destroy_node()
        self.test_node.destroy_node()

    def cloud_cb(self, msg: PointCloud2):
        self.received_cloud = msg

    def test_pointcloud_filtering(self):
        header = Header()
        header.stamp = self.test_node.get_clock().now().to_msg()
        header.frame_id = "laser_frame"

        # Define test points:
        # [0]: Valid obstacle at 1.0m
        # [1]: Inside self box (-0.05, 0.05, -0.05)
        # [2]: Below ground plane z = -0.15
        # [3]: Above ceiling z = 2.5
        # [4]: Too close (< 0.30m) (0.10, 0.10, 0.10)
        # [5]: On sensor cap top z = +0.0538m (0.02, -0.02, 0.0538) inside self box
        points = np.array([
            [1.0, 0.5, 0.10],
            [-0.05, 0.05, -0.05],
            [1.0, 1.0, -0.20],
            [1.0, 1.0, 2.50],
            [0.10, 0.10, 0.10],
            [0.02, -0.02, 0.0538],
        ], dtype=np.float32)

        in_msg = pc2.create_cloud_xyz32(header, points)

        # Publish and spin
        for _ in range(5):
            self.pub_raw.publish(in_msg)
            deadline = time.monotonic() + 0.05
            while time.monotonic() < deadline:
                self.executor.spin_once(timeout_sec=0.02)

        self.assertIsNotNone(self.received_cloud)
        filtered_points = pc2.read_points_numpy(self.received_cloud, field_names=["x", "y", "z"])

        # Only the 1st point should survive
        self.assertEqual(len(filtered_points), 1)
        self.assertAlmostEqual(filtered_points[0][0], 1.0, places=2)
        self.assertAlmostEqual(filtered_points[0][1], 0.5, places=2)
        self.assertAlmostEqual(filtered_points[0][2], 0.10, places=2)

    def test_rate_limiter_drops_immediate_second_frame(self):
        """Back-to-back callbacks should publish at most one frame at 10 Hz default target."""
        header = Header()
        header.stamp = self.test_node.get_clock().now().to_msg()
        header.frame_id = "laser_frame"
        points = np.array([[1.0, 0.0, 0.10]], dtype=np.float32)
        in_msg = pc2.create_cloud_xyz32(header, points)

        self.filter_node.last_published_time_s = 0.0
        self.filter_node.cloud_callback(in_msg)
        first_publish_time = self.filter_node.last_published_time_s
        self.assertGreater(first_publish_time, 0.0)

        # Immediate second callback should be rate-limited and not update publish timestamp.
        self.filter_node.cloud_callback(in_msg)
        self.assertEqual(self.filter_node.last_published_time_s, first_publish_time)


if __name__ == "__main__":
    unittest.main()
