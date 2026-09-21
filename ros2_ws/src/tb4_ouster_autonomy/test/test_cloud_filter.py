#!/usr/bin/env python3
"""Unit tests for CloudFilter node.

Tests self-envelope cropping, ground plane rejection, ceiling rejection,
minimum range thresholding, Enhanced RANSAC ground segmentation,
normal angle gating against vertical walls, and temporal smoothing.
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
        self.received_ground = None

        self.sub_filtered = self.test_node.create_subscription(
            PointCloud2, "/ouster/cloud_filtered", self.cloud_cb, 10
        )
        self.sub_ground = self.test_node.create_subscription(
            PointCloud2, "/ouster/ground_points", self.ground_cb, 10
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

    def ground_cb(self, msg: PointCloud2):
        self.received_ground = msg

    def test_pointcloud_filtering(self):
        header = Header()
        header.stamp = self.test_node.get_clock().now().to_msg()
        header.frame_id = "laser_frame"

        # Define test points:
        # [0]: Valid obstacle at 1.0m (0.27m above ground)
        # [1]: Inside self box (-0.05, 0.05, -0.05)
        # [2]: Below ground plane z = -0.20
        # [3]: Above ceiling z = 2.50
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

    def test_enhanced_ransac_ground_and_low_obstacle_separation(self):
        """Enhanced RANSAC extracts ground plane and preserves low obstacles (5 cm tall)."""
        header = Header()
        header.stamp = self.test_node.get_clock().now().to_msg()
        header.frame_id = "laser_frame"

        # Generate 150 synthetic floor points around z = -0.1712 m
        x_floor = np.linspace(-2.0, 2.0, 15)
        y_floor = np.linspace(-2.0, 2.0, 10)
        xx, yy = np.meshgrid(x_floor, y_floor)
        floor_pts = np.column_stack([
            xx.ravel(),
            yy.ravel(),
            np.full(xx.size, -0.1712, dtype=np.float32),
        ]).astype(np.float32)

        # Filter out points inside self box or too close for clean setup
        mask_valid = (floor_pts[:, 0] ** 2 + floor_pts[:, 1] ** 2 >= 0.35 ** 2)
        floor_pts = floor_pts[mask_valid]

        # Add 3 low obstacle points (e.g. 5 cm above floor => z = -0.1212 m)
        low_obstacle_pts = np.array([
            [1.2, 0.5, -0.1212],
            [1.2, 0.55, -0.1212],
            [1.2, 0.6, -0.1212],
        ], dtype=np.float32)

        all_points = np.vstack([floor_pts, low_obstacle_pts])
        in_msg = pc2.create_cloud_xyz32(header, all_points)

        self.filter_node.last_published_time_s = 0.0
        self.received_cloud = None
        self.received_ground = None

        for _ in range(5):
            self.pub_raw.publish(in_msg)
            deadline = time.monotonic() + 0.05
            while time.monotonic() < deadline:
                self.executor.spin_once(timeout_sec=0.02)

        self.assertIsNotNone(self.received_cloud, "Filtered obstacle cloud must be published")
        self.assertIsNotNone(self.received_ground, "Ground cloud must be published")

        filtered_pts = pc2.read_points_numpy(self.received_cloud, field_names=["x", "y", "z"])
        ground_pts = pc2.read_points_numpy(self.received_ground, field_names=["x", "y", "z"])

        # Ground cloud should contain the floor points
        self.assertGreater(len(ground_pts), 50)
        # Low obstacle points should be preserved in obstacle cloud, while floor is excluded
        self.assertEqual(len(filtered_pts), 3)
        for pt in filtered_pts:
            self.assertAlmostEqual(pt[0], 1.2, places=1)
            self.assertAlmostEqual(pt[2], -0.1212, places=3)

    def test_normal_angle_gating_rejects_vertical_wall(self):
        """Dense vertical wall points must not trick RANSAC into estimating a vertical ground."""
        # 100 vertical wall points at x = 1.0 m, spanning z = [-0.25, 0.50]
        y_vals = np.linspace(-0.5, 0.5, 20)
        z_vals = np.linspace(-0.25, 0.50, 5)
        yy, zz = np.meshgrid(y_vals, z_vals)
        wall_pts = np.column_stack([
            np.full(yy.size, 1.0, dtype=np.float32),
            yy.ravel(),
            zz.ravel(),
        ]).astype(np.float32)

        # Run fit_ground_plane_ransac on seeds from wall
        normal, d, success = self.filter_node.fit_ground_plane_ransac(wall_pts)

        # Normal angle gating must reject wall hypothesis: normal z should stay upright
        self.assertGreaterEqual(
            normal[2],
            self.filter_node.min_cos_tilt,
            "Fitted normal z must be upright (>= cos(15 deg))",
        )
        self.assertFalse(success, "RANSAC must reject vertical wall as ground")

    def test_heterogeneous_fields_ouster_compatibility(self):
        """PointCloud2 with mixed field datatypes (float32, uint16, uint32) is handled without error."""
        from sensor_msgs.msg import PointField
        header = Header()
        header.stamp = self.test_node.get_clock().now().to_msg()
        header.frame_id = "laser_frame"

        # Simulates Ouster OS0 field structure
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
            PointField(name="range", offset=18, datatype=PointField.UINT32, count=1),
        ]
        # Obstacle at x=1.5, y=0.0, z=0.1
        pts = [[1.5, 0.0, 0.1, 100.0, 63, 1500]]
        cloud_msg = pc2.create_cloud(header, fields, pts)

        self.filter_node.last_published_time_s = 0.0
        # Must execute without raising ValueError: All fields need to have the same datatype
        self.filter_node.cloud_callback(cloud_msg)
        self.assertGreater(self.filter_node.last_published_time_s, 0.0)


if __name__ == "__main__":
    unittest.main()
