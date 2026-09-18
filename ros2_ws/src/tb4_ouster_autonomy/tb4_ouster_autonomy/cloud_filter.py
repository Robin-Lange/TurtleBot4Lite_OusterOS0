#!/usr/bin/env python3
"""Point cloud filter node for iRobot Create 3 + Ouster OS0.

Filters robot self-returns, ground plane, and high ceiling returns,
and downsamples the PointCloud2 rate to a steady target rate (e.g. 10 Hz)
to reduce Jetson Orin Nano compute load while preserving 3D obstacle geometry.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class CloudFilter(Node):
    """Filters robot self-returns, floor, and ceiling from 3D LiDAR point clouds."""

    def __init__(self):
        super().__init__("cloud_filter")

        # Parameters
        self.declare_parameter("input_topic", "/ouster/points")
        self.declare_parameter("output_topic", "/ouster/cloud_filtered")
        self.declare_parameter("target_rate_hz", 10.0)
        # Bounding box of robot envelope to filter self-returns
        # Approx +/-0.20m in X/Y, -0.20m to +0.02m in Z relative to os_lidar/laser_frame
        self.declare_parameter("self_box_min_x", -0.22)
        self.declare_parameter("self_box_max_x", 0.22)
        self.declare_parameter("self_box_min_y", -0.22)
        self.declare_parameter("self_box_max_y", 0.22)
        self.declare_parameter("self_box_min_z", -0.25)
        # Total robot height is 0.225 m (22.5 cm) above floor.
        # Optical height is 0.1712 m above floor, so robot top is at z = +0.0538 m.
        # self_box_max_z = 0.06 m fully encloses the sensor cap.
        self.declare_parameter("self_box_max_z", 0.06)
        # Height limits relative to laser_frame / os_lidar frame:
        # min_height = -0.12 m means points below 0.051 m above floor (ground) are removed.
        # max_height = 1.5 m keeps obstacles up to ~1.67 m tall.
        self.declare_parameter("min_height", -0.12)
        self.declare_parameter("max_height", 1.50)
        self.declare_parameter("min_range", 0.30)
        self.declare_parameter("max_range", 25.0)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.target_rate_hz = self.get_parameter("target_rate_hz").value
        self.min_period_s = 1.0 / self.target_rate_hz if self.target_rate_hz > 0 else 0.0
        self.last_published_time_s = 0.0

        self.self_box_min_x = self.get_parameter("self_box_min_x").value
        self.self_box_max_x = self.get_parameter("self_box_max_x").value
        self.self_box_min_y = self.get_parameter("self_box_min_y").value
        self.self_box_max_y = self.get_parameter("self_box_max_y").value
        self.self_box_min_z = self.get_parameter("self_box_min_z").value
        self.self_box_max_z = self.get_parameter("self_box_max_z").value

        self.min_height = self.get_parameter("min_height").value
        self.max_height = self.get_parameter("max_height").value
        self.min_range = self.get_parameter("min_range").value
        self.max_range = self.get_parameter("max_range").value

        self.sub_cloud = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            qos_profile_sensor_data,
        )

        self.pub_cloud = self.create_publisher(
            PointCloud2,
            self.output_topic,
            10,
        )

        self.get_logger().info(
            f"CloudFilter initialized. Filtering {self.input_topic} -> {self.output_topic} "
            f"at target rate <= {self.target_rate_hz} Hz."
        )

    def cloud_callback(self, in_msg: PointCloud2):
        now_s = time.monotonic()
        if (now_s - self.last_published_time_s) < (self.min_period_s * 0.9):
            # Downsample to target rate to protect Jetson CPU
            return

        try:
            # Fast vectorized reading of XYZ using numpy
            # Ouster points typically contain x, y, z as float32
            field_names = [f.name for f in in_msg.fields]
            if "x" not in field_names or "y" not in field_names or "z" not in field_names:
                return

            points_gen = pc2.read_points_numpy(in_msg, field_names=["x", "y", "z"])
            if points_gen.size == 0:
                return

            x = points_gen[:, 0]
            y = points_gen[:, 1]
            z = points_gen[:, 2]

            # Range filter
            dist_sq = x * x + y * y + z * z
            valid_mask = (dist_sq >= (self.min_range * self.min_range)) & (
                dist_sq <= (self.max_range * self.max_range)
            )

            # Height filter (relative to sensor optical frame)
            valid_mask &= (z >= self.min_height) & (z <= self.max_height)

            # Self-filter: remove points inside the robot bounding envelope
            in_self_box = (
                (x >= self.self_box_min_x)
                & (x <= self.self_box_max_x)
                & (y >= self.self_box_min_y)
                & (y <= self.self_box_max_y)
                & (z >= self.self_box_min_z)
                & (z <= self.self_box_max_z)
            )
            valid_mask &= ~in_self_box

            filtered_points = points_gen[valid_mask]

            # Construct filtered PointCloud2 message
            out_msg = pc2.create_cloud_xyz32(in_msg.header, filtered_points)
            self.pub_cloud.publish(out_msg)
            self.last_published_time_s = now_s

        except Exception as exc:
            self.get_logger().error(f"Error in cloud_callback: {exc}", throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = CloudFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
