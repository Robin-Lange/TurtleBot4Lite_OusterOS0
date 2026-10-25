"""Transform D455 observations into base_link before range, height, and self cropping."""

from copy import deepcopy
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformException, TransformListener

from .systems import obstacle_filter_system, transform_points


class CloudFilter(Node):
    def __init__(self):
        super().__init__("cloud_filter")
        defaults = {
            "input_topic": "/camera/camera/depth/color/points",
            "output_topic": "/camera/cloud_filtered", "target_frame": "base_link",
            "target_rate_hz": 15.0, "min_range": 0.10, "max_range": 6.0,
            "min_height": 0.04, "max_height": 1.5,
            "self_box_min_x": -0.18, "self_box_max_x": 0.18,
            "self_box_min_y": -0.18, "self_box_max_y": 0.18,
            "self_box_min_z": -0.20, "self_box_max_z": 0.25,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
            setattr(self, name, self.get_parameter(name).value)
        if not math.isfinite(self.target_rate_hz) or self.target_rate_hz <= 0:
            raise ValueError("target_rate_hz must be finite and positive")
        if not 0 <= self.min_range < self.max_range or not self.min_height < self.max_height:
            raise ValueError("Invalid crop limits")
        self.box_min = np.array([getattr(self, f"self_box_min_{axis}") for axis in "xyz"])
        self.box_max = np.array([getattr(self, f"self_box_max_{axis}") for axis in "xyz"])
        self.last_published_time_s = -math.inf
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.sub_cloud = self.create_subscription(
            PointCloud2, self.input_topic, self.cloud_callback, qos_profile_sensor_data,
        )
        self.pub_cloud = self.create_publisher(PointCloud2, self.output_topic, 1)

    def cloud_callback(self, msg):
        now = time.monotonic()
        if now - self.last_published_time_s < 1.0 / self.target_rate_hz:
            return
        try:
            if not {"x", "y", "z"}.issubset(f.name for f in msg.fields) or not msg.header.frame_id:
                return
            raw = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False)
            # Humble sensor_msgs_py versions can return a generator or a structured ndarray.
            if isinstance(raw, np.ndarray) and raw.dtype.names:
                points = np.column_stack([raw[axis].reshape(-1) for axis in "xyz"])
            else:
                points = np.asarray(list(raw), dtype=np.float32).reshape(-1, 3)
            if points.size and not np.isfinite(points).all(axis=1).any():
                return  # Invalid depth is not evidence of an empty scene.
            if msg.header.frame_id != self.target_frame:
                transform = self.buffer.lookup_transform(
                    self.target_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp),
                ).transform
                t, q = transform.translation, transform.rotation
                points = transform_points(points, (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
            filtered = obstacle_filter_system(
                points, self.min_range, self.max_range, self.min_height, self.max_height,
                self.box_min, self.box_max,
            )
            header = deepcopy(msg.header)
            header.frame_id = self.target_frame
            # Keep acquisition time, including empty valid clouds; never stamp stale data as now.
            self.pub_cloud.publish(pc2.create_cloud_xyz32(header, filtered))
            self.last_published_time_s = now
        except (TransformException, ValueError, TypeError) as exc:
            self.get_logger().warn(f"Dropping cloud: {exc}", throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = CloudFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
