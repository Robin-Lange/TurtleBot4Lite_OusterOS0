# flake8: noqa: E402
"""Exercise ROS cloud conversion, frame preservation, and valid empty observations."""

import time
from unittest.mock import Mock

import numpy as np

import pytest

rclpy = pytest.importorskip('rclpy')
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped

from tb4_ouster_autonomy.cloud_filter import CloudFilter


@pytest.fixture
def cloud_filter():
    rclpy.init()
    node = CloudFilter()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_filter_publishes_empty_base_link_cloud(cloud_filter):
    header = Header()
    header.frame_id = 'base_link'
    header.stamp = cloud_filter.get_clock().now().to_msg()
    msg = pc2.create_cloud_xyz32(header, [])
    cloud_filter.cloud_callback(msg)
    assert cloud_filter.last_published_time_s > 0


def test_missing_optical_transform_does_not_publish_false_health(cloud_filter):
    header = Header()
    header.frame_id = 'unconnected_optical_frame'
    header.stamp = cloud_filter.get_clock().now().to_msg()
    msg = pc2.create_cloud_xyz32(header, [[0, 0, 2]])
    before = cloud_filter.last_published_time_s
    cloud_filter.cloud_callback(msg)
    assert cloud_filter.last_published_time_s == before


def test_all_invalid_depth_is_not_an_empty_scene(cloud_filter):
    header = Header()
    header.frame_id = 'base_link'
    header.stamp = cloud_filter.get_clock().now().to_msg()
    cloud_filter.cloud_callback(pc2.create_cloud_xyz32(header, [[0, 0, float('nan')]]))
    assert cloud_filter.last_published_time_s < time.monotonic() - 100


def test_all_zero_depth_is_not_a_clear_scene(cloud_filter):
    header = Header()
    header.frame_id = 'base_link'
    header.stamp = cloud_filter.get_clock().now().to_msg()
    cloud_filter.cloud_callback(pc2.create_cloud_xyz32(header, [[0, 0, 0]]))
    assert cloud_filter.last_published_time_s < time.monotonic() - 100


def test_optical_cloud_adapter_uses_tf_and_keeps_acquisition_stamp(cloud_filter):
    transform = TransformStamped()
    transform.header.frame_id = 'base_link'
    transform.child_frame_id = 'camera_depth_optical_frame'
    transform.transform.translation.x = -0.05
    transform.transform.translation.y = -0.005
    transform.transform.translation.z = 0.176
    q = transform.transform.rotation
    q.x, q.y, q.z, q.w = -0.5, 0.5, -0.5, 0.5
    cloud_filter.buffer.set_transform_static(transform, 'test_mount')
    header = Header()
    header.frame_id = 'camera_depth_optical_frame'
    header.stamp = cloud_filter.get_clock().now().to_msg()
    msg = pc2.create_cloud_xyz32(header, [[0, 0, 3], [0, 0.176, 2]])
    cloud_filter.pub_cloud = Mock()
    cloud_filter.cloud_callback(msg)
    cloud_filter.pub_cloud.publish.assert_called_once()
    result = cloud_filter.pub_cloud.publish.call_args.args[0]
    assert result.header.frame_id == 'base_link'
    assert result.header.stamp == header.stamp
    points = [tuple(point) for point in pc2.read_points(result, field_names=('x', 'y', 'z'))]
    np.testing.assert_allclose(points, [[2.95, -0.005, 0.176]], atol=1e-6)
