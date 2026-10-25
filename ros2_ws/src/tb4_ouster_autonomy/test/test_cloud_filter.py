# flake8: noqa: E402
"""Exercise ROS cloud conversion, frame preservation, and valid empty observations."""

import time

import pytest

rclpy = pytest.importorskip('rclpy')
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

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
