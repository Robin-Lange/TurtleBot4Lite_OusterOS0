# flake8: noqa: E402
"""Humble message, publisher, timestamp, and passive-node contracts."""

import math
import time
from unittest.mock import Mock

import pytest

rclpy = pytest.importorskip('rclpy')
from diagnostic_msgs.msg import DiagnosticStatus
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

from tb4_ouster_autonomy.visibility_monitor import VisibilityMonitor


@pytest.fixture
def monitor():
    rclpy.init()
    node = VisibilityMonitor()
    node.diagnostics = Mock()
    node.markers = Mock()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def message(node):
    msg = LaserScan()
    msg.header.frame_id = 'laser'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.angle_min, msg.angle_increment = 0., math.tau / 360
    msg.angle_max = 359 * msg.angle_increment
    msg.range_min, msg.range_max = 0.1, 30.
    msg.ranges = [2.] * 180 + [5.] * 180
    return msg


def output(node):
    return (node.diagnostics.publish.call_args.args[0].status[0],
            node.markers.publish.call_args.args[0])


def test_preserves_scan_acquisition_frame_and_stamp_and_reports_edges(monitor):
    msg = message(monitor)
    monitor.on_scan(msg)
    monitor.publish_observation()
    status, marker = output(monitor)
    assert status.level == DiagnosticStatus.WARN
    assert dict((item.key, item.value) for item in status.values)['candidate_edge_count'] == '2'
    assert marker.action == Marker.ADD and marker.type == Marker.POINTS
    assert marker.header == msg.header
    assert len(marker.points) == 2
    assert marker.points[0].x == pytest.approx(2.)
    assert marker.lifetime.sec == 1


@pytest.mark.parametrize('fault', ['old', 'future', 'missing_stamp', 'missing_frame', 'malformed', 'dropout'])
def test_bad_or_expired_data_clear_previously_visible_edges(monitor, fault):
    monitor.on_scan(message(monitor))
    monitor.publish_observation()
    msg = message(monitor)
    if fault == 'old':
        msg.header.stamp.sec -= 2
    elif fault == 'future':
        msg.header.stamp.sec += 2
    elif fault == 'missing_stamp':
        msg.header.stamp.sec = msg.header.stamp.nanosec = 0
    elif fault == 'missing_frame':
        msg.header.frame_id = ''
    elif fault == 'malformed':
        msg.angle_increment = 0.
    if fault == 'dropout':
        monitor.sample = type(monitor.sample)(time.monotonic() - 2., monitor.sample.stamp)
    else:
        monitor.on_scan(msg)
    monitor.publish_observation()
    status, marker = output(monitor)
    assert status.level in (DiagnosticStatus.STALE, DiagnosticStatus.ERROR)
    assert marker.action == Marker.DELETE and not marker.points
    assert 'candidate_edge_count' not in {item.key for item in status.values}


def test_no_scan_is_stale_and_unknown_returns_are_not_clearance(monitor):
    monitor.publish_observation()
    assert output(monitor)[0].level == DiagnosticStatus.STALE
    msg = message(monitor)
    msg.ranges = [math.inf] * 360
    monitor.on_scan(msg)
    monitor.publish_observation()
    status, marker = output(monitor)
    assert status.level == DiagnosticStatus.WARN
    assert 'unknown' in status.message and not marker.points


def test_only_diagnostic_and_visualization_publishers_exist(monitor):
    topics = {publisher.topic_name for publisher in monitor.publishers}
    assert topics <= {'/visibility/diagnostics', '/visibility/edges', '/rosout', '/parameter_events'}


def test_real_sensor_qos_subscription_delivers_scan(monitor):
    from rclpy.qos import qos_profile_sensor_data
    publisher = monitor.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    deadline = time.monotonic() + 5.
    try:
        while time.monotonic() < deadline:
            publisher.publish(message(monitor))
            rclpy.spin_once(monitor, timeout_sec=0.05)
            if monitor.observation is not None:
                break
        assert monitor.observation is not None and len(monitor.observation.edges) == 2
    finally:
        monitor.destroy_publisher(publisher)
