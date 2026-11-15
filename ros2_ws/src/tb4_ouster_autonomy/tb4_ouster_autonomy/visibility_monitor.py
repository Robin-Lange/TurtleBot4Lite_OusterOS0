"""ROS adapter for optional passive scan diagnostics and RViz edge points."""

from dataclasses import fields
import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

from .components import Sample, VisibilityConfig, VisibilityScan
from .visibility import scan_visibility_system, visibility_health_system


class VisibilityMonitor(Node):
    def __init__(self, **kwargs):
        super().__init__('visibility_monitor', **kwargs)
        defaults = VisibilityConfig()
        self.config = VisibilityConfig(**{
            field.name: self.declare_parameter(field.name, getattr(defaults, field.name)).value
            for field in fields(defaults)
        })
        self.sample = None
        self.observation = None
        self.error = ''
        self.header = None
        self.scan_topic = self.declare_parameter('scan_topic', '/scan').value
        self.diagnostics = self.create_publisher(DiagnosticArray, '/visibility/diagnostics', 1)
        self.markers = self.create_publisher(Marker, '/visibility/edges', 1)
        self.subscription = self.create_subscription(
            LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data,
        )
        # Process the newest sample only, at a bounded rate, on the normal executor.
        self.latest_scan = None
        self.timer = self.create_timer(0.2, self.publish_observation)

    def on_scan(self, message):
        self.latest_scan = message
        self.sample = Sample(time.monotonic(), message.header.stamp.sec + message.header.stamp.nanosec * 1e-9)

    def publish_observation(self):
        now = self.get_clock().now()
        ros_now = now.nanoseconds * 1e-9
        if self.latest_scan is not None:
            message, self.latest_scan = self.latest_scan, None
            self.header = message.header
            self.observation, self.error = None, ''
            try:
                if not message.header.frame_id.strip():
                    raise ValueError('Missing scan frame')
                self.observation = scan_visibility_system(VisibilityScan(
                    tuple(message.ranges), message.angle_min, message.angle_max,
                    message.angle_increment, message.range_min, message.range_max,
                ), self.config)
            except ValueError as error:
                self.error = str(error)
        health = visibility_health_system(
            self.observation, self.sample, self.error, self.config, time.monotonic(), ros_now,
        )
        status = DiagnosticStatus()
        status.name, status.hardware_id = 'scan_visibility', self.scan_topic
        status.level = {
            'STALE': DiagnosticStatus.STALE, 'INVALID': DiagnosticStatus.ERROR,
            'LIMITED': DiagnosticStatus.WARN, 'EDGES': DiagnosticStatus.WARN,
            'OBSERVED': DiagnosticStatus.OK,
        }[health.state]
        status.message = health.reason
        values = {'state': health.state, 'control_authority': 'none'}
        if self.header is not None:
            values['frame_id'] = self.header.frame_id
            values['acquisition_stamp_s'] = self.sample.stamp
            values['acquisition_age_s'] = ros_now - self.sample.stamp
        if self.observation is not None and health.show_edges:
            observation = self.observation
            values.update({
                'beam_count': observation.beam_count,
                'finite_return_fraction': observation.finite_count / observation.beam_count,
                'unknown_return_count': observation.beam_count - observation.finite_count,
                'full_circle': observation.full_circle,
                'candidate_edge_count': len(observation.edges),
            })
        status.values = [KeyValue(key=key, value=str(value)) for key, value in values.items()]
        report = DiagnosticArray()
        report.header.stamp = now.to_msg()
        report.status = [status]
        self.diagnostics.publish(report)

        marker = Marker()
        marker.ns, marker.id = 'visibility_candidates', 0
        marker.type = Marker.POINTS
        marker.action = Marker.DELETE
        if self.header is not None:
            marker.header = self.header  # Acquisition frame and stamp; never re-stamp geometry.
        if health.show_edges:
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = 0.10
            marker.color.r, marker.color.g, marker.color.a = 1.0, 0.6, 1.0
            marker.lifetime.sec = 1
            marker.points = [Point(
                x=edge.near_range * math.cos(edge.angle),
                y=edge.near_range * math.sin(edge.angle), z=0.0,
            ) for edge in self.observation.edges]
        self.markers.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = VisibilityMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
