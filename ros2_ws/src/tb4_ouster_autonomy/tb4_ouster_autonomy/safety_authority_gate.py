"""ROS adapters for the ECS safety system; Collision Monitor owns final /cmd_vel."""

from dataclasses import fields
import json
import math
import time

from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import DockStatus, HazardDetection, HazardDetectionVector
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from .components import Command, SafetyConfig, Sample, Velocity, World
from .systems import safety_system, telemetry_problem


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class SafetyAuthorityGate(Node):
    """Only ROS transport and component updates live in this adapter."""

    def __init__(self):
        super().__init__("safety_authority_gate")
        defaults = SafetyConfig()
        values = {}
        for field in fields(defaults):
            self.declare_parameter(field.name, getattr(defaults, field.name))
            values[field.name] = self.get_parameter(field.name).value
        self.config = SafetyConfig(**values)
        self.world = World()
        self.subscriptions_owned = []
        for source, topic in (
            ("TELEOP", "/teleop/cmd_vel"), ("NAV2", "/nav2/cmd_vel"),
            ("EXPLORER", "/explorer/cmd_vel"),
        ):
            self.subscriptions_owned.append(self.create_subscription(
                Twist, topic, lambda msg, entity=source: self.command_callback(entity, msg), 1,
            ))
        for msg_type, topic, callback in (
            (DockStatus, "/dock_status", self.dock_callback),
            (HazardDetectionVector, "/hazard_detection", self.hazard_callback),
            (Odometry, "/odom", self.odom_callback),
            (TFMessage, "/tf", self.tf_callback),
            (LaserScan, "/scan", self.scan_callback),
            (PointCloud2, "/camera/cloud_filtered", lambda msg: self.sample("cloud", msg.header)),
        ):
            self.subscriptions_owned.append(self.create_subscription(
                msg_type, topic, callback, qos_profile_sensor_data,
            ))
        self.subscriptions_owned.extend([
            self.create_subscription(Bool, "/teleop/deadman", self.deadman_callback, 1),
            self.create_subscription(Bool, "/operator/stop", self.operator_stop_callback, 1),
        ])
        self.srv_resume = self.create_service(Trigger, "/safety/resume", self.resume_service_callback)
        self.srv_estop = self.create_service(Trigger, "/safety/emergency_stop", self.estop_service_callback)
        self.pub_cmd_vel_safe = self.create_publisher(Twist, "/cmd_vel_safe", 1)
        self.pub_status = self.create_publisher(String, "/safety/status", 1)
        self.timer = self.create_timer(1.0 / self.config.publish_rate_hz, self.control_loop)

    def sample(self, entity, header):
        self.world.samples[entity] = Sample(time.monotonic(), stamp_to_sec(header.stamp))

    def command_callback(self, source, msg):
        values = (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)
        velocity = Velocity(msg.linear.x, msg.angular.z) if all(map(math.isfinite, values)) else Velocity(math.nan)
        self.world.commands[source] = Command(velocity, time.monotonic())
        if source == "TELEOP":
            self.world.authority.autonomy_paused = True

    def deadman_callback(self, msg):
        self.world.authority.deadman = msg.data
        self.world.authority.deadman_received = time.monotonic()

    def operator_stop_callback(self, msg):
        if msg.data:
            self.world.authority.fault = "Operator stop"
            self.world.commands.clear()

    def dock_callback(self, msg):
        self.world.authority.docked = msg.is_docked
        self.sample("dock", msg.header)

    def hazard_callback(self, msg):
        self.sample("hazard", msg.header)
        self.world.authority.hazards = tuple(d.type for d in msg.detections)
        stopping = (HazardDetection.BUMP, HazardDetection.CLIFF, HazardDetection.STALL, HazardDetection.WHEEL_DROP)
        if any(kind in stopping for kind in self.world.authority.hazards):
            self.world.authority.fault = f"Create 3 hazard: {self.world.authority.hazards}"
            self.world.commands.clear()

    def odom_callback(self, msg):
        if msg.header.frame_id == "odom" and msg.child_frame_id in ("base_link", "base_footprint"):
            self.sample("odom", msg.header)

    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == "odom" and transform.child_frame_id in ("base_link", "base_footprint"):
                self.sample("tf", transform.header)

    def scan_callback(self, msg):
        # Positive infinity is a legitimate no-return; an all-NaN scan is not.
        if msg.ranges and msg.range_max > msg.range_min >= 0 and any(
            (math.isfinite(r) and msg.range_min <= r <= msg.range_max) or r == math.inf for r in msg.ranges
        ):
            self.sample("scan", msg.header)

    def estop_service_callback(self, request, response):
        self.world.authority.fault = "Emergency stop service"
        self.world.commands.clear()
        response.success = True
        response.message = self.world.authority.fault
        return response

    def resume_service_callback(self, request, response):
        problem = telemetry_problem(
            self.world, self.config, time.monotonic(), self.get_clock().now().nanoseconds * 1e-9,
        )
        if self.world.authority.docked or any(h in (1, 2, 3, 4) for h in self.world.authority.hazards) or problem:
            response.success = False
            response.message = problem or "Cannot resume while docked or hazards persist"
        else:
            self.world.authority.fault = ""
            self.world.authority.autonomy_paused = False
            self.world.commands.clear()
            response.success = True
            response.message = "Resumed; awaiting new commands"
        return response

    def control_loop(self):
        decision = safety_system(
            self.world, self.config, time.monotonic(), self.get_clock().now().nanoseconds * 1e-9,
        )
        twist = Twist()
        twist.linear.x = decision.velocity.linear
        twist.angular.z = decision.velocity.angular
        self.pub_cmd_vel_safe.publish(twist)
        status = String()
        status.data = json.dumps({
            "state": decision.state, "active_source": decision.source, "reason": decision.reason,
            "is_docked": self.world.authority.docked, "latched_fault": bool(self.world.authority.fault),
            "autonomy_paused": self.world.authority.autonomy_paused,
        })
        self.pub_status.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyAuthorityGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.pub_cmd_vel_safe.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
