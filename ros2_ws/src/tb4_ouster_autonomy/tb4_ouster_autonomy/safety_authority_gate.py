#!/usr/bin/env python3
"""Safety Authority and Freshness Gate Node for iRobot Create 3.

Enforces strict command hierarchy, source freshness timeouts, deadman switch,
dock status inhibition, telemetry freshness watchdogs (/odom, dynamic /tf, /scan),
and latched fault states on bumper/cliff hazards.

Publishes `/cmd_vel_safe` which feeds `nav2_collision_monitor`.
`nav2_collision_monitor` is the single external writer to Create 3 `/cmd_vel`.
"""

import json
import math
import time
from typing import Optional

from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import DockStatus, HazardDetectionVector
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class RecoveryStage:
    """Stages of bumper escape recovery state machine."""

    IDLE = "IDLE"
    BACKUP = "BACKUP"
    ROTATE = "ROTATE"
    SETTLE = "SETTLE"


class SafetyAuthorityGate(Node):
    """Authority arbiter and safety watchdog."""

    def __init__(self):
        super().__init__("safety_authority_gate")

        # Parameters
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("cmd_timeout_s", 0.20)
        self.declare_parameter("odom_timeout_s", 0.25)
        self.declare_parameter("tf_timeout_s", 0.25)
        self.declare_parameter("scan_timeout_s", 0.35)
        self.declare_parameter("scan_topic", "/ouster/scan")
        self.declare_parameter("allow_nav2", False)
        self.declare_parameter("require_deadman_topic", True)
        self.declare_parameter("deadman_timeout_s", 0.50)
        self.declare_parameter("require_telemetry_for_motion", True)
        self.declare_parameter("max_linear_speed", 0.15)
        self.declare_parameter("max_angular_speed", 0.50)
        self.declare_parameter("bump_backup_speed", 0.06)
        self.declare_parameter("bump_backup_duration_s", 0.50)
        self.declare_parameter("bump_rotate_speed", 0.40)
        self.declare_parameter("bump_rotate_duration_s", 1.20)
        self.declare_parameter("bump_center_rotate_duration_s", 2.00)
        self.declare_parameter("bump_settle_duration_s", 0.20)
        self.declare_parameter("max_consecutive_bumps", 4)
        self.declare_parameter("bump_history_window_s", 10.0)

        self.publish_rate_hz = self.get_parameter("publish_rate_hz").value
        self.cmd_timeout_s = self.get_parameter("cmd_timeout_s").value
        self.odom_timeout_s = self.get_parameter("odom_timeout_s").value
        self.tf_timeout_s = self.get_parameter("tf_timeout_s").value
        self.scan_timeout_s = self.get_parameter("scan_timeout_s").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.allow_nav2 = self.get_parameter("allow_nav2").value
        self.require_deadman_topic = self.get_parameter("require_deadman_topic").value
        self.deadman_timeout_s = self.get_parameter("deadman_timeout_s").value
        self.require_telemetry_for_motion = self.get_parameter("require_telemetry_for_motion").value
        self.max_linear_speed = self.get_parameter("max_linear_speed").value
        self.max_angular_speed = self.get_parameter("max_angular_speed").value
        self.bump_backup_speed = self.get_parameter("bump_backup_speed").value
        self.bump_backup_duration_s = self.get_parameter("bump_backup_duration_s").value
        self.bump_rotate_speed = self.get_parameter("bump_rotate_speed").value
        self.bump_rotate_duration_s = self.get_parameter("bump_rotate_duration_s").value
        self.bump_center_rotate_duration_s = self.get_parameter("bump_center_rotate_duration_s").value
        self.bump_settle_duration_s = self.get_parameter("bump_settle_duration_s").value
        self.max_consecutive_bumps = self.get_parameter("max_consecutive_bumps").value
        self.bump_history_window_s = self.get_parameter("bump_history_window_s").value

        # Internal state
        self.latched_fault = False
        self.fault_reason = ""
        self.is_docked = True  # Safe initial state until confirmed undocked
        self.deadman_active = True if not self.require_deadman_topic else False
        self.last_deadman_time: float = 0.0
        self.active_hazards: list = []

        # Multi-stage Bumper Recovery State Machine
        self.recovery_stage: str = RecoveryStage.IDLE
        self.recovery_deadline: float = 0.0
        self.recovery_twist: Twist = Twist()
        self.escape_direction: str = "none"
        self.consecutive_bumps: int = 0
        self.last_bump_time: float = 0.0
        self.bumper_in_contact: bool = False

        # Last received commands
        self.last_teleop_cmd: Optional[Twist] = None
        self.last_teleop_time: float = 0.0

        self.last_nav2_cmd: Optional[Twist] = None
        self.last_nav2_time: float = 0.0

        # Telemetry timestamps
        self.last_odom_stamp: float = 0.0
        self.last_odom_recv: float = 0.0

        self.last_dynamic_tf_stamp: float = 0.0
        self.last_dynamic_tf_recv: float = 0.0

        self.last_scan_stamp: float = 0.0
        self.last_scan_recv: float = 0.0

        # Subscriptions
        self.sub_teleop = self.create_subscription(
            Twist, "/teleop/cmd_vel", self.teleop_callback, 10
        )
        self.sub_nav2 = self.create_subscription(
            Twist, "/nav2/cmd_vel", self.nav2_callback, 10
        )
        self.sub_deadman = self.create_subscription(
            Bool, "/teleop/deadman", self.deadman_callback, 10
        )
        self.sub_op_stop = self.create_subscription(
            Bool, "/operator/stop", self.operator_stop_callback, 10
        )

        # Base and Sensor Telemetry
        self.sub_dock = self.create_subscription(
            DockStatus, "/dock_status", self.dock_callback, qos_profile_sensor_data
        )
        self.sub_hazard = self.create_subscription(
            HazardDetectionVector, "/hazard_detection", self.hazard_callback, qos_profile_sensor_data
        )
        self.sub_odom = self.create_subscription(
            Odometry, "/odom", self.odom_callback, qos_profile_sensor_data
        )
        self.sub_tf = self.create_subscription(
            TFMessage, "/tf", self.tf_callback, qos_profile_sensor_data
        )
        self.sub_scan = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data
        )

        # Services
        self.srv_resume = self.create_service(
            Trigger, "/safety/resume", self.resume_service_callback
        )
        self.srv_estop = self.create_service(
            Trigger, "/safety/emergency_stop", self.estop_service_callback
        )

        # Publishers
        self.pub_cmd_vel_safe = self.create_publisher(Twist, "cmd_vel_safe", 10)
        self.pub_status = self.create_publisher(String, "/safety/status", 10)
        self.pub_bumper_cloud = self.create_publisher(
            PointCloud2, "/safety/bumper_contact_cloud", 10
        )

        # Timer loop
        timer_period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0 else 0.1
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(
            f"SafetyAuthorityGate started. Rate={self.publish_rate_hz}Hz, "
            f"allow_nav2={self.allow_nav2}, max_vel=({self.max_linear_speed}m/s, {self.max_angular_speed}rad/s)"
        )

    def teleop_callback(self, msg: Twist):
        self.last_teleop_cmd = msg
        self.last_teleop_time = time.monotonic()

    def nav2_callback(self, msg: Twist):
        self.last_nav2_cmd = msg
        self.last_nav2_time = time.monotonic()

    def deadman_callback(self, msg: Bool):
        self.deadman_active = msg.data
        self.last_deadman_time = time.monotonic()

    def operator_stop_callback(self, msg: Bool):
        if msg.data:
            self.latched_fault = True
            self.fault_reason = "Operator emergency stop triggered via /operator/stop"
            self.get_logger().warn(self.fault_reason)
            self._abort_recovery()

    def dock_callback(self, msg: DockStatus):
        self.is_docked = msg.is_docked

    def hazard_callback(self, msg: HazardDetectionVector):
        self.active_hazards = [d.type for d in msg.detections]
        now = time.monotonic()
        hard_hazards = []
        bump_frames = []
        is_stall = False
        is_backup_limit = False

        for d in msg.detections:
            # 0: BACKUP_LIMIT, 1: BUMP, 2: CLIFF, 3: STALL, 4: WHEEL_DROP, 5: OBJECT_PROXIMITY
            if d.type in (2, 4):  # CLIFF, WHEEL_DROP
                hard_hazards.append(f"type_{d.type}")
            elif d.type == 1:  # BUMP
                bump_frames.append(d.header.frame_id)
            elif d.type == 3:  # STALL
                is_stall = True
            elif d.type == 0:  # BACKUP_LIMIT
                is_backup_limit = True

        # 1. Critical hardware safety fault (cliff edge or lifted off ground)
        if hard_hazards:
            self.latched_fault = True
            self.fault_reason = f"Critical safety hazard detected: {', '.join(hard_hazards)}"
            self.get_logger().error(self.fault_reason)
            self._abort_recovery()
            return

        # 2. If backup limit is hit while reversing during recovery, advance immediately to rotate phase
        if is_backup_limit and self.recovery_stage == RecoveryStage.BACKUP:
            self.get_logger().warn("Backup limit reached during recovery backup; advancing directly to rotation")
            self._transition_to_rotate(now)
            return

        # 3. Handle physical collision / stall with rising-edge discrete event tracking
        is_collision_now = bool(bump_frames or is_stall)
        rising_edge_collision = is_collision_now and not self.bumper_in_contact
        self.bumper_in_contact = is_collision_now

        if is_collision_now and not self.latched_fault and not self.is_docked:
            if rising_edge_collision:
                # Consecutive bump check for trapped robot protection
                if (now - self.last_bump_time) > self.bump_history_window_s:
                    self.consecutive_bumps = 0
                self.consecutive_bumps += 1
                self.last_bump_time = now

                if self.consecutive_bumps >= self.max_consecutive_bumps:
                    self.latched_fault = True
                    self.fault_reason = (
                        f"Robot trapped: {self.consecutive_bumps} consecutive collisions within "
                        f"{self.bump_history_window_s:.1f}s"
                    )
                    self.get_logger().error(self.fault_reason)
                    self._abort_recovery()
                    return

            # If already actively recovering and not a new rising edge, do not restart maneuver
            if not rising_edge_collision and self.recovery_stage in (RecoveryStage.BACKUP, RecoveryStage.ROTATE):
                if self.recovery_stage == RecoveryStage.BACKUP and is_stall:
                    self._transition_to_rotate(now)
                return

            # Determine escape direction
            has_right = any("right" in fid for fid in bump_frames)
            has_left = any("left" in fid for fid in bump_frames)

            if has_right and not has_left:
                self.escape_direction = "left"
                reason_detail = "right bumper impact -> rerouting left"
            elif has_left and not has_right:
                self.escape_direction = "right"
                reason_detail = "left bumper impact -> rerouting right"
            else:
                if self.consecutive_bumps % 2 == 0:
                    self.escape_direction = "right"
                else:
                    self.escape_direction = "left"
                reason_detail = f"center impact/stall -> rerouting {self.escape_direction}"

            self.fault_reason = f"Bumper contact ({reason_detail})"
            self.get_logger().warn(self.fault_reason)

            # Invalidate any stale pre-collision drive proposals
            self.last_nav2_cmd = None
            self.last_teleop_cmd = None

            # Publish synthetic bumper contact cloud for costmap obstacle marking
            self.publish_bumper_contact_cloud(self.escape_direction)

            # Start Stage 1: BACKUP
            self.recovery_stage = RecoveryStage.BACKUP
            self.recovery_deadline = now + self.bump_backup_duration_s
            backup_twist = Twist()
            backup_twist.linear.x = -abs(self.bump_backup_speed)
            backup_twist.angular.z = 0.0
            self.recovery_twist = backup_twist

    def odom_callback(self, msg: Odometry):
        self.last_odom_stamp = stamp_to_sec(msg.header.stamp)
        self.last_odom_recv = time.monotonic()

    def tf_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            # Monitor dynamic transforms (odom -> base_link or odom -> base_footprint)
            if transform.header.frame_id == "odom" and transform.child_frame_id in ("base_link", "base_footprint"):
                self.last_dynamic_tf_stamp = stamp_to_sec(transform.header.stamp)
                self.last_dynamic_tf_recv = time.monotonic()

    def scan_callback(self, msg: LaserScan):
        self.last_scan_stamp = stamp_to_sec(msg.header.stamp)
        self.last_scan_recv = time.monotonic()

    def estop_service_callback(self, request, response):
        self.latched_fault = True
        self.fault_reason = "Manual e-stop triggered via /safety/emergency_stop service"
        self._abort_recovery()
        self.get_logger().warn(self.fault_reason)
        response.success = True
        response.message = self.fault_reason
        return response

    def resume_service_callback(self, request, response):
        if self.is_docked:
            response.success = False
            response.message = "Cannot resume while robot is docked."
            return response

        critical_hazards = [h for h in self.active_hazards if h in (2, 4)]
        if critical_hazards:
            response.success = False
            response.message = f"Cannot resume while critical hazards persist: {critical_hazards}"
            return response

        self.latched_fault = False
        self.fault_reason = ""
        self.consecutive_bumps = 0
        self._abort_recovery()
        self.get_logger().info("Latched fault cleared via /safety/resume")
        response.success = True
        response.message = "Safety gate resumed. Fault cleared."
        return response

    @property
    def bump_recovery_active(self) -> bool:
        return self.recovery_stage != RecoveryStage.IDLE

    def _abort_recovery(self):
        """Immediately abort any active recovery."""
        self.recovery_stage = RecoveryStage.IDLE
        self.recovery_twist = Twist()

    def _transition_to_rotate(self, now: float):
        """Transition from BACKUP to ROTATE stage."""
        self.recovery_stage = RecoveryStage.ROTATE
        rotate_twist = Twist()
        rotate_twist.linear.x = 0.0

        if self.escape_direction == "left":
            rotate_twist.angular.z = abs(self.bump_rotate_speed)
            duration = self.bump_rotate_duration_s
        elif self.escape_direction == "right":
            rotate_twist.angular.z = -abs(self.bump_rotate_speed)
            duration = self.bump_rotate_duration_s
        else:
            rotate_twist.angular.z = abs(self.bump_rotate_speed)
            duration = self.bump_center_rotate_duration_s

        if self.consecutive_bumps > 1:
            duration *= 1.3

        self.recovery_twist = rotate_twist
        self.recovery_deadline = now + duration
        self.get_logger().info(
            f"Recovery Stage 2: turning {self.escape_direction} "
            f"(z={rotate_twist.angular.z:.2f} rad/s for {duration:.1f}s)"
        )

    def _transition_to_settle(self, now: float):
        """Transition from ROTATE to SETTLE stage."""
        self.recovery_stage = RecoveryStage.SETTLE
        self.recovery_twist = Twist()
        self.recovery_deadline = now + self.bump_settle_duration_s
        self.get_logger().info(f"Recovery Stage 3: settling for {self.bump_settle_duration_s:.1f}s")

    def _complete_recovery(self):
        """Complete recovery maneuver and restore normal operation."""
        self.get_logger().info("Bumper recovery maneuver complete. Normal motion restored.")
        self.recovery_stage = RecoveryStage.IDLE
        self.recovery_twist = Twist()
        self.last_nav2_cmd = None
        self.last_teleop_cmd = None

    def publish_bumper_contact_cloud(self, side: str):
        """Publish synthetic 3D points at bumper contact location for costmap marking."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_link"

        if side == "right":
            angles = [-0.8, -0.5, -0.2]
        elif side == "left":
            angles = [0.2, 0.5, 0.8]
        else:
            angles = [-0.4, -0.2, 0.0, 0.2, 0.4]

        radius = 0.175
        points = []
        for a in angles:
            x = radius * math.cos(a)
            y = radius * math.sin(a)
            points.append([x, y, 0.03])
            points.append([x, y, 0.07])

        cloud = pc2.create_cloud_xyz32(header, points)
        self.pub_bumper_cloud.publish(cloud)

    def check_telemetry_freshness(self, now: float) -> tuple[bool, str]:
        """Check whether odom, dynamic TF, and scan have fresh data."""
        if self.last_odom_recv == 0.0 or (now - self.last_odom_recv) > self.odom_timeout_s:
            return False, f"Stale /odom (age {now - self.last_odom_recv:.3f}s > {self.odom_timeout_s}s)"
        if self.last_dynamic_tf_recv == 0.0 or (now - self.last_dynamic_tf_recv) > self.tf_timeout_s:
            return False, f"Stale dynamic /tf (age {now - self.last_dynamic_tf_recv:.3f}s > {self.tf_timeout_s}s)"
        if self.last_scan_recv == 0.0 or (now - self.last_scan_recv) > self.scan_timeout_s:
            return False, f"Stale /scan (age {now - self.last_scan_recv:.3f}s > {self.scan_timeout_s}s)"
        return True, "Fresh"

    def clamp_twist(self, twist: Twist) -> Twist:
        clamped = Twist()
        max_lin = abs(self.max_linear_speed)
        max_ang = abs(self.max_angular_speed)
        clamped.linear.x = max(-max_lin, min(max_lin, twist.linear.x))
        clamped.linear.y = 0.0  # Create 3 is a non-holonomic differential-drive base
        clamped.linear.z = 0.0
        clamped.angular.x = 0.0
        clamped.angular.y = 0.0
        clamped.angular.z = max(-max_ang, min(max_ang, twist.angular.z))
        return clamped

    def control_loop(self):
        now = time.monotonic()
        out_twist = Twist()  # Default zero velocity
        state = "IDLE"
        active_source = "NONE"
        reason = "No active command"

        # 1. Check Latched Fault
        if self.latched_fault:
            state = "LATCHED_FAULT"
            reason = self.fault_reason
            self.pub_cmd_vel_safe.publish(out_twist)
            self.publish_status(state, active_source, reason, now)
            return

        # 2. Check Dock State
        if self.is_docked:
            state = "INHIBITED_DOCKED"
            reason = "Robot is docked; wheel motion prohibited"
            self.pub_cmd_vel_safe.publish(out_twist)
            self.publish_status(state, active_source, reason, now)
            return

        # 3. Check Active Bumper Recovery State Machine
        if self.recovery_stage != RecoveryStage.IDLE:
            if self.recovery_stage == RecoveryStage.BACKUP:
                if now >= self.recovery_deadline:
                    self._transition_to_rotate(now)
            elif self.recovery_stage == RecoveryStage.ROTATE:
                if now >= self.recovery_deadline:
                    self._transition_to_settle(now)
            elif self.recovery_stage == RecoveryStage.SETTLE:
                if now >= self.recovery_deadline:
                    self._complete_recovery()

            if self.recovery_stage != RecoveryStage.IDLE:
                state = "BUMP_RECOVERY"
                active_source = "BUMP_REFLEX"
                reason = f"Bumper recovery ({self.recovery_stage.lower()}): {self.fault_reason}"
                self.pub_cmd_vel_safe.publish(self.recovery_twist)
                self.publish_status(state, active_source, reason, now, stage=self.recovery_stage)
                return

        # 4. Check Deadman Switch & Heartbeat
        deadman_expired = (
            self.require_deadman_topic
            and (self.last_deadman_time == 0.0 or (now - self.last_deadman_time) > self.deadman_timeout_s)
        )
        if self.require_deadman_topic and (not self.deadman_active or deadman_expired):
            state = "INHIBITED_DEADMAN"
            reason = "Deadman switch not pressed or heartbeat timed out"
            self.pub_cmd_vel_safe.publish(out_twist)
            self.publish_status(state, active_source, reason, now)
            return

        # 4. Command Priority Selection: Manual Teleop > Nav2
        teleop_fresh = (
            self.last_teleop_cmd is not None
            and (now - self.last_teleop_time) <= self.cmd_timeout_s
        )
        nav2_fresh = (
            self.allow_nav2
            and self.last_nav2_cmd is not None
            and (now - self.last_nav2_time) <= self.cmd_timeout_s
        )

        desired_twist = None
        if teleop_fresh:
            desired_twist = self.last_teleop_cmd
            active_source = "TELEOP"
        elif nav2_fresh:
            desired_twist = self.last_nav2_cmd
            active_source = "NAV2"

        # If a command is active, verify telemetry freshness before forwarding
        if desired_twist is not None:
            is_moving_cmd = (
                abs(desired_twist.linear.x) > 1e-4
                or abs(desired_twist.linear.y) > 1e-4
                or abs(desired_twist.angular.z) > 1e-4
            )
            if is_moving_cmd and self.require_telemetry_for_motion:
                telemetry_ok, telemetry_reason = self.check_telemetry_freshness(now)
                if not telemetry_ok:
                    state = "INHIBITED_STALE_TELEMETRY"
                    reason = telemetry_reason
                    self.pub_cmd_vel_safe.publish(out_twist)
                    self.publish_status(state, active_source, reason, now)
                    return

            state = "ACTIVE"
            reason = f"Forwarding {active_source} command"
            out_twist = self.clamp_twist(desired_twist)
        else:
            state = "IDLE"
            reason = "Commands timed out or idle"

        self.pub_cmd_vel_safe.publish(out_twist)
        self.publish_status(state, active_source, reason, now)

    def publish_status(
        self, state: str, source: str, reason: str, now: float, stage: str = "NONE"
    ):
        status = {
            "state": state,
            "active_source": source,
            "reason": reason,
            "is_docked": self.is_docked,
            "latched_fault": self.latched_fault,
            "deadman_active": self.deadman_active,
            "recovery_stage": stage,
            "escape_direction": self.escape_direction if self.recovery_stage != RecoveryStage.IDLE else "none",
            "consecutive_bumps": self.consecutive_bumps,
            "odom_age_s": round(now - self.last_odom_recv, 3) if self.last_odom_recv > 0 else None,
            "tf_age_s": round(now - self.last_dynamic_tf_recv, 3) if self.last_dynamic_tf_recv > 0 else None,
            "scan_age_s": round(now - self.last_scan_recv, 3) if self.last_scan_recv > 0 else None,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyAuthorityGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
