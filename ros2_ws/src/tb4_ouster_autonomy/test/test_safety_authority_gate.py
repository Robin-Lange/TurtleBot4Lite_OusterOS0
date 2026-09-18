#!/usr/bin/env python3
"""Offline unit and integration tests for SafetyAuthorityGate.

Runs on an isolated ROS domain with mock messages. Validates command selection,
deadman release, source timeout, telemetry freshness watchdogs, dock inhibition,
hazard fault latching, and explicit resume behavior per TODO.md Phase 1b gate.
"""

import json
import os
import time
from typing import Optional
import unittest

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from irobot_create_msgs.msg import DockStatus, HazardDetectionVector, HazardDetection

from tb4_ouster_autonomy.safety_authority_gate import SafetyAuthorityGate


class MockEnvironment(Node):
    """Publishes mock inputs and records safety gate outputs."""

    def __init__(self):
        super().__init__("mock_test_environment")
        self.pub_teleop = self.create_publisher(Twist, "/teleop/cmd_vel", 10)
        self.pub_nav2 = self.create_publisher(Twist, "/nav2/cmd_vel", 10)
        self.pub_deadman = self.create_publisher(Bool, "/teleop/deadman", 10)
        self.pub_op_stop = self.create_publisher(Bool, "/operator/stop", 10)
        self.pub_dock = self.create_publisher(DockStatus, "/dock_status", 10)
        self.pub_hazard = self.create_publisher(HazardDetectionVector, "/hazard_detection", 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_tf = self.create_publisher(TFMessage, "/tf", 10)
        self.pub_scan = self.create_publisher(LaserScan, "/ouster/scan", 10)

        self.last_cmd_safe: Twist = Twist()
        self.last_status: dict = {}
        self.last_bumper_cloud: Optional[PointCloud2] = None

        self.sub_cmd_safe = self.create_subscription(
            Twist, "cmd_vel_safe", self.cmd_safe_cb, 10
        )
        self.sub_status = self.create_subscription(
            String, "/safety/status", self.status_cb, 10
        )
        self.sub_bumper_cloud = self.create_subscription(
            PointCloud2, "/safety/bumper_contact_cloud", self.bumper_cloud_cb, 10
        )

        self.cli_resume = self.create_client(Trigger, "/safety/resume")
        self.cli_estop = self.create_client(Trigger, "/safety/emergency_stop")

    def cmd_safe_cb(self, msg: Twist):
        self.last_cmd_safe = msg

    def status_cb(self, msg: String):
        try:
            self.last_status = json.loads(msg.data)
        except Exception:
            pass

    def bumper_cloud_cb(self, msg: PointCloud2):
        self.last_bumper_cloud = msg

    def send_healthy_telemetry(self):
        now_stamp = self.get_clock().now().to_msg()

        # Odom
        odom = Odometry()
        odom.header.stamp = now_stamp
        odom.header.frame_id = "odom"
        self.pub_odom.publish(odom)

        # Dynamic TF
        tf_msg = TFMessage()
        t = TransformStamped()
        t.header.stamp = now_stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        tf_msg.transforms.append(t)
        self.pub_tf.publish(tf_msg)

        # Scan
        scan = LaserScan()
        scan.header.stamp = now_stamp
        scan.header.frame_id = "laser_frame"
        self.pub_scan.publish(scan)

        # Deadman active
        deadman = Bool()
        deadman.data = True
        self.pub_deadman.publish(deadman)

        # Undocked
        dock = DockStatus()
        dock.header.stamp = now_stamp
        dock.is_docked = False
        self.pub_dock.publish(dock)

        # No hazards
        hazard = HazardDetectionVector()
        hazard.header.stamp = now_stamp
        self.pub_hazard.publish(hazard)


class TestSafetyAuthorityGate(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Isolate ROS domain for testing
        os.environ["ROS_DOMAIN_ID"] = "88"
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.gate = SafetyAuthorityGate()
        self.mock = MockEnvironment()
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.gate)
        self.executor.add_node(self.mock)

    def tearDown(self):
        self.executor.remove_node(self.gate)
        self.executor.remove_node(self.mock)
        self.gate.destroy_node()
        self.mock.destroy_node()

    def spin_for(self, duration_s: float):
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)

    def test_01_idle_publishes_zero(self):
        """Initial state should be docked/idle with zero output."""
        self.spin_for(0.3)
        self.assertEqual(self.mock.last_cmd_safe.linear.x, 0.0)
        self.assertEqual(self.mock.last_cmd_safe.angular.z, 0.0)
        self.assertTrue(self.mock.last_status.get("is_docked", True))

    def test_02_docked_inhibits_motion(self):
        """When docked, motion commands must be blocked (output zero)."""
        # Send fresh telemetry but is_docked = True
        self.mock.send_healthy_telemetry()
        dock = DockStatus()
        dock.is_docked = True
        self.mock.pub_dock.publish(dock)

        # Try to command motion
        cmd = Twist()
        cmd.linear.x = 0.10
        self.mock.pub_teleop.publish(cmd)
        self.spin_for(0.3)

        self.assertEqual(self.mock.last_cmd_safe.linear.x, 0.0)
        self.assertEqual(self.mock.last_status.get("state"), "INHIBITED_DOCKED")

    def test_03_teleop_forwarding_and_clamping(self):
        """When undocked and healthy, teleop command forwards clamped to safe caps."""
        self.mock.send_healthy_telemetry()
        cmd = Twist()
        cmd.linear.x = 0.50  # Above 0.15 limit
        cmd.angular.z = 1.0  # Above 0.50 limit

        # Send command repeatedly for 0.3s
        for _ in range(6):
            self.mock.send_healthy_telemetry()
            self.mock.pub_teleop.publish(cmd)
            self.spin_for(0.05)

        self.assertEqual(self.mock.last_status.get("state"), "ACTIVE")
        self.assertAlmostEqual(self.mock.last_cmd_safe.linear.x, 0.15, places=2)
        self.assertAlmostEqual(self.mock.last_cmd_safe.angular.z, 0.50, places=2)

    def test_03b_deadman_release_stops_motion(self):
        """Releasing deadman switch immediately zeroes command and reports INHIBITED_DEADMAN."""
        self.mock.send_healthy_telemetry()
        cmd = Twist()
        cmd.linear.x = 0.10

        # Send command with deadman False
        deadman = Bool()
        deadman.data = False
        for _ in range(6):
            self.mock.send_healthy_telemetry()
            self.mock.pub_deadman.publish(deadman)
            self.mock.pub_teleop.publish(cmd)
            self.spin_for(0.05)

        self.assertEqual(self.mock.last_cmd_safe.linear.x, 0.0)
        self.assertEqual(self.mock.last_status.get("state"), "INHIBITED_DEADMAN")

    def test_04_source_timeout_stops_motion(self):
        """Stopping command stream causes velocity to drop to 0 within timeout."""
        self.mock.send_healthy_telemetry()
        cmd = Twist()
        cmd.linear.x = 0.10
        self.mock.pub_teleop.publish(cmd)
        self.spin_for(0.05)

        # Stop publishing teleop, keep telemetry fresh
        for _ in range(7):
            self.mock.send_healthy_telemetry()
            self.spin_for(0.05)

        # cmd_timeout_s is 0.20s; after ~0.35s command must be IDLE / 0.0
        self.assertEqual(self.mock.last_cmd_safe.linear.x, 0.0)
        self.assertEqual(self.mock.last_status.get("state"), "IDLE")

    def test_05_stale_telemetry_inhibits_motion(self):
        """Missing or stale odom/scan stops motion immediately."""
        # Only publish dock=undocked, no odom or scan
        dock = DockStatus()
        dock.is_docked = False
        self.mock.pub_dock.publish(dock)

        cmd = Twist()
        cmd.linear.x = 0.10
        deadman = Bool()
        deadman.data = True
        for _ in range(6):
            self.mock.pub_dock.publish(dock)
            self.mock.pub_deadman.publish(deadman)
            self.mock.pub_teleop.publish(cmd)
            self.spin_for(0.05)

        self.assertEqual(self.mock.last_cmd_safe.linear.x, 0.0)
        self.assertEqual(self.mock.last_status.get("state"), "INHIBITED_STALE_TELEMETRY")

    def test_06_hazard_fault_latching_and_resume(self):
        """Cliff/wheel-drop hazard latches fault; recovery requires /safety/resume."""
        self.mock.send_healthy_telemetry()

        # Inject critical hazard (Cliff)
        hazard = HazardDetectionVector()
        h = HazardDetection()
        h.type = HazardDetection.CLIFF
        hazard.detections.append(h)
        self.mock.pub_hazard.publish(hazard)
        self.spin_for(0.1)

        self.assertTrue(self.mock.last_status.get("latched_fault"))
        self.assertEqual(self.mock.last_status.get("state"), "LATCHED_FAULT")
        self.assertEqual(self.mock.last_cmd_safe.linear.x, 0.0)

        # Clear hazard on telemetry topic
        self.mock.send_healthy_telemetry()
        cmd = Twist()
        cmd.linear.x = 0.10
        self.mock.pub_teleop.publish(cmd)
        self.spin_for(0.2)

        # It must STILL be latched!
        self.assertTrue(self.mock.last_status.get("latched_fault"))
        self.assertEqual(self.mock.last_cmd_safe.linear.x, 0.0)

        # Call resume service
        req = Trigger.Request()
        future = self.mock.cli_resume.call_async(req)
        while not future.done():
            self.executor.spin_once(timeout_sec=0.05)

        self.assertTrue(future.result().success)
        self.spin_for(0.1)
        self.assertFalse(self.mock.last_status.get("latched_fault"))

    def test_06b_bumper_contact_multi_stage_recovery(self):
        """Bumper contact executes Stage 1 (BACKUP) then Stage 2 (ROTATE) away from obstacle."""
        self.mock.send_healthy_telemetry()

        # 1. Bump on right side -> Stage 1 (BACKUP: negative linear.x, zero angular.z)
        hazard_right = HazardDetectionVector()
        hr = HazardDetection()
        hr.type = HazardDetection.BUMP
        hr.header.frame_id = "bump_front_right"
        hazard_right.detections.append(hr)
        self.mock.pub_hazard.publish(hazard_right)
        self.spin_for(0.1)

        self.assertFalse(self.mock.last_status.get("latched_fault"))
        self.assertEqual(self.mock.last_status.get("state"), "BUMP_RECOVERY")
        self.assertEqual(self.mock.last_status.get("active_source"), "BUMP_REFLEX")
        self.assertEqual(self.mock.last_status.get("recovery_stage"), "BACKUP")
        self.assertLess(self.mock.last_cmd_safe.linear.x, 0.0)  # Reversing
        self.assertEqual(self.mock.last_cmd_safe.angular.z, 0.0)
        self.assertIsNotNone(self.mock.last_bumper_cloud)  # Synthetic cloud published

        # 2. Advance beyond backup duration (0.5s) -> Stage 2 (ROTATE: positive angular.z, turning left)
        self.mock.send_healthy_telemetry()
        self.spin_for(0.55)

        self.assertEqual(self.mock.last_status.get("recovery_stage"), "ROTATE")
        self.assertEqual(self.mock.last_cmd_safe.linear.x, 0.0)
        self.assertGreater(self.mock.last_cmd_safe.angular.z, 0.0)  # Turning left away from right obstacle

    def test_06c_bumper_contact_left_turns_right(self):
        """Left bumper contact routes right in ROTATE stage (negative angular.z)."""
        self.mock.send_healthy_telemetry()

        hazard_left = HazardDetectionVector()
        hl = HazardDetection()
        hl.type = HazardDetection.BUMP
        hl.header.frame_id = "bump_front_left"
        hazard_left.detections.append(hl)
        self.mock.pub_hazard.publish(hazard_left)
        self.spin_for(0.1)

        # Stage 1: BACKUP
        self.assertEqual(self.mock.last_status.get("recovery_stage"), "BACKUP")
        self.assertLess(self.mock.last_cmd_safe.linear.x, 0.0)

        # Advance to Stage 2: ROTATE
        self.mock.send_healthy_telemetry()
        self.spin_for(0.55)
        self.assertEqual(self.mock.last_status.get("recovery_stage"), "ROTATE")
        self.assertLess(self.mock.last_cmd_safe.angular.z, 0.0)  # Turning right away from left obstacle

    def test_06d_backup_limit_aborts_backup_to_rotate(self):
        """BACKUP_LIMIT hazard while reversing immediately transitions from BACKUP to ROTATE."""
        self.mock.send_healthy_telemetry()

        # Trigger right bumper
        hazard = HazardDetectionVector()
        h = HazardDetection()
        h.type = HazardDetection.BUMP
        h.header.frame_id = "bump_front_right"
        hazard.detections.append(h)
        self.mock.pub_hazard.publish(hazard)
        self.spin_for(0.1)
        self.assertEqual(self.mock.last_status.get("recovery_stage"), "BACKUP")

        # Now trigger BACKUP_LIMIT (type 0)
        hazard_bl = HazardDetectionVector()
        h_bl = HazardDetection()
        h_bl.type = HazardDetection.BACKUP_LIMIT
        hazard_bl.detections.append(h_bl)
        self.mock.pub_hazard.publish(hazard_bl)
        self.spin_for(0.1)

        # Must immediately be in ROTATE stage without waiting for 0.5s timer
        self.assertEqual(self.mock.last_status.get("recovery_stage"), "ROTATE")
        self.assertGreater(self.mock.last_cmd_safe.angular.z, 0.0)

    def test_06e_stall_triggers_recovery(self):
        """Wheel stall (STALL, type 3) triggers escape recovery."""
        self.mock.send_healthy_telemetry()

        hazard = HazardDetectionVector()
        h = HazardDetection()
        h.type = HazardDetection.STALL
        hazard.detections.append(h)
        self.mock.pub_hazard.publish(hazard)
        self.spin_for(0.1)

        self.assertFalse(self.mock.last_status.get("latched_fault"))
        self.assertEqual(self.mock.last_status.get("state"), "BUMP_RECOVERY")
        self.assertEqual(self.mock.last_status.get("recovery_stage"), "BACKUP")

    def test_06f_trapped_consecutive_bumps_latches_fault(self):
        """Exceeding max_consecutive_bumps (4) latches fault to protect motors from endless traps."""
        self.mock.send_healthy_telemetry()

        # Inject 4 consecutive bumps with release between them
        for _ in range(4):
            hazard = HazardDetectionVector()
            h = HazardDetection()
            h.type = HazardDetection.BUMP
            h.header.frame_id = "bump_front_center"
            hazard.detections.append(h)
            self.mock.pub_hazard.publish(hazard)
            self.spin_for(0.05)

            # Release contact
            hazard_clear = HazardDetectionVector()
            self.mock.pub_hazard.publish(hazard_clear)
            self.spin_for(0.05)

        self.assertTrue(self.mock.last_status.get("latched_fault"))
        self.assertEqual(self.mock.last_status.get("state"), "LATCHED_FAULT")
        self.assertIn("trapped", self.mock.last_status.get("reason").lower())
        self.assertEqual(self.mock.last_cmd_safe.linear.x, 0.0)
        self.assertEqual(self.mock.last_cmd_safe.angular.z, 0.0)

    def test_07_estop_service_latches_fault(self):
        """Calling /safety/emergency_stop latches fault."""
        self.mock.send_healthy_telemetry()
        req = Trigger.Request()
        future = self.mock.cli_estop.call_async(req)
        while not future.done():
            self.executor.spin_once(timeout_sec=0.05)

        self.assertTrue(future.result().success)
        self.spin_for(0.1)
        self.assertTrue(self.mock.last_status.get("latched_fault"))
        self.assertEqual(self.mock.last_status.get("state"), "LATCHED_FAULT")

    def test_08_nav2_inhibited_when_allow_nav2_false(self):
        """Nav2 commands are ignored when allow_nav2 is False (Phase 1b safety)."""
        self.mock.send_healthy_telemetry()
        cmd = Twist()
        cmd.linear.x = 0.10
        for _ in range(6):
            self.mock.send_healthy_telemetry()
            self.mock.pub_nav2.publish(cmd)
            self.spin_for(0.05)

        # Output must be 0.0 because allow_nav2=False
        self.assertEqual(self.mock.last_cmd_safe.linear.x, 0.0)
        self.assertNotEqual(self.mock.last_status.get("active_source"), "NAV2")

    def test_09_negative_velocity_clamping(self):
        """Negative velocities must clamp symmetrically and linear.y forced to zero."""
        self.mock.send_healthy_telemetry()
        cmd = Twist()
        cmd.linear.x = -0.50  # Below -0.15 limit
        cmd.linear.y = 0.20   # Must be forced to 0.0 (non-holonomic base)
        cmd.angular.z = -1.0  # Below -0.50 limit

        for _ in range(5):
            self.mock.send_healthy_telemetry()
            self.mock.pub_teleop.publish(cmd)
            self.spin_for(0.05)

        self.assertAlmostEqual(self.mock.last_cmd_safe.linear.x, -0.15, places=2)
        self.assertEqual(self.mock.last_cmd_safe.linear.y, 0.0)
        self.assertAlmostEqual(self.mock.last_cmd_safe.angular.z, -0.50, places=2)

    def test_10_operator_stop_topic_latches_fault(self):
        """Publishing True on /operator/stop must latch emergency stop."""
        self.mock.send_healthy_telemetry()
        stop_msg = Bool()
        stop_msg.data = True
        self.mock.pub_op_stop.publish(stop_msg)
        self.spin_for(0.15)

        self.assertTrue(self.mock.last_status.get("latched_fault"))
        self.assertEqual(self.mock.last_status.get("state"), "LATCHED_FAULT")
        self.assertEqual(self.mock.last_cmd_safe.linear.x, 0.0)

    def test_11_resume_rejected_when_docked_or_hazards(self):
        """Resuming must fail if docked or active hazards persist."""
        # Trigger fault via estop service
        req = Trigger.Request()
        fut = self.mock.cli_estop.call_async(req)
        while not fut.done():
            self.executor.spin_once(timeout_sec=0.05)

        # Set robot docked
        dock = DockStatus()
        dock.is_docked = True
        self.mock.pub_dock.publish(dock)
        self.spin_for(0.1)

        # Try to resume while docked -> must fail
        resume_fut = self.mock.cli_resume.call_async(Trigger.Request())
        while not resume_fut.done():
            self.executor.spin_once(timeout_sec=0.05)
        self.assertFalse(resume_fut.result().success)
        self.assertIn("docked", resume_fut.result().message)

    def test_12_deadman_heartbeat_timeout(self):
        """If deadman heartbeat is not refreshed within deadman_timeout_s, motion is blocked."""
        self.mock.send_healthy_telemetry()
        cmd = Twist()
        cmd.linear.x = 0.10
        self.mock.pub_teleop.publish(cmd)
        self.spin_for(0.1)
        self.assertEqual(self.mock.last_status.get("state"), "ACTIVE")

        # Stop publishing deadman and wait > 0.5s for heartbeat timeout
        self.spin_for(0.6)
        self.mock.pub_teleop.publish(cmd)
        self.spin_for(0.1)
        self.assertEqual(self.mock.last_cmd_safe.linear.x, 0.0)
        self.assertEqual(self.mock.last_status.get("state"), "INHIBITED_DEADMAN")


if __name__ == "__main__":
    unittest.main()
