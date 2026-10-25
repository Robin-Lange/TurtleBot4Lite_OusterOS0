# flake8: noqa: E402
"""Exercise the real ROS adapter callbacks and pure policy together, without hardware."""

import time

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
from std_srvs.srv import Trigger

from tb4_ouster_autonomy.components import Sample
from tb4_ouster_autonomy.safety_authority_gate import SafetyAuthorityGate


@pytest.fixture
def gate():
    rclpy.init()
    node = SafetyAuthorityGate()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_contact_latches_without_automatic_reverse(gate):
    msg = HazardDetectionVector()
    hazard = HazardDetection()
    hazard.type = HazardDetection.BUMP
    msg.detections.append(hazard)
    gate.hazard_callback(msg)
    assert gate.world.authority.fault
    gate.control_loop()
    assert gate.world.commands == {}


def test_manual_nan_in_unused_axis_is_rejected_and_pauses_autonomy(gate):
    msg = Twist()
    msg.linear.x = 0.1
    msg.linear.y = float('nan')
    gate.command_callback('TELEOP', msg)
    assert gate.world.authority.autonomy_paused
    assert gate.world.commands['TELEOP'].velocity.linear != gate.world.commands['TELEOP'].velocity.linear


def test_resume_requires_current_telemetry_and_discards_old_commands(gate):
    response = gate.resume_service_callback(Trigger.Request(), Trigger.Response())
    assert not response.success
    gate.world.authority.docked = False
    gate.world.authority.fault = 'Previous contact'
    gate.command_callback('TELEOP', Twist())
    stamp = gate.get_clock().now().nanoseconds * 1e-9
    gate.world.samples = {name: Sample(time.monotonic(), stamp)
                          for name in ('dock', 'hazard', 'odom', 'tf', 'scan', 'cloud')}
    response = gate.resume_service_callback(Trigger.Request(), Trigger.Response())
    assert response.success
    assert not gate.world.authority.autonomy_paused
    assert gate.world.commands == {}
