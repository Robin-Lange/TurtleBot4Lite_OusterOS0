# flake8: noqa: E402
"""Action cancellation and undocking regressions using real ROS message adapters."""

from types import SimpleNamespace
from unittest.mock import Mock
import time
import subprocess

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import TransformStamped

from tb4_ouster_autonomy.autonomous_explorer import AutonomousExplorer


@pytest.fixture
def explorer():
    rclpy.init()
    node = AutonomousExplorer()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_undock_timeout_finishes_instead_of_spinning(explorer):
    explorer.phase = 'UNDOCKING'
    explorer.phase_start_time = time.monotonic() - 21
    explorer.is_docked = True
    explorer.undock_requested = True
    explorer.control_step()
    assert explorer.phase == 'FINISHED'
    assert not explorer.undock_succeeded


def test_unknown_dock_state_is_never_assumed_undocked(explorer):
    explorer.scan_received = explorer.odom_received = True
    explorer.phase_start_time = time.monotonic() - 10
    explorer.control_step()
    assert explorer.phase == 'WAIT_SENSORS'


def test_late_accepted_goal_is_cancelled_after_stop(explorer):
    explorer.nav_goal_active = True
    old_epoch = explorer.goal_epoch
    explorer.stop_robot()
    handle = Mock(accepted=True)
    future = Mock()
    future.result.return_value = handle
    explorer._goal_response_callback(future, old_epoch)
    handle.cancel_goal_async.assert_called_once()
    assert explorer.cancel_pending


def test_late_rejected_goal_releases_cancel_wait(explorer):
    explorer.nav_goal_active = True
    old_epoch = explorer.goal_epoch
    explorer.stop_robot()
    future = Mock()
    future.result.return_value = Mock(accepted=False)
    explorer._goal_response_callback(future, old_epoch)
    assert not explorer.cancel_pending


def test_undock_acceptance_after_finish_is_cancelled(explorer):
    explorer.exploration_finished = True
    handle = Mock(accepted=True)
    future = Mock()
    future.result.return_value = handle
    explorer.undock_response(future)
    handle.cancel_goal_async.assert_called_once()


def test_map_pose_uses_map_tf_instead_of_odometry(explorer):
    transform = TransformStamped()
    transform.header.stamp = explorer.get_clock().now().to_msg()
    transform.transform.translation.x = 4.0
    transform.transform.translation.y = 7.0
    transform.transform.rotation.w = 1.0
    explorer.buffer.lookup_transform = Mock(return_value=transform)
    assert explorer.update_map_pose()
    assert (explorer.robot_x, explorer.robot_y) == (4.0, 7.0)
    assert explorer.buffer.lookup_transform.call_args.args[:2] == ('map', 'base_link')


def test_finished_mission_keeps_stop_proposal_fresh(explorer):
    explorer.exploration_finished = True
    explorer.pub_cmd_vel = Mock()
    explorer.control_step()
    explorer.pub_cmd_vel.publish.assert_called_once()
    assert explorer.pub_cmd_vel.publish.call_args.args[0].linear.x == 0


def test_undock_success_alone_does_not_override_docked_state(explorer):
    explorer.phase = 'UNDOCKING'
    explorer.is_docked = True
    explorer.undock_succeeded = True
    explorer.undock_requested = True
    explorer.phase_start_time = time.monotonic()
    explorer.control_step()
    assert explorer.phase == 'UNDOCKING'


def test_undock_failed_result_finishes(explorer):
    future = Mock()
    future.result.return_value = SimpleNamespace(status=6)
    explorer.undock_result(future)
    assert explorer.phase == 'FINISHED'


@pytest.mark.parametrize('mode', ['diagnose', 'map', 'auto_map', 'navigate'])
def test_launch_arguments_resolve_for_every_mode(mode):
    result = subprocess.run(
        ['ros2', 'launch', 'tb4_ouster_autonomy', 'robot.launch.py', f'mode:={mode}', '--show-args'],
        capture_output=True, text=True, timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
