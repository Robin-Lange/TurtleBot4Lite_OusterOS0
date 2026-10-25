# flake8: noqa: E402
"""Real Humble gate + Collision Monitor integration on an isolated ROS domain."""

import subprocess
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import TransformStamped, Twist
from irobot_create_msgs.msg import DockStatus, HazardDetectionVector
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage


@pytest.fixture
def pipeline():
    process = subprocess.Popen([
        'ros2', 'launch', 'tb4_ouster_autonomy', 'safety.launch.py', 'allow_nav2:=true',
    ])
    rclpy.init()
    node = rclpy.create_node('collision_pipeline_test')
    yield node
    node.destroy_node()
    rclpy.shutdown()
    process.terminate()
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)


def test_final_output_stops_for_obstacle_and_camera_dropout(pipeline):
    node = pipeline
    publishers = {
        'odom': node.create_publisher(Odometry, '/odom', 1),
        'tf': node.create_publisher(TFMessage, '/tf', 1),
        'dock': node.create_publisher(DockStatus, '/dock_status', 1),
        'hazard': node.create_publisher(HazardDetectionVector, '/hazard_detection', 1),
        'scan': node.create_publisher(LaserScan, '/scan', 1),
        'cloud': node.create_publisher(PointCloud2, '/camera/cloud_filtered', 1),
        'cmd': node.create_publisher(Twist, '/nav2/cmd_vel', 1),
    }
    received = []
    subscription = node.create_subscription(Twist, '/cmd_vel', lambda msg: received.append(msg.linear.x), 10)

    def drive(seconds, points, camera=True):
        received.clear()
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            stamp = node.get_clock().now().to_msg()
            odom = Odometry()
            odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
            odom.header.stamp = stamp
            odom.pose.pose.orientation.w = 1.
            publishers['odom'].publish(odom)
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = 'odom', 'base_link'
            tf.header.stamp = stamp
            tf.transform.rotation.w = 1.
            message = TFMessage()
            message.transforms.append(tf)
            publishers['tf'].publish(message)
            dock = DockStatus()
            dock.header.stamp, dock.is_docked = stamp, False
            publishers['dock'].publish(dock)
            hazard = HazardDetectionVector()
            hazard.header.stamp = stamp
            publishers['hazard'].publish(hazard)
            scan = LaserScan()
            scan.header.frame_id, scan.header.stamp = 'base_link', stamp
            scan.angle_min, scan.angle_max, scan.angle_increment = -3.14, 3.14, 0.1
            scan.range_min, scan.range_max = 0.05, 30.
            scan.ranges = [float('inf')] * 64
            publishers['scan'].publish(scan)
            if camera:
                header = Header()
                header.frame_id, header.stamp = 'base_link', stamp
                publishers['cloud'].publish(pc2.create_cloud_xyz32(header, points))
            cmd = Twist()
            cmd.linear.x = 0.12
            publishers['cmd'].publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.03)
        return list(received)

    # Wait for lifecycle startup and DDS matching; require actual output, never a default zero.
    values = drive(6.0, [])
    assert any(value > 0 for value in values), 'Healthy proposal never reached final /cmd_vel'
    values = drive(1.5, [[0.22, 0, 0.12]])
    assert values and all(value == 0 for value in values[-5:]), 'Obstacle did not stop final output'
    values = drive(1.5, [])
    assert any(value > 0 for value in values[-5:]), 'Cleared observation did not release stop'
    values = drive(1.5, [], camera=False)
    assert values and all(value == 0 for value in values[-5:]), 'Camera dropout did not stop final output'
    node.destroy_subscription(subscription)
