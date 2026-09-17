"""Publish the provisional, measured Create 3-to-OS0 mount transform.

Assumes the plate logo is directly above the Create 3 rotation center,
the sensor is level, and its cable points toward the robot rear.
Also publishes the optical origin frame laser_frame at +38.195 mm above
os_sensor with zero rotation (horizontal axes aligned with base_link).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_os_sensor',
            arguments=[
                '--x', '0.005', '--y', '-0.010', '--z', '0.133',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link', '--child-frame-id', 'os_sensor',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='os_sensor_to_laser_frame',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.038195',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'os_sensor', '--child-frame-id', 'laser_frame',
            ],
        ),
    ])
