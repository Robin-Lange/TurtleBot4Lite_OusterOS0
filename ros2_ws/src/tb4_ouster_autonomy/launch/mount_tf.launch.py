"""Publish measured Create 3 mount transforms for RPLIDAR S2 and RealSense D455.

Publishes:
- base_link -> laser (Slamtec RPLIDAR S2 mounted on top plate)
- base_link -> camera_link (Intel RealSense D455 mounted on top plate)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    laser_x = LaunchConfiguration('laser_x', default='-0.010')
    laser_y = LaunchConfiguration('laser_y', default='0.005')
    laser_z = LaunchConfiguration('laser_z', default='0.118')
    laser_yaw = LaunchConfiguration('laser_yaw', default='0.0')

    camera_x = LaunchConfiguration('camera_x', default='-0.050')
    camera_y = LaunchConfiguration('camera_y', default='-0.005')
    camera_z = LaunchConfiguration('camera_z', default='0.176')
    camera_yaw = LaunchConfiguration('camera_yaw', default='0.0')

    return LaunchDescription([
        DeclareLaunchArgument('laser_x', default_value='-0.010', description='LiDAR X offset from base_link (m)'),
        DeclareLaunchArgument('laser_y', default_value='0.005', description='LiDAR Y offset from base_link (m)'),
        DeclareLaunchArgument('laser_z', default_value='0.118', description='LiDAR Z offset from base_link (m)'),
        DeclareLaunchArgument('laser_yaw', default_value='0.0', description='LiDAR Yaw offset (rad)'),

        DeclareLaunchArgument('camera_x', default_value='-0.050', description='Camera X offset from base_link (m)'),
        DeclareLaunchArgument('camera_y', default_value='-0.005', description='Camera Y offset from base_link (m)'),
        DeclareLaunchArgument('camera_z', default_value='0.176', description='Camera Z offset from base_link (m)'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0', description='Camera Yaw offset (rad)'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=[
                '--x', laser_x, '--y', laser_y, '--z', laser_z,
                '--roll', '0', '--pitch', '0', '--yaw', laser_yaw,
                '--frame-id', 'base_link', '--child-frame-id', 'laser',
            ],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_link',
            arguments=[
                '--x', camera_x, '--y', camera_y, '--z', camera_z,
                '--roll', '0', '--pitch', '0', '--yaw', camera_yaw,
                '--frame-id', 'base_link', '--child-frame-id', 'camera_link',
            ],
            output='screen',
        ),
    ])

