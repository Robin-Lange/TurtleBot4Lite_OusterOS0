"""Optional observation-only monitor; start the sensor pipeline separately."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='tb4_ouster_autonomy', executable='visibility_monitor',
            parameters=[{
                'scan_topic': LaunchConfiguration('scan_topic'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }], output='screen',
        ),
    ])
