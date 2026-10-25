"""Launch mapping stack: perception + SLAM Toolbox.

Uses asynchronous SLAM with the 2D scan from Slamtec RPLIDAR S2 and
fuses 3D obstacle avoidance from Intel RealSense D455.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('tb4_ouster_autonomy')
    slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'perception.launch.py')
        )
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
    )

    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'safety.launch.py')),
        condition=IfCondition(LaunchConfiguration('enable_safety')),
    )
    return LaunchDescription([
        DeclareLaunchArgument('enable_safety', default_value='true'),
        safety,
        perception_launch,
        slam_node,
    ])
