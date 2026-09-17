"""Launch mapping stack (perception, safety path, and SLAM Toolbox).

Uses asynchronous SLAM with 2D scan generated from the Ouster cloud.
All manual teleop commands must be published to /teleop/cmd_vel and will
be routed through the safety authority gate and collision monitor.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
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

    safety_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'safety.launch.py')
        )
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
    )

    return LaunchDescription([
        perception_launch,
        safety_launch,
        slam_node,
    ])
