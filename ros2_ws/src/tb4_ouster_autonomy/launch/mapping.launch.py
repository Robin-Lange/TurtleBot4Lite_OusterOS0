"""Launch mapping stack: perception + SLAM Toolbox only.

Uses asynchronous SLAM with the 2D scan ring from the Ouster cloud.
Teleop publishes directly to /cmd_vel (no safety gate or collision monitor).
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

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
    )

    return LaunchDescription([
        perception_launch,
        slam_node,
    ])
