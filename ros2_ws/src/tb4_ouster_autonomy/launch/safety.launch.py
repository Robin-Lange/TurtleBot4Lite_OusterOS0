"""Launch the safety path and collision monitor.

Launches the narrow authority/freshness gate and nav2_collision_monitor.
Collision monitor is the ONLY external writer to Create 3 /cmd_vel.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('tb4_ouster_autonomy')
    collision_params = os.path.join(pkg_share, 'config', 'collision_monitor_params.yaml')

    allow_nav2_arg = DeclareLaunchArgument(
        'allow_nav2',
        default_value='false',
        description='Whether to allow Nav2 autonomous velocity proposals',
    )

    safety_gate_node = Node(
        package='tb4_ouster_autonomy',
        executable='safety_authority_gate',
        name='safety_authority_gate',
        output='screen',
        parameters=[{
            'publish_rate_hz': 10.0,
            'cmd_timeout_s': 1.0,
            'odom_timeout_s': 1.0,
            'tf_timeout_s': 1.0,
            'scan_timeout_s': 1.0,
            'require_deadman_topic': False,
            'allow_nav2': LaunchConfiguration('allow_nav2'),
            'max_linear_speed': 0.15,
            'max_angular_speed': 0.50,
            'bump_backup_speed': 0.06,
            'bump_backup_duration_s': 0.50,
            'bump_rotate_speed': 0.40,
            'bump_rotate_duration_s': 1.20,
            'bump_center_rotate_duration_s': 2.00,
            'bump_settle_duration_s': 0.20,
            'max_consecutive_bumps': 4,
            'bump_history_window_s': 10.0,
        }],
    )

    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[collision_params],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_safety',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['collision_monitor'],
        }],
    )

    return LaunchDescription([
        allow_nav2_arg,
        safety_gate_node,
        collision_monitor_node,
        lifecycle_manager_node,
    ])
