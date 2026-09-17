"""Unified robot bringup entry point supporting diagnose, map, and navigate modes.

Usage:
  ros2 launch tb4_ouster_autonomy robot.launch.py mode:=diagnose (default)
  ros2 launch tb4_ouster_autonomy robot.launch.py mode:=map
  ros2 launch tb4_ouster_autonomy robot.launch.py mode:=navigate map:=/path/to/map.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('tb4_ouster_autonomy')

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='diagnose',
        description=(
            'Operating mode: diagnose (stationary, no velocity), '
            'map (manual SLAM), or navigate (goal navigation)'
        ),
        choices=['diagnose', 'map', 'navigate'],
    )

    start_sensor_arg = DeclareLaunchArgument(
        'start_sensor',
        default_value='false',
        description='Whether to start the ouster_ros sensor driver (may reinitialize sensor)',
    )

    mode = LaunchConfiguration('mode')

    is_diagnose = PythonExpression(["'", mode, "' == 'diagnose'"])
    is_map = PythonExpression(["'", mode, "' == 'map'"])
    is_navigate = PythonExpression(["'", mode, "' == 'navigate'"])

    # 1. Diagnose mode: read-only perception TF + stationary diagnostics
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'perception.launch.py')
        ),
        launch_arguments={'start_driver': LaunchConfiguration('start_sensor')}.items(),
        condition=IfCondition(is_diagnose),
    )

    diagnostics_node = Node(
        package='tb4_ouster_autonomy',
        executable='stationary_diagnostics',
        name='stationary_diagnostics',
        output='screen',
        condition=IfCondition(is_diagnose),
    )

    diagnostics_info = LogInfo(
        msg="Starting tb4_ouster_autonomy in DIAGNOSE mode. Stationary diagnostics and TF active, no velocity output.",
        condition=IfCondition(is_diagnose),
    )

    # 2. Map mode: perception + safety + slam_toolbox
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'mapping.launch.py')
        ),
        condition=IfCondition(is_map),
    )

    map_info = LogInfo(
        msg="Starting tb4_ouster_autonomy in MAP mode. Teleop commands must publish to /teleop/cmd_vel.",
        condition=IfCondition(is_map),
    )

    # 3. Navigate mode: perception + safety + nav2
    navigate_safety_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'safety.launch.py')
        ),
        condition=IfCondition(is_navigate),
    )
    navigate_perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'perception.launch.py')
        ),
        condition=IfCondition(is_navigate),
    )
    navigate_info = LogInfo(
        msg="Starting tb4_ouster_autonomy in NAVIGATE mode.",
        condition=IfCondition(is_navigate),
    )

    # 4. Optional RViz visualization
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to launch RViz2 with view_robot.rviz config',
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'view_robot.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        mode_arg,
        start_sensor_arg,
        rviz_arg,
        diagnostics_info,
        diagnostics_node,
        perception_launch,
        map_info,
        mapping_launch,
        navigate_info,
        navigate_perception_launch,
        navigate_safety_launch,
        rviz_node,
    ])
