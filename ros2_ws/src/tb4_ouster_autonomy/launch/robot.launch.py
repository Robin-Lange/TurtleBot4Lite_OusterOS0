"""Unified robot bringup entry point supporting diagnose, map, and navigate modes.

Usage:
  ros2 launch tb4_ouster_autonomy robot.launch.py mode:=diagnose (default)
  ros2 launch tb4_ouster_autonomy robot.launch.py mode:=map rviz:=true
  ros2 launch tb4_ouster_autonomy robot.launch.py mode:=navigate rviz:=true
  ros2 launch tb4_ouster_autonomy robot.launch.py mode:=navigate map:=/path/to/map.yaml rviz:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
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
            'map (manual SLAM), or navigate (waypoint navigation)'
        ),
        choices=['diagnose', 'map', 'navigate'],
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_share, 'maps', 'arena_map.yaml'),
        description='Full path to map yaml (navigate mode only)',
    )

    start_sensor_arg = DeclareLaunchArgument(
        'start_sensor',
        default_value='false',
        description='Whether to start the RPLIDAR S2 and RealSense D455 drivers',
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to launch RViz2',
    )

    mode = LaunchConfiguration('mode')

    is_diagnose = PythonExpression(["'", mode, "' == 'diagnose'"])
    is_map = PythonExpression(["'", mode, "' == 'map'"])
    is_navigate = PythonExpression(["'", mode, "' == 'navigate'"])
    is_map_or_navigate = PythonExpression([
        "'", mode, "' == 'map' or '", mode, "' == 'navigate'"
    ])

    # 1. Diagnose mode: read-only perception TF + stationary diagnostics
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'perception.launch.py')
        ),
        launch_arguments={
            'start_lidar': LaunchConfiguration('start_sensor'),
            'start_camera': LaunchConfiguration('start_sensor'),
        }.items(),
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
        msg='Starting in DIAGNOSE mode — stationary diagnostics and TF active, no velocity output.',
        condition=IfCondition(is_diagnose),
    )

    # 2. Map mode: perception + slam_toolbox
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'mapping.launch.py')
        ),
        condition=IfCondition(is_map),
    )

    map_info = LogInfo(
        msg='Starting in MAP mode — drive with teleop_twist_keyboard, save map with map_saver_cli.',
        condition=IfCondition(is_map),
    )

    # 3. Navigate mode: full Nav2 stack with loaded map and 3D VoxelLayer costmap
    navigate_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'navigate.launch.py')
        ),
        launch_arguments={'map': LaunchConfiguration('map')}.items(),
        condition=IfCondition(is_navigate),
    )

    navigate_info = LogInfo(
        msg='Starting in NAVIGATE mode — set 2D Pose Estimate in RViz, then send Nav2 Goals.',
        condition=IfCondition(is_navigate),
    )

    # 4. RViz — different config per mode
    rviz_map_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        arguments=[
            '-d', os.path.join(pkg_share, 'rviz', 'view_robot.rviz'),
            '-f', 'map',
        ],
        condition=IfCondition(PythonExpression([
            "True if ('", LaunchConfiguration('rviz'), "' == 'true' and '", mode, "' == 'map') else False"
        ])),
    )

    rviz_navigate_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'navigate.rviz')],
        condition=IfCondition(PythonExpression([
            "True if ('", LaunchConfiguration('rviz'), "' == 'true' and '", mode, "' == 'navigate') else False"
        ])),
    )

    rviz_diagnose_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'view_robot.rviz')],
        condition=IfCondition(PythonExpression([
            "True if ('", LaunchConfiguration('rviz'), "' == 'true' and '", mode, "' == 'diagnose') else False"
        ])),
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            '/home/ivlaborin2/TurtleBot4Lite_OusterOS0/config/fastdds_create3.xml',
        ),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        mode_arg,
        map_arg,
        start_sensor_arg,
        rviz_arg,
        # diagnose
        diagnostics_info,
        diagnostics_node,
        perception_launch,
        # map
        map_info,
        mapping_launch,
        # navigate
        navigate_info,
        navigate_launch,
        # rviz
        rviz_diagnose_node,
        rviz_map_node,
        rviz_navigate_node,
    ])
