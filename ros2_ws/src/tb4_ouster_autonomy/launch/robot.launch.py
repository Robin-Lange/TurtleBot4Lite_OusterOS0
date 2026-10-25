"""Unified robot bringup entry point supporting diagnose, map, auto_map, and navigate modes.

Usage:
  ros2 launch tb4_ouster_autonomy robot.launch.py mode:=diagnose (default)
  ros2 launch tb4_ouster_autonomy robot.launch.py mode:=auto_map rviz:=false
  ros2 launch tb4_ouster_autonomy robot.launch.py mode:=map rviz:=true
  ros2 launch tb4_ouster_autonomy robot.launch.py mode:=navigate rviz:=false
  ros2 launch tb4_ouster_autonomy robot.launch.py mode:=navigate map:=/path/to/map.yaml rviz:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
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
            'map (manual SLAM), auto_map (autonomous frontier SLAM + auto-save), '
            'or navigate (waypoint navigation)'
        ),
        choices=['diagnose', 'map', 'auto_map', 'navigate'],
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('~/maps/arena_map.yaml'),
        description='Full path to map yaml (navigate mode only)',
    )

    map_output_prefix_arg = DeclareLaunchArgument(
        'map_output_prefix',
        default_value=os.path.expanduser('~/maps/arena_map'),
        description='File prefix where autonomous mapping saves .pgm and .yaml files',
    )

    auto_undock_arg = DeclareLaunchArgument(
        'auto_undock',
        default_value='true',
        description='Whether auto_map mode should automatically undock from the Create 3 dock',
    )

    start_sensor_arg = DeclareLaunchArgument(
        'start_sensor',
        default_value='false',
        description='Whether to start the RPLIDAR S2 and RealSense D455 drivers (diagnose mode)',
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to launch RViz2 (set false when running headless via SSH)',
    )

    mode = LaunchConfiguration('mode')

    is_diagnose = PythonExpression(["'", mode, "' == 'diagnose'"])
    is_map = PythonExpression(["'", mode, "' == 'map'"])
    is_auto_map = PythonExpression(["'", mode, "' == 'auto_map'"])
    is_navigate = PythonExpression(["'", mode, "' == 'navigate'"])

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

    # 2. Map mode: perception + slam_toolbox (manual teleop)
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'mapping.launch.py')
        ),
        condition=IfCondition(is_map),
    )

    map_info = LogInfo(
        msg='MAP mode: use ros2 run tb4_ouster_autonomy teleop_keyboard; save with map_saver_cli.',
        condition=IfCondition(is_map),
    )

    # 3. Auto-Map mode: autonomous frontier exploration + SLAM + auto map save on finish/Ctrl+C
    auto_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'auto_map.launch.py')
        ),
        launch_arguments={
            'map_output_prefix': LaunchConfiguration('map_output_prefix'),
            'auto_undock': LaunchConfiguration('auto_undock'),
        }.items(),
        condition=IfCondition(is_auto_map),
    )

    auto_map_info = LogInfo(
        msg=(
            'Starting in AUTO_MAP mode — autonomous frontier exploration active. '
            'Map saves automatically on completion or Ctrl+C.'
        ),
        condition=IfCondition(is_auto_map),
    )

    # 4. Navigate mode: full Nav2 stack with loaded map and 3D VoxelLayer costmap
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

    # 5. RViz — different config per mode
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
            "True if ('", LaunchConfiguration('rviz'), "' == 'true' and ('",
            mode, "' == 'map' or '", mode, "' == 'auto_map')) else False"
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
        mode_arg,
        map_arg,
        map_output_prefix_arg,
        auto_undock_arg,
        start_sensor_arg,
        rviz_arg,
        # diagnose
        diagnostics_info,
        diagnostics_node,
        perception_launch,
        # map
        map_info,
        mapping_launch,
        # auto_map
        auto_map_info,
        auto_map_launch,
        # navigate
        navigate_info,
        navigate_launch,
        # rviz
        rviz_diagnose_node,
        rviz_map_node,
        rviz_navigate_node,
    ])
