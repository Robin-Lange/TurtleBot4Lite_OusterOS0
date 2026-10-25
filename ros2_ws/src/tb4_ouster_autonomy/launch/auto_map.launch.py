"""Launch autonomous frontier exploration + SLAM mapping + auto map saving.

Combines:
- Perception pipeline (RPLIDAR S2 + RealSense D455 + static mount TFs)
- SLAM Toolbox (online asynchronous mapping publishing /map and map->odom)
- Nav2 path planning & 3D Voxel obstacle avoidance (without conflicting map_server/amcl)
- AutonomousExplorer node (auto-undock, frontier detection, NavigateToPose dispatch,
  periodic map checkpoints, and automatic map save on completion or Ctrl+C abort)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from tb4_ouster_autonomy.bringup import navigation_container
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = get_package_share_directory('tb4_ouster_autonomy')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    map_output_arg = DeclareLaunchArgument(
        'map_output_prefix',
        default_value=os.path.expanduser('~/maps/arena_map'),
        description='File path prefix (without extension) where .pgm and .yaml map files are saved',
    )

    auto_undock_arg = DeclareLaunchArgument(
        'auto_undock',
        default_value='true',
        description='Automatically undock from Create 3 charging dock before exploring',
    )

    # 1. Perception + SLAM Toolbox
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={'enable_safety': 'false'}.items(),
    )

    nav2_slam_container = navigation_container(configured_params)

    bumper_cloud_node = Node(
        package='tb4_ouster_autonomy',
        executable='bumper_contact_cloud',
        name='bumper_contact_cloud',
        output='screen',
        parameters=[{
            'point_decay_s': 5.0,
            'publish_rate_hz': 5.0,
        }],
    )

    # 3. Autonomous Frontier Explorer + Auto Map Saver
    explorer_node = Node(
        package='tb4_ouster_autonomy',
        executable='autonomous_explorer',
        name='autonomous_explorer',
        output='screen',
        parameters=[{
            'map_output_prefix': LaunchConfiguration('map_output_prefix'),
            'auto_undock': LaunchConfiguration('auto_undock'),
            'auto_save_interval_s': 20.0,
            'goal_timeout_s': 35.0,
            'min_frontier_size': 6,
            'obstacle_clearance_m': 0.25,
            'blacklist_radius_m': 0.45,
        }],
    )

    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'safety.launch.py')),
        launch_arguments={'allow_nav2': 'true'}.items(),
    )
    return LaunchDescription([
        map_output_arg,
        auto_undock_arg,
        mapping_launch,
        safety,
        nav2_slam_container,
        bumper_cloud_node,
        explorer_node,
    ])
