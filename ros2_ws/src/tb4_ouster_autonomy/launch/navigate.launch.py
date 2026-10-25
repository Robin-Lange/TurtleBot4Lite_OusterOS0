"""Launch composed Nav2 navigation with RPLIDAR S2 + RealSense D455 perception and bumper coverage.

All Nav2 lifecycle nodes share one component container. This keeps DDS
participant usage low enough for the memory-constrained Create 3 base while
routing smoothed proposals through the authority gate and Collision Monitor.
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

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('~/maps/arena_map.yaml'),
        description='Full path to the map yaml file to load for navigation',
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'perception.launch.py')
        )
    )

    nav2_container = navigation_container(configured_params, LaunchConfiguration('map'))

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

    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'safety.launch.py')),
        launch_arguments={'allow_nav2': 'true'}.items(),
    )
    return LaunchDescription([
        map_arg,
        perception_launch,
        safety,
        nav2_container,
        bumper_cloud_node,
    ])
