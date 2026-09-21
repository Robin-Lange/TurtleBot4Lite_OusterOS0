"""Launch composed Nav2 navigation with Ouster perception and bumper coverage.

All Nav2 lifecycle nodes share one component container. This keeps DDS
participant usage low enough for the memory-constrained Create 3 base while
retaining one final velocity writer through velocity_smoother.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
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
        default_value=os.path.join(pkg_share, 'maps', 'arena_map.yaml'),
        description='Full path to the map yaml file to load for navigation',
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'perception.launch.py')
        )
    )

    lifecycle_nodes = [
        'map_server',
        'amcl',
        'controller_server',
        'velocity_smoother',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]
    tf_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    nav2_container = ComposableNodeContainer(
        name='nav2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        output='screen',
        parameters=[configured_params],
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[configured_params, {
                    'yaml_filename': LaunchConfiguration('map'),
                }],
                remappings=tf_remappings,
            ),
            ComposableNode(
                package='nav2_amcl',
                plugin='nav2_amcl::AmclNode',
                name='amcl',
                parameters=[configured_params],
                remappings=tf_remappings,
            ),
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=tf_remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=tf_remappings + [
                    ('cmd_vel', 'cmd_vel_nav'),
                    ('cmd_vel_smoothed', '/cmd_vel'),
                ],
            ),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=tf_remappings,
            ),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
                remappings=tf_remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=tf_remappings,
            ),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[configured_params],
                remappings=tf_remappings,
            ),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{
                    'use_sim_time': False,
                    'autostart': True,
                    'node_names': lifecycle_nodes,
                }],
            ),
        ],
    )

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

    return LaunchDescription([
        map_arg,
        perception_launch,
        nav2_container,
        bumper_cloud_node,
    ])
