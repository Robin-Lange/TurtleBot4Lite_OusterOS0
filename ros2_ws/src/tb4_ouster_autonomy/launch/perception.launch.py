"""Launch perception pipeline for Create 3 + Ouster OS0.

Publishes mount static transforms and brings up the live Ouster OS0 sensor driver
configured at 1024x10 (10 Hz) with scan_ring=63 to natively output 2D /ouster/scan
and full 3D point cloud /ouster/points without software downsampling or extra conversion nodes.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('tb4_ouster_autonomy')
    ouster_share = get_package_share_directory('ouster_ros')

    sensor_hostname_arg = DeclareLaunchArgument(
        'sensor_hostname',
        default_value='169.254.97.211',
        description='Sensor hostname or IP address',
    )
    lidar_mode_arg = DeclareLaunchArgument(
        'lidar_mode',
        default_value='1024x10',
        description='Lidar resolution and rate: 1024x10 for native 10 Hz',
    )
    scan_ring_arg = DeclareLaunchArgument(
        'scan_ring',
        default_value='63',
        description='Ouster beam ring to extract as 2D LaserScan (-0.11 deg horizontal)',
    )
    start_driver_arg = DeclareLaunchArgument(
        'start_driver',
        default_value='true',
        description='Whether to start the ouster_ros sensor driver',
    )
    relay_scan_arg = DeclareLaunchArgument(
        'relay_to_scan',
        default_value='true',
        description='Whether to relay /ouster/scan to /scan for standard tools',
    )

    mount_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'mount_tf.launch.py')
        )
    )

    ouster_driver = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(ouster_share, 'launch', 'sensor.launch.xml')
        ),
        launch_arguments={
            'sensor_hostname': LaunchConfiguration('sensor_hostname'),
            'lidar_mode': LaunchConfiguration('lidar_mode'),
            'timestamp_mode': 'TIME_FROM_ROS_TIME',
            'proc_mask': 'PCL|IMU|SCAN',
            'scan_ring': LaunchConfiguration('scan_ring'),
            'viz': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_driver')),
    )

    cloud_filter_node = Node(
        package='tb4_ouster_autonomy',
        executable='cloud_filter',
        name='cloud_filter',
        output='screen',
        parameters=[{
            'input_topic': '/ouster/points',
            'output_topic': '/ouster/cloud_filtered',
            'target_rate_hz': 10.0,
            'self_box_min_x': -0.22,
            'self_box_max_x': 0.22,
            'self_box_min_y': -0.22,
            'self_box_max_y': 0.22,
            'self_box_min_z': -0.25,
            'self_box_max_z': 0.06,
        }],
    )

    scan_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='scan_relay',
        output='screen',
        arguments=['/ouster/scan', '/scan'],
        condition=IfCondition(LaunchConfiguration('relay_to_scan')),
    )

    return LaunchDescription([
        sensor_hostname_arg,
        lidar_mode_arg,
        scan_ring_arg,
        start_driver_arg,
        relay_scan_arg,
        mount_tf_launch,
        ouster_driver,
        cloud_filter_node,
        scan_relay_node,
    ])
