"""Launch perception pipeline for Create 3 + Slamtec RPLIDAR S2 + Intel RealSense D455.

Publishes:
- Static mount transforms (mount_tf.launch.py)
- Slamtec RPLIDAR S2 driver (rplidar_node -> /scan at 10 Hz)
- Intel RealSense D455 driver (realsense2_camera_node -> RGB, Depth, PointCloud)
- Point cloud filtering for RealSense 3D obstacle avoidance
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('tb4_ouster_autonomy')

    # LiDAR launch arguments
    rplidar_port_arg = DeclareLaunchArgument(
        'rplidar_port',
        default_value='/dev/rplidar',
        description='Serial device path for Slamtec RPLIDAR S2',
    )
    rplidar_baudrate_arg = DeclareLaunchArgument(
        'rplidar_baudrate',
        default_value='1000000',
        description='Baud rate for RPLIDAR S2 (1000000)',
    )
    rplidar_frame_arg = DeclareLaunchArgument(
        'rplidar_frame',
        default_value='laser',
        description='Frame ID for RPLIDAR scan',
    )
    rplidar_scan_mode_arg = DeclareLaunchArgument(
        'rplidar_scan_mode',
        default_value='DenseBoost',
        description='Scan mode for RPLIDAR S2 (DenseBoost)',
    )
    start_lidar_arg = DeclareLaunchArgument(
        'start_lidar',
        default_value='true',
        description='Whether to start the RPLIDAR S2 driver',
    )

    # RealSense launch arguments
    start_camera_arg = DeclareLaunchArgument(
        'start_camera',
        default_value='true',
        description='Whether to start the Intel RealSense D455 driver',
    )
    enable_pointcloud_arg = DeclareLaunchArgument(
        'enable_pointcloud',
        default_value='true',
        description='Whether to enable RealSense PointCloud2 publishing',
    )

    # Include static mount transforms (base_link -> laser, base_link -> camera_link)
    mount_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'mount_tf.launch.py')
        )
    )

    # RPLIDAR S2 driver node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('rplidar_port'),
            'serial_baudrate': LaunchConfiguration('rplidar_baudrate'),
            'frame_id': LaunchConfiguration('rplidar_frame'),
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': LaunchConfiguration('rplidar_scan_mode'),
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_lidar')),
    )

    # Intel RealSense D455 driver node
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[{
            'camera_name': 'camera',
            'publish_tf': True,
            'pointcloud.enable': LaunchConfiguration('enable_pointcloud'),
            'pointcloud__neon_.enable': LaunchConfiguration('enable_pointcloud'),
            'rgb_camera.color_profile': '640x480x30',
            'depth_module.depth_profile': '640x480x30',
            'depth_module.infra_profile': '640x480x30',
            'align_depth.enable': True,
            'enable_sync': True,
            'initial_reset': False,
            'clip_distance': 6.0,
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_camera')),
    )

    # Cloud filter node for RealSense 3D point cloud
    cloud_filter_node = Node(
        package='tb4_ouster_autonomy',
        executable='cloud_filter',
        name='cloud_filter',
        output='screen',
        parameters=[{
            'input_topic': '/camera/camera/depth/color/points',
            'output_topic': '/camera/cloud_filtered',
            'ground_topic': '/camera/ground_points',
            'publish_ground_cloud': False,
            'target_rate_hz': 15.0,
            'min_range': 0.35,
            'max_range': 6.0,
            'min_height': 0.04,
            'max_height': 1.5,
            'enable_ransac': False,
            'self_box_min_x': -0.10,
            'self_box_max_x': 0.25,
            'self_box_min_y': -0.20,
            'self_box_max_y': 0.20,
            'self_box_min_z': -0.20,
            'self_box_max_z': 0.20,
        }],
        condition=IfCondition(LaunchConfiguration('enable_pointcloud')),
    )

    return LaunchDescription([
        rplidar_port_arg,
        rplidar_baudrate_arg,
        rplidar_frame_arg,
        rplidar_scan_mode_arg,
        start_lidar_arg,
        start_camera_arg,
        enable_pointcloud_arg,
        mount_tf_launch,
        rplidar_node,
        realsense_node,
        cloud_filter_node,
    ])

