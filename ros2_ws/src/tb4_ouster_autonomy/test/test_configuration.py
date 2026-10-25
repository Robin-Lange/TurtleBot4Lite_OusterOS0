"""Cross-file contracts that syntax checks alone cannot catch."""

import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[2]


def test_monitor_observes_both_real_sensors_and_humble_threshold():
    config = yaml.safe_load((PACKAGE/'config/collision_monitor_params.yaml').read_text())['collision_monitor']
    params = config['ros__parameters']
    assert params['scan']['topic'] == '/scan'
    assert params['pointcloud']['topic'] == '/camera/cloud_filtered'
    assert set(params['observation_sources']) == {'scan', 'pointcloud'}
    assert params['PolygonStop']['max_points'] == 0  # Humble triggers above max_points.
    assert params['cmd_vel_in_topic'] == '/cmd_vel_safe'
    assert params['cmd_vel_out_topic'] == '/cmd_vel'


def test_no_application_publishes_directly_to_base_velocity():
    for path in (PACKAGE/'tb4_ouster_autonomy').glob('*.py'):
        tree = ast.parse(path.read_text())
        for call in ast.walk(tree):
            if isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute):
                if call.func.attr == 'create_publisher' and len(call.args) > 1:
                    assert not (isinstance(call.args[1], ast.Constant) and call.args[1].value == '/cmd_vel'), path
    assert "('cmd_vel_smoothed', '/nav2/cmd_vel')" in (PACKAGE/'tb4_ouster_autonomy/bringup.py').read_text()


def test_each_motion_mode_includes_safety_and_diagnose_does_not():
    for mode in ('mapping', 'auto_map', 'navigate'):
        assert 'safety.launch.py' in (PACKAGE/f'launch/{mode}.launch.py').read_text()
    source = (PACKAGE/'launch/robot.launch.py').read_text()
    assert 'SetEnvironmentVariable' not in source


def test_no_fabricated_dynamic_tf_from_bumper_node():
    source = (PACKAGE/'tb4_ouster_autonomy/bumper_contact_cloud.py').read_text()
    assert 'TransformBroadcaster' not in source
    assert 'lookup_transform("odom", "base_link", Time.from_msg(msg.header.stamp))' in source


def test_planar_scan_can_mark_one_voxel_and_humble_checker_key():
    params = yaml.safe_load((PACKAGE/'config/nav2_params.yaml').read_text())
    assert params['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']['mark_threshold'] == 0
    assert params['controller_server']['ros__parameters']['progress_checker_plugin'] == 'progress_checker'
    assert 'bumper_contact' not in params['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']


def test_declared_runtime_dependencies_cover_launch_packages():
    dependencies = {element.text for element in ET.parse(PACKAGE/'package.xml').findall('exec_depend')}
    for name in ('rplidar_ros', 'realsense2_camera', 'nav2_common', 'nav2_bringup', 'python3-numpy',
                 'nav2_collision_monitor', 'nav2_velocity_smoother', 'rclcpp_components'):
        assert name in dependencies


def test_profiles_and_yaml_are_parseable():
    for path in (ROOT/'config').glob('*.xml'):
        ET.parse(path)
    for path in (PACKAGE/'config').glob('*.yaml'):
        assert isinstance(yaml.safe_load(path.read_text()), dict)
