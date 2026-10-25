"""Isolate all ROS integration tests from a robot and any host DDS profiles."""

import os
import sys
from pathlib import Path

os.environ['ROS_DOMAIN_ID'] = '88'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
