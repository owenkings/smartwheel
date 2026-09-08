"""Fail-closed compatibility entry for right-lidar FAST-LIO mapping.

The reviewed calibration contract has no runtime-eligible transform. Keep the
historical command name available, but route it only to the strict contract
gate. No sensor, mapping, RViz, TF, or motion action is constructed here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup = get_package_share_directory("wheelchair_bringup")
    blocked_entry = os.path.join(
        bringup, "launch", "right_lidar_stage1_mapping.launch.py"
    )
    return LaunchDescription(
        [IncludeLaunchDescription(PythonLaunchDescriptionSource(blocked_entry))]
    )
