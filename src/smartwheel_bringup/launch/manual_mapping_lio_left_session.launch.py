"""Left FAST-LIO manual mapping with a complete map-products session.

This wrapper starts the formal map-products node before including the existing
left manual mapping launch.  It deliberately leaves that existing launch
untouched because it may contain unrelated operator-workbench changes.

Safety: this is a software orchestration contract only.  The included manual
launch selects the left physical sensor path, keeps motor control disabled by
default, and must not be run on a vehicle without separate hardware approval.
The right-lidar entry remains blocked by its existing fail-closed contract.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    smartwheel_bringup = get_package_share_directory("smartwheel_bringup")
    wheelchair_bringup = get_package_share_directory("wheelchair_bringup")
    map_export_launch = os.path.join(smartwheel_bringup, "launch", "map_export.launch.py")
    manual_left_launch = os.path.join(wheelchair_bringup, "launch", "manual_mapping_lio_left.launch.py")
    default_profile = str(Path(smartwheel_bringup) / "config" / "hardware_profile.mock.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("map_name", default_value="manual_lio_left"),
            DeclareLaunchArgument("output_root", default_value="maps/versions"),
            DeclareLaunchArgument("hardware_profile", default_value=default_profile),
            DeclareLaunchArgument("motion_control_enabled", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("enable_loop_backend", default_value="false"),
            LogInfo(
                msg=(
                    "[manual_mapping_lio_left_session] starting formal session capture "
                    "before LEFT FAST-LIO; motors remain disabled by default"
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(map_export_launch),
                launch_arguments={
                    "map_name": LaunchConfiguration("map_name"),
                    "output_root": LaunchConfiguration("output_root"),
                    "hardware_profile": LaunchConfiguration("hardware_profile"),
                    "mapping_backend": "rtabmap",
                    "state_mode": "lio_primary",
                    "lidar_mode": "single",
                    "cloud_topic": "/cloud_registered",
                    "odom_topic": "/Odometry",
                    "cloud_frame_mode": "world_registered",
                    "expected_cloud_frame": "camera_init",
                    "require_session_stop": "true",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(manual_left_launch),
                launch_arguments={
                    "motion_control_enabled": LaunchConfiguration("motion_control_enabled"),
                    "rviz": LaunchConfiguration("rviz"),
                    "enable_loop_backend": LaunchConfiguration("enable_loop_backend"),
                }.items(),
            ),
        ]
    )
