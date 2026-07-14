from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_profile = str(
        Path(get_package_share_directory("smartwheel_bringup")) / "config" / "hardware_profile.mock.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_name", default_value="mapping_export"),
            DeclareLaunchArgument("output_root", default_value="maps/versions"),
            DeclareLaunchArgument("hardware_profile", default_value=default_profile),
            DeclareLaunchArgument("mapping_backend", default_value="rtabmap", choices=["rtabmap", "slam_toolbox"]),
            DeclareLaunchArgument("state_mode", default_value="lio_primary", choices=["lio_primary", "wheel_imu_fallback"]),
            DeclareLaunchArgument("lidar_mode", default_value="dual_map_only", choices=["single", "dual_map_only", "dual_lio"]),
            DeclareLaunchArgument("bag_path", default_value=""),
            DeclareLaunchArgument("enable_offline_colorization", default_value="true", choices=["true", "false"]),
            Node(
                package="smartwheel_map_products",
                executable="map_products_node",
                output="screen",
                parameters=[
                    {
                        "map_name": LaunchConfiguration("map_name"),
                        "output_root": LaunchConfiguration("output_root"),
                        "hardware_profile_path": LaunchConfiguration("hardware_profile"),
                        "mapping_backend": LaunchConfiguration("mapping_backend"),
                        "state_mode": LaunchConfiguration("state_mode"),
                        "lidar_mode": LaunchConfiguration("lidar_mode"),
                        "bag_path": LaunchConfiguration("bag_path"),
                        "enable_offline_colorization": LaunchConfiguration("enable_offline_colorization"),
                    }
                ],
            ),
        ]
    )
