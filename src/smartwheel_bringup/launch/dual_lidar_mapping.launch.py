from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("smartwheel_global_mapping")) / "config"
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="mock", choices=["mock", "real"]),
            DeclareLaunchArgument("mapping_backend", default_value="rtabmap", choices=["rtabmap", "slam_toolbox"]),
            DeclareLaunchArgument("lidar_mode", default_value="dual_map_only", choices=["dual_map_only", "dual_lio"]),
            DeclareLaunchArgument("state_mode", default_value="lio_primary", choices=["lio_primary", "wheel_imu_fallback"]),
            DeclareLaunchArgument("hardware_enabled", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("database_path", default_value="maps/rtabmap.db"),
            Node(
                package="dual_lidar_fusion",
                executable="dual_lidar_fusion_node",
                output="screen",
                parameters=[
                    {
                        "integration_mode": PythonExpression(
                            ["'dual_lio' if '", LaunchConfiguration("lidar_mode"), "' == 'dual_lio' else 'map_only'"]
                        )
                    }
                ],
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", LaunchConfiguration("mapping_backend"), "' == 'rtabmap'"])
                ),
                parameters=[str(config / "rtabmap_params.yaml"), {"database_path": LaunchConfiguration("database_path")}],
                remappings=[("odom", "/odom/fused"), ("scan_cloud", "/lidar/merged/points"), ("map", "/rtabmap/map")],
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", LaunchConfiguration("mapping_backend"), "' == 'slam_toolbox'"])
                ),
                parameters=[str(config / "slam_toolbox_params.yaml")],
            ),
            Node(
                package="smartwheel_global_mapping",
                executable="cloud_to_scan_node",
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", LaunchConfiguration("mapping_backend"), "' == 'slam_toolbox'"])
                ),
            ),
        ]
    )
