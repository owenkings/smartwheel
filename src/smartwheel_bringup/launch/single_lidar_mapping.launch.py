from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    fallback_config = str(
        Path(get_package_share_directory("smartwheel_global_mapping"))
        / "config"
        / "robot_localization_fallback.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="mock", choices=["mock", "real"]),
            DeclareLaunchArgument("state_mode", default_value="lio_primary", choices=["lio_primary", "wheel_imu_fallback"]),
            DeclareLaunchArgument("hardware_enabled", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("primary_lidar", default_value="right", choices=["left", "right"]),
            Node(
                package="smartwheel_global_mapping",
                executable="lio_input_adapter",
                output="screen",
                parameters=[
                    {
                        "input_topic": ["/lidar/", LaunchConfiguration("primary_lidar"), "/points_raw"],
                        "output_topic": "/lidar/primary/points_lio",
                        "expected_frame_id": ["xtm60_", LaunchConfiguration("primary_lidar"), "_link"],
                    }
                ],
            ),
            Node(
                package="smartwheel_state_estimation",
                executable="state_selector_node",
                output="screen",
                parameters=[
                    {
                        "state_mode": LaunchConfiguration("state_mode"),
                        "wheel_topic": PythonExpression(
                            [
                                "'/odom/fallback' if '",
                                LaunchConfiguration("state_mode"),
                                "' == 'wheel_imu_fallback' else '/wheel/odom'",
                            ]
                        ),
                    }
                ],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", LaunchConfiguration("state_mode"), "' == 'wheel_imu_fallback'"])
                ),
                parameters=[fallback_config],
                remappings=[("odometry/filtered", "/odom/fallback")],
            ),
        ]
    )
