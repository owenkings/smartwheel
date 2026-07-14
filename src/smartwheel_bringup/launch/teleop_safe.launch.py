from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="mock", choices=["mock", "real"]),
            DeclareLaunchArgument("hardware_enabled", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("max_linear_mps", default_value="0.25"),
            DeclareLaunchArgument("max_angular_rps", default_value="0.6"),
            Node(
                package="smartwheel_teleop",
                executable="safe_teleop_node",
                output="screen",
                parameters=[
                    {
                        "hardware_enabled": LaunchConfiguration("hardware_enabled"),
                        "max_linear_mps": LaunchConfiguration("max_linear_mps"),
                        "max_angular_rps": LaunchConfiguration("max_angular_rps"),
                    }
                ],
            ),
            Node(
                package="smartwheel_teleop",
                executable="command_watchdog_node",
                output="screen",
                parameters=[{"hardware_enabled": LaunchConfiguration("hardware_enabled")}],
            ),
        ]
    )
