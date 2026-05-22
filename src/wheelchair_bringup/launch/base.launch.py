from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    bringup_share = FindPackageShare("wheelchair_bringup")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="real",
                description="real uses configured Modbus registers; mock publishes open-loop odometry only.",
            ),
            Node(
                package="wheelchair_base",
                executable="zlac8030_driver_node",
                name="zlac8030_driver_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution([bringup_share, "config", "zlac8030_base.yaml"]),
                    {"mode": mode},
                ],
            ),
        ]
    )
