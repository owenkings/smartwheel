from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    # publish_tf controls whether zlac8030_driver_node broadcasts the
    # odom -> base_link TF. When the robot_localization EKF is enabled we
    # MUST set this to false, otherwise both the wheel-only driver and the
    # EKF will publish odom -> base_link and TF will end up jittering.
    publish_tf = LaunchConfiguration("publish_tf")
    hold_zero_before_motion_init = LaunchConfiguration("hold_zero_before_motion_init")
    bringup_share = FindPackageShare("wheelchair_bringup")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="real",
                description="real uses configured Modbus registers; mock publishes open-loop odometry only.",
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="true",
                description="Whether zlac8030_driver_node publishes odom->base_link TF. Set to false when robot_localization EKF is the TF publisher.",
            ),
            DeclareLaunchArgument(
                "hold_zero_before_motion_init",
                default_value="false",
                description="If true, a zero command in navigation mode initializes servo hold before any non-zero command.",
            ),
            Node(
                package="wheelchair_base",
                executable="zlac8030_driver_node",
                name="zlac8030_driver_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution([bringup_share, "config", "zlac8030_base.yaml"]),
                    {
                        "mode": mode,
                        "publish_tf": publish_tf,
                        "hold_zero_before_motion_init": hold_zero_before_motion_init,
                    },
                ],
            ),
        ]
    )
