from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    output = LaunchConfiguration("output")
    return LaunchDescription(
        [
            DeclareLaunchArgument("output", default_value="bags/wheelchair_session"),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "-o",
                    output,
                    "/tf",
                    "/tf_static",
                    "/xtm60/points",
                    "/scan",
                    "/imu/data",
                    "/ultrasonic/range_0",
                    "/ultrasonic/range_1",
                    "/camera/front/image_raw",
                    "/camera/left/image_raw",
                    "/wheel/odom",
                    "/cmd_vel_nav",
                    "/cmd_vel_safe",
                    "/safety_state",
                    "/hardware/status",
                    "/localization/health",
                    "/passability/status",
                    "/base/status",
                ],
                output="screen",
            ),
        ]
    )
