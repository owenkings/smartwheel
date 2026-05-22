from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bag = LaunchConfiguration("bag")
    rate = LaunchConfiguration("rate")
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag", default_value="bags/wheelchair_session"),
            DeclareLaunchArgument("rate", default_value="1.0"),
            ExecuteProcess(
                cmd=["ros2", "bag", "play", bag, "--clock", "--rate", rate],
                output="screen",
            ),
        ]
    )
