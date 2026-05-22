from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    map_name = LaunchConfiguration("map_name")
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_name", default_value="maps/indoor_map"),
            ExecuteProcess(
                cmd=["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", map_name],
                output="screen",
            ),
        ]
    )
