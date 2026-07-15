import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    version = Path(LaunchConfiguration("map_version").perform(context)).expanduser().resolve()
    if not version.is_dir():
        raise RuntimeError(f"map_version is not a directory: {version}")
    rviz_config = Path(LaunchConfiguration("rviz_config").perform(context)).expanduser().resolve()
    git_commit = subprocess.run(
        ["git", "rev-parse", "--short=12", "HEAD"], check=False, capture_output=True, text=True
    ).stdout.strip() or "UNKNOWN"
    backend = Node(
        package="smartwheel_workbench",
        executable="workbench_node",
        name="smartwheel_map_preview_backend",
        output="screen",
        parameters=[{
            "mode": "mock",
            "hardware_enabled": False,
            "mapping_backend": "rtabmap",
            "map_root": str(version.parent),
            "experiment_root": "experiments",
            "map_cloud_source": "/preview/no_live_cloud",
            "map_source": "/preview/no_live_map",
            "path_source": "/preview/no_live_path",
            "preview_version": str(version),
            "git_commit": git_commit,
            "hardware_profile": "preview: saved profile in map version",
            "algorithm_profile": "preview: saved profile in map version",
        }],
    )
    rviz = TimerAction(
        period=1.5,
        actions=[Node(
            package="rviz2", executable="rviz2", name="smartwheel_map_preview_rviz",
            output="screen", arguments=["-d", str(rviz_config)],
        )],
    )
    return [backend, rviz]


def generate_launch_description():
    default_rviz = str(
        Path(get_package_share_directory("smartwheel_bringup"))
        / "rviz" / "smartwheel_operator_workbench.rviz"
    )
    return LaunchDescription([
        DeclareLaunchArgument("map_version"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        OpaqueFunction(function=_setup),
    ])
