"""Sensor-only launch. It does not start SLAM, TF or any actuator."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    mode = value("source_mode")
    if mode not in ("dual", "single_left", "single_right"):
        raise ValueError("source_mode must be dual, single_left or single_right")
    policy = value("device_config_policy")
    if policy not in ("preserve_current", "apply_xtcfg"):
        raise ValueError("device_config_policy must be preserve_current or apply_xtcfg")
    session_id = value("session_id")
    if not session_id:
        raise ValueError("root-managed session_id is required")
    config_directory = value("config_directory")
    if not config_directory:
        from wc_runtime.project_config import lidar_config_directory
        from wc_runtime.project_paths import project_root
        config_directory = str(lidar_config_directory(project_root()))
    result = []
    sides = ("left", "right") if mode == "dual" else (mode[7:],)
    for side in sides:
        node = Node(package="wc_xt_driver", executable="xt_source_node", name="xt_" + side,
                    namespace="wc_mapping/lidar_" + side, output="screen",
                    parameters=[os.path.join(config_directory, side + ".yaml"), {
                        "source_config_path": os.path.join(config_directory, side + "-2026-09-11.xtcfg"),
                        "session_id": session_id,
                        "run_root": value("run_root"),
                        "allow_hardware": value("allow_hardware").lower() == "true",
                        "read_only_probe": value("read_only_probe").lower() == "true",
                        "device_config_policy": policy,
                        "publish_cloud_mirror": value("publish_cloud_mirror").lower() == "true",
                        "require_recorder": value("require_recorder").lower() == "true",
                        "max_runtime_seconds": int(value("max_runtime_seconds")),
                        "source_stale_seconds": int(value("source_stale_seconds")),
                    }])
        result.extend([node, RegisterEventHandler(OnProcessExit(target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason="One source process exited; stop the coordinated sensor session"))]))])
    return result


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("source_mode", default_value="dual"),
        DeclareLaunchArgument("config_directory", default_value="", description="Explicit device/host snapshot; empty selects current project configuration"),
        DeclareLaunchArgument("session_id", default_value=""),
        DeclareLaunchArgument("run_root", default_value=os.path.expanduser("~/wheelchair/run")),
        DeclareLaunchArgument("allow_hardware", default_value="false"),
        DeclareLaunchArgument("read_only_probe", default_value="true"),
        DeclareLaunchArgument("device_config_policy", default_value="preserve_current"),
        DeclareLaunchArgument("publish_cloud_mirror", default_value="false"),
        DeclareLaunchArgument("require_recorder", default_value="false"),
        DeclareLaunchArgument("max_runtime_seconds", default_value="30",
                              description="Acquisition runtime in seconds; 0 means until the owner requests stop"),
        DeclareLaunchArgument("source_stale_seconds", default_value="3",
                              description="Point-source silence shutdown; 0 keeps waiting for data"),
        OpaqueFunction(function=_launch),
    ])
