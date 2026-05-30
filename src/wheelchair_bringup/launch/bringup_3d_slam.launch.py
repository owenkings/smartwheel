"""Top-level LiDAR-Visual-Inertial-Wheel 3D SLAM bring-up.

Orchestrates: real sensors (dual XT-M60 + IMU + cameras + ultrasonic) ->
dual-lidar fusion -> external LIVO backend -> wheel/IMU/LIVO EKF + consistency
monitor -> 3D->2D occupancy projection -> optional RGB colorizer.

TF safety: exactly ONE node publishes odom->base_link, selected by tf_owner:
  tf_owner:=ekf  (default) EKF publishes it; ZLAC publish_tf=false; LIVO TF off.
  tf_owner:=wheel          ZLAC publishes it; EKF publish_tf=false.
  tf_owner:=livo           external LIVO publishes it; EKF+ZLAC TF off.

Motor safety: base.launch.py is started in real mode but the ZLAC driver keeps
motion_control_enabled=false and command registers -1 (see zlac8030_base.yaml),
so NO motor commands are written. Autonomous motion stays gated.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _b(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() == "true"


def _s(context, name):
    return LaunchConfiguration(name).perform(context).strip()


def _setup(context, *args, **kwargs):
    bringup = get_package_share_directory("wheelchair_bringup")
    mapping = get_package_share_directory("wheelchair_3d_mapping")

    main_camera = _s(context, "main_camera") or "left"
    tf_owner = _s(context, "tf_owner") or "ekf"
    backend = _s(context, "backend")
    use_sim = _s(context, "use_sim_time")

    # Resolve main camera physical topic from camera_roles.yaml.
    roles_path = os.path.join(bringup, "config", "camera_roles.yaml")
    image_topic = f"/camera/{main_camera}/image_raw"
    alias_topic = "/main_camera/image_raw"
    try:
        with open(roles_path) as f:
            roles = (yaml.safe_load(f) or {}).get("camera_roles", {})
        image_topic = roles.get(main_camera, {}).get("image_topic", image_topic)
        alias_topic = roles.get("main_camera_alias_topic", alias_topic)
    except OSError:
        pass

    actions = [LogInfo(msg=f"[bringup_3d_slam] backend={backend or 'none'} main_camera={main_camera} "
                           f"tf_owner={tf_owner} (single odom->base_link owner enforced)")]

    # 1. Real sensors: dual XT-M60 + IMU + cameras + ultrasonic.
    if _b(context, "enable_sensors"):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "sensors.launch.py")),
            launch_arguments={
                "mode": "real", "enable_xtm60": "false",
                "enable_xtm60_left": "true", "enable_xtm60_right": "true",
                "enable_imu": "true", "enable_camera": "true", "enable_ultrasonic": "true",
            }.items(),
        ))

    # 2. Dual-lidar fusion -> /points_merged.
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mapping, "launch", "dual_lidar_fusion.launch.py")),
        launch_arguments={"use_sim_time": use_sim}.items(),
    ))

    # 3. Main-camera alias /camera/<side>/image_raw -> /main_camera/image_raw.
    #    Uses topic_tools relay; disable with provide_main_camera_alias:=false
    #    (then point livo_interface.yaml image_topic at the physical topic).
    if _b(context, "provide_main_camera_alias"):
        actions.append(Node(
            package="topic_tools", executable="relay", name="main_camera_relay",
            output="screen", arguments=[image_topic, alias_topic],
        ))

    # 4. External LIVO backend (graceful if not installed / backend:=none).
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mapping, "launch", "livo_3d_mapping.launch.py")),
        launch_arguments={"backend": backend}.items(),
    ))

    # 5. Base driver. publish_tf only when tf_owner=wheel. Motors stay gated.
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "base.launch.py")),
        launch_arguments={"mode": "real", "publish_tf": "true" if tf_owner == "wheel" else "false"}.items(),
    ))

    # 6. Wheel/IMU/LIVO EKF + consistency monitor (EKF publish_tf when tf_owner=ekf).
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mapping, "launch", "livo_wheel_fusion.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim, "tf_owner": tf_owner,
            "use_wheel_fusion": _s(context, "use_wheel_fusion"),
        }.items(),
    ))

    # 7. Optional 3D->2D occupancy projection for Nav2.
    if _b(context, "use_cloud_to_2d"):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mapping, "launch", "cloud_to_2d_map.launch.py")),
            launch_arguments={"use_sim_time": use_sim}.items(),
        ))

    # 8. Optional fallback RGB colorizer (only if backend doesn't already color).
    if _b(context, "use_colorizer"):
        actions.append(Node(
            package="wheelchair_3d_mapping", executable="rgb_cloud_colorizer_node",
            name="rgb_cloud_colorizer_node", output="screen",
            parameters=[os.path.join(mapping, "config", "rgb_colorizer.yaml"),
                        {"image_topic": alias_topic, "use_sim_time": use_sim == "true"}],
        ))

    if tf_owner == "livo":
        actions.append(LogInfo(msg="[bringup_3d_slam] tf_owner=livo: ensure the external backend "
                                   "publishes odom->base_link; EKF and ZLAC TF are disabled."))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("backend", default_value="none",
                              description="fast_livo2 | r3live | none."),
        DeclareLaunchArgument("main_camera", default_value="left",
                              description="left | right - which forward camera feeds LIVO."),
        DeclareLaunchArgument("tf_owner", default_value="ekf",
                              description="ekf | livo | wheel - the single odom->base_link publisher."),
        DeclareLaunchArgument("use_wheel_fusion", default_value="true"),
        DeclareLaunchArgument("use_colorizer", default_value="false"),
        DeclareLaunchArgument("use_cloud_to_2d", default_value="true"),
        DeclareLaunchArgument("enable_sensors", default_value="true"),
        DeclareLaunchArgument("provide_main_camera_alias", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
