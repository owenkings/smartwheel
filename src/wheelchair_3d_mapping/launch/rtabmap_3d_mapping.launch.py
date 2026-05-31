"""RTAB-Map 3D mapping (LiDAR-primary) — the current 3D mapping main line.

The merged XT-M60 cloud (points_topic, default /points_merged, already in
base_link) drives the map. By default rtabmap_odom/icp_odometry computes 3D
odometry from the cloud and is the SINGLE odom->base_link TF owner; then
rtabmap_slam/rtabmap builds:
  /rtabmap/cloud_map  - the 3D point-cloud map (PRIMARY result)
  /rtabmap/mapData    - the pose graph / keyframes
  /rtabmap/grid_map   - a 2D occupancy grid PROJECTED from the cloud (Nav2 only)
  /rtabmap/odom       - the 3D trajectory

odom_mode:
  icp (default) -> icp_odometry owns odom->base_link; do NOT also run EKF/ZLAC TF.
  external      -> use an existing odom (odom_topic, e.g. /wheel/odom or
                   /odometry/filtered); that node owns odom->base_link, no icp.

Camera is OPTIONAL (subscribe_rgb:=true): texture / visual loop closure only; 3D
geometry never depends on camera<->LiDAR extrinsics.

Real raw XT-M60 topics are /xtm60/left/points and /xtm60/right/points; they are
fused into points_topic by dual_lidar_cloud_fusion_node. Requires
ros-humble-rtabmap-ros (sudo apt install ros-humble-rtabmap-ros).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _setup(context, *args, **kwargs):
    mapping = get_package_share_directory("wheelchair_3d_mapping")
    bringup = get_package_share_directory("wheelchair_bringup")

    def s(name):
        return LaunchConfiguration(name).perform(context).strip()

    def flag(name):
        return s(name).lower() == "true"

    cfg = s("config")
    points = s("points_topic")
    imu = s("imu_topic")
    odom = s("odom_topic")
    odom_mode = s("odom_mode").lower()
    frame_id = s("frame_id")
    subscribe_rgb = flag("subscribe_rgb")
    localization = flag("localization")
    use_sim = s("use_sim_time")
    qsize = int(s("queue_size"))
    common = {"use_sim_time": use_sim == "true"}
    actions = []

    # Optional self-contained sensors + fusion (NO base/EKF -> icp_odometry is the
    # only odom->base_link publisher). Leave false if your stack already runs them.
    if flag("bringup_sensors"):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "sensors.launch.py")),
            launch_arguments={
                "mode": "real", "enable_xtm60": "false",
                "enable_xtm60_left": "true", "enable_xtm60_right": "true",
                "enable_imu": "true", "enable_ultrasonic": "false",
                "enable_camera": "true" if subscribe_rgb else "false",
            }.items()))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mapping, "launch", "dual_lidar_fusion.launch.py")),
            launch_arguments={"use_sim_time": use_sim}.items()))

    # ICP odometry owns odom->base_link (skipped in external-odom mode).
    if odom_mode == "icp":
        actions.append(Node(
            package="rtabmap_odom", executable="icp_odometry", name="icp_odometry", output="screen",
            parameters=[cfg, common, {"frame_id": frame_id, "publish_tf": True,
                                      "topic_queue_size": qsize, "sync_queue_size": qsize}],
            remappings=[("scan_cloud", points), ("imu", imu), ("odom", odom)]))
    else:
        actions.append(LogInfo(msg=f"[rtabmap_3d_mapping] odom_mode=external: using {odom} as odometry; "
                                   f"that node must own odom->base_link (icp_odometry NOT started)."))

    # rtabmap -> 3D cloud map + graph + projected 2D grid.
    rtab_params = [cfg, common, {
        "frame_id": frame_id,
        "subscribe_scan_cloud": flag("subscribe_scan_cloud"),
        "subscribe_rgb": subscribe_rgb,
        "approx_sync": flag("approx_sync"),
        "topic_queue_size": qsize, "sync_queue_size": qsize,
        "Mem/IncrementalMemory": "false" if localization else "true",
        "database_path": s("database_path")}]
    rtab_remaps = [("scan_cloud", points), ("odom", odom),
                   ("cloud_map", "/rtabmap/cloud_map"), ("map", "/rtabmap/grid_map"),
                   ("mapData", "/rtabmap/mapData")]
    if subscribe_rgb:
        rtab_remaps += [("rgb/image", s("rgb_topic")), ("rgb/camera_info", s("camera_info_topic"))]
    delete = flag("delete_db_on_start") and not localization
    actions.append(Node(
        package="rtabmap_slam", executable="rtabmap", name="rtabmap", output="screen",
        parameters=rtab_params, remappings=rtab_remaps, arguments=["-d"] if delete else []))

    if flag("rviz"):
        actions.append(Node(
            package="rtabmap_viz", executable="rtabmap_viz", name="rtabmap_viz", output="screen",
            parameters=rtab_params, remappings=[("scan_cloud", points), ("odom", odom)]))
    return actions


def generate_launch_description():
    pkg = FindPackageShare("wheelchair_3d_mapping")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("config", default_value=PathJoinSubstitution([pkg, "config", "rtabmap_params.yaml"])),
        DeclareLaunchArgument("points_topic", default_value="/points_merged",
                              description="Merged XT-M60 cloud (base_link). Real raw topics: /xtm60/left|right/points."),
        DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
        DeclareLaunchArgument("odom_topic", default_value="/rtabmap/odom",
                              description="icp mode: icp_odometry output; external mode: existing odom e.g. /wheel/odom."),
        DeclareLaunchArgument("odom_mode", default_value="icp", description="icp | external"),
        DeclareLaunchArgument("frame_id", default_value="base_link"),
        DeclareLaunchArgument("subscribe_rgb", default_value="false",
                              description="Optional camera RGB for texture/visual loop closure; geometry independent of it."),
        DeclareLaunchArgument("rgb_topic", default_value="/camera/left/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/left/camera_info"),
        DeclareLaunchArgument("subscribe_scan_cloud", default_value="true"),
        DeclareLaunchArgument("approx_sync", default_value="true"),
        DeclareLaunchArgument("queue_size", default_value="10"),
        DeclareLaunchArgument("localization", default_value="false"),
        DeclareLaunchArgument("delete_db_on_start", default_value="true",
                              description="Start a fresh map (ignored in localization mode)."),
        DeclareLaunchArgument("database_path", default_value=os.path.expanduser("~/.ros/rtabmap.db")),
        DeclareLaunchArgument("bringup_sensors", default_value="false",
                              description="Also start sensors + dual-lidar fusion (no base/EKF) so icp_odometry uniquely owns odom->base_link."),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=_setup),
    ])
