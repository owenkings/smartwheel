"""PROVISIONAL RTAB-Map 3D mapping (LiDAR-primary) software pipeline.

This launch assembles and validates the ROS graph contract, but it is not a
formal-map approval gate.  Device timestamps, dynamic/final extrinsics,
dual-LiDAR synchronization, odometry accuracy and runtime performance still
require independent evidence before an output may be labelled FINAL_PRODUCT.

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

Camera adapters are enabled by default for the site profile, but they are kept
out of RTAB-Map's required synchronization path unless subscribe_rgb:=true is
explicitly requested. This keeps /rtabmap/cloud_map driven by LiDAR + ICP even
when USB camera timestamps or frame rates jitter. The optional
rgb_cloud_colorizer_node uses both forward cameras to publish /rgb_cloud_map for
user-facing color inspection. 3D geometry remains LiDAR-primary and never
depends on camera<->LiDAR extrinsics.

Real raw XT-M60 topics are /xtm60/left/points and /xtm60/right/points; they are
fused into points_topic by dual_lidar_cloud_fusion_node. Requires
ros-humble-rtabmap-ros (sudo apt install ros-humble-rtabmap-ros).
"""
import os


def _require_ros_launch_api():
    """Load ROS launch dependencies only when the graph is constructed.

    Contract/unit tests and offline review tools should be able to import this
    module without a sourced ROS overlay.  ``ros2 launch`` still gets the same
    imports, but a missing runtime dependency is reported only when a launch
    description is actually requested.
    """

    try:
        from ament_index_python.packages import get_package_share_directory
        from launch import LaunchDescription
        from launch.actions import (
            DeclareLaunchArgument,
            IncludeLaunchDescription,
            LogInfo,
            OpaqueFunction,
        )
        from launch.launch_description_sources import PythonLaunchDescriptionSource
        from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
        from launch_ros.actions import Node
        from launch_ros.substitutions import FindPackageShare
    except ImportError as exc:  # pragma: no cover - exercised on ROS hosts
        raise RuntimeError(
            "rtabmap_3d_mapping requires ROS 2 launch and launch_ros; "
            "source the ROS environment before building the launch graph"
        ) from exc
    return {
        "get_package_share_directory": get_package_share_directory,
        "LaunchDescription": LaunchDescription,
        "DeclareLaunchArgument": DeclareLaunchArgument,
        "IncludeLaunchDescription": IncludeLaunchDescription,
        "LogInfo": LogInfo,
        "OpaqueFunction": OpaqueFunction,
        "PythonLaunchDescriptionSource": PythonLaunchDescriptionSource,
        "LaunchConfiguration": LaunchConfiguration,
        "PathJoinSubstitution": PathJoinSubstitution,
        "Node": Node,
        "FindPackageShare": FindPackageShare,
    }


def _validate_launch_contract(
    *,
    config_path: str,
    points_topic: str,
    odom_topic: str,
    odom_mode: str,
    frame_id: str,
    queue_size: int,
    bringup_sensors: bool,
    subscribe_scan_cloud: bool,
    subscribe_rgb: bool,
    rgb_topic: str,
    camera_info_topic: str,
    database_path: str,
) -> None:
    """Validate the static RTAB-Map launch contract before node construction.

    This is deliberately independent of ROS graph state: runtime checks still
    have to verify that the selected odometry publisher and TF edges actually
    exist. ``bringup_sensors`` only controls sensor/fusion includes and does
    not imply an odometry owner, so an enclosing launch may pair it with an
    explicit external odom topic. Keeping this function pure makes the
    fail-closed ownership rules unit-testable without starting hardware or a
    ROS process.
    """

    if not config_path:
        raise RuntimeError("config must be non-empty")
    if odom_mode not in ("icp", "external"):
        raise RuntimeError(
            f"INVALID_ODOM_MODE: {odom_mode!r}; expected 'icp' or 'external'"
        )
    if queue_size < 1:
        raise RuntimeError("queue_size must be a positive integer")
    for name, value in (("points_topic", points_topic), ("odom_topic", odom_topic)):
        if not value or not value.startswith("/"):
            raise RuntimeError(f"{name} must be a non-empty absolute ROS topic")
    if not frame_id or frame_id.startswith("/"):
        raise RuntimeError("frame_id must be a non-empty TF frame without a leading '/'")
    if odom_mode == "external" and odom_topic == "/rtabmap/odom":
        raise RuntimeError(
            "INVALID_EXTERNAL_ODOM: odom_mode=external requires an existing "
            "odom_topic (for example /lio/odom or /odometry/filtered), not the "
            "icp_odometry default /rtabmap/odom"
        )
    if not subscribe_rgb and not subscribe_scan_cloud:
        raise RuntimeError(
            "INVALID_INPUT: LiDAR-only RTAB-Map mapping requires "
            "subscribe_scan_cloud=true"
        )
    if subscribe_rgb:
        for name, value in (("rgb_topic", rgb_topic), ("camera_info_topic", camera_info_topic)):
            if not value or not value.startswith("/"):
                raise RuntimeError(f"{name} must be a non-empty absolute ROS topic")
    if not database_path:
        raise RuntimeError("database_path must be non-empty")


def _setup(context, *args, **kwargs):
    ros = _require_ros_launch_api()
    get_package_share_directory = ros["get_package_share_directory"]
    IncludeLaunchDescription = ros["IncludeLaunchDescription"]
    PythonLaunchDescriptionSource = ros["PythonLaunchDescriptionSource"]
    LaunchConfiguration = ros["LaunchConfiguration"]
    Node = ros["Node"]
    LogInfo = ros["LogInfo"]
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
    subscribe_scan_cloud = flag("subscribe_scan_cloud")
    use_colorizer = flag("use_colorizer")
    localization = flag("localization")
    enable_loop_closure = flag("enable_loop_closure")
    use_sim = s("use_sim_time")
    try:
        qsize = int(s("queue_size"))
    except ValueError as exc:
        raise RuntimeError("queue_size must be a positive integer") from exc

    database_path = s("database_path")
    # Fail before constructing any ROS action when the graph contract is
    # ambiguous.  These checks are launch-time guards, not calibration
    # approval or a substitute for runtime TF diagnostics.
    _validate_launch_contract(
        points_topic=points,
        config_path=cfg,
        odom_topic=odom,
        odom_mode=odom_mode,
        frame_id=frame_id,
        queue_size=qsize,
        bringup_sensors=flag("bringup_sensors"),
        subscribe_scan_cloud=subscribe_scan_cloud,
        subscribe_rgb=subscribe_rgb,
        rgb_topic=s("rgb_topic"),
        camera_info_topic=s("camera_info_topic"),
        database_path=database_path,
    )
    delete_db = flag("delete_db_on_start")
    if os.path.exists(database_path):
        if delete_db and not localization:
            actions_warning = LogInfo(
                msg=(
                    "[rtabmap_3d_mapping] WARNING: delete_db_on_start=true "
                    f"will replace existing RTAB-Map database {database_path!r}. "
                    "Use a unique session path before formal mapping."
                )
            )
            # Keep this warning ahead of all node actions; no file is touched
            # by the guard itself.
            actions = [actions_warning]
        elif not delete_db and not localization:
            actions_warning = LogInfo(
                msg=(
                    "[rtabmap_3d_mapping] WARNING: an existing RTAB-Map "
                    f"database {database_path!r} will be appended because "
                    "delete_db_on_start=false; verify this is intentional."
                )
            )
            actions = [actions_warning]
        else:
            actions = []
    else:
        actions = []
    common = {"use_sim_time": use_sim == "true"}

    # Optional self-contained sensors + fusion (NO base/EKF -> icp_odometry is the
    # only odom->base_link publisher). Leave false if your stack already runs them.
    if flag("bringup_sensors"):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "sensors.launch.py")),
            launch_arguments={
                "mode": "real", "enable_xtm60": "false",
                "enable_xtm60_left": s("enable_xtm60_left"),
                "enable_xtm60_right": s("enable_xtm60_right"),
                "enable_imu": s("enable_imu"),
                "enable_ultrasonic": "true" if flag("enable_ultrasonic") else "false",
                "enable_camera": "true" if (subscribe_rgb or use_colorizer) else "false",
            }.items()))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mapping, "launch", "dual_lidar_fusion.launch.py")),
            launch_arguments={
                "use_sim_time": use_sim,
                "allow_single_lidar_fallback": s("allow_single_lidar_fallback"),
                "enable_left_input": s("enable_xtm60_left"),
                "enable_right_input": s("enable_xtm60_right"),
            }.items()))

    # ICP odometry owns odom->base_link (skipped in external-odom mode).
    if odom_mode == "icp":
        actions.append(Node(
            package="rtabmap_odom", executable="icp_odometry", name="icp_odometry", output="screen",
            parameters=[cfg, common, {"frame_id": frame_id, "publish_tf": True,
                                      "topic_queue_size": qsize, "sync_queue_size": qsize}],
            remappings=[("scan_cloud", points), ("imu", imu), ("odom", odom)]))
    else:
        # ``bringup_sensors`` may still be true here: the enclosing launch can
        # start an external wheel/IMU estimator alongside the sensor include.
        # The static guard above only rejects the ambiguous historical default;
        # this log makes the remaining runtime ownership obligation explicit.
        actions.append(LogInfo(msg=f"[rtabmap_3d_mapping] odom_mode=external: using {odom} as odometry; the "
                                   f"external node must own odom->base_link and actually publish "
                                   f"(icp_odometry NOT started)."))

    # rtabmap -> 3D cloud map + graph + projected 2D grid.
    # NOTE: the YAML param file (cfg) is unreliable here -- rtabmap_slam ignores
    # file-provided Grid/RGBD/Reg/* keys in this setup (verified: file params show
    # library defaults at runtime while dict params apply). So the ESSENTIAL
    # mapping params are passed explicitly in this dict, which definitely reaches
    # the node. Keep cfg first for any params that do load; dict overrides win.
    # This dict is the authoritative source (D041: 双真值源隐患已消除 — yaml 标注「dict 为准」).
    essential = {
        # RTAB-Map upstream definition: Reg/Strategy 0=visual, 1=ICP,
        # 2=visual+ICP. This LiDAR-only route therefore needs strategy 1 for
        # geometric proximity/loop constraints. It stays opt-in until a real
        # right-lidar small-loop bag can reject false constraints.
        "Reg/Strategy": "1" if enable_loop_closure else "0",
        "Reg/Force3DoF": "true",       # Planar chair: constrain loop registration to x/y/yaw.
        "Mem/IncrementalMemory": "true",
        "Mem/STMSize": "30",
        # Neighbor refinement stays off so ICP cannot rewrite every odometry
        # edge. The guarded switch only enables spatial proximity constraints.
        "RGBD/NeighborLinkRefining": "false",
        "RGBD/ProximityBySpace": "true" if enable_loop_closure else "false",
        "RGBD/ProximityPathMaxNeighbors": "10" if enable_loop_closure else "0",
        "RGBD/ProximityOdomGuess": "true" if enable_loop_closure else "false",
        "RGBD/LocalRadius": "3.0",
        "RGBD/AngularUpdate": "0.05",
        "RGBD/LinearUpdate": "0.05",
        "RGBD/OptimizeMaxError": "3.0",
        "Rtabmap/DetectionRate": "1.0",
        # Match the topic contracts explicitly.  RTAB-Map's parameter-file
        # loading has differed across Humble builds, so these ownership/QoS
        # values are kept in the launch dictionary as the runtime authority.
        "qos_scan": 2,              # sensor-data / BEST_EFFORT cloud input
        "qos_odom": 1,              # reliable odometry input
        "Grid/FromDepth": "false",
        "Icp/VoxelSize": "0.05",
        "Icp/PointToPlane": "true",
        "Icp/MaxCorrespondenceDistance": "0.5",
        "Icp/Iterations": "30",
        "Icp/CorrespondenceRatio": "0.1",
        # --- 2D occupancy grid: ground/obstacle split (the wide-FOV flash camera
        #     sees lots of floor; without this the floor becomes obstacle and the
        #     2D map goes all-black). ---
        "Grid/Sensor": "0",
        "Grid/3D": "false",
        "Grid/NormalsSegmentation": "true",
        "Grid/NormalK": "10",
        "Grid/MaxGroundAngle": "30",
        "Grid/ClusterRadius": "0.1",
        "Grid/MinClusterSize": "5",
        "Grid/CellSize": "0.05",
        "Grid/RangeMax": "8.0",
        "Grid/MaxGroundHeight": "0.15",
        "Grid/MaxObstacleHeight": "1.8",
        "Grid/GroundIsObstacle": "false",
        "Grid/RayTracing": "true",
        "GridGlobal/MinSize": "20.0",
    }
    rtab_params = [cfg, common, essential, {
        "frame_id": frame_id,
        # RTAB-Map owns the global correction. With external FAST-LIO odometry,
        # the odometry message names camera_init as its parent frame, so this is
        # map->camera_init. The FAST-LIO identity bridge must be disabled then.
        "map_frame_id": "map",
        "publish_tf": True,
        "subscribe_scan_cloud": subscribe_scan_cloud,
        "subscribe_rgb": subscribe_rgb,
        # RTAB-Map defaults subscribe_depth=true; for LiDAR-only mapping we must
        # explicitly disable depth/rgbd, otherwise the node waits on /rgb/image +
        # /depth/image and reports "Did not receive data since 5 seconds".
        "subscribe_depth": subscribe_rgb,
        "subscribe_rgbd": False,
        "approx_sync": flag("approx_sync"),
        "topic_queue_size": qsize, "sync_queue_size": qsize,
        "Mem/IncrementalMemory": "false" if localization else "true",
        "database_path": database_path}]
    rtab_remaps = [("scan_cloud", points), ("odom", odom),
                   ("cloud_map", "/rtabmap/cloud_map"), ("map", "/rtabmap/grid_map"),
                   ("mapData", "/rtabmap/mapData")]
    if subscribe_rgb:
        rtab_remaps += [("rgb/image", s("rgb_topic")), ("rgb/camera_info", s("camera_info_topic"))]
    delete = delete_db and not localization
    actions.append(Node(
        package="rtabmap_slam", executable="rtabmap", name="rtabmap", output="screen",
        parameters=rtab_params, remappings=rtab_remaps, arguments=["-d"] if delete else []))

    if use_colorizer:
        actions.append(Node(
            package="wheelchair_3d_mapping", executable="rgb_cloud_colorizer_node",
            name="rgb_cloud_colorizer_node", output="screen",
            parameters=[
                os.path.join(mapping, "config", "rgb_colorizer.yaml"),
                {"use_sim_time": use_sim == "true"},
            ],
        ))

    if flag("rviz"):
        actions.append(Node(
            package="rtabmap_viz", executable="rtabmap_viz", name="rtabmap_viz", output="screen",
            # The visualization process is not a TF owner.  Keep the value
            # explicit so a distro-specific default cannot create a second
            # map->odom publisher when the backend is running.
            parameters=[*rtab_params, {"publish_tf": False}],
            remappings=[("scan_cloud", points), ("odom", odom)]))
    return actions


def generate_launch_description():
    ros = _require_ros_launch_api()
    FindPackageShare = ros["FindPackageShare"]
    LaunchDescription = ros["LaunchDescription"]
    DeclareLaunchArgument = ros["DeclareLaunchArgument"]
    OpaqueFunction = ros["OpaqueFunction"]
    PathJoinSubstitution = ros["PathJoinSubstitution"]
    pkg = FindPackageShare("wheelchair_3d_mapping")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("config", default_value=PathJoinSubstitution([pkg, "config", "rtabmap_params.yaml"])),
        DeclareLaunchArgument("points_topic", default_value="/points_merged",
                              description="Merged XT-M60 cloud (base_link). Real raw topics: /xtm60/left|right/points."),
        DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
        DeclareLaunchArgument(
            "enable_imu",
            default_value="false",
            description="Start H30 when bringup_sensors is true. Current right-only baseline leaves it off.",
        ),
        DeclareLaunchArgument("odom_topic", default_value="/rtabmap/odom",
                              description="icp mode: icp_odometry output; external mode: existing odom e.g. /wheel/odom."),
        DeclareLaunchArgument("odom_mode", default_value="icp", description="icp | external"),
        DeclareLaunchArgument("frame_id", default_value="base_link"),
        DeclareLaunchArgument("subscribe_rgb", default_value="false",
                              description="Experimental: synchronize one RGB camera into RTAB-Map. Keep false for robust LiDAR 3D mapping."),
        DeclareLaunchArgument("rgb_topic", default_value="/camera/left/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/left/camera_info"),
        DeclareLaunchArgument("use_colorizer", default_value="false",
                              description="Publish /rgb_cloud_map using the two forward cameras for map coloring."),
        DeclareLaunchArgument("enable_ultrasonic", default_value="false",
                              description="Start ultrasonic adapter when bringup_sensors is true."),
        DeclareLaunchArgument("enable_xtm60_left", default_value="false",
                              description="Start the left XT-M60 when bringup_sensors is true. Disabled in the current baseline."),
        DeclareLaunchArgument("enable_xtm60_right", default_value="true",
                              description="Start the right XT-M60 when bringup_sensors is true. Current mapping baseline."),
        DeclareLaunchArgument("allow_single_lidar_fallback", default_value="true",
                              description="Allow mapping from one lidar. Enable only for an approved single-lidar profile."),
        DeclareLaunchArgument("subscribe_scan_cloud", default_value="true"),
        DeclareLaunchArgument("approx_sync", default_value="true"),
        DeclareLaunchArgument("queue_size", default_value="10"),
        DeclareLaunchArgument("localization", default_value="false"),
        DeclareLaunchArgument(
            "enable_loop_closure",
            default_value="false",
            description=(
                "Enable guarded LiDAR ICP spatial loop constraints. Keep false "
                "until a right-lidar small-loop bag is available for validation."
            ),
        ),
        DeclareLaunchArgument("delete_db_on_start", default_value="true",
                              description="Start a fresh map (ignored in localization mode)."),
        DeclareLaunchArgument("database_path", default_value=os.path.expanduser("~/.ros/rtabmap.db")),
        DeclareLaunchArgument("bringup_sensors", default_value="false",
                              description="Also start sensors + dual-lidar fusion (no base/EKF) so icp_odometry uniquely owns odom->base_link."),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=_setup),
    ])
