"""RIGHT XT-M60 manual-drive mapping — DIAGNOSTIC route (calibration NOT approved).

WHY THIS FILE EXISTS
--------------------
The reviewed route for the right radar is fail-closed in three places, so every
historical right-side command aborts before a single node starts:

  1. launch/right_lidar_stage1_mapping.launch.py   -> raises unconditionally
     (manual_mapping_lio_right.launch.py is only a forwarding shell for it)
  2. launch/sensors.launch.py::_block_unapproved_right_lidar
     -> mode=real + enable_xtm60[_right] raises BLOCKED_CONFLICT
  3. wheelchair_3d_mapping/launch/fast_lio_mapping.launch.py::_setup
     -> radar=right raises BLOCKED_CONFLICT

Reason for the block: right_lidar_stage1_calibration only ever produced a GROUND
plane fit (height / pitch / roll). Longitudinal x and yaw were never measured, so
the contract has no runtime-eligible base_link->xtm60_right_link transform and
config/right_lidar_stage1_calibration_contract.json stays BLOCKED_CONFLICT.

This launch does NOT modify, weaken or re-approve any of those gates. It is a
separate, explicitly-labelled diagnostic path that wires the same nodes directly
so the operator can drive the chair and watch live 3D/2D mapping. It exists to
answer "does the hardware work", not "is the map metrically correct".

ACCEPTED INACCURACY: this route now uses diagnostic_measured_layout.yaml as one
source for the URDF and derived IMU-frame FAST-LIO extrinsics. Both radar heights
are the user's 0.735 m measurement, with zero relative fore/aft displacement.
Absolute x, yaw and retained provisional rotations are NOT newly calibrated.
The old 0.499 m installation-epoch candidate remains historical, not the active
diagnostic mount. Do not promote maps made here.

TF OWNERSHIP with pose_owner=fastlio (historical diagnostic only)
  camera_init -> body        FAST-LIO (dynamic)
  body        -> base_link   static bridge, full inverse of calibrated base->imu
  map         -> camera_init initial base->IMU reference bridge
  base_link   -> xtm60_right_link   URDF / robot_state_publisher
The diagnostic wheel/IMU EKF and wheel base both run with publish_tf:=false;
only FAST-LIO owns base_link. Wheel forward-speed updates inside FAST-LIO are
separate from this comparison EKF and never consume its correlated IMU output.

With pose_owner=wheel_imu, the wheel/IMU EKF estimates continuous pose; its
freshness/feedback gate alone broadcasts odom->base_link. RTAB-Map owns
map->odom. FAST-LIO is isolated under /lio/shadow with TF disabled. No static
camera_init/body bridges or LIO-derived occupancy grid are started in that mode.
The wheel-primary path is an explicit planar indoor diagnostic, not approval of
the formally gated full-3D calibration/navigation pipeline.

CHAIN: RViz TeleopPanel (W/A/S/D, Space=stop) -> /cmd_vel_nav
       -> safety_supervisor -> /cmd_vel_safe -> zlac8030 base
Motors move ONLY with motion_control_enabled:=true (default false = read-only).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix
from wheelchair_3d_mapping.diagnostic_layout import load_diagnostic_layout

# Right-unit deployment identity (deployment_identity block of the contract).
RIGHT_DEVICE_IP = "192.168.1.101"
RIGHT_HOST_BIND_IP = "192.168.1.100"
RIGHT_BIND_PORT = "7687"


def _setup(context, *args, **kwargs):
    bringup = get_package_share_directory("wheelchair_bringup")
    description = get_package_share_directory("wheelchair_description")
    mapping = get_package_share_directory("wheelchair_3d_mapping")
    layout_profile = os.path.join(description, "config", "diagnostic_measured_layout.yaml")
    layout = load_diagnostic_layout(layout_profile, radar="right")
    bringup_prefix = FindPackagePrefix("wheelchair_bringup").perform(context)
    bindshim = os.path.join(
        bringup_prefix, "lib", "wheelchair_bringup", "libxt_bindshim.so"
    )

    def s(name):
        return LaunchConfiguration(name).perform(context).strip()

    motion = s("motion_control_enabled").lower()
    enable_camera = s("enable_camera").lower() == "true"
    enable_ultrasonic = s("enable_ultrasonic").lower() == "true"
    lio_wheel_aiding = s("lio_wheel_aiding").lower() == "true"
    lio_zupt = s("lio_zupt").lower() == "true"
    pose_owner = s("pose_owner").lower()
    if pose_owner not in ("fastlio", "wheel_imu"):
        raise RuntimeError("pose_owner must be fastlio or wheel_imu")
    wheel_primary = pose_owner == "wheel_imu"
    run_lio = not wheel_primary or s("enable_fastlio_shadow").lower() == "true"
    if wheel_primary and s("use_wheel_imu_ekf").lower() != "true":
        raise RuntimeError("wheel_imu pose owner requires use_wheel_imu_ekf=true")

    # Which camera slots to start. One isolated process per role, so an absent or
    # faulty unit cannot drag the others down: a dead device reconnects every
    # camera_reconnect_interval_sec (3 s) and that churn is visible on the shared
    # USB hub as reduced throughput on the healthy streams.
    camera_roles = [
        role.strip().lower()
        for role in s("camera_roles").split(",")
        if role.strip()
    ]
    unknown = [r for r in camera_roles if r not in ("front", "left", "right", "rear")]
    if unknown:
        raise RuntimeError(
            f"INVALID_CAMERA_ROLES: {unknown}; expected front, left, right, rear"
        )

    # Explicit diagnostic layout opt-in; the formally approved route's
    # BLOCKED_CONFLICT contract is unchanged.
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ",
                os.path.join(description, "urdf", "wheelchair.urdf.xacro"),
                " diagnostic_measured_layout:=true",
                " diagnostic_layout_profile:=", layout_profile,
            ]),
            value_type=str,
        )
    }

    actions = [
        LogInfo(msg=(
            "[right_lidar_diag_mapping] DIAGNOSTIC ROUTE. right_lidar_stage1_calibration "
            "is BLOCKED_CONFLICT (x/yaw uncalibrated); the mount TF used here is a "
            "provisional candidate. Hardware/behaviour check only - do not promote maps. "
            "Equal-height measured-layout profile: z=0.735 m; yaw/x not calibrated. "
            f"motion_control_enabled={motion} (true = motors may move)."
        )),
        LogInfo(msg=(
            "[pose ownership] health-gated wheel/IMU EKF -> odom->base; RTAB -> map->odom; "
            f"FAST-LIO shadow enabled={run_lio}, never owns vehicle TF. "
            "Planar indoor trajectory, full XYZ cloud."
            if wheel_primary else
            "[pose ownership] FAST-LIO -> camera_init->body; wheel/IMU EKF "
            "comparison only; NO RTAB-Map loop closure in this mode."
        )),

        # --- TF tree from the URDF (includes base_link->xtm60_right_link) ---
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="robot_state_publisher", output="screen",
            parameters=[robot_description],
        ),

        # --- Right XT-M60 driver. Bound to the right subnet via the bind shim,
        #     publishing /xtm60/right/points. This is a single-radar route, so
        #     disable the periodic stop/start phase re-alignment that exists only
        #     to keep two unsynchronised radar streams apart. ---
        Node(
            package="wheelchair_sensors", executable="xtm60_adapter_node",
            name="xtm60_right_adapter_node", output="screen",
            parameters=[
                os.path.join(bringup, "config", "xtm60_right.yaml"),
                {
                    "mode": "real",
                    "phase_realign_interval_sec": 0.0,
                    # Same-pixel range deltas are a useful diagnostic while
                    # stationary, but legitimate driving changes them. Keep the
                    # absolute validity remains a hard gate; temporal and
                    # relative changes are reported without dropping moving
                    # frames on this route.
                    "quality_temporal_hard_reject": False,
                },
            ],
            remappings=[
                ("/xtm60/points", "/xtm60/right/points"),
                ("/xtm60/points_rejected", "/xtm60/right/points_rejected"),
                ("/xtm60/quality", "/xtm60/right/quality"),
                ("/xtm60/status", "/xtm60/right/status"),
                ("/xtm60/timing", "/xtm60/right/timing"),
                ("/xtm60/phase", "/xtm60/right/phase"),
            ],
            additional_env={
                "LD_PRELOAD": bindshim,
                "XT_BIND_IP": RIGHT_HOST_BIND_IP,
                "XT_BIND_PORT": RIGHT_BIND_PORT,
            },
        ),

        # --- H30 IMU (~200 Hz), FAST-LIO's /imu/data source. ---
        Node(
            package="wheelchair_sensors", executable="imu_adapter_node",
            name="imu_adapter_node", output="screen",
            parameters=[
                os.path.join(bringup, "config", "h30_imu.yaml"),
                {"mode": "real"},
            ],
        ),
    ]

    if enable_ultrasonic:
        actions.append(Node(
            package="wheelchair_sensors", executable="ultrasonic_adapter_node",
            name="ultrasonic_adapter_node", output="screen",
            parameters=[
                os.path.join(bringup, "config", "ultrasonic.yaml"),
                {"mode": "real"},
            ],
        ))

    if enable_camera:
        actions.append(LogInfo(msg=(
            f"[right_lidar_diag_mapping] camera roles: {', '.join(camera_roles)}. "
            "Slot names are compatibility labels, not physical directions "
            "(see camera_quad.yaml): left->left-front, right->right-front, "
            "front->left-side, rear->right-side."
        )))
        # One isolated process per stream so a damaged camera cannot stall the rest.
        for role in camera_roles:
            actions.append(Node(
                package="wheelchair_sensors", executable="camera_adapter_node",
                namespace=f"camera_{role}_worker", name="camera_adapter_node",
                output="screen",
                parameters=[
                    os.path.join(bringup, "config", "camera_quad.yaml"),
                    {"mode": "real", "enabled_cameras": [role]},
                ],
            ))

    actions += [
        # --- /xtm60/right/points -> /scan_right -> /scan (RViz polar view, and the
        #     input a future Nav2/costmap layer would use). ---
        Node(
            package="wheelchair_perception", executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan_right_node", output="screen",
            parameters=[os.path.join(bringup, "config", "pointcloud_to_scan_right.yaml")],
        ),
        Node(
            package="wheelchair_perception", executable="scan_merger_node",
            name="scan_merger_node", output="screen",
            parameters=[os.path.join(bringup, "config", "scan_merger_right_only.yaml")],
        ),

        # --- 3D: FAST-LIO2 (LiDAR-inertial). Adapter reshapes the flash-ToF frame
        #     as whole-frame XYZI; no synthetic per-point time or ring. ---
        Node(
            package="wheelchair_3d_mapping", executable="lio_cloud_adapter",
            name="lio_cloud_adapter_node", output="screen",
            condition=IfCondition(str(run_lio).lower()),
            parameters=[{
                "input_topic": "/xtm60/right/points",
                "output_topic": "/lio/cloud_in",
                "restamp_to_now": False,
                "add_zero_time_field": False,
                "min_range": 0.3,
                "max_range": 12.0,
                "output_qos": "best_effort",
            }],
        ),
        Node(
            package="fast_lio", executable="fastlio_mapping",
            name="fastlio_mapping", output="screen",
            condition=IfCondition(str(run_lio).lower()),
            parameters=[os.path.join(mapping, "config", "xtm60_right_lio.yaml"), {
                "mapping.extrinsic_R": layout["extrinsic_R"],
                "mapping.extrinsic_T": layout["extrinsic_T"],
                "wheel_update.enabled": False if wheel_primary else lio_wheel_aiding,
                "zupt.enabled": False if wheel_primary else lio_zupt,
                "publish.tf_en": not wheel_primary,
                "wheel_update.base_to_imu_R": layout["base_to_imu_R"],
                "wheel_update.base_to_imu_T": layout["base_to_imu_T"],
            }],
            remappings=([
                ("/Odometry", "/lio/shadow/odometry"),
                ("/path", "/lio/shadow/path"),
                ("/cloud_registered", "/lio/shadow/cloud_registered"),
                ("/cloud_registered_body", "/lio/shadow/cloud_registered_body"),
                ("/cloud_effected", "/lio/shadow/cloud_effected"),
                ("/Laser_map", "/lio/shadow/laser_map"),
                ("map_save", "/lio/shadow/map_save"),
            ] if wheel_primary else []),
        ),
        # FAST-LIO hardcodes camera_init/body in C++, so bridge them statically.
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="lio_map_to_camera_init", output="screen",
            condition=IfCondition(str(not wheel_primary).lower()),
            # camera_init is the INITIAL IMU frame, not the ground plane.
            # Use the same base->IMU transform as the URDF so the initial
            # map->base chain is identity when combined with body->base below.
            # This is a presentation/reference-frame alignment, not a new
            # sensor calibration or a correction to the estimated trajectory.
            arguments=[str(v) for v in (
                layout["map_to_camera_init_translation"] +
                layout["map_to_camera_init_quaternion"])] + ["map", "camera_init"],
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="lio_body_to_base_link", output="screen",
            condition=IfCondition(str(not wheel_primary).lower()),
            # Full inverse of base_link->imu_link, including the measured H30
            # roll/pitch correction; quaternion form avoids Euler order ambiguity.
            arguments=[str(v) for v in (
                layout["body_to_base_translation"] +
                layout["body_to_base_quaternion"])] + ["body", "base_link"],
        ),

        # --- 2D: project FAST-LIO's registered cloud into an occupancy grid. ---
        Node(
            package="wheelchair_3d_mapping", executable="cloud_to_occupancy_grid_node",
            name="cloud_to_occupancy_grid_node", output="screen",
            condition=IfCondition(str(not wheel_primary).lower()),
            parameters=[
                os.path.join(mapping, "config", "cloud_to_occupancy_grid.yaml"),
                {"input_cloud_topic": "/cloud_registered", "map_frame": "map"},
            ],
        ),

        # --- Diagnostics + safety chain. ---
        Node(
            package="wheelchair_diagnostics", executable="sensor_watchdog_node",
            name="sensor_watchdog_node", output="screen",
            parameters=[os.path.join(bringup, "config", "diagnostics_right_lidar_mapping.yaml")],
        ),
        Node(
            package="wheelchair_safety", executable="emergency_stop_node",
            name="emergency_stop_node", output="screen",
            parameters=[os.path.join(bringup, "config", "safety_params_manual_mapping.yaml")],
        ),
        Node(
            package="wheelchair_safety", executable="safety_supervisor_node",
            name="safety_supervisor_node", output="screen",
            parameters=[
                os.path.join(bringup, "config", "safety_params_manual_mapping.yaml"),
                {
                    "require_localization_healthy": False,
                    # Must match what the RViz panel publishes, or W/A/S/D does nothing.
                    "cmd_vel_nav_topic": "/cmd_vel_nav",
                },
            ],
        ),

        # --- Base. Never a TF owner in either mapping mode. ---
        Node(
            package="wheelchair_base", executable="zlac8030_driver_node",
            name="zlac8030_driver_node", output="screen",
            parameters=[
                os.path.join(bringup, "config", "zlac8030_base.yaml"),
                {
                    "mode": "real",
                    "publish_tf": False,
                    "hold_zero_before_motion_init": False,
                    "allow_manual_push_mode": wheel_primary,
                    "motion_control_enabled": motion == "true",
                },
            ],
        ),

        # --- Wheel + IMU: estimate only; never publish unchecked predictions as TF.
        # In wheel-primary mode the helper's health gate alone forwards its
        # fresh, controller-confirmed poses to TF and RTAB-Map.
        Node(
            package="robot_localization", executable="ekf_node",
            name="wheel_imu_ekf", output="screen",
            parameters=[
                os.path.join(bringup, "config", "right_diag_wheel_imu_ekf.yaml"),
                {"publish_tf": False},
            ],
            condition=IfCondition(LaunchConfiguration("use_wheel_imu_ekf")),
        ),

        # --- RViz last, after TF/topics exist: 3D cloud + 2D grid + W/A/S/D panel
        #     + camera views. right_diag_mapping.rviz rather than
        #     manual_mapping_lio_right.rviz: the latter ships no QMainWindow State,
        #     so Qt collapses all four camera docks to zero height and the image
        #     panels can never show anything.
        #
        #     on_exit=Shutdown: RViz is the operator's console, so closing its
        #     window ends the session. Without this, closing RViz only ends one
        #     node - the other 15 keep running and keep writing to the terminal,
        #     which looks like a session that cannot be stopped. ---
        TimerAction(period=6.0, actions=[Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=[
                "-d", os.path.join(bringup, "rviz", "wheel_imu_mapping.rviz"
                                   if wheel_primary else "right_diag_mapping.rviz"),
                "--qwindowgeometry", s("rviz_window_geometry"),
            ],
            condition=IfCondition(LaunchConfiguration("rviz")),
            on_exit=Shutdown(reason="RViz was closed by the operator"),
        )]),
    ]
    if wheel_primary:
        from wheelchair_3d_mapping.wheel_primary_pipeline import build_wheel_primary_nodes
        actions += build_wheel_primary_nodes(
            bringup, mapping, layout, s("database_path"), enable_lio_monitor=run_lio)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "motion_control_enabled", default_value="false",
            description="HIGH RISK. true lets /cmd_vel_safe write real motor speeds. "
                        "Default false = read-only sensor/mapping check."),
        DeclareLaunchArgument(
            "use_wheel_imu_ekf", default_value="true",
            description="Run the wheel/IMU EKF with its own TF disabled. Required in "
                        "wheel_imu mode, where the health gate owns odom->base_link.",
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "enable_fastlio_shadow", default_value="false",
            description="Optional recorded-data comparison only in wheel_imu mode; "
                        "off by default to avoid unnecessary CPU and debug-log load."),
        DeclareLaunchArgument(
            "pose_owner", default_value="fastlio",
            description="fastlio: historical diagnostic; wheel_imu: planar wheel/IMU "
                        "primary + RTAB loops + isolated FAST shadow. Use the dedicated "
                        "run_wheel_imu_mapping.sh operator entry for wheel-primary mode."),
        DeclareLaunchArgument(
            "database_path", default_value="",
            description="Required fresh RTAB database path in wheel_imu mode; "
                        "the dedicated operator script supplies a unique session path."),
        DeclareLaunchArgument(
            "lio_wheel_aiding", default_value="true",
            description="FAST-LIO forward-speed observation, not wheel-owned pose. "
                        "Disable only for matched recorded-data comparisons."),
        DeclareLaunchArgument(
            "lio_zupt", default_value="true",
            description="Bounded stationary velocity observation in legacy fastlio mode. "
                        "Forced off in wheel_imu mode's isolated FAST-LIO shadow."),
        DeclareLaunchArgument(
            "enable_camera", default_value="true",
            description="UVC camera streams for the four RViz image panels. Set false "
                        "for a mapping-only session with the lowest CPU/USB load "
                        "(this host browns out under current spikes, see steering "
                        "orin-host-ops)."),
        DeclareLaunchArgument(
            "camera_roles", default_value="front,left,right,rear",
            description="Comma-separated camera slots to start: front, left, right, "
                        "rear. All four work over compressed transport - measured "
                        "front 29.9 / left 29.0 / right 23.1 / rear 25.0 Hz "
                        "(auto_test/20260903_173000_right_diag_route/"
                        "compressed_four.txt). Drop a role only to reduce load or to "
                        "isolate a suspect unit."),
        DeclareLaunchArgument(
            "enable_ultrasonic", default_value="true",
            description="Ultrasonic ring feeding the safety supervisor."),
        DeclareLaunchArgument(
            "rviz_window_geometry", default_value="1920x1080+0+0",
            description="Qt main-window geometry for the Orin operator display."),
        OpaqueFunction(function=_setup),
    ])
