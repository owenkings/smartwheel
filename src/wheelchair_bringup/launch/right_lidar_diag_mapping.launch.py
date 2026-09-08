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

ACCEPTED INACCURACY: the mount transform used here (URDF's blocked candidate,
xyz 0.606 -0.24 0.499) is geometrically self-consistent with the FAST-LIO
extrinsic in xtm60_right_lio.yaml (verified identical to 6e-7, floor-levelling
checked: tilt 0.00 deg) but its x/yaw component is UNCALIBRATED. Expect a
systematic longitudinal/heading offset. Do not promote maps made here.

TF OWNERSHIP (single parent for base_link)
  camera_init -> body        FAST-LIO (dynamic)
  body        -> base_link   static bridge, full inverse of calibrated base->imu
  map         -> camera_init identity bridge
  base_link   -> xtm60_right_link   URDF / robot_state_publisher
No EKF, and the wheel base runs with publish_tf:=false, so nothing else claims
base_link.

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

# Right-unit deployment identity (deployment_identity block of the contract).
RIGHT_DEVICE_IP = "192.168.1.101"
RIGHT_HOST_BIND_IP = "192.168.1.100"
RIGHT_BIND_PORT = "7687"


def _setup(context, *args, **kwargs):
    bringup = get_package_share_directory("wheelchair_bringup")
    description = get_package_share_directory("wheelchair_description")
    mapping = get_package_share_directory("wheelchair_3d_mapping")
    bringup_prefix = FindPackagePrefix("wheelchair_bringup").perform(context)
    bindshim = os.path.join(
        bringup_prefix, "lib", "wheelchair_bringup", "libxt_bindshim.so"
    )

    def s(name):
        return LaunchConfiguration(name).perform(context).strip()

    motion = s("motion_control_enabled").lower()
    enable_camera = s("enable_camera").lower() == "true"
    enable_ultrasonic = s("enable_ultrasonic").lower() == "true"

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

    # robot_state_publisher with the right-radar link switched ON. That xacro arg
    # is named "..._for_mock" because the reviewed model must not ship this
    # provisional TF by default; here it is enabled deliberately and only for
    # this diagnostic route.
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ",
                os.path.join(description, "urdf", "wheelchair.urdf.xacro"),
                " include_blocked_right_lidar_for_mock:=true",
            ]),
            value_type=str,
        )
    }

    actions = [
        LogInfo(msg=(
            "[right_lidar_diag_mapping] DIAGNOSTIC ROUTE. right_lidar_stage1_calibration "
            "is BLOCKED_CONFLICT (x/yaw uncalibrated); the mount TF used here is a "
            "provisional candidate. Hardware/behaviour check only - do not promote maps. "
            f"motion_control_enabled={motion} (true = motors may move)."
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
            parameters=[os.path.join(mapping, "config", "xtm60_right_lio.yaml")],
        ),
        # FAST-LIO hardcodes camera_init/body in C++, so bridge them statically.
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="lio_map_to_camera_init", output="screen",
            arguments=["0", "0", "0", "0", "0", "0", "map", "camera_init"],
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="lio_body_to_base_link", output="screen",
            # Full inverse of base_link->imu_link, including the measured H30
            # roll/pitch correction; quaternion form avoids Euler order ambiguity.
            arguments=[
                "0.000495116765", "-0.004454081453", "-0.449977683911",
                "-0.004949042248", "-0.000550123085", "0.000002722616",
                "0.999987602092", "body", "base_link",
            ],
        ),

        # --- 2D: project FAST-LIO's registered cloud into an occupancy grid. ---
        Node(
            package="wheelchair_3d_mapping", executable="cloud_to_occupancy_grid_node",
            name="cloud_to_occupancy_grid_node", output="screen",
            parameters=[
                os.path.join(mapping, "config", "cloud_to_occupancy_grid.yaml"),
                {"input_cloud_topic": "/cloud_registered", "map_frame": "camera_init"},
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

        # --- Base. publish_tf:=false because FAST-LIO owns base_link. ---
        Node(
            package="wheelchair_base", executable="zlac8030_driver_node",
            name="zlac8030_driver_node", output="screen",
            parameters=[
                os.path.join(bringup, "config", "zlac8030_base.yaml"),
                {
                    "mode": "real",
                    "publish_tf": False,
                    "hold_zero_before_motion_init": False,
                    "motion_control_enabled": motion == "true",
                },
            ],
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
                "-d", os.path.join(bringup, "rviz", "right_diag_mapping.rviz"),
                "--qwindowgeometry", s("rviz_window_geometry"),
            ],
            condition=IfCondition(LaunchConfiguration("rviz")),
            on_exit=Shutdown(reason="RViz was closed by the operator"),
        )]),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "motion_control_enabled", default_value="false",
            description="HIGH RISK. true lets /cmd_vel_safe write real motor speeds. "
                        "Default false = read-only sensor/mapping check."),
        DeclareLaunchArgument("rviz", default_value="true"),
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
