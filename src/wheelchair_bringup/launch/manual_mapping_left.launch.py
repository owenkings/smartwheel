"""Stage 1 (RViz-first MVP): manual-drive mapping, single LEFT XT-M60.

= manual_teleop (sensors + TF + scan + EKF + watchdog + safety + base + RViz
  TeleopPanel) PLUS dual_lidar fusion (-> /points_merged) PLUS RTAB-Map building
  the 3D cloud map and 2D projected grid while the user drives by hand.

Mapping data flow:
  /points_merged (left-only fallback) + /odometry/filtered (EKF: wheel + IMU)
     -> RTAB-Map (external odom, 零几何配准) -> /rtabmap/cloud_map (3D)  /rtabmap/grid_map (2D)

Control (safety never bypassed):
  RViz TeleopPanel -> /cmd_vel_nav -> safety_supervisor -> /cmd_vel_safe -> base

Odometry = HARDWARE FIRST (纯 EKF 位姿, ICP 已关闭 — req 1.2/1.3):
  robot_localization EKF fuses wheel odometry + H30 IMU → /odometry/filtered → odom->base_link.
  RTAB-Map 工作于「纯外部里程计」模式 (odom_mode=external)：
    - icp_odometry 节点 **不启动** (odom->base_link 仅 EKF 发布, 无 TF 争用)
    - Reg/Strategy=0, RGBD/NeighborLinkRefining=false, RGBD/ProximityBySpace=false
      → RTAB-Map 不做任何几何配准, 直接按 EKF 位姿堆叠点云
  XT-M60 是 120°×60° 窄视场 flash ToF; 平墙方向 ICP 无约束会滑移旋转 yaw (Triangle_Distortion),
  故关闭 ICP 是必要的, 而非可选优化。

SAFETY: motors move only with motion_control_enabled:=true (default false =
read-only). Off-ground/clear-area test first, keep the physical E-stop in reach.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("wheelchair_bringup")
    bringup = get_package_share_directory("wheelchair_bringup")
    mapping = get_package_share_directory("wheelchair_3d_mapping")

    motion_control_enabled = LaunchConfiguration("motion_control_enabled")
    use_rviz = LaunchConfiguration("rviz")
    delete_db = LaunchConfiguration("delete_db_on_start")
    database_path = LaunchConfiguration("database_path")
    radar = LaunchConfiguration("radar")

    manual_teleop_launch = os.path.join(bringup, "launch", "manual_teleop.launch.py")
    fusion_launch = os.path.join(mapping, "launch", "dual_lidar_fusion.launch.py")
    rtabmap_launch = os.path.join(mapping, "launch", "rtabmap_3d_mapping.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument(
            "motion_control_enabled", default_value="false",
            description="HIGH RISK. true lets /cmd_vel_safe write real motor speeds. "
                        "Default false = read-only (verify the chain without moving)."),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("delete_db_on_start", default_value="true",
                              description="Start a fresh map. Set false to append to an existing db."),
        DeclareLaunchArgument(
            "database_path", default_value=os.path.expanduser("~/.ros/rtabmap.db")),
        DeclareLaunchArgument(
            "radar", default_value="left",
            description="Which XT-M60(s) to map with: left | right | both. "
                        "Selects the radar driver(s), scan and the ground-plane "
                        "calibrator (calibrator only runs for the left radar, whose "
                        "URDF joint was removed). Default left."),

        LogInfo(msg="[manual_mapping_left] LEFT XT-M60 (+ optional RIGHT) manual-drive mapping. "
                    "Drive from the RViz TeleopPanel; RTAB-Map builds the map as you go. "
                    "No Nav2 / no autonomous explorer."),

        # 1. Manual teleop base stack (sensors, TF, scan, watchdog, safety,
        #    base, RViz TeleopPanel) WITH the EKF enabled: wheel-odom + IMU is the
        #    primary odom->base_link source (hardware odometry, more accurate than
        #    narrow-FOV ICP on a planar floor). RViz started separately below.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(manual_teleop_launch),
            launch_arguments={
                "motion_control_enabled": motion_control_enabled,
                "rviz": "false",
                "enable_ekf": "true",
                "radar": radar,
            }.items(),
        ),

        # 2. Ground-plane calibrator — FIXED-HEIGHT MODE (req 2.4/2.7).
        #
        # USER-DECIDED MANUAL OVERRIDE of req 2.7: the LEFT radar uses a fixed
        # 0.50 m height and auto ground-plane calibration is DISABLED (the floor
        # is too sparse/noisy to lock; the prior auto-cal got 0/240 valid samples
        # and so never published the TF). ground_plane_calibrator_node is still
        # the SOLE owner of the base_link→xtm60_left_link TF edge (URDF
        # xtm60_left_fixed_joint stays removed, design 方案 A) — in fixed mode it
        # publishes the full static transform ONCE at startup from the fixed
        # height/pitch via StaticTransformBroadcaster (no RANSAC, no lock-failure
        # path). This is what restores the RViz point cloud: with the TF always
        # present the fusion node stops skipping frames (D178) and /points_merged
        # populates.
        #
        # ORDERING: started BEFORE the fusion node so its static (latched) TF is
        # available by the time the fusion node looks up xtm60_left_link→base_link
        # to merge into /points_merged (req 2.4).
        #
        # x/y/yaw reproduce the fixed geometric part formerly carried by the URDF
        # joint (xyz=[0.45, 0.24, *], yaw=0); z is the user-fixed 0.50 m.
        Node(
            package="wheelchair_3d_mapping",
            executable="ground_plane_calibrator",
            name="ground_plane_calibrator",
            output="screen",
            condition=IfCondition(PythonExpression(
                ["'", radar, "' in ('left', 'both')"])),
            parameters=[{
                "input_topic": "/xtm60/left/points",
                "target_frame": "base_link",
                "radar_frame": "xtm60_left_link",
                "x_offset": 0.45,
                "y_offset": 0.24,
                "yaw": 0.0,
                # USER-DECIDED MANUAL OVERRIDE of req 2.7: the LEFT radar uses a
                # FIXED 0.50 m height; auto ground-plane calibration is DISABLED.
                # The floor is too sparse/noisy to lock (a ~0.13 m near surface +
                # 0/240 valid samples), so the auto-cal never published the
                # base_link→xtm60_left_link TF — the fusion node then skipped
                # every frame (D178: no TF → skip), /points_merged stayed empty
                # and RViz showed NO POINT CLOUD. Fixing the height makes the
                # calibrator publish the TF unconditionally at startup, which is
                # what restores the RViz point cloud. fixed_height > 0 routes the
                # node to its no-RANSAC fixed path (no lock-failure / missing-TF
                # path); fixed_pitch_deg=0 keeps the radar level.
                "fixed_height": 0.50,
                "fixed_pitch_deg": 0.0,
            }],
        ),

        # 2b. Ground-plane calibrator for the RIGHT radar — AUTO mode (req 2.1-2.7).
        #
        # Unlike the left radar (fixed mode), the RIGHT radar runs FULL auto
        # ground-plane calibration: it derives BOTH the mount height (z) and the
        # pitch from the first frames' ground plane (RANSAC). The user reported
        # the right radar is aimed slightly UP (ground points tilt instead of
        # lying flat); auto-cal measures that pitch from the lowest plane (the
        # floor — nothing can be below it) and corrects it automatically, so the
        # radar can be re-aimed/moved without editing the URDF (req 2.7).
        #
        # This node is the SOLE owner of base_link→xtm60_right_link (the URDF
        # xtm60_right_fixed_joint was removed, design 方案 A). fixed_height is left
        # at its 0.0 default -> the RANSAC auto path runs (no fixed short-circuit).
        # The calibrator's tuned defaults (400 iters, 15° vertical tol,
        # extrapolation gate, farthest-down prior) target exactly this right-radar
        # case. x/y/yaw reproduce the fixed geometric part the URDF joint carried
        # (xyz=[0.45, -0.24, *], yaw=0); z + pitch come from the ground plane.
        #
        # Started BEFORE the fusion node so its (latched) TF is available when the
        # fusion node looks up xtm60_right_link→base_link.
        Node(
            package="wheelchair_3d_mapping",
            executable="ground_plane_calibrator",
            name="ground_plane_calibrator_right",
            output="screen",
            condition=IfCondition(PythonExpression(
                ["'", radar, "' in ('right', 'both')"])),
            parameters=[{
                "input_topic": "/xtm60/right/points",
                "target_frame": "base_link",
                "radar_frame": "xtm60_right_link",
                # Measured: right radar is 15.65 cm offset front/back from the
                # left radar (left x=0.45). Both face straight forward, lateral
                # spacing ~50 cm (left y=+0.24, right y=-0.24). Right radar is
                # 15.65 cm IN FRONT of the left -> x = 0.45 + 0.156 = 0.606.
                "x_offset": 0.606,
                "y_offset": -0.24,
                "yaw": 0.0,
                # fixed_height=0.0 (default) -> RANSAC auto height+pitch path.
                "recalibrate_period_sec": 0.0,   # calibrate once at startup, then lock
                "settle_frames": 5,
            }],
        ),

        # 3. Dual-lidar fusion (single-left fallback) -> /points_merged.
        #
        # Single lidar fallback confirmed – req 3.3/4.1/4.2
        # allow_single_lidar_fallback=true : fusion node publishes /points_merged
        #   from the left radar alone when the right radar is absent or stale.
        # enable_xtm60_right=false (default) : right radar driver not started,
        #   so only the left XT-M60 (192.168.0.101) is online in Stage 1.
        # This combination satisfies:
        #   req 3.3 – Fusion_Node publishes from left radar when right is absent.
        #   req 4.1 – Pipeline uses only Left_Radar as point-cloud source.
        #   req 4.2 – Pipeline does NOT require the right radar to be online.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fusion_launch),
            launch_arguments={
                "allow_single_lidar_fallback": "true",
            }.items(),
        ),

        # 4. RTAB-Map in EXTERNAL odom mode: it consumes the EKF odometry
        #    (/odometry/filtered, wheel + IMU) as the trajectory. ICP is fully
        #    DISABLED (Reg/Strategy=0, NeighborLinkRefining=false, ProximityBySpace=false).
        #    RTAB-Map 直接按 EKF 位姿堆叠点云, 不做任何几何配准 (req 1.2, 1.3).
        #    icp_odometry 节点不启动; EKF 是 odom->base_link 的唯一发布者 (无 TF 争用).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch),
            launch_arguments={
                "bringup_sensors": "false",
                "points_topic": "/points_merged",
                "imu_topic": "/imu/data",
                "odom_mode": "external",
                "odom_topic": "/odometry/filtered",
                "subscribe_scan_cloud": "true",
                "subscribe_rgb": "false",
                "use_colorizer": "false",
                "localization": "false",
                "delete_db_on_start": delete_db,
                "database_path": database_path,
                "rviz": "false",
            }.items(),
        ),

        # 5. RViz: sensor view + embedded TeleopPanel + map displays.
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", PathJoinSubstitution(
                [bringup_share, "rviz", "manual_mapping_left.rviz"])],
            condition=IfCondition(use_rviz),
        ),
    ])
