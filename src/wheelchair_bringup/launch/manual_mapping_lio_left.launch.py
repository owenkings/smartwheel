"""Top-level: manual-drive FAST-LIO mapping, single LEFT XT-M60 (spec Task 8/9/11).

Replaces the old RTAB-Map zero-registration chain (manual_mapping_left.launch.py)
with the LiDAR-inertial FAST-LIO chain. Brings up:

  1. manual_teleop base stack WITHOUT the EKF (enable_ekf:=false) so FAST-LIO is
     the single odom/pose owner — no TF fight (Task 8). Sensors (left XT-M60 +
     IMU + ultrasonic + camera), scan, safety chain, base (publish_tf:=false),
     RViz TeleopPanel source. EKF is OFF: FAST-LIO owns base_link's pose via its
     frame bridges (map->camera_init, body->base_link) from fast_lio_mapping.
  2. fast_lio_mapping: adapter (/xtm60/left/points -> /lio/cloud_in) + fastlio +
     frame bridges + base_link->xtm60_left_link static TF.
  3. cloud_to_occupancy_grid: /cloud_registered -> /map_2d_from_3d for Nav2 (Task 9).
  4. RViz with the new LIO mapping layout (Task 10).

TF ownership (Task 8): with enable_ekf:=false and base publish_tf:=false, the ONLY
publisher chain to base_link is FAST-LIO (camera_init->body dynamic + body->base_link
static). No odom->base_link from EKF/ZLAC. Fixed frame = map (bridged to camera_init).

SAFETY: motors move only with motion_control_enabled:=true (default false).
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

    motion = LaunchConfiguration("motion_control_enabled")
    use_rviz = LaunchConfiguration("rviz")
    enable_loop_backend = LaunchConfiguration("enable_loop_backend")

    manual_teleop_launch = os.path.join(bringup, "launch", "manual_teleop.launch.py")
    fast_lio_launch = os.path.join(mapping, "launch", "fast_lio_mapping.launch.py")
    rtabmap_launch = os.path.join(mapping, "launch", "rtabmap_3d_mapping.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument(
            "motion_control_enabled", default_value="false",
            description="HIGH RISK. true lets /cmd_vel_safe write real motor speeds. "
                        "Default false = read-only."),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "enable_loop_backend", default_value="false",
            description="Optional RTAB-Map loop-closure/export backend consuming FAST-LIO "
                        "pose (Task 12). Default off; FAST-LIO alone produces the map."),

        LogInfo(msg="[manual_mapping_lio_left] LEFT XT-M60 FAST-LIO (LiDAR-inertial) "
                    "manual-drive mapping. EKF OFF -> FAST-LIO owns the pose. "
                    "Drive from the RViz TeleopPanel."),

        # 1. Base/sensor/safety stack, EKF OFF (FAST-LIO is the pose owner).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(manual_teleop_launch),
            launch_arguments={
                "motion_control_enabled": motion,
                "rviz": "false",
                "enable_ekf": "false",       # Task 8: FAST-LIO owns odom/pose, not EKF.
                "radar": "left",
                # manual_mapping_lio_left.rviz embeds wheelchair_bringup/TeleopPanel,
                # which publishes /cmd_vel_nav (not the workbench /teleop/cmd_vel).
                "teleop_topic": "/cmd_vel_nav",
                # Task 8 for real: FAST-LIO owns base_link via body->base_link, so the
                # wheel base must not broadcast odom->base_link as well.
                "base_publish_tf": "false",
                # slam_toolbox needs odom->base_link (now unowned) and neither RViz
                # layout uses /map; the 2D grid comes from cloud_to_occupancy_grid.
                "enable_2d_mapping": "false",
            }.items(),
        ),

        # 2. FAST-LIO chain (adapter + fastlio + frame bridges). The identity
        #    map->camera_init bridge is enabled only without RTAB-Map; with the
        #    loop backend, RTAB-Map alone owns that corrected map edge.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fast_lio_launch),
            launch_arguments={
                "rviz": "false",
                "publish_map_tf": PythonExpression(
                    ["'", enable_loop_backend, "' != 'true'"]
                ),
                "publish_radar_tf": "false",
            }.items(),
        ),

        # 3. 3D -> 2D occupancy projection for Nav2 (Task 9), fed by FAST-LIO's
        #    registered cloud (in camera_init/map world frame).
        Node(
            package="wheelchair_3d_mapping", executable="cloud_to_occupancy_grid_node",
            name="cloud_to_occupancy_grid_node", output="screen",
            parameters=[os.path.join(mapping, "config", "cloud_to_occupancy_grid.yaml"),
                        {"input_cloud_topic": "/cloud_registered",
                         "map_frame": "camera_init"}],
        ),

        # 4. Optional RTAB-Map loop/export backend (Task 12), default off.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch),
            launch_arguments={
                "bringup_sensors": "false",
                "points_topic": "/cloud_registered",
                "odom_mode": "external",
                "odom_topic": "/Odometry",
                "subscribe_scan_cloud": "true",
                "rviz": "false",
            }.items(),
            condition=IfCondition(enable_loop_backend),
        ),

        # 5. RViz: LIO mapping layout (Task 10).
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", PathJoinSubstitution(
                [bringup_share, "rviz", "manual_mapping_lio_left.rviz"])],
            condition=IfCondition(use_rviz),
        ),
    ])
