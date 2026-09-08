"""FAST-LIO2 single-XT-M60 LiDAR-inertial mapping (spec: fastlio-narrow-fov-mapping).

The LEFT radar remains available for diagnostic mapping. The RIGHT route is
fail-closed because its calibration contract is BLOCKED_CONFLICT; historical
right-side parameters remain below as evidence, not as a deployable baseline.
Starts:

  1. lio_cloud_adapter_node : /xtm60/<radar>/points -> /lio/cloud_in (drop
     NaN/(0,0,0), forward whole-frame XYZI through FAST-LIO's generic
     PointCloud2 path; XT-M60 flash frames have no synthetic time or ring fields).
  2. fast_lio (fastlio_mapping) : consumes /lio/cloud_in + /imu/data, runs the
     iEKF, publishes /Odometry, /cloud_registered, /path and TF camera_init->body.
     (LASER_POINT_COV=100 patch in vendored source makes the IMU lead the weak
     narrow-FOV LiDAR update — see docs/fastlio_mapping.md / Task 7.)
  3. Frame bridges (static TFs) so FAST-LIO's hardcoded frames join the robot tree:
       map -> camera_init  (identity; camera_init = LIO world origin)
       body -> base_link   ([0,0,-0.45] = inverse of base->imu; 'body' = IMU frame)
  4. base_link -> xtm60_<radar>_link static TF from the measured mount calibration.

NOTE: this fork hardcodes 'camera_init'/'body' in C++ (not params), so we bridge
with static TFs. EKF/base TF disabling is done by the top-level launch (Task 8).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Measured mount calibration -> base_link->xtm60_<radar>_link static TF (quat) and
# the FAST-LIO config file per radar.
# LEFT  (20260622_ground_calib_bag): 0.545 m, pitch +1.1, roll +0.78 (floor-visible).
# RIGHT candidate (20260623, recomputed): retained for evidence only. The strict
#        calibration contract has no runtime-eligible transform.
RADAR_TF = {
    "left": dict(
        xyz=("0.45", "0.24", "0.545"),
        quat=("0.498548", "0.491742", "0.508190", "0.501380"),
        frame="xtm60_left_link", topic="/xtm60/left/points",
        cfg="xtm60_left_lio.yaml",
    ),
    "right": dict(
        xyz=("0.606", "-0.24", "0.499"),
        quat=("0.461852", "0.539985", "0.456948", "0.535077"),
        frame="xtm60_right_link", topic="/xtm60/right/points",
        cfg="xtm60_right_lio.yaml",
    ),
}


def _setup(context, *args, **kwargs):
    mapping_share = get_package_share_directory("wheelchair_3d_mapping")
    radar = LaunchConfiguration("radar").perform(context).strip().lower()
    if radar not in RADAR_TF:
        raise RuntimeError(
            f"INVALID_RADAR_SELECTION: {radar!r}; expected left or right"
        )
    if radar == "right":
        raise RuntimeError(
            "BLOCKED_CONFLICT: right_lidar_stage1_calibration has no "
            "runtime-eligible transform"
        )
    spec = RADAR_TF[radar]

    # explicit overrides (fall back to per-radar defaults)
    cfg_override = LaunchConfiguration("config_file").perform(context).strip()
    topic_override = LaunchConfiguration("input_topic").perform(context).strip()
    config_file = cfg_override or os.path.join(mapping_share, "config", spec["cfg"])
    input_topic = topic_override or spec["topic"]
    add_zero_time_field = LaunchConfiguration("add_zero_time_field")
    use_rviz = LaunchConfiguration("rviz")

    return [
        Node(
            package="wheelchair_3d_mapping", executable="lio_cloud_adapter",
            name="lio_cloud_adapter_node", output="screen",
            parameters=[{
                "input_topic": input_topic,
                "output_topic": "/lio/cloud_in",
                "restamp_to_now": False,
                "add_zero_time_field": add_zero_time_field,
            }],
        ),
        Node(
            package="fast_lio", executable="fastlio_mapping",
            name="fastlio_mapping", output="screen",
            parameters=[config_file],
        ),
        # map -> camera_init (identity). Disable when a graph backend owns the
        # map correction; it will publish map -> camera_init from /Odometry.
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="lio_map_to_camera_init",
            arguments=["0", "0", "0", "0", "0", "0", "map", "camera_init"],
            output="screen",
            condition=IfCondition(LaunchConfiguration("publish_map_tf")),
        ),
        # body -> base_link (inverse of base->imu [0,0,0.45])
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="lio_body_to_base_link",
            arguments=["0", "0", "-0.45", "0", "0", "0", "body", "base_link"],
            output="screen",
        ),
        # base_link -> xtm60_<radar>_link (measured mount calibration).
        # ONLY for standalone/bag-replay use, where robot_state_publisher is not
        # running. In a live bringup the URDF already publishes this edge, and a
        # second /tf_static publisher with a different value makes the winner a
        # startup race (the two disagree by ~0.24 m and ~10 deg on the right unit).
        # Top-level launches therefore pass publish_radar_tf:=false.
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name=f"base_to_{spec['frame']}",
            arguments=["--x", spec["xyz"][0], "--y", spec["xyz"][1], "--z", spec["xyz"][2],
                       "--qx", spec["quat"][0], "--qy", spec["quat"][1],
                       "--qz", spec["quat"][2], "--qw", spec["quat"][3],
                       "--frame-id", "base_link", "--child-frame-id", spec["frame"]],
            output="screen",
            condition=IfCondition(LaunchConfiguration("publish_radar_tf")),
        ),
        Node(
            package="rviz2", executable="rviz2", name="fastlio_rviz", output="screen",
            condition=IfCondition(use_rviz),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("radar", default_value="right",
                              description="left | right. Right is currently blocked by its "
                                          "calibration contract; left is diagnostic only."),
        DeclareLaunchArgument("config_file", default_value="",
                              description="Override FAST-LIO yaml; empty = per-radar default."),
        DeclareLaunchArgument("input_topic", default_value="",
                              description="Override raw cloud topic; empty = per-radar default."),
        DeclareLaunchArgument("add_zero_time_field", default_value="false",
                              description="Do not synthesize per-point time/ring for XT-M60 flash frames."),
        DeclareLaunchArgument(
            "publish_map_tf",
            default_value="true",
            description="Publish the identity map->camera_init bridge. Set false when "
                        "RTAB-Map owns the dynamic map correction.",
        ),
        DeclareLaunchArgument("publish_radar_tf", default_value="true",
                              description="Publish base_link->xtm60_left_link for standalone left-side "
                                          "diagnostics (no robot_state_publisher). Set false "
                                          "in a live bringup, where the URDF owns that edge."),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="FAST-LIO's own RViz; top-level launch owns the mapping view."),
        OpaqueFunction(function=_setup),
    ])
