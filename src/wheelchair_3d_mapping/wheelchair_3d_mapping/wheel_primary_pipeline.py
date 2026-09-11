"""Launch helpers for the diagnostic wheel/IMU-primary mapping mode.

This module is deliberately diagnostic-only.  It does not inspect, modify, or
weaken the formal dual-LiDAR/IMU calibration contract.  The parent launch owns
the real sensors, robot_state_publisher, safety chain, base driver, RViz, and
the mutually-exclusive pose-owner selection.

TF/topic contract in wheel-primary mode:

* robot_localization publishes an internal pose with TF disabled;
* ``wheel_pose_health_gate_node`` is the only ``odom -> base_link`` TF owner and
  broadcasts only accepted, fresh wheel/IMU poses;
* RTAB-Map consumes ``/wheel_mapping/odometry_gated`` plus the native
  right XYZI cloud, and owns the global ``map -> odom`` correction;
* ``wheel_registered_cloud_node`` publishes a high-rate, non-accumulated XYZI
  view in ``odom`` for responsive RViz display;
* RTAB-Map's optimized-cloud helper publishes the accumulated, loop-corrected
  PointCloud+Amp product in ``map``;
* FAST-LIO, when the parent starts it, is a TF-suppressed shadow estimator and
  must use private ``/lio/shadow/*`` outputs.  Nothing in this helper promotes
  that shadow pose into the active TF tree.
"""

from __future__ import annotations

import os
from collections.abc import Mapping


FILTERED_ODOM_TOPIC = "/odometry/filtered"
GATED_ODOM_TOPIC = "/wheel_mapping/odometry_gated"
WHEEL_PATH_TOPIC = "/wheel_mapping/path"
RAW_CLOUD_TOPIC = "/xtm60/right/points"
REGISTERED_CLOUD_TOPIC = "/wheel_mapping/cloud_registered"


def _validate_inputs(bringup, mapping, layout, database_path) -> None:
    if not isinstance(bringup, str) or not bringup:
        raise ValueError("bringup package-share path is required")
    if not isinstance(mapping, str) or not mapping:
        raise ValueError("mapping package-share path is required")
    if not isinstance(layout, Mapping):
        raise ValueError("diagnostic layout mapping is required")
    # ``load_diagnostic_layout`` returns derived matrices rather than the source
    # file's status marker.  Requiring those keys keeps this helper coupled to
    # that validated diagnostic loader without accepting an arbitrary dict.
    required_layout = {"extrinsic_R", "extrinsic_T", "base_to_imu_R", "base_to_imu_T"}
    if not required_layout.issubset(layout):
        raise ValueError("layout must be produced by load_diagnostic_layout")
    if database_path is None or (isinstance(database_path, str) and not database_path.strip()):
        raise ValueError("an explicit per-session RTAB-Map database path is required")


def build_wheel_primary_nodes(
    bringup, mapping, layout, database_path, *, use_sim_time=False, enable_lio_monitor=True
):
    """Return the wheel-primary mapper actions for the enclosing launch.

    The enclosing launch must start robot_localization with
    ``publish_tf:=false`` on :data:`FILTERED_ODOM_TOPIC`.  The health gate below
    republishes only fresh, controller-confirmed estimates on
    :data:`GATED_ODOM_TOPIC`; RTAB-Map and the live registered-cloud view consume
    only that gated topic.

    FAST-LIO is intentionally absent here.  The parent owns its mode-dependent
    construction because it must suppress every FAST-LIO/static bridge TF in
    wheel-primary mode and remap all outputs to ``/lio/shadow/*``.
    """

    _validate_inputs(bringup, mapping, layout, database_path)

    # Lazy ROS imports keep this file importable by offline contract tests.
    from launch.actions import GroupAction, IncludeLaunchDescription
    from launch.conditions import IfCondition
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import Node

    rtabmap_launch = os.path.join(mapping, "launch", "rtabmap_3d_mapping.launch.py")
    ekf_gate_config = os.path.join(bringup, "config", "wheel_primary_health_gate.yaml")
    use_sim = "true" if bool(use_sim_time) else "false"

    return [
        # robot_localization continues predicting when an input times out.  This
        # explicit gate prevents those prediction-only poses from entering the
        # map after wheel feedback or IMU reception becomes stale/unhealthy.
        Node(
            package="wheelchair_3d_mapping",
            executable="wheel_pose_health_gate_node",
            name="wheel_pose_health_gate",
            output="screen",
            parameters=[ekf_gate_config, {"use_sim_time": bool(use_sim_time)}],
        ),
        # Responsive, non-accumulated 3-D view.  The node transforms only XYZ;
        # all other PointCloud2 bytes (including real XT-M60 intensity) remain
        # bit-for-bit unchanged.
        Node(
            package="wheelchair_3d_mapping",
            executable="wheel_registered_cloud_node",
            name="wheel_registered_cloud",
            output="screen",
            parameters=[
                {
                    "input_cloud_topic": RAW_CLOUD_TOPIC,
                    "output_cloud_topic": REGISTERED_CLOUD_TOPIC,
                    "gated_odom_topic": GATED_ODOM_TOPIC,
                    "feedback_health_topic": "/base/wheel_feedback_healthy",
                    "expected_source_frame": "xtm60_right_link",
                    "fixed_frame": "odom",
                    "max_odom_receive_age_sec": 0.20,
                    "max_feedback_receive_age_sec": 0.25,
                    "transform_timeout_sec": 0.0,
                    "use_sim_time": bool(use_sim_time),
                }
            ],
        ),
        # RTAB-Map receives the native sensor-frame cloud, so its database keeps
        # the real scan origin/local transform.  It places that scan using the
        # gated planar wheel/gyro odometry and applies only global ICP proximity
        # constraints.  Feeding FAST-LIO's already-world-registered cloud here
        # would apply a second pose transform and is intentionally forbidden.
        GroupAction(
            scoped=True,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rtabmap_launch),
                launch_arguments={
                    "bringup_sensors": "false",
                    "points_topic": RAW_CLOUD_TOPIC,
                    "imu_topic": "/imu/data",
                    "odom_mode": "external",
                    "odom_topic": GATED_ODOM_TOPIC,
                    "frame_id": "base_link",
                    "subscribe_scan_cloud": "true",
                    "subscribe_rgb": "false",
                    "use_colorizer": "false",
                    "enable_loop_closure": "true",
                    "localization": "false",
                    "delete_db_on_start": "false",
                    "database_path": database_path,
                    "rviz": "false",
                    "use_sim_time": use_sim,
                }.items(),
            )],
        ),
        # Build the accumulated cloud from optimized RTAB-Map graph poses.  This
        # existing helper fails closed if stored scans mix XYZI and XYZ-only data
        # instead of inventing an amplitude channel.
        Node(
            package="smartwheel_global_mapping",
            executable="rtabmap_optimized_cloud_node",
            name="wheel_primary_rtabmap_optimized_cloud",
            output="screen",
            parameters=[
                {
                    "service_name": "/rtabmap/get_map_data",
                    "output_topic": "/rtabmap/optimized_cloud",
                    "path_output_topic": "/rtabmap/optimized_path",
                    "request_interval_sec": 2.0,
                    "voxel_size_m": 0.05,
                    "minimum_nodes": 2,
                    "use_sim_time": bool(use_sim_time),
                }
            ],
        ),
        # Report-only comparison.  The FAST shadow pose is never selected as TF
        # or RTAB odometry merely because this score is good.
        Node(
            package="wheelchair_3d_mapping",
            executable="lio_consistency_monitor",
            name="lio_consistency_monitor",
            output="screen",
            condition=IfCondition(str(bool(enable_lio_monitor)).lower()),
            parameters=[{
                "reference_topic": GATED_ODOM_TOPIC,
                "lio_topic": "/lio/shadow/odometry",
                "base_to_imu_R": layout["base_to_imu_R"],
                "base_to_imu_T": layout["base_to_imu_T"],
                "use_sim_time": bool(use_sim_time),
            }],
        ),
    ]
