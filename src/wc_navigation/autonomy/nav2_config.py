"""Nav2, slam_toolbox and AMCL parameters for the wheelchair autonomy stack.

Parameter names follow the installed Humble 1.1.20 packages (checked against
``nav2_bringup/params/nav2_params.yaml`` and the collision monitor template).
Key values are read back from the running nodes before a run proceeds.

Velocity chain (every motion source passes the smoother, collision monitor and
the wheelchair safety gate; nothing publishes ``/cmd_vel``)::

    controller_server, behavior_server -> /cmd_vel_nav -> velocity_smoother
    -> /cmd_vel_smoothed -> collision_monitor -> /cmd_vel_monitored
    -> wc safety gate -> /wc_nav/cmd_vel_gated -> base (simulator here)
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from ..geometry import _number

CMD_NAV, CMD_SMOOTHED, CMD_MONITORED, CMD_GATED = (
    "/cmd_vel_nav", "/cmd_vel_smoothed", "/cmd_vel_monitored", "/wc_nav/cmd_vel_gated")

NAV2_NODES = ("controller_server", "planner_server", "behavior_server", "bt_navigator",
              "velocity_smoother", "collision_monitor")
STOP_RING_M = 0.02  # collision-monitor stop circle: assumed body radius plus this ring
# Humble collision monitor: a zone acts once it holds more than this many points.
MONITOR_MAX_POINTS = 3
MAPPERS = ("odom_grid", "odom_icp", "slam_toolbox")
LOCALIZATION_NODES = ("map_server", "amcl")

# Same plugin list as the installed Humble bringup; trees below use a subset.
BT_PLUGINS = [
    "nav2_compute_path_to_pose_action_bt_node", "nav2_compute_path_through_poses_action_bt_node",
    "nav2_smooth_path_action_bt_node", "nav2_follow_path_action_bt_node", "nav2_spin_action_bt_node",
    "nav2_wait_action_bt_node", "nav2_assisted_teleop_action_bt_node", "nav2_back_up_action_bt_node",
    "nav2_drive_on_heading_bt_node", "nav2_clear_costmap_service_bt_node",
    "nav2_is_stuck_condition_bt_node", "nav2_goal_reached_condition_bt_node",
    "nav2_goal_updated_condition_bt_node", "nav2_globally_updated_goal_condition_bt_node",
    "nav2_is_path_valid_condition_bt_node", "nav2_initial_pose_received_condition_bt_node",
    "nav2_reinitialize_global_localization_service_bt_node", "nav2_rate_controller_bt_node",
    "nav2_distance_controller_bt_node", "nav2_speed_controller_bt_node",
    "nav2_truncate_path_action_bt_node", "nav2_truncate_path_local_action_bt_node",
    "nav2_goal_updater_node_bt_node", "nav2_recovery_node_bt_node", "nav2_pipeline_sequence_bt_node",
    "nav2_round_robin_node_bt_node", "nav2_transform_available_condition_bt_node",
    "nav2_time_expired_condition_bt_node", "nav2_path_expiring_timer_condition",
    "nav2_distance_traveled_condition_bt_node", "nav2_single_trigger_bt_node",
    "nav2_goal_updated_controller_bt_node", "nav2_is_battery_low_condition_bt_node",
    "nav2_navigate_through_poses_action_bt_node", "nav2_navigate_to_pose_action_bt_node",
    "nav2_remove_passed_goals_action_bt_node", "nav2_planner_selector_bt_node",
    "nav2_controller_selector_bt_node", "nav2_goal_checker_selector_bt_node",
    "nav2_controller_cancel_bt_node", "nav2_path_longer_on_approach_bt_node",
    "nav2_wait_cancel_bt_node", "nav2_spin_cancel_bt_node", "nav2_back_up_cancel_bt_node",
    "nav2_assisted_teleop_cancel_bt_node", "nav2_drive_on_heading_cancel_bt_node",
    "nav2_is_battery_charging_condition_bt_node",
]


@dataclass(frozen=True)
class NavProfile:
    """Tunables shared by planner, controller, smoother, monitor and gate.

    The circle radius is the existing offline envelope assumption, not a
    measured wheelchair outline. ``margin_m`` is enforced as part of the lethal
    costmap radius; the simulator's collision check uses the bare radius.
    """
    radius_m: float = 0.55
    margin_m: float = 0.10
    max_linear: float = 0.30
    max_angular: float = 0.50
    linear_accel: float = 0.30
    linear_decel: float = 0.60
    angular_accel: float = 0.80
    sensor_range_m: float = 8.0
    sensor_min_range_m: float = 0.3
    controller_frequency_hz: float = 10.0
    # Optional convex body outline [(x, y), ...] in base_link (axle origin inside). Empty: the circle above.
    footprint: tuple = ()
    # Obstacle input (perception.nav2_obstacles): "scan"/"raw"/"" keep /scan everywhere.
    obstacle_input: str = "scan"
    obstacle_clouds: tuple = ()
    obstacle_scan: str = "raw"
    obstacle_monitor: str = ""          # collision monitor input; empty: the same as obstacle_input

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            if name not in ("footprint", "obstacle_input", "obstacle_clouds", "obstacle_scan", "obstacle_monitor"):
                object.__setattr__(self, name, _number(getattr(self, name), name, positive=True))
        if self.max_linear > 1.0 or self.max_angular > 1.5:
            raise ValueError("navigation speed limits exceed the supported simulation range")
        object.__setattr__(self, "footprint", _convex_footprint(self.footprint))
        from ..perception.nav2_obstacles import check
        obstacle_input, clouds, obstacle_scan, monitor = check(self.obstacle_input, self.obstacle_clouds,
                                                               self.obstacle_scan, self.obstacle_monitor)
        object.__setattr__(self, "obstacle_input", obstacle_input)
        object.__setattr__(self, "obstacle_clouds", clouds)
        object.__setattr__(self, "obstacle_scan", obstacle_scan)
        object.__setattr__(self, "obstacle_monitor", monitor)

    @property
    def monitor_input(self):
        """What the collision monitor reads: scan, cloud or scan_and_cloud."""
        return self.obstacle_monitor or self.obstacle_input

    @property
    def body_inscribed_m(self):
        """Distance from base_link to the nearest body edge (the circle radius without an outline)."""
        if not self.footprint:
            return self.radius_m
        points = self.footprint
        return min(_distance_to_segment((0.0, 0.0), a, b) for a, b in zip(points, points[1:] + points[:1]))

    @property
    def body_circumscribed_m(self):
        """Farthest body point from base_link: what an in-place turn sweeps."""
        if not self.footprint:
            return self.radius_m
        return max(math.hypot(x, y) for x, y in self.footprint)

    @property
    def lethal_radius_m(self):
        """Planning lethal radius: the body plus margin as Nav2 inflation sees it (inscribed radius)."""
        return self.body_inscribed_m + self.margin_m

    def as_dict(self):
        values = {name: getattr(self, name) for name in self.__dataclass_fields__}
        values["footprint"] = [list(p) for p in self.footprint]
        if self.obstacle_input == "scan" and self.obstacle_scan == "raw" and not self.obstacle_monitor:
            for name in ("obstacle_input", "obstacle_clouds", "obstacle_scan", "obstacle_monitor"):
                values.pop(name)                    # default: recorded profile unchanged
        else:
            values["obstacle_clouds"] = list(self.obstacle_clouds)
            if not self.obstacle_monitor:
                values.pop("obstacle_monitor")      # profiles recorded before the option stay identical
        return values


def _distance_to_segment(p, a, b):
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / max(dx * dx + dy * dy, 1e-12)))
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)


def _convex_footprint(points):
    """Validate an outline: at least 3 finite points, convex, base_link strictly inside, within 2 m."""
    points = tuple(tuple(_number(v, "footprint coordinate") for v in p) for p in (points or ()))
    if not points:
        return ()
    if len(points) < 3 or any(len(p) != 2 for p in points):
        raise ValueError("footprint needs at least three (x, y) points")
    if any(math.hypot(*p) > 2.0 for p in points):
        raise ValueError("footprint points must lie within 2 m of base_link")
    n = len(points)
    crosses = [(points[(k + 1) % n][0] - points[k][0]) * (points[(k + 2) % n][1] - points[(k + 1) % n][1])
               - (points[(k + 1) % n][1] - points[k][1]) * (points[(k + 2) % n][0] - points[(k + 1) % n][0])
               for k in range(n)]
    if not (all(c > 0 for c in crosses) or all(c < 0 for c in crosses)):
        raise ValueError("footprint must be a convex polygon in consistent order")
    inside = [(x2 - x1) * (0.0 - y1) - (y2 - y1) * (0.0 - x1)
              for (x1, y1), (x2, y2) in zip(points, points[1:] + points[:1])]
    if not (all(v > 0 for v in inside) or all(v < 0 for v in inside)):
        raise ValueError("base_link must lie strictly inside the footprint")
    return points


def footprint_string(points):
    """Nav2 costmap footprint parameter text, e.g. '[[0.7, 0.35], [0.7, -0.35], ...]'."""
    return "[" + ", ".join("[%s, %s]" % (repr(float(x)), repr(float(y))) for x, y in points) + "]"


def offset_polygon(points, distance):
    """Convex outline grown outward by ``distance`` along each vertex bisector (flat x, y list)."""
    out = []
    n = len(points)
    area = sum(points[k][0] * points[(k + 1) % n][1] - points[(k + 1) % n][0] * points[k][1] for k in range(n))
    sign = 1.0 if area > 0 else -1.0                      # counter-clockwise: outward normal is (dy, -dx)
    for k in range(n):
        prev, cur, nxt = points[k - 1], points[k], points[(k + 1) % n]
        normals = []
        for a, b in ((prev, cur), (cur, nxt)):
            dx, dy = b[0] - a[0], b[1] - a[1]
            length = math.hypot(dx, dy)
            normals.append((sign * dy / length, -sign * dx / length))
        nx, ny = normals[0][0] + normals[1][0], normals[0][1] + normals[1][1]
        scale = distance / max(1e-9, 1.0 + normals[0][0] * normals[1][0] + normals[0][1] * normals[1][1])
        out.extend([cur[0] + nx * scale, cur[1] + ny * scale])
    return out


def _p(values):
    return {"ros__parameters": dict(values)}


def slowdown_polygon(profile):
    """Collision-monitor slowdown box ahead of the axle, flat [x0, y0, x1, y1, ...] in base_link."""
    if profile.footprint:
        front = max(x for x, _ in profile.footprint)
        side = max(abs(y) for _, y in profile.footprint)
    else:
        front, side = profile.radius_m, profile.radius_m * 0.8
    start = min(0.40, front - 0.15)
    return [front + 0.65, side + 0.20, front + 0.65, -side - 0.20, start, -side - 0.20, start, side + 0.20]


def monitor_polygons(profile):
    """Collision-monitor zones: stop just outside the body, slowdown ahead, approach along the motion."""
    common = {"max_points": MONITOR_MAX_POINTS, "visualize": False, "enabled": True}
    slow = dict(common, type="polygon", points=slowdown_polygon(profile), action_type="slowdown",
                slowdown_ratio=0.4, polygon_pub_topic="polygon_slowdown")
    approach = dict(common, action_type="approach", time_before_collision=1.2, simulation_time_step=0.1)
    if profile.footprint:
        stop = dict(common, type="polygon", points=offset_polygon(profile.footprint, STOP_RING_M),
                    action_type="stop", polygon_pub_topic="polygon_stop")
        approach.update(type="polygon", points=[float(v) for p in profile.footprint for v in p])
        return {"FootprintStop": stop, "PolygonSlow": slow, "FootprintApproach": approach}
    stop = dict(common, type="circle", radius=profile.radius_m + STOP_RING_M, action_type="stop",
                polygon_pub_topic="polygon_stop")
    approach.update(type="circle", radius=profile.radius_m)
    return {"CircleStop": stop, "PolygonSlow": slow, "FootprintApproach": approach}


def _scan_source(profile):
    from ..perception.nav2_obstacles import scan_topic
    return {"topic": scan_topic(profile.obstacle_scan), "data_type": "LaserScan", "marking": True, "clearing": True,
            "max_obstacle_height": 2.0, "inf_is_valid": False,
            "obstacle_max_range": min(6.0, profile.sensor_range_m),
            "obstacle_min_range": 0.0,
            "raytrace_max_range": profile.sensor_range_m - 0.05, "raytrace_min_range": 0.0}


def global_plugins(mode):
    """Exploration plans on the live SLAM map only; a saved map adds live scans.

    During exploration the SLAM grid already integrates every scan. A second
    obstacle layer in the map frame would keep marks made before each
    map->odom correction at their old positions, where they can narrow a
    doorway. The local costmap still marks live scans in both modes.
    """
    if mode == "explore":
        return ["static_layer", "inflation_layer"]
    return ["static_layer", "obstacle_layer", "inflation_layer"]


def _costmap(profile, *, global_map, inflate_around_unknown, plugins=None):
    """Two footprints on purpose.

    Global (planning): lethal radius = assumed body + margin, so every planned
    path keeps the margin. Local (controller collision check): the assumed
    body radius, the same disc the collision monitor and the simulator use.
    With the margin also in the local footprint (run 20261104T143857Z), a
    1.8 m door left 0.25 m of slack per side; Regulated Pure Pursuit reported
    "collision ahead" 334 times and five goals aborted while the simulated
    centre was still 0.82-0.93 m from the nearest wall.

    With a body outline both costmaps carry the polygon instead of a radius:
    the global one padded by the margin (Nav2 inflation then treats its
    inscribed radius as lethal, so the planner passes anywhere the narrowest
    body width fits), the local one unpadded for the controller's footprint
    collision check along each arc, rotations included.
    """
    lethal = profile.lethal_radius_m
    params = {
        "use_sim_time": True, "robot_base_frame": "base_link", "resolution": 0.05,
        "robot_radius": lethal if global_map else profile.radius_m,
        "footprint_padding": 0.0, "transform_tolerance": 0.5,
        "always_send_full_costmap": True,
        "obstacle_layer": {"plugin": "nav2_costmap_2d::ObstacleLayer", "enabled": True,
                           "footprint_clearing_enabled": True, "observation_sources": "scan",
                           "scan": _scan_source(profile)},
        "inflation_layer": {"plugin": "nav2_costmap_2d::InflationLayer", "enabled": True,
                            "cost_scaling_factor": 3.0,
                            "inflation_radius": max(lethal + 0.35, profile.body_circumscribed_m + profile.margin_m),
                            "inflate_unknown": False, "inflate_around_unknown": inflate_around_unknown},
    }
    if profile.footprint:
        params.update(footprint=footprint_string(profile.footprint),
                      footprint_padding=profile.margin_m if global_map else 0.0)
    if global_map:
        params.update({"global_frame": "map", "update_frequency": 2.0, "publish_frequency": 1.0,
                       "rolling_window": False, "track_unknown_space": True,
                       "plugins": list(plugins or global_plugins("navigate")),
                       "static_layer": {"plugin": "nav2_costmap_2d::StaticLayer", "enabled": True,
                                        "map_topic": "/map", "map_subscribe_transient_local": True}})
    else:
        params.update({"global_frame": "odom", "update_frequency": 5.0, "publish_frequency": 2.0,
                       "rolling_window": True, "width": 6, "height": 6, "track_unknown_space": False,
                       "plugins": ["obstacle_layer", "inflation_layer"]})
        if profile.obstacle_input != "scan":       # obstacle clouds (perception.nav2_obstacles)
            from ..perception import nav2_obstacles
            params.update(plugins=nav2_obstacles.local_plugins(profile.obstacle_input),
                          voxel_layer=nav2_obstacles.voxel_layer(profile.obstacle_clouds))
            if profile.obstacle_input == "cloud":
                params.pop("obstacle_layer")
    return params


def behavior_tree_xml(*, through_poses=False, recovery_retries=4, wait_s=3.0, fallback_planner=False,
                      rescue_planner=False):
    """Replanning tree without Spin or BackUp recoveries.

    The wheelchair has no rear range sensor, so reversing is never used, and an
    unprompted spin would sweep the unobserved rear of a non-circular body.
    Recovery only clears costmaps and waits before replanning;
    ``recovery_retries`` and ``wait_s`` (defaults 4 and 3 s) set how often and
    how long. With ``fallback_planner`` a second planner plugin named
    ``Fallback`` is asked only when ``GridBased`` finds no path; with
    ``rescue_planner`` a last one named ``Rescue`` only when all before it
    found none (window B, planning/stack.py).
    """
    retries = int(recovery_retries)
    wait = float(wait_s)
    if not 0 <= retries <= 20 or not 0.5 <= wait <= 30.0:
        raise ValueError("recovery retries must be 0..20 and the wait 0.5..30 s")
    wait_text = ("%g" % wait)
    if through_poses:
        template = '<ComputePathThroughPoses goals="{{goals}}" path="{{path}}" planner_id="{}"/>'
        goal_updated = "<GoalUpdated/>"
        head = ('<RemovePassedGoals input_goals="{goals}" output_goals="{goals}" radius="0.7"/>\n            ')
    else:
        template = '<ComputePathToPose goal="{{goal}}" path="{{path}}" planner_id="{}"/>'
        goal_updated = "<GoalUpdated/>"
        head = ""
    planner = template.format("GridBased")
    ids = ["GridBased"] + (["Fallback"] if fallback_planner else []) + (["Rescue"] if rescue_planner else [])
    if len(ids) > 1:
        planner = ('<Fallback name="PlannerFallback">\n              '
                   + "\n              ".join(template.format(i) for i in ids) + "\n            </Fallback>")
    return f"""<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="{retries}" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePath">
            <Sequence>
            {head}{planner}
            </Sequence>
            <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{{path}}" controller_id="FollowPath"/>
          <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        {goal_updated}
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <Wait wait_duration="{wait_text}"/>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
"""


def navigation_parameters(profile, *, mode, bt_to_pose_path, bt_through_poses_path, planning=None):
    """Parameters for the Nav2 servers. ``mode`` is ``explore`` or ``navigate``.

    Exploration lets the planner cross unknown cells at near-lethal cost (the
    robot starts with its own unobserved body area unknown, and frontier goals
    sit next to unknown space); goals are still chosen in known free space.
    Navigation on a saved map blocks unknown space entirely.
    ``planning`` (``wc_navigation.planning.stack.PlanningChoice``) selects
    another planner or controller plugin; None or the default choice keeps
    NavFn and Regulated Pure Pursuit as below.
    """
    params = _navigation_parameters(profile, mode=mode, bt_to_pose_path=bt_to_pose_path,
                                    bt_through_poses_path=bt_through_poses_path)
    if planning is not None:
        from ..planning.stack import apply
        apply(params, profile, planning, exploring=mode == "explore")
    return params


def _navigation_parameters(profile, *, mode, bt_to_pose_path, bt_through_poses_path):
    if mode not in ("explore", "navigate"):
        raise ValueError("mode must be explore or navigate")
    exploring = mode == "explore"
    lethal = profile.lethal_radius_m
    # Collision Monitor "stop" zeroes rotation as well. A forward stop box
    # 5 cm past the body (run 20261105T024902Z) froze the robot in a 1.8 m door
    # for 170 s with a jamb corner 0.60 m from its centre, although turning in
    # place inside the assumed circle was collision free. The stop zone is
    # therefore a circle just outside the body (imminent contact from any
    # side); forward speed near obstacles is limited by the "approach" model,
    # which predicts contact along the commanded motion. With a body outline the
    # stop zone is that outline grown by the same ring (see monitor_polygons).
    zones = monitor_polygons(profile)
    sources = {"observation_sources": ["scan"], "scan": {"type": "scan", "topic": "/scan", "enabled": True}}
    if profile.monitor_input != "scan":            # obstacle clouds (perception.nav2_obstacles)
        from ..perception import nav2_obstacles
        sources.update(nav2_obstacles.monitor_sources(profile.obstacle_clouds),
                       observation_sources=nav2_obstacles.monitor_source_names(profile.monitor_input,
                                                                                profile.obstacle_clouds))
        if profile.monitor_input == "cloud":
            sources.pop("scan")
    return {
        "controller_server": _p({
            "use_sim_time": True, "controller_frequency": profile.controller_frequency_hz,
            "min_x_velocity_threshold": 0.001, "min_y_velocity_threshold": 0.5,
            "min_theta_velocity_threshold": 0.001, "failure_tolerance": 0.5,
            "progress_checker_plugin": "progress_checker",
            "goal_checker_plugins": ["general_goal_checker"], "controller_plugins": ["FollowPath"],
            # Rotating to a new heading counts as progress: near walls the
            # collision monitor slows turns to 40 %, and the translation-only
            # SimpleProgressChecker then aborted FollowPath mid-turn.
            "progress_checker": {"plugin": "nav2_controller::PoseProgressChecker",
                                 "required_movement_radius": 0.3, "required_movement_angle": 0.5,
                                 "movement_time_allowance": 20.0},
            "general_goal_checker": {"plugin": "nav2_controller::SimpleGoalChecker", "stateful": True,
                                     "xy_goal_tolerance": 0.25, "yaw_goal_tolerance": 0.35},
            "FollowPath": {
                "plugin": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
                # A shorter lookahead keeps the pursuit arc closer to the
                # planned path, which already holds the margin, at door turns.
                "desired_linear_vel": profile.max_linear, "lookahead_dist": 0.6,
                "min_lookahead_dist": 0.4, "max_lookahead_dist": 0.9, "lookahead_time": 1.5,
                "rotate_to_heading_angular_vel": min(0.4, profile.max_angular),
                "transform_tolerance": 0.3, "use_velocity_scaled_lookahead_dist": False,
                "min_approach_linear_velocity": 0.05, "approach_velocity_scaling_dist": 0.6,
                "use_collision_detection": True, "max_allowed_time_to_collision_up_to_carrot": 1.5,
                "use_regulated_linear_velocity_scaling": True,
                "use_cost_regulated_linear_velocity_scaling": True,
                "regulated_linear_scaling_min_radius": 0.9, "regulated_linear_scaling_min_speed": 0.10,
                "use_rotate_to_heading": True, "allow_reversing": False,
                "rotate_to_heading_min_angle": 0.5, "max_angular_accel": profile.angular_accel,
                "max_robot_pose_search_dist": 10.0, "use_interpolation": True,
                "cost_scaling_dist": 0.6, "cost_scaling_gain": 1.0, "inflation_cost_scaling_factor": 3.0},
        }),
        "local_costmap": {"local_costmap": _p(_costmap(profile, global_map=False, inflate_around_unknown=False))},
        "global_costmap": {"global_costmap": _p(_costmap(profile, global_map=True,
                                                         inflate_around_unknown=not exploring,
                                                         plugins=global_plugins(mode)))},
        # With use_astar the same run logged six NavFn "Failed to create a plan
        # from potential when a legal potential was found" errors. Dijkstra,
        # the Nav2 default, propagates the full potential field instead.
        "planner_server": _p({
            "use_sim_time": True, "expected_planner_frequency": 1.0, "planner_plugins": ["GridBased"],
            "GridBased": {"plugin": "nav2_navfn_planner/NavfnPlanner", "tolerance": 0.25,
                          "use_astar": False, "allow_unknown": exploring,
                          "use_final_approach_orientation": False}}),
        "behavior_server": _p({
            "use_sim_time": True, "costmap_topic": "local_costmap/costmap_raw",
            "footprint_topic": "local_costmap/published_footprint", "cycle_frequency": 10.0,
            "behavior_plugins": ["spin", "wait"],
            "spin": {"plugin": "nav2_behaviors/Spin"}, "wait": {"plugin": "nav2_behaviors/Wait"},
            "global_frame": "odom", "robot_base_frame": "base_link", "transform_tolerance": 0.3,
            "simulate_ahead_time": 2.0, "max_rotational_vel": min(0.4, profile.max_angular),
            "min_rotational_vel": 0.15, "rotational_acc_lim": profile.angular_accel}),
        "bt_navigator": _p({
            "use_sim_time": True, "global_frame": "map", "robot_base_frame": "base_link",
            "odom_topic": "/odom", "bt_loop_duration": 10, "default_server_timeout": 20,
            "wait_for_service_timeout": 1000, "transform_tolerance": 0.3,
            "default_nav_to_pose_bt_xml": str(bt_to_pose_path),
            "default_nav_through_poses_bt_xml": str(bt_through_poses_path),
            "plugin_lib_names": list(BT_PLUGINS)}),
        "bt_navigator_navigate_through_poses_rclcpp_node": _p({"use_sim_time": True}),
        "bt_navigator_navigate_to_pose_rclcpp_node": _p({"use_sim_time": True}),
        "velocity_smoother": _p({
            "use_sim_time": True, "smoothing_frequency": 20.0, "scale_velocities": False,
            "feedback": "OPEN_LOOP", "max_velocity": [profile.max_linear, 0.0, profile.max_angular],
            "min_velocity": [0.0, 0.0, -profile.max_angular],
            "max_accel": [profile.linear_accel, 0.0, profile.angular_accel],
            "max_decel": [-profile.linear_decel, 0.0, -profile.angular_accel],
            "odom_topic": "odom", "odom_duration": 0.1, "deadband_velocity": [0.0, 0.0, 0.0],
            "velocity_timeout": 1.0}),
        "collision_monitor": _p({
            "use_sim_time": True, "base_frame_id": "base_link", "odom_frame_id": "odom",
            "cmd_vel_in_topic": CMD_SMOOTHED, "cmd_vel_out_topic": CMD_MONITORED,
            "transform_tolerance": 0.3, "source_timeout": 1.0, "base_shift_correction": True,
            "stop_pub_timeout": 2.0,
            "polygons": list(zones), **zones, **sources}),
        "lifecycle_manager_navigation": _p({
            "use_sim_time": True, "autostart": True, "node_names": list(NAV2_NODES),
            "bond_timeout": 4.0}),
    }


def slam_parameters(profile, *, loop_closure=False):
    """Online asynchronous slam_toolbox mapping from the simulated scan.

    Loop closure is off by default: with the narrow forward sensor and
    repetitive rooms, a false closure moved the simulated map->odom by about
    2 m, which is worse for a wheelchair than slow odometry-anchored drift. The
    existing offline RTAB-Map rebuilds likewise run without extra closures.
    """
    return {"slam_toolbox": _p({
        "use_sim_time": True, "solver_plugin": "solver_plugins::CeresSolver",
        "ceres_linear_solver": "SPARSE_NORMAL_CHOLESKY", "ceres_preconditioner": "SCHUR_JACOBI",
        "ceres_trust_strategy": "LEVENBERG_MARQUARDT", "ceres_dogleg_type": "TRADITIONAL_DOGLEG",
        "ceres_loss_function": "None", "odom_frame": "odom", "map_frame": "map",
        "base_frame": "base_link", "scan_topic": "/scan", "use_map_saver": False, "mode": "mapping",
        "debug_logging": False, "throttle_scans": 1, "transform_publish_period": 0.02,
        "map_update_interval": 1.0, "resolution": 0.05, "min_laser_range": profile.sensor_min_range_m,
        # slam_toolbox 2.6 pre-filters scans on base translation only, so a
        # zero travel distance plus a time interval is what lets in-place
        # look-around rotations (needed with a ~115 degree forward sensor) be mapped.
        "max_laser_range": profile.sensor_range_m, "minimum_time_interval": 0.5,
        "transform_timeout": 0.2, "tf_buffer_duration": 30.0, "stack_size_to_use": 40000000,
        "enable_interactive_mode": False, "use_scan_matching": True, "use_scan_barycenter": True,
        "minimum_travel_distance": 0.0, "minimum_travel_heading": 0.15, "scan_buffer_size": 10,
        "scan_buffer_maximum_scan_distance": 10.0, "link_match_minimum_response_fine": 0.1,
        "link_scan_maximum_distance": 1.5, "loop_search_maximum_distance": 3.0,
        "do_loop_closing": bool(loop_closure), "loop_match_minimum_chain_size": 15,
        "loop_match_maximum_variance_coarse": 3.0, "loop_match_minimum_response_coarse": 0.5,
        "loop_match_minimum_response_fine": 0.6, "correlation_search_space_dimension": 0.3,
        "correlation_search_space_resolution": 0.01, "correlation_search_space_smear_deviation": 0.1,
        "loop_search_space_dimension": 8.0, "loop_search_space_resolution": 0.05,
        # With a ~115 degree forward sensor close to flat walls, the correlative
        # matcher's response is nearly flat in yaw. The stock weak odometry prior
        # (variance 1.0, floor 0.9) let in-place rotations drift ~55 degrees in
        # simulation, so heading deviations from odometry are penalized harder
        # and searched more finely.
        "loop_search_space_smear_deviation": 0.03, "distance_variance_penalty": 0.1,
        "angle_variance_penalty": 0.05, "fine_search_angle_offset": 0.00349,
        "coarse_search_angle_offset": 0.175, "coarse_angle_resolution": 0.0175,
        "minimum_angle_penalty": 0.5, "minimum_distance_penalty": 0.3,
        "use_response_expansion": True, "min_pass_through": 2, "occupancy_threshold": 0.1})}


def initial_pose_covariance(std_xy_m, std_yaw_rad):
    """6x6 row-major covariance for an operator's approximate initial pose."""
    std_xy = _number(std_xy_m, "initial pose xy std", positive=True)
    std_yaw = _number(std_yaw_rad, "initial pose yaw std", positive=True)
    if std_xy > 2.0 or std_yaw > math.pi:
        raise ValueError("initial pose uncertainty is outside the supported range")
    covariance = [0.0] * 36
    covariance[0] = covariance[7] = std_xy * std_xy
    covariance[35] = std_yaw * std_yaw
    return covariance


def odom_grid_mapper_config(profile, size_m=60.0, *, scan_matching=False):
    """Odometry-anchored mapper settings (see mapper_node, occupancy, scan_match)."""
    from .scan_match import IcpConfig
    config = {"grid": {"resolution": 0.05, "size_m": float(size_m), "center_xy": [0.0, 0.0],
                       "hit": 0.85, "miss": -0.4, "lower": -2.0, "upper": 3.5,
                       "occupied_at": 1.0, "free_at": -1.0, "clear_no_return": True},
              "publish_period_s": 1.0, "max_scan_age_s": 1.0,
              "sensor_range_m": [profile.sensor_min_range_m, profile.sensor_range_m],
              "scan_matching": bool(scan_matching),
              "pose_source": ("ODOMETRY_PREDICTION_BOUNDED_SCAN_TO_MAP_CORRECTION" if scan_matching
                              else "ODOMETRY_ONLY_NO_SCAN_MATCHING")}
    if scan_matching:
        config["icp"] = IcpConfig().as_dict()
    return config


def localization_parameters(profile, map_yaml):
    """map_server plus AMCL that waits for an explicit /initialpose.

    AMCL's own ``set_initial_pose`` uses a zero covariance, i.e. a filter that
    is certain of a guessed pose. The supervisor instead publishes the
    approximate pose with its stated uncertainty, as an operator would.
    """
    return {
        "map_server": _p({"use_sim_time": True, "yaml_filename": str(map_yaml),
                          "topic_name": "map", "frame_id": "map"}),
        "amcl": _p({
            "use_sim_time": True, "alpha1": 0.2, "alpha2": 0.2, "alpha3": 0.2, "alpha4": 0.2,
            "alpha5": 0.2, "base_frame_id": "base_link", "beam_skip_distance": 0.5,
            "beam_skip_error_threshold": 0.9, "beam_skip_threshold": 0.3, "do_beamskip": False,
            "global_frame_id": "map", "lambda_short": 0.1, "laser_likelihood_max_dist": 2.0,
            "laser_max_range": profile.sensor_range_m, "laser_min_range": profile.sensor_min_range_m,
            "laser_model_type": "likelihood_field", "max_beams": 60, "max_particles": 2000,
            "min_particles": 500, "odom_frame_id": "odom", "pf_err": 0.05, "pf_z": 0.99,
            "recovery_alpha_fast": 0.0, "recovery_alpha_slow": 0.0, "resample_interval": 1,
            "robot_model_type": "nav2_amcl::DifferentialMotionModel", "save_pose_rate": 0.5,
            "sigma_hit": 0.2, "tf_broadcast": True, "transform_tolerance": 0.5,
            "update_min_a": 0.1, "update_min_d": 0.1, "z_hit": 0.5, "z_max": 0.05,
            "z_rand": 0.5, "z_short": 0.05, "scan_topic": "scan",
            "set_initial_pose": False, "always_reset_initial_pose": False}),
        "lifecycle_manager_localization": _p({
            "use_sim_time": True, "autostart": True, "node_names": list(LOCALIZATION_NODES),
            "bond_timeout": 4.0}),
    }


def readback_expectations(profile, mode, planning=None):
    """Parameters whose running values must equal the request before motion."""
    expected = _readback_expectations(profile, mode)
    if planning is not None:
        from ..planning.stack import adjust_readback
        adjust_readback(expected, planning, profile, exploring=mode == "explore")
    # A layer left out of a costmap (e.g. planning on the saved map only) declares no parameters;
    # its "plugins" entry above already checks that it is absent.
    for values in expected.values():
        plugins = values.get("plugins")
        if isinstance(plugins, list):
            for key in [k for k in values if k.split(".", 1)[0].endswith("_layer")
                        and "." in k and k.split(".", 1)[0] not in plugins]:
                del values[key]
    return expected


def _readback_expectations(profile, mode):
    exploring = mode == "explore"
    zones = monitor_polygons(profile)
    if profile.footprint:
        outline = footprint_string(profile.footprint)
        global_body = {"footprint": outline, "footprint_padding": profile.margin_m}
        local_body = {"footprint": outline, "footprint_padding": 0.0}
        stop = {"FootprintStop.points": zones["FootprintStop"]["points"]}
    else:
        global_body = {"robot_radius": profile.lethal_radius_m}
        local_body = {"robot_radius": profile.radius_m}
        stop = {"CircleStop.radius": profile.radius_m + STOP_RING_M}
    if profile.obstacle_input != "scan":           # the obstacle clouds must really be wired in
        from ..perception import nav2_obstacles
        local_body = dict(local_body, plugins=nav2_obstacles.local_plugins(profile.obstacle_input))
    if profile.monitor_input != "scan":
        from ..perception import nav2_obstacles
        stop = dict(stop, observation_sources=nav2_obstacles.monitor_source_names(profile.monitor_input,
                                                                                   profile.obstacle_clouds))
    if profile.obstacle_scan != "raw":             # and so must the filtered scan
        from ..perception import nav2_obstacles
        topic = {"obstacle_layer.scan.topic": nav2_obstacles.scan_topic(profile.obstacle_scan)}
        if mode != "explore":
            global_body = dict(global_body, **topic)
        if profile.obstacle_input != "cloud":
            local_body = dict(local_body, **topic)
    return {
        "controller_server": {"FollowPath.desired_linear_vel": profile.max_linear,
                              "FollowPath.allow_reversing": False,
                              "FollowPath.use_collision_detection": True,
                              "progress_checker.plugin": "nav2_controller::PoseProgressChecker"},
        "planner_server": {"GridBased.allow_unknown": exploring, "GridBased.use_astar": False},
        "global_costmap/global_costmap": {**global_body, "plugins": global_plugins(mode),
                                          "inflation_layer.inflate_around_unknown": not exploring},
        "local_costmap/local_costmap": local_body,
        "velocity_smoother": {"max_velocity": [profile.max_linear, 0.0, profile.max_angular],
                              "min_velocity": [0.0, 0.0, -profile.max_angular]},
        "behavior_server": {"behavior_plugins": ["spin", "wait"]},
        "collision_monitor": {"cmd_vel_in_topic": CMD_SMOOTHED, "cmd_vel_out_topic": CMD_MONITORED,
                              "polygons": list(zones), **stop},
    }
