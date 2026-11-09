"""Selectable global planner and local controller for the Nav2 stack (explicit options; defaults unchanged).

``PlanningChoice()`` (NavFn + Regulated Pure Pursuit, no overrides) leaves the
parameters of ``nav2_config.navigation_parameters`` exactly as they are. Other
choices replace the ``GridBased`` planner plugin and/or the ``FollowPath``
controller plugin under the same names, so the behaviour trees stay as they are.

Planners (Humble 1.1.20 packages installed on the Orin):

- ``navfn``: Dijkstra on the inflated costmap (default). Passes wherever the
  inscribed radius of the padded body fits; the outline is not checked.
- ``smac_2d``: cost-aware 2D A* with path smoothing; same inscribed-radius pass.
- ``theta_star``: any-angle Theta*; same inscribed-radius pass.
- ``smac_hybrid``: Hybrid-A* with forward-only Dubins motion and the full body
  outline checked at every heading (SE2); minimum turning radius 0.40 m (the
  drivers' median turning radius in the RoomA1/RoomAB/ZL1107 recordings is
  0.37-0.42 m and they almost never turn in place).
- ``smac_lattice``: state lattice on the installed 5 cm / 0.5 m differential
  primitives (which include in-place rotations, penalized), reverse expansion
  off, full outline checked.

Controllers (see ``controller_plugin``): ``rpp`` (default Regulated Pure
Pursuit), ``shim_rpp`` (rotation shim + RPP that turns while driving) and
``mppi`` (forward-only MPPI with the outline in its cost critic).
"""
from __future__ import annotations

from dataclasses import dataclass, field
import json
from pathlib import Path

DEFAULT_PLANNER = "navfn"
DEFAULT_CONTROLLER = "rpp"
SMAC_PRIMITIVES = Path("/opt/ros/humble/share/nav2_smac_planner/sample_primitives/5cm_resolution")
LATTICE_DIFF_0_5 = SMAC_PRIMITIVES / "0.5m_turning_radius/diff/output.json"
PLANNER_PLUGINS = {
    "navfn": "nav2_navfn_planner/NavfnPlanner",
    "smac_2d": "nav2_smac_planner/SmacPlanner2D",
    "theta_star": "nav2_theta_star_planner/ThetaStarPlanner",
    "smac_hybrid": "nav2_smac_planner/SmacPlannerHybrid",
    "smac_lattice": "nav2_smac_planner/SmacPlannerLattice",
}
PLANNERS = tuple(PLANNER_PLUGINS)
FOOTPRINT_PLANNERS = ("smac_hybrid", "smac_lattice")     # check the body outline at each heading (SE2)
RPP_PLUGIN = "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
CONTROLLER_PLUGINS = {
    "rpp": RPP_PLUGIN,
    "shim_rpp": "nav2_rotation_shim_controller::RotationShimController",
    "mppi": "nav2_mppi_controller::MPPIController",
}
CONTROLLERS = tuple(CONTROLLER_PLUGINS)
MPPI_CRITICS = ["ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic", "PathAlignCritic",
                "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic"]
GOAL_TOLERANCE_M = 0.25                                  # same as NavFn's GridBased.tolerance
DEFAULT_RECOVERY_RETRIES = 4                             # nav2_config.behavior_tree_xml defaults
DEFAULT_RECOVERY_WAIT_S = 3.0
SERVER_SECTIONS = ("general_goal_checker.", "progress_checker.")
MAX_LOCAL_PADDING_M = 0.3
_SMOOTHER = {"max_iterations": 1000, "w_smooth": 0.3, "w_data": 0.2, "tolerance": 1.0e-10, "do_refinement": True}


def _value(text):
    """Override value from the command line: JSON (numbers, true/false, lists) or a bare string."""
    try:
        return json.loads(text)
    except ValueError:
        return text


def parse_overrides(items):
    """['key=value', ...] -> ((key, value), ...); keys are plugin parameter names (dots allowed)."""
    out = []
    for item in items or ():
        key, sep, raw = str(item).partition("=")
        key = key.strip()
        if not sep or not key or not all(part.isidentifier() for part in key.split(".")):
            raise ValueError("override must look like name=value: " + str(item))
        out.append((key, _value(raw.strip())))
    return tuple(out)


@dataclass(frozen=True)
class PlanningChoice:
    planner: str = DEFAULT_PLANNER
    controller: str = DEFAULT_CONTROLLER
    planner_params: tuple = field(default_factory=tuple)      # ((name, value), ...) applied last
    controller_params: tuple = field(default_factory=tuple)
    # controller_server goal and progress checkers, e.g. ("general_goal_checker.xy_goal_tolerance", 0.4)
    server_params: tuple = field(default_factory=tuple)
    recovery_retries: int = DEFAULT_RECOVERY_RETRIES          # behaviour tree: clear costmaps / wait rounds
    recovery_wait_s: float = DEFAULT_RECOVERY_WAIT_S
    # Navigate mode: False plans on the saved map only (live scans stay in the local costmap and the monitor).
    global_live_scans: bool = True
    # Second planner plugin ("Fallback"), asked by the behaviour tree only when GridBased finds no path.
    fallback_planner: str = ""
    # Footprint padding of the local costmap (m), i.e. of the controller's own collision checks (RPP, rotation
    # shim). 0 keeps the stack as it is; the planning margin of the global costmap is the profile's margin_m.
    local_padding_m: float = 0.0

    def __post_init__(self):
        if self.planner not in PLANNER_PLUGINS:
            raise ValueError("planner must be one of " + ", ".join(PLANNERS))
        if self.controller not in CONTROLLERS:
            raise ValueError("controller must be one of " + ", ".join(CONTROLLERS))
        for name in ("planner_params", "controller_params", "server_params"):
            value = tuple((str(k), v) for k, v in (getattr(self, name) or ()))
            object.__setattr__(self, name, value)
        for key, _ in self.server_params:
            if not key.startswith(SERVER_SECTIONS):
                raise ValueError("controller server overrides are limited to " + ", ".join(SERVER_SECTIONS) + ": " + key)
        retries, wait = self.recovery_retries, float(self.recovery_wait_s)
        if type(retries) is not int or not 0 <= retries <= 20 or not 0.5 <= wait <= 30.0:
            raise ValueError("recovery retries must be 0..20 and the wait 0.5..30 s")
        object.__setattr__(self, "recovery_wait_s", wait)
        if not isinstance(self.global_live_scans, bool):
            raise ValueError("global_live_scans must be true or false")
        if self.fallback_planner not in ("", *PLANNERS) or self.fallback_planner == self.planner:
            raise ValueError("fallback planner must be another one of " + ", ".join(PLANNERS))
        padding = float(self.local_padding_m)
        if not 0.0 <= padding <= MAX_LOCAL_PADDING_M:
            raise ValueError("local padding must be 0..%g m" % MAX_LOCAL_PADDING_M)
        object.__setattr__(self, "local_padding_m", padding)

    @property
    def is_default(self):
        return (self.planner == DEFAULT_PLANNER and self.controller == DEFAULT_CONTROLLER
                and not self.planner_params and not self.controller_params and not self.server_params
                and self.recovery_retries == DEFAULT_RECOVERY_RETRIES and self.recovery_wait_s == DEFAULT_RECOVERY_WAIT_S
                and self.global_live_scans and not self.fallback_planner and not self.local_padding_m)

    def as_dict(self):
        value = {"planner": self.planner, "controller": self.controller,
                 "planner_params": [list(p) for p in self.planner_params],
                 "controller_params": [list(p) for p in self.controller_params],
                 "server_params": [list(p) for p in self.server_params],
                 "recovery_retries": self.recovery_retries, "recovery_wait_s": self.recovery_wait_s,
                 "global_live_scans": self.global_live_scans, "fallback_planner": self.fallback_planner}
        if self.local_padding_m:                         # runs without it keep their recorded config unchanged
            value["local_padding_m"] = self.local_padding_m
        return value

    def behavior_tree_options(self):
        options = {"recovery_retries": self.recovery_retries, "wait_s": self.recovery_wait_s}
        if self.fallback_planner:
            options["fallback_planner"] = True
        return options

    @classmethod
    def from_dict(cls, value):
        value = value or {}
        return cls(planner=value.get("planner", DEFAULT_PLANNER), controller=value.get("controller", DEFAULT_CONTROLLER),
                   planner_params=tuple(tuple(p) for p in value.get("planner_params", ())),
                   controller_params=tuple(tuple(p) for p in value.get("controller_params", ())),
                   server_params=tuple(tuple(p) for p in value.get("server_params", ())),
                   recovery_retries=int(value.get("recovery_retries", DEFAULT_RECOVERY_RETRIES)),
                   recovery_wait_s=float(value.get("recovery_wait_s", DEFAULT_RECOVERY_WAIT_S)),
                   global_live_scans=bool(value.get("global_live_scans", True)),
                   fallback_planner=str(value.get("fallback_planner", "")),
                   local_padding_m=float(value.get("local_padding_m", 0.0)))


def _set(section, dotted, value):
    parts = dotted.split(".")
    for part in parts[:-1]:
        section = section.setdefault(part, {})
    section[parts[-1]] = value


def planner_plugin(name, *, exploring, tolerance=GOAL_TOLERANCE_M):
    """``GridBased`` plugin parameters of one planner (navfn equals the stack default)."""
    unknown = bool(exploring)
    if name == "navfn":
        return {"plugin": PLANNER_PLUGINS[name], "tolerance": tolerance, "use_astar": False,
                "allow_unknown": unknown, "use_final_approach_orientation": False}
    if name == "theta_star":
        return {"plugin": PLANNER_PLUGINS[name], "how_many_corners": 8, "w_euc_cost": 1.0, "w_traversal_cost": 2.0,
                "allow_unknown": unknown, "use_final_approach_orientation": False}
    smac = {"plugin": PLANNER_PLUGINS[name], "tolerance": tolerance, "allow_unknown": unknown,
            "max_iterations": 1000000, "max_on_approach_iterations": 1000, "max_planning_time": 2.0,
            "smoother": dict(_SMOOTHER)}
    if name == "smac_2d":
        smac.update(downsample_costmap=False, downsampling_factor=1, cost_travel_multiplier=2.0,
                    use_final_approach_orientation=False)
        return smac
    smac.update(analytic_expansion_ratio=3.5, analytic_expansion_max_length=3.0, cost_penalty=2.0,
                non_straight_penalty=1.2, change_penalty=0.0, reverse_penalty=2.0, retrospective_penalty=0.015,
                lookup_table_size=20.0, cache_obstacle_heuristic=False, smooth_path=True)
    if name == "smac_hybrid":
        smac.update(downsample_costmap=False, downsampling_factor=1, motion_model_for_search="DUBIN",
                    angle_quantization_bins=72, minimum_turning_radius=0.40)
        return smac
    if name == "smac_lattice":
        smac.update(lattice_filepath=str(LATTICE_DIFF_0_5), allow_reverse_expansion=False, rotation_penalty=5.0,
                    non_straight_penalty=1.05, change_penalty=0.05)
        return smac
    raise ValueError("unknown planner: " + str(name))


def controller_plugin(name, rpp, profile):
    """``FollowPath`` parameters of one controller; ``rpp`` is the stack's Regulated Pure Pursuit section.

    ``shim_rpp``: the rotation shim turns in place only when a new path starts
    more than 45 degrees off the heading (checked for collision over 1 s), then
    hands over to RPP with its own rotate-to-heading off, so RPP turns while
    driving along the path curvature (regulated down to 0.10 m/s on tight arcs).
    ``mppi``: sampling MPC on the differential-drive model with forward speeds
    only (vx_min 0), the body outline in its costmap critic, and no hard stop of
    its own; the collision monitor and the gate still stop the chair.
    """
    if name == "rpp":
        return dict(rpp)
    if name == "shim_rpp":
        out = dict(rpp)
        out.update(plugin=CONTROLLER_PLUGINS[name], primary_controller=RPP_PLUGIN, use_rotate_to_heading=False,
                   angular_dist_threshold=0.785, forward_sampling_distance=0.5, simulate_ahead_time=1.0,
                   rotate_to_goal_heading=False)
        return out
    if name == "mppi":
        critics = {
            "ConstraintCritic": {"enabled": True, "cost_power": 1, "cost_weight": 4.0},
            "CostCritic": {"enabled": True, "cost_power": 1, "cost_weight": 3.81, "critical_cost": 300.0,
                           "consider_footprint": True, "collision_cost": 1000000.0, "near_goal_distance": 0.5,
                           "trajectory_point_step": 2},
            "GoalCritic": {"enabled": True, "cost_power": 1, "cost_weight": 5.0, "threshold_to_consider": 1.4},
            "GoalAngleCritic": {"enabled": True, "cost_power": 1, "cost_weight": 3.0, "threshold_to_consider": 0.5},
            "PathAlignCritic": {"enabled": True, "cost_power": 1, "cost_weight": 14.0, "max_path_occupancy_ratio": 0.05,
                                "trajectory_point_step": 4, "threshold_to_consider": 0.5, "offset_from_furthest": 20,
                                "use_path_orientations": False},
            "PathFollowCritic": {"enabled": True, "cost_power": 1, "cost_weight": 5.0, "offset_from_furthest": 5,
                                 "threshold_to_consider": 1.4},
            "PathAngleCritic": {"enabled": True, "cost_power": 1, "cost_weight": 2.0, "offset_from_furthest": 4,
                                "threshold_to_consider": 0.5, "max_angle_to_furthest": 1.0, "forward_preference": True},
            "PreferForwardCritic": {"enabled": True, "cost_power": 1, "cost_weight": 5.0, "threshold_to_consider": 0.5},
        }
        return {"plugin": CONTROLLER_PLUGINS[name], "motion_model": "DiffDrive", "time_steps": 30,
                "model_dt": round(1.0 / profile.controller_frequency_hz, 6), "batch_size": 1000,
                "iteration_count": 1, "temperature": 0.3, "gamma": 0.015,
                "vx_std": 0.2, "vy_std": 0.2, "wz_std": 0.4, "vx_max": profile.max_linear, "vx_min": 0.0,
                "vy_max": 0.0, "wz_max": profile.max_angular, "prune_distance": 1.7, "transform_tolerance": 0.3,
                "reset_period": 1.0, "regenerate_noises": False, "retry_attempt_limit": 1, "visualize": False,
                "critics": list(MPPI_CRITICS), **critics}
    raise ValueError("unknown controller: " + str(name))


def apply(params, profile, choice, *, exploring):
    """Replace planner and controller plugins in a ``navigation_parameters`` result, in place."""
    if choice.is_default:
        return params
    planner = params["planner_server"]["ros__parameters"]
    planner["GridBased"] = planner_plugin(choice.planner, exploring=exploring)
    for key, value in choice.planner_params:
        _set(planner["GridBased"], key, value)
    if choice.fallback_planner:
        planner["planner_plugins"] = ["GridBased", "Fallback"]
        planner["Fallback"] = planner_plugin(choice.fallback_planner, exploring=exploring)
    controller = params["controller_server"]["ros__parameters"]
    controller["FollowPath"] = controller_plugin(choice.controller, controller["FollowPath"], profile)
    for key, value in choice.controller_params:
        _set(controller["FollowPath"], key, value)
    for key, value in choice.server_params:
        _set(controller, key, value)
    if not choice.global_live_scans:
        costmap = params["global_costmap"]["global_costmap"]["ros__parameters"]
        costmap["plugins"] = [p for p in costmap["plugins"] if p != "obstacle_layer"]
    if choice.local_padding_m:
        params["local_costmap"]["local_costmap"]["ros__parameters"]["footprint_padding"] = choice.local_padding_m
    return params


def adjust_readback(expected, choice, profile, *, exploring):
    """Adapt ``nav2_config.readback_expectations`` in place to a non-default choice.

    The planner and controller plugin types, the no-reverse settings and every
    override are read back from the running servers before a run proceeds.
    """
    if choice.is_default:
        return expected
    plugin = planner_plugin(choice.planner, exploring=exploring)
    planner = {"GridBased.plugin": plugin["plugin"], "GridBased.allow_unknown": bool(exploring)}
    if choice.planner == "navfn":
        planner["GridBased.use_astar"] = False
    if choice.planner == "smac_hybrid":
        planner["GridBased.motion_model_for_search"] = "DUBIN"
    if choice.planner == "smac_lattice":
        planner.update({"GridBased.allow_reverse_expansion": False,
                        "GridBased.lattice_filepath": plugin["lattice_filepath"]})
    for key, value in choice.planner_params:
        if not isinstance(value, dict):
            planner["GridBased." + key] = value
    if choice.fallback_planner:
        backup = planner_plugin(choice.fallback_planner, exploring=exploring)
        planner.update({"planner_plugins": ["GridBased", "Fallback"], "Fallback.plugin": backup["plugin"],
                        "Fallback.allow_unknown": bool(exploring)})
        if choice.fallback_planner == "smac_lattice":
            planner["Fallback.allow_reverse_expansion"] = False
        if choice.fallback_planner == "smac_hybrid":
            planner["Fallback.motion_model_for_search"] = "DUBIN"
    expected["planner_server"] = planner
    controller = expected.setdefault("controller_server", {})
    controller["FollowPath.plugin"] = CONTROLLER_PLUGINS[choice.controller]
    if choice.controller == "shim_rpp":
        controller.update({"FollowPath.primary_controller": RPP_PLUGIN, "FollowPath.use_rotate_to_heading": False})
    if choice.controller == "mppi":
        for key in ("FollowPath.desired_linear_vel", "FollowPath.allow_reversing", "FollowPath.use_collision_detection"):
            controller.pop(key, None)
        controller.update({"FollowPath.vx_min": 0.0, "FollowPath.vx_max": profile.max_linear,
                           "FollowPath.wz_max": profile.max_angular, "FollowPath.motion_model": "DiffDrive",
                           "FollowPath.CostCritic.consider_footprint": True})
    for key, value in choice.controller_params:
        if not isinstance(value, dict):
            controller["FollowPath." + key] = value
    for key, value in choice.server_params:
        if not isinstance(value, dict):
            controller[key] = value
    if not choice.global_live_scans:
        plugins = expected["global_costmap/global_costmap"]["plugins"]
        expected["global_costmap/global_costmap"]["plugins"] = [p for p in plugins if p != "obstacle_layer"]
    if choice.local_padding_m:
        expected.setdefault("local_costmap/local_costmap", {})["footprint_padding"] = choice.local_padding_m
    return expected


def check_installed(choice):
    """Fail early when a selected plugin's resources are missing on this machine."""
    if choice.planner == "smac_lattice" and not LATTICE_DIFF_0_5.is_file():
        raise FileNotFoundError("lattice primitives missing: " + str(LATTICE_DIFF_0_5))
