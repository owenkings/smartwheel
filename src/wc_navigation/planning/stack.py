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

Controllers: ``rpp`` (default Regulated Pure Pursuit), ``shim_rpp`` and ``mppi``
are added by the controller comparison (see ``CONTROLLERS``).
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
CONTROLLERS = ("rpp",)
GOAL_TOLERANCE_M = 0.25                                  # same as NavFn's GridBased.tolerance
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

    def __post_init__(self):
        if self.planner not in PLANNER_PLUGINS:
            raise ValueError("planner must be one of " + ", ".join(PLANNERS))
        if self.controller not in CONTROLLERS:
            raise ValueError("controller must be one of " + ", ".join(CONTROLLERS))
        for name in ("planner_params", "controller_params"):
            value = tuple((str(k), v) for k, v in (getattr(self, name) or ()))
            object.__setattr__(self, name, value)

    @property
    def is_default(self):
        return (self.planner == DEFAULT_PLANNER and self.controller == DEFAULT_CONTROLLER
                and not self.planner_params and not self.controller_params)

    def as_dict(self):
        return {"planner": self.planner, "controller": self.controller,
                "planner_params": [list(p) for p in self.planner_params],
                "controller_params": [list(p) for p in self.controller_params]}

    @classmethod
    def from_dict(cls, value):
        value = value or {}
        return cls(planner=value.get("planner", DEFAULT_PLANNER), controller=value.get("controller", DEFAULT_CONTROLLER),
                   planner_params=tuple(tuple(p) for p in value.get("planner_params", ())),
                   controller_params=tuple(tuple(p) for p in value.get("controller_params", ())))


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


def apply(params, profile, choice, *, exploring):
    """Replace planner and controller plugins in a ``navigation_parameters`` result, in place."""
    if choice.is_default:
        return params
    planner = params["planner_server"]["ros__parameters"]
    planner["GridBased"] = planner_plugin(choice.planner, exploring=exploring)
    for key, value in choice.planner_params:
        _set(planner["GridBased"], key, value)
    controller = params["controller_server"]["ros__parameters"]
    for key, value in choice.controller_params:
        _set(controller["FollowPath"], key, value)
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
    expected["planner_server"] = planner
    controller = expected.setdefault("controller_server", {})
    for key, value in choice.controller_params:
        if not isinstance(value, dict):
            controller["FollowPath." + key] = value
    return expected


def check_installed(choice):
    """Fail early when a selected plugin's resources are missing on this machine."""
    if choice.planner == "smac_lattice" and not LATTICE_DIFF_0_5.is_file():
        raise FileNotFoundError("lattice primitives missing: " + str(LATTICE_DIFF_0_5))
