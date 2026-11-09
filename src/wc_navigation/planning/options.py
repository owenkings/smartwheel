"""Command-line options shared by the simulation and shadow-run entry points.

The body outline given here is the one ``NavProfile.footprint`` carries into
the costmaps, the collision monitor and (in simulation) the collision truth.
"""
from __future__ import annotations

import argparse
import math
import re

CIRCLE_LABEL = "CIRCLE_ASSUMED_NOT_MEASURED"
DEFAULT_FOOTPRINT_SOURCE = "EXAMPLE_NOT_MEASURED"


def _finite(value):
    number = float(value)
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError("expected a finite number")
    return number


def add_body_arguments(parser):
    parser.add_argument("--footprint", nargs="+", type=_finite, metavar="X Y",
                        help="convex body outline in base_link (axle origin, x forward, y left), "
                             "x1 y1 x2 y2 ...; replaces the assumed circle everywhere, simulated collisions included")
    parser.add_argument("--footprint-source", default=None,
                        help="what the outline is, recorded in the result: EXAMPLE_NOT_MEASURED (default), "
                             "FRONT_FROM_BLIND_RECORDING_REST_EXAMPLE or MEASURED_<date>")


def resolve_body_arguments(parser, args):
    """Turn ``--footprint`` into [[x, y], ...] validated like NavProfile; fill the source label."""
    from ..autonomy.nav2_config import NavProfile
    source = args.footprint_source
    if args.footprint is None:
        if source is not None:
            parser.error("--footprint-source needs --footprint")
        return
    values = args.footprint
    if len(values) < 6 or len(values) % 2:
        parser.error("--footprint needs at least three x y pairs")
    args.footprint = [values[k:k + 2] for k in range(0, len(values), 2)]
    try:
        NavProfile(footprint=args.footprint)
    except ValueError as exc:
        parser.error("--footprint: " + str(exc))
    source = DEFAULT_FOOTPRINT_SOURCE if source is None else source
    if not re.fullmatch(r"[A-Z][A-Z0-9_]{2,63}", source):
        parser.error("--footprint-source must be an upper-case label such as MEASURED_20261120")
    args.footprint_source = source


def body_model_label(footprint, source):
    """Result label of the body: POLYGON_<source> for an outline, else the assumed circle."""
    return ("POLYGON_" + str(source or DEFAULT_FOOTPRINT_SOURCE)) if footprint else CIRCLE_LABEL


def add_planning_arguments(parser):
    from .stack import CONTROLLERS, DEFAULT_CONTROLLER, DEFAULT_PLANNER, PLANNERS
    parser.add_argument("--planner", choices=PLANNERS, default=DEFAULT_PLANNER,
                        help="global planner plugin (default navfn, unchanged); smac_hybrid and smac_lattice "
                             "check the body outline at every heading, the others pass by its inscribed radius")
    parser.add_argument("--planner-param", action="append", default=[], metavar="NAME=VALUE",
                        help="override a GridBased planner parameter (JSON value), e.g. minimum_turning_radius=0.6")
    parser.add_argument("--planner-fallback", choices=("none", *PLANNERS), default="none",
                        help="second planner the behaviour tree asks only when --planner finds no path "
                             "(e.g. smac_lattice, which can turn in place, behind forward-only smac_hybrid)")
    parser.add_argument("--controller", choices=CONTROLLERS, default=DEFAULT_CONTROLLER,
                        help="local controller plugin (default rpp: Regulated Pure Pursuit, unchanged)")
    parser.add_argument("--controller-param", action="append", default=[], metavar="NAME=VALUE",
                        help="override a FollowPath controller parameter (JSON value), e.g. use_rotate_to_heading=false")
    parser.add_argument("--checker-param", action="append", default=[], metavar="NAME=VALUE",
                        help="override a goal or progress checker parameter of the controller server, e.g. "
                             "general_goal_checker.xy_goal_tolerance=0.35 or progress_checker.movement_time_allowance=30")
    parser.add_argument("--recovery-retries", type=int, default=4,
                        help="behaviour-tree recovery rounds (clear costmaps, wait) before a goal fails (default 4)")
    parser.add_argument("--recovery-wait", type=float, default=3.0, help="seconds of each recovery wait (default 3)")
    parser.add_argument("--global-costmap", choices=["live", "static"], default="live",
                        help="live (default): the global costmap also marks live scans; static: plan on the saved "
                             "map only, live scans stay in the local costmap, the controller check and the monitor")
    parser.add_argument("--local-padding", type=float, default=0.0, metavar="M",
                        help="footprint padding of the local costmap, i.e. of the controller's collision checks "
                             "(default 0, unchanged); e.g. 0.10 keeps the planning margin while driving too")


def resolve_planning_arguments(parser, args):
    """``args.planning``: the PlanningChoice the options describe (the default one without options)."""
    from .stack import PlanningChoice, check_installed, parse_overrides
    try:
        args.planning = PlanningChoice(planner=args.planner, controller=args.controller,
                                       planner_params=parse_overrides(args.planner_param),
                                       controller_params=parse_overrides(args.controller_param),
                                       server_params=parse_overrides(args.checker_param),
                                       recovery_retries=args.recovery_retries, recovery_wait_s=args.recovery_wait,
                                       global_live_scans=args.global_costmap == "live",
                                       fallback_planner="" if args.planner_fallback == "none" else args.planner_fallback,
                                       local_padding_m=args.local_padding)
        check_installed(args.planning)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    fixed_controller = ("plugin", "primary_controller", "allow_reversing", "vx_min", "motion_model")
    fixed_planner = ("plugin", "allow_unknown", "allow_reverse_expansion", "motion_model_for_search")
    if any(key in fixed_controller for key, _ in args.planning.controller_params) or \
            any(key in fixed_planner for key, _ in args.planning.planner_params):
        parser.error("plugin types, reversing and unknown-space use are fixed by --planner/--controller and the mode")
