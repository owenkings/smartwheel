"""Shadow-run options for the obstacle clouds; keeps the wiring in autonomy/shadow_cli.py to single calls.

Defaults leave a shadow run exactly as before (``--obstacle-input scan``): no extra
arrays in the recording, no extra topics, unchanged Nav2 parameters and result.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

from .nav2_obstacles import OBSTACLE_INPUTS, SIDES
from .obstacle_cloud import ObstacleFilter


def add_arguments(parser):
    group = parser.add_argument_group("obstacle clouds (window C)")
    group.add_argument("--obstacle-input", choices=OBSTACLE_INPUTS, default="scan",
                       help="local costmap and collision monitor input: scan (default, unchanged), cloud "
                            "(filtered 3D obstacle points replace /scan there) or scan_and_cloud")
    group.add_argument("--obstacle-right-cell", type=Path,
                       help="frozen single-right frontend of the same recording (right cloud and witness)")
    group.add_argument("--obstacle-sides", nargs="+", choices=SIDES,
                       help="lidars whose obstacle clouds are published (default: left, plus right with a right cell)")
    group.add_argument("--obstacle-no-consensus", action="store_true",
                       help="do not use the other lidar to reject unsupported clusters")
    group.add_argument("--obstacle-right-lever", default="frozen",
                       help="right lidar lever arm: frozen (field extrinsic in the right cell), symmetric (mirror of "
                            "the left mount) or 'X,Y,Z' in metres")
    group.add_argument("--obstacle-filter", default="{}", help="JSON object of ObstacleFilter overrides")


def check_arguments(parser, args):
    if args.obstacle_input == "scan":
        if args.obstacle_right_cell or args.obstacle_sides:
            parser.error("--obstacle-right-cell and --obstacle-sides need --obstacle-input cloud or scan_and_cloud")
        args.obstacle_filter_settings = None
        return
    try:
        args.obstacle_filter_settings = ObstacleFilter(**json.loads(args.obstacle_filter))
    except (TypeError, ValueError) as exc:
        parser.error("--obstacle-filter: " + str(exc))
    if not args.obstacle_sides:
        args.obstacle_sides = ["left", "right"] if args.obstacle_right_cell else ["left"]
    if "right" in args.obstacle_sides and not args.obstacle_right_cell:
        parser.error("the right obstacle cloud needs --obstacle-right-cell")
    lever = args.obstacle_right_lever
    if lever not in ("frozen", "symmetric"):
        try:
            values = [float(v) for v in lever.split(",")]
        except ValueError:
            values = []
        if len(values) != 3 or not all(math.isfinite(v) for v in values) or math.hypot(*values) > 3.0:
            parser.error("--obstacle-right-lever must be frozen, symmetric or X,Y,Z within 3 m")
        args.obstacle_right_lever = values


def _sides(args):
    return tuple(s for s in SIDES if s in (args.obstacle_sides or ()))


def prepare_kwargs(args):
    """Keyword arguments of shadow_obstacles.prepare, or None for the default scan input."""
    if args.obstacle_input == "scan":
        return None
    return {"sides": _sides(args), "right_cell": args.obstacle_right_cell, "cfg": args.obstacle_filter_settings,
            "footprint": args.footprint or (), "radius_m": None if args.footprint else args.assumed_radius,
            "consensus": not args.obstacle_no_consensus,
            "right_lever": None if args.obstacle_right_lever == "frozen" else args.obstacle_right_lever,
            "window_s": list(args.window) if getattr(args, "window", None) else None}


def profile_fields(args):
    if args.obstacle_input == "scan":
        return {}
    return {"obstacle_input": args.obstacle_input, "obstacle_clouds": _sides(args)}


def config_entries(args, recording):
    if args.obstacle_input == "scan":
        return {}
    from .shadow_obstacles import settings
    return {"obstacles": settings(recording)}


def result_entries(recording, config, profile, window):
    """Cloud analogue of the scan zone audit, only for runs that carry obstacle clouds."""
    from .shadow_obstacles import cloud_sides, zone_audit
    if not cloud_sides(recording):
        return {}
    from ..autonomy import nav2_config
    if profile.footprint:
        stop = nav2_config.offset_polygon(profile.footprint, nav2_config.STOP_RING_M)
    else:
        r = profile.radius_m + nav2_config.STOP_RING_M
        stop = [v for k in range(32) for v in (r * math.cos(2 * math.pi * k / 32), r * math.sin(2 * math.pi * k / 32))]
    audit = zone_audit(recording, window, stop, nav2_config.slowdown_polygon(profile), nav2_config.MONITOR_MAX_POINTS)
    return {"obstacle_cloud_audit": {"per_lidar": audit, "stop_polygon": stop,
                                     "slowdown_polygon": nav2_config.slowdown_polygon(profile),
                                     "monitor_max_points": nav2_config.MONITOR_MAX_POINTS,
                                     "basis": "EACH_CLOUD_AT_ITS_CAPTURE_POSE_BASE_LINK"}}
