"""Shadow-run options for the obstacle clouds and the lidar timing; keeps the wiring in autonomy/shadow_cli.py
to single calls.

Defaults leave a shadow run exactly as before (``--obstacle-input scan``, ``--lidar-time-offset-ms 0``): no
extra arrays in the recording, no extra topics, unchanged Nav2 parameters and result.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from .nav2_obstacles import OBSTACLE_INPUTS, OBSTACLE_SCANS, SIDES
from .obstacle_cloud import ObstacleFilter

# The same bound as the live chain and the AMCL replay (live.scan_adapter.MAX_LIDAR_TIME_OFFSET_MS).
MAX_LIDAR_TIME_OFFSET_MS = 250.0


def add_arguments(parser):
    group = parser.add_argument_group("obstacle clouds (window C)")
    group.add_argument("--obstacle-input", choices=OBSTACLE_INPUTS, default="scan",
                       help="local costmap and collision monitor input: scan (default, unchanged), cloud "
                            "(filtered 3D obstacle points replace /scan there) or scan_and_cloud")
    group.add_argument("--obstacle-monitor", choices=OBSTACLE_INPUTS,
                       help="collision monitor input when it should differ from --obstacle-input, e.g. "
                            "--obstacle-input scan --obstacle-scan filtered --obstacle-monitor cloud")
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
    group.add_argument("--obstacle-scan", choices=OBSTACLE_SCANS, default="raw",
                       help="LaserScan of the costmaps' obstacle layers, including the global costmap's: raw /scan "
                            "(default) or filtered (/obstacles/left/scan: the left lidar's filtered obstacle points "
                            "in the same 161 bins). With --obstacle-input scan both costmaps stay 2D on the filtered "
                            "scan and the collision monitor keeps /scan")
    timing = parser.add_argument_group("lidar timing (window C)")
    timing.add_argument("--lidar-time-offset-ms", type=float, default=0.0, metavar="MS",
                        help="the lidar stamps are late by MS against the wheel/IMU odometry: every lidar frame "
                             "(/scan, both obstacle clouds, their clearing rays and the filtered scan) is paired "
                             "with the recorded pose of stamp - MS (default 0 = as recorded), as in the live chain "
                             "and the AMCL replay")


def _monitor(args):
    return args.obstacle_monitor or args.obstacle_input


def _off(args):
    """The default: no obstacle clouds prepared at all."""
    return args.obstacle_input == "scan" and args.obstacle_scan == "raw" and _monitor(args) == "scan"


def check_arguments(parser, args):
    offset = args.lidar_time_offset_ms
    if not math.isfinite(offset) or abs(offset) > MAX_LIDAR_TIME_OFFSET_MS:
        parser.error("--lidar-time-offset-ms must be finite and within +-%g" % MAX_LIDAR_TIME_OFFSET_MS)
    if _off(args):
        if args.obstacle_right_cell or args.obstacle_sides:
            parser.error("--obstacle-right-cell and --obstacle-sides need a cloud input (--obstacle-input or "
                         "--obstacle-monitor cloud|scan_and_cloud) or --obstacle-scan filtered")
        args.obstacle_filter_settings = None
        return
    if args.obstacle_input == "scan" and _monitor(args) == "scan":
        # only the left cloud is prepared, to build the filtered scan; the right cell is its witness
        if args.obstacle_sides and list(args.obstacle_sides) != ["left"]:
            parser.error("with --obstacle-input scan only the left cloud is used (for the filtered scan)")
        args.obstacle_sides = ["left"]
    try:
        args.obstacle_filter_settings = ObstacleFilter(**json.loads(args.obstacle_filter))
    except (TypeError, ValueError) as exc:
        parser.error("--obstacle-filter: " + str(exc))
    if not args.obstacle_sides:
        args.obstacle_sides = ["left", "right"] if args.obstacle_right_cell else ["left"]
    if "right" in args.obstacle_sides and not args.obstacle_right_cell:
        parser.error("the right obstacle cloud needs --obstacle-right-cell")
    if args.obstacle_scan == "filtered" and "left" not in args.obstacle_sides:
        parser.error("--obstacle-scan filtered needs the left obstacle cloud")
    if (args.obstacle_scan == "filtered" and args.obstacle_input == "cloud"
            and getattr(args, "global_costmap", "live") == "static"):
        parser.error("--obstacle-scan filtered with --obstacle-input cloud and --global-costmap static: "
                     "no costmap would read the filtered scan")
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
    if _off(args):
        return None
    return {"sides": _sides(args), "right_cell": args.obstacle_right_cell, "cfg": args.obstacle_filter_settings,
            "footprint": args.footprint or (), "radius_m": None if args.footprint else args.assumed_radius,
            "consensus": not args.obstacle_no_consensus,
            "right_lever": None if args.obstacle_right_lever == "frozen" else args.obstacle_right_lever,
            "window_s": list(args.window) if getattr(args, "window", None) else None,
            "scan": args.obstacle_scan == "filtered",
            "lidar_time_offset_s": args.lidar_time_offset_ms * 1e-3}


def recording_entries(args):
    """Extra recording.npz arrays: the lidar time offset the replay applies to every lidar stamp ({} at 0)."""
    if not args.lidar_time_offset_ms:
        return {}
    return {"lidar_time_offset_s": np.float64(args.lidar_time_offset_ms * 1e-3)}


def profile_fields(args):
    if _off(args):
        return {}
    monitor = _monitor(args)
    return {"obstacle_input": args.obstacle_input, "obstacle_clouds": _sides(args),
            "obstacle_scan": args.obstacle_scan,
            **({"obstacle_monitor": monitor} if monitor != args.obstacle_input else {})}


def config_entries(args, recording):
    entries = {}
    if args.lidar_time_offset_ms:
        entries["lidar_time_offset_ms"] = float(args.lidar_time_offset_ms)
    if _off(args):
        return entries
    from .shadow_obstacles import settings
    return {**entries, "obstacles": settings(recording)}


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
