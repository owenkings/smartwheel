"""Closed-loop autonomy simulation: autonomous exploration or a navigation mission.

``explore``  builds a map (odometry grid by default) while frontier goals drive Nav2.
``navigate`` localizes with AMCL on a saved map and drives a goal sequence.
Everything runs in localhost ROS domain 96 (97-101 with WC_AUTONOMY_ROS_DOMAIN)
against a simulated wheelchair.
Outputs go to the configured reports storage; nothing touches the vehicle.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import uuid

import numpy as np


def _finite(value):
    number = float(value)
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError("expected a finite number")
    return number


def _positive(value):
    number = _finite(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("expected a positive number")
    return number


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, required=True)
    sub = parser.add_subparsers(dest="mode", required=True)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--start", nargs=3, type=_finite, metavar=("X", "Y", "YAW"),
                        help="start pose in the simulated world frame (m, m, rad)")
    common.add_argument("--obstacle", nargs=3, type=_finite, action="append", default=[],
                        metavar=("X", "Y", "R"), help="extra disc in the world, absent from any map")
    common.add_argument("--assumed-radius", type=_positive, default=0.55,
                        help="assumed circular body radius (m); not a measured outline")
    common.add_argument("--margin", type=_positive, default=0.10)
    common.add_argument("--max-linear", type=_positive, default=0.30)
    common.add_argument("--max-angular", type=_positive, default=0.50)
    common.add_argument("--odometry-error", choices=["wheel_imu", "wheel_only", "ideal"], default="wheel_imu",
                        help="simulated odometry error model (default: wheel distance + gyro heading)")
    common.add_argument("--real-time-factor", type=_positive, default=1.0)
    common.add_argument("--max-duration", type=_positive, default=900.0, help="simulated seconds for the task")
    common.add_argument("--seed", type=int, default=1)
    explore = sub.add_parser("explore", parents=[common], help="autonomous mapping of an unknown world")
    world = explore.add_mutually_exclusive_group(required=True)
    world.add_argument("--world", help="synthetic world name: two_rooms or office_floor")
    world.add_argument("--world-map", type=Path, help="saved trinary map used as ground truth (unknown = solid)")
    explore.add_argument("--no-initial-spin", action="store_true")
    explore.add_argument("--no-return-home", action="store_true")
    explore.add_argument("--max-goals", type=int, default=40)
    explore.add_argument("--mapper", choices=["odom_grid", "odom_icp", "slam_toolbox"], default="odom_grid",
                         help="odom_grid: wheel+IMU odometry as pose authority, as in ADR 0001 (default); "
                              "odom_icp: experimental bounded scan-to-map correction (better in simulation, "
                              "worse on the recorded corridor); slam_toolbox: correlative scan matching")
    explore.add_argument("--slam-loop-closure", action="store_true",
                         help="with --mapper slam_toolbox: enable loop closure (off by default)")
    navigate = sub.add_parser("navigate", parents=[common], help="AMCL + Nav2 mission on a saved map")
    navigate.add_argument("--from-exploration", type=Path, required=True,
                          help="result directory of an explore run (world, saved map and frame link)")
    navigate.add_argument("--goal", nargs=3, type=_finite, action="append", default=[],
                          metavar=("X", "Y", "YAW"), help="map-frame goal; default chooses far reachable goals")
    navigate.add_argument("--auto-goals", type=int, default=3)
    navigate.add_argument("--initial-pose-error", nargs=3, type=_finite, default=[0.15, -0.10, 0.06],
                          metavar=("DX", "DY", "DYAW"), help="error added to the AMCL initial pose (map frame)")
    navigate.add_argument("--initial-pose-std", nargs=2, type=_positive, default=[0.25, 0.15],
                          metavar=("XY_M", "YAW_RAD"), help="uncertainty sent with the initial pose")
    args = parser.parse_args(argv)
    if not 0.1 <= args.real_time_factor <= 4.0:
        parser.error("--real-time-factor must be within 0.1..4")
    if args.mode == "explore" and args.slam_loop_closure and args.mapper != "slam_toolbox":
        parser.error("--slam-loop-closure requires --mapper slam_toolbox")
    if args.max_linear > 0.6 or args.max_angular > 1.0:
        parser.error("speed limits above 0.6 m/s or 1.0 rad/s are outside this simulation's scope")
    return args


def _write_json(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def _auto_goals(map_yaml, start_map, radius, margin, count):
    """Farthest-point goals in reachable, inflated known-free space of the saved map."""
    from ..geometry import Grid, inflate_grid
    from .frontier import reachable_distances, with_robot_disc_free
    from .maps import read_back
    _, data, meta = read_back(map_yaml)
    grid = with_robot_disc_free(Grid(data, meta["resolution"], tuple(meta["origin"])), start_map[:2], radius + margin)
    inflated = inflate_grid(grid, radius + grid.resolution * math.sqrt(2), margin)
    start = grid.world_to_cell(start_map[:2])
    distances = reachable_distances(inflated.free, start, grid.resolution)
    rows, cols = np.nonzero(np.isfinite(distances))
    if len(rows) < 2:
        raise ValueError("saved map has no reachable free space around the start")
    xs = grid.origin[0] + (cols + 0.5) * grid.resolution
    ys = grid.origin[1] + (rows + 0.5) * grid.resolution
    chosen = [tuple(start_map[:2])]
    nearest = np.hypot(xs - start_map[0], ys - start_map[1])
    goals = []
    for _ in range(max(1, count)):
        k = int(np.argmax(nearest))
        if nearest[k] < 1.0:
            break
        x, y = float(xs[k]), float(ys[k])
        px, py = chosen[-1]
        goals.append([x, y, math.atan2(y - py, x - px)])
        chosen.append((x, y))
        nearest = np.minimum(nearest, np.hypot(xs - x, ys - y))
    return goals


def build_config(args):
    from .kinematics import OdometryErrorModel
    from .nav2_config import NavProfile
    from .ros_util import compose, inverse
    from .world import CircleObstacle, LaserModel, auto_start, map_world, synthetic_world
    profile = NavProfile(radius_m=args.assumed_radius, margin_m=args.margin,
                         max_linear=args.max_linear, max_angular=args.max_angular)
    obstacles = [list(o) for o in args.obstacle]
    for o in obstacles:
        CircleObstacle(*o)
    config = {"schema_version": 1, "mode": args.mode, "profile": profile.as_dict(),
              "laser": LaserModel().as_dict(),
              "odometry": OdometryErrorModel.preset(args.odometry_error, seed=args.seed).as_dict(),
              "real_time_factor": args.real_time_factor, "max_duration_s": args.max_duration,
              "seed": args.seed, "simulation_only": True, "hardware_control": False}
    if args.mode == "explore":
        if args.world:
            world, default_start = synthetic_world(args.world, obstacles)
        else:
            world = map_world(args.world_map, obstacles)
            default_start = auto_start(world, profile.radius_m, profile.margin_m)
        start = tuple(args.start) if args.start else tuple(default_start)
        spec = dict(world.source, obstacles=obstacles)
        config.update(world=spec, start_pose_world=list(start), world_T_map=list(start),
                      mapper=args.mapper, slam_loop_closure=bool(args.slam_loop_closure),
                      explorer={"initial_spin": not args.no_initial_spin, "return_home": not args.no_return_home,
                                "max_goals": args.max_goals})
    else:
        source = args.from_exploration.expanduser().resolve(strict=True)
        previous = json.loads((source / "result.json").read_text(encoding="utf-8"))
        explored = previous.get("exploration") or {}
        saved = (explored.get("map") or {}).get("yaml")
        if previous.get("mode") != "explore" or not saved:
            raise ValueError("--from-exploration must point at an explore run that saved a map")
        run_config = previous["config"]
        spec = dict(run_config["world"], obstacles=obstacles)
        world_T_map = tuple(run_config["world_T_map"])
        start = tuple(args.start) if args.start else tuple(run_config["start_pose_world"])
        start_map = compose(inverse(world_T_map), start)
        initial = (start_map[0] + args.initial_pose_error[0], start_map[1] + args.initial_pose_error[1],
                   start_map[2] + args.initial_pose_error[2])
        goals = [list(g) for g in args.goal] or _auto_goals(saved, start_map, profile.radius_m,
                                                          profile.margin_m, args.auto_goals)
        if not goals:
            raise ValueError("no navigation goals were given or found")
        config.update(world=spec, start_pose_world=list(start), world_T_map=list(world_T_map),
                      map_yaml=saved, source_exploration=str(source), initial_pose_map=list(initial),
                      initial_pose_std=list(args.initial_pose_std),
                      true_start_map=list(start_map), goals_map=goals)
    return config


def summarize(config, report, output):
    """Turn the supervisor report into result.json, trajectory.csv and preview.png."""
    from . import report as metrics
    from .maps import read_back
    from .ros_util import compose
    from .world import load_world
    world = load_world(config["world"])
    radius = config["profile"]["radius_m"]
    recorded = report.pop("recorded", {})
    rows = recorded.get("rows", [])
    metrics.write_trajectory(output / "trajectory.csv", rows)
    with (output / "events.jsonl").open("x", encoding="utf-8") as stream:
        for event in recorded.get("events", []):
            stream.write(json.dumps(event, ensure_ascii=False) + "\n")
    sim = recorded.get("final_sim_status") or {}
    gate = recorded.get("final_gate_status") or {}
    collisions = [e["value"] for e in recorded.get("events", []) if e["kind"] == "collision"]
    result = {"schema_version": 1, "mode": config["mode"], "status": "FAIL", "config": config,
              "simulation_only": True, "hardware_control": False, "physical_navigation_validated": False,
              "supervisor": report,
              "trajectory": metrics.trajectory_metrics(world, rows, radius, config.get("start_pose_world")),
              "collisions": {"count": int(sim.get("collisions", 0)), "events": collisions},
              "distance_m": sim.get("distance_m"), "gate": {"final": gate, "tick_period_s": 0.05}}
    goals_plot = []
    map_layer = None
    if config["mode"] == "explore":
        path = output / "explorer_result.json"
        explored = json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}
        result["exploration"] = explored
        saved = (explored.get("map") or {}).get("yaml")
        if saved:
            _, data, meta = read_back(saved)
            result["coverage"] = metrics.coverage_metrics(world, config["start_pose_world"][:2], data,
                                                          meta["resolution"], meta["origin"], config["world_T_map"],
                                                          radius_m=radius + config["profile"]["margin_m"])
            map_layer = metrics.map_in_world(world, data, meta["resolution"], meta["origin"], config["world_T_map"])
        for g in explored.get("goals", []):
            if g.get("goal_xy"):
                x, y, _ = compose(config["world_T_map"], (*g["goal_xy"], 0.0))
                goals_plot.append((x, y, bool(g.get("succeeded"))))
        ok = explored.get("status") == "COMPLETE" and saved
    else:
        path = output / "mission_result.json"
        mission = json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}
        result["mission"] = mission
        finals = []
        truth = np.asarray(rows, dtype=float) if rows else np.zeros((0, 4))
        for record in mission.get("records", []):
            goal_world = compose(config["world_T_map"], tuple(record["goal"]))
            final = None
            if len(truth):
                end_time = record.get("finished_time_s", truth[-1, 0])
                k = int(np.argmin(np.abs(truth[:, 0] - end_time)))
                final = truth[k, 1:4]
            entry = {"index": record["index"], "succeeded": record["succeeded"], "goal_world": list(goal_world)}
            if final is not None:
                entry["true_position_error_m"] = float(math.hypot(final[0] - goal_world[0], final[1] - goal_world[1]))
                entry["true_yaw_error_deg"] = float(abs(math.degrees(math.atan2(
                    math.sin(final[2] - goal_world[2]), math.cos(final[2] - goal_world[2])))))
            finals.append(entry)
            goals_plot.append((goal_world[0], goal_world[1], record["succeeded"]))
        result["goal_outcomes"] = finals
        ok = mission.get("status") == "COMPLETE"
    if ok and not result["collisions"]["count"] and "error" not in report:
        result["status"] = "PASS"
    elif "error" not in report and not result["collisions"]["count"]:
        result["status"] = "PARTIAL"
    try:
        title = f"{config['mode']} | {Path(str(config['world'].get('name') or config['world'].get('yaml'))).name}"
        metrics.render_preview(output / "preview.png", world, rows, title=title, map_layer=map_layer,
                               goals=goals_plot, collisions=[(c["x"], c["y"]) for c in collisions],
                               radius_m=radius)
        result["preview"] = str(output / "preview.png")
    except Exception as exc:  # the report must still be written
        result["preview_error"] = type(exc).__name__ + ": " + str(exc)
    _write_json(output / "result.json", result)
    return result


def main(argv=None):
    args = arguments(argv)
    from wc_runtime.storage_policy import StoragePolicy
    from .bringup import Supervisor, domain_lock
    storage = StoragePolicy(args.project_root)
    storage.check()
    config = build_config(args)
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + args.mode + "_" + uuid.uuid4().hex[:8]
    output = storage.resolve("reports/navigation_autonomy/" + run_id)
    with domain_lock(args.project_root):
        output.mkdir(parents=True, exist_ok=False)
        _write_json(output / "run_config.json", config)
        report = Supervisor(args.project_root, output, config).run()
    storage.check()
    result = summarize(config, report, output)
    brief = {"status": result["status"], "mode": result["mode"], "output": str(output),
             "collisions": result["collisions"]["count"], "distance_m": result.get("distance_m"),
             "error": report.get("error"), "task_exit_code": report.get("task_exit_code"),
             "localization_position_error_m": (result["trajectory"].get("localization_position_error_m") or {}),
             "coverage": result.get("coverage"), "goal_outcomes": result.get("goal_outcomes"),
             "simulation_only": True, "hardware_control": False}
    print(json.dumps(brief, ensure_ascii=True, allow_nan=False), flush=True)
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
