"""Planner bench: every candidate planner answers the same recorded start/goal requests on one costmap.

One planner_server loads all selected planners side by side over the global
costmap of the navigate profile (static map + inflation; no live scans, so
only the planners differ). Requests come from a frozen recording: the recorded
axle pose at t is the start, the pose ``--lead`` seconds later the goal, every
``--every`` seconds. Each request goes to each planner through
ComputePathToPose with an explicit start; the server's own ``planning_time``
is recorded. Paths are then scored with the true body against the map
(planning.bench). Everything runs in the localhost ROS domain of this checkout
(98 for window B); nothing touches a vehicle.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time
import uuid

import numpy as np

from ..autonomy import nav2_config
from ..autonomy.bringup import EXECUTABLES, PACKAGES, RunError, Supervisor, _same, domain_lock
from ..autonomy.ros_util import (latched_qos, pose_stamped, quaternion_to_yaw, require_isolated_domain,
                                 simulation_domain, yaw_to_quaternion)
from . import bench, stack
from .body import Body
from .options import add_body_arguments, body_model_label, resolve_body_arguments


def _finite(value):
    number = float(value)
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError("expected a finite number")
    return number


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, required=True)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--recording", type=Path,
                        help="recording.npz of a shadow run (times_s, base_poses), or a frozen single-left source cell")
    source.add_argument("--requests-file", type=Path,
                        help='JSON list of {"start": [x, y, yaw], "goal": [x, y, yaw]} in the map frame (no recording)')
    parser.add_argument("--map", dest="map_yaml", type=Path, required=True, help="map in the recording's odometry frame")
    parser.add_argument("--label", default="", help="short name of the recording in the result (e.g. RoomA1)")
    parser.add_argument("--window", nargs=2, type=_finite, metavar=("START", "END"),
                        help="recording seconds; default: the shadow run's window or the whole recording")
    parser.add_argument("--lead", type=_finite, default=15.0, help="goal: recorded pose this many seconds ahead")
    parser.add_argument("--every", type=_finite, default=4.0, help="seconds between requests")
    parser.add_argument("--planners", nargs="+", choices=stack.PLANNERS, default=list(stack.PLANNERS))
    parser.add_argument("--planner-param", action="append", default=[], metavar="PLANNER.NAME=VALUE",
                        help="override one planner's parameter, e.g. smac_hybrid.minimum_turning_radius=0.6")
    parser.add_argument("--assumed-radius", type=_finite, default=0.55)
    parser.add_argument("--margin", type=_finite, default=0.10)
    parser.add_argument("--timeout", type=_finite, default=10.0, help="s to wait for one plan")
    parser.add_argument("--plan-radius", type=_finite,
                        help="plan with this circle (plus the margin) instead of the outline, still scoring the "
                             "outline: e.g. 0.35 m, the outline's half width, for grid planners")
    add_body_arguments(parser)
    args = parser.parse_args(argv)
    resolve_body_arguments(parser, args)
    if args.plan_radius is not None and (not args.footprint or not 0.1 <= args.plan_radius <= 1.0):
        parser.error("--plan-radius needs --footprint and must be within 0.1..1.0 m")
    if not 1.0 <= args.lead <= 60.0 or not 0.5 <= args.every <= 60.0:
        parser.error("--lead must be within 1..60 s and --every within 0.5..60 s")
    overrides = {}
    for item in args.planner_param:
        name, _, rest = item.partition(".")
        if name not in args.planners:
            parser.error("--planner-param must start with a selected planner name: " + item)
        try:
            overrides.setdefault(name, []).extend(stack.parse_overrides([rest]))
        except ValueError as exc:
            parser.error(str(exc))
        if any(k in ("plugin", "allow_unknown", "allow_reverse_expansion", "motion_model_for_search")
               for k, _ in overrides[name]):
            parser.error("plugin types, reversing and unknown-space use are fixed")
    args.overrides = overrides
    return args


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def load_recording(path):
    """(times, base poses, provenance, default window) from a shadow run's recording.npz or a source cell."""
    path = Path(path).expanduser().resolve(strict=True)
    if path.is_file():
        with np.load(path) as data:
            times, poses = np.asarray(data["times_s"], dtype=float), np.asarray(data["base_poses"], dtype=float)
        provenance = {"recording_npz": str(path), "sha256": _sha256(path)}
        window = None
        for name in ("run_config.json", "input_contract.json"):
            candidate = path.parent / name
            if candidate.is_file():
                value = json.loads(candidate.read_text(encoding="utf-8"))
                provenance[name] = {k: value.get(k) for k in ("source_cell", "source_sha256", "window_s", "map_yaml")
                                    if k in value}
                window = window or value.get("window_s")
        return times, poses, provenance, window
    from ..autonomy.shadow_cli import prepare_recording
    source, recording, hashes = prepare_recording(path, (-0.5, 0.5))
    return (recording["times_s"], recording["base_poses"], {"source_cell": str(source), "source_sha256": hashes},
            None)


def bench_parameters(profile, planners, overrides):
    """planner_server + global costmap of the navigate profile with every planner loaded; wall clock."""
    nav = nav2_config.navigation_parameters(profile, mode="navigate", bt_to_pose_path="/dev/null",
                                            bt_through_poses_path="/dev/null")
    server = dict(nav["planner_server"]["ros__parameters"])
    server.pop("GridBased")
    server.update(use_sim_time=False, planner_plugins=list(planners), expected_planner_frequency=0.0)
    for name in planners:
        plugin = stack.planner_plugin(name, exploring=False)
        for key, value in overrides.get(name, ()):
            stack._set(plugin, key, value)
        server[name] = plugin
    costmap = dict(nav["global_costmap"]["global_costmap"]["ros__parameters"])
    costmap.update(use_sim_time=False, plugins=["static_layer", "inflation_layer"])
    costmap.pop("obstacle_layer", None)
    return {"planner_server": {"ros__parameters": server},
            "global_costmap": {"global_costmap": {"ros__parameters": costmap}},
            "lifecycle_manager_bench": {"ros__parameters": {"use_sim_time": False, "autostart": True,
                                                            "node_names": ["planner_server"], "bond_timeout": 4.0}}}


class BenchClient:
    """Wall-clock node: static map->odom->base_link, ComputePathToPose client, costmap snapshot."""

    def __init__(self, context):
        import rclpy
        from geometry_msgs.msg import TransformStamped
        from nav2_msgs.action import ComputePathToPose
        from nav_msgs.msg import OccupancyGrid
        from rclpy.action import ActionClient
        from rclpy.executors import SingleThreadedExecutor
        from tf2_ros import StaticTransformBroadcaster
        self.node = rclpy.create_node("wc_planner_bench", context=context)
        self.executor = SingleThreadedExecutor(context=context)
        self.executor.add_node(self.node)
        self.Action = ComputePathToPose
        self.client = ActionClient(self.node, ComputePathToPose, "/compute_path_to_pose")
        self.costmap = None
        self.map_messages = 0
        self.node.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._on_costmap, latched_qos())
        self.node.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "map_messages",
                                                                              self.map_messages + 1), latched_qos())
        self.static = StaticTransformBroadcaster(self.node)
        transforms = []
        for parent, child in (("map", "odom"), ("odom", "base_link")):
            t = TransformStamped()
            t.header.stamp = self.node.get_clock().now().to_msg()
            t.header.frame_id, t.child_frame_id = parent, child
            t.transform.rotation.w = 1.0
            transforms.append(t)
        self.static.sendTransform(transforms)

    def _on_costmap(self, message):
        self.costmap = message

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=min(0.05, max(0.0, end - time.monotonic())))

    def now(self):
        return time.monotonic()

    def trigger(self, service, timeout_s=2.0):
        from std_srvs.srv import Trigger
        client = self.node.create_client(Trigger, service)
        try:
            if not client.wait_for_service(timeout_sec=0.5):
                return None
            future = client.call_async(Trigger.Request())
            end = time.monotonic() + timeout_s
            while not future.done() and time.monotonic() < end:
                self.executor.spin_once(timeout_sec=0.05)
            return future.result().success if future.done() and future.result() is not None else None
        finally:
            self.node.destroy_client(client)

    def get_parameters(self, node_name, names, timeout_s=5.0):
        from rcl_interfaces.srv import GetParameters
        from ..autonomy.bringup import _parameter_value
        client = self.node.create_client(GetParameters, "/" + node_name + "/get_parameters")
        try:
            if not client.wait_for_service(timeout_sec=timeout_s):
                raise RunError("parameter service unavailable: " + node_name)
            future = client.call_async(GetParameters.Request(names=list(names)))
            end = time.monotonic() + timeout_s
            while not future.done() and time.monotonic() < end:
                self.executor.spin_once(timeout_sec=0.05)
            if not future.done() or future.result() is None:
                raise RunError("parameter readback timed out: " + node_name)
            return dict(zip(names, (_parameter_value(v) for v in future.result().values)))
        finally:
            self.node.destroy_client(client)

    def foreign_nodes(self):
        own = (self.node.get_name(), self.node.get_namespace())
        return [n for n in self.node.get_node_names_and_namespaces() if n != own]

    def plan(self, planner, start, goal, timeout_s):
        """One ComputePathToPose with an explicit start; returns status, planning time, poses and wall time."""
        stamp = self.node.get_clock().now().to_msg()
        request = self.Action.Goal()
        request.goal = pose_stamped("map", stamp, *goal)
        request.start = pose_stamped("map", stamp, *start)
        request.planner_id, request.use_start = planner, True
        t0 = time.monotonic()
        future = self.client.send_goal_async(request)
        end = t0 + timeout_s
        while not future.done() and time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.01)
        if not future.done() or future.result() is None or not future.result().accepted:
            return {"status": "REJECTED_OR_TIMEOUT", "wall_s": time.monotonic() - t0}
        result_future = future.result().get_result_async()
        while not result_future.done() and time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.01)
        wall = time.monotonic() - t0
        if not result_future.done():
            future.result().cancel_goal_async()
            self.spin(0.5)
            return {"status": "TIMEOUT", "wall_s": wall}
        response = result_future.result()
        status = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}.get(int(response.status), str(response.status))
        planning = response.result.planning_time
        poses = [(p.pose.position.x, p.pose.position.y, quaternion_to_yaw(p.pose.orientation))
                 for p in response.result.path.poses]
        return {"status": status, "wall_s": wall, "planning_time_s": planning.sec + planning.nanosec * 1e-9,
                "poses": poses}

    def close(self):
        self.executor.remove_node(self.node)
        self.executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()


class BenchSupervisor(Supervisor):
    def prepare_files(self, profile, planners, overrides, map_yaml):
        (self.output / "logs").mkdir()
        files = {"planner": self.write("planner_params.yaml", bench_parameters(profile, planners, overrides))}
        localization = nav2_config.localization_parameters(profile, map_yaml)
        server = dict(localization["map_server"]["ros__parameters"], use_sim_time=False)
        files["map"] = self.write("map_server_params.yaml", {
            "map_server": {"ros__parameters": server},
            "lifecycle_manager_map": {"ros__parameters": {"use_sim_time": False, "autostart": True,
                                                          "node_names": ["map_server"], "bond_timeout": 4.0}}})
        return files

    def run_bench(self, profile, planners, overrides, map_yaml, requests, timeout_s):
        require_isolated_domain()
        for name in ("planner_server", "map_server", "lifecycle_manager"):
            if not EXECUTABLES[name].is_file():
                raise RunError("installed executable missing: " + str(EXECUTABLES[name]))
        versions = subprocess.run(["dpkg-query", "-W", "-f=${Package} ${Version}\n", *PACKAGES],
                                  capture_output=True, text=True, timeout=10.0)
        self.report["package_versions"] = versions.stdout.strip().splitlines()
        files = self.prepare_files(profile, planners, overrides, map_yaml)
        import rclpy
        from rclpy.context import Context
        context = Context()
        rclpy.init(args=[], context=context)
        client = None
        self.t0 = time.monotonic()
        answers = []
        try:
            client = BenchClient(context)
            client.spin(2.0)
            foreign = client.foreign_nodes()
            if foreign:
                raise RunError(f"ROS domain {simulation_domain()} is already in use: {foreign}")
            self.launch_ros("map_server", "map_server", files["map"])
            self.launch_ros("lifecycle_manager_map", "lifecycle_manager", files["map"],
                            ["-r", "__node:=lifecycle_manager_map"])
            self.wait_for(client, lambda: client.trigger("/lifecycle_manager_map/is_active") is True, 60, "map_active")
            self.launch_ros("planner_server", "planner_server", files["planner"])
            self.launch_ros("lifecycle_manager_bench", "lifecycle_manager", files["planner"],
                            ["-r", "__node:=lifecycle_manager_bench"])
            self.wait_for(client, lambda: client.trigger("/lifecycle_manager_bench/is_active") is True, 120,
                          "planner_active")
            self.wait_for(client, lambda: client.costmap is not None and client.client.server_is_ready(), 60,
                          "costmap_and_action")
            expected = {"planner_plugins": list(planners)}
            for name in planners:
                expected[name + ".plugin"] = stack.PLANNER_PLUGINS[name]
                expected[name + ".allow_unknown"] = False
                for key, value in overrides.get(name, ()):
                    if not isinstance(value, dict):
                        expected[name + "." + key] = value
            if "smac_hybrid" in planners:
                expected["smac_hybrid.motion_model_for_search"] = "DUBIN"
            if "smac_lattice" in planners:
                expected["smac_lattice.allow_reverse_expansion"] = False
            actual = client.get_parameters("planner_server", list(expected))
            bad = [k for k in expected if not _same(actual.get(k), expected[k])]
            if bad:
                raise RunError(f"parameter readback mismatch on planner_server: {bad} -> {actual}")
            body_key = "footprint" if profile.footprint else "robot_radius"
            costmap_expected = ({"footprint": nav2_config.footprint_string(profile.footprint),
                                 "footprint_padding": profile.margin_m} if profile.footprint
                                else {"robot_radius": profile.lethal_radius_m})
            costmap_actual = client.get_parameters("global_costmap/global_costmap", list(costmap_expected))
            if any(not _same(costmap_actual.get(k), v) for k, v in costmap_expected.items()):
                raise RunError(f"global costmap {body_key} readback mismatch: {costmap_actual}")
            self.report["parameter_readback"] = {"planner_server": actual, "global_costmap": costmap_actual}
            self.report["costmap"] = _costmap_summary(client.costmap)
            for request in requests:
                for name in planners:
                    answer = client.plan(name, request["start"], request["goal"], timeout_s)
                    answer.update(index=request["index"], planner=name)
                    answers.append(answer)
                self.check_alive()
            self.report["stages"].append({"stage": "requests_done", "wall_s": round(time.monotonic() - self.t0, 2)})
        except Exception as exc:
            self.report["error"] = type(exc).__name__ + ": " + str(exc)
        finally:
            self.stop_all()
            if client is not None:
                client.close()
            if context.ok():
                context.shutdown()
        self.report["logs"] = self.logs
        self.report["process_args"] = self.commands
        self.report["wall_s"] = round(time.monotonic() - self.t0, 2)
        return answers


def _costmap_summary(message):
    if message is None:
        return None
    data = np.asarray(message.data, dtype=np.int16)
    return {"size": [int(message.info.width), int(message.info.height)], "resolution": float(message.info.resolution),
            "lethal_or_inscribed_cells": int(np.sum(data >= 99)), "unknown_cells": int(np.sum(data < 0))}


def _write_json(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=1, allow_nan=False)
        stream.write("\n")


def _json_safe(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    if isinstance(value, np.generic):
        return _json_safe(value.item())
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    return value


def score(answers, requests, geometry, body, margin_m, planners):
    """Join answers with request context and path scores (true body, static occupied cells)."""
    by_index = {r["index"]: r for r in requests}
    contexts = {r["index"]: bench.request_context(geometry, body, margin_m, r) for r in requests}
    rows = []
    for answer in answers:
        request = by_index[answer["index"]]
        driver = request["driver"]
        row = {k: v for k, v in answer.items() if k != "poses"}
        row["context"] = contexts[answer["index"]]
        row["driver_length_m"] = float(np.sum(np.hypot(*np.diff(driver[:, :2], axis=0).T)))
        poses = np.asarray(answer.get("poses") or [], dtype=float).reshape(-1, 3)
        if answer["status"] == "SUCCEEDED" and len(poses) >= 2:
            use_yaw = answer["planner"] in stack.FOOTPRINT_PLANNERS
            row["score"] = bench.score_path(geometry, body, poses[:, :2], poses[:, 2] if use_yaw else None,
                                            driver_xy=driver[:, :2])
        rows.append(row)
    return rows, contexts


def render(path, data, meta, requests, answers, planners, label, body):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    res, origin = meta["resolution"], meta["origin"]
    extent = [origin[0], origin[0] + data.shape[1] * res, origin[1], origin[1] + data.shape[0] * res]
    image = np.where(data == 100, 0.0, np.where(data == 0, 1.0, 0.75))
    colors = {"navfn": "#1f77b4", "smac_2d": "#2ca02c", "theta_star": "#9467bd", "smac_hybrid": "#d62728",
              "smac_lattice": "#ff7f0e"}
    fig, axes = plt.subplots(len(planners), 1, figsize=(14, 4.2 * len(planners)), constrained_layout=True)
    axes = np.atleast_1d(axes)
    driver = np.vstack([r["driver"][:, :2] for r in requests]) if requests else np.zeros((0, 2))
    for ax, name in zip(axes, planners):
        ax.imshow(image, origin="lower", cmap="gray", vmin=0, vmax=1, extent=extent)
        if len(driver):
            ax.plot(driver[:, 0], driver[:, 1], ".", color="#2b7bb9", markersize=1.0)
        ok = failed = 0
        for answer in answers:
            if answer["planner"] != name:
                continue
            poses = np.asarray(answer.get("poses") or [], dtype=float).reshape(-1, 3)
            start = requests[answer["index"]]["start"]
            if answer["status"] == "SUCCEEDED" and len(poses):
                ax.plot(poses[:, 0], poses[:, 1], color=colors.get(name, "k"), linewidth=0.6, alpha=0.5)
                ok += 1
            else:
                ax.scatter([start[0]], [start[1]], marker="v", s=28, color="#c0392b", zorder=5)
                failed += 1
        ax.set_title("%s | %s: %d planned, %d failed (red: start of a failed request) | body %s"
                     % (label, name, ok, failed, body.source))
        ax.set_aspect("equal")
    fig.suptitle("PLANNER BENCH on a static map (no live scans) - recorded start/goal requests; not a vehicle test")
    fig.savefig(path, dpi=100)
    plt.close(fig)


def main(argv=None):
    args = arguments(argv)
    from wc_runtime.storage_policy import StoragePolicy
    from ..autonomy.maps import read_back
    storage = StoragePolicy(args.project_root)
    storage.check()
    profile = nav2_config.NavProfile(radius_m=args.assumed_radius, margin_m=args.margin, footprint=args.footprint or ())
    label = body_model_label(profile.footprint, args.footprint_source)
    body = Body.from_profile(profile, source=label)           # scored body: the outline (or the circle)
    if args.plan_radius is not None:                         # planned body: a circle, the outline only scored
        profile = nav2_config.NavProfile(radius_m=args.plan_radius, margin_m=args.margin)
    if args.requests_file is not None:
        listed = json.loads(args.requests_file.read_text(encoding="utf-8"))
        requests = [{"index": k, "t": float(k), "goal_t": float(k), "start": [float(v) for v in r["start"]],
                     "goal": [float(v) for v in r["goal"]], "driver": np.asarray([r["start"], r["goal"]], dtype=float)}
                    for k, r in enumerate(listed)]
        provenance = {"requests_file": str(args.requests_file.resolve()), "sha256": _sha256(args.requests_file),
                      "note": "explicit requests: the route context covers only the start and goal poses"}
        window = None
    else:
        times, poses, provenance, default_window = load_recording(args.recording)
        window = list(args.window or default_window or [float(times[0]), float(times[-1])])
        requests = bench.make_requests(times, poses, window, args.lead, args.every)
    if not requests:
        raise SystemExit("no requests in the window")
    map_yaml = args.map_yaml.expanduser().resolve(strict=True)
    _, data, meta = read_back(map_yaml)
    geometry = bench.MapGeometry(data, meta["resolution"], meta["origin"])
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_bench_" + (args.label or "run") + "_" + \
        uuid.uuid4().hex[:6]
    output = storage.resolve("reports/navigation_planner_bench/" + run_id)
    config = {"schema_version": 1, "mode": "planner_bench", "label": args.label, "profile": profile.as_dict(),
              "body_model": label, "scored_body": body.as_config(), "plan_radius_m": args.plan_radius,
              "planners": list(args.planners),
              "planner_overrides": {k: [list(p) for p in v] for k, v in args.overrides.items()},
              "map_yaml": str(map_yaml), "map_sha256": _sha256(map_yaml), "recording": provenance,
              "window_s": window, "lead_s": args.lead, "every_s": args.every, "requests": len(requests),
              "costmap": "STATIC_LAYER_AND_INFLATION_OF_THE_NAVIGATE_PROFILE_NO_LIVE_SCANS",
              "scoring": "TRUE_BODY_NO_MARGIN_AGAINST_OCCUPIED_CELLS_GAP_CAPPED_1M",
              "open_loop": True, "commands_sent": False, "hardware_control": False}
    with domain_lock(args.project_root):
        output.mkdir(parents=True, exist_ok=False)
        _write_json(output / "run_config.json", _json_safe(config))
        supervisor = BenchSupervisor(args.project_root, output, config)
        answers = supervisor.run_bench(profile, args.planners, args.overrides, map_yaml, requests, args.timeout)
    storage.check()
    rows, _ = score(answers, requests, geometry, body, profile.margin_m, args.planners)
    summary = bench.summarize(rows, args.planners)
    result = {"schema_version": 1, "mode": "planner_bench", "status": "FAIL" if "error" in supervisor.report else "COMPLETE",
              "config": config, "supervisor": supervisor.report, "summary": summary,
              "interpretation": ("STATIC_MAP_PLANNER_BENCH: same requests and costmap for every planner; live scans, "
                                 "control and localization are not involved; gaps are for the true body")}
    with (output / "answers.jsonl").open("x", encoding="utf-8") as stream:
        for row, answer in zip(rows, answers):
            stream.write(json.dumps(_json_safe(dict(row, poses=[[round(v, 4) for v in p] for p in
                                                                 answer.get("poses", [])])), ensure_ascii=False) + "\n")
    _write_json(output / "requests.json", _json_safe([{k: (v.tolist() if isinstance(v, np.ndarray) else v)
                                                       for k, v in r.items()} for r in requests]))
    try:
        render(output / "preview.png", data, meta, requests, answers, args.planners, args.label, body)
        result["preview"] = str(output / "preview.png")
    except Exception as exc:  # the result must still be written
        result["preview_error"] = type(exc).__name__ + ": " + str(exc)
    _write_json(output / "result.json", _json_safe(result))
    brief = {"status": result["status"], "output": str(output), "error": supervisor.report.get("error"),
             "body_model": label, "requests": len(requests),
             "categories": {c: summary[args.planners[0]][c]["requests"] for c in bench.CATEGORIES},
             "safe_success": {n: {c: summary[n][c]["safe_success_fraction"] for c in ("all", *bench.CATEGORIES)}
                              for n in args.planners},
             "unsafe_successes": {n: summary[n]["all"]["unsafe_succeeded"] for n in args.planners},
             "median_planning_time_s": {n: (summary[n]["all"]["planning_time_s"] or {}).get("median")
                                        for n in args.planners},
             "hardware_control": False}
    print(json.dumps(_json_safe(brief), ensure_ascii=True), flush=True)
    return 0 if result["status"] == "COMPLETE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
