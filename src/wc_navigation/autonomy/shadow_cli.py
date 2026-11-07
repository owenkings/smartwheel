"""Shadow run of the full Nav2 stack along a frozen single-left recording.

The recorded wheelchair motion (frozen wheel/IMU prior at the planar axle
reference) moves base_link, the recorded scans feed the costmaps and the
collision monitor, and Nav2 plans toward the recorded pose a few seconds
ahead. Every stage of the velocity chain is recorded and compared with the
driver's recorded motion; nothing subscribes to the gated command. Everything
runs in localhost ROS domain 96 (97-101 with WC_AUTONOMY_ROS_DOMAIN). Open loop:
the result shows agreement, slowdowns, stops and planning failures on real
sensor data, not what would happen with Nav2 in control.
"""
from __future__ import annotations

import argparse
import bisect
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import time
import uuid

import numpy as np

from . import nav2_config, shadow
from .bringup import EXECUTABLES, PACKAGES, Recorder, RunError, Supervisor, _same, domain_lock, planning_choice
from .ros_util import latched_qos, require_isolated_domain, simulation_domain, stamp_seconds

ANGLE_MIN, ANGLE_INCREMENT, BINS = -math.pi / 3, math.pi / 240, 161   # amcl_replay.build_scan bins
RANGE_LIMITS = (0.30, 8.0)
ACTIONS = ("compute_path_to_pose", "follow_path", "navigate_to_pose")
TERMINAL_STATUS = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}       # action_msgs/GoalStatus
PLANNER_TOLERANCE_M = 0.25                                             # GridBased.tolerance in nav2_config
# Log lines of the installed Humble 1.1.20 servers, counted per run.
LOG_PATTERNS = {
    # One planner_server line per failed request (NavFn adds its own "failed to create plan" line).
    "planner_failed": ("planner_server", r"failed to generate a valid path|plugin failed to plan"),
    "planner_start_or_goal_blocked": ("planner_server", r"lethal|off the global costmap"),
    "planner_start_or_goal_invalid": ("planner_server", r"no valid start or goal|goal is occupied"),
    "planner_iterations_exceeded": ("planner_server", r"exceeded maximum iterations|iterations has exceeded"),
    # Regulated Pure Pursuit and the rotation shim ("detected (a potential) collision ahead").
    "controller_collision_ahead": ("controller_server", r"collision ahead"),
    "controller_optimizer_failed": ("controller_server", r"optimizer fail"),          # MPPI
    "controller_missed_rate": ("controller_server", r"missed its desired rate"),
    "controller_no_progress": ("controller_server", r"failed to make progress"),
    "collision_monitor_stop": ("collision_monitor", r"robot to stop"),
    "collision_monitor_slowdown": ("collision_monitor", r"robot to slow ?down"),   # 1.1.20 prints "slowdown"
    "collision_monitor_approach": ("collision_monitor", r"robot to approach"),
    "collision_monitor_source_ignored": ("collision_monitor", r"ignoring the source"),
    "bt_goal_failed": ("bt_navigator", r"goal failed"),
    "bt_goal_succeeded": ("bt_navigator", r"goal succeeded"),
    "bt_goal_preempted": ("bt_navigator", r"preemption request"),
    "tf_message_filter_drops": ("*", r"message filter dropping message"),
}


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
    parser.add_argument("--source-cell", type=Path, required=True, help="exact frozen single-left frontend cell")
    parser.add_argument("--map", dest="map_yaml", type=Path, required=True,
                        help="trinary map YAML in the recording's odometry frame (e.g. a replay odom_grid map)")
    parser.add_argument("--window", nargs=2, type=_finite, metavar=("START", "END"),
                        help="recording seconds; default from 5 s before the first motion to the last frame")
    parser.add_argument("--mode", choices=["navigate", "follow"], default="navigate",
                        help="navigate: NavigateToPose toward the recorded pose ahead (planner chooses the route); "
                             "follow: the driver's recorded path goes straight to the controller (no planner)")
    parser.add_argument("--lead", type=_positive, default=15.0,
                        help="carrot goal / followed path: this many recorded seconds ahead")
    parser.add_argument("--update-period", type=_positive, default=4.0, help="s between carrot goal updates")
    parser.add_argument("--assumed-radius", type=_positive, default=0.55,
                        help="assumed circular body radius (m); not a measured outline")
    parser.add_argument("--footprint", nargs="+", type=_finite, metavar="X Y",
                        help="convex body outline in base_link (axle origin), x1 y1 x2 y2 ...; replaces the circle")
    parser.add_argument("--footprint-source", default="EXAMPLE_NOT_MEASURED",
                        help="what the outline is, recorded in the result (default: an example, not a measurement)")
    parser.add_argument("--margin", type=_positive, default=0.10)
    parser.add_argument("--max-linear", type=_positive, default=0.30)
    parser.add_argument("--max-angular", type=_positive, default=0.50)
    parser.add_argument("--scan-height", nargs=2, type=_finite, default=[-0.5, 0.5], metavar=("MIN", "MAX"),
                        help="height band in the prepared lidar frame, as in the AMCL and mapping replays")
    parser.add_argument("--real-time-factor", type=_positive, default=1.0)
    from ..planning.options import add_planning_arguments, resolve_planning_arguments
    add_planning_arguments(parser)
    args = parser.parse_args(argv)
    resolve_planning_arguments(parser, args)
    if args.footprint is not None:
        if len(args.footprint) < 6 or len(args.footprint) % 2:
            parser.error("--footprint needs at least three x y pairs")
        args.footprint = [args.footprint[k:k + 2] for k in range(0, len(args.footprint), 2)]
        try:
            nav2_config.NavProfile(footprint=args.footprint)
        except ValueError as exc:
            parser.error("--footprint: " + str(exc))
    if not 0.25 <= args.real_time_factor <= 2.0:
        parser.error("--real-time-factor must be within 0.25..2")
    if args.window and not 0 <= args.window[0] < args.window[1]:
        parser.error("--window needs 0 <= START < END")
    if not 1.0 <= args.lead <= 60.0 or not 0.5 <= args.update_period <= 30.0:
        parser.error("--lead must be within 1..60 s and --update-period within 0.5..30 s")
    if args.max_linear > 0.6 or args.max_angular > 1.0:
        parser.error("speed limits above 0.6 m/s or 1.0 rad/s are outside this run's scope")
    if not -2 < args.scan_height[0] < args.scan_height[1] < 2:
        parser.error("--scan-height needs -2 < MIN < MAX < 2")
    return args


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def prepare_recording(source, scan_height):
    """Hashed frozen frames -> planar axle poses and 161-bin scans (missing bins stay inf)."""
    from ..amcl_replay import build_scan
    from ..rebuild import validate_frontend_row
    from ..replay_cli import checked_frames, select_rows
    from .replay_mapping import planar
    source = Path(source).expanduser().resolve(strict=True)
    runtime_raw = (source / "runtime_config.json").read_bytes()
    index_raw = (source / "frontend/index.jsonl").read_bytes()
    runtime = json.loads(runtime_raw)
    if runtime.get("mode") != "left" or runtime.get("offline_source_selection", {}).get("selected_sides") != ["left"]:
        raise ValueError("exact frozen single-left source is required")
    if runtime.get("motion_model") != "planar_ekf":
        raise ValueError("frozen planar FLU reference is required for the axle lever arm")
    lever = np.asarray(runtime["mounts"]["left"]["t_axle_lidar_m"][:2], dtype=float)
    if lever.shape != (2,) or not np.isfinite(lever).all() or np.hypot(*lever) > 2.0:
        raise ValueError("finite frozen planar lever arm required")
    rows = [json.loads(line) for line in index_raw.decode("utf-8").splitlines()]
    stamps = [validate_frontend_row(row) for row in rows]
    if len(stamps) < 2 or any(a >= b for a, b in zip(stamps, stamps[1:])):
        raise ValueError("unique increasing source timestamps required")
    hashes = {str(source / "runtime_config.json"): hashlib.sha256(runtime_raw).hexdigest(),
              str(source / "frontend/index.jsonl"): hashlib.sha256(index_raw).hexdigest()}
    frames, _ = checked_frames(source, select_rows(rows, stamps[0], (0.0, 1e7), 0.0), runtime, hashes)
    ranges, observed = [], []
    for frame in frames:
        scan, stats = build_scan(frame["points"], *scan_height)
        if len(scan) != BINS:
            raise ValueError("unexpected scan bin count")
        ranges.append(scan)
        observed.append(stats["observed_bins"])
    stamps_ns = np.asarray([f["stamp_ns"] for f in frames], dtype=np.int64)
    lidar = np.asarray([planar(f["odometry_T"]) for f in frames], dtype=float)
    recording = {"stamps_ns": stamps_ns, "times_s": (stamps_ns - stamps_ns[0]).astype(np.float64) * 1e-9,
                 "lidar_poses": lidar, "base_poses": shadow.base_poses_from_lidar(lidar, lever),
                 "ranges": np.asarray(ranges, dtype=np.float32), "observed_bins": np.asarray(observed, dtype=np.int32),
                 "lever_xy": lever, "angle_min": np.float64(ANGLE_MIN), "angle_increment": np.float64(ANGLE_INCREMENT),
                 "range_limits": np.asarray(RANGE_LIMITS, dtype=float)}
    return source, recording, hashes


def map_fit(map_yaml, poses):
    """Share of recorded axle positions on free / occupied / unknown / outside cells of the map."""
    from .maps import read_back
    _, data, meta = read_back(map_yaml)
    res, origin = float(meta["resolution"]), meta["origin"]
    cols = np.floor((poses[:, 0] - origin[0]) / res).astype(int)
    rows = np.floor((poses[:, 1] - origin[1]) / res).astype(int)
    inside = (cols >= 0) & (cols < data.shape[1]) & (rows >= 0) & (rows < data.shape[0])
    values = np.full(len(poses), -2)
    values[inside] = data[rows[inside], cols[inside]]
    return {name: float(np.mean(values == code)) for name, code in
            (("free", 0), ("occupied", 100), ("unknown", -1), ("outside", -2))}


def map_provenance(map_yaml, source):
    """Whether the map came from a replay of this same recording (navigation_replay_mapping layout)."""
    result = Path(map_yaml).resolve().parent.parent / "result.json"
    try:
        value = json.loads(result.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return "UNKNOWN"
    if Path(str(value.get("source_cell", ""))).resolve() == Path(source).resolve():
        return "SAME_RECORDING_REPLAY_MAP"
    return "OTHER_SOURCE_REPLAY_MAP"


class ShadowRecorder(Recorder):
    """Supervisor node: recorded pose, every velocity-chain stage, plans and status."""

    def __init__(self, context):
        super().__init__(context, (0.0, 0.0, 0.0))
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry, Path as PathMessage
        from std_msgs.msg import String
        node = self.node
        self.commands = {stage: [] for stage in shadow.STAGES}
        topics = (nav2_config.CMD_NAV, nav2_config.CMD_SMOOTHED, nav2_config.CMD_MONITORED, nav2_config.CMD_GATED)
        for stage, topic in zip(shadow.STAGES, topics):
            node.create_subscription(Twist, topic, lambda m, s=stage: self.commands[s].append(
                (round(self.now(), 4), m.linear.x, m.angular.z)), 50)
        node.create_subscription(Odometry, "/wc_shadow/recorded", lambda m: setattr(self, "truth", self._pose(m)), 20)
        node.create_subscription(PathMessage, "/plan", self._on_plan, 10)
        node.create_subscription(String, "/wc_shadow/status", self._on_replay, latched_qos())
        from action_msgs.msg import GoalStatusArray
        from nav2_msgs.msg import Costmap
        for action in ACTIONS:
            node.create_subscription(GoalStatusArray, "/" + action + "/_action/status",
                                     lambda m, a=action: self._on_action_status(a, m), 50)
        node.create_subscription(Costmap, "/global_costmap/costmap_raw", self._on_costmap, 2)
        self.plans = []
        self.replay_status = None
        self.mission_state = None
        self.action_results = []
        self._finished_goals = set()
        self.costmaps = {"times_s": [], "packed": [], "layout": None, "layout_changes": 0}

    def _on_costmap(self, message):
        """Keep where the global planner was blocked (cost >= 253: inscribed, lethal, unknown), 1 bit per cell."""
        meta = message.metadata
        shape = (int(meta.size_y), int(meta.size_x))
        data = np.frombuffer(message.data, dtype=np.uint8)
        if data.size != shape[0] * shape[1]:
            return
        layout = {"shape": list(shape), "resolution": round(float(meta.resolution), 6),
                  "origin": [round(float(meta.origin.position.x), 6), round(float(meta.origin.position.y), 6)]}
        if self.costmaps["layout"] is None:
            self.costmaps["layout"] = layout
        elif layout != self.costmaps["layout"]:
            self.costmaps["layout_changes"] += 1
            return
        self.costmaps["times_s"].append(round(self.now(), 3))
        self.costmaps["packed"].append(np.packbits(data >= 253))

    def _on_action_status(self, action, message):
        """First terminal status of every goal of the planner, controller and navigator servers."""
        for status in message.status_list:
            key = (action, bytes(status.goal_info.goal_id.uuid))
            if status.status in TERMINAL_STATUS and key not in self._finished_goals:
                self._finished_goals.add(key)
                self.action_results.append({"action": action, "status": TERMINAL_STATUS[status.status],
                                            "time_s": round(self.now(), 3),
                                            "accepted_s": round(stamp_seconds(status.goal_info.stamp), 3)})

    def _event(self, kind, message):
        super()._event(kind, message)
        if kind == "mission_status":
            self.mission_state = self.events[-1]["value"].get("state")

    def _on_replay(self, message):
        value = json.loads(message.data)
        if self.replay_status is None or value.get("state") != self.replay_status.get("state"):
            self.events.append({"kind": "replay_state", "time_s": round(self.now(), 3), "value": value})
        self.replay_status = value

    def _on_plan(self, message):
        kept, last = [], None
        for pose in message.poses:
            x, y = pose.pose.position.x, pose.pose.position.y
            if last is None or math.hypot(x - last[0], y - last[1]) >= 0.1:
                kept.append([round(x, 3), round(y, 3)])
                last = (x, y)
        if message.poses:
            end = message.poses[-1].pose.position
            if kept[-1] != [round(end.x, 3), round(end.y, 3)]:
                kept.append([round(end.x, 3), round(end.y, 3)])
        self.plans.append({"time_s": round(self.now(), 3), "stamp_s": stamp_seconds(message.header.stamp),
                           "frame": message.header.frame_id, "xy": kept})


class ShadowSupervisor(Supervisor):
    def prepare(self):
        cfg = self.config
        profile = nav2_config.NavProfile(**cfg["profile"])
        (self.output / "logs").mkdir()
        files = {}
        tree = planning_choice(cfg).behavior_tree_options()
        files["bt_to_pose"] = self.write("bt_navigate_to_pose.xml", nav2_config.behavior_tree_xml(**tree))
        files["bt_through"] = self.write("bt_navigate_through_poses.xml",
                                         nav2_config.behavior_tree_xml(through_poses=True, **tree))
        files["nav2"] = self.write("nav2_params.yaml", nav2_config.navigation_parameters(
            profile, mode="navigate", bt_to_pose_path=files["bt_to_pose"], bt_through_poses_path=files["bt_through"],
            planning=planning_choice(cfg)))
        localization = nav2_config.localization_parameters(profile, cfg["map_yaml"])
        manager = dict(localization["lifecycle_manager_localization"]["ros__parameters"], node_names=["map_server"])
        files["mapping"] = self.write("map_server_params.yaml", {
            "map_server": localization["map_server"],
            "lifecycle_manager_localization": {"ros__parameters": manager}})
        files["replay"] = self.write("replay_config.json", {
            "recording": cfg["recording"], "window_s": cfg["window_s"], "real_time_factor": cfg["real_time_factor"],
            "tail_hold_s": cfg["tail_hold_s"], "hold_scan_period_s": 0.3, "odom_hz": 50.0, "start_time_s": 100.0})
        files["gate"] = self.write("gate_config.json", {"gate": {
            "max_linear": profile.max_linear, "max_angular": profile.max_angular, "require_covariance": False}})
        files["task"] = self.write("carrot_config.json", {
            "output_dir": str(self.output), "recording": cfg["recording"], "window_s": cfg["window_s"],
            "mode": cfg.get("shadow_mode", "navigate"), "lead_s": cfg["lead_s"],
            "update_period_s": cfg["update_period_s"]})
        return profile, files

    def run(self):
        require_isolated_domain()
        domain = simulation_domain()
        self.report["ros_domain_id"] = int(domain)
        for name in (*nav2_config.NAV2_NODES, "lifecycle_manager", "map_server"):
            if not EXECUTABLES[name].is_file():
                raise RunError("installed executable missing: " + str(EXECUTABLES[name]))
        versions = subprocess.run(["dpkg-query", "-W", "-f=${Package} ${Version}\n", *PACKAGES],
                                  capture_output=True, text=True, timeout=10.0)
        self.report["package_versions"] = versions.stdout.strip().splitlines()
        cfg = self.config
        profile, files = self.prepare()
        self.report["files"] = {k: str(v) for k, v in files.items()}
        import rclpy
        from rclpy.context import Context
        context = Context()
        rclpy.init(args=[], context=context)
        recorder = None
        self.t0 = time.monotonic()
        duration = cfg["window_s"][1] - cfg["window_s"][0] + cfg["tail_hold_s"]
        wall_budget = duration / cfg["real_time_factor"] * 1.3 + 300.0
        task_status = None
        try:
            recorder = ShadowRecorder(context)
            recorder.spin(2.0)
            foreign = recorder.foreign_nodes()
            if foreign:
                raise RunError(f"ROS domain {domain} is already in use: {foreign}")
            self.launch_python("replay", "wc_navigation.autonomy.replay_node", files["replay"])
            self.wait_for(recorder, lambda: recorder.clock_messages > 10 and recorder.truth is not None
                          and (recorder.replay_status or {}).get("state") == "HOLD", 30, "replay_clock")
            self.launch_ros("map_server", "map_server", files["mapping"])
            self.launch_ros("lifecycle_manager_localization", "lifecycle_manager", files["mapping"],
                            ["-r", "__node:=lifecycle_manager_localization"])
            self.wait_for(recorder, lambda: recorder.trigger("/lifecycle_manager_localization/is_active") is True,
                          60, "map_server_active")
            self.wait_for(recorder, lambda: recorder.map_messages > 0 and recorder.estimate_world() is not None,
                          30, "map_and_tf")
            for name in nav2_config.NAV2_NODES:
                self.launch_ros(name, name, files["nav2"])
            self.launch_ros("lifecycle_manager_navigation", "lifecycle_manager", files["nav2"],
                            ["-r", "__node:=lifecycle_manager_navigation"])
            self.wait_for(recorder, lambda: recorder.trigger("/lifecycle_manager_navigation/is_active") is True,
                          120, "nav2_active")
            readback = {}
            for node_name, expected in nav2_config.readback_expectations(profile, "navigate",
                                                                         planning=planning_choice(cfg)).items():
                actual = recorder.get_parameters(node_name, list(expected))
                readback[node_name] = {"expected": expected, "actual": actual}
                bad = [k for k in expected if not _same(actual.get(k), expected[k])]
                if bad:
                    raise RunError(f"parameter readback mismatch on {node_name}: {bad} -> {actual}")
            self.report["parameter_readback"] = readback
            self.launch_python("safety_gate", "wc_navigation.autonomy.gate_node", files["gate"])
            recorder.enabled = True
            self.wait_for(recorder, lambda: (recorder.gate_status or {}).get("state") in ("COMMAND_STALE", "ACTIVE"),
                          60, "gate_ready")
            process = self.launch_python("carrot", "wc_navigation.autonomy.carrot_node", files["task"])
            self.wait_for(recorder, lambda: recorder.mission_state == "READY", 60, "carrot_ready")
            if recorder.trigger("/wc_shadow/start", timeout_s=5.0) is not True:
                raise RunError("replay did not start")
            self.report["stages"].append({"stage": "playback_started", "wall_s": round(time.monotonic() - self.t0, 2),
                                          "sim_s": round(recorder.now(), 3)})
            while process.poll() is None:
                if time.monotonic() - self.t0 > wall_budget:
                    raise RunError("wall-clock budget exceeded")
                self.check_alive(allow=("carrot",))
                recorder.spin(0.2)
            task_status = process.returncode
            self.report["stages"].append({"stage": "carrot_finished", "exit_code": task_status,
                                          "wall_s": round(time.monotonic() - self.t0, 2),
                                          "sim_s": round(recorder.now(), 3)})
            recorder.enabled = False
            recorder.spin(1.0)
        except Exception as exc:
            self.report["error"] = type(exc).__name__ + ": " + str(exc)
        finally:
            if recorder is not None:
                recorder.enabled = False
                try:
                    recorder.spin(0.5)
                except Exception:
                    pass
            self.stop_all()
            if recorder is not None:
                self.report["recorded"] = {"rows": recorder.rows, "events": recorder.events,
                                           "commands": recorder.commands, "plans": recorder.plans,
                                           "action_results": recorder.action_results,
                                           "costmaps": recorder.costmaps,
                                           "final_gate_status": recorder.gate_status,
                                           "final_replay_status": recorder.replay_status}
                recorder.close()
            if context.ok():
                context.shutdown()
        self.report["task_exit_code"] = task_status
        self.report["logs"] = self.logs
        self.report["process_args"] = self.commands
        self.report["wall_s"] = round(time.monotonic() - self.t0, 2)
        return self.report


def _write_json(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def log_counts(logs):
    counts = {}
    for key, (process, pattern) in LOG_PATTERNS.items():
        expression = re.compile(pattern, re.IGNORECASE)
        total = 0
        for name, path in logs.items():
            if process != "*" and name != process:
                continue
            try:
                with open(path, encoding="utf-8", errors="replace") as stream:
                    total += sum(1 for line in stream if expression.search(line))
            except OSError:
                continue
        counts[key] = total
    return counts


def _json_safe(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    if isinstance(value, np.generic):
        return _json_safe(value.item())
    return value


LIVE_REASONS = {"START_ENCLOSED": "LIVE_COSTMAP_BLOCKS_THE_START", "GOAL_NOT_CONNECTED": "LIVE_COSTMAP_CUTS_THE_ROUTE",
                "REACHABLE": "NOT_EXPLAINED_BY_RECORDED_COSTMAP", "OUTSIDE_MAP": "OUTSIDE_MAP"}


class RecordedCostmaps:
    """Global-costmap snapshots (1 Hz) as blocked masks, for failures the static map does not explain."""

    def __init__(self, recorded):
        recorded = recorded or {}
        layout = recorded.get("layout")
        self.times = list(recorded.get("times_s", []))
        self.packed = list(recorded.get("packed", []))
        self.layout = layout
        self.layout_changes = int(recorded.get("layout_changes", 0))

    def __len__(self):
        return len(self.times) if self.layout else 0

    def at(self, sim_time):
        """Latest snapshot at or before ``sim_time`` as (blocked mask, age in s), or None."""
        k = bisect.bisect_right(self.times, sim_time) - 1
        if not len(self) or k < 0:
            return None
        shape = tuple(self.layout["shape"])
        blocked = np.unpackbits(np.asarray(self.packed[k], dtype=np.uint8), count=shape[0] * shape[1])
        return blocked.reshape(shape).astype(bool), sim_time - self.times[k]

    def reason(self, sim_time, start_xy, goal_xy):
        found = self.at(sim_time)
        if found is None:
            return "NO_RECORDED_COSTMAP", None
        blocked, age = found
        planning = shadow.StaticPlanningMap.from_blocked(blocked, self.layout["resolution"], self.layout["origin"])
        return LIVE_REASONS[planning.check(start_xy, goal_xy, PLANNER_TOLERANCE_M)], age

    def save(self, path):
        if not len(self):
            return None
        with Path(path).open("xb") as stream:
            np.savez_compressed(stream, times_s=np.asarray(self.times), packed=np.stack(self.packed),
                                shape=np.asarray(self.layout["shape"]), resolution=self.layout["resolution"],
                                origin=np.asarray(self.layout["origin"]))
        return str(path)


def planner_report(action_results, records, times, poses, w0, start, conservative, occupied_only, costmaps=None):
    """Terminal outcomes of the Nav2 action servers and a reason for each failed plan.

    ``action_results`` carry simulated receipt times; ``records`` are the carrot
    goals with their simulated send times. The robot pose at a failure is the
    recorded pose at the matching recording time. The static map is checked
    first; a failure it does not explain is checked against the latest recorded
    global costmap, which also holds live scan marks.
    """
    counts = {action: {} for action in ACTIONS}
    for row in action_results:
        bucket = counts.setdefault(row["action"], {})
        bucket[row["status"]] = bucket.get(row["status"], 0) + 1
    sent = sorted((float(r["sent_time_s"]), tuple(r["goal"])) for r in records if r.get("goal"))
    sent_times = [s for s, _ in sent]
    costmaps = costmaps or RecordedCostmaps(None)
    failures, reasons = [], {}
    for row in action_results:
        if row["action"] != "compute_path_to_pose" or row["status"] != "ABORTED":
            continue
        k = bisect.bisect_right(sent_times, row["time_s"]) - 1
        if k < 0:
            continue
        goal = sent[k][1]
        r = w0 + row["time_s"] - start
        x, y, _ = shadow.interpolate_poses(times, poses, r)[0]
        reason = shadow.attribute_planner_failure(conservative, occupied_only, (x, y), goal[:2], PLANNER_TOLERANCE_M)
        entry = {"recording_time_s": round(r, 3), "start_xy": [round(float(x), 3), round(float(y), 3)],
                 "goal_xy": [round(goal[0], 3), round(goal[1], 3)]}
        if reason == "NOT_EXPLAINED_BY_STATIC_MAP":
            reason, age = costmaps.reason(row["time_s"], (x, y), goal[:2])
            entry["costmap_age_s"] = None if age is None else round(age, 3)
        reasons[reason] = reasons.get(reason, 0) + 1
        to_occupied, to_unknown = conservative.clearance(x, y)
        entry.update(reason=reason, start_clearance_to_occupied_m=round(to_occupied, 3),
                     start_clearance_to_unknown_m=round(to_unknown, 3))
        failures.append(entry)
    planned = counts.get("compute_path_to_pose", {})
    attempts = sum(planned.values())
    # Server acceptance stamp to the terminal status (simulated clock, 10 ms steps; equals wall time at 1x).
    durations = {status: [r["time_s"] - r["accepted_s"] for r in action_results
                          if r["action"] == "compute_path_to_pose" and r["status"] == status
                          and r.get("accepted_s") is not None]
                 for status in ("SUCCEEDED", "ABORTED")}
    return {"terminal_status_counts": counts, "plan_requests": attempts,
            "plan_duration_s": {status: shadow._stats(values) for status, values in durations.items()},
            "plan_success_fraction": (planned.get("SUCCEEDED", 0) / attempts) if attempts else None,
            "failed_plans": len(failures), "failure_reasons": reasons, "failures": failures[:300],
            "planner_tolerance_m": PLANNER_TOLERANCE_M, "recorded_costmaps": len(costmaps),
            "costmap_layout_changes": costmaps.layout_changes,
            "status_note": ("follow_path and navigate_to_pose ABORTED include preemption: in Humble a new "
                            "path or goal aborts the previous handle; real failures are in log_counts"),
            "model": "STATIC_LAYER_THEN_RECORDED_GLOBAL_COSTMAP_NAVFN_LIKE_REACHABILITY"}


def summarize(config, report, output, recording):
    """Compare the recorded commands with the driver's motion; write result.json, samples.csv, preview.png."""
    from .replay_node import window_frames
    profile = nav2_config.NavProfile(**config["profile"])
    recorded = report.pop("recorded", {})
    with (output / "events.jsonl").open("x", encoding="utf-8") as stream:
        for event in recorded.get("events", []):
            stream.write(json.dumps(_json_safe(event), ensure_ascii=False) + "\n")
    replay = recorded.get("final_replay_status") or {}
    index = window_frames(recording, config["window_s"])
    times = recording["times_s"][index]
    poses = recording["base_poses"][index]
    ranges = recording["ranges"][index]
    angles = ANGLE_MIN + ANGLE_INCREMENT * np.arange(BINS)
    stop_radius = profile.radius_m + nav2_config.STOP_RING_M
    outline = profile.footprint
    stop_polygon = nav2_config.offset_polygon(outline, nav2_config.STOP_RING_M) if outline else None
    lethal_polygon = nav2_config.offset_polygon(outline, profile.margin_m) if outline else None
    audit = shadow.zone_audit(angles, ranges, recording["lever_xy"], stop_radius,
                              nav2_config.slowdown_polygon(profile), nav2_config.MONITOR_MAX_POINTS,
                              stop_polygon=stop_polygon)
    near = shadow.near_points(angles, ranges, recording["lever_xy"], profile.lethal_radius_m, polygon=lethal_polygon)
    mission_path = output / "shadow_mission_result.json"
    mission = json.loads(mission_path.read_text(encoding="utf-8")) if mission_path.exists() else {}
    result = {
        "schema_version": 1, "mode": "shadow", "shadow_mode": config.get("shadow_mode", "navigate"),
        "status": "INCOMPLETE", "config": config,
        "open_loop": True, "commands_sent": False, "hardware_control": False,
        "physical_navigation_validated": False, "supervisor": report, "replay": replay,
        "mission": {k: mission.get(k) for k in ("status", "goals_sent", "outcome_counts", "lead_s", "update_period_s")},
        "log_counts": log_counts(report.get("logs", {})),
        "scan_zone_audit": {
            "frames": int(len(times)), "stop_radius_m": None if outline else stop_radius,
            "stop_polygon": stop_polygon, "body_model": config.get("body_model", "CIRCLE_ASSUMED_NOT_MEASURED"),
            "slowdown_polygon": nav2_config.slowdown_polygon(profile),
            "monitor_max_points": nav2_config.MONITOR_MAX_POINTS,
            "frames_stop_expected": int(audit["stop_expected"].sum()),
            "frames_slow_expected": int(audit["slow_expected"].sum()),
            "stop_expected_times_s": [round(float(t), 3) for t in times[audit["stop_expected"]]],
            "nearest_return_m": shadow._stats(audit["nearest_m"]),
            "frames_with_returns_inside_lethal_radius": len({row[0] for row in near}),
            "returns_inside_lethal_radius": len(near),
            "lethal_radius_m": profile.lethal_radius_m,
            "basis": "EACH_SCAN_AT_ITS_CAPTURE_POSE_AXLE_FRAME_" + ("BODY_OUTLINE" if outline else "ASSUMED_CIRCLE")},
        "interpretation": ("OPEN_LOOP_SHADOW: the recording does not react to the commands; agreement and stop "
                           "counts describe this recording and the assumed circular body only"),
    }
    from .maps import read_back
    _, map_data, map_meta = read_back(config["map_yaml"])
    conservative = shadow.StaticPlanningMap(map_data, map_meta["resolution"], map_meta["origin"],
                                            profile.lethal_radius_m, inflate_unknown=True)
    result["static_map_audit"] = shadow.path_map_audit(conservative, poses[:, :2])
    table = None
    failures = []
    start = replay.get("playback_start_s")
    if start is not None:
        w0 = float(replay.get("window_s", [times[0]])[0])
        occupied_only = shadow.StaticPlanningMap(map_data, map_meta["resolution"], map_meta["origin"],
                                                 profile.lethal_radius_m, inflate_unknown=False)
        costmaps = RecordedCostmaps(recorded.get("costmaps"))
        result["planner"] = planner_report(recorded.get("action_results", []), mission.get("records", []),
                                           times, poses, w0, float(start), conservative, occupied_only, costmaps)
        result["planner"]["costmap_file"] = costmaps.save(output / "global_costmap_blocked.npz")
        failures = result["planner"]["failures"]
        commands = recorded.get("commands", {})
        samples = {}
        for stage in shadow.STAGES:
            rows = np.asarray(commands.get(stage, []), dtype=float).reshape(-1, 3)
            samples[stage] = (w0 + rows[:, 0] - float(start), rows[:, 1:3])
        end = float(times[-1])
        table, summary = shadow.evaluate(times, poses, (float(times[0]), end), samples)
        plans = [{"t": w0 + p["time_s"] - float(start), "xy": p["xy"]} for p in recorded.get("plans", [])
                 if p.get("xy")]
        plans = [p for p in plans if times[0] <= p["t"] <= end]
        plan_summary, per_plan = shadow.plan_agreement(times, poses, plans, horizon_s=config["lead_s"] / 2)
        if plans:
            # Open loop: Nav2 cannot pull the recording onto its path, so a lateral offset persists and the
            # controller's lookahead point (0.6 m) falls more than 0.5 rad off the heading -> turn in place.
            offset = shadow.offset_to_latest_plan(table["t"], shadow.interpolate_poses(times, poses, table["t"])[:, :2],
                                                  plans)
            rotating = table["moving"] & (table["outcome"] == "ROTATES_IN_PLACE") & np.isfinite(offset)
            plan_summary["offset_to_latest_plan_m_while_moving"] = shadow._stats(offset[table["moving"]])
            plan_summary["rotations_with_offset_over_0_3_m"] = (
                float(np.mean(offset[rotating] > 0.3)) if rotating.any() else None)
            table["offset_to_plan"] = offset
        result.update(commands=summary, plans=plan_summary,
                      command_messages={stage: len(commands.get(stage, [])) for stage in shadow.STAGES})
        _write_json(output / "plans.json", {"plans": plans, "per_plan": per_plan, "frame": "map"})
        with (output / "samples.csv").open("x", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["recording_time_s", "recorded_linear", "recorded_angular", "recording_moving",
                             *[f"{s}_{c}" for s in shadow.STAGES for c in ("linear", "angular")],
                             "outcome", "slowed_by_monitor", "offset_to_latest_plan_m"])
            offset = table.get("offset_to_plan", np.full(len(table["t"]), np.nan))
            for k in range(len(table["t"])):
                writer.writerow([round(float(table["t"][k]), 3), round(float(table["v_rec"][k]), 4),
                                 round(float(table["w_rec"][k]), 4), int(table["moving"][k]),
                                 *[("" if not math.isfinite(table[s][k, c]) else round(float(table[s][k, c]), 4))
                                   for s in shadow.STAGES for c in (0, 1)],
                                 table["outcome"][k], int(table["slowed"][k]),
                                 "" if not math.isfinite(offset[k]) else round(float(offset[k]), 3)])
        complete = (replay.get("state") == "FINISHED" and mission.get("status") == "COMPLETE"
                    and "error" not in report)
        result["status"] = "COMPLETE" if complete else "INCOMPLETE"
    try:
        render(output / "preview.png", config, times, poses, audit, table, recorded.get("plans", []),
               replay, profile, failures)
        result["preview"] = str(output / "preview.png")
    except Exception as exc:  # the report must still be written
        result["preview_error"] = type(exc).__name__ + ": " + str(exc)
    _write_json(output / "result.json", _json_safe(result))
    return result


FAILURE_STYLE = {"INFLATED_UNKNOWN_CUTS_THE_ROUTE": ("#7f3fbf", "unknown cells (inflated) cut the route"),
                 "START_WITHIN_LETHAL_RADIUS_OF_UNKNOWN": ("#b07ad6", "start within lethal radius of unknown"),
                 "START_WITHIN_LETHAL_RADIUS_OF_OCCUPIED": ("#c0392b", "start within lethal radius of occupied"),
                 "OCCUPIED_CELLS_CUT_THE_ROUTE_OR_GOAL": ("#7b241c", "occupied cells cut the route or goal"),
                 "LIVE_COSTMAP_BLOCKS_THE_START": ("#d35400", "live scan marks block the start"),
                 "LIVE_COSTMAP_CUTS_THE_ROUTE": ("#f39c12", "live scan marks cut the route"),
                 "NOT_EXPLAINED_BY_RECORDED_COSTMAP": ("#1f618d", "not explained by the recorded costmap"),
                 "NO_RECORDED_COSTMAP": ("#7f8c8d", "no costmap recorded yet"),
                 "OUTSIDE_MAP": ("#555555", "outside the map")}
OUTCOME_STYLE = (("STOPPED_BY_COLLISION_MONITOR", "#c0392b", "x"), ("NAV2_NOT_COMMANDING", "#e08e0b", "."),
                 ("CONTROLLER_ZERO", "#6c3483", "."), ("ROTATES_IN_PLACE", "#17a589", "."),
                 ("STOPPED_BY_GATE", "#8e44ad", "+"))


def render(path, config, times, poses, audit, table, plans, replay, profile, failures=()):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    from .maps import read_back
    _, data, meta = read_back(config["map_yaml"])
    res, origin = meta["resolution"], meta["origin"]
    fig, axes = plt.subplots(4, 1, figsize=(16, 15), gridspec_kw={"height_ratios": [2.4, 1, 1, 0.8]},
                             constrained_layout=True)
    ax = axes[0]
    image = np.where(data == 100, 0.0, np.where(data == 0, 1.0, 0.75))
    ax.imshow(image, origin="lower", cmap="gray", vmin=0, vmax=1,
              extent=[origin[0], origin[0] + data.shape[1] * res, origin[1], origin[1] + data.shape[0] * res])
    start = replay.get("playback_start_s")
    w0 = float((replay.get("window_s") or [times[0]])[0])
    shown = plans[::max(1, len(plans) // 25)]
    for k, plan in enumerate(shown):
        xy = np.asarray(plan["xy"], dtype=float).reshape(-1, 2)
        if len(xy):
            ax.plot(xy[:, 0], xy[:, 1], color="#2e9e5b", linewidth=0.8, alpha=0.55,
                    label="Nav2 global plan (sampled)" if k == 0 else None)
    ax.plot(poses[:, 0], poses[:, 1], color="#2b7bb9", linewidth=1.6, label="Recorded axle path (driver)")
    if profile.footprint:
        x0, y0, yaw0 = poses[0]
        c, s = np.cos(yaw0), np.sin(yaw0)
        body = np.array([[x0 + c * px - s * py, y0 + s * px + c * py] for px, py in profile.footprint])
        ax.add_patch(plt.Polygon(body, closed=True, fill=False, color="#2b7bb9", linewidth=1.2))
    else:
        ax.add_patch(plt.Circle(poses[0, :2], profile.radius_m, fill=False, color="#2b7bb9", linewidth=1.2))
    for reason, (color, label) in FAILURE_STYLE.items():
        xy = np.asarray([f["start_xy"] for f in failures if f["reason"] == reason], dtype=float).reshape(-1, 2)
        if len(xy):
            ax.scatter(xy[:, 0], xy[:, 1], marker="v", s=36, color=color, zorder=7,
                       label="plan failed: " + label + " (%d)" % len(xy))
    stop_xy = poses[audit["stop_expected"], :2]
    if len(stop_xy):
        ax.scatter(stop_xy[:, 0], stop_xy[:, 1], marker="o", s=40, facecolors="none", edgecolors="#c0392b",
                   label="Scan inside stop circle (> %d points)" % nav2_config.MONITOR_MAX_POINTS, zorder=5)
    if table is not None:
        at = shadow.interpolate_poses(times, poses, table["t"])
        for outcome, color, marker in OUTCOME_STYLE:
            mask = table["moving"] & (table["outcome"] == outcome)
            if mask.any():
                ax.scatter(at[mask, 0], at[mask, 1], marker=marker, s=18, color=color, zorder=6,
                           label=outcome.replace("_", " ").lower() + " while driver moves")
    ax.set_aspect("equal")
    ax.set_xlabel("map x (m) = recording odometry frame")
    ax.set_ylabel("map y (m)")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=3, fontsize=8)
    mode = config.get("shadow_mode", "navigate")
    choice = config.get("planning")
    if choice:
        mode += ", planner %s, controller %s" % (choice.get("planner"), choice.get("controller"))
    ax.set_title("SHADOW RUN (open loop, %s) on a frozen recording - commands recorded, never sent\n"
                 "%s; assumed circular body r=%.2f m (+%.2f m planning margin); not a vehicle test"
                 % (mode, "Nav2 plans toward the recorded pose ahead" if mode == "navigate"
                    else "controller follows the driver's recorded path", profile.radius_m, profile.margin_m)
                 if not profile.footprint else
                 "SHADOW RUN (open loop, %s) on a frozen recording - commands recorded, never sent\n"
                 "body outline %s (inscribed %.2f m, circumscribed %.2f m, +%.2f m planning margin); "
                 "not a vehicle test" % (mode, config.get("body_model", "POLYGON"), profile.body_inscribed_m,
                                         profile.body_circumscribed_m, profile.margin_m))
    if table is not None:
        t = table["t"]
        axes[1].plot(t, table["v_rec"], color="black", linewidth=1.2, label="recorded (driver)")
        axes[1].plot(t, table["nav"][:, 0], color="#2b7bb9", linewidth=0.9, linestyle="--", label="controller")
        axes[1].plot(t, table["gated"][:, 0], color="#1d8a4f", linewidth=1.1, label="after monitor + gate")
        axes[1].set_ylabel("linear (m/s)")
        axes[2].plot(t, table["w_rec"], color="black", linewidth=1.2, label="recorded (driver)")
        axes[2].plot(t, table["nav"][:, 1], color="#2b7bb9", linewidth=0.9, linestyle="--", label="controller")
        axes[2].plot(t, table["gated"][:, 1], color="#1d8a4f", linewidth=1.1, label="after monitor + gate")
        axes[2].set_ylabel("angular (rad/s)")
        for axis in axes[1:3]:
            for name, color, _ in OUTCOME_STYLE[:4]:
                for a, b in shadow._intervals(t, table["moving"] & (table["outcome"] == name)):
                    axis.axvspan(a, b, color=color, alpha=0.15, linewidth=0)
            axis.grid(alpha=0.25)
            axis.legend(loc="upper right", fontsize=8)
    axes[3].plot(times, np.where(np.isfinite(audit["nearest_m"]), audit["nearest_m"], np.nan), color="#555",
                 linewidth=1.0, label="nearest return to axle")
    axes[3].axhline(profile.body_inscribed_m + nav2_config.STOP_RING_M, color="#c0392b", linewidth=1.0,
                    linestyle="--", label="stop zone, nearest body edge" if profile.footprint else "stop circle")
    axes[3].set_ylim(0, 2.5)
    axes[3].set_ylabel("distance (m)")
    axes[3].set_xlabel("recording time (s)")
    axes[3].grid(alpha=0.25)
    axes[3].legend(loc="upper right", fontsize=8)
    fig.savefig(path, dpi=110)
    plt.close(fig)


def build_config(args, output, recording, source):
    profile = nav2_config.NavProfile(radius_m=args.assumed_radius, margin_m=args.margin,
                                     max_linear=args.max_linear, max_angular=args.max_angular,
                                     footprint=args.footprint or ())
    times, poses = recording["times_s"], recording["base_poses"]
    window = list(args.window) if args.window else list(shadow.motion_window(times, poses))
    window = [max(window[0], float(times[0])), min(window[1], float(times[-1]))]
    if window[1] - window[0] < 5.0:
        raise ValueError("the replay window must cover at least 5 s of the recording")
    map_yaml = args.map_yaml.expanduser().resolve(strict=True)
    from .replay_node import window_frames
    fit = map_fit(map_yaml, poses[window_frames(recording, window)])
    if fit["free"] < 0.5:
        raise ValueError(f"only {fit['free']:.0%} of the recorded path lies on free map cells; "
                         "the map must be in the recording's odometry frame")
    return {"schema_version": 1, "mode": "shadow", "profile": profile.as_dict(), "map_yaml": str(map_yaml),
            "map_sha256": _sha256(map_yaml), "map_fit_of_recorded_path": fit,
            "map_provenance": map_provenance(map_yaml, source), "source_cell": str(source),
            "recording": str(output / "recording.npz"), "window_s": window, "shadow_mode": args.mode,
            "lead_s": args.lead,
            "update_period_s": args.update_period, "tail_hold_s": 5.0, "scan_height_m": list(args.scan_height),
            "real_time_factor": args.real_time_factor, "localization": "IDENTITY_MAP_IS_RECORDING_ODOMETRY_FRAME",
            "base_reference": "FROZEN_PLANAR_AXLE_FROM_LEVER_ARM_UNVALIDATED",
            "body_model": ("POLYGON_" + args.footprint_source) if args.footprint else "CIRCLE_ASSUMED_NOT_MEASURED",
            "planning": args.planning.as_dict(),
            "open_loop": True, "commands_sent": False, "hardware_control": False}


def main(argv=None):
    args = arguments(argv)
    from wc_runtime.storage_policy import StoragePolicy
    storage = StoragePolicy(args.project_root)
    storage.check()
    source, recording, hashes = prepare_recording(args.source_cell, args.scan_height)
    run_id = (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_shadow_" + args.mode + "_"
              + uuid.uuid4().hex[:8])
    output = storage.resolve("reports/navigation_nav2_shadow/" + run_id)
    config = build_config(args, output, recording, source)
    with domain_lock(args.project_root):
        output.mkdir(parents=True, exist_ok=False)
        with (output / "recording.npz").open("xb") as stream:
            np.savez_compressed(stream, **recording)
        _write_json(output / "run_config.json", config)
        _write_json(output / "input_contract.json", {
            "source_cell": str(source), "source_sha256": hashes, "source_files_checked": len(hashes),
            "frames": int(len(recording["times_s"])), "map_yaml": config["map_yaml"],
            "map_sha256": config["map_sha256"], "scan": {"bins": BINS, "angle_min_rad": ANGLE_MIN,
                                                         "angle_increment_rad": ANGLE_INCREMENT,
                                                         "range_m": list(RANGE_LIMITS),
                                                         "height_band_m": list(args.scan_height),
                                                         "missing_bins": "inf, never cleared"},
            "lever_xy_m": recording["lever_xy"].tolist(), "hardware_started": False})
        report = ShadowSupervisor(args.project_root, output, config).run()
    storage.check()
    result = summarize(config, report, output, recording)
    unchanged = all(_sha256(path) == digest for path, digest in hashes.items()) and \
        _sha256(config["map_yaml"]) == config["map_sha256"]
    commands = result.get("commands") or {}
    brief = {"status": result["status"], "shadow_mode": args.mode, "output": str(output), "error": report.get("error"),
             "sources_unchanged": unchanged, "window_s": config["window_s"], "planning": config.get("planning"),
             "body_model": config.get("body_model"),
             "drives_forward_while_driver_drives_forward":
                 commands.get("drives_forward_fraction_while_recording_drives_forward"),
             "outcomes_while_moving": commands.get("outcome_fractions_while_recording_moves"),
             "turn_direction_agreement": commands.get("turn_direction_agreement"),
             "plans": {k: (result.get("plans") or {}).get(k) for k in (
                 "plans_compared", "max_deviation_m", "offset_to_latest_plan_m_while_moving",
                 "rotations_with_offset_over_0_3_m")},
             "planner": {k: (result.get("planner") or {}).get(k)
                         for k in ("plan_requests", "plan_success_fraction", "failure_reasons", "plan_duration_s")},
             "static_map_audit": {k: result["static_map_audit"][k] for k in (
                 "fraction_within_lethal_of_occupied", "fraction_within_lethal_of_unknown",
                 "unknown_pinholes_within_lethal_of_path")},
             "log_counts": result["log_counts"], "frames_stop_expected": result["scan_zone_audit"]["frames_stop_expected"],
             "commands_sent": False, "hardware_control": False}
    print(json.dumps(_json_safe(brief), ensure_ascii=True, allow_nan=False), flush=True)
    return 0 if result["status"] == "COMPLETE" and unchanged else 1


if __name__ == "__main__":
    raise SystemExit(main())
