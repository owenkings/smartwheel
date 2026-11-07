"""Localization modes of the recorded-data shadow run.

``identity`` (default, unchanged): the map was built in the recording's own
odometry frame, so the replay publishes a static identity ``map -> odom``.

``fixed``: the map comes from another recording. A static ``map -> odom``
equal to an explicit rigid alignment of the recording's odometry frame stands
in for localization: the cross-recording counterpart of identity, with no
correction of odometry drift.

``amcl``: map_server + AMCL localize on the saved map from the recorded scans
and odometry. The supervisor gives an operator-style start (the alignment
start plus an optional offset, with a stated uncertainty), AMCL alone
publishes ``map -> odom`` and the safety gate also requires AMCL's covariance.

In both new modes the replay's own static transforms are remapped away; the
helper node (shadow_localization_node) publishes ``base_link -> laser`` with
the same lever arm and, for ``fixed``, ``map -> odom``. Carrot goals and the
followed path are the recorded axle path placed in the map with the alignment.
With AMCL, the offset between that path and the robot's estimate is AMCL's
correction relative to the alignment, as when a global route stays fixed while
localization corrects the robot's position. The summary compares AMCL with the
alignment, evaluates the gate's localization conditions on the recorded AMCL
output and replays them offline with other limits.
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

MODES = ("identity", "fixed", "amcl")
LABELS = {"identity": "IDENTITY_MAP_IS_RECORDING_ODOMETRY_FRAME",
          "fixed": "FIXED_MAP_TO_ODOM_FROM_RIGID_ALIGNMENT_NO_CORRECTION",
          "amcl": "AMCL_ON_SAVED_MAP_FROM_ALIGNMENT_START"}
REPLAY_TF_STATIC = "/wc_shadow/replay_tf_static_unused"
SAMPLES_FILE = "localization_samples.csv"
MAP_RECORDING = "recording_map.npz"
# Gate defaults (safety_gate.GateConfig / CorrectionMonitor, since 2026-11-07) and the offline what-if grid.
GATE_TRANSLATION_M, GATE_YAW_DEG, GATE_WINDOW_S, GATE_HOLD_S = 0.25, 10.0, 2.0, 3.0
GATE_REFERENCE = "robot"
GATE_COV_XY_M, GATE_COV_YAW_RAD = 0.50, 0.25
WHAT_IF_TRANSLATION_M = (0.25, 0.35, 0.5, 0.75, 1.0)
WHAT_IF_YAW_DEG = (5.0, 8.0, 10.0, 15.0)
WHAT_IF_COV_XY_M = (0.3, 0.4, 0.5, 0.75)
WHAT_IF_SUPPORT = (0.2, 0.3, 0.4, 0.5)
WHAT_IF_SUPPORT_HOLD_S = 1.0


def _finite(value, label):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(label + " must be finite")
    return value


def add_arguments(parser):
    from ..live.amcl_cross import CANDIDATES
    group = parser.add_argument_group("localization (default identity: map in the recording's odometry frame)")
    group.add_argument("--localization", choices=MODES, default="identity",
                       help="identity (default); fixed: static map->odom from --map-alignment; "
                            "amcl: map_server + AMCL from the aligned start")
    group.add_argument("--map-alignment", nargs=3, type=float, metavar=("X_M", "Y_M", "YAW_DEG"),
                       help="fixed/amcl: rigid pose of the recording's odometry frame in the map")
    group.add_argument("--map-alignment-json", type=Path,
                       help="fixed/amcl: alignment check result with best_alignment (dx_m, dy_m, yaw_deg)")
    group.add_argument("--initial-pose-offset", nargs=3, type=float, metavar=("FORWARD_M", "LEFT_M", "YAW_DEG"),
                       help="amcl: operator start error in the start axle frame (default none)")
    group.add_argument("--initial-pose-std", nargs=2, type=float, metavar=("XY_M", "YAW_DEG"),
                       help="amcl: stated start uncertainty (default 0.25 m, 8.6 deg = 0.15 rad)")
    group.add_argument("--amcl-params", choices=("framework", *CANDIDATES),
                       help="amcl: nav2_config parameters (framework, default) or a named candidate change")
    group.add_argument("--gate-correction-limits", nargs=2, type=float, metavar=("TRANSLATION_M", "YAW_DEG"),
                       help="amcl: correction change within 2 s that stops the gate (default 0.25 m, 10 deg)")
    group.add_argument("--gate-covariance-limits", nargs=2, type=float, metavar=("XY_STD_M", "YAW_STD_DEG"),
                       help="amcl: largest AMCL standard deviations the gate accepts (default 0.50 m, 14.3 deg)")
    group.add_argument("--gate-correction-reference", choices=("odom_origin", "robot"),
                       help="amcl: measure the correction change as the move of the robot's map position "
                            "(robot, default) or as map->odom itself (odom_origin, the earlier measure)")


def check_arguments(parser, args):
    mode = args.localization
    specific = {"--map-alignment": args.map_alignment, "--map-alignment-json": args.map_alignment_json}
    amcl_only = {"--initial-pose-offset": args.initial_pose_offset, "--initial-pose-std": args.initial_pose_std,
                 "--amcl-params": args.amcl_params, "--gate-correction-limits": args.gate_correction_limits,
                 "--gate-covariance-limits": args.gate_covariance_limits,
                 "--gate-correction-reference": args.gate_correction_reference}
    if mode == "identity":
        given = [k for k, v in {**specific, **amcl_only}.items() if v is not None]
        if given:
            parser.error(", ".join(given) + " need --localization fixed or amcl")
        return
    if (args.map_alignment is None) == (args.map_alignment_json is None):
        parser.error("--localization %s needs exactly one of --map-alignment, --map-alignment-json" % mode)
    if mode == "fixed":
        given = [k for k, v in amcl_only.items() if v is not None]
        if given:
            parser.error(", ".join(given) + " need --localization amcl")
    values = [*(args.map_alignment or []), *(args.initial_pose_offset or []), *(args.initial_pose_std or []),
              *(args.gate_correction_limits or []), *(args.gate_covariance_limits or [])]
    if not all(math.isfinite(v) for v in values):
        parser.error("localization options must be finite")
    if args.initial_pose_offset and (math.hypot(*args.initial_pose_offset[:2]) > 2.0
                                     or abs(args.initial_pose_offset[2]) > 45.0):
        parser.error("--initial-pose-offset is limited to 2 m and 45 degrees")
    if args.initial_pose_std and not (0 < args.initial_pose_std[0] <= 2.0 and 0 < args.initial_pose_std[1] <= 90.0):
        parser.error("--initial-pose-std needs 0 < XY <= 2 m and 0 < YAW <= 90 degrees")
    for name in ("gate_correction_limits", "gate_covariance_limits"):
        value = getattr(args, name)
        if value is not None and not all(v > 0 for v in value):
            parser.error("--" + name.replace("_", "-") + " must be positive")


def alignment_from_args(args):
    if args.map_alignment_json is not None:
        path = args.map_alignment_json.expanduser().resolve(strict=True)
        raw = path.read_bytes()
        best = json.loads(raw)["best_alignment"]
        return {"x_m": _finite(best["dx_m"], "dx_m"), "y_m": _finite(best["dy_m"], "dy_m"),
                "yaw_deg": _finite(best["yaw_deg"], "yaw_deg"), "source": str(path),
                "sha256": hashlib.sha256(raw).hexdigest()}
    x, y, yaw = args.map_alignment
    return {"x_m": float(x), "y_m": float(y), "yaw_deg": float(yaw), "source": "COMMAND_LINE"}


def _xyyaw(alignment):
    return alignment["x_m"], alignment["y_m"], math.radians(alignment["yaw_deg"])


def compose_poses(transform, poses):
    """``transform * pose`` for each planar pose (x, y, yaw) of an (N, 3) array."""
    poses = np.asarray(poses, dtype=float).reshape(-1, 3)
    x, y, yaw = transform
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack([x + c * poses[:, 0] - s * poses[:, 1], y + s * poses[:, 0] + c * poses[:, 1],
                            np.arctan2(np.sin(poses[:, 2] + yaw), np.cos(poses[:, 2] + yaw))])


def active(config):
    return config.get("localization_mode", "identity") != "identity"


def config_section(args, output, recording, window):
    """Run-config entries of the chosen mode; identity adds nothing beyond the existing label."""
    from .replay_node import window_frames
    mode = args.localization
    if mode == "identity":
        return {}
    alignment = alignment_from_args(args)
    section = {"localization": LABELS[mode], "localization_mode": mode, "map_alignment": alignment,
               "carrot_recording": str(Path(output) / MAP_RECORDING),
               "base_reference": "FROZEN_PLANAR_AXLE_FROM_LEVER_ARM_UNVALIDATED_PLACED_IN_MAP_BY_ALIGNMENT"}
    if mode == "amcl":
        from ..live.amcl_cross import CANDIDATES
        name = args.amcl_params or "framework"
        offset = [float(v) for v in (args.initial_pose_offset or (0.0, 0.0, 0.0))]
        std_xy, std_yaw_deg = args.initial_pose_std or (0.25, math.degrees(0.15))
        first = recording["base_poses"][window_frames(recording, window)[0]]
        start = compose_poses(_xyyaw(alignment), first)[0]
        c, s = math.cos(start[2]), math.sin(start[2])
        pose = [float(start[0] + c * offset[0] - s * offset[1]), float(start[1] + s * offset[0] + c * offset[1]),
                float(math.atan2(math.sin(start[2] + math.radians(offset[2])),
                                 math.cos(start[2] + math.radians(offset[2]))))]
        translation, yaw = args.gate_correction_limits or (GATE_TRANSLATION_M, GATE_YAW_DEG)
        cov_xy, cov_yaw_deg = args.gate_covariance_limits or (GATE_COV_XY_M, math.degrees(GATE_COV_YAW_RAD))
        section["amcl"] = {"params": name, "changes_from_framework": dict(CANDIDATES.get(name, {})),
                           "initial_pose_map": pose, "aligned_start_map": [float(v) for v in start],
                           "initial_pose_offset_forward_left_yawdeg": offset,
                           "initial_pose_std": [float(std_xy), math.radians(float(std_yaw_deg))]}
        section["gate_limits"] = {"correction_window_s": GATE_WINDOW_S, "correction_hold_s": GATE_HOLD_S,
                                  "correction_translation_m": float(translation), "correction_yaw_deg": float(yaw),
                                  "correction_reference": args.gate_correction_reference or GATE_REFERENCE,
                                  "covariance_xy_std_m": float(cov_xy),
                                  "covariance_yaw_std_rad": math.radians(float(cov_yaw_deg)),
                                  "defaults": bool(args.gate_correction_limits is None
                                                   and args.gate_covariance_limits is None
                                                   and args.gate_correction_reference is None)}
    return section


def map_frame_poses(config, poses):
    """Recorded axle poses in the map: unchanged for identity, the alignment applied otherwise."""
    if not active(config):
        return np.asarray(poses, dtype=float)
    return compose_poses(_xyyaw(config["map_alignment"]), poses)


def map_frame_recording(config, recording):
    """The recording with axle (and lidar) poses in the map; the odometry-frame axle poses are kept."""
    if not active(config):
        return recording
    result = dict(recording)
    result["base_poses_odom"] = recording["base_poses"]
    result["base_poses"] = map_frame_poses(config, recording["base_poses"])
    result["lidar_poses"] = map_frame_poses(config, recording["lidar_poses"])
    return result


def write_map_recording(config, recording_map):
    if active(config):
        with Path(config["carrot_recording"]).open("xb") as stream:
            np.savez_compressed(stream, **recording_map)


def gate_config(profile, config):
    """Gate configuration: the earlier shadow gate unless AMCL runs; then covariance is also required."""
    gate = {"max_linear": profile.max_linear, "max_angular": profile.max_angular, "require_covariance": False}
    if config.get("localization_mode") != "amcl":
        return {"gate": gate}
    limits = config["gate_limits"]
    gate.update(require_covariance=True, covariance_xy_std_max_m=limits["covariance_xy_std_m"],
                covariance_yaw_std_max_rad=limits["covariance_yaw_std_rad"])
    monitor = {"window_s": limits["correction_window_s"], "max_translation_m": limits["correction_translation_m"],
               "max_yaw_rad": math.radians(limits["correction_yaw_deg"]), "hold_s": limits["correction_hold_s"],
               "reference": limits.get("correction_reference", GATE_REFERENCE)}
    return {"gate": gate, "correction_monitor": monitor}


def amcl_parameters(profile, config):
    """nav2_config localization parameters with the named candidate's changes (if any)."""
    from . import nav2_config
    from ..amcl_replay import amcl_parameter_overrides
    params = nav2_config.localization_parameters(profile, config["map_yaml"])
    changes = amcl_parameter_overrides(config.get("amcl", {}).get("changes_from_framework"))
    params["amcl"]["ros__parameters"].update(changes)
    return params


def write_files(supervisor, profile, config):
    """Localization parameter file (map_server, AMCL if used, lifecycle manager) and the helper config."""
    mode = config["localization_mode"]
    params = amcl_parameters(profile, config)
    nodes = ["map_server", "amcl"] if mode == "amcl" else ["map_server"]
    manager = dict(params["lifecycle_manager_localization"]["ros__parameters"], node_names=nodes)
    content = {"map_server": params["map_server"],
               "lifecycle_manager_localization": {"ros__parameters": manager}}
    if mode == "amcl":
        content["amcl"] = params["amcl"]
    with np.load(config["recording"]) as data:
        lever = [float(v) for v in data["lever_xy"]]
    helper = {"mode": mode, "lever_xy": lever, "map_odom_xyyaw": list(_xyyaw(config["map_alignment"])),
              "samples_path": str(supervisor.output / SAMPLES_FILE)}
    return {"mapping": supervisor.write("localization_params.yaml", content),
            "localization_helper": supervisor.write("localization_helper.json", helper)}


def start(supervisor, recorder, files, config):
    """Replay without its static transforms, the helper, map_server (+ AMCL and its start pose), map and TF."""
    from . import nav2_config
    from .bringup import EXECUTABLES, RunError
    mode = config["localization_mode"]
    if mode == "amcl" and not EXECUTABLES["amcl"].is_file():
        raise RunError("installed executable missing: " + str(EXECUTABLES["amcl"]))
    supervisor.launch("replay", [sys.executable, "-B", "-s", "-m", "wc_navigation.autonomy.replay_node",
                                 "--config", files["replay"], "--ros-args", "-r", "/tf_static:=" + REPLAY_TF_STATIC])
    supervisor.wait_for(recorder, lambda: recorder.clock_messages > 10 and recorder.truth is not None
                        and (recorder.replay_status or {}).get("state") == "HOLD", 30, "replay_clock")
    supervisor.launch("localization_helper", [sys.executable, "-B", "-s", "-m",
                                              "wc_navigation.autonomy.shadow_localization_node",
                                              "--config", files["localization_helper"]])
    supervisor.launch_ros("map_server", "map_server", files["mapping"])
    if mode == "amcl":
        supervisor.launch_ros("amcl", "amcl", files["mapping"])
    supervisor.launch_ros("lifecycle_manager_localization", "lifecycle_manager", files["mapping"],
                          ["-r", "__node:=lifecycle_manager_localization"])
    supervisor.wait_for(recorder, lambda: recorder.trigger("/lifecycle_manager_localization/is_active") is True,
                        60, "localization_active")
    check_single_map_to_odom(supervisor, recorder, config)
    if mode == "amcl":
        settings = config["amcl"]
        covariance = nav2_config.initial_pose_covariance(*settings["initial_pose_std"])
        published = {"count": 0, "last": -1e9}

        def localized():
            # Re-send until AMCL publishes map->odom; every /initialpose re-initializes the filter.
            if recorder.estimate_world() is not None:
                return True
            import time
            if time.monotonic() - published["last"] > 1.0:
                recorder.publish_initial_pose(settings["initial_pose_map"], covariance)
                published["count"] += 1
                published["last"] = time.monotonic()
            return False

        supervisor.wait_for(recorder, localized, 30, "amcl_tf")
        supervisor.report["initial_pose"] = {"pose_map": settings["initial_pose_map"], "covariance": covariance,
                                             "messages_sent": published["count"]}
    supervisor.wait_for(recorder, lambda: recorder.map_messages > 0 and recorder.estimate_world() is not None,
                        30, "map_and_tf")


def check_single_map_to_odom(supervisor, recorder, config, settle_s=1.0):
    """Refuse to run if map->odom has a publisher other than this mode's (e.g. the replay's identity).

    Before the start pose AMCL has not published yet, so in amcl mode no map->odom may exist; in fixed mode
    it must be the alignment. A replay that ignored the /tf_static remap would otherwise publish a second,
    identity map->odom and the run would silently mix the two.
    """
    from .bringup import RunError
    from .ros_util import lookup_pose
    recorder.spin(settle_s)
    found = lookup_pose(recorder.buffer, "map", "odom")
    if config["localization_mode"] == "amcl":
        if found is not None:
            raise RunError("map->odom exists before AMCL was given a start pose: another publisher (the replay's "
                           "identity?) is active; refusing to mix it with AMCL")
    else:
        expected = _xyyaw(config["map_alignment"])
        if found is None or math.hypot(found[0][0] - expected[0], found[0][1] - expected[1]) > 1e-6 or \
                abs(math.atan2(math.sin(found[0][2] - expected[2]), math.cos(found[0][2] - expected[2]))) > 1e-6:
            raise RunError("map->odom is not the fixed alignment (%s); another publisher is active" % (found,))
    supervisor.report["map_to_odom_publisher_check"] = "PASS"


# -- after the run --------------------------------------------------------------------------------
def summarize_safely(config, recorded, output, replay, recording):
    """``summarize`` that never prevents result.json from being written."""
    try:
        return summarize(config, recorded, output, replay, recording)
    except Exception as exc:  # the shadow result must still be written
        return {"mode": config.get("localization_mode", "identity"), "status": "SUMMARY_ERROR",
                "error": type(exc).__name__ + ": " + str(exc)}


def read_samples(path):
    """map->odom samples (sim time, x, y, yaw) and AMCL poses (sim time, stamp, x, y, yaw, covariance)."""
    map_odom, poses = [], []
    try:
        with open(path, newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                values = {k: float(v) for k, v in row.items() if k != "kind"}
                if row["kind"] == "map_odom":
                    map_odom.append((values["sim_time_s"], values["x"], values["y"], values["yaw"]))
                elif row["kind"] == "amcl_pose":
                    poses.append(values)
    except OSError:
        return None, None
    return np.asarray(map_odom, dtype=float).reshape(-1, 4), poses


def gate_intervals(events, end_time):
    """[(start, end, state, reasons)] from the recorded gate state changes."""
    changes = [e for e in events if e.get("kind") == "gate_state" and e.get("time_s") is not None]
    result = []
    for k, event in enumerate(changes):
        stop = changes[k + 1]["time_s"] if k + 1 < len(changes) else end_time
        value = event.get("value") or {}
        result.append((float(event["time_s"]), float(stop), value.get("state"), tuple(value.get("reasons") or ())))
    return result


def untrusted_time(intervals, start, end):
    """Seconds of LOCALIZATION_UNTRUSTED inside [start, end], by reason."""
    totals = {}
    for a, b, state, reasons in intervals:
        overlap = min(b, end) - max(a, start)
        if state == "LOCALIZATION_UNTRUSTED" and overlap > 0:
            key = reasons[0] if reasons else "UNSPECIFIED"
            totals[key] = round(totals.get(key, 0.0) + overlap, 3)
    return totals


def replay_gate_offline(times, map_odom, cov_times, cov_values, translation_m, yaw_deg, cov_xy_m,
                        cov_yaw_rad=GATE_COV_YAW_RAD, robot_xy=None):
    """Untrusted flags of the gate's correction and covariance conditions on the recorded samples.

    With ``robot_xy`` (the robot's odom position at each sample) the correction is measured at the robot.
    """
    from .safety_gate import CorrectionMonitor
    monitor = CorrectionMonitor(GATE_WINDOW_S, translation_m, math.radians(yaw_deg), GATE_HOLD_S,
                                reference="odom_origin" if robot_xy is None else "robot")
    if robot_xy is None:
        correction = np.array([not monitor.update(t, row)[0] for t, row in zip(times, map_odom)], dtype=bool)
    else:
        correction = np.array([not monitor.update(t, row, p)[0] for t, row, p in zip(times, map_odom, robot_xy)],
                              dtype=bool)
    covariance = np.zeros(len(times), dtype=bool)
    if len(cov_times):
        k = np.searchsorted(cov_times, times, side="right") - 1
        valid = k >= 0
        xy = np.sqrt(np.maximum(cov_values[np.clip(k, 0, None), 0], cov_values[np.clip(k, 0, None), 1]))
        yaw = np.sqrt(cov_values[np.clip(k, 0, None), 2])
        covariance = valid & ((xy > cov_xy_m) | (yaw > cov_yaw_rad))
    return correction, covariance, monitor.events


def support_untrusted(times, support, limit, hold_s=WHAT_IF_SUPPORT_HOLD_S):
    """Flag frames once the map support has stayed below ``limit`` for longer than ``hold_s``."""
    flags = np.zeros(len(times), dtype=bool)
    below_since = None
    for k, (t, value) in enumerate(zip(times, support)):
        low = not (math.isfinite(value) and value >= limit)
        below_since = (below_since if below_since is not None else t) if low else None
        flags[k] = low and t - below_since >= hold_s
    return flags


def segment_fractions(flags, labels):
    """Share of flagged samples in each door/mapped segment."""
    return {name: round(float(np.mean(flags[labels == name])), 4) for name in ("START_DOOR", "MIDDLE", "END_DOOR")
            if np.any(labels == name)}


def summarize(config, recorded, output, replay, recording):
    """Localization section of result.json; identity and fixed runs only report their contract."""
    mode = config.get("localization_mode", "identity")
    if mode != "amcl":
        section = {"mode": mode, "label": config.get("localization", LABELS["identity"])}
        if mode == "fixed":
            section["map_alignment"] = config["map_alignment"]
        return section
    from .maps import read_back
    from .replay_node import window_frames
    from .shadow import interpolate_poses, segment_twist
    from ..live.amcl_cross import (CAP_M, EndpointScorer, door_segments, heading_components, stats,
                                   window_medians)
    from .replay_mapping import ANGLES, RANGE_MAX, RANGE_MIN
    from .scan_match import scan_points
    map_odom, poses = read_samples(Path(output) / SAMPLES_FILE)
    start = replay.get("playback_start_s")
    section = {"mode": "amcl", "label": config["localization"], "map_alignment": config["map_alignment"],
               "amcl": config["amcl"], "gate_limits": config["gate_limits"], "samples_file": str(Path(output) / SAMPLES_FILE)}
    if map_odom is None or not len(map_odom) or start is None:
        section["status"] = "NO_LOCALIZATION_SAMPLES_OR_PLAYBACK"
        return section
    w0 = float(replay["window_s"][0])
    index = window_frames(recording, config["window_s"])
    times = recording["times_s"][index]
    odom = recording["base_poses_odom"][index]
    fixed = recording["base_poses"][index]
    lever = recording["lever_xy"]
    alignment = _xyyaw(config["map_alignment"])
    playing = (map_odom[:, 0] >= float(start)) & (map_odom[:, 0] <= float(start) + times[-1] - w0)
    samples = map_odom[playing]
    section["map_odom_samples_during_playback"] = int(len(samples))
    if len(samples) < 2:
        section["status"] = "TOO_FEW_SAMPLES"
        return section
    r = w0 + samples[:, 0] - float(start)
    _, data, meta = read_back(config["map_yaml"])
    scorer = EndpointScorer(data, meta["resolution"], meta["origin"])
    labels = door_segments(scorer.cell_values(fixed[:, :2]))
    label_at = labels[np.clip(np.searchsorted(times, r, side="right") - 1, 0, len(times) - 1)]
    # AMCL axle pose at each sample and its offset from the alignment, in the heading frame.
    b = interpolate_poses(times, odom, r)
    amcl = np.array([compose_poses(tuple(m[1:]), b[k:k + 1])[0] for k, m in enumerate(samples)])
    fixed_at = compose_poses(alignment, b)
    along, across = np.array([heading_components(a[0] - f[0], a[1] - f[1], a[2])
                              for a, f in zip(amcl, fixed_at)]).T
    dyaw = np.degrees(np.abs(np.arctan2(np.sin(amcl[:, 2] - fixed_at[:, 2]), np.cos(amcl[:, 2] - fixed_at[:, 2]))))
    # Endpoint fit of each recorded scan: AMCL map->odom at the frame (latest sample) vs. the alignment.
    sample_r = r
    fits_amcl, fits_fixed, support_amcl = [], [], []
    for k, t in enumerate(times):
        j = np.searchsorted(sample_r, t, side="right") - 1
        endpoints = scan_points(ANGLES, recording["ranges"][index[k]].astype(float), RANGE_MIN, RANGE_MAX, 400)
        lidar_odom = compose_poses(tuple(odom[k]), np.array([[lever[0], lever[1], 0.0]]))[0]
        for transform, sink in (((samples[j, 1], samples[j, 2], samples[j, 3]) if j >= 0 else None, fits_amcl),
                                (alignment, fits_fixed)):
            if transform is None:
                sink.append(math.nan)
                if sink is fits_amcl:
                    support_amcl.append(math.nan)
                continue
            lx, ly, lyaw = compose_poses(transform, lidar_odom[None, :])[0]
            c, s = math.cos(lyaw), math.sin(lyaw)
            pose = np.array([[c, -s, lx], [s, c, ly], [0.0, 0.0, 1.0]])
            fit, _, support = scorer.frame_fit_support(pose, endpoints)
            sink.append(fit)
            if sink is fits_amcl:
                support_amcl.append(support)
    rel = times - times[0]
    fits_amcl, fits_fixed, support_amcl = np.asarray(fits_amcl), np.asarray(fits_fixed), np.asarray(support_amcl)
    section["fit_m"] = {"amcl": window_medians(rel, fits_amcl), "fixed_alignment": window_medians(rel, fits_fixed),
                        "amcl_by_segment": {n: stats(fits_amcl[labels == n]) for n in ("START_DOOR", "MIDDLE", "END_DOOR")},
                        "fixed_by_segment": {n: stats(fits_fixed[labels == n]) for n in ("START_DOOR", "MIDDLE", "END_DOOR")},
                        "cap_m": CAP_M, "windows": "seconds since the window start"}
    section["map_support"] = {
        "by_segment": {n: stats(support_amcl[labels == n]) for n in ("START_DOOR", "MIDDLE", "END_DOOR")},
        "what_if": [{"min_support": limit, "hold_s": WHAT_IF_SUPPORT_HOLD_S,
                     "by_segment": segment_fractions(support_untrusted(rel, support_amcl, limit), labels)}
                    for limit in WHAT_IF_SUPPORT],
        "basis": "share of the recorded scan's endpoints within 0.15 m of an occupied map cell at AMCL's pose; "
                 "untrusted while below the limit for longer than hold_s (a candidate gate condition, not in the gate)"}
    section["amcl_vs_alignment"] = {
        n: {"abs_along_m": stats(np.abs(along[label_at == n])), "abs_across_m": stats(np.abs(across[label_at == n])),
            "abs_yaw_deg": stats(dyaw[label_at == n])} for n in ("START_DOOR", "MIDDLE", "END_DOOR")}
    # Gate conditions on the recorded AMCL output, as recorded and offline with other limits.
    pose_rows = sorted((p for p in poses if p["sim_time_s"] >= float(start) - 30.0), key=lambda p: p["sim_time_s"])
    cov_times = np.array([p["sim_time_s"] for p in pose_rows])
    cov_values = np.array([[p["cov_xx"], p["cov_yy"], p["cov_yawyaw"]] for p in pose_rows]).reshape(-1, 3)
    section["amcl_pose_updates_during_playback"] = int(np.sum(cov_times >= float(start)))
    limits = config["gate_limits"]
    end = float(start) + times[-1] - w0
    intervals = gate_intervals(recorded.get("events", []), end)
    v, w = segment_twist(times, odom)
    moving_r = times[:-1][(np.abs(v) > 0.05) | (np.abs(w) > 0.15)]
    section["gate_recorded"] = {
        "untrusted_s_by_reason": untrusted_time(intervals, float(start), end),
        "playback_s": round(end - float(start), 3),
        "driver_moving_s": round(float(len(moving_r)) * float(np.median(np.diff(times))), 1),
        "note": "LOCALIZATION_UNTRUSTED intervals of the running gate (first reason), playback only"}
    what_if = []
    middle = label_at == "MIDDLE"
    for reference, robot_xy in (("odom_origin", None), ("robot", b[:, :2])):
        for translation in WHAT_IF_TRANSLATION_M:
            for yaw in WHAT_IF_YAW_DEG:
                correction, _, events = replay_gate_offline(samples[:, 0], samples[:, 1:], cov_times, cov_values,
                                                            translation, yaw, limits["covariance_xy_std_m"],
                                                            limits["covariance_yaw_std_rad"], robot_xy=robot_xy)
                what_if.append({"reference": reference, "translation_m": translation, "yaw_deg": yaw,
                                "untrusted_fraction": round(float(correction.mean()), 4),
                                "by_segment": segment_fractions(correction, label_at), "events": events})
    covariance_what_if = []
    for limit in WHAT_IF_COV_XY_M:
        _, covariance, _ = replay_gate_offline(samples[:, 0], samples[:, 1:], cov_times, cov_values,
                                               1e9, 1e9, limit, limits["covariance_yaw_std_rad"])
        covariance_what_if.append({"xy_std_m": limit, "yaw_std_rad": limits["covariance_yaw_std_rad"],
                                   "untrusted_fraction": round(float(covariance.mean()), 4),
                                   "by_segment": segment_fractions(covariance, label_at)})
    change = []
    for k in range(len(samples)):
        j0 = np.searchsorted(samples[:, 0], samples[k, 0] - GATE_WINDOW_S, side="left")
        window = samples[j0:k + 1]
        here = compose_poses(tuple(samples[k, 1:]), b[k:k + 1])[0]
        earlier = np.array([compose_poses(tuple(row[1:]), b[k:k + 1])[0] for row in window])
        change.append((float(np.max(np.hypot(window[:, 1] - samples[k, 1], window[:, 2] - samples[k, 2]))),
                       float(np.max(np.hypot(earlier[:, 0] - here[0], earlier[:, 1] - here[1]))),
                       float(np.degrees(np.max(np.abs(np.arctan2(np.sin(window[:, 3] - samples[k, 3]),
                                                                 np.cos(window[:, 3] - samples[k, 3]))))))))
    change = np.asarray(change)
    std_xy = (np.sqrt(np.maximum(cov_values[:, 0], cov_values[:, 1])) if len(cov_values) else np.array([]))
    section["gate_offline"] = {
        "correction_what_if": what_if, "covariance_what_if": covariance_what_if,
        "map_odom_change_2s": {n: {"xy_m": stats(change[label_at == n, 0]),
                                   "robot_xy_m": stats(change[label_at == n, 1]),
                                   "yaw_deg": stats(change[label_at == n, 2])}
                               for n in ("START_DOOR", "MIDDLE", "END_DOOR")},
        "amcl_std_xy_m": stats(std_xy[cov_times >= float(start)]) if len(std_xy) else None,
        "amcl_std_yaw_deg": stats(np.degrees(np.sqrt(cov_values[cov_times >= float(start), 2]))) if len(cov_values) else None,
        "basis": "safety_gate.CorrectionMonitor (2 s window, 3 s hold) and the covariance check replayed on the "
                 "10 Hz map->odom samples and the AMCL poses of this run; fraction of samples flagged"}
    section["status"] = "COMPLETE"
    section["brief"] = {"fit_m": section["fit_m"]["amcl"], "fit_fixed_m": section["fit_m"]["fixed_alignment"],
                        "gate_untrusted_s": section["gate_recorded"]["untrusted_s_by_reason"]}
    return section
