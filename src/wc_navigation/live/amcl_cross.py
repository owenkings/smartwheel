"""Cross-recording AMCL check: one recording localized on another recording's saved map.

The validation recording's frozen scans and wheel/IMU odometry go through
map_server + AMCL (``amcl_replay``, isolated localhost domain 95) on the map
built from the mapping recording. The start pose is an explicit rigid alignment
of the two odometry frames, plus deliberate perturbations of it.

There is no ground truth. Three read-only comparisons are reported:

* map fit of the scan endpoints (per-frame mean distance to the nearest occupied
  cell, capped at 0.5 m; the measure of the alignment check) for AMCL and for
  the same alignment held fixed (pure odometry, no relocalization);
* perturbed starts against the unperturbed AMCL run (convergence and failure);
* AMCL covariance, correction along and across the heading where the scan fits
  the map equally well along the heading, and the safety gate's covariance and
  correction-jump conditions evaluated on the returned ``map -> odom``.

Frames whose axle lies outside the mapped free space (the door at the start and
end of the validation recording) are counted separately. Recordings and maps
are never modified; no device, vehicle or control topic is used.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import uuid

import numpy as np

CAP_M = 0.5
SUPPORT_M = 0.15            # localization_coverage support band
SCAN_HEIGHT_M = (-0.5, 0.5)
WINDOWS = (("first_40s", -math.inf, 40.0), ("40_80s", 40.0, 80.0), ("after_80s", 80.0, math.inf))
SEGMENTS = ("START_DOOR", "MIDDLE", "END_DOOR")
JOIN_XY_M, JOIN_YAW_DEG, JOIN_SHARE = 0.25, 5.0, 0.9
CONVERGED_XY_M, CONVERGED_YAW_DEG = 0.10, 2.0
DIVERGED_XY_M, DIVERGED_YAW_DEG = 0.50, 10.0
SCOPE = "CROSS_RECORDING_SAVED_MAP_ALIGNMENT_PRIOR_NOT_ABSOLUTE_TRUTH"
# Body-frame start perturbations (forward m, left m, yaw deg) around the alignment start.
DEFAULT_PERTURBATIONS = ([(0.0, 0.0, 0.0)]
                         + [(d, 0.0, 0.0) for d in (-1.0, -0.5, -0.25, 0.25, 0.5, 1.0)]
                         + [(0.0, d, 0.0) for d in (-1.0, -0.5, -0.25, 0.25, 0.5, 1.0)]
                         + [(0.0, 0.0, a) for a in (-20.0, -10.0, -5.0, 5.0, 10.0, 20.0)]
                         + [(0.5, 0.5, 10.0), (-0.5, -0.5, -10.0)])
# Operator-style start uncertainty used by the navigation simulation (cli --initial-pose-std).
DEFAULT_INITIAL_STD = (0.25, 0.25, 0.15)


# -- planar helpers -------------------------------------------------------------------------------
def se2(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, float(x)], [s, c, float(y)], [0.0, 0.0, 1.0]])


def xyyaw(matrix):
    return float(matrix[0, 2]), float(matrix[1, 2]), math.atan2(matrix[1, 0], matrix[0, 0])


def planar_from_rigid(transform):
    """SE(2) part of a frozen planar 4x4 pose (z and roll/pitch are zero in the planar frontend)."""
    t = np.asarray(transform, dtype=float)
    return se2(t[0, 3], t[1, 3], math.atan2(t[1, 0], t[0, 0]))


def rigid_from_planar(matrix):
    result = np.eye(4)
    result[:2, :2] = matrix[:2, :2]
    result[:2, 3] = matrix[:2, 2]
    return result


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def heading_components(dx, dy, yaw):
    """Split a planar offset into along-heading and across-heading parts."""
    c, s = math.cos(yaw), math.sin(yaw)
    return dx * c + dy * s, -dx * s + dy * c


def heading_std(covariance, yaw):
    """Standard deviations along and across ``yaw`` from a row-major 6x6 pose covariance."""
    xx, xy, yy = float(covariance[0]), float(covariance[1]), float(covariance[7])
    c, s = math.cos(yaw), math.sin(yaw)
    along = c * c * xx + 2 * c * s * xy + s * s * yy
    across = s * s * xx - 2 * c * s * xy + c * c * yy
    return math.sqrt(max(along, 0.0)), math.sqrt(max(across, 0.0))


def perturbed_start(start_axle, perturbation):
    """Move the axle start by (forward, left) in its own frame, then turn it about the axle."""
    forward, left, yaw_deg = (float(v) for v in perturbation)
    return start_axle @ se2(forward, left, math.radians(yaw_deg))


# -- map fit ---------------------------------------------------------------------------------------
class EndpointScorer:
    """Distance of endpoints to the nearest occupied cell, capped; outside the map counts as the cap."""

    def __init__(self, data, resolution, origin):
        from scipy import ndimage
        data = np.asarray(data)
        if data.ndim != 2 or not np.isin(data, (-1, 0, 100)).all() or not (data == 100).any():
            raise ValueError("trinary grid with occupied cells required")
        if abs(float(origin[2])) > 1e-9:
            raise ValueError("axis-aligned map origin required")
        self.data = data
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.distance = np.minimum(ndimage.distance_transform_edt(data != 100) * self.resolution, CAP_M)

    def _cells(self, xy):
        col = np.floor((xy[..., 0] - self.origin[0]) / self.resolution).astype(np.int64)
        row = np.floor((xy[..., 1] - self.origin[1]) / self.resolution).astype(np.int64)
        inside = (row >= 0) & (row < self.data.shape[0]) & (col >= 0) & (col < self.data.shape[1])
        return row, col, inside

    def lookup(self, xy):
        row, col, inside = self._cells(xy)
        values = np.full(xy.shape[:-1], CAP_M)
        values[inside] = self.distance[row[inside], col[inside]]
        return values

    def cell_values(self, xy):
        """-2 outside the map, otherwise the trinary value at each point."""
        row, col, inside = self._cells(xy)
        values = np.full(xy.shape[:-1], -2, dtype=np.int16)
        values[inside] = self.data[row[inside], col[inside]]
        return values

    def frame_fit(self, lidar_pose, points):
        """(mean capped distance, fraction of endpoints on known cells) of one scan at ``lidar_pose``."""
        fit, known, _ = self.frame_fit_support(lidar_pose, points)
        return fit, known

    def frame_fit_support(self, lidar_pose, points, support_m=SUPPORT_M):
        """Mean capped distance, known-cell fraction and fraction within ``support_m`` of an occupied cell."""
        if not len(points):
            return math.nan, math.nan, math.nan
        world = points @ lidar_pose[:2, :2].T + lidar_pose[:2, 2]
        known = np.isin(self.cell_values(world), (0, 100))
        distance = self.lookup(world)
        return float(distance.mean()), float(known.mean()), float(np.mean(distance <= support_m))


def scan_endpoints(points):
    """The same 161-bin projection and endpoint list as the mapping replay and alignment check."""
    from ..amcl_replay import build_scan
    from ..autonomy.replay_mapping import ANGLES, RANGE_MAX, RANGE_MIN
    from ..autonomy.scan_match import scan_points
    ranges, _ = build_scan(points, *SCAN_HEIGHT_M)
    return ranges, scan_points(ANGLES, ranges.astype(float), RANGE_MIN, RANGE_MAX, 400)


def door_segments(values):
    """START_DOOR / MIDDLE / END_DOOR from the trinary value under the axle (free = mapped)."""
    values = np.asarray(values)
    labels = np.full(len(values), "MIDDLE", dtype=object)
    mapped = np.nonzero(values == 0)[0]
    if not len(mapped):
        labels[:] = "START_DOOR"
        return labels
    labels[:mapped[0]] = "START_DOOR"
    labels[mapped[-1] + 1:] = "END_DOOR"
    return labels


def stats(values):
    values = np.asarray([v for v in values if v is not None and math.isfinite(v)], dtype=float)
    if not len(values):
        return None
    return {"n": int(len(values)), "median": round(float(np.median(values)), 4),
            "p90": round(float(np.quantile(values, 0.9)), 4), "max": round(float(values.max()), 4),
            "mean": round(float(values.mean()), 4)}


def window_medians(times, values, mask=None):
    times, values = np.asarray(times, dtype=float), np.asarray(values, dtype=float)
    mask = np.ones(len(times), dtype=bool) if mask is None else np.asarray(mask, dtype=bool)
    result = {}
    for name, start, end in WINDOWS:
        selected = mask & (times > start) & (times <= end) & np.isfinite(values)
        if name == "first_40s":
            selected = mask & (times <= end) & np.isfinite(values)
        result[name] = round(float(np.median(values[selected])), 4) if selected.any() else None
    return result


# -- one AMCL run against the fixed alignment --------------------------------------------------------
def map_odom_series(result, odom_axle):
    """map->odom per frame stamp: AMCL's broadcast (motion-gated runs) or pose * odom^-1 (forced runs)."""
    if result.get("map_odom_trajectory") is not None:
        return {row["stamp_ns"]: se2(row["x_m"], row["y_m"], row["yaw_rad"])
                for row in result["map_odom_trajectory"]}
    series = {}
    for row in result.get("trajectory", []):
        if row["stamp_ns"] in odom_axle:
            series[row["stamp_ns"]] = se2(row["x_m"], row["y_m"], row["yaw_rad"]) @ np.linalg.inv(
                odom_axle[row["stamp_ns"]])
    return series


def evaluate_run(result, frames, alignment, scorer, gate_config=None, analyzer=None):
    """Per-frame table of one AMCL run; frames carry stamp, t, odom lidar/axle poses and endpoints."""
    from ..autonomy.safety_gate import CorrectionMonitor, GateConfig, SafetyGate
    gate = SafetyGate(gate_config or GateConfig(require_covariance=True))
    monitor = CorrectionMonitor()
    robot_monitor = CorrectionMonitor(reference="robot")
    poses = {row["stamp_ns"]: row for row in result.get("trajectory", [])}
    series = map_odom_series(result, {f["stamp_ns"]: f["axle"] for f in frames})
    rows, history = [], []
    last_map_odom = last_covariance = None
    for frame in frames:
        stamp, t = frame["stamp_ns"], frame["t"]
        received = series.get(stamp)
        map_odom = received if received is not None else last_map_odom
        pose_row = poses.get(stamp)
        if pose_row is not None:
            last_covariance = pose_row["covariance"]
        row = {"stamp_ns": stamp, "t": t, "acknowledged": received is not None,
               "filter_updated": pose_row is not None, "segment": frame["segment"]}
        if map_odom is None:
            rows.append(row)
            continue
        last_map_odom = map_odom
        axle = map_odom @ frame["axle"]
        lidar = map_odom @ frame["lidar"]
        fit, known, support = scorer.frame_fit_support(lidar, frame["endpoints"])
        x, y, yaw = xyyaw(axle)
        bx, by, byaw = xyyaw(alignment @ frame["axle"])
        along, across = heading_components(x - bx, y - by, yaw)
        mx, my, myaw = xyyaw(map_odom)
        history.append((t, map_odom))
        while history and t - history[0][0] > 2.0:
            history.pop(0)
        # Largest change within 2 s: of map->odom itself (as the gate measures it) and of the robot's
        # current map position under the earlier corrections (what the correction does to the robot).
        robot = frame["axle"][:, 2]
        jump_xy = max(math.hypot(mx - h[0, 2], my - h[1, 2]) for _, h in history)
        jump_robot = max(float(np.hypot(*((map_odom - h) @ robot)[:2])) for _, h in history)
        jump_yaw = max(abs(wrap(myaw - math.atan2(h[1, 0], h[0, 0]))) for _, h in history)
        correction_ok, _ = monitor.update(t, (mx, my, myaw))
        robot_ok, _ = robot_monitor.update(t, (mx, my, myaw), robot[:2])
        row.update(x=x, y=y, yaw=yaw, map_odom=[mx, my, myaw], fit=fit, known=known, support=support,
                   along_vs_fixed=along, across_vs_fixed=across, yaw_vs_fixed=wrap(yaw - byaw),
                   change_2s_xy=jump_xy, change_2s_robot_xy=jump_robot, change_2s_yaw=jump_yaw,
                   correction_ok=correction_ok, correction_robot_ok=robot_ok)
        if last_covariance is not None:
            std_along, std_across = heading_std(last_covariance, yaw)
            covariance_ok, reason = gate.check_covariance(last_covariance)
            row.update(std_x=math.sqrt(max(last_covariance[0], 0.0)), std_y=math.sqrt(max(last_covariance[7], 0.0)),
                       std_yaw=math.sqrt(max(last_covariance[35], 0.0)), std_along=std_along,
                       std_across=std_across, covariance_ok=covariance_ok, covariance_reason=reason)
        if analyzer is not None:
            report = analyzer.analyze(frame["ranges"], frame["angles"], list(xyyaw(lidar)), heading_yaw_rad=yaw)
            row.update(coverage_insufficient=report["coverage_insufficient"],
                       along_heading_ambiguity=report["along_heading_ambiguity_evidence"])
        rows.append(row)
    return rows, {"correction_jump_events": monitor.events, "correction_jump_events_robot": robot_monitor.events}


def summarize_run(rows, frames):
    times = np.array([f["t"] for f in frames])
    fit = np.array([r.get("fit", math.nan) for r in rows], dtype=float)
    labels = np.array([f["segment"] for f in frames], dtype=object)
    summary = {"frames": len(rows), "acknowledged": int(sum(r["acknowledged"] for r in rows)),
               "filter_updates": int(sum(r["filter_updated"] for r in rows)),
               "fit_windows_all_frames": window_medians(times, fit),
               "fit_windows_middle": window_medians(times, fit, labels == "MIDDLE"),
               "fit_by_segment": {name: stats(fit[labels == name]) for name in SEGMENTS},
               "fit_under_0_10_m_middle": (round(float(np.mean(fit[(labels == "MIDDLE") & np.isfinite(fit)] < 0.10)), 3)
                                           if np.isfinite(fit[labels == "MIDDLE"]).any() else None)}
    for key in ("std_x", "std_y", "std_yaw", "std_along", "std_across", "along_vs_fixed", "across_vs_fixed",
                "yaw_vs_fixed", "change_2s_xy", "change_2s_robot_xy", "change_2s_yaw", "known", "support"):
        values = np.array([r.get(key, math.nan) for r in rows], dtype=float)
        if key.startswith("std_yaw") or key.endswith("_yaw") or key == "yaw_vs_fixed":
            values = np.degrees(np.abs(values))
        elif key in ("along_vs_fixed", "across_vs_fixed"):
            values = np.abs(values)
        suffix = "_deg" if "yaw" in key else ("_fraction" if key in ("known", "support") else "_m")
        summary[key + suffix] = {name: stats(values[labels == name]) for name in SEGMENTS}
    flags = [r for r in rows if "covariance_ok" in r]
    summary["gate_covariance_failures"] = {name: int(sum(1 for r in flags if not r["covariance_ok"] and r["segment"] == name))
                                           for name in SEGMENTS}
    summary["gate_covariance_reasons"] = sorted({r["covariance_reason"] for r in flags if not r["covariance_ok"]})
    summary["gate_correction_untrusted_frames"] = {
        name: int(sum(1 for r in rows if r.get("correction_ok") is False and r["segment"] == name)) for name in SEGMENTS}
    summary["gate_correction_robot_reference_untrusted_frames"] = {
        name: int(sum(1 for r in rows if r.get("correction_robot_ok") is False and r["segment"] == name))
        for name in SEGMENTS}
    ambiguous = [r for r in rows if r.get("along_heading_ambiguity") and r["segment"] == "MIDDLE"]
    if any("along_heading_ambiguity" in r for r in rows):
        clear = [r for r in rows if r.get("along_heading_ambiguity") is False and r["segment"] == "MIDDLE"
                 and not r.get("coverage_insufficient")]
        summary["along_heading_ambiguity"] = {
            "middle_frames_ambiguous": len(ambiguous),
            "middle_frames_insufficient_coverage": int(sum(1 for r in rows if r.get("coverage_insufficient")
                                                           and r["segment"] == "MIDDLE")),
            "middle_frames_clear": len(clear),
            "std_along_m": {"ambiguous": stats([r.get("std_along") for r in ambiguous]),
                            "clear": stats([r.get("std_along") for r in clear])},
            "std_across_m": {"ambiguous": stats([r.get("std_across") for r in ambiguous]),
                             "clear": stats([r.get("std_across") for r in clear])},
            "abs_along_vs_fixed_m": {"ambiguous": stats([abs(r["along_vs_fixed"]) for r in ambiguous]),
                                     "clear": stats([abs(r["along_vs_fixed"]) for r in clear])},
            "abs_across_vs_fixed_m": {"ambiguous": stats([abs(r["across_vs_fixed"]) for r in ambiguous]),
                                      "clear": stats([abs(r["across_vs_fixed"]) for r in clear])},
            "basis": "localization_coverage.CoverageAnalyzer at the AMCL pose: similar endpoint fit at +-0.5/1/2 m "
                     "along the heading; diagnostic, not proof of uniqueness"}
    return summary


def convergence(case_rows, reference_rows):
    """A perturbed start against the unperturbed run, over the mapped (MIDDLE) frames.

    Joined: from that frame on, at least 90 % of the remaining MIDDLE frames lie within 0.25 m and 5 degrees of
    the unperturbed run (AMCL's own frame-to-frame scatter is about 0.1 m, so a tighter band measures noise).
    CONVERGED: joined, and the median distance after joining is at most 0.10 m and 2 degrees. DIVERGED: the last
    MIDDLE frame is more than 0.5 m or 10 degrees away. Otherwise NOT_CONVERGED.
    """
    middle = [k for k, r in enumerate(reference_rows) if r["segment"] == "MIDDLE"]
    distance, yaw = [], []
    for k in range(len(reference_rows)):
        a, b = case_rows[k], reference_rows[k]
        if "x" not in a or "x" not in b:
            distance.append(math.inf)
            yaw.append(math.inf)
            continue
        distance.append(math.hypot(a["x"] - b["x"], a["y"] - b["y"]))
        yaw.append(abs(math.degrees(wrap(a["yaw"] - b["yaw"]))))
    if not middle:
        return {"status": "NO_MAPPED_FRAMES"}
    inside = np.array([distance[k] <= JOIN_XY_M and yaw[k] <= JOIN_YAW_DEG for k in middle], dtype=float)
    share_after = np.cumsum(inside[::-1])[::-1] / np.arange(len(inside), 0, -1)
    # A join can only follow the last gross error (beyond the divergence band).
    gross = [i for i, k in enumerate(middle) if not (distance[k] <= DIVERGED_XY_M and yaw[k] <= DIVERGED_YAW_DEG)]
    first = gross[-1] + 1 if gross else 0
    joined_index = next((i for i in range(first, len(middle)) if share_after[i] >= JOIN_SHARE), None)
    last = middle[-1]
    final = {"xy_m": round(distance[last], 4) if math.isfinite(distance[last]) else None,
             "yaw_deg": round(yaw[last], 3) if math.isfinite(yaw[last]) else None}
    after = {}
    if joined_index is not None:
        later = [middle[i] for i in range(joined_index, len(middle))]
        d = np.array([distance[k] for k in later])
        a = np.array([yaw[k] for k in later])
        after = {"median_xy_m": round(float(np.median(d)), 4), "p90_xy_m": round(float(np.quantile(d, 0.9)), 4),
                 "median_yaw_deg": round(float(np.median(a)), 3)}
    if math.isfinite(distance[last]) and (distance[last] > DIVERGED_XY_M or yaw[last] > DIVERGED_YAW_DEG):
        status = "DIVERGED"
    elif not math.isfinite(distance[last]):
        status = "DIVERGED"
    elif after and after["median_xy_m"] <= CONVERGED_XY_M and after["median_yaw_deg"] <= CONVERGED_YAW_DEG:
        status = "CONVERGED"
    else:
        status = "NOT_CONVERGED"
    travelled = None
    if joined_index is not None:
        path = [(r["x"], r["y"]) for r in reference_rows[:middle[joined_index] + 1] if "x" in r]
        travelled = round(float(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))), 2)
    return {"status": status,
            "joined_at_s": None if joined_index is None else round(reference_rows[middle[joined_index]]["t"], 2),
            "travelled_before_join_m": travelled, "after_join": after, "final_middle": final,
            "thresholds": {"join_xy_m": JOIN_XY_M, "join_yaw_deg": JOIN_YAW_DEG, "join_share": JOIN_SHARE,
                           "converged_median_xy_m": CONVERGED_XY_M, "converged_median_yaw_deg": CONVERGED_YAW_DEG,
                           "diverged_xy_m": DIVERGED_XY_M, "diverged_yaw_deg": DIVERGED_YAW_DEG}}


# -- inputs -------------------------------------------------------------------------------------------
def load_alignment(args):
    if args.alignment_json:
        path = args.alignment_json.expanduser().resolve(strict=True)
        raw = path.read_bytes()
        value = json.loads(raw)
        best = value["best_alignment"]
        return ((float(best["dx_m"]), float(best["dy_m"]), float(best["yaw_deg"])),
                {"source": str(path), "sha256": hashlib.sha256(raw).hexdigest(),
                 "map": value.get("map"), "validation_cell": value.get("validation_cell"),
                 "reference_fit": value.get("per_frame_mean_endpoint_distance_m"),
                 "second_best_score_ratio": value.get("second_best_score_ratio")})
    return tuple(args.alignment), {"source": "COMMAND_LINE"}


ALPHAS_0_05 = {"alpha1": 0.05, "alpha2": 0.05, "alpha3": 0.05, "alpha4": 0.05, "alpha5": 0.05}
# Candidate changes on top of the framework parameters, each named by what it changes; not defaults.
CANDIDATES = {
    "framework_alpha_0_05": dict(ALPHAS_0_05),
    "framework_sharp_laser": {"sigma_hit": 0.10, "z_hit": 0.90, "z_rand": 0.10, "max_beams": 120},
    "candidate": {**ALPHAS_0_05, "sigma_hit": 0.10, "z_hit": 0.90, "z_rand": 0.10, "max_beams": 120},
}
PROFILES = ("framework", *CANDIDATES, "replay_tool")


def profile_settings(name):
    """AMCL as configured for navigation/shadow runs, a named candidate change, or the earlier replay checks."""
    from ..amcl_replay import AMCL_OVERRIDE_TYPES
    if name == "framework" or name in CANDIDATES:
        from ..autonomy import nav2_config
        params = nav2_config.localization_parameters(nav2_config.NavProfile(), "unused.yaml")["amcl"]["ros__parameters"]
        overrides = {k: v for k, v in params.items() if k in AMCL_OVERRIDE_TYPES}
        changes = CANDIDATES.get(name, {})
        return {"laser_model": params["laser_model_type"], "amcl_overrides": {**overrides, **changes},
                "update_policy": "AMCL_MOTION_GATED_UPDATES", "changes_from_framework": dict(changes),
                "source": "nav2_config.localization_parameters (simulation navigate, shadow and live AMCL)"
                          + ("" if name == "framework" else " with the listed candidate changes")}
    if name == "replay_tool":
        return {"laser_model": "likelihood_field", "amcl_overrides": {},
                "update_policy": "REQUEST_NOMOTION_UPDATE_BEFORE_EACH_ORIGINAL_SCAN",
                "source": "amcl_replay defaults (earlier same-recording functionality checks)"}
    raise ValueError("unknown AMCL profile: " + str(name))


def _finite(value):
    number = float(value)
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError("finite number required")
    return number


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, required=True)
    parser.add_argument("--source-cell", type=Path, required=True,
                        help="frozen single-left frontend of the recording to localize")
    parser.add_argument("--map", dest="map_yaml", type=Path, required=True,
                        help="saved trinary map of the other (mapping) recording")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--alignment", type=_finite, nargs=3, metavar=("X_M", "Y_M", "YAW_DEG"),
                       help="rigid pose of the recording's odometry frame in the map")
    group.add_argument("--alignment-json", type=Path, help="alignment check result with best_alignment")
    parser.add_argument("--profile", action="append", choices=PROFILES,
                        help="AMCL parameter set (repeatable; default framework and replay_tool)")
    parser.add_argument("--perturbation", action="append", type=_finite, nargs=3,
                        metavar=("FORWARD_M", "LEFT_M", "YAW_DEG"),
                        help="start perturbation in the start axle frame (repeatable; default grid)")
    parser.add_argument("--perturb-profile", action="append", choices=PROFILES,
                        help="profiles that get the perturbation grid (default framework)")
    parser.add_argument("--initial-std", type=_finite, nargs=2, default=[DEFAULT_INITIAL_STD[0],
                                                                       math.degrees(DEFAULT_INITIAL_STD[2])],
                        metavar=("XY_M", "YAW_DEG"))
    parser.add_argument("--timeout", type=_finite, default=600.0, help="wall seconds per AMCL run")
    args = parser.parse_args(argv)
    args.profile = args.profile or ["framework", "replay_tool"]
    args.perturb_profile = args.perturb_profile or ["framework"]
    args.perturbation = [tuple(p) for p in (args.perturbation or DEFAULT_PERTURBATIONS)]
    if (0.0, 0.0, 0.0) not in args.perturbation:
        args.perturbation.insert(0, (0.0, 0.0, 0.0))
    if any(math.hypot(p[0], p[1]) > 2.0 or abs(p[2]) > 45.0 for p in args.perturbation):
        parser.error("perturbations are limited to 2 m and 45 degrees")
    if not 0 < args.initial_std[0] <= 2.0 or not 0 < args.initial_std[1] <= 90.0:
        parser.error("--initial-std needs 0 < XY <= 2 m and 0 < YAW <= 90 degrees")
    if not 10 <= args.timeout <= 900:
        parser.error("--timeout must be within 10..900 s")
    return args


def load_frames(source):
    from ..rebuild import validate_frontend_row
    from ..replay_cli import checked_frames, select_rows
    source = source.expanduser().resolve(strict=True)
    runtime_raw = (source / "runtime_config.json").read_bytes()
    index_raw = (source / "frontend/index.jsonl").read_bytes()
    runtime = json.loads(runtime_raw)
    if runtime.get("mode") != "left" or runtime.get("offline_source_selection", {}).get("selected_sides") != ["left"]:
        raise ValueError("exact frozen single-left source is required")
    if runtime.get("motion_model") != "planar_ekf":
        raise ValueError("frozen planar FLU reference is required for the axle lever arm")
    rows = [json.loads(line) for line in index_raw.decode("utf-8").splitlines()]
    stamps = [validate_frontend_row(row) for row in rows]
    if len(stamps) < 5 or any(a >= b for a, b in zip(stamps, stamps[1:])):
        raise ValueError("at least five unique increasing source timestamps required")
    hashes = {str(source / "runtime_config.json"): hashlib.sha256(runtime_raw).hexdigest(),
              str(source / "frontend/index.jsonl"): hashlib.sha256(index_raw).hexdigest()}
    frames, _ = checked_frames(source, select_rows(rows, stamps[0], (0.0, 1e7), 0.0), runtime, hashes)
    lever = np.asarray(runtime["mounts"]["left"]["t_axle_lidar_m"][:2], dtype=float)
    if lever.shape != (2,) or not np.isfinite(lever).all() or np.hypot(*lever) > 2.0:
        raise ValueError("finite frozen planar lever arm required")
    return source, frames, lever, hashes


def _sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _write_json(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def _clean(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: _clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean(v) for v in value]
    if isinstance(value, np.generic):
        return _clean(value.item())
    return value


def case_name(perturbation):
    forward, left, yaw = perturbation
    return "f%+.2f_l%+.2f_y%+.1f" % (forward, left, yaw)


CSV_FIELDS = ("stamp_ns", "t", "segment", "acknowledged", "filter_updated", "x", "y", "yaw", "fit", "known",
              "support", "std_x", "std_y", "std_yaw", "std_along", "std_across", "along_vs_fixed",
              "across_vs_fixed", "yaw_vs_fixed", "change_2s_xy", "change_2s_robot_xy", "change_2s_yaw",
              "correction_ok", "correction_robot_ok", "covariance_ok", "coverage_insufficient",
              "along_heading_ambiguity")


def write_rows(path, rows, frames, scorer, alignment):
    with Path(path).open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([*CSV_FIELDS, "fixed_x", "fixed_y", "fixed_yaw", "fixed_fit"])
        for row, frame in zip(rows, frames):
            fixed = alignment @ frame["axle"]
            fx, fy, fyaw = xyyaw(fixed)
            values = []
            for key in CSV_FIELDS:
                value = row.get(key, "")
                values.append(round(value, 5) if isinstance(value, float) and math.isfinite(value) else
                              ("" if isinstance(value, float) else value))
            writer.writerow([*values, round(fx, 4), round(fy, 4), round(fyaw, 5), round(frame["fixed_fit"], 4)])


def render(path, scorer, frames, alignment, references, cases):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    fixed = np.array([xyyaw(alignment @ f["axle"])[:2] for f in frames])
    times = np.array([f["t"] for f in frames])
    labels = np.array([f["segment"] for f in frames], dtype=object)
    fig, axes = plt.subplots(4, 1, figsize=(15, 18), gridspec_kw={"height_ratios": [2.2, 1, 1, 1]},
                             constrained_layout=True)
    ax = axes[0]
    data = scorer.data
    image = np.where(data == 100, 0.0, np.where(data == 0, 1.0, 0.75))
    r, (ox, oy) = scorer.resolution, scorer.origin
    ax.imshow(image, origin="lower", cmap="gray", vmin=0, vmax=1,
              extent=[ox, ox + data.shape[1] * r, oy, oy + data.shape[0] * r])
    ax.plot(fixed[:, 0], fixed[:, 1], "--", color="#2b7bb9", linewidth=1.2, label="odometry, one rigid alignment")
    colors = {"framework": "#a33da1", "framework_alpha_0_05": "#d35400", "framework_sharp_laser": "#2980b9",
              "candidate": "#c0392b", "replay_tool": "#1d8a4f"}
    for name, rows in references.items():
        xy = np.array([[row["x"], row["y"]] for row in rows if "x" in row])
        if len(xy):
            ax.plot(xy[:, 0], xy[:, 1], color=colors.get(name, "k"), linewidth=1.3, label="AMCL " + name)
    status_color = {"CONVERGED": "#1d8a4f", "NOT_CONVERGED": "#e08e0b", "DIVERGED": "#c0392b"}
    for case in cases:
        start = case.get("start_axle_xy_yaw")
        if start:
            ax.plot(start[0], start[1], "x", color=status_color.get(case["convergence"].get("status"), "#555"), ms=6)
    door = fixed[labels != "MIDDLE"]
    if len(door):
        ax.scatter(door[:, 0], door[:, 1], s=8, color="#e67e22", label="axle outside mapped free space (door)", zorder=5)
    pad = 2.0
    ax.set_xlim(fixed[:, 0].min() - pad, fixed[:, 0].max() + pad)
    ax.set_ylim(fixed[:, 1].min() - pad, fixed[:, 1].max() + pad)
    ax.set_aspect("equal")
    ax.legend(fontsize=8, loc="best")
    ax.set_title("Cross-recording AMCL on the mapping recording's map (no ground truth; x = perturbed starts, "
                 "green converged / orange close / red diverged)", fontsize=10)
    for segment, color in (("START_DOOR", "#e67e22"), ("END_DOOR", "#e67e22")):
        mask = labels == segment
        for axis in axes[1:]:
            if mask.any():
                axis.axvspan(times[mask].min(), times[mask].max(), color=color, alpha=0.12, linewidth=0)
    axes[1].plot(times, [f["fixed_fit"] for f in frames], color="#2b7bb9", linewidth=1.0, label="odometry, fixed alignment")
    for name, rows in references.items():
        axes[1].plot(times, [row.get("fit", np.nan) for row in rows], color=colors.get(name, "k"), linewidth=1.0,
                     label="AMCL " + name)
    axes[1].set_ylabel("endpoint-to-map mean (m)")
    axes[1].set_ylim(0, CAP_M + 0.02)
    first = next(iter(references.values()), [])
    for key, style in (("std_along", "-"), ("std_across", "--")):
        axes[2].plot(times, [row.get(key, np.nan) for row in first], style, color="#a33da1", linewidth=1.0,
                     label=key.replace("_", " ") + " (first profile)")
    ambiguous = np.array([bool(row.get("along_heading_ambiguity")) for row in first])
    if ambiguous.any():
        axes[2].scatter(times[ambiguous], np.full(ambiguous.sum(), 0.01), marker="|", color="#c0392b",
                        label="similar fit along heading")
    axes[2].set_ylabel("AMCL std (m)")
    for case in cases:
        rows = case.get("rows") or []
        axes[3].plot(times, [row.get("distance_to_reference", np.nan) for row in rows], linewidth=0.8,
                     color=status_color.get(case["convergence"].get("status"), "#555"))
    axes[3].axhline(CONVERGED_XY_M, color="#555", linewidth=0.8, linestyle=":")
    axes[3].set_yscale("log")
    axes[3].set_ylabel("distance to unperturbed AMCL (m)")
    axes[3].set_xlabel("time since the first frame (s)")
    for axis in axes[1:]:
        axis.grid(alpha=0.25)
        axis.legend(fontsize=8, loc="upper right")
    fig.savefig(path, dpi=100)
    plt.close(fig)


def main(argv=None):
    args = arguments(argv)
    from wc_runtime.storage_policy import StoragePolicy
    from ..amcl_replay import run_amcl_replay
    from ..autonomy.maps import read_back
    from ..geometry import Grid
    from ..localization_coverage import CoverageAnalyzer
    storage = StoragePolicy(args.project_root)
    storage.check()
    source, raw_frames, lever, hashes = load_frames(args.source_cell)
    map_yaml = args.map_yaml.expanduser().resolve(strict=True)
    _, data, meta = read_back(map_yaml)
    scorer = EndpointScorer(data, meta["resolution"], meta["origin"])
    analyzer = CoverageAnalyzer(Grid(data, meta["resolution"], tuple(meta["origin"])))
    map_hashes = {str(map_yaml): _sha(map_yaml), str(map_yaml.parent / meta["image"]): _sha(map_yaml.parent / meta["image"])}
    (ax_, ay_, ayaw_deg), alignment_info = load_alignment(args)
    alignment = se2(ax_, ay_, math.radians(ayaw_deg))
    lever_m = se2(lever[0], lever[1], 0.0)
    base_from_lidar = np.eye(4)
    base_from_lidar[:2, 3] = lever
    t0 = raw_frames[0]["stamp_ns"]
    frames = []
    angles = -math.pi / 3 + (math.pi / 240) * np.arange(161)
    for frame in raw_frames:
        lidar = planar_from_rigid(frame["odometry_T"])
        ranges, endpoints = scan_endpoints(frame["points"])
        frames.append({"stamp_ns": frame["stamp_ns"], "t": (frame["stamp_ns"] - t0) * 1e-9, "lidar": lidar,
                       "axle": lidar @ np.linalg.inv(lever_m), "endpoints": endpoints, "ranges": ranges,
                       "angles": angles})
    axle_xy = np.array([xyyaw(alignment @ f["axle"])[:2] for f in frames])
    labels = door_segments(scorer.cell_values(axle_xy))
    for frame, label in zip(frames, labels):
        frame["segment"] = label
        frame["fixed_fit"], frame["fixed_known"], frame["fixed_support"] = scorer.frame_fit_support(
            alignment @ frame["lidar"], frame["endpoints"])
    times = np.array([f["t"] for f in frames])
    fixed_fit = np.array([f["fixed_fit"] for f in frames])
    fixed_support = np.array([f["fixed_support"] for f in frames])
    segments = {}
    for name in SEGMENTS:
        mask = labels == name
        segments[name] = {"frames": int(mask.sum()),
                          "t_s": [round(float(times[mask].min()), 2), round(float(times[mask].max()), 2)] if mask.any() else None}
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + uuid.uuid4().hex[:8]
    output = storage.resolve("reports/navigation_amcl_cross/" + run_id)
    output.mkdir(parents=True, exist_ok=False)
    start_axle = alignment @ frames[0]["axle"]
    initial_std = (args.initial_std[0], args.initial_std[0], math.radians(args.initial_std[1]))
    _write_json(output / "input_contract.json", _clean({
        "source_cell": str(source), "source_sha256": {k: v for k, v in hashes.items() if k.endswith(("json", "jsonl"))},
        "source_files_checked": len(hashes), "map_yaml": str(map_yaml), "map_sha256": map_hashes,
        "alignment_x_y_yaw_deg": [ax_, ay_, ayaw_deg], "alignment": alignment_info,
        "lever_xy_m": lever.tolist(), "frames": len(frames), "profiles": args.profile,
        "perturbations_forward_left_yawdeg": args.perturbation, "perturbed_profiles": args.perturb_profile,
        "initial_std_xy_m_yaw_rad": list(initial_std), "evaluation_scope": SCOPE,
        "ros_domain_id": 95, "hardware_started": False, "control_topics_published": False}))
    profiles = {}
    references = {}
    cases = []
    for profile in args.profile:
        settings = profile_settings(profile)
        profile_dir = output / "cases" / profile
        profile_dir.mkdir(parents=True)
        perturbations = args.perturbation if profile in args.perturb_profile else [(0.0, 0.0, 0.0)]
        reference_rows = None
        profile_cases = []
        for perturbation in perturbations:
            name = case_name(perturbation)
            start = perturbed_start(start_axle, perturbation)
            initial_lidar = rigid_from_planar(start @ lever_m)
            print("AMCL_CROSS", profile, name, flush=True)
            result = run_amcl_replay(map_yaml, raw_frames, initial_lidar, profile_dir / name, args.timeout,
                                     scan_height_m=SCAN_HEIGHT_M, base_from_lidar=base_from_lidar,
                                     laser_model=settings["laser_model"], evaluation_scope=SCOPE,
                                     amcl_overrides=settings["amcl_overrides"], initial_std=initial_std,
                                     update_policy=settings["update_policy"])
            reference = perturbation == (0.0, 0.0, 0.0)
            rows, extra = evaluate_run(result, frames, alignment, scorer, analyzer=analyzer if reference else None)
            case = {"profile": profile, "name": name, "perturbation_forward_left_yawdeg": list(perturbation),
                    "start_axle_xy_yaw": list(xyyaw(start)), "amcl_status": result.get("status"),
                    "error": result.get("error"), "wall_s": round(result.get("wall_elapsed_s", 0.0), 1),
                    "unacknowledged_frames": len(result.get("unacknowledged_stamps_ns", [])),
                    **extra, "summary": summarize_run(rows, frames)}
            write_rows(profile_dir / (name + ".csv"), rows, frames, scorer, alignment)
            if reference:
                reference_rows = rows
                references[profile] = rows
                case["convergence"] = {"status": "REFERENCE"}
            else:
                case["convergence"] = (convergence(rows, reference_rows) if reference_rows is not None
                                       else {"status": "NO_REFERENCE"})
                for row, ref in zip(rows, reference_rows or []):
                    if "x" in row and "x" in ref:
                        row["distance_to_reference"] = math.hypot(row["x"] - ref["x"], row["y"] - ref["y"])
                case["rows"] = rows
                cases.append(case)
            profile_cases.append({k: v for k, v in case.items() if k != "rows"})
        profiles[profile] = {"settings": settings, "cases": profile_cases}
    perturbed = [c for c in cases if c["convergence"].get("status") not in ("REFERENCE", None)]
    counts = {}
    for case in perturbed:
        key = case["profile"] + ":" + case["convergence"]["status"]
        counts[key] = counts.get(key, 0) + 1
    summary = {
        "schema_version": 1, "status": "COMPLETE" if all(
            c["amcl_status"] == "AMCL_REPLAY_COMPLETE" for p in profiles.values() for c in p["cases"]) else "PARTIAL",
        "evaluation_scope": SCOPE, "output": str(output), "frames": len(frames),
        "recording_span_s": round(float(times[-1]), 2), "segments": segments,
        "segment_rule": "axle position (fixed alignment) on a free map cell = MIDDLE; before the first / after the "
                        "last such frame = START_DOOR / END_DOOR",
        "fixed_alignment": {"x_y_yaw_deg": [ax_, ay_, ayaw_deg], **alignment_info,
                            "fit_windows_all_frames": window_medians(times, fixed_fit),
                            "fit_windows_middle": window_medians(times, fixed_fit, labels == "MIDDLE"),
                            "fit_by_segment": {name: stats(fixed_fit[labels == name]) for name in SEGMENTS},
                            "support_by_segment": {name: stats(fixed_support[labels == name]) for name in SEGMENTS},
                            "fit_under_0_10_m_middle": round(float(np.mean(fixed_fit[labels == "MIDDLE"] < 0.10)), 3)},
        "initial_std_xy_m_yaw_deg": [initial_std[0], round(math.degrees(initial_std[2]), 3)],
        "profiles": profiles, "convergence_counts": counts,
        "metric": "per-frame mean distance of the 161-bin scan endpoints to the nearest occupied cell of the map, "
                  "capped at 0.5 m (outside the map = cap); same as the alignment check",
        "interpretation": "Map fit and agreement between runs, not accuracy against ground truth. The odometry "
                          "of both recordings comes from the same frontend; the alignment used only the first 40 s.",
        "hardware_started": False, "control_topics_published": False, "navigation_validated": False}
    try:
        render(output / "preview.png", scorer, frames, alignment, references, cases)
        summary["preview"] = str(output / "preview.png")
    except Exception as exc:  # the summary must still be written
        summary["preview_error"] = type(exc).__name__ + ": " + str(exc)
    unchanged = (all(_sha(path) == digest for path, digest in map_hashes.items())
                 and all(_sha(path) == digest for path, digest in hashes.items()))
    summary["sources_unchanged"] = unchanged
    storage.check()
    _write_json(output / "summary.json", _clean(summary))
    brief = {"status": summary["status"], "output": str(output), "segments": segments,
             "fixed_alignment_fit": summary["fixed_alignment"]["fit_windows_all_frames"],
             "amcl_fit": {p: v["cases"][0]["summary"]["fit_windows_all_frames"] for p, v in profiles.items()},
             "convergence_counts": counts, "sources_unchanged": unchanged}
    print(json.dumps(_clean(brief), ensure_ascii=True), flush=True)
    return 0 if summary["status"] == "COMPLETE" and unchanged else 1


if __name__ == "__main__":
    raise SystemExit(main())
