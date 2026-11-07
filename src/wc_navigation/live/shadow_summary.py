"""Summary of a live shadow run from the recorder's files (read-only).

The chair's motion is the live odometry; every velocity-chain stage is compared
with it on a 0.1 s grid with the recorded-shadow outcome labels
(``autonomy.shadow.evaluate``). Gate stops are timed by reason from the gate's
state changes; AMCL is summarized from its poses and the sampled map->odom. For
a bag run with a reference frontend and alignment, AMCL's chair pose is also
compared with the aligned frozen path (agreement, not ground truth).
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import numpy as np


def _rows(path):
    try:
        with open(path, newline="", encoding="utf-8") as stream:
            return list(csv.DictReader(stream))
    except OSError:
        return []


def gate_untrusted(events, start, end):
    """Seconds per gate state and per localization reason within [start, end]."""
    changes = [e for e in events if e.get("topic") == "/wc_nav/gate_status"]
    states, reasons = {}, {}
    for k, event in enumerate(changes):
        a = float(event["time_s"])
        b = float(changes[k + 1]["time_s"]) if k + 1 < len(changes) else end
        overlap = min(b, end) - max(a, start)
        if overlap <= 0:
            continue
        value = event["value"]
        states[value.get("state")] = round(states.get(value.get("state"), 0.0) + overlap, 2)
        if value.get("state") == "LOCALIZATION_UNTRUSTED":
            reason = (value.get("reasons") or ["UNSPECIFIED"])[0]
            reasons[reason] = round(reasons.get(reason, 0.0) + overlap, 2)
    return states, reasons


def summarize_shadow(config, output):
    from ..autonomy.shadow import STAGES, evaluate
    output = Path(output)
    odom = np.array([[float(r["stamp_s"]), float(r["x"]), float(r["y"]), float(r["yaw"])]
                     for r in _rows(output / "odom.csv")], dtype=float).reshape(-1, 4)
    if len(odom) < 10:
        return {"summary_error": "too few odometry samples"}
    keep = np.concatenate([[True], np.diff(odom[:, 0]) > 0])
    odom = odom[keep]
    commands = {stage: [] for stage in STAGES}
    for row in _rows(output / "commands.csv"):
        commands[row["stage"]].append((float(row["time_s"]), float(row["linear"]), float(row["angular"])))
    samples = {stage: (np.array([r[0] for r in rows]), np.array([r[1:] for r in rows]).reshape(-1, 2))
               for stage, rows in commands.items()}
    window = (float(odom[0, 0]), float(odom[-1, 0]))
    _, summary = evaluate(odom[:, 0], odom[:, 1:4], window, samples)
    events = []
    try:
        events = [json.loads(line) for line in (output / "events.jsonl").read_text(encoding="utf-8").splitlines()
                  if line.strip()]
    except OSError:
        pass
    states, reasons = gate_untrusted(events, *window)
    inputs = [e["value"] for e in events if str(e.get("topic", "")).startswith("/wc_live/inputs_status")]
    localization = _rows(output / "localization.csv")
    poses = [r for r in localization if r["kind"] == "amcl_pose"]
    stds = [math.sqrt(max(float(r["cov_xx"]), float(r["cov_yy"]))) for r in poses]
    result = {
        "commands": summary, "outcomes_while_moving": summary["outcome_fractions_while_recording_moves"],
        "gate_state_s": states, "gate_untrusted_s": reasons, "window_s": list(window),
        "inputs_states": sorted({v.get("state") for v in inputs if v.get("state")}),
        "inputs_failure": next((v.get("failure") for v in inputs if v.get("failure")), None),
        "localization": {"amcl_poses": len(poses),
                         "map_odom_samples": sum(1 for r in localization if r["kind"] == "map_odom"),
                         "std_xy_m_median": float(np.median(stds)) if stds else None,
                         "std_xy_m_max": float(np.max(stds)) if stds else None},
        "command_messages": {stage: len(rows) for stage, rows in commands.items()},
        "goal_events": [e["value"] for e in events if e.get("topic") == "/wc_live/goal_status"][-5:],
    }
    reference = config.get("reference")
    if reference:
        result["localization"]["vs_aligned_reference"] = compare_with_reference(reference, odom, localization)
    return result


def compare_with_reference(reference, odom, localization):
    """AMCL chair pose (map->odom x live odometry) vs. the aligned frozen frontend path at its frame stamps."""
    from ..autonomy.shadow import interpolate_poses
    from ..autonomy.shadow_localization import compose_poses
    from .bag_check import frozen_frontend
    stamps, _, axle, _ = frozen_frontend(reference["cell"])
    best = json.loads(Path(reference["alignment_json"]).read_text(encoding="utf-8"))["best_alignment"]
    aligned = compose_poses((best["dx_m"], best["dy_m"], math.radians(best["yaw_deg"])), axle)
    samples = np.array([[float(r["stamp_s"]) - 0.5 if r["stamp_s"] else math.nan, float(r["x"]), float(r["y"]),
                         float(r["yaw"])] for r in localization if r["kind"] == "map_odom"], dtype=float).reshape(-1, 4)
    if len(samples) < 2:
        return {"status": "NO_MAP_TO_ODOM"}
    inside = (stamps >= max(odom[0, 0], samples[0, 0])) & (stamps <= min(odom[-1, 0], samples[-1, 0]))
    if inside.sum() < 2:
        return {"status": "NO_OVERLAP"}
    chair = interpolate_poses(odom[:, 0], odom[:, 1:4], stamps[inside])
    rows = []
    for t, c, a in zip(stamps[inside], chair, aligned[inside]):
        j = int(np.searchsorted(samples[:, 0], t, side="right")) - 1
        if j < 0:
            continue
        m = compose_poses(tuple(samples[j, 1:]), c[None, :])[0]
        rows.append((math.hypot(m[0] - a[0], m[1] - a[1]),
                     abs(math.degrees(math.atan2(math.sin(m[2] - a[2]), math.cos(m[2] - a[2]))))))
    values = np.asarray(rows)
    stats = lambda v: {"median": round(float(np.median(v)), 3), "p90": round(float(np.quantile(v, 0.9)), 3),
                       "max": round(float(v.max()), 3)}
    return {"status": "COMPARED", "frames": int(len(values)), "position_m": stats(values[:, 0]),
            "heading_deg": stats(values[:, 1]),
            "basis": "AMCL map->odom (stamp minus AMCL's 0.5 s tolerance) x live odometry vs. the frozen "
                     "frontend placed with the rigid alignment; agreement, not ground truth"}
