"""Offline check of the live 3D obstacle input after a rosbag test (``navigation_live_shadow inputs --obstacle-*``).

Read-only, no ROS nodes; from the run directory (run_config.json, obstacles.jsonl, events.jsonl) and the bag:

1. Inputs: every processed frame whose stamp is in the frozen frontend cell of the same lidar (within 1 ms) must
   have the same reference cloud (sha256 of the float32 points). The cells hold about every third frame of each
   lidar, so about a third of the processed frames are checked; the feasibility check found them bit-identical
   (reports/navigation_obstacles/20261109T035705Z_live_obstacle_feasibility_ZL1107).
2. Outputs: the node's cycles are recomputed from the bag's frames with the node's logged pairs and poses
   (``live_obstacles.process_cycle``); points, clearing rays and the filtered scan must be identical. This checks
   the node's pairing, pose handling, persistence state and publication against the pure functions.
3. Timing: per lidar the processed rate, pose waits and timeouts, latency from arrival to publication, the CPU
   time of the host filter and of the obstacle filter.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from . import live_obstacles as lo
from .obstacle_cloud import ObstacleFilter

MATCH_NS = 1_000_000
STAT_KEYS = ("unsupported_clusters", "unsupported_confirmed_by_witness", "removed_contradicted", "removed_floating",
             "removed_transient", "voxels")


def read_log(path):
    path = Path(path)
    if not path.is_file():
        return []
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def entries_parts(entries):
    """(plan, timing config, ObstacleFilter, body, geometry) of a run's ``obstacles`` entry."""
    return (entries["plan"], lo.LiveObstacleConfig(**entries["config"]), ObstacleFilter(**entries["filter"]),
            [tuple(p) for p in entries["body"]], entries["geometry"])


def frozen_point_hashes(cell, stamps_ns, tolerance_ns=MATCH_NS):
    """{record stamp: sha256 of the frozen cell's points} for the stamps that have a cell frame within tolerance."""
    from .frontend_frames import read_cell
    stamps = np.asarray(sorted(int(s) for s in stamps_ns), dtype=np.int64)
    if not len(stamps):
        return {}
    out = {}
    for stamp, points, _ in read_cell(cell, start_ns=int(stamps[0]) - tolerance_ns, end_ns=int(stamps[-1]) + tolerance_ns):
        k = int(np.searchsorted(stamps, stamp))
        near = [j for j in (k - 1, k) if 0 <= j < len(stamps) and abs(int(stamps[j]) - stamp) <= tolerance_ns]
        if near:
            out[int(stamps[min(near, key=lambda j: abs(int(stamps[j]) - stamp))])] = lo.array_sha256(points)
    return out


def compare_inputs(records, frozen):
    """Per lidar: frames, frames checked against a frozen cell, identical, and the stamps that differ."""
    out = {}
    for side in sorted({r["side"] for r in records}):
        rows = [r for r in records if r["side"] == side and r.get("input_sha256")]
        cell = frozen.get(side)
        if cell is None:
            out[side] = {"frames": len(rows), "checked": 0, "identical": 0, "mismatched": [], "status": "NOT_CHECKED"}
            continue
        checked = [r for r in rows if r["stamp_ns"] in cell]
        bad = [r["stamp_ns"] for r in checked if cell[r["stamp_ns"]] != r["input_sha256"]]
        out[side] = {"frames": len(rows), "checked": len(checked), "identical": len(checked) - len(bad),
                     "mismatched": bad[:20], "status": "PASS" if checked and not bad else "FAIL"}
    return out


def released_from_records(rows):
    """The released cycle the node processed, rebuilt from its obstacles.jsonl lines (one per frame)."""
    first = rows[0]
    frames = {r["side"]: {"stamp_ns": r["stamp_ns"], "arrival_ns": r["arrival_ns"], "payload": None} for r in rows}
    return {"cycle": {"cycle": first["cycle"], "paired": first["paired"], "frames": frames},
            "poses": {r["side"]: (tuple(r["pose"]) if r["pose"] is not None else None) for r in rows},
            "measured_ns": {r["side"]: r["measured_ns"] for r in rows}, "pose_timeout": first["pose_timeout"],
            "waited_ns": first["waited_ns"]}


def recompute(records, clouds_of, entries):
    """Replay the node's cycles offline; ``clouds_of(side, stamp_ns)`` gives the reference points or None."""
    plan, _, cfg, body, geometry = entries_parts(entries)
    histories = lo.persistence_histories(plan["published"], cfg)
    cycles = {}
    for r in records:
        cycles.setdefault(r["cycle"], []).append(r)
    result = {"cycles": len(cycles), "frames": 0, "identical": 0, "mismatched": [], "input_mismatched": [],
              "missing_inputs": []}
    for number in sorted(cycles):
        rows = cycles[number]
        released = released_from_records(rows)
        clouds = {}
        for r in rows:
            if not r.get("input_sha256"):
                continue                                  # the node rejected this frame: no cloud
            cloud = clouds_of(r["side"], r["stamp_ns"])
            if cloud is None:
                result["missing_inputs"].append([r["side"], r["stamp_ns"]])
                continue
            if lo.array_sha256(cloud) != r["input_sha256"]:
                result["input_mismatched"].append([r["side"], r["stamp_ns"]])
            clouds[r["side"]] = cloud
        products = lo.process_cycle(released, clouds, geometry, plan, cfg, body, histories)
        for r in rows:
            if not r["published"]:
                continue
            result["frames"] += 1
            p = products.get(r["side"])
            same = p is not None and lo.array_sha256(p["points"]) == r["output_sha256"]
            if same and "rays_sha256" in r:
                same = p["rays"] is not None and lo.array_sha256(p["rays"]) == r["rays_sha256"]
            if same and "scan_sha256" in r:
                same = p["scan"] is not None and lo.array_sha256(p["scan"]) == r["scan_sha256"]
            if same:
                result["identical"] += 1
            elif len(result["mismatched"]) < 20:
                result["mismatched"].append([r["side"], r["stamp_ns"]])
    result["status"] = ("PASS" if result["frames"] and result["identical"] == result["frames"]
                        and not result["input_mismatched"] and not result["missing_inputs"] else "FAIL")
    return result


def _stats(values):
    v = np.asarray([x for x in values if x is not None], dtype=float)
    if not len(v):
        return None
    return {"median": round(float(np.median(v)), 1), "p90": round(float(np.percentile(v, 90)), 1),
            "p99": round(float(np.percentile(v, 99)), 1), "max": round(float(v.max()), 1)}


def summarize(records, entries):
    """Per lidar: rate, pairing, witness and persistence use, pose waits and timeouts, timing, artefact counts."""
    plan, cfg, _, _, _ = entries_parts(entries)
    out = {"rate_hz_configured": cfg.rate_hz, "cycles": len({r["cycle"] for r in records}), "sides": {}}
    for side in plan["read"]:
        rows = [r for r in records if r["side"] == side]
        if not rows:
            out["sides"][side] = {"frames": 0}
            continue
        stamps = np.asarray([r["stamp_ns"] for r in rows], dtype=np.int64)
        span_s = float(stamps.max() - stamps.min()) * 1e-9
        published = [r for r in rows if r["published"]]
        timing = [r.get("timing_ms") or {} for r in rows]
        entry = {"frames": len(rows), "span_s": round(span_s, 2),
                 "rate_hz": round((len(rows) - 1) / span_s, 2) if span_s > 0 else None,
                 "paired_share": round(float(np.mean([r["paired"] for r in rows])), 3),
                 "rejected": sum(1 for r in rows if r.get("rejected")),
                 "without_pose": sum(1 for r in rows if r["pose"] is None),
                 "published": len(published),
                 "pose_wait_ms": _stats(r["waited_ns"] * 1e-6 for r in rows),
                 "latency_ms": _stats(t.get("latency") for t in timing),
                 "host_filter_ms": _stats(t.get("host_filter") for t in timing),
                 "filter_ms": _stats(t.get("filter") for t in timing)}
        if published:
            entry["witness_share"] = round(float(np.mean([r["witness"] for r in published])), 3)
            entry["recent_share"] = round(float(np.mean([r["recent"] for r in published])), 3)
            entry["per_frame"] = {k: round(float(np.mean([(r.get("stats") or {}).get(k, 0) if k != "voxels"
                                                          else r["voxels"] for r in published])), 2)
                                  for k in STAT_KEYS}
        out["sides"][side] = entry
    out["pose_timeouts"] = len({r["cycle"] for r in records if r["pose_timeout"]})
    return out


def node_states(events):
    """States the node reported (recorder's events.jsonl) and its first failure."""
    values = [e["value"] for e in events if e.get("topic") == lo.STATUS_TOPIC]
    return sorted({v.get("state") for v in values if v.get("state")}), next(
        (v.get("failure") for v in values if v.get("failure")), None)


def evaluate(summary, inputs=None, outputs=None, states=(), failure=None):
    """PASS when the inputs were checked and identical, the recomputation identical, the rate and pose waits fine."""
    reasons = []
    if inputs is not None:
        left = inputs.get("left") or {}
        if left.get("status") != "PASS":
            reasons.append("left inputs not checked or not identical to the frozen cell")
        if (inputs.get("right") or {}).get("status") == "FAIL":
            reasons.append("right inputs differ from the frozen cell")
    if outputs is not None and outputs.get("status") != "PASS":
        reasons.append("recomputed outputs differ from the node's")
    rate = summary["rate_hz_configured"]
    for side, entry in summary["sides"].items():
        if not entry.get("frames"):
            reasons.append(side + " lidar: no frame processed")
        elif entry.get("span_s", 0) >= 5.0 and (entry.get("rate_hz") or 0) < 0.8 * rate:
            reasons.append("%s lidar processed at %.2f Hz, below 0.8 x %g Hz" % (side, entry["rate_hz"], rate))
    if summary["cycles"] and summary["pose_timeouts"] > 0.05 * summary["cycles"]:
        reasons.append("pose timeouts in %d of %d cycles" % (summary["pose_timeouts"], summary["cycles"]))
    if "FAILED" in states or failure:
        reasons.append("the node failed: " + str(failure))
    return {"status": "FAIL" if reasons else "PASS", "reasons": reasons}


def bag_clouds(bag_dir, entries, host_filter, wanted):
    """Reference points of the wanted frames {side: {stamp_ns}} from a rosbag2 sqlite bag (D's reference_points)."""
    import sqlite3
    from rclpy.serialization import deserialize_message
    from wc_interfaces.msg import SourceFrame
    from ..live.scan_adapter import ScanRejected, reference_points
    plan, _, _, _, geometry = entries_parts(entries)
    out = {side: {} for side in wanted}
    for db in sorted(Path(bag_dir).glob("*.db3")):
        con = sqlite3.connect("file:%s?mode=ro" % db, uri=True)
        try:
            ids = {name: tid for tid, name in con.execute("select id, name from topics")}
            for side, stamps in wanted.items():
                topic = plan["source_topics"][side]
                if topic not in ids or not stamps:
                    continue
                low, high = min(stamps) - 10 ** 9, max(stamps) + 10 ** 9
                for (data,) in con.execute("select data from messages where topic_id=? and timestamp between ? and ?",
                                           (ids[topic], low, high)):
                    message = deserialize_message(bytes(data), SourceFrame)
                    stamp = message.header.stamp.sec * 10 ** 9 + message.header.stamp.nanosec
                    if stamp not in stamps:
                        continue
                    g = geometry[side]
                    try:
                        out[side][stamp] = reference_points(message, host_filter, {"T_base_axle": g["T_base_axle"]},
                                                            g["R_reference_base"])[0]
                    except ScanRejected:
                        out[side][stamp] = None
        finally:
            con.close()
    return out


def check_run(output, config, left_cell=None, right_cell=None):
    """The inputs command's obstacle check (see the module docstring); ``config`` is the run's run_config."""
    output = Path(output)
    entries = config["obstacles"]
    records = read_log(output / lo.LOG_NAME)
    if not records:
        return {"status": "FAIL", "reasons": ["no obstacles.jsonl lines"]}
    live = json.loads(Path(config["live_config_file"]).read_text(encoding="utf-8"))
    frozen = {}
    for side, cell in (("left", left_cell), ("right", right_cell)):
        if cell is not None and side in entries["plan"]["read"]:
            frozen[side] = frozen_point_hashes(cell, [r["stamp_ns"] for r in records if r["side"] == side])
    inputs = compare_inputs(records, frozen)
    wanted = {}
    for r in records:
        if r.get("input_sha256"):
            wanted.setdefault(r["side"], set()).add(r["stamp_ns"])
    clouds = bag_clouds(config["bag"], entries, live["filter"], wanted)
    outputs = recompute(records, lambda side, stamp: clouds.get(side, {}).get(stamp), entries)
    summary = summarize(records, entries)
    events = read_log(output / "events.jsonl")
    states, failure = node_states(events)
    verdict = evaluate(summary, inputs, outputs, states, failure)
    return {**verdict, "inputs": inputs, "outputs": outputs, "summary": summary, "node_states": states,
            "frozen_cells": {"left": None if left_cell is None else str(left_cell),
                             "right": None if right_cell is None else str(right_cell)}}


def summarize_run(output, config):
    """The shadow runs' obstacle section of result.json (no recomputation)."""
    output = Path(output)
    records = read_log(output / lo.LOG_NAME)
    states, failure = node_states(read_log(output / "events.jsonl"))
    if not records:
        return {"status": "NO_RECORDS", "node_states": states, "failure": failure}
    summary = summarize(records, config["obstacles"])
    verdict = evaluate(summary, states=states, failure=failure)
    return {**verdict, "summary": summary, "node_states": states}
