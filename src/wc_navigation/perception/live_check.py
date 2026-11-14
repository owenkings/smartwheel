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

Shadow runs (vehicle, bag, closed loop) are judged by FIELD_CRITERIA (``summarize_run``, ``main``), counted from the
node's first RUNNING to ``end_grace_s`` before its last processed frame (``judged_window``).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from . import live_obstacles as lo
from .obstacle_cloud import ObstacleFilter

MATCH_NS = 1_000_000
# What the input must reach in a shadow run before the closed loop uses it (relay 2026-11-12 item 1): both lidars
# all along, the configured rate, the other lidar as the witness, and fresh clouds for the collision monitor.
FIELD_CRITERIA = {
    "min_span_s": 60.0,             # at least a minute of processed frames
    "rate_share": 0.95,             # each lidar processed at >= 95 % of the configured rate (4.75 of 5 Hz)
    "paired_share": 0.95,           # cycles with both lidars' frames
    "witness_share": 0.95,          # published frames judged with the other lidar
    # A cycle without a pose goes out without the witness and the persistence check (one in 10 s at 2 %); on the
    # -x route of the next closed loop the raw ghosts never reached the stop zone or the slowdown box (11-12).
    "pose_timeout_share": 0.02,
    # measurement time (lidar stamp - time offset) -> publication. On the vehicle /scan reached its recorder 120 ms
    # (median) and 136-172 ms (p99) after the measurement time (11-09, 11-11); bag shadow runs through the relay
    # (11-11) reached 350 ms (p99) and 425 ms (max), mostly waiting for /odom; 500 ms is 5 cm at 0.10 m/s. The
    # collision monitor ignores a cloud older than its source_timeout of 1.0 s (Humble: the source then counts as
    # empty, the chair is not stopped).
    "age_p99_ms": 500.0,
    "age_max_ms": 800.0,
    # Every check counts the cycles from the node's first RUNNING (both lidars and /odom fresh; window D's gate holds
    # the chair until then) to end_grace_s before its last processed frame, where the sources stop one by one: in
    # the panel's recordings the wheel feedback starts up to 3.2 s after the lidars and ends up to 3.3 s before
    # them, and on the vehicle /odom ended 1.7-3.3 s before /scan (11-09, 11-11; status/C.md 2026-11-13, C4).
    "end_grace_s": 5.0,
}
# Status changes this close to the last processed frame belong to the end of the run (the sources stop first).
END_GRACE_S = FIELD_CRITERIA["end_grace_s"]
# Stage counts that exist only with a non-default filter option (witness_scope "all"), summarised when logged.
OPTIONAL_STAT_KEYS = ("clusters_judged_by_witness", "removed_seen_through")
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
            keys = STAT_KEYS + tuple(k for k in OPTIONAL_STAT_KEYS
                                     if any(k in (r.get("stats") or {}) for r in published))
            entry["per_frame"] = {k: round(float(np.mean([(r.get("stats") or {}).get(k, 0) if k != "voxels"
                                                          else r["voxels"] for r in published])), 2)
                                  for k in keys}
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


def first_running_s(events):
    """When the node first reported RUNNING (recorder's events.jsonl, the node's clock), or None."""
    times = [float(e["time_s"]) for e in events if e.get("topic") == lo.STATUS_TOPIC and "time_s" in e
             and isinstance(e.get("value"), dict) and e["value"].get("state") == "RUNNING"]
    return min(times) if times else None


def not_running_after_start(events, end_s=None, grace_s=END_GRACE_S):
    """Seconds the node reported anything but RUNNING after it first reported RUNNING, and what it named stale.

    From the recorder's events.jsonl, which logs the status when its state or stale inputs change. ``end_s`` is the
    last processed frame's arrival: a change less than ``grace_s`` before it is the end of the run (the sources
    stop before the node: the bag ends, Ctrl-C), and an interval still open at the end ends there.
    """
    changes = [(float(e["time_s"]), e["value"]) for e in events
               if e.get("topic") == lo.STATUS_TOPIC and isinstance(e.get("value"), dict) and "time_s" in e]
    changes.sort(key=lambda c: c[0])
    started = next((t for t, v in changes if v.get("state") == "RUNNING"), None)
    if started is None:
        return {"first_running_s": None, "seconds": None, "intervals": 0, "stale_inputs": []}
    seconds, intervals, names, since = 0.0, 0, set(), None
    for t, v in changes:
        if t < started:
            continue
        if v.get("state") != "RUNNING":
            if end_s is not None and t >= end_s - grace_s:
                continue                              # the end of the run
            names.update(v.get("stale_inputs") or [])
            if since is None:
                since, intervals = t, intervals + 1
        elif since is not None:
            seconds, since = seconds + (t if end_s is None else min(t, end_s)) - since, None
    if since is not None:
        seconds += max(0.0, (end_s if end_s is not None else since) - since)
    return {"first_running_s": started, "seconds": round(seconds, 2), "intervals": intervals,
            "stale_inputs": sorted(names)}


def judged_window(records, events, end_grace_s=END_GRACE_S):
    """The part of a run that FIELD_CRITERIA judge, on the node's clock (seconds), and the cycles in it.

    From the node's first RUNNING (from its first frame when it never reported one; that run fails not_running_s)
    to ``end_grace_s`` before its last processed frame. A cycle belongs to it by the arrival of its first frame.
    """
    if not records:
        return {"start_s": None, "end_s": None, "first_frame_s": None, "last_frame_s": None, "skipped_start_s": None,
                "skipped_end_s": None, "cycles": 0, "cycles_judged": 0, "from_running": False, "judged": set()}
    arrival = {}
    for r in records:
        arrival[r["cycle"]] = min(arrival.get(r["cycle"], r["arrival_ns"]), r["arrival_ns"])
    first_s, last_s = min(arrival.values()) * 1e-9, max(r["arrival_ns"] for r in records) * 1e-9
    running = first_running_s(events)
    start_s = running if running is not None else first_s
    end_s = last_s - float(end_grace_s)
    judged = {c for c, t in arrival.items() if start_s <= t * 1e-9 <= end_s}
    return {"start_s": round(start_s, 3), "end_s": round(end_s, 3), "first_frame_s": round(first_s, 3),
            "last_frame_s": round(last_s, 3), "skipped_start_s": round(max(0.0, start_s - first_s), 2),
            "skipped_end_s": round(float(end_grace_s), 2), "cycles": len(arrival), "cycles_judged": len(judged),
            "from_running": running is not None, "judged": judged}


def field_criteria(records, entries, events=(), limits=None):
    """The checks of FIELD_CRITERIA on a run's obstacles.jsonl and events.jsonl: {"status", "checks", "limits",
    "not_running", "window"}.

    Each check is {"name", "side", "value", "limit", "ok"}, the shares also with "count" and "total"; any failed
    check fails the run's obstacle section. All of them count only the cycles of ``judged_window``: before the
    node first reports RUNNING its inputs are still starting, and at the end the sources stop one by one.
    """
    limits = dict(FIELD_CRITERIA, **(limits or {}))
    plan, cfg, _, _, _ = entries_parts(entries)
    window = judged_window(records, events, limits["end_grace_s"])
    judged = window.pop("judged")
    kept = [r for r in records if r["cycle"] in judged]
    checks = []

    def check(name, side, value, limit, ok, count=None, total=None):
        entry = {"name": name, "side": side, "value": None if value is None else round(float(value), 3),
                 "limit": limit, "ok": bool(ok)}
        if total is not None:
            entry.update(count=int(count), total=int(total))
        checks.append(entry)

    for side in plan["read"]:
        rows = [r for r in kept if r["side"] == side]
        stamps = np.asarray([r["stamp_ns"] for r in rows], dtype=np.int64)
        span = float(stamps.max() - stamps.min()) * 1e-9 if len(rows) > 1 else 0.0
        rate = (len(rows) - 1) / span if span > 0 else 0.0
        check("span_s", side, span, limits["min_span_s"], span >= limits["min_span_s"])
        check("rate_hz", side, rate, round(limits["rate_share"] * cfg.rate_hz, 3), rate >= limits["rate_share"] * cfg.rate_hz)
        if not rows:
            continue
        paired = sum(bool(r["paired"]) for r in rows)
        check("paired_share", side, paired / len(rows), limits["paired_share"],
              paired / len(rows) >= limits["paired_share"], paired, len(rows))
        published = [r for r in rows if r["published"]]
        if side in plan["published"]:
            seen = sum(bool(r["witness"]) for r in published)
            share = seen / len(published) if published else 0.0
            if plan["witness"]:
                check("witness_share", side, share, limits["witness_share"], share >= limits["witness_share"],
                      seen, len(published))
            age = [(r["arrival_ns"] - r["measured_ns"]) * 1e-6 + r["timing_ms"]["latency"] for r in published
                   if (r.get("timing_ms") or {}).get("latency") is not None]
            a99 = float(np.percentile(age, 99)) if age else None
            check("age_p99_ms", side, a99, limits["age_p99_ms"], a99 is not None and a99 <= limits["age_p99_ms"])
            amax = float(np.max(age)) if age else None
            check("age_max_ms", side, amax, limits["age_max_ms"], amax is not None and amax <= limits["age_max_ms"])
    cycles = len(judged)
    timeouts = len({r["cycle"] for r in kept if r["pose_timeout"]})
    share = timeouts / cycles if cycles else 1.0
    check("pose_timeout_share", None, share, limits["pose_timeout_share"], share <= limits["pose_timeout_share"],
          timeouts, cycles)
    gaps = not_running_after_start(events, window["last_frame_s"], limits["end_grace_s"])
    check("not_running_s", None, gaps["seconds"], 0.0, gaps["seconds"] == 0.0)
    failed = [c for c in checks if not c["ok"]]
    return {"status": "FAIL" if failed else "PASS", "checks": checks, "limits": limits, "not_running": gaps,
            "window": window}


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
    """The shadow runs' obstacle section of result.json (no recomputation), with the field criteria."""
    output = Path(output)
    records = read_log(output / lo.LOG_NAME)
    events = read_log(output / "events.jsonl")
    states, failure = node_states(events)
    if not records:
        return {"status": "NO_RECORDS", "node_states": states, "failure": failure}
    summary = summarize(records, config["obstacles"])
    # the rate and pose-timeout rules of ``evaluate`` count the same cycles as the field criteria
    judged = judged_window(records, events, FIELD_CRITERIA["end_grace_s"])["judged"]
    steady = summarize([r for r in records if r["cycle"] in judged] or records, config["obstacles"])
    verdict = evaluate(steady, states=states, failure=failure)
    criteria = field_criteria(records, config["obstacles"], events)
    reasons = verdict["reasons"] + ["%s%s %s, limit %s%s" % (c["name"], " (%s)" % c["side"] if c["side"] else "",
                                                             c["value"], c["limit"],
                                                             " (%d of %d)" % (c["count"], c["total"])
                                                             if "total" in c else "")
                                    for c in criteria["checks"] if not c["ok"]]
    return {"status": "FAIL" if reasons else "PASS", "reasons": reasons, "summary": summary,
            "summary_judged": steady, "criteria": criteria, "node_states": states}


CHECK_TEXT = {"span_s": "处理时长 s", "rate_hz": "处理频率 Hz", "paired_share": "两台同周期的份额",
              "witness_share": "用上另一台见证的份额",
              "age_p99_ms": "测量→发布 p99 ms", "age_max_ms": "测量→发布 最大 ms",
              "pose_timeout_share": "等不到位姿的周期份额", "not_running_s": "开始以后不是 RUNNING 的时间 s"}
SIDE_TEXT = {"left": "左", "right": "右", None: ""}


def shortfall_text(check):
    """How far a failed check misses its limit, in the operator's words ("" when it passes or has no value)."""
    name, value, limit = check["name"], check["value"], check["limit"]
    if check["ok"] or value is None:
        return ""
    if "total" in check:
        if name == "pose_timeout_share":
            return "多 %d 个周期" % (check["count"] - int(math.floor(limit * check["total"] + 1e-9)))
        return "差 %d 帧" % (int(math.ceil(limit * check["total"] - 1e-9)) - check["count"])
    if name == "span_s":
        return "差 %.1f s" % (limit - value)
    if name == "rate_hz":
        return "差 %.2f Hz" % (limit - value)
    if name.endswith("_ms"):
        return "超 %.0f ms" % (value - limit)
    return ""


def window_text(window, grace_s):
    """The operator's line on which cycles the criteria counted."""
    if not window.get("cycles"):
        return "  没有处理过的周期"
    if not window.get("from_running"):
        return "  节点没报过 RUNNING：从第一帧算起，最后 %g s 不算（%d 个周期里算了 %d 个）" % (
            grace_s, window["cycles"], window["cycles_judged"])
    return "  只算节点第一次 RUNNING 以后、最后 %g s 以前的周期：%d 个周期里算了 %d 个（开头去掉 %.1f s）" % (
        grace_s, window["cycles"], window["cycles_judged"], window["skipped_start_s"])


def report_text(section):
    """Chinese lines for the operator: which cycles count, one line per field criterion, then the verdict."""
    if section.get("status") == "NO_RECORDS":
        return ["三维障碍输入：没有 obstacles.jsonl 记录（节点没处理任何帧）", "结论：不通过"]
    lines = ["三维障碍输入（%s）：" % ", ".join(section.get("node_states") or [])]
    criteria = section["criteria"]
    if criteria.get("window"):
        lines.append(window_text(criteria["window"], criteria["limits"].get("end_grace_s", END_GRACE_S)))
    for c in criteria["checks"]:
        limit = c["limit"]
        relation = "≤" if c["name"].endswith("_ms") or c["name"] in ("pose_timeout_share", "not_running_s") else "≥"
        counted = ""
        if "total" in c:
            counted = "%d / %d %s；" % (c["count"], c["total"], "个周期" if c["name"] == "pose_timeout_share" else "帧")
        short = shortfall_text(c)
        lines.append("  %s %s%s：%s（%s要 %s %s）%s%s" % (
            "通过" if c["ok"] else "不通过", SIDE_TEXT[c["side"]], CHECK_TEXT.get(c["name"], c["name"]),
            "无" if c["value"] is None else c["value"], counted, relation, limit,
            "  " + short if short else "", "" if c["ok"] else "  <==="))
    gaps = section["criteria"].get("not_running") or {}
    if gaps.get("stale_inputs"):
        lines.append("  停过的输入：" + ", ".join(gaps["stale_inputs"]))
    for side, e in (section.get("summary_judged") or section.get("summary") or {}).get("sides", {}).items():
        timing = {k: e.get(k) or {} for k in ("latency_ms", "pose_wait_ms", "filter_ms")}
        if timing["latency_ms"]:
            lines.append("  %s 参考：到达→发布 中位 %s / p99 %s ms（其中等位姿 中位 %s / p99 %s），过滤 中位 %s / p99 %s ms" % (
                SIDE_TEXT[side], timing["latency_ms"].get("median"), timing["latency_ms"].get("p99"),
                timing["pose_wait_ms"].get("median"), timing["pose_wait_ms"].get("p99"),
                timing["filter_ms"].get("median"), timing["filter_ms"].get("p99")))
    for reason in section.get("reasons", []):
        if not any(reason.startswith(c["name"]) for c in section["criteria"]["checks"]):
            lines.append("  另外：" + reason)
    lines.append("结论：" + ("通过" if section["status"] == "PASS" else "不通过"))
    return lines


def main(argv=None):
    """``python3 -m wc_navigation.perception.live_check RUN_DIR``: the obstacle section of a shadow run, judged."""
    import argparse
    parser = argparse.ArgumentParser(description="判定一次影子运行（实车、录包或闭环）的三维障碍输入，只读运行目录。")
    parser.add_argument("run_dir", type=Path, help="reports/navigation_live_shadow/<运行>")
    args = parser.parse_args(argv)
    config = json.loads((args.run_dir / "run_config.json").read_text(encoding="utf-8"))
    if not config.get("obstacles"):
        print("这次运行没有开三维障碍输入（run_config.json 里没有 obstacles）")
        return 2
    section = summarize_run(args.run_dir, config)
    print("\n".join(report_text(section)))
    return 0 if section["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
