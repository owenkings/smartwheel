"""Live 3D obstacle input (window C): the shadow runs' obstacle filter on live lidar frames; pure computation.

The ROS node feeds this module with the two lidars' host-filtered SourceFrames and /odom; ``live_check``
recomputes its output offline from the bag. Per lidar cycle (both XT-M60 run at 10 Hz with a fixed phase of a few
milliseconds, status/C.md 11-09 12:10):

1. ``CycleSelector`` pairs the left and right frames of one cycle (stamps within ``pair_window_s``) and keeps one
   cycle per ``1 / rate_hz`` (default 5 Hz). A frame whose partner has not come within ``partner_wait_s`` is a
   cycle of its own (no witness). Frames of dropped cycles are never filtered.
2. The node turns each kept frame into the reference cloud (window D's ``scan_adapter``: production host filter,
   rotation into mapping_reference, float32), with each lidar's own mount and sensor id (``side_geometry``).
3. ``PoseGate`` holds the cycle until ``PoseHistory`` (base_link in odom from /odom) covers every frame's
   measurement time, the lidar stamp minus the lidar time offset, for at most ``pose_wait_s``. A frame still
   without a pose is filtered without the witness and the persistence check: nothing is deleted on a guessed
   pose (D, 11-09 15:45), and the timeout is counted.
4. ``process_cycle`` runs ``obstacle_cloud.filter_frame`` per published lidar as ``shadow_obstacles.prepare``
   does, with the other lidar's frame of the same cycle as the witness (``consensus.BoxWitness``, the same
   verdicts as ``Witness``) and the same lidar's persistence history.

Outputs keep the shadow runs' topics and messages (``nav2_obstacles``): obstacle points in
``obstacles_lidar_<side>`` (static base_link -> obstacles_lidar_<side> at the lever arm, axes of base_link),
clearing rays only when the local costmap has the VoxelLayer, the left lidar's filtered 161-bin scan only with
the filtered planning scan. Message stamps are the lidar stamps minus the lidar time offset, as for /scan
(``scan_adapter.scan_stamp_ns``); the offset is the one the live command resolved for /scan.
"""
from __future__ import annotations

import bisect
from collections import deque
from dataclasses import asdict, dataclass
import hashlib
import math
import time

import numpy as np

from ..live.config import LEFT_FILTERED_TOPIC, RIGHT_FILTERED_TOPIC
from ..live.scan_adapter import MAX_LIDAR_TIME_OFFSET_MS, scan_stamp_ns
from .consensus import BoxWitness, RecentFrames, move, planar_relative
from .nav2_obstacles import CLEAR_TOPICS, CLOUD_FRAMES, CLOUD_TOPICS, FILTERED_SCAN_TOPIC, SIDES, check
from .obstacle_cloud import ObstacleFilter, filter_frame

SCHEMA_VERSION = 1
STATUS_TOPIC = "/wc_live/inputs_status/obstacles"
LOG_NAME = "obstacles.jsonl"
# The SDK host-filtered frames of both lidars (the left one is the live chain's /scan source).
SOURCE_TOPICS = {"left": LEFT_FILTERED_TOPIC, "right": RIGHT_FILTERED_TOPIC}
NO_STAMP = -(1 << 62)


@dataclass(frozen=True)
class LiveObstacleConfig:
    """Timing of the live input (see the module docstring)."""
    rate_hz: float = 5.0
    pair_window_s: float = 0.03
    partner_wait_s: float = 0.05
    pose_wait_s: float = 0.3            # /odom was at most 0.27 s late in D's live shadow runs of 11-09
    odom_history_s: float = 3.0
    stale_after_s: float = 1.0

    def __post_init__(self):
        limits = {"rate_hz": (0.0, 10.0), "pair_window_s": (0.0, 0.05), "partner_wait_s": (0.0, 0.2),
                  "pose_wait_s": (0.0, 1.0), "odom_history_s": (0.0, 10.0), "stale_after_s": (0.0, 10.0)}
        for name, (low, high) in limits.items():
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not low < float(value) <= high:
                raise ValueError("%s must be within (%g, %g]" % (name, low, high))
            object.__setattr__(self, name, float(value))
        if self.odom_history_s < self.pose_wait_s + self.partner_wait_s + 0.5:
            raise ValueError("odom_history_s must cover the waits plus 0.5 s")

    def as_dict(self):
        return asdict(self)


def output_plan(obstacle_input="scan", obstacle_clouds=(), obstacle_scan="raw", obstacle_monitor="", witness=True):
    """Lidars read and products published, from the NavProfile obstacle fields (``nav2_obstacles.check``).

    ``published``: lidars whose obstacle points go out (the shadow runs' obstacle clouds); ``read``: those plus the
    other lidar when it is the witness. Raises ValueError for the default, where nothing uses an obstacle cloud.
    """
    obstacle_input, sides, obstacle_scan, monitor = check(obstacle_input, obstacle_clouds, obstacle_scan,
                                                          obstacle_monitor)
    if not sides:
        raise ValueError("no obstacle cloud is used: the live obstacle input stays off")
    clearing = obstacle_input != "scan"            # only the local costmap's VoxelLayer uses clearing rays
    read = tuple(SIDES) if witness else sides
    return {"obstacle_input": obstacle_input, "obstacle_scan": obstacle_scan, "obstacle_monitor": monitor,
            "published": list(sides), "read": list(read), "witness": bool(witness),
            "clearing": clearing, "filtered_scan": obstacle_scan == "filtered",
            "topics": {s: CLOUD_TOPICS[s] for s in sides},
            "clearing_topics": {s: CLEAR_TOPICS[s] for s in sides} if clearing else {},
            "filtered_scan_topic": FILTERED_SCAN_TOPIC if obstacle_scan == "filtered" else None,
            "frames": {s: CLOUD_FRAMES[s] for s in sides},
            "source_topics": {s: SOURCE_TOPICS[s] for s in read}}


def side_geometry(runtime, side):
    """Host-filter transform, rotation into mapping_reference, lever arm and sensor id of one lidar.

    From runtime_config.json of the cell given as --cell (frontend cell or frozen live configuration):
    ``mounts.<side>`` (R_axle_lidar, t_axle_lidar_m) and ``sensor_ids.<side>``. The live chain's T_base_axle,
    R_reference_base and identities.lidar_left (live.config) belong to the left lidar only.
    """
    if side not in SIDES:
        raise ValueError("side must be left or right")
    try:
        mount = runtime["mounts"][side]
        rotation = np.asarray(mount["R_axle_lidar"], dtype=float)
        lever = np.asarray(mount["t_axle_lidar_m"], dtype=float)
    except (KeyError, TypeError) as error:
        raise ValueError("runtime_config has no mount for the %s lidar" % side) from error
    if rotation.shape != (3, 3) or lever.shape != (3,) or not (np.isfinite(rotation).all() and np.isfinite(lever).all()):
        raise ValueError("the %s mount must be a finite 3x3 rotation and a lever arm" % side)
    if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-5) or abs(np.linalg.det(rotation) - 1.0) > 1e-5:
        raise ValueError("the %s mount rotation is not a rotation" % side)
    if np.linalg.norm(lever) > 3.0:
        raise ValueError("the %s lever arm must be within 3 m" % side)
    sensor = (runtime.get("sensor_ids") or {}).get(side)
    if not isinstance(sensor, str) or not sensor:
        raise ValueError("runtime_config has no sensor id for the %s lidar" % side)
    axle_lidar = np.eye(4)
    axle_lidar[:3, :3], axle_lidar[:3, 3] = rotation, lever
    base_axle = np.linalg.inv(axle_lidar)
    return {"side": side, "sensor_id": sensor, "source_frame_id": "lidar_" + side,
            "T_base_axle": base_axle.tolist(), "R_reference_base": base_axle[:3, :3].T.tolist(),
            "lever": lever.tolist()}


def check_left_geometry(left, live):
    """The left lidar's geometry from runtime_config must be the live chain's (``live.config.load_live_config``)."""
    same = (np.allclose(left["T_base_axle"], live["T_base_axle"], rtol=0.0, atol=1e-9)
            and np.allclose(left["R_reference_base"], live["R_reference_base"], rtol=0.0, atol=1e-9)
            and np.allclose(left["lever"][:2], live["lever_xy"], rtol=0.0, atol=1e-9)
            and left["sensor_id"] == live["identities"]["lidar_left"])
    if not same:
        raise ValueError("the left lidar's mount in runtime_config.json differs from the live chain's")


def published_stamp_ns(stamp_ns, lidar_time_offset_ms):
    """Stamp of the obstacle messages: the lidar stamp minus the offset, exactly as /scan (scan_stamp_ns)."""
    return scan_stamp_ns(stamp_ns, lidar_time_offset_ms)


def offset_ns(lidar_time_offset_ms):
    offset = float(lidar_time_offset_ms)
    if not math.isfinite(offset) or abs(offset) > MAX_LIDAR_TIME_OFFSET_MS:
        raise ValueError("lidar time offset must be finite and within +-%g ms" % MAX_LIDAR_TIME_OFFSET_MS)
    return int(round(offset * 1e6))


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class PoseHistory:
    """base_link poses in odom from /odom, interpolated linearly in x, y and unwrapped yaw.

    As lidar_time_offset.json's how_to_apply, except that the end poses are not held: a time after the newest
    sample is not covered yet (the cycle waits for it), a time before the oldest one has no pose.
    """

    def __init__(self, history_s=3.0):
        self.history_ns = int(round(float(history_s) * 1e9))
        self.stamps, self.poses = [], []
        self.counts = {"samples": 0, "out_of_order": 0, "rejected": 0, "resets": 0}

    def add(self, stamp_ns, x, y, yaw):
        stamp_ns = int(stamp_ns)
        if not all(math.isfinite(float(v)) for v in (x, y, yaw)):
            self.counts["rejected"] += 1
            return
        if self.stamps and stamp_ns <= self.stamps[-1]:
            if stamp_ns >= self.stamps[-1] - 1_000_000_000:
                self.counts["out_of_order"] += 1
                return
            self.stamps, self.poses = [], []          # the clock went back: a new bag or a restart
            self.counts["resets"] += 1
        if self.poses:
            previous = self.poses[-1][2]
            yaw = previous + _wrap(float(yaw) - previous)
        self.stamps.append(stamp_ns)
        self.poses.append((float(x), float(y), float(yaw)))
        self.counts["samples"] += 1
        cut = bisect.bisect_left(self.stamps, stamp_ns - self.history_ns)
        if cut:
            del self.stamps[:cut]
            del self.poses[:cut]

    @property
    def newest_ns(self):
        return self.stamps[-1] if self.stamps else None

    def covers(self, stamp_ns):
        return bool(self.stamps) and self.stamps[-1] >= int(stamp_ns)

    def at(self, stamp_ns):
        """(x, y, yaw) at ``stamp_ns``, or None outside the history."""
        stamp_ns = int(stamp_ns)
        if not self.stamps or stamp_ns < self.stamps[0] or stamp_ns > self.stamps[-1]:
            return None
        k = bisect.bisect_right(self.stamps, stamp_ns) - 1
        if k >= len(self.stamps) - 1:
            x, y, yaw = self.poses[-1]
            return (x, y, _wrap(yaw))
        a = (stamp_ns - self.stamps[k]) / (self.stamps[k + 1] - self.stamps[k])
        p, q = self.poses[k], self.poses[k + 1]
        return (p[0] + a * (q[0] - p[0]), p[1] + a * (q[1] - p[1]), _wrap(p[2] + a * (q[2] - p[2])))


class CycleSelector:
    """Pairs the two lidars' frames of a cycle and keeps one cycle per ``1 / rate_hz``.

    ``offer`` takes every arriving frame (side, header stamp, arrival time on the node's clock, and a payload the
    caller wants back), ``poll`` the clock; both return the cycles to process now, oldest first:
    ``{"cycle": n, "paired": bool, "frames": {side: {"stamp_ns", "arrival_ns", "payload"}}}``. A frame pairs with
    the other lidar's pending frame when the stamps are within ``pair_window_s``; a frame whose partner has not
    come after ``partner_wait_s``, or comes from another cycle, is a cycle of its own. A cycle is kept when each of
    its lidars' previous kept frame is at least 0.9 / rate_hz older; frames of dropped cycles are never filtered.
    """

    def __init__(self, sides, cfg=LiveObstacleConfig()):
        self.sides = tuple(s for s in SIDES if s in tuple(sides))
        if not self.sides or len(self.sides) != len(tuple(sides)):
            raise ValueError("sides must name left and/or right once each")
        self.min_gap_ns = int(round(0.9e9 / cfg.rate_hz))
        self.window_ns = int(round(cfg.pair_window_s * 1e9))
        self.wait_ns = int(round(cfg.partner_wait_s * 1e9))
        self.pending = {}
        self.last_kept = {}
        self.last_seen = {}
        self.cycles = 0
        self.counts = {s: {"frames": 0, "kept": 0, "skipped_by_rate": 0, "unpaired": 0, "out_of_order": 0}
                       for s in self.sides}

    def _other(self, side):
        return next((s for s in self.sides if s != side), None)

    def _close(self, frames, paired):
        if not paired and len(self.sides) == 2:
            for side in frames:
                self.counts[side]["unpaired"] += 1
        if any(f[0] - self.last_kept.get(side, NO_STAMP) < self.min_gap_ns for side, f in frames.items()):
            for side in frames:
                self.counts[side]["skipped_by_rate"] += 1
            return []
        for side, f in frames.items():
            self.last_kept[side] = f[0]
            self.counts[side]["kept"] += 1
        self.cycles += 1
        return [{"cycle": self.cycles, "paired": bool(paired),
                 "frames": {s: {"stamp_ns": frames[s][0], "arrival_ns": frames[s][1], "payload": frames[s][2]}
                            for s in SIDES if s in frames}}]

    def offer(self, side, stamp_ns, arrival_ns, payload=None):
        if side not in self.sides:
            raise ValueError("frame of a lidar that is not read: " + str(side))
        stamp_ns, arrival_ns = int(stamp_ns), int(arrival_ns)
        self.counts[side]["frames"] += 1
        if stamp_ns <= self.last_seen.get(side, NO_STAMP):
            self.counts[side]["out_of_order"] += 1
            return self.poll(arrival_ns)
        self.last_seen[side] = stamp_ns
        out = []
        held = self.pending.pop(side, None)
        if held is not None:                           # its partner never came
            out += self._close({side: held}, paired=False)
        frame = (stamp_ns, arrival_ns, payload)
        other = self._other(side)
        if other is None:
            return out + self._close({side: frame}, paired=False)
        partner = self.pending.get(other)
        if partner is not None and abs(partner[0] - stamp_ns) <= self.window_ns:
            del self.pending[other]
            return out + self._close({side: frame, other: partner}, paired=True)
        if partner is not None and partner[0] < stamp_ns:
            del self.pending[other]                    # an older cycle's frame: it will not pair any more
            out += self._close({other: partner}, paired=False)
        self.pending[side] = frame
        return out + self.poll(arrival_ns)

    def poll(self, now_ns):
        out = []
        for side in sorted(self.pending, key=lambda s: self.pending[s][0]):
            if int(now_ns) - self.pending[side][1] >= self.wait_ns:
                out += self._close({side: self.pending.pop(side)}, paired=False)
        return out


class PoseGate:
    """Holds kept cycles until the pose history covers every frame's measurement time (stamp - lidar time offset),
    at most ``pose_wait_s`` after the cycle was queued; releases them in order."""

    def __init__(self, lidar_time_offset_ms, cfg=LiveObstacleConfig()):
        self.offset_ns = offset_ns(lidar_time_offset_ms)
        self.wait_ns = int(round(cfg.pose_wait_s * 1e9))
        self.queue = deque()
        self.counts = {"cycles": 0, "released": 0, "pose_timeouts": 0, "frames_without_pose": 0,
                       "frames_before_history": 0}

    def add(self, cycle, now_ns):
        self.queue.append((int(now_ns), cycle))
        self.counts["cycles"] += 1

    def release(self, history, now_ns):
        """Released cycles: {"cycle", "poses": {side: pose or None}, "measured_ns", "pose_timeout", "waited_ns"}."""
        out = []
        while self.queue:
            queued, cycle = self.queue[0]
            measured = {s: f["stamp_ns"] - self.offset_ns for s, f in cycle["frames"].items()}
            covered = {s: history.covers(t) for s, t in measured.items()}
            if not all(covered.values()) and int(now_ns) - queued < self.wait_ns:
                break
            self.queue.popleft()
            poses = {s: history.at(t) if covered[s] else None for s, t in measured.items()}
            timeout = not all(covered.values())
            self.counts["released"] += 1
            self.counts["pose_timeouts"] += int(timeout)
            self.counts["frames_without_pose"] += sum(p is None for p in poses.values())
            self.counts["frames_before_history"] += sum(covered[s] and poses[s] is None for s in poses)
            out.append({"cycle": cycle, "poses": poses, "measured_ns": measured, "pose_timeout": timeout,
                        "waited_ns": int(now_ns) - queued})
        return out


def persistence_histories(sides, cfg=ObstacleFilter()):
    """Per lidar the persistence history of ``shadow_obstacles.prepare`` (None while the check is off)."""
    return {s: (RecentFrames(cfg.persist_min_age_s, cfg.persist_max_age_s, cfg.cluster_voxel_m)
                if cfg.persist_min_age_s > 0 else None) for s in sides}


def process_cycle(released, clouds, geometry, plan, cfg=ObstacleFilter(), body=(), histories=None):
    """Obstacle products of the published lidars of one released cycle.

    ``clouds``: side -> reference points (float32 N x 3, mapping_reference) of the frames that could be read;
    ``geometry``: side -> ``side_geometry``; ``histories``: ``persistence_histories`` (mutated as in prepare).
    Returns side -> {"points", "rays", "scan", "stats", "witness", "recent", "partner_stamp_ns"}.
    """
    cycle, poses = released["cycle"], released["poses"]
    histories = histories or {}
    out = {}
    for side in plan["published"]:
        frame = cycle["frames"].get(side)
        if frame is None or side not in clouds:
            continue
        pose = poses.get(side)
        lever = np.asarray(geometry[side]["lever"], dtype=float)
        other = "right" if side == "left" else "left"
        partner = cycle["frames"].get(other)
        witness = None
        if plan["witness"] and partner is not None and other in clouds and pose is not None \
                and poses.get(other) is not None:
            relative = planar_relative(poses[other], pose)
            other_lever = np.asarray(geometry[other]["lever"], dtype=float)
            witness = BoxWitness(move(np.asarray(clouds[other], dtype=float) + other_lever, relative),
                                 move(other_lever[None, :], relative)[0])
        history = histories.get(side)
        t_s = released["measured_ns"][side] * 1e-9
        started = time.perf_counter()
        recent = history.recent(t_s, pose) if history is not None and pose is not None else None
        result = filter_frame(clouds[side], lever, cfg, body, witness, clearing=plan["clearing"],
                              scan=plan["filtered_scan"] and side == "left", recent=recent)
        if history is not None and pose is not None:
            history.add(t_s, pose, result["candidates"])
        out[side] = {"points": result["points"], "rays": result.get("rays"), "scan": result.get("scan"),
                     "stats": result["stats"], "witness": witness is not None, "recent": recent is not None,
                     "partner_stamp_ns": partner["stamp_ns"] if partner is not None else None,
                     "filter_ms": (time.perf_counter() - started) * 1e3}
    return out


def array_sha256(values):
    """sha256 of an array as contiguous little-endian float32 (the clouds' and scans' wire type)."""
    return hashlib.sha256(np.ascontiguousarray(np.asarray(values, dtype="<f4")).tobytes()).hexdigest()


def frame_records(released, clouds, products, lidar_time_offset_ms, timing_ms=None, rejected=None):
    """One obstacles.jsonl line per frame of a released cycle, enough for ``live_check.recompute``.

    ``rejected``: side -> reason for frames whose cloud could not be made (host filter left nothing, ...).
    Poses are written at full precision so the recomputation uses exactly the node's.
    """
    cycle = released["cycle"]
    rejected = rejected or {}
    lines = []
    for side, frame in cycle["frames"].items():
        pose = released["poses"].get(side)
        line = {"schema_version": SCHEMA_VERSION, "cycle": cycle["cycle"], "side": side, "paired": cycle["paired"],
                "stamp_ns": frame["stamp_ns"], "arrival_ns": frame["arrival_ns"],
                "published_stamp_ns": published_stamp_ns(frame["stamp_ns"], lidar_time_offset_ms),
                "measured_ns": released["measured_ns"][side], "pose": None if pose is None else list(pose),
                "pose_timeout": released["pose_timeout"], "waited_ns": released["waited_ns"],
                "published": side in products, "rejected": rejected.get(side)}
        if side in clouds:
            line.update(input_points=int(len(clouds[side])), input_sha256=array_sha256(clouds[side]))
        result = products.get(side)
        if result is not None:
            stats = {k: int(v) for k, v in result["stats"].items() if isinstance(v, (int, np.integer))
                     and not isinstance(v, bool)}
            line.update(partner_stamp_ns=result["partner_stamp_ns"], witness=result["witness"],
                        recent=result["recent"], voxels=int(len(result["points"])),
                        output_sha256=array_sha256(result["points"]), stats=stats)
            if result["rays"] is not None:
                line.update(rays=int(len(result["rays"])), rays_sha256=array_sha256(result["rays"]))
            if result["scan"] is not None:
                line.update(scan_bins=int(np.isfinite(result["scan"]).sum()), scan_sha256=array_sha256(result["scan"]))
        if timing_ms and side in timing_ms:
            line["timing_ms"] = {k: round(float(v), 2) for k, v in timing_ms[side].items()}
        lines.append(line)
    return lines


def _summary(values):
    v = np.asarray(list(values), dtype=float)
    if not len(v):
        return None
    return {"median": round(float(np.median(v)), 1), "p90": round(float(np.percentile(v, 90)), 1),
            "max": round(float(v.max()), 1)}


class LiveStatus:
    """The node's status JSON (``STATUS_TOPIC``), in the format of window D's inputs status."""

    def __init__(self, read_sides, cfg=LiveObstacleConfig(), lidar_time_offset_ms=0.0, offset_source=None):
        self.cfg = cfg
        self.received = {**{s: None for s in read_sides}, "odom": None}
        self.offset_ms, self.offset_source = float(lidar_time_offset_ms), offset_source
        self.failure = None
        self.published = {s: 0 for s in read_sides}
        self.rejections = {}
        self.timing = {k: deque(maxlen=300) for k in ("host_filter_ms", "filter_ms", "pose_wait_ms", "latency_ms")}

    def mark(self, name, now_s):
        self.received[name] = float(now_s)

    def reject(self, side, reason):
        key = side + ": " + str(reason)[:80]
        self.rejections[key] = self.rejections.get(key, 0) + 1

    def fail(self, where, error):
        self.failure = where + ": " + type(error).__name__ + ": " + str(error)

    def status(self, now_s, counts=None):
        now_s = float(now_s)
        stale = [name for name, last in self.received.items()
                 if last is None or now_s - last > self.cfg.stale_after_s or now_s < last]
        state = "FAILED" if self.failure else ("STALE" if stale else "RUNNING")
        return {"state": state, "role": "obstacles", "time_s": round(now_s, 3), "stale_inputs": stale,
                "failure": self.failure, "lidar_time_offset_ms": self.offset_ms,
                "lidar_time_offset_source": self.offset_source, "published": dict(self.published),
                "rejections": dict(self.rejections), "counts": counts or {},
                "timing_ms": {k: _summary(v) for k, v in self.timing.items()},
                "commands_subscribed": False, "hardware_control": False}
