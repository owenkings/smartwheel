"""Evidence from other captures, used to judge one lidar's unsupported clusters.

Both XT-M60 units return small clusters in open space near their own forward axis
(about 75 % of the corridor frames, each lidar; mostly within +-12 deg of its axis,
0.5-1.6 m above the floor, nothing beneath them). A real object there would also
stop the other lidar's rays, which start 0.46 m to the side. So a cluster with
nothing beneath it is dropped when most of its voxels lie on the other lidar's
rays in front of their returns and hold none of its returns (``Witness``).

Both lidars sometimes return such clusters at the same place, so the witness also
confirms some of them. Those clusters then move with the chair instead of staying
put: seen again a second later they are somewhere else in the world. ``Recent``
holds the same lidar's candidates of earlier frames (``RecentFrames`` collects
them), so a cluster with nothing at its world position 0.6-2 s earlier can be
dropped while a real overhang, seen from frame to frame at the same place, stays.

Everything here works in one planar frame (the frame being filtered): points of
other captures are moved there with the recorded axle poses of both capture times.
``BoxWitness`` gives the same verdicts as ``Witness`` with less work, for the live
input: the whole frame filter took about 62 -> 24 ms per frame on the ZL1107
validation recording, with outputs equal on all 971 frames (and 2905 mapping frames).
"""
from __future__ import annotations

import math

import numpy as np

_OFFSET = 1 << 20


def _codes(keys):
    keys = np.asarray(keys, dtype=np.int64) + _OFFSET // 2
    return (keys[:, 0] * _OFFSET + keys[:, 1]) * _OFFSET + keys[:, 2]


def planar_relative(pose_from, pose_to):
    """(dx, dy, dyaw): pose_from expressed in the frame of pose_to (both world x, y, yaw)."""
    c, s = math.cos(pose_to[2]), math.sin(pose_to[2])
    dx, dy = pose_from[0] - pose_to[0], pose_from[1] - pose_to[1]
    return c * dx + s * dy, -s * dx + c * dy, math.atan2(math.sin(pose_from[2] - pose_to[2]),
                                                          math.cos(pose_from[2] - pose_to[2]))


def move(points_base, relative):
    """Base-frame points of one capture, expressed in another capture's base frame."""
    dx, dy, dyaw = relative
    c, s = math.cos(dyaw), math.sin(dyaw)
    p = np.asarray(points_base, dtype=float).reshape(-1, 3)
    return np.column_stack([dx + c * p[:, 0] - s * p[:, 1], dy + s * p[:, 0] + c * p[:, 1], p[:, 2]])


class Witness:
    """Voxels the other lidar saw through (free) and hit (occupied), in the filtered frame's base_link."""

    def __init__(self, points_base, origin_base, voxel_m=0.10, step_m=0.05, stop_short_m=0.15, max_range_m=6.0):
        self._inputs = (points_base, origin_base, step_m, stop_short_m, max_range_m)
        self.voxel = float(voxel_m)
        self.free = self.occupied = None
        self.rays = None

    def _build(self):
        points_base, origin_base, step_m, stop_short_m, max_range_m = self._inputs
        p = np.asarray(points_base, dtype=float).reshape(-1, 3)
        o = np.asarray(origin_base, dtype=float).reshape(3)
        d = p - o
        length = np.linalg.norm(d, axis=1)
        ok = np.isfinite(length) & (length > stop_short_m + step_m)
        d, length, p = d[ok], length[ok], p[ok]
        self.rays = int(len(p))
        if len(p):
            unit = d / length[:, None]
            reach = np.minimum(length - stop_short_m, max_range_m)
            samples = np.arange(step_m, float(reach.max()) + step_m, step_m)
            free = []
            for start in range(0, len(p), 2000):
                u, r = unit[start:start + 2000], reach[start:start + 2000]
                valid = samples[None, :] <= r[:, None]
                pts = o + u[:, None, :] * samples[None, :, None]
                free.append(_codes(np.floor(pts[valid] / self.voxel)))
            self.free = np.unique(np.concatenate(free))
            occ = np.floor(p / self.voxel).astype(np.int64)
            grown = [occ + np.array([i, j, k]) for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1)]
            self.occupied = np.unique(_codes(np.concatenate(grown)))
        else:
            self.free = self.occupied = np.zeros(0, dtype=np.int64)

    def shares(self, points_base):
        """(seen through, confirmed): shares of the cluster's voxels on the other lidar's rays without any of
        its returns in or next to them, and with one of its returns in or next to them."""
        if self.free is None:
            self._build()
        keys = np.unique(np.floor(np.asarray(points_base, dtype=float).reshape(-1, 3) / self.voxel).astype(np.int64),
                         axis=0)
        if not len(keys):
            return 0.0, 0.0
        codes = _codes(keys)
        hit = np.isin(codes, self.occupied) if len(self.occupied) else np.zeros(len(codes), dtype=bool)
        through = (np.isin(codes, self.free) if len(self.free) else np.zeros(len(codes), dtype=bool)) & ~hit
        return float(through.mean()), float(hit.mean())


class BoxWitness(Witness):
    """``Witness`` with the same verdicts, answered around each queried cluster only (used by the live input).

    ``Witness`` samples every ray of the other lidar (about 8000 rays, up to 120 samples each) and sorts all those
    voxel codes once per frame, mostly for voxels no cluster touches. Here a ``shares`` query samples only the rays
    whose sampled segment can enter the cluster's voxel box (slab test on the box grown by 1 um) and looks only at
    returns next to the box. The samples use the same arithmetic as ``Witness``, so the free and occupied voxels of
    the box, and with them the shares, are the same.
    """

    _GROW = np.array([[i, j, k] for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1)], dtype=np.int64)
    _PAD_M = 1e-6

    def _prepare(self):
        points_base, origin_base, step_m, stop_short_m, max_range_m = self._inputs
        p = np.asarray(points_base, dtype=float).reshape(-1, 3)
        o = np.asarray(origin_base, dtype=float).reshape(3)
        d = p - o
        length = np.linalg.norm(d, axis=1)
        ok = np.isfinite(length) & (length > stop_short_m + step_m)
        d, length, p = d[ok], length[ok], p[ok]
        self.rays = int(len(p))
        self._origin = o
        if len(p):
            self._unit = d / length[:, None]
            self._reach = np.minimum(length - stop_short_m, max_range_m)
            self._samples = np.arange(step_m, float(self._reach.max()) + step_m, step_m)
        else:
            self._unit, self._reach, self._samples = np.zeros((0, 3)), np.zeros(0), np.zeros(0)
        self._returns = np.floor(p / self.voxel).astype(np.int64)

    def _crossing(self, lo, hi):
        """Rays whose sampled segment [first sample, reach] can enter the box of voxels lo..hi."""
        box_lo, box_hi = lo * self.voxel - self._PAD_M, (hi + 1) * self.voxel + self._PAD_M
        u, o = self._unit, self._origin
        with np.errstate(divide="ignore", invalid="ignore"):
            t1, t2 = (box_lo - o) / u, (box_hi - o) / u
        tmin, tmax = np.minimum(t1, t2), np.maximum(t1, t2)
        parallel = u == 0
        inside = (o >= box_lo) & (o <= box_hi)
        tmin = np.where(parallel, np.where(inside, -np.inf, np.inf), tmin)
        tmax = np.where(parallel, np.where(inside, np.inf, -np.inf), tmax)
        enter, leave = tmin.max(axis=1), tmax.min(axis=1)
        return np.flatnonzero((enter <= leave) & (leave >= self._samples[0] - self._PAD_M)
                              & (enter <= self._reach + self._PAD_M))

    def shares(self, points_base):
        if not hasattr(self, "_unit"):
            self._prepare()
        keys = np.unique(np.floor(np.asarray(points_base, dtype=float).reshape(-1, 3) / self.voxel).astype(np.int64),
                         axis=0)
        if not len(keys):
            return 0.0, 0.0
        codes = _codes(keys)
        lo, hi = keys.min(axis=0), keys.max(axis=0)
        near = np.all((self._returns >= lo - 1) & (self._returns <= hi + 1), axis=1)
        if near.any():
            grown = (self._returns[near][:, None, :] + self._GROW[None, :, :]).reshape(-1, 3)
            hit = np.isin(codes, np.unique(_codes(grown)))
        else:
            hit = np.zeros(len(codes), dtype=bool)
        through = np.zeros(len(codes), dtype=bool)
        if len(self._unit):
            chosen = self._crossing(lo, hi)
            if len(chosen):
                unit, reach = self._unit[chosen], self._reach[chosen]
                valid = self._samples[None, :] <= reach[:, None]
                pts = self._origin + unit[:, None, :] * self._samples[None, :, None]
                sample_keys = np.floor(pts[valid] / self.voxel)
                sample_keys = sample_keys[np.all((sample_keys >= lo) & (sample_keys <= hi), axis=1)]
                if len(sample_keys):
                    through = np.isin(codes, _codes(sample_keys)) & ~hit
        return float(through.mean()), float(hit.mean())


class Recent:
    """Earlier obstacle candidates of the same lidar, in the filtered frame's base_link."""

    def __init__(self, points_base, voxel_m=0.10):
        p = np.asarray(points_base, dtype=float).reshape(-1, 3)
        self.voxel = float(voxel_m)
        self.points = int(len(p))
        if len(p):
            occ = np.floor(p / self.voxel).astype(np.int64)
            grown = [occ + np.array([i, j, k]) for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1)]
            self.codes = np.unique(_codes(np.concatenate(grown)))
        else:
            self.codes = np.zeros(0, dtype=np.int64)

    def share(self, points_base):
        """Share of the cluster's voxels with an earlier candidate in or next to them."""
        keys = np.unique(np.floor(np.asarray(points_base, dtype=float).reshape(-1, 3) / self.voxel).astype(np.int64),
                         axis=0)
        if not len(keys) or not len(self.codes):
            return 0.0
        return float(np.isin(_codes(keys), self.codes).mean())


class RecentFrames:
    """Candidates of a lidar's earlier frames with their capture poses; ``recent`` moves them to a new frame."""

    def __init__(self, min_age_s, max_age_s, voxel_m=0.10):
        self.min_age_s, self.max_age_s, self.voxel = float(min_age_s), float(max_age_s), float(voxel_m)
        self.entries = []

    def add(self, t_s, pose, points_base):
        self.entries.append((float(t_s), tuple(float(v) for v in pose), np.asarray(points_base, dtype=float)))
        self.entries = [e for e in self.entries if float(t_s) - e[0] <= self.max_age_s + 1e-9]

    def recent(self, t_s, pose):
        """``Recent`` of the frames ``min_age_s``-``max_age_s`` older than ``t_s``, or None if there are none."""
        chosen = [e for e in self.entries if self.min_age_s - 1e-9 <= t_s - e[0] <= self.max_age_s + 1e-9]
        if not chosen:
            return None
        moved = [move(points, planar_relative(capture_pose, pose)) for _, capture_pose, points in chosen]
        return Recent(np.concatenate(moved) if moved else np.zeros((0, 3)), self.voxel)
