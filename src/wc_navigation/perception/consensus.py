"""Free space seen by the other lidar, used to reject one lidar's unsupported clusters.

Both XT-M60 units return small clusters in open space near their own forward axis
(about 75 % of the corridor frames, each lidar; mostly within +-12 deg of its axis,
0.5-1.6 m above the floor, nothing beneath them). A real object there would also
stop the other lidar's rays, which start 0.46 m to the side. So a cluster with
nothing beneath it is dropped when most of its voxels lie on the other lidar's
rays in front of their returns and hold none of its returns.

Everything here works in one planar frame (the frame being filtered): the other
lidar's points are moved there with the recorded axle poses of both capture times.
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
