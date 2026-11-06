"""Log-odds occupancy grid built from planar scans at known sensor poses.

Used by the odometry-anchored mapper: poses come from wheel + IMU odometry,
the project's mapping pose authority (docs/adr/0001), and are not corrected by
scan matching or loop closure. Rays mark free space up to just before their
end cell, finite returns mark the end cell occupied, no-return beams (+inf)
clear up to the configured range, and too-close beams (-inf) change nothing.
Grids use ROS bottom-up row order: ``data[row, column]``.
"""
from __future__ import annotations

import math

import numpy as np

from ..geometry import _number

MAX_CELLS_PER_SIDE = 4000


class LogOddsGrid:
    """Fixed square log-odds grid centred on ``center_xy`` in the odom frame."""

    def __init__(self, resolution=0.05, size_m=60.0, center_xy=(0.0, 0.0), *, hit=0.85, miss=-0.4,
                 lower=-2.0, upper=3.5, occupied_at=1.0, free_at=-1.0, clear_no_return=True):
        self.resolution = _number(resolution, "resolution", positive=True)
        size = _number(size_m, "size", positive=True)
        cells = int(math.ceil(size / self.resolution))
        if cells > MAX_CELLS_PER_SIDE:
            raise ValueError("grid exceeds %d cells per side" % MAX_CELLS_PER_SIDE)
        self.hit = _number(hit, "hit", positive=True)
        self.miss = _number(miss, "miss")
        self.lower = _number(lower, "lower")
        self.upper = _number(upper, "upper", positive=True)
        self.occupied_at = _number(occupied_at, "occupied_at", positive=True)
        self.free_at = _number(free_at, "free_at")
        if not (self.miss < 0 and self.lower < 0 and self.free_at < 0):
            raise ValueError("miss, lower and free_at must be negative log-odds")
        if not (self.lower <= self.free_at and self.occupied_at <= self.upper):
            raise ValueError("classification thresholds must lie within the clamp range")
        if type(clear_no_return) is not bool:
            raise ValueError("clear_no_return must be a boolean")
        self.clear_no_return = clear_no_return
        cx, cy = (_number(v, "center") for v in center_xy)
        self.cells = cells
        self.origin = (cx - cells * self.resolution / 2.0, cy - cells * self.resolution / 2.0)
        self.logodds = np.zeros((cells, cells), dtype=np.float32)
        # Per-cell evidence counts (at most one per scan), kept beside the clamped log-odds,
        # whose value depends on the order of hits and misses.
        self.hits = np.zeros((cells, cells), dtype=np.uint16)
        self.misses = np.zeros((cells, cells), dtype=np.uint16)
        self.scans = 0

    def _flat(self, x, y):
        res = self.resolution
        col = np.floor((x - self.origin[0]) / res).astype(np.int64)
        row = np.floor((y - self.origin[1]) / res).astype(np.int64)
        inside = (col >= 0) & (col < self.cells) & (row >= 0) & (row < self.cells)
        return row * self.cells + col, inside

    def integrate(self, sensor_pose, angles, ranges, range_min, range_max):
        """Add one scan taken at ``sensor_pose`` = (x, y, yaw) of the sensor."""
        x, y, yaw = (_number(v, "sensor pose") for v in sensor_pose)
        angles = np.asarray(angles, dtype=float)
        ranges = np.asarray(ranges, dtype=float)
        if angles.ndim != 1 or angles.shape != ranges.shape or not len(angles):
            raise ValueError("angles and ranges must be matching nonempty vectors")
        if not np.isfinite(angles).all():
            raise ValueError("scan angles must be finite")
        range_min = _number(range_min, "range_min", positive=True)
        range_max = _number(range_max, "range_max", positive=True)
        if range_min >= range_max or range_max > 50.0:
            raise ValueError("scan range must satisfy 0 < min < max <= 50 m")
        with np.errstate(invalid="ignore"):
            finite = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
        no_return = np.isposinf(ranges) if self.clear_no_return else np.zeros(len(ranges), dtype=bool)
        length = np.where(finite, ranges, np.where(no_return, range_max, 0.0))
        res = self.resolution
        step = res * 0.5
        samples = step * np.arange(1, int(math.ceil(range_max / step)) + 1)
        cos, sin = np.cos(yaw + angles)[:, None], np.sin(yaw + angles)[:, None]
        # Stop short of the end cell so a return never clears its own wall.
        within = samples[None, :] < (length[:, None] - res * 0.75)
        flat, inside = self._flat(x + cos * samples, y + sin * samples)
        free = np.unique(flat[within & inside])
        hit_flat, hit_inside = self._flat(x + cos[finite, 0] * ranges[finite], y + sin[finite, 0] * ranges[finite])
        hits = np.unique(hit_flat[hit_inside])
        free = np.setdiff1d(free, hits, assume_unique=True)
        values = self.logodds.reshape(-1)
        values[free] = np.maximum(values[free] + self.miss, self.lower)
        values[hits] = np.minimum(values[hits] + self.hit, self.upper)
        for counts, index in ((self.misses.reshape(-1), free), (self.hits.reshape(-1), hits)):
            counts[index] = np.minimum(counts[index].astype(np.int32) + 1, np.iinfo(np.uint16).max)
        self.scans += 1
        return {"beams": int(len(ranges)), "finite_returns": int(finite.sum()),
                "no_return": int(no_return.sum()), "free_cells": int(len(free)), "hit_cells": int(len(hits))}

    def trinary(self, *, contested_free=False, min_observations=5, max_hit_ratio=0.25):
        """Free / occupied / unknown by the log-odds thresholds.

        With ``contested_free`` a cell between the thresholds is also free when
        it was observed in at least ``min_observations`` scans and returned in
        at most ``max_hit_ratio`` of them: rays usually passed through it, as
        where people walked or a few late returns follow many misses. Cells
        observed fewer times stay unknown, and the occupied threshold still wins.
        """
        data = np.full(self.logodds.shape, -1, dtype=np.int8)
        free = self.logodds <= self.free_at
        if contested_free:
            hits = self.hits.astype(np.int64)
            total = hits + self.misses
            free |= (total >= int(min_observations)) & (hits <= float(max_hit_ratio) * total)
        data[free] = 0
        data[self.logodds >= self.occupied_at] = 100
        return data

    def cropped(self, padding_m=1.0, block_m=2.0, **classification):
        """Trinary data and origin covering observed cells, padded and snapped
        outward to whole ``block_m`` blocks so the published size rarely changes.
        ``classification`` is passed to :meth:`trinary`. Returns None before
        anything was observed."""
        observed = self.logodds != 0
        if not observed.any():
            return None
        res = self.resolution
        pad = int(math.ceil(_number(padding_m, "padding", nonnegative=True) / res))
        block = max(1, int(round(_number(block_m, "block", positive=True) / res)))
        rows = np.flatnonzero(observed.any(axis=1))
        cols = np.flatnonzero(observed.any(axis=0))
        r0 = max(0, (int(rows[0]) - pad) // block * block)
        c0 = max(0, (int(cols[0]) - pad) // block * block)
        r1 = min(self.cells, -(-(int(rows[-1]) + 1 + pad) // block) * block)
        c1 = min(self.cells, -(-(int(cols[-1]) + 1 + pad) // block) * block)
        data = self.trinary(**classification)[r0:r1, c0:c1]
        return data, (self.origin[0] + c0 * res, self.origin[1] + r0 * res, 0.0)

    def parameters(self):
        return {"resolution_m": self.resolution, "cells_per_side": self.cells, "origin": list(self.origin),
                "hit": self.hit, "miss": self.miss, "lower": self.lower, "upper": self.upper,
                "occupied_at": self.occupied_at, "free_at": self.free_at,
                "clear_no_return": self.clear_no_return}
