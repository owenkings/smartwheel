"""XT-M60 phase-wrap ghosts predicted from a 3D surface model of the place: pure functions, off unless asked for.

Why (window A, coordinator relay 2026-11-12 A3 item 2, user decision 11-12 22:05): a weak return from a surface
12.5-25 m away (or 25-37.5 m) comes out 12.491 m (or twice that) short, a stable point in the free corridor in front of
the chair (phase_wrap.py, reports/lidar_frequency/20261111T1445Z_room/README.md). With the chair localized in a known
place, the model says what each pixel's ray should hit.

Evaluated (window A, 2026-11-13, reports/lidar_phase_wrap_model/20261112T1722Z_ZL1107/README.md): ZL1107 corridor,
its model walls_m, the FoldPrediction defaults, five recordings of 11-09 and 11-11. Of the ghosts window C's 3D
obstacle input receives, 75-80 % are flagged facing map -x and 2 % facing +x (their surfaces lie behind a partition
that the 2D map extrudes to the ceiling); no map wall point is flagged; 4.9 ms median, 6.7 ms at most per frame on
the Orin. Most ghosts near the floor are not folds at all (unfolded, they lie below the floor).

Rule (``predict``): a candidate pixel (range below ``max_range_m``, amplitude below ``max_amplitude``) is predicted
folded when the first surface along its ray lies at d, |d - (r + k * unambiguous_range_m)| < ``tolerance_m`` for
some k in ``folds``; r is the pixel's measured 3D range, the ray its measured direction. The first surface is a model
voxel, searched from ``min_surface_m`` out to r + max(folds) * unambiguous_range_m + tolerance_m, or the model's
floor plane (metadata ``floor_z_m``) when the ray meets it within ``floor_block_m``. Not searched, and not folded:
a point within ``known_surface_m`` of a model surface (a return from that surface), and a ray that meets the floor
plane within ``floor_block_m`` before its nearest fold distance.

Floor: window A's corridor models hold no floor voxels but the walls' mirror image below the floor plane. A polished
floor met at a grazing angle reflects the ray on to the walls: in the 11-09 and 11-11 corridor recordings the ghosts
0.7 m above the floor 1.3-2 m ahead unfold (r + 12.49 m) onto the corridor's end wall although their rays reach the
floor plane first (reports/lidar_phase_wrap_model/20261112T1722Z_ZL1107/README.md). Steeper rays stop at the floor.

The model (``SurfaceModel``): occupied voxels of a map frame and, per voxel, the Euclidean distance to the nearest
occupied voxel (in voxels, 0 = occupied, at most 255). Rays march by that distance (sphere tracing): far from
surfaces in long steps, near them in half-voxel steps. A built model is a .npz file plus its metadata; the predicted
fold only means "a known surface sits at the folded distance". Real objects standing at such a distance are flagged
too: a consumer must confirm before deleting (window C's three-dimensional obstacle input: amplitude, not on the
floor, the other lidar does not see it, not in the stop zone, localization trusted).

Poses: the lidar's rotation and origin in the model's map frame at the frame's measurement time. ``lidar_pose``
composes them from a planar base_link (axle) pose, the lidar's mount (R_axle_lidar, t_axle_lidar_m) and the model's
axle height in its frame (``SurfaceModel.axle_z_m``: the map frame's z = 0 is where the model was built from).
"""
from collections.abc import Mapping
from dataclasses import asdict, dataclass, fields
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from .phase_wrap import UNAMBIGUOUS_12MHZ_M, frame_points

SCHEMA = "wc_phase_wrap_surface_model_v1"
SQRT3 = math.sqrt(3.0)
OUTSIDE = 255                  # the march grid's padding; distances inside are capped one below


class SurfaceModel:
    """Occupied voxels and their distance field in a map frame (see the module text)."""

    def __init__(self, distance, origin, voxel_m, metadata=None):
        d = np.asarray(distance)
        if d.ndim != 3 or d.dtype != np.uint8 or min(d.shape) < 1:
            raise ValueError("distance must be a 3D uint8 array")
        o = np.asarray(origin, dtype=np.float64).reshape(-1)
        if o.shape != (3,) or not np.isfinite(o).all():
            raise ValueError("origin must be three finite numbers")
        voxel = float(voxel_m)
        if not (math.isfinite(voxel) and 0.01 <= voxel <= 1.0):
            raise ValueError("voxel_m must be within 0.01..1 m")
        self.distance, self.origin, self.voxel_m = d, o, voxel
        self.shape = np.asarray(d.shape, dtype=np.int64)
        self._march = None
        self.metadata = dict(metadata or {})
        axle = self.metadata.get("axle_z_m", 0.0)
        if isinstance(axle, bool) or not isinstance(axle, (int, float)) or not math.isfinite(axle):
            raise ValueError("metadata axle_z_m must be a finite number")
        self.axle_z_m = float(axle)
        floor = self.metadata.get("floor_z_m")
        if floor is not None and (isinstance(floor, bool) or not isinstance(floor, (int, float))
                                  or not math.isfinite(floor)):
            raise ValueError("metadata floor_z_m must be a finite number when given")
        self.floor_z_m = None if floor is None else float(floor)

    @classmethod
    def from_occupancy(cls, occupied, origin, voxel_m, metadata=None):
        """Model of a boolean occupancy grid (x, y, z index order); needs scipy."""
        from scipy import ndimage
        occ = np.asarray(occupied, dtype=bool)
        if occ.ndim != 3 or not occ.any():
            raise ValueError("occupied must be a 3D boolean array with at least one occupied voxel")
        edt = ndimage.distance_transform_edt(~occ)
        return cls(np.minimum(np.floor(edt), 255).astype(np.uint8), origin, voxel_m, metadata)

    @property
    def occupied(self):
        return self.distance == 0

    def march_grid(self):
        """(flat uint8 distances padded by one voxel of OUTSIDE, index strides, highest padded index): built once."""
        if self._march is None:
            padded = np.pad(np.minimum(self.distance, OUTSIDE - 1), 1, constant_values=OUTSIDE)
            strides = np.array([padded.shape[1] * padded.shape[2], padded.shape[2], 1], dtype=np.int64)
            self._march = (padded.reshape(-1), strides, np.array(padded.shape, dtype=np.int64) - 1)
        return self._march

    def save(self, path):
        """Write ``path`` (.npz); returns its sha256."""
        path = Path(path)
        with path.open("xb") as stream:
            np.savez_compressed(stream, schema=np.array(SCHEMA), distance=self.distance, origin=self.origin,
                                voxel_m=np.array(self.voxel_m),
                                metadata=np.array(json.dumps(self.metadata, sort_keys=True, allow_nan=False)))
        return file_sha256(path)

    @classmethod
    def load(cls, path, sha256=None):
        """Read a saved model; with ``sha256`` the file must have exactly that hash."""
        path = Path(path)
        if sha256 is not None and file_sha256(path) != sha256:
            raise ValueError("surface model %s does not have the expected sha256" % path)
        with np.load(path, allow_pickle=False) as data:
            if str(data["schema"]) != SCHEMA:
                raise ValueError("not a %s file: %s" % (SCHEMA, path))
            return cls(data["distance"], data["origin"], float(data["voxel_m"]), json.loads(str(data["metadata"])))


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def lidar_pose(axle_pose, rotation_axle_lidar, t_axle_lidar_m, axle_z_m):
    """(R_map_lidar 3x3, t_map_lidar 3): a lidar in the map for a planar axle pose (x, y, yaw) and its mount."""
    x, y, yaw = (float(v) for v in axle_pose)
    c, s = math.cos(yaw), math.sin(yaw)
    rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    rot = np.asarray(rotation_axle_lidar, dtype=np.float64).reshape(3, 3)
    t = np.asarray(t_axle_lidar_m, dtype=np.float64).reshape(3)
    return rz @ rot, np.array([x, y, float(axle_z_m)]) + rz @ t


def first_surface(origin, directions, model, min_m, max_m, max_steps=400):
    """Distance along each unit ray (map frame) to its first occupied voxel; NaN when none before ``max_m``.

    ``max_m``: one distance for all rays or one per ray. Sphere tracing on the model's distance field: a voxel whose
    centre is D voxels from the nearest occupied voxel centre lets the ray advance (D - sqrt(3)) voxels without
    passing one, at least half a voxel. The hit is the first sample inside an occupied voxel, so it is at most half a
    voxel beyond the voxel's face; a ray that leaves the grid, or runs out of ``max_steps``, has none.
    """
    o = np.asarray(origin, dtype=np.float64).reshape(3)
    u = np.asarray(directions, dtype=np.float64).reshape(-1, 3)
    n = len(u)
    out = np.full(n, np.nan)
    if not n:
        return out
    flat, strides, top = model.march_grid()
    voxel = model.voxel_m
    limit = np.broadcast_to(np.asarray(max_m, dtype=np.float64), (n,)) / voxel
    rel0 = (o - model.origin) / voxel + 1.0          # in voxels of the grid padded by one on every side
    s = np.full(n, float(min_m) / voxel)
    index = np.flatnonzero(s <= limit)
    s, du, limit = s[index], u[index], limit[index]
    for _ in range(int(max_steps)):
        if not len(index):
            break
        cell = (du * s[:, None] + rel0).astype(np.int64)  # truncation: the floor wherever it matters (>= 0)
        np.minimum(cell, top, out=cell)                  # outside the grid: the padding, value OUTSIDE
        np.maximum(cell, 0, out=cell)                    # (ufuncs: np.clip costs 25 us a call here)
        dist = flat[cell @ strides]
        hit = dist == 0
        if hit.any():
            out[index[hit]] = s[hit] * voxel
        step = dist.astype(np.float64)
        step -= SQRT3
        np.maximum(step, 0.5, out=step)
        s += step
        keep = ~hit & (dist != OUTSIDE) & (s <= limit)
        if not keep.all():
            index, s, du, limit = index[keep], s[keep], du[keep], limit[keep]
    return out


@dataclass(frozen=True)
class FoldPrediction:
    """Settings of ``predict``; the defaults are window A's evaluated rule (2026-11-13)."""
    max_range_m: float = 6.5           # only pixels nearer than this can be ghosts
    max_amplitude: float = 400.0       # only pixels dimmer than this can be ghosts (the frame's amplitude scale)
    tolerance_m: float = 0.5           # |d - (r + k * unambiguous_range_m)| below this
    folds: tuple = (1, 2)              # k
    min_surface_m: float = 0.3         # the first-surface search starts this far from the lidar
    floor_block_m: float = 6.0         # the model's floor plane stops a ray that meets it this near; None: never
    known_surface_m: float = 0.2       # a point this near a model surface is that surface's return: not searched
    max_steps: int = 400
    unambiguous_range_m: float = UNAMBIGUOUS_12MHZ_M

    def __post_init__(self):
        for name in ("max_range_m", "max_amplitude", "tolerance_m", "min_surface_m", "unambiguous_range_m",
                     "known_surface_m"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float, np.integer, np.floating)) \
                    or not math.isfinite(float(value)) or float(value) <= 0:
                raise ValueError(name + " must be a positive finite number")
            object.__setattr__(self, name, float(value))
        block = self.floor_block_m
        if block is not None:
            if isinstance(block, bool) or not isinstance(block, (int, float, np.integer, np.floating)) \
                    or not math.isfinite(float(block)) or not 0.0 < float(block) <= 100.0:
                raise ValueError("floor_block_m must be None or a distance within (0, 100] m")
            object.__setattr__(self, "floor_block_m", float(block))
        folds = self.folds
        if isinstance(folds, (str, bytes)) or not hasattr(folds, "__iter__"):
            raise ValueError("folds must be a list of 1, 2 or 3")
        folds = tuple(folds)
        if not folds or len(set(folds)) != len(folds) or any(isinstance(k, bool) or k not in (1, 2, 3) for k in folds):
            raise ValueError("folds must be a list of 1, 2 or 3")
        object.__setattr__(self, "folds", tuple(sorted(int(k) for k in folds)))
        steps = self.max_steps
        if isinstance(steps, bool) or not isinstance(steps, (int, np.integer)) or not 10 <= int(steps) <= 5000:
            raise ValueError("max_steps must be an integer within 10..5000")
        object.__setattr__(self, "max_steps", int(steps))
        if not 5.0 <= self.unambiguous_range_m <= 50.0:
            raise ValueError("unambiguous_range_m must be within 5..50 m")
        if self.tolerance_m > 2.0 or self.max_range_m >= self.unambiguous_range_m:
            raise ValueError("tolerance_m must be at most 2 m and max_range_m below unambiguous_range_m")
        if self.min_surface_m >= self.max_range_m:
            raise ValueError("min_surface_m must be below max_range_m")
        if self.known_surface_m > 1.0:
            raise ValueError("known_surface_m must be at most 1 m")

    @property
    def search_m(self):
        return self.max_range_m + max(self.folds) * self.unambiguous_range_m + self.tolerance_m

    def as_dict(self):
        values = asdict(self)
        values["folds"] = list(self.folds)
        return values

    @classmethod
    def from_dict(cls, value):
        """Settings from ``as_dict`` output or a partial dict of the same keys; unknown keys are refused."""
        if not isinstance(value, Mapping):
            raise ValueError("fold prediction settings must be an object")
        unknown = set(value) - {field.name for field in fields(cls)}
        if unknown:
            raise ValueError("unknown fold prediction settings: " + ", ".join(sorted(unknown)))
        return cls(**dict(value))


def predict(xyz, amplitudes, rotation, origin, model, cfg=FoldPrediction()):
    """(folded bool (n,), d float64 (n,)) per pixel of one frame; the inputs are not modified.

    ``xyz`` (n, 3): the pixels' points in the lidar frame, NaN, infinite or (0, 0, 0) for no return; ``amplitudes``
    (n,); ``rotation``, ``origin``: the lidar in the model's map frame (``lidar_pose``). ``d`` is the first surface
    along a candidate pixel's ray: a model voxel, or the model's floor plane (``floor_z_m``) when the ray meets it
    within ``floor_block_m``; NaN when there is none within r + max(folds) * unambiguous_range_m + tolerance_m, for
    the candidates that are not searched (a point within ``known_surface_m`` of a model surface; a ray that meets the
    floor plane within floor_block_m nearer than r + min(folds) * unambiguous_range_m - tolerance_m: neither can be
    folded), and for every pixel that is not a candidate.
    """
    p = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    a = np.asarray(amplitudes, dtype=np.float64).reshape(-1)
    if len(a) != len(p):
        raise ValueError("xyz and amplitudes must have one entry per pixel")
    rot = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    o = np.asarray(origin, dtype=np.float64).reshape(3)
    if not (np.isfinite(rot).all() and np.isfinite(o).all()):
        raise ValueError("the lidar pose must be finite")
    folded = np.zeros(len(p), dtype=bool)
    d = np.full(len(p), np.nan)
    with np.errstate(invalid="ignore", over="ignore"):
        r = np.linalg.norm(p, axis=1)
        candidate = np.isfinite(r) & (r > 0.0) & (r < cfg.max_range_m) & np.isfinite(a) & (a < cfg.max_amplitude)
    index = np.flatnonzero(candidate)
    if not len(index):
        return folded, d
    rays = (p[index] / r[index, None]) @ rot.T
    # a point on a known surface is that surface's return (not folded): one look-up instead of a march
    flat, strides, top = model.march_grid()
    cell = ((o + rays * r[index, None] - model.origin) / model.voxel_m + 1.0).astype(np.int64)
    np.minimum(cell, top, out=cell)
    np.maximum(cell, 0, out=cell)
    known = flat[cell @ strides] * model.voxel_m <= cfg.known_surface_m
    index, rays = index[~known], rays[~known]
    if cfg.floor_block_m is not None and model.floor_z_m is not None and o[2] > model.floor_z_m:
        floor = np.full(len(index), np.inf)
        down = rays[:, 2] < 0.0
        floor[down] = (o[2] - model.floor_z_m) / -rays[down, 2]
        floor[(floor < cfg.min_surface_m) | (floor > cfg.floor_block_m)] = np.inf
        # a ray stopped by the floor before its nearest fold distance cannot be folded: not searched
        hopeless = floor < r[index] + min(cfg.folds) * cfg.unambiguous_range_m - cfg.tolerance_m
        index, rays, floor = index[~hopeless], rays[~hopeless], floor[~hopeless]
    else:
        floor = np.full(len(index), np.inf)
    limit = np.minimum(r[index] + max(cfg.folds) * cfg.unambiguous_range_m + cfg.tolerance_m, floor)
    hit = first_surface(o, rays, model, cfg.min_surface_m, limit, cfg.max_steps)
    hit = np.where(np.isfinite(floor) & ~(hit <= floor), floor, hit)
    d[index] = hit
    with np.errstate(invalid="ignore"):
        for k in cfg.folds:
            folded[index] |= np.abs(hit - (r[index] + k * cfg.unambiguous_range_m)) < cfg.tolerance_m
    return folded, d


def source_frame_predict(message, rotation, origin, model, cfg=FoldPrediction()):
    """``predict`` for a SourceFrame (ROS message or dict alike), entries in cloud order; not modified."""
    xyz, amplitude, _ = frame_points(message)
    return predict(xyz, amplitude, rotation, origin, model, cfg)
