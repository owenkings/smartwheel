"""Obstacle points from one prepared lidar cloud; pure numpy, no ROS.

Input points are the frontend's prepared cloud: origin at the lidar, axes aligned
with the drive axle (``mapping_reference``). Adding the frozen lever arm
``t_axle_lidar_m`` gives base_link (axle midpoint, x forward, y left, z up). The
floor lies ``axle_height_m`` below base_link.

Stages, in order (each removed point is counted under the first stage that drops it):

1. ``height``: outside ``[min_height_m, max_height_m]`` above the floor (floor and
   overhead structure).
2. ``range``: horizontal range from the lidar beyond ``max_range_m``.
3. ``far_low``: below ``low_height_m`` and farther than ``low_max_range_m`` from the
   lidar. On the corridor recordings the left lidar's far floor (2.5-4 m, near the
   middle of its field of view) bends up to 0.3-0.45 m above the floor; those
   returns would otherwise be obstacles in mid-corridor (task 0 audit,
   reports/navigation_obstacles/20261107T131555Z_live_mark_audit). Low objects are
   seen again once they are within ``low_max_range_m``.
4. ``body``: inside the body outline (grown by ``body_margin_m``) up to
   ``body_max_height_m``: footrests, the occupant's legs, the chair itself.
5. ``contradicted`` and ``floating``: clusters (at most ``unsupported_max_points``
   points) whose lowest point is at least ``floating_min_gap_m`` above every return
   beneath them, while that column is inside the lidar's view ("unsupported").
   Both lidars return such clusters in open space near their own forward axis in
   about 75 % of the corridor frames, including while the chair stood still, at
   ranges that jumped by +-0.25 m between frames. With a witness (the other
   lidar's nearest frame) a cluster is dropped as ``contradicted`` when the other
   lidar saw through most of its voxels, and kept when the other lidar also hit
   it. Without a verdict, only small clusters (``floating_max_points`` points,
   ``floating_max_extent_m`` across) are dropped as ``floating``; large overhangs
   such as table edges are kept.
5a. ``seen_through`` (off by default, ``witness_scope`` "all"): the remaining
   clusters that stage 5 did not judge (supported ones) are judged by the witness as
   well. A cluster is dropped when the other lidar saw through at least
   ``witness_min_share`` of its voxels and hit fewer than ``WITNESS_CONFIRM_SHARE`` of
   them. Two guards keep what could be real: clusters larger than
   ``unsupported_max_points`` (walls, large furniture), and clusters standing on the
   floor, i.e. with a return of the same lidar between ``min_height_m`` and
   ``seen_through_floor_m`` within ``cluster_voxel_m`` (x, y) of their points, from
   any stage (so legs that stage 3 drops beyond 2 m still count). Meant for
   phase-wrap ghosts of the XT-M60 (12 MHz: a surface 12.5-18.5 m away returns
   0-6 m away along the same ray, fixed in the world; they hang 0.5-1.3 m above the
   floor, and other ghost points beneath them make them "supported", so stage 5 does
   not judge them; status/C.md 2026-11-12). A real object stops the other lidar's
   rays at or before it, so it is hit or not judged. What it can still wrongly drop:
   a real thing that does not stand on the floor in this lidar's view (an overhang
   without visible legs, a hanging object) and is at most 400 points, when the other
   lidar's beams pass beside or through it (thinner than the beam spacing, a mesh,
   glass), when the two lidars' relative mounting is off by more than about 0.1 m,
   or when the other lidar returns a wrong far value there. A ghost lying in front of
   the other lidar is usually not judged (that lidar's own wrapped return stops its
   ray before the ghost) and stays.
6. ``transient`` (off by default, ``persist_min_age_s`` > 0): an unsupported cluster
   that survived stage 5 is dropped when less than ``persist_min_share`` of its
   voxels had a candidate of the same lidar at the same world position
   ``persist_min_age_s``-``persist_max_age_s`` earlier (``consensus.Recent``).
   Clusters that both lidars return at the same place pass the witness, but they
   move with the chair; a real overhang stays where it is.
7. ``sparse``: voxels holding fewer than ``min_voxel_points`` returns (off by default).

The kept points are reduced to one point (the mean) per ``voxel_m`` voxel and
returned in the lidar frame, ready to publish in a frame placed at the lidar.
``clearing_rays`` gives the frame's free-space rays for costmap clearing.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math

import numpy as np

AXLE_HEIGHT_M = 0.185        # drive-wheel hub centre above the floor, measured 2026-10-05
STAGES = ("height", "range", "far_low", "body", "contradicted", "floating", "transient", "sparse")
WITNESS_SCOPES = ("unsupported", "all")
# A cluster with at least this share of its voxels hit by the other lidar counts as confirmed (seen by both).
WITNESS_CONFIRM_SHARE = 0.3
# The shadow runs' 161-bin scan (amcl_replay.build_scan): +-60 deg in 0.75 deg bins, 0.3-8 m.
SCAN_ANGLE_MIN, SCAN_INCREMENT, SCAN_BINS = -math.pi / 3, math.pi / 240, 161
SCAN_RANGE_M = (0.30, 8.0)


def _positive(value, name):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise ValueError(name + " must be a positive finite number")
    return number


@dataclass(frozen=True)
class ObstacleFilter:
    """Tunables of the obstacle filter (heights above the floor, ranges from the lidar)."""
    axle_height_m: float = AXLE_HEIGHT_M
    min_height_m: float = 0.05
    max_height_m: float = 1.60          # vehicle with occupant plus margin: assumed, not measured
    max_range_m: float = 4.0
    low_height_m: float = 0.50
    low_max_range_m: float = 2.0
    body_margin_m: float = 0.0
    body_max_height_m: float = 1.60
    floating_max_points: int = 60
    floating_max_extent_m: float = 0.30
    floating_min_gap_m: float = 0.30
    unsupported_max_points: int = 400   # larger unsupported clusters are always kept
    witness_min_share: float = 0.5      # share of a cluster's voxels the other lidar saw through
    # Clusters the witness judges: "unsupported" (stage 5 only, the default) or "all" (stage 5a as well).
    witness_scope: str = "unsupported"
    # Stage 5a keeps a cluster with a return of the same lidar between min_height_m and this height in its column.
    seen_through_floor_m: float = 0.25
    # Stage "transient": 0 = off; otherwise the age window of the same lidar's earlier frames checked.
    persist_min_age_s: float = 0.0
    persist_max_age_s: float = 2.0
    persist_min_share: float = 0.5
    lowest_ray_deg: float = 28.0        # lowest ray below horizontal (blind-zone recording, A)
    voxel_m: float = 0.05
    cluster_voxel_m: float = 0.10
    min_voxel_points: int = 1
    # Filtered 2D scan (same bins as /scan) from all kept points up to this range.
    scan_max_range_m: float = 8.0
    # Clearing rays (free space for the costmap): nearest return per direction bin, shortened.
    clear_max_range_m: float = 10.0
    clear_bin_deg: float = 1.5
    clear_stop_short_m: float = 0.20
    clear_stop_fraction: float = 0.10
    clear_min_end_height_m: float = 0.15

    def __post_init__(self):
        for name in ("axle_height_m", "max_height_m", "max_range_m", "low_height_m", "low_max_range_m",
                     "body_max_height_m", "floating_max_extent_m", "floating_min_gap_m", "lowest_ray_deg",
                     "voxel_m", "cluster_voxel_m", "scan_max_range_m", "clear_max_range_m", "clear_bin_deg",
                     "clear_stop_short_m", "clear_stop_fraction", "clear_min_end_height_m"):
            object.__setattr__(self, name, _positive(getattr(self, name), name))
        for name in ("min_height_m", "body_margin_m"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0:
                raise ValueError(name + " must be a non-negative finite number")
            object.__setattr__(self, name, value)
        for name in ("floating_max_points", "min_voxel_points", "unsupported_max_points"):
            value = int(getattr(self, name))
            if value < 0 or value != getattr(self, name):
                raise ValueError(name + " must be a non-negative integer")
            object.__setattr__(self, name, value)
        for name in ("witness_min_share", "persist_min_share"):
            share = float(getattr(self, name))
            if not 0 < share <= 1:
                raise ValueError(name + " must be within (0, 1]")
            object.__setattr__(self, name, share)
        low, high = float(self.persist_min_age_s), float(self.persist_max_age_s)
        if not (math.isfinite(low) and math.isfinite(high) and 0 <= low and (low == 0 or low < high <= 5.0)):
            raise ValueError("persist ages must satisfy 0 (off) or 0 < min < max <= 5 s")
        object.__setattr__(self, "persist_min_age_s", low)
        object.__setattr__(self, "persist_max_age_s", high)
        if not self.min_height_m < self.max_height_m <= 3.0:
            raise ValueError("height window must satisfy min < max <= 3 m")
        if self.max_range_m > 15.0 or self.low_max_range_m > self.max_range_m:
            raise ValueError("ranges must satisfy low_max_range <= max_range <= 15 m")
        if not 0 < self.lowest_ray_deg < 89:
            raise ValueError("lowest_ray_deg must be within 0..89")
        if self.clear_max_range_m > 15.0 or self.clear_stop_fraction >= 1.0 or self.clear_bin_deg > 10.0:
            raise ValueError("clearing settings out of range")
        if self.scan_max_range_m > SCAN_RANGE_M[1]:
            raise ValueError("scan_max_range_m cannot exceed the scan's 8 m")
        if self.witness_scope not in WITNESS_SCOPES:
            raise ValueError("witness_scope must be one of " + ", ".join(WITNESS_SCOPES))
        floor_m = float(self.seen_through_floor_m)
        if not (math.isfinite(floor_m) and self.min_height_m < floor_m <= 1.0):
            raise ValueError("seen_through_floor_m must be above min_height_m and at most 1 m")
        object.__setattr__(self, "seen_through_floor_m", floor_m)

    def as_dict(self):
        values = asdict(self)
        if self.witness_scope == "unsupported":
            values.pop("witness_scope")     # the default: recorded settings stay as before the option
            if self.seen_through_floor_m == 0.25:
                values.pop("seen_through_floor_m")
        return values

    @property
    def stage_order(self):
        """Stage names in the order they drop points (``seen_through`` only while witness_scope is "all")."""
        if self.witness_scope == "all":
            return STAGES[:6] + ("seen_through",) + STAGES[6:]
        return STAGES

    @property
    def floor_z_base(self):
        """Floor height in base_link."""
        return -self.axle_height_m


def inside_convex(points_xy, polygon):
    """Points inside (or on) a convex polygon given as [(x, y), ...] in either orientation."""
    poly = np.asarray(polygon, dtype=float).reshape(-1, 2)
    pts = np.asarray(points_xy, dtype=float).reshape(-1, 2)
    if len(poly) < 3 or not len(pts):
        return np.zeros(len(pts), dtype=bool)
    edges = np.roll(poly, -1, axis=0) - poly
    rel = pts[:, None, :] - poly[None, :, :]
    cross = edges[None, :, 0] * rel[:, :, 1] - edges[None, :, 1] * rel[:, :, 0]
    return (cross >= -1e-12).all(axis=1) | (cross <= 1e-12).all(axis=1)


def grow_convex(polygon, distance):
    """Convex outline grown outward by ``distance`` (vertex bisector offset, like nav2_config.offset_polygon)."""
    poly = [tuple(map(float, p)) for p in polygon]
    if not distance or not poly:
        return poly
    n = len(poly)
    area = sum(poly[k][0] * poly[(k + 1) % n][1] - poly[(k + 1) % n][0] * poly[k][1] for k in range(n))
    sign = 1.0 if area > 0 else -1.0
    out = []
    for k in range(n):
        prev, cur, nxt = poly[k - 1], poly[k], poly[(k + 1) % n]
        normals = []
        for a, b in ((prev, cur), (cur, nxt)):
            dx, dy = b[0] - a[0], b[1] - a[1]
            length = math.hypot(dx, dy)
            normals.append((sign * dy / length, -sign * dx / length))
        nx, ny = normals[0][0] + normals[1][0], normals[0][1] + normals[1][1]
        scale = distance / max(1e-9, 1.0 + normals[0][0] * normals[1][0] + normals[0][1] * normals[1][1])
        out.append((cur[0] + nx * scale, cur[1] + ny * scale))
    return out


def body_outline(footprint=(), radius_m=None):
    """Body region for self-filtering: the convex outline, or a 32-gon for an assumed circle."""
    if footprint:
        return [tuple(map(float, p)) for p in footprint]
    if radius_m is None:
        return []
    return [(radius_m * math.cos(a), radius_m * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 32, endpoint=False)]


def _voxel_keys(points, size):
    return np.floor(points / size).astype(np.int64)


def unsupported_clusters(base, h, keep, horizontal, lever_z, cfg):
    """Clusters of kept points with nothing in the ``floating_min_gap_m`` beneath them (inside the view).

    Returns a list of (member indices into ``base``, points, horizontal extent) for
    clusters of at most ``unsupported_max_points`` points.
    """
    from scipy import ndimage
    from scipy.spatial import cKDTree
    idx = np.flatnonzero(keep)
    if not len(idx):
        return []
    keys = _voxel_keys(base[idx], cfg.cluster_voxel_m)
    lo = keys.min(axis=0)
    shape = keys.max(axis=0) - lo + 1
    if np.prod(shape) > 4_000_000:
        return []
    grid = np.zeros(tuple(shape), dtype=bool)
    k = keys - lo
    grid[k[:, 0], k[:, 1], k[:, 2]] = True
    labels, count = ndimage.label(grid, structure=np.ones((3, 3, 3), dtype=bool))
    point_label = labels[k[:, 0], k[:, 1], k[:, 2]]
    sizes = np.bincount(point_label, minlength=count + 1)
    candidates = np.flatnonzero((sizes > 0) & (sizes <= cfg.unsupported_max_points))
    candidates = candidates[candidates > 0]
    if not len(candidates):
        return []
    lidar_h = lever_z + cfg.axle_height_m
    tan_low = math.tan(math.radians(cfg.lowest_ray_deg))
    tree = cKDTree(base[:, :2])
    out = []
    for lab in candidates:
        members = idx[point_label == lab]
        xy = base[members, :2]
        bottom = float(h[members].min())
        # the lowest height the lidar can see at this range: below it, absence of returns says nothing
        visible_floor = max(0.0, lidar_h - float(horizontal[members].min()) * tan_low)
        if bottom - visible_floor < cfg.floating_min_gap_m:
            continue
        near = set()
        for found in tree.query_ball_point(xy, 0.10):
            near.update(found)
        near = np.fromiter(near, dtype=np.int64)
        below = near[(h[near] < bottom - 0.02) & (h[near] >= bottom - cfg.floating_min_gap_m)]
        if len(below):
            continue
        out.append((members, len(members), float(max(np.ptp(xy[:, 0]), np.ptp(xy[:, 1])))))
    return out


def all_clusters(base, keep, voxel_m):
    """Every 26-connected cluster of the kept points on ``voxel_m`` voxels: a list of index arrays into ``base``.

    Same voxels and connectivity as ``unsupported_clusters``; empty when the grid would be implausibly large.
    """
    from scipy import ndimage
    idx = np.flatnonzero(keep)
    if not len(idx):
        return []
    keys = _voxel_keys(base[idx], voxel_m)
    k = keys - keys.min(axis=0)
    shape = k.max(axis=0) + 1
    if np.prod(shape) > 4_000_000:
        return []
    grid = np.zeros(tuple(shape), dtype=bool)
    grid[k[:, 0], k[:, 1], k[:, 2]] = True
    labels, _ = ndimage.label(grid, structure=np.ones((3, 3, 3), dtype=bool))
    point_label = labels[k[:, 0], k[:, 1], k[:, 2]]
    order = np.argsort(point_label, kind="stable")
    bounds = np.flatnonzero(np.r_[True, point_label[order][1:] != point_label[order][:-1], True])
    return [idx[order[bounds[i]:bounds[i + 1]]] for i in range(len(bounds) - 1)]


def clearing_rays(points, lever, exclude, cfg=ObstacleFilter()):
    """Free-space rays of one frame for costmap clearing, as end points in the lidar frame (float32 Kx3).

    The nearest return in each ``clear_bin_deg`` direction bin (within
    ``clear_max_range_m``, artefact points in ``exclude`` left out) defines a ray that
    stops ``max(clear_stop_short_m, clear_stop_fraction * range)`` before it and,
    when it descends, no lower than ``clear_min_end_height_m`` above the floor. So
    clearing never reaches the surface that returned the light, nor the lowest layer
    above the floor, where grazing rays would erase low objects they barely missed.
    """
    p = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    r = np.linalg.norm(p, axis=1)
    ok = np.isfinite(r) & (r > 0.3) & (r <= cfg.clear_max_range_m)
    if exclude is not None:
        ok &= ~np.asarray(exclude, dtype=bool)
    q, rr = p[ok], r[ok]
    if not len(q):
        return np.zeros((0, 3), dtype=np.float32)
    az = np.degrees(np.arctan2(q[:, 1], q[:, 0]))
    el = np.degrees(np.arcsin(np.clip(q[:, 2] / rr, -1.0, 1.0)))
    key = np.floor(az / cfg.clear_bin_deg).astype(np.int64) * 100_003 + np.floor(el / cfg.clear_bin_deg).astype(np.int64)
    order = np.lexsort((rr, key))
    first = np.r_[True, key[order][1:] != key[order][:-1]]
    sel = order[first]
    unit = q[sel] / rr[sel, None]
    length = rr[sel] - np.maximum(cfg.clear_stop_short_m, cfg.clear_stop_fraction * rr[sel])
    lowest_z = cfg.clear_min_end_height_m - cfg.axle_height_m - float(np.asarray(lever)[2])   # lidar frame
    with np.errstate(divide="ignore", invalid="ignore"):
        limit = np.where(unit[:, 2] < -1e-6, lowest_z / unit[:, 2], np.inf)
    length = np.minimum(length, limit)
    keep = length > 0.3
    return (unit[keep] * length[keep, None]).astype(np.float32)


def filter_points(points, lever, cfg=ObstacleFilter(), body=(), witness=None, clearing=False):
    """Obstacle points of one frame: (voxels, stats) and, with ``clearing``, the clearing rays too.

    See ``filter_frame`` for the arguments.
    """
    result = filter_frame(points, lever, cfg, body, witness, clearing=clearing)
    if clearing:
        return result["points"], result["stats"], result["rays"]
    return result["points"], result["stats"]


def scan_ranges(points, cfg=ObstacleFilter()):
    """Nearest horizontal range per bin of the shadow runs' 161-bin scan (inf where empty), float32."""
    p = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    horizontal = np.hypot(p[:, 0], p[:, 1])
    azimuth = np.arctan2(p[:, 1], p[:, 0])
    ok = ((horizontal >= SCAN_RANGE_M[0]) & (horizontal <= min(SCAN_RANGE_M[1], cfg.scan_max_range_m))
          & (np.abs(azimuth) <= -SCAN_ANGLE_MIN + 1e-12))
    index = np.floor((azimuth[ok] - SCAN_ANGLE_MIN) / SCAN_INCREMENT + 0.5).astype(np.int64)
    ranges = np.full(SCAN_BINS, np.inf, dtype=np.float32)
    np.minimum.at(ranges, np.clip(index, 0, SCAN_BINS - 1), horizontal[ok].astype(np.float32))
    return ranges


def filter_frame(points, lever, cfg=ObstacleFilter(), body=(), witness=None, *, clearing=False, scan=False,
                 recent=None):
    """Obstacle products of one frame.

    ``points``: Nx3 prepared cloud (lidar origin, axle-aligned axes); ``lever``: the
    lidar origin in base_link; ``body``: convex outline [(x, y), ...] in base_link
    (empty: no body exclusion); ``witness``: optional ``consensus.Witness`` built
    from the other lidar's nearest frame in this frame's base_link. Unsupported
    clusters the witness saw through are dropped (stage ``contradicted``); clusters
    it also hit are kept whatever their size; without a witness, or where it saw
    nothing, the size rule of stage ``floating`` applies. ``recent``: optional
    ``consensus.Recent`` of the same lidar's earlier candidates for stage
    ``transient`` (only with ``persist_min_age_s`` > 0; None gives no verdict).

    Returns a dict: ``points`` (kept points within ``max_range_m`` in the lidar
    frame, float32 Mx3 after voxel reduction), ``stats`` (per-stage removal counts),
    with ``clearing`` also ``rays`` (``clearing_rays`` without the points dropped
    as floating, contradicted or transient) and with ``scan`` also ``scan``
    (``scan_ranges`` of all kept points up to ``scan_max_range_m``, before voxel
    reduction). With ``persist_min_age_s`` > 0 also ``candidates``: this frame's
    points that reached stage ``transient``, one per ``cluster_voxel_m`` voxel in
    base_link, for the next frames' ``recent``.
    """
    p = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    lever = np.asarray(lever, dtype=np.float64).reshape(3)
    if not np.isfinite(lever).all() or np.linalg.norm(lever) > 3.0:
        raise ValueError("finite lever arm within 3 m required")
    finite = np.isfinite(p).all(axis=1)
    p = p[finite]
    base = p + lever
    h = base[:, 2] + cfg.axle_height_m
    horizontal = np.hypot(p[:, 0], p[:, 1])
    stats = {"input_points": int(finite.size), "finite_points": int(len(p))}
    keep = np.ones(len(p), dtype=bool)

    def drop(name, mask):
        removed = keep & mask
        stats["removed_" + name] = int(removed.sum())
        keep[removed] = False

    drop("height", (h < cfg.min_height_m) | (h > cfg.max_height_m))
    drop("range", horizontal > (max(cfg.max_range_m, cfg.scan_max_range_m) if scan else cfg.max_range_m))
    drop("far_low", (h < cfg.low_height_m) & (horizontal > cfg.low_max_range_m))
    outline = grow_convex(body, cfg.body_margin_m) if body else []
    if outline:
        drop("body", inside_convex(base[:, :2], outline) & (h <= cfg.body_max_height_m))
    else:
        stats["removed_body"] = 0
    contradicted = np.zeros(len(p), dtype=bool)
    floating = np.zeros(len(p), dtype=bool)
    clusters = unsupported_clusters(base, h, keep, horizontal, float(lever[2]), cfg)
    confirmed = 0
    for members, size, extent in clusters:
        if witness is not None:
            through, hit = witness.shares(base[members])
            if through >= cfg.witness_min_share:
                contradicted[members] = True
                continue
            if hit >= WITNESS_CONFIRM_SHARE:
                confirmed += 1
                continue
        if size <= cfg.floating_max_points and extent <= cfg.floating_max_extent_m:
            floating[members] = True
    stats["unsupported_clusters"] = len(clusters)
    stats["unsupported_confirmed_by_witness"] = confirmed
    drop("contradicted", contradicted)
    drop("floating", floating)
    seen_through = np.zeros(len(p), dtype=bool)
    if cfg.witness_scope == "all":
        judged = 0
        if witness is not None:
            from scipy.spatial import cKDTree
            # returns just above the floor, from any stage (also those far_low dropped): what stands on the floor
            low = np.flatnonzero((h >= cfg.min_height_m) & (h <= cfg.seen_through_floor_m))
            floor_tree = cKDTree(base[low, :2]) if len(low) else None
            for members in all_clusters(base, keep, cfg.cluster_voxel_m):
                if len(members) > cfg.unsupported_max_points:
                    continue                    # large structure: never judged (as in stage 5)
                if floor_tree is not None and any(floor_tree.query_ball_point(base[members, :2], cfg.cluster_voxel_m)):
                    continue                    # stands on the floor: people, furniture, door leaves
                judged += 1
                if witness.seen_through(base[members], cfg.witness_min_share, WITNESS_CONFIRM_SHARE):
                    seen_through[members] = True
        stats["clusters_judged_by_witness"] = judged
        drop("seen_through", seen_through)
    result = {}
    transient = np.zeros(len(p), dtype=bool)
    if cfg.persist_min_age_s > 0:
        candidates = np.flatnonzero(keep)
        if len(candidates):
            keys = np.unique(_voxel_keys(base[candidates], cfg.cluster_voxel_m), axis=0)
            result["candidates"] = ((keys + 0.5) * cfg.cluster_voxel_m).astype(np.float32)
        else:
            result["candidates"] = np.zeros((0, 3), dtype=np.float32)
        judged = 0
        if recent is not None:
            for members, size, extent in clusters:
                if contradicted[members[0]] or floating[members[0]]:
                    continue
                judged += 1
                if recent.share(base[members]) < cfg.persist_min_share:
                    transient[members] = True
        stats["unsupported_judged_by_persistence"] = judged
    drop("transient", transient)
    if scan:
        result["scan"] = scan_ranges(p[keep], cfg)
        stats["scan_bins"] = int(np.isfinite(result["scan"]).sum())
        beyond = keep & (horizontal > cfg.max_range_m)
        stats["beyond_cloud_range"] = int(beyond.sum())
        keep &= ~beyond                     # the cloud itself stays within max_range_m
    kept = np.flatnonzero(keep)
    if len(kept):
        keys = _voxel_keys(base[kept], cfg.voxel_m)
        _, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
        inverse = inverse.reshape(-1)
        if cfg.min_voxel_points > 1:
            sparse = counts[inverse] < cfg.min_voxel_points
            mask = np.zeros(len(p), dtype=bool)
            mask[kept[sparse]] = True
            drop("sparse", mask)
            kept = kept[~sparse]
            keys = _voxel_keys(base[kept], cfg.voxel_m)
            _, inverse = np.unique(keys, axis=0, return_inverse=True)
            inverse = inverse.reshape(-1)
        else:
            stats["removed_sparse"] = 0
        sums = np.zeros((inverse.max() + 1 if len(inverse) else 0, 3))
        np.add.at(sums, inverse, p[kept])
        n = np.bincount(inverse, minlength=len(sums))[:, None]
        out = (sums / np.maximum(n, 1)).astype(np.float32)
    else:
        stats["removed_sparse"] = 0
        out = np.zeros((0, 3), dtype=np.float32)
    stats["kept_points"] = int(len(kept))
    stats["voxels"] = int(len(out))
    result.update(points=out, stats=stats)
    if clearing:
        result["rays"] = clearing_rays(p, lever, contradicted | floating | seen_through | transient, cfg)
        stats["clearing_rays"] = int(len(result["rays"]))
    return result


def heights_above_floor(points, lever, cfg=ObstacleFilter()):
    """Heights above the floor of prepared-cloud points (for audits)."""
    return np.asarray(points, dtype=float).reshape(-1, 3)[:, 2] + float(np.asarray(lever)[2]) + cfg.axle_height_m
