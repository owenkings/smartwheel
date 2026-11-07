"""Wheelchair body for collision truth and clearance: the assumed circle or a convex outline.

Pure geometry, no ROS imports. Coordinates are base_link (axle midpoint origin,
x forward, y left, metres), the same convention as ``NavProfile.footprint``.
The simulator uses the body as collision truth, the reports measure the gap
between the body (not the axle point) and obstacles, and the planner bench
measures path clearance with it.

Grid cells are closed squares. A body that only touches a cell edge does not
hit it, matching the circle check (``clearance < radius`` is a hit).
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

CIRCLE_SOURCE = "CIRCLE_ASSUMED_NOT_MEASURED"
_EPS = 1e-9


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def counter_clockwise(points):
    """(N, 2) float copy of an outline in counter-clockwise order."""
    pts = np.array(points, dtype=float).reshape(-1, 2)
    area = np.sum(pts[:, 0] * np.roll(pts[:, 1], -1) - np.roll(pts[:, 0], -1) * pts[:, 1])
    return pts if area > 0 else pts[::-1].copy()


def _edge_normals(polygon):
    """Outward unit normals and offsets of a CCW convex polygon: inside is ``n . p <= c`` for every edge."""
    d = np.roll(polygon, -1, axis=0) - polygon
    length = np.hypot(d[:, 0], d[:, 1])
    if np.any(length < 1e-12):
        raise ValueError("outline has a repeated vertex")
    normals = np.column_stack([d[:, 1], -d[:, 0]]) / length[:, None]
    return normals, np.einsum("ij,ij->i", normals, polygon)


def grow_outline(polygon, distance):
    """Mitered outward offset of a CCW convex polygon: every edge moves out by ``distance``."""
    polygon = np.asarray(polygon, dtype=float)
    if distance == 0.0:
        return polygon.copy()
    normals, _ = _edge_normals(polygon)
    previous = np.roll(normals, 1, axis=0)                 # normal of the edge ending at each vertex
    bisector = normals + previous
    scale = distance / np.maximum(1e-9, 1.0 + np.einsum("ij,ij->i", normals, previous))
    return polygon + bisector * scale[:, None]


def point_polygon_distance(points, polygon):
    """Distance from each point to a CCW convex polygon; 0 inside or on the boundary."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    normals, offsets = _edge_normals(polygon)
    inside = np.all(points @ normals.T - offsets <= _EPS, axis=1)
    a = polygon[None, :, :]
    ab = (np.roll(polygon, -1, axis=0) - polygon)[None, :, :]
    p = points[:, None, :]
    t = np.clip(np.sum((p - a) * ab, axis=-1) / np.maximum(np.sum(ab * ab, axis=-1), 1e-12), 0.0, 1.0)
    distance = np.linalg.norm(p - (a + t[..., None] * ab), axis=-1).min(axis=1)
    return np.where(inside, 0.0, distance)


def polygon_hits_cells(polygon, solid, resolution, origin_xy):
    """Whether a CCW convex polygon (world frame) overlaps a solid cell; outside the grid counts as solid.

    Exact separating-axis test of the polygon against every cell square in its
    bounding box (axes: x, y and the polygon's edge normals).
    """
    polygon = np.asarray(polygon, dtype=float)
    res = float(resolution)
    height, width = solid.shape
    gx = (polygon[:, 0] - origin_xy[0]) / res
    gy = (polygon[:, 1] - origin_xy[1]) / res
    if gx.min() < -_EPS or gy.min() < -_EPS or gx.max() > width + _EPS or gy.max() > height + _EPS:
        return True
    c0, c1 = max(0, int(math.floor(gx.min()))), min(width - 1, int(math.floor(gx.max())))
    r0, r1 = max(0, int(math.floor(gy.min()))), min(height - 1, int(math.floor(gy.max())))
    block = solid[r0:r1 + 1, c0:c1 + 1]
    if not block.any():
        return False
    rows, cols = np.nonzero(block)
    h = 0.5 * res
    cx = origin_xy[0] + (cols + c0 + 0.5) * res
    cy = origin_xy[1] + (rows + r0 + 0.5) * res
    overlap = ((cx - h < polygon[:, 0].max() - _EPS) & (cx + h > polygon[:, 0].min() + _EPS)
               & (cy - h < polygon[:, 1].max() - _EPS) & (cy + h > polygon[:, 1].min() + _EPS))
    if not overlap.any():
        return False
    cx, cy = cx[overlap], cy[overlap]
    normals, offsets = _edge_normals(polygon)
    reach = h * (np.abs(normals[:, 0]) + np.abs(normals[:, 1]))
    nearest = cx[:, None] * normals[None, :, 0] + cy[:, None] * normals[None, :, 1] - reach[None, :]
    return bool(np.any(np.all(nearest < offsets[None, :] - _EPS, axis=1)))


def _solid_cells_near(solid, resolution, origin_xy, lo, hi):
    """Centres of solid cells whose squares may lie within the box [lo, hi] (world x, y)."""
    res = float(resolution)
    height, width = solid.shape
    c0 = max(0, int(math.floor((lo[0] - origin_xy[0]) / res)))
    c1 = min(width - 1, int(math.floor((hi[0] - origin_xy[0]) / res)))
    r0 = max(0, int(math.floor((lo[1] - origin_xy[1]) / res)))
    r1 = min(height - 1, int(math.floor((hi[1] - origin_xy[1]) / res)))
    if c0 > c1 or r0 > r1:
        return np.zeros(0), np.zeros(0)
    rows, cols = np.nonzero(solid[r0:r1 + 1, c0:c1 + 1])
    return origin_xy[0] + (cols + c0 + 0.5) * res, origin_xy[1] + (rows + r0 + 0.5) * res


def _point_square_distance(points, cx, cy, half):
    """(P, M) distances from points to axis-aligned squares (centres cx, cy, half size ``half``)."""
    dx = np.maximum(np.abs(points[:, None, 0] - cx[None, :]) - half, 0.0)
    dy = np.maximum(np.abs(points[:, None, 1] - cy[None, :]) - half, 0.0)
    return np.hypot(dx, dy)


def point_cell_distance(point, solid, resolution, origin_xy, max_distance):
    """Exact distance from a point to the nearest solid cell square, capped at ``max_distance``."""
    p = np.asarray(point, dtype=float).reshape(1, 2)
    cx, cy = _solid_cells_near(solid, resolution, origin_xy, p[0] - max_distance, p[0] + max_distance)
    if not len(cx):
        return float(max_distance)
    return float(min(max_distance, _point_square_distance(p, cx, cy, 0.5 * resolution).min()))


def polygon_cell_distance(polygon, solid, resolution, origin_xy, max_distance):
    """Exact distance from a convex polygon to the nearest solid cell square, capped at ``max_distance``.

    Valid for a polygon that overlaps no solid cell (check ``polygon_hits_cells``
    first): two disjoint convex shapes are closest at a vertex of one of them.
    """
    polygon = np.asarray(polygon, dtype=float)
    cx, cy = _solid_cells_near(solid, resolution, origin_xy, polygon.min(axis=0) - max_distance,
                               polygon.max(axis=0) + max_distance)
    if not len(cx):
        return float(max_distance)
    h = 0.5 * resolution
    to_squares = _point_square_distance(polygon, cx, cy, h).min()
    corners = np.vstack([np.column_stack([cx + sx * h, cy + sy * h]) for sx in (-1, 1) for sy in (-1, 1)])
    to_polygon = point_polygon_distance(corners, polygon).min()
    return float(min(max_distance, to_squares, to_polygon))


@dataclass(frozen=True)
class Body:
    """The assumed circle (``radius_m``) or, when ``outline`` is given, a convex outline."""
    radius_m: float = 0.55
    outline: tuple = ()
    source: str = CIRCLE_SOURCE

    def __post_init__(self):
        radius = float(self.radius_m)
        if not math.isfinite(radius) or radius <= 0:
            raise ValueError("body radius must be a positive finite number")
        object.__setattr__(self, "radius_m", radius)
        if self.outline:
            points = counter_clockwise(self.outline)
            if len(points) < 3 or not np.isfinite(points).all():
                raise ValueError("body outline needs at least three finite points")
            _, offsets = _edge_normals(points)
            e = np.roll(points, -1, axis=0) - points
            f = np.roll(e, -1, axis=0)
            turns = e[:, 0] * f[:, 1] - e[:, 1] * f[:, 0]
            if np.any(turns <= 0) or np.any(offsets <= 0):
                raise ValueError("body outline must be convex with base_link strictly inside")
            object.__setattr__(self, "outline", tuple((float(x), float(y)) for x, y in points))
        object.__setattr__(self, "source", str(self.source))

    @classmethod
    def from_profile(cls, profile, source=None):
        """Body of a NavProfile or its ``as_dict()``; the outline replaces the circle."""
        values = profile if isinstance(profile, dict) else profile.as_dict()
        outline = tuple(tuple(p) for p in (values.get("footprint") or ()))
        if source is None:
            source = "POLYGON" if outline else CIRCLE_SOURCE
        return cls(radius_m=values.get("radius_m", 0.55), outline=outline, source=source)

    @classmethod
    def from_config(cls, value):
        """``{"type": "circle", "radius_m": r}`` or ``{"type": "polygon", "points": [[x, y], ...]}``."""
        kind = value.get("type")
        if kind == "circle":
            return cls(radius_m=value["radius_m"], source=value.get("source", CIRCLE_SOURCE))
        if kind == "polygon":
            return cls(radius_m=value.get("radius_m", 0.55), outline=tuple(tuple(p) for p in value["points"]),
                       source=value.get("source", "POLYGON"))
        raise ValueError("collision body type must be circle or polygon")

    def as_config(self):
        if self.is_polygon:
            return {"type": "polygon", "points": [list(p) for p in self.outline], "radius_m": self.radius_m,
                    "source": self.source}
        return {"type": "circle", "radius_m": self.radius_m, "source": self.source}

    @property
    def is_polygon(self):
        return bool(self.outline)

    @property
    def circumscribed_m(self):
        """Farthest body point from the axle: what an in-place turn sweeps."""
        if not self.is_polygon:
            return self.radius_m
        return max(math.hypot(x, y) for x, y in self.outline)

    @property
    def inscribed_m(self):
        """Distance from the axle to the nearest body edge."""
        if not self.is_polygon:
            return self.radius_m
        _, offsets = _edge_normals(np.asarray(self.outline))
        return float(offsets.min())

    def outline_at(self, pose, grow=0.0):
        """World-frame CCW outline at ``pose`` (x, y, yaw), optionally grown outward by ``grow``."""
        if not self.is_polygon:
            raise ValueError("a circular body has no outline")
        local = np.asarray(self.outline, dtype=float)
        if grow:
            local = grow_outline(local, grow)
        x, y, yaw = (float(v) for v in pose)
        c, s = math.cos(yaw), math.sin(yaw)
        return np.column_stack([x + c * local[:, 0] - s * local[:, 1], y + s * local[:, 0] + c * local[:, 1]])

    def swept_samples(self, start, end, resolution):
        """Intermediate poses from ``start`` to ``end`` and the growth that covers the motion between them.

        No body point moves more than half a cell between samples; growing the
        body by half the largest step makes the samples cover the swept area.
        """
        x0, y0, a0 = (float(v) for v in start)
        x1, y1, a1 = (float(v) for v in end)
        turn = wrap(a1 - a0)
        reach = math.hypot(x1 - x0, y1 - y0) + abs(turn) * self.circumscribed_m
        count = max(1, int(math.ceil(reach / (resolution * 0.5))))
        grow = reach / count * 0.5
        poses = [(x0 + k / count * (x1 - x0), y0 + k / count * (y1 - y0), a0 + k / count * turn)
                 for k in range(count + 1)]
        return poses, grow
