"""Frontier-based exploration goal selection on an occupancy grid.

A frontier cell is known free and 4-adjacent to unknown space. Goals are placed
in known free space that keeps the stated circular clearance from occupied and
unknown cells, and must be reachable through that space from the robot. The
robot's own disc is treated as free because the body occupies it. Unknown
space is never relabelled free beyond that disc.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

from ..geometry import Grid, inflate_grid, _number

_EIGHT = np.ones((3, 3), dtype=bool)


def trinary(data, occupied_threshold=65, free_threshold=25):
    """Convert ROS OccupancyGrid values to -1 / 0 / 100."""
    values = np.asarray(data, dtype=np.int16)
    out = np.full(values.shape, -1, dtype=np.int8)
    out[(values >= 0) & (values <= free_threshold)] = 0
    out[values >= occupied_threshold] = 100
    return out


@dataclass(frozen=True)
class Frontier:
    index: int
    cells: np.ndarray
    centroid_xy: tuple
    length_m: float


@dataclass(frozen=True)
class GoalCandidate:
    frontier_index: int
    goal_xy: tuple
    goal_yaw: float
    path_distance_m: float
    gain_m: float
    score: float
    frontier_centroid_xy: tuple


def with_robot_disc_free(grid, robot_xy, radius_m):
    """Copy of the grid with unknown cells under the robot disc marked free."""
    data = np.array(grid.data, copy=True)
    res = grid.resolution
    rows, cols = np.indices(data.shape)
    cx = grid.origin[0] + (cols + 0.5) * res
    cy = grid.origin[1] + (rows + 0.5) * res
    inside = np.hypot(cx - robot_xy[0], cy - robot_xy[1]) <= radius_m
    data[inside & (data < 0)] = 0
    return Grid(data, res, grid.origin)


def find_frontiers(grid, min_length_m=0.5):
    """Return frontier clusters (8-connected), largest first."""
    data = np.asarray(grid.data)
    free = data == 0
    unknown = data < 0
    padded = np.pad(unknown, 1, mode="constant", constant_values=False)
    near_unknown = (padded[:-2, 1:-1] | padded[2:, 1:-1] | padded[1:-1, :-2] | padded[1:-1, 2:])
    frontier = free & near_unknown
    labels, count = ndimage.label(frontier, structure=_EIGHT)
    result = []
    res = grid.resolution
    if not count:
        return []
    all_rows, all_cols = np.nonzero(labels)
    order = np.argsort(labels[all_rows, all_cols], kind="stable")
    all_rows, all_cols = all_rows[order], all_cols[order]
    splits = np.flatnonzero(np.diff(labels[all_rows, all_cols])) + 1
    for rows, cols in zip(np.split(all_rows, splits), np.split(all_cols, splits)):
        length = len(rows) * res
        if length + 1e-9 < min_length_m:
            continue
        cx = grid.origin[0] + (float(cols.mean()) + 0.5) * res
        cy = grid.origin[1] + (float(rows.mean()) + 0.5) * res
        result.append((np.column_stack([cols, rows]), (cx, cy), length))
    result.sort(key=lambda item: -item[2])
    return [Frontier(i, cells, centroid, length) for i, (cells, centroid, length) in enumerate(result)]


def reachable_distances(free, start_cell, resolution):
    """Shortest 8-connected path length (m) through free cells; inf if unreachable.

    Diagonal moves require both side-adjacent cells to be free, matching the
    corner rule of the offline geometry tools.
    """
    free = np.asarray(free, dtype=bool)
    height, width = free.shape
    col, row = start_cell
    distances = np.full(free.shape, np.inf)
    if not (0 <= col < width and 0 <= row < height) or not free[row, col]:
        return distances
    ids = -np.ones(free.shape, dtype=np.int64)
    ids[free] = np.arange(int(free.sum()))
    src, dst, weight = [], [], []
    for dx, dy in ((1, 0), (0, 1), (1, 1), (1, -1)):
        r0, r1 = max(0, -dy), height - max(0, dy)
        c0, c1 = max(0, -dx), width - max(0, dx)
        a = free[r0:r1, c0:c1]
        b = free[r0 + dy:r1 + dy, c0 + dx:c1 + dx]
        ok = a & b
        if dx and dy:
            ok &= free[r0:r1, c0 + dx:c1 + dx] & free[r0 + dy:r1 + dy, c0:c1]
        ia = ids[r0:r1, c0:c1][ok]
        ib = ids[r0 + dy:r1 + dy, c0 + dx:c1 + dx][ok]
        step = resolution * (math.sqrt(2.0) if dx and dy else 1.0)
        src.extend([ia, ib])
        dst.extend([ib, ia])
        weight.extend([np.full(len(ia), step), np.full(len(ia), step)])
    n = int(free.sum())
    graph = coo_matrix((np.concatenate(weight), (np.concatenate(src), np.concatenate(dst))), shape=(n, n)).tocsr()
    values = dijkstra(graph, directed=True, indices=int(ids[row, col]))
    distances[free] = values
    return distances


def _in_blacklist(xy, blacklist):
    return any(math.hypot(xy[0] - bx, xy[1] - by) <= br for bx, by, br in blacklist)


def select_goal(grid, robot_xy, *, radius_m, margin_m, min_frontier_length_m=0.5,
                max_goal_offset_m=3.0, min_goal_distance_m=0.4, gain_weight=1.0, blacklist=(),
                goal_extra_clearance_m=0.15):
    """Choose the best reachable frontier goal; returns (candidate|None, report).

    Goals prefer cells with ``goal_extra_clearance_m`` beyond the planning
    clearance: the map keeps changing during exploration, and a robot parked at
    exactly the lethal radius can end up inside the inflated costmap, where no
    forward-only plan can leave. Cells at the bare clearance are a fallback.
    """
    radius = _number(radius_m, "radius", positive=True)
    margin = _number(margin_m, "margin", nonnegative=True)
    extra = _number(goal_extra_clearance_m, "goal extra clearance", nonnegative=True)
    planning = with_robot_disc_free(grid, robot_xy, radius + margin + grid.resolution)
    frontiers = find_frontiers(planning, min_frontier_length_m)
    report = {"frontiers": len(frontiers), "frontier_lengths_m": [round(f.length_m, 3) for f in frontiers[:20]],
              "skipped_blacklisted": 0, "skipped_unreachable": 0, "skipped_too_close": 0, "candidates": 0}
    if not frontiers:
        return None, report
    inflated = inflate_grid(planning, radius, margin)
    free = inflated.free
    try:
        start = planning.world_to_cell(robot_xy)
    except ValueError:
        report["robot_outside_map"] = True
        return None, report
    if not free[start[1], start[0]]:
        # The disc was marked free, so an occupied cell (or the map edge) is
        # within the clearance. Search from the nearest clear cell nearby; the
        # Nav2 planner still plans from the true pose.
        rows, cols = np.nonzero(free)
        if not len(rows):
            report["robot_cell_blocked"] = True
            return None, report
        offsets = np.hypot(cols - start[0], rows - start[1]) * planning.resolution
        k = int(np.argmin(offsets))
        if offsets[k] > 0.6:
            report["robot_cell_blocked"] = True
            return None, report
        start = (int(cols[k]), int(rows[k]))
        report["start_cell_snapped_m"] = round(float(offsets[k]), 3)
    distances = reachable_distances(free, start, planning.resolution)
    # A frontier is only worth approaching if it lies in the robot's region of
    # known free space; a frontier seen behind a wall cannot be resolved here.
    raw_labels, _ = ndimage.label(np.asarray(planning.data) == 0, structure=_EIGHT)
    robot_cell = planning.world_to_cell(robot_xy)
    robot_label = raw_labels[robot_cell[1], robot_cell[0]] or raw_labels[start[1], start[0]]
    reachable = np.isfinite(distances)
    roomy = reachable & inflate_grid(planning, radius, margin + extra).free if extra > 0 else reachable
    res = planning.resolution
    sets = []
    for mask in (roomy, reachable):
        rows, cols = np.nonzero(mask)
        sets.append((rows, cols, planning.origin[0] + (cols + 0.5) * res, planning.origin[1] + (rows + 0.5) * res))
    best = None
    report["fallback_goals_at_bare_clearance"] = 0
    for f in frontiers:
        if _in_blacklist(f.centroid_xy, blacklist):
            report["skipped_blacklisted"] += 1
            continue
        if robot_label == 0 or not (raw_labels[f.cells[:, 1], f.cells[:, 0]] == robot_label).any():
            report["skipped_unreachable"] += 1
            continue
        for level, (reach_rows, reach_cols, reach_x, reach_y) in enumerate(sets):
            offset = np.hypot(reach_x - f.centroid_xy[0], reach_y - f.centroid_xy[1])
            if len(offset) and float(offset.min()) <= max_goal_offset_m:
                break
        else:
            report["skipped_unreachable"] += 1
            continue
        report["fallback_goals_at_bare_clearance"] += level
        k = int(np.argmin(offset))
        goal = (float(reach_x[k]), float(reach_y[k]))
        if _in_blacklist(goal, blacklist):
            report["skipped_blacklisted"] += 1
            continue
        if math.hypot(goal[0] - robot_xy[0], goal[1] - robot_xy[1]) < min_goal_distance_m:
            report["skipped_too_close"] += 1
            continue
        path = float(distances[reach_rows[k], reach_cols[k]])
        yaw = math.atan2(f.centroid_xy[1] - goal[1], f.centroid_xy[0] - goal[0])
        score = gain_weight * f.length_m - path
        report["candidates"] += 1
        if best is None or score > best.score:
            best = GoalCandidate(f.index, goal, yaw, path, f.length_m, score, f.centroid_xy)
    return best, report
