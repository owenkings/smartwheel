"""Read-only observed-disc overlay and conservative grid-path diagnostics.

Unknown cells stay unknown. A NO_PATH result concerns the stated, discretized
model and connector set; it is not a claim that no physical route exists.
"""
from __future__ import annotations

import heapq
import math
from numbers import Integral

import numpy as np

from .geometry import Grid, InflatedGrid, _number, _point, _neighbors


MAX_GRID_CELLS = 2_000_000
MAX_TRACKS = 100
MAX_RASTER_UPDATES = 5_000_000
MAX_EXPANSIONS = 500_000
MAX_EDGE_CACHE = 1_000_000
MAX_SHORTCUT_CHECKS = 2_000


def overlay_observed_discs(grid, tracks):
    """Return a fresh Grid with intersected known-free cells made occupied.

    ``center_xy`` is the supplied observed world center, not a world schedule.
    ``id``, ``velocity_xy`` and ``stamp_s`` are optional observation metadata.
    No extrapolation, expiry, or persistence inference is performed here.
    """
    if not isinstance(grid, Grid) or grid.data.size > MAX_GRID_CELLS:
        raise ValueError('bounded Grid required')
    if not isinstance(tracks, (tuple, list)) or len(tracks) > MAX_TRACKS:
        raise ValueError('bounded explicit observed-track list required')
    parsed = []
    allowed = {'center_xy', 'radius_m', 'id', 'velocity_xy', 'stamp_s'}
    for track in tracks:
        if not isinstance(track, dict) or set(track) - allowed:
            raise ValueError('only current observation fields allowed; no future world schedule')
        center = _point(track['center_xy'], 'observed center')
        radius = _number(track['radius_m'], 'observed radius', positive=True)
        if max(map(abs, center)) > 1_000_000 or radius > 100:
            raise ValueError('observation metric resource bound')
        if 'velocity_xy' in track:
            velocity = _point(track['velocity_xy'], 'observed velocity')
            if math.hypot(*velocity) > 100:
                raise ValueError('velocity resource bound')
        if 'stamp_s' in track:
            _number(track['stamp_s'], 'observation stamp', nonnegative=True)
        if 'id' in track and (not isinstance(track['id'], str) or not track['id']):
            raise ValueError('observed id must be a nonempty string')
        parsed.append((center, radius))
    covered = np.zeros(grid.data.shape, dtype=bool)
    outside = partial = updates = 0
    for center, radius in parsed:
        cx, cy = grid.world_to_grid(center)
        rr = radius / grid.resolution
        if not math.isfinite(rr) or not all(math.isfinite(v) for v in [cx-rr, cx+rr, cy-rr, cy+rr]):
            raise ValueError('observation-to-grid overflow')
        # ceil(low)-1 includes the adjacent square when a circle is tangent to
        # an exact grid edge. The subsequent circle/AABB test removes corners.
        x0, x1 = math.ceil(cx-rr-1e-10)-1, math.floor(cx+rr+1e-10)
        y0, y1 = math.ceil(cy-rr-1e-10)-1, math.floor(cy+rr+1e-10)
        left, right = max(0, x0), min(grid.width-1, x1)
        bottom, top = max(0, y0), min(grid.height-1, y1)
        if left > right or bottom > top:
            outside += 1
            continue
        count = (right-left+1)*(top-bottom+1)
        updates += count
        if updates > MAX_RASTER_UPDATES:
            raise ValueError('observation raster update budget exceeded')
        cols = np.arange(left, right+1, dtype=float)
        rows = np.arange(bottom, top+1, dtype=float)
        dx = np.maximum(np.maximum(cols-cx, cx-(cols+1)), 0.)
        dy = np.maximum(np.maximum(rows-cy, cy-(rows+1)), 0.)
        # Tiny outward tolerance handles rigid-transform roundoff. It can only
        # add occupied cells and cannot create free space.
        intersect = np.hypot(dy[:, None], dx[None, :]) <= rr + 1e-10
        if not intersect.any():
            outside += 1
            continue
        partial += (cx-rr < -1e-10 or cx+rr > grid.width+1e-10
                    or cy-rr < -1e-10 or cy+rr > grid.height+1e-10)
        covered[bottom:top+1, left:right+1] |= intersect
    data = grid.data.copy()
    marked = covered & (data == 0)
    data[marked] = 100
    result = Grid(data, grid.resolution, grid.origin)
    return result, {
        'status': 'OBSERVED_OVERLAY_CREATED', 'tracks': len(parsed),
        'covered_cells': int(covered.sum()), 'free_to_occupied_cells': int(marked.sum()),
        'covered_unknown_cells_preserved': int(np.count_nonzero(covered & (grid.data == -1))),
        'covered_existing_occupied_cells': int(np.count_nonzero(covered & (grid.data == 100))),
        'fully_outside_tracks': outside, 'partially_clipped_tracks': int(partial),
        'raster_candidate_updates': updates, 'unknown_cells_preserved': True,
        'new_free_cells': 0, 'base_map_modified': False,
        'rasterization': 'DISC_INTERSECTS_CLOSED_CELL_SQUARE_WITH_OUTWARD_ROUNDOFF_TOLERANCE',
        'center_time_semantics': 'SUPPLIED_OBSERVATION_CENTER_NO_EXTRAPOLATION',
        'physical_navigation_validated': False,
    }


def find_path(inflated, start, goal, *, max_expansions=MAX_EXPANSIONS):
    """Search a bounded 8-connected conservative graph and certify its chords.

    Exact start/goal connect to checked cell centers in a two-cell neighborhood.
    Diagonal corner cutting is prohibited. A direct certified segment is used
    first; returned shortcuts are also checked with the same continuous sweep.
    """
    if (not isinstance(inflated, InflatedGrid) or inflated.grid.data.size > MAX_GRID_CELLS
            or inflated.blocked.shape != inflated.grid.data.shape or inflated.blocked.dtype.kind != 'b'
            or inflated.clearance_m.shape != inflated.grid.data.shape
            or not np.isfinite(inflated.clearance_m).all() or np.any(inflated.clearance_m < 0)):
        raise ValueError('bounded valid InflatedGrid required')
    radius = _number(inflated.radius_m, 'radius', positive=True)
    margin = _number(inflated.margin_m, 'margin', nonnegative=True)
    if not math.isfinite(radius+margin) or radius+margin > 100:
        raise ValueError('footprint resource bound')
    if (isinstance(max_expansions, (bool, np.bool_)) or not isinstance(max_expansions, Integral)
            or not 1 <= max_expansions <= MAX_EXPANSIONS):
        raise ValueError('bounded positive integer expansion budget required')
    start, goal = _point(start, 'start'), _point(goal, 'goal')
    grid = inflated.grid
    metadata = {
        'algorithm': 'EIGHT_CONNECTED_ASTAR_CONTINUOUS_EDGE_CHECKS',
        'connector_neighborhood_cells': 2, 'expanded_cells': 0, 'edge_checks': 0,
        'shortcut_checks': 0, 'radius_m': radius, 'margin_m': margin,
        'unknown_space': 'BLOCKED', 'map_boundary': 'BLOCKED',
        'no_path_meaning': 'NO_ROUTE_IN_THIS_CONSERVATIVE_DISCRETE_GRAPH_AND_CONNECTOR_SET_NOT_PHYSICAL_IMPOSSIBILITY',
        'footprint_relaxed': False, 'physical_navigation_validated': False,
    }

    def finish(status, reason, path=()):
        return {'status': status, 'reason': reason,
                'path': [list(p) for p in path], 'metadata': dict(metadata)}

    if not inflated.is_free_world(start):
        return finish('NO_PATH', 'START_NOT_CERTIFIED_FREE')
    if not inflated.is_free_world(goal):
        return finish('NO_PATH', 'GOAL_NOT_CERTIFIED_FREE')
    if math.dist(start, goal)/grid.resolution <= 500_000 and inflated.segment_is_free(start, goal):
        return finish('PATH', 'DIRECT_CONTINUOUS_SWEEP_CERTIFIED', [start] if start == goal else [start, goal])
    width, height = grid.width, grid.height
    free = inflated.free

    def connectors(point):
        col, row = grid.world_to_cell(point)
        result = {}
        for y in range(max(0, row-2), min(height, row+3)):
            for x in range(max(0, col-2), min(width, col+3)):
                if free[y, x]:
                    center = grid.cell_to_world((x, y))
                    if inflated.segment_is_free(point, center):
                        result[y*width+x] = math.dist(point, center)
        return result

    starts, goals = connectors(start), connectors(goal)
    metadata.update(start_connectors=len(starts), goal_connectors=len(goals))
    if not starts or not goals:
        return finish('NO_PATH', 'NO_CERTIFIED_CONTINUOUS_ENDPOINT_CONNECTOR')
    gscore = np.full(width*height, math.inf)
    parent = np.full(width*height, -1, dtype=np.int64)
    closed = np.zeros(width*height, dtype=bool)
    gx, gy = grid.world_to_grid(goal)

    def heuristic(node):
        return math.hypot(node % width + .5-gx, node // width + .5-gy)*grid.resolution

    pending = []
    for node, distance in starts.items():
        gscore[node] = distance; parent[node] = -2
        heapq.heappush(pending, (distance+heuristic(node), distance, node))
    edge_cache = {}
    found = None
    while pending:
        _, cost, node = heapq.heappop(pending)
        if closed[node] or cost != gscore[node]:
            continue
        if metadata['expanded_cells'] >= max_expansions:
            return finish('RESOURCE_LIMIT', 'EXPANSION_BUDGET_EXHAUSTED')
        closed[node] = True; metadata['expanded_cells'] += 1
        if node in goals:
            found = node
            break
        x, y = node % width, node // width
        for nx, ny in _neighbors(free, x, y):
            neighbor = ny*width+nx
            if closed[neighbor]:
                continue
            distance = math.hypot(nx-x, ny-y)*grid.resolution
            candidate = cost+distance
            if candidate >= gscore[neighbor]:
                continue
            edge = (min(node, neighbor), max(node, neighbor))
            safe = edge_cache.get(edge)
            if safe is None:
                # Distance-to-obstacle is 1-Lipschitz. This sufficient bound is
                # faster than sampling well-inside-free edges and equally safe.
                sufficient = min(inflated.clearance_m[y, x], inflated.clearance_m[ny, nx])
                if sufficient >= inflated.required_clearance_m+distance*.5+1e-12:
                    safe = True
                else:
                    if len(edge_cache) >= MAX_EDGE_CACHE:
                        return finish('RESOURCE_LIMIT', 'CONTINUOUS_EDGE_CACHE_BUDGET_EXHAUSTED')
                    metadata['edge_checks'] += 1
                    safe = inflated.segment_is_free(grid.cell_to_world((x,y)), grid.cell_to_world((nx,ny)))
                    edge_cache[edge] = safe
            if safe:
                gscore[neighbor] = candidate; parent[neighbor] = node
                heapq.heappush(pending, (candidate+heuristic(neighbor), candidate, neighbor))
    if found is None:
        return finish('NO_PATH', 'CONSERVATIVE_GRAPH_DISCONNECTED')
    nodes = []
    while found >= 0:
        nodes.append(found)
        if len(nodes) > 200_000:
            return finish('RESOURCE_LIMIT', 'PATH_POINT_BUDGET_EXHAUSTED')
        found = int(parent[found])
    raw = [start]+[grid.cell_to_world((n % width, n // width)) for n in reversed(nodes)]+[goal]
    raw = [p for i,p in enumerate(raw) if i == 0 or math.dist(p,raw[i-1]) > 1e-10]
    path = [raw[0]]; at = 0
    while at < len(raw)-1:
        chosen = at+1
        if metadata['shortcut_checks'] < MAX_SHORTCUT_CHECKS:
            for candidate in range(len(raw)-1, at+1, -1):
                if metadata['shortcut_checks'] >= MAX_SHORTCUT_CHECKS:
                    break
                if math.dist(raw[at],raw[candidate])/grid.resolution > 500_000:
                    continue
                metadata['shortcut_checks'] += 1
                if inflated.segment_is_free(raw[at],raw[candidate]):
                    chosen = candidate
                    break
        path.append(raw[chosen]); at = chosen
    if any(not inflated.segment_is_free(a,b) for a,b in zip(path[:-1],path[1:])):
        return finish('NO_PATH', 'FINAL_CONTINUOUS_VALIDATION_REJECTED')
    metadata.update(raw_path_points=len(raw), path_points=len(path),
                    path_length_m=sum(math.dist(a,b) for a,b in zip(path[:-1],path[1:])))
    return finish('PATH', 'GRID_ROUTE_AND_CONTINUOUS_SWEEPS_CERTIFIED', path)
