"""Read-only occupancy-grid diagnostics for offline reconstruction experiments.

Unknown space and map boundaries remain blocked. Metrics describe the supplied
map, points and hypothetical circular footprint; they never qualify real motion.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from .geometry import Grid, connected_components, demo_endpoints, inflate_grid


_MAX_POINTS = 2_000_000
_MAX_COMPARISON_CELLS = 2_000_000
_LABELS = ((-1, 'unknown'), (0, 'free'), (100, 'occupied'))


def _points(value, name):
    if value is None:
        return None
    try:
        array = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(name + ' must contain finite x/y points') from exc
    if array.size == 0:
        array = np.empty((0, 2), dtype=float)
    if (array.ndim != 2 or array.shape[1] != 2 or len(array) > _MAX_POINTS
            or not np.isfinite(array).all()):
        raise ValueError(name + ' must contain bounded finite x/y points')
    return array


def _cells(grid, points):
    """Return continuous cell coordinates, integer indices, and in-map mask."""
    if not len(points):
        return np.empty((0, 2)), np.empty((0, 2), dtype=np.int64), np.zeros(0, dtype=bool)
    ox, oy, yaw = grid.origin
    c, s = math.cos(yaw), math.sin(yaw)
    with np.errstate(over='ignore', invalid='ignore'):
        delta = points - np.asarray([ox, oy])
        coordinates = delta @ np.asarray([[c, -s], [s, c]]) / grid.resolution
    if not np.isfinite(coordinates).all():
        raise ValueError('point-to-grid coordinate overflow')
    inside = ((coordinates[:, 0] >= 0) & (coordinates[:, 0] < grid.width)
              & (coordinates[:, 1] >= 0) & (coordinates[:, 1] < grid.height))
    # Do not cast arbitrary finite outside coordinates to fixed-width integers.
    indices = np.zeros((len(points), 2), dtype=np.int64)
    indices[inside] = np.floor(coordinates[inside]).astype(np.int64)
    return coordinates, indices, inside


def _occupancy(grid, points):
    coordinates, indices, inside = _cells(grid, points)
    values = np.full(len(points), -128, dtype=np.int16)
    values[inside] = grid.data[indices[inside, 1], indices[inside, 0]]
    return values, inside


def _counts(values):
    return {name: int(np.count_nonzero(values == label)) for label, name in _LABELS}


def _point_occupancy(grid, points):
    values, inside = _occupancy(grid, points)
    n = len(points)
    counts = _counts(values)
    return {'points': n, 'in_map': int(inside.sum()), 'outside_map': int((~inside).sum()),
            **{name + '_points': count for name, count in counts.items()},
            **{name + '_fraction': count / n if n else None for name, count in counts.items()},
            'outside_fraction': int((~inside).sum()) / n if n else None,
            'classification_basis': 'INPUT_POINT_CELL_OCCUPANCY_NOT_PHYSICAL_TRUTH'}


def _wall_support(grid, points, threshold_m=0.10):
    from scipy.spatial import cKDTree
    result = _point_occupancy(grid, points)
    rows, cols = np.nonzero(grid.data == 100)
    if len(rows) > _MAX_POINTS:
        raise ValueError('occupied-cell support exceeds resource bound')
    within = np.zeros(len(points), dtype=bool)
    distances = np.full(len(points), np.inf)
    if len(rows) and len(points):
        local = np.column_stack([(cols + .5) * grid.resolution,
                                 (rows + .5) * grid.resolution])
        ox, oy, yaw = grid.origin
        c, s = math.cos(yaw), math.sin(yaw)
        centers = local @ np.asarray([[c, s], [-s, c]]) + [ox, oy]
        distances, _ = cKDTree(centers).query(points)
        within = distances <= threshold_m + 1e-12
    # Outside-map points cannot count as supported even near a boundary cell.
    _, inside = _occupancy(grid, points)
    within &= inside
    finite_inside = distances[inside & np.isfinite(distances)]
    result.update(occupied_cell_center_radius_m=threshold_m,
                  supported_points=int(within.sum()),
                  supported_fraction=int(within.sum()) / len(points) if len(points) else None,
                  distance_median_m=float(np.median(finite_inside)) if len(finite_inside) else None,
                  distance_p95_m=float(np.percentile(finite_inside, 95)) if len(finite_inside) else None,
                  support_basis='DISTANCE_TO_OCCUPIED_CELL_CENTER_NOT_FITTED_WALL_TRUTH')
    return result


def _world_bbox(grid, cells):
    if not cells:
        return None
    local = (np.asarray(cells, dtype=float) + .5) * grid.resolution
    ox, oy, yaw = grid.origin
    c, s = math.cos(yaw), math.sin(yaw)
    world = local @ np.asarray([[c, s], [-s, c]]) + [ox, oy]
    return {'minimum_xy_m': world.min(axis=0).tolist(),
            'maximum_xy_m': world.max(axis=0).tolist(),
            'span_xy_m': np.ptp(world, axis=0).tolist()}


def audit_array(grid, assumed_radius=.55, margin=.10, trajectory_xy=None,
                wall_points_xy=None, low_points_xy=None):
    """Summarize a Grid without changing any occupancy cell.

    Wall and low-band labels are supplied by the caller and are not presumed to
    be independently verified objects. Wall support uses occupied-cell centers.
    """
    if not isinstance(grid, Grid):
        raise ValueError('grid must be a Grid')
    trajectory = _points(trajectory_xy, 'trajectory_xy')
    walls = _points(wall_points_xy, 'wall_points_xy')
    low = _points(low_points_xy, 'low_points_xy')
    inflated = inflate_grid(grid, assumed_radius, margin)
    components = connected_components(inflated)
    count = _counts(grid.data)
    total = int(grid.data.size)
    largest = components[0] if components else []
    result = {'schema_version': 1, 'navigation_validated': False,
              'ground_height_validated': False, 'physical_test_performed': False,
              'footprint': {'model': 'EXPLICIT_ASSUMED_CIRCLE',
                            'radius_m': inflated.radius_m, 'margin_m': inflated.margin_m,
                            'measured': False},
              'unknown_space': 'BLOCKED', 'map_boundary': 'BLOCKED',
              'grid': {'width': grid.width, 'height': grid.height,
                       'resolution_m': grid.resolution, 'origin_x_y_yaw': list(grid.origin),
                       'cells': total, **{name + '_cells': value for name, value in count.items()},
                       **{name + '_fraction': value / total for name, value in count.items()}},
              'traversability': {'free_cells_after_inflation': int(inflated.free.sum()),
                                'connected_components': len(components),
                                'largest_component_cells': len(largest),
                                'largest_component_area_m2': len(largest) * grid.resolution ** 2,
                                'largest_component_center_bbox': _world_bbox(grid, largest),
                                'demo_endpoints_xy_m': None, 'demo_endpoint_distance_m': None,
                                'endpoint_selection': 'DOUBLE_BFS_HEURISTIC_NOT_EXACT_DIAMETER'}}
    if len(largest) >= 2:
        start, goal = demo_endpoints(inflated)
        result['traversability'].update(demo_endpoints_xy_m=[list(start), list(goal)],
                                        demo_endpoint_distance_m=math.dist(start, goal))
    if trajectory is not None:
        stats = _point_occupancy(grid, trajectory)
        free = np.asarray([inflated.is_free_world(point) for point in trajectory], dtype=bool)
        segments = np.asarray([inflated.segment_is_free(a, b)
                               for a, b in zip(trajectory[:-1], trajectory[1:])], dtype=bool)
        lengths = np.linalg.norm(np.diff(trajectory, axis=0), axis=1) if len(trajectory) else np.empty(0)
        total_length = float(lengths.sum())
        stats.update(circle_clear_points=int(free.sum()),
                     circle_clear_fraction=int(free.sum()) / len(free) if len(free) else None,
                     swept_segments=int(len(segments)), swept_clear_segments=int(segments.sum()),
                     swept_clear_fraction=int(segments.sum()) / len(segments) if len(segments) else None,
                     sampled_route_length_m=total_length,
                     swept_clear_route_length_m=float(lengths[segments].sum()),
                     swept_clear_length_fraction=float(lengths[segments].sum()) / total_length if total_length else None,
                     minimum_clearance_m=min((inflated.world_clearance(point) for point in trajectory), default=None),
                     route_source='CALLER_SUPPLIED_REPLAY_OR_VIRTUAL_ROUTE_NOT_PHYSICAL_TRUTH')
        result['trajectory_coverage'] = stats
    if walls is not None:
        result['wall_point_support'] = _wall_support(grid, walls)
    if low is not None:
        result['low_band_point_occupancy'] = _point_occupancy(grid, low)
        result['low_band_point_occupancy']['limitation'] = 'LOW_HEIGHT_RETURNS_MAY_INCLUDE_GROUND_BIAS_AND_REAL_OBSTACLES'
    return result


def audit_grid(path, assumed_radius=.55, margin=.10, trajectory_xy=None,
               wall_points_xy=None, low_points_xy=None):
    """Read existing validated YAML/PGM and return a JSON-serializable audit."""
    from wc_runtime.map_viewer import inspect_grid
    source, data, meta = inspect_grid(Path(path))
    result = audit_array(Grid(data, meta['resolution'], tuple(meta['origin'])),
                         assumed_radius, margin, trajectory_xy, wall_points_xy, low_points_xy)
    result['source'] = source
    return result


def _named_grids(named_grids):
    if not isinstance(named_grids, dict) or not named_grids:
        raise ValueError('named_grids must be a nonempty name-to-Grid mapping')
    for name, grid in named_grids.items():
        if not isinstance(name, str) or not name.strip() or not isinstance(grid, Grid):
            raise ValueError('named_grids must contain named Grid values')
    return named_grids


def compare_grids(named_grids, baseline_name):
    """Compare occupancy at baseline cell centers inside common world bounds.

    Different origins are supported without relabeling or modifying pixels.
    Equal resolution and axis-aligned maps are required. No common-coordinate
    transform is estimated: all supplied grids must already share a map frame.
    """
    grids = _named_grids(named_grids)
    if baseline_name not in grids:
        raise ValueError('baseline_name must identify one supplied map')
    baseline = grids[baseline_name]
    for grid in grids.values():
        yaw = math.atan2(math.sin(grid.origin[2]), math.cos(grid.origin[2]))
        if abs(yaw) > 1e-10:
            raise ValueError('common-ROI comparison requires axis-aligned map origins')
        if not math.isclose(grid.resolution, baseline.resolution, rel_tol=1e-9, abs_tol=1e-12):
            raise ValueError('common-ROI comparison requires identical resolution')
    low_x = max(grid.origin[0] for grid in grids.values())
    low_y = max(grid.origin[1] for grid in grids.values())
    high_x = min(grid.origin[0] + grid.width * grid.resolution for grid in grids.values())
    high_y = min(grid.origin[1] + grid.height * grid.resolution for grid in grids.values())
    if low_x >= high_x or low_y >= high_y:
        raise ValueError('maps have no overlapping world region')
    xs = baseline.origin[0] + (np.arange(baseline.width) + .5) * baseline.resolution
    ys = baseline.origin[1] + (np.arange(baseline.height) + .5) * baseline.resolution
    xs = xs[(xs >= low_x) & (xs < high_x)]
    ys = ys[(ys >= low_y) & (ys < high_y)]
    if not len(xs) or not len(ys):
        raise ValueError('overlap contains no baseline cell centers')
    size = len(xs) * len(ys)
    if size > _MAX_COMPARISON_CELLS:
        raise ValueError('common-ROI sample exceeds resource bound')
    xx, yy = np.meshgrid(xs, ys)
    points = np.column_stack([xx.ravel(), yy.ravel()])
    original, _ = _occupancy(baseline, points)
    result = {'schema_version': 1, 'baseline_name': baseline_name,
              'sampling': 'BASELINE_CELL_CENTERS_INSIDE_ALL_MAP_EXTENTS',
              'frame_alignment': 'CALLER_DECLARED_COMMON_MAP_FRAME_NO_AUTOMATIC_REGISTRATION',
              'resolution_m': baseline.resolution, 'sampled_cells': int(size),
              'sampled_cell_area_m2': size * baseline.resolution ** 2,
              'world_intersection_bounds_m': {'minimum_xy': [low_x, low_y],
                                             'maximum_xy': [high_x, high_y]},
              'navigation_validated': False, 'maps': {}}
    for name, grid in grids.items():
        values, inside = _occupancy(grid, points)
        if not inside.all():
            raise ValueError('numeric world sampling left a common map extent')
        counts = _counts(values)
        transitions = {old_name + '_to_' + new_name: int(np.count_nonzero((original == old) & (values == new)))
                       for old, old_name in _LABELS for new, new_name in _LABELS}
        result['maps'][name] = {'cells': counts,
                               'fractions': {key: value / size for key, value in counts.items()},
                               'baseline_transitions': transitions,
                               'changed_cells': int(np.count_nonzero(values != original))}
    return result


def render_comparison(output_path, named_grids, assumed_radius=.55, margin=.10,
                      trajectories=None):
    """Render original cells and assumed-footprint inflation in world axes."""
    grids = _named_grids(named_grids)
    if len(grids) > 8:
        raise ValueError('render at most eight named maps together')
    if trajectories is not None and not isinstance(trajectories, dict):
        raise ValueError('trajectories must map map names to x/y samples')
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Patch
    from matplotlib.transforms import Affine2D
    # Compute a common visual world extent, including rotated grid corners.
    corners = []
    for grid in grids.values():
        local = np.asarray([[0., 0.], [grid.width * grid.resolution, 0.],
                            [0., grid.height * grid.resolution],
                            [grid.width * grid.resolution, grid.height * grid.resolution]])
        ox, oy, yaw = grid.origin
        c, s = math.cos(yaw), math.sin(yaw)
        corners.extend((local @ np.asarray([[c, s], [-s, c]]) + [ox, oy]).tolist())
    corners = np.asarray(corners)
    low, high = corners.min(axis=0), corners.max(axis=0)
    aspect = (high[0] - low[0]) / (high[1] - low[1])
    height_per_map = min(6., max(2.8, 12. / max(aspect, .1)))
    fig, axes = plt.subplots(len(grids), 1, figsize=(13, height_per_map * len(grids)),
                             squeeze=False, constrained_layout=True)
    try:
        for ax, (name, grid) in zip(axes[:, 0], grids.items()):
            inflated = inflate_grid(grid, assumed_radius, margin)
            image = np.zeros(grid.data.shape, dtype=np.uint8)
            image[grid.data == 0] = 1
            image[grid.data == 100] = 2
            ox, oy, yaw = grid.origin
            transform = Affine2D().rotate(yaw).translate(ox, oy) + ax.transData
            extent = [0, grid.width * grid.resolution, 0, grid.height * grid.resolution]
            ax.imshow(image, origin='lower', interpolation='nearest', extent=extent,
                      cmap=ListedColormap(['#8a8a8a', '#ffffff', '#101010']),
                      vmin=0, vmax=2, transform=transform)
            overlay = np.zeros((*grid.data.shape, 4), dtype=float)
            overlay[(grid.data == 0) & inflated.blocked] = [.96, .55, .15, .45]
            ax.imshow(overlay, origin='lower', interpolation='nearest', extent=extent, transform=transform)
            if trajectories is not None and name in trajectories:
                route = _points(trajectories[name], 'trajectory')
                if len(route):
                    ax.plot(route[:, 0], route[:, 1], color='#2369ad', linewidth=1.2)
            audit = audit_array(grid, assumed_radius, margin)
            span = audit['traversability']['demo_endpoint_distance_m']
            span_text = f'{span:.2f} m' if span is not None else 'none'
            ax.set_title(f'{name} | largest inflated component: '
                         f"{audit['traversability']['largest_component_area_m2']:.2f} m²; demo span {span_text}")
            ax.set_xlim(low[0], high[0]); ax.set_ylim(low[1], high[1])
            ax.set_aspect('equal'); ax.set_xlabel('Map x (m)'); ax.set_ylabel('Map y (m)')
        fig.suptitle(f'Offline map candidates | assumed circle r={assumed_radius:.2f} m + margin={margin:.2f} m\n'
                     'Unknown remains blocked; real navigation and ground height are unvalidated')
        axes[-1, 0].legend(handles=[Patch(facecolor='#101010', label='Occupied'),
                                    Patch(facecolor='#8a8a8a', label='Unknown'),
                                    Patch(facecolor='#ffffff', edgecolor='#aaaaaa', label='Free'),
                                    Patch(facecolor='#facb9b', label='Blocked by assumed footprint')],
                          loc='upper center', bbox_to_anchor=(.5, -.28), ncol=4, fontsize=8)
        output = Path(output_path)
        output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output, dpi=160)
        return str(output)
    finally:
        plt.close(fig)
