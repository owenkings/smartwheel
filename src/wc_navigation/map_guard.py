"""Protect repeated wall/low-height evidence in an offline candidate map.

This module only adds occupied or unknown cells. It never makes space free,
fits a floor, changes a footprint, or qualifies physical navigation. Callers
must retain the evidence that their supplied observation voxels meet the stated
multi-view support rule; this rasterizer cannot verify that from XY centers.
"""
from __future__ import annotations

import hashlib
import json
import math
from numbers import Real
from pathlib import Path

import numpy as np

from .geometry import Grid


MAX_CELLS = 5_000_000
MAX_RASTER_UPDATES = 5_000_000
MAX_OBSERVATIONS = 500_000


def _positive(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real):
        raise ValueError(name + ' must be a finite positive number')
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise ValueError(name + ' must be a finite positive number')
    return number


def _grid(grid):
    if not isinstance(grid, Grid):
        raise ValueError('grid must be a Grid')
    if grid.data.size > MAX_CELLS:
        raise ValueError('guarded map exceeds five-million-cell resource bound')
    yaw = math.atan2(math.sin(grid.origin[2]), math.cos(grid.origin[2]))
    if abs(yaw) > 1e-10:
        raise ValueError('observation protection requires an axis-aligned map')
    return grid


def _points(values, name):
    try:
        array = np.asarray(values)
    except (TypeError, ValueError) as exc:
        raise ValueError(name + ' must be an Nx2 finite numeric array') from exc
    if array.shape == (0,):
        array = np.empty((0, 2), dtype=float)
    if (array.ndim != 2 or array.shape[1] != 2 or len(array) > MAX_OBSERVATIONS
            or array.dtype.kind not in 'iuf' or not np.isfinite(array).all()):
        raise ValueError(name + ' must be a bounded Nx2 finite numeric array')
    return array.astype(float, copy=False)


def _snap_integer(values):
    """Stabilize roundoff at exact grid edges without changing sub-cell edges."""
    rounded = np.rint(values)
    return np.where(np.abs(values - rounded) <= 1e-9, rounded, values)


def _rasterize(grid, points, voxel_m):
    mask = np.zeros(grid.data.shape, dtype=bool)
    stats = {'input_voxels': len(points), 'intersecting_voxels': 0,
             'outside_voxels': 0, 'partially_clipped_voxels': 0,
             'raster_cell_updates_including_overlap': 0, 'unique_covered_cells': 0}
    if not len(points):
        return mask, stats
    half = voxel_m / 2
    with np.errstate(over='ignore', invalid='ignore'):
        low = (points - half - np.asarray(grid.origin[:2])) / grid.resolution
        high = (points + half - np.asarray(grid.origin[:2])) / grid.resolution
    if not np.isfinite(low).all() or not np.isfinite(high).all():
        raise ValueError('observation voxel-to-grid coordinate overflow')
    low, high = _snap_integer(low), _snap_integer(high)
    # The square's positive-area intersection with a map cell is protected.
    # Exact boundary-only contact does not add an extra neighboring cell.
    for lower, upper in zip(low, high):
        x0, y0 = math.floor(lower[0]), math.floor(lower[1])
        x1, y1 = math.ceil(upper[0]), math.ceil(upper[1])
        left, bottom = max(0, x0), max(0, y0)
        right, top = min(grid.width, x1), min(grid.height, y1)
        if left >= right or bottom >= top:
            stats['outside_voxels'] += 1
            continue
        stats['intersecting_voxels'] += 1
        if (left, bottom, right, top) != (x0, y0, x1, y1):
            stats['partially_clipped_voxels'] += 1
        stats['raster_cell_updates_including_overlap'] += (right - left) * (top - bottom)
        if stats['raster_cell_updates_including_overlap'] > MAX_RASTER_UPDATES:
            raise ValueError('observation rasterization exceeds five-million-cell update bound')
        mask[bottom:top, left:right] = True
    stats['unique_covered_cells'] = int(mask.sum())
    return mask, stats


def _counts(data):
    return {'occupied': int(np.count_nonzero(data == 100)),
            'free': int(np.count_nonzero(data == 0)),
            'unknown': int(np.count_nonzero(data == -1))}


def protect_observations(grid, wall_points_xy, low_points_xy, support_voxel_m=.1):
    """Return a new Grid plus protection provenance, leaving the input intact.

    Each input point is the center of an axis-aligned world-space observation
    voxel square. Repeated wall support forces intersecting cells occupied.
    Repeated low-height support only turns otherwise-free cells unknown.
    Existing occupied cells are preserved, and wall support wins over low-band
    support. No unobserved space is promoted to free.
    """
    grid = _grid(grid)
    walls = _points(wall_points_xy, 'wall_points_xy')
    low = _points(low_points_xy, 'low_points_xy')
    voxel = _positive(support_voxel_m, 'support_voxel_m')
    if not .01 <= voxel <= 1.0:
        raise ValueError('support_voxel_m must be within [0.01,1.0] meters')
    wall_mask, wall_stats = _rasterize(grid, walls, voxel)
    low_mask, low_stats = _rasterize(grid, low, voxel)
    low_only = low_mask & ~wall_mask
    original = grid.data
    output = original.copy()
    output[low_only & (original == 0)] = -1
    output[wall_mask] = 100
    report = {
        'schema_version': 1, 'mode': 'REPEATED_OBSERVATION_GUARD',
        'navigation_validated': False, 'ground_height_validated': False,
        'physical_test_performed': False, 'hardware_started': False,
        'observation_support_requirement': 'CALLER_PREDECLARED_AT_LEAST_THREE_DISTINCT_OBSERVER_BINS',
        'support_verified_here': False,
        'support_voxel_m': voxel,
        'voxel_geometry': 'AXIS_ALIGNED_WORLD_SQUARE_CENTER_PLUS_MINUS_HALF_VOXEL',
        'cell_overlap_rule': 'POSITIVE_AREA_INTERSECTION_EDGE_ROUNDOFF_SNAPPED_WITHIN_1E_MINUS9_CELLS',
        'policy': {'wall': 'FORCE_OCCUPIED', 'low_only': 'FREE_TO_UNKNOWN_ONLY',
                   'overlap': 'WALL_WINS', 'unknown_space': 'BLOCKED_BY_NAVIGATION_CONSUMER',
                   'footprint_modified': False, 'erosion_or_cleanup': False},
        'grid': {'width': grid.width, 'height': grid.height, 'resolution_m': grid.resolution,
                 'origin_x_y_yaw': list(grid.origin)},
        'wall_observations': wall_stats, 'low_observations': low_stats,
        'low_only_covered_cells': int(low_only.sum()),
        'changes': {
            'wall_free_to_occupied': int(np.count_nonzero(wall_mask & (original == 0))),
            'wall_unknown_to_occupied': int(np.count_nonzero(wall_mask & (original == -1))),
            'wall_existing_occupied': int(np.count_nonzero(wall_mask & (original == 100))),
            'low_free_to_unknown': int(np.count_nonzero(low_only & (original == 0))),
            'changed_cells': int(np.count_nonzero(output != original)),
        },
        'before_cells': _counts(original), 'after_cells': _counts(output),
        'original_grid_data_sha256': hashlib.sha256(original.tobytes()).hexdigest(),
        'guarded_grid_data_sha256': hashlib.sha256(output.tobytes()).hexdigest(),
        'invariants': {
            'original_occupied_preserved': bool(np.all(output[original == 100] == 100)),
            'no_occupied_or_unknown_became_free': bool(np.all(output[original != 0] != 0)),
            'no_new_free_cells': bool(np.all((output != 0) | (original == 0))),
            'uncovered_cells_unchanged': bool(np.array_equal(output[~(wall_mask | low_mask)],
                                                           original[~(wall_mask | low_mask)])),
        },
        'limitation': 'Repeated observations protect map evidence; they do not prove ground clearance, '
                      'obstacle identity, sensor accuracy, or physical route safety.',
    }
    if not all(report['invariants'].values()):
        raise RuntimeError('observation protection violated a conservative-map invariant')
    return Grid(output, grid.resolution, grid.origin), report


def _ordinary(path):
    result = Path(path).absolute()
    if '..' in result.parts:
        raise ValueError('output path may not contain parent traversal')
    for parent in (result, *result.parents):
        if parent.is_symlink() or getattr(parent, 'is_junction', lambda: False)():
            raise ValueError('output path may not contain linked directories')
    return result


def write_guarded_map(grid, directory, source_metadata):
    """Export to a fresh directory as exact YAML/PGM plus caller provenance.

    The caller chooses an already guarded, configured storage location. This
    function never overwrites a directory or source map. YAML preserves the
    supplied resolution and origin; PGM uses ROS trinary map conventions.
    """
    grid = _grid(grid)
    if not isinstance(source_metadata, dict):
        raise ValueError('source_metadata must be an explicit provenance object')
    # Reject unserializable/nonfinite provenance before creating any output.
    json.dumps(source_metadata, ensure_ascii=False, allow_nan=False)
    import yaml
    target = _ordinary(directory)
    if target.exists():
        raise FileExistsError('guarded map output directory must be fresh')
    pixels = np.full(grid.data.shape, 205, dtype=np.uint8)
    pixels[grid.data == 0] = 254
    pixels[grid.data == 100] = 0
    pgm = (f'P5\n{grid.width} {grid.height}\n255\n'.encode('ascii')
           + np.flipud(pixels).tobytes())
    specification = {'image': 'map.pgm', 'mode': 'trinary', 'resolution': grid.resolution,
                     'origin': list(grid.origin), 'negate': 0,
                     'occupied_thresh': .65, 'free_thresh': .196}
    yaml_raw = yaml.safe_dump(specification, sort_keys=False).encode('utf-8')
    metadata = {
        'schema_version': 1, 'mode': 'GUARDED_OFFLINE_MAP_EXPORT',
        'navigation_validated': False, 'ground_height_validated': False,
        'physical_test_performed': False, 'hardware_started': False,
        'map_yaml': 'map.yaml', 'map_image': 'map.pgm',
        'occupancy_values': {'occupied': 100, 'free': 0, 'unknown': -1},
        'pgm_values': {'occupied': 0, 'free': 254, 'unknown': 205},
        'row_order': 'PGM_TOP_TO_BOTTOM_FROM_ROS_BOTTOM_TO_TOP_GRID',
        'origin_x_y_yaw': list(grid.origin), 'resolution_m': grid.resolution,
        'width': grid.width, 'height': grid.height,
        'grid_data_sha256': hashlib.sha256(grid.data.tobytes()).hexdigest(),
        'file_sha256': {'map.yaml': hashlib.sha256(yaml_raw).hexdigest(),
                        'map.pgm': hashlib.sha256(pgm).hexdigest()},
        'source_metadata': source_metadata,
    }
    provenance_raw = (json.dumps(metadata, ensure_ascii=False, allow_nan=False, indent=2) + '\n').encode('utf-8')
    target.mkdir(parents=True, exist_ok=False)
    for name, raw in [('map.pgm', pgm), ('map.yaml', yaml_raw), ('provenance.json', provenance_raw)]:
        with (target / name).open('xb') as stream:
            stream.write(raw)
    return {'directory': str(target), 'map_yaml': str(target / 'map.yaml'),
            'map_image': str(target / 'map.pgm'), 'provenance': str(target / 'provenance.json'),
            'file_sha256': {**metadata['file_sha256'],
                            'provenance.json': hashlib.sha256(provenance_raw).hexdigest()},
            'navigation_validated': False, 'physical_test_performed': False}
