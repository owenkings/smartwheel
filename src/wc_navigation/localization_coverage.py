"""Read-only scan/map coverage and along-heading ambiguity evidence.

These are endpoint diagnostics, not a localization acceptance test. Unknown and
outside endpoints receive a missing-coverage penalty; they are never free space.
The sampled offsets do not prove a pose is unique and no ray visibility or
physical ground truth is inferred. Nothing here updates a map or controls motion.
"""
from __future__ import annotations

import math
from numbers import Real

import numpy as np
from scipy.spatial import cKDTree

from .geometry import Grid


MAX_GRID_CELLS = 5_000_000
MAX_OCCUPIED_CELLS = 2_000_000
MAX_BEAMS = 4096
OFFSETS_M = (-2., -1., -.5, .5, 1., 2.)
POLICY = {
    'support_distance_m': .15,
    'cost_cap_and_missing_penalty_m': 1.,
    'minimum_observed_beams': 20,
    'minimum_known_fraction': .80,
    'minimum_base_support_fraction': .70,
    'maximum_support_fraction_drop': .05,
    'maximum_cost_increase_m': .025,
    'meaning': 'FIXED_DIAGNOSTIC_REVIEW_TRIGGERS_NOT_ACCURACY_OR_NAVIGATION_GATES',
}


def _number(value, label):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(label + ' must be a finite number')
    return float(value)


def _pose(value):
    try:
        if len(value) != 3:
            raise ValueError
        pose = tuple(_number(v, 'pose') for v in value)
    except (TypeError, ValueError) as exc:
        raise ValueError('pose must contain finite x, y, yaw') from exc
    if max(abs(pose[0]), abs(pose[1])) > 1_000_000:
        raise ValueError('pose exceeds metric resource bound')
    return pose


def _scan(ranges, angles):
    ranges, angles = np.asarray(ranges), np.asarray(angles)
    if (ranges.ndim != 1 or angles.shape != ranges.shape or not 1 <= len(ranges) <= MAX_BEAMS
            or ranges.dtype.kind not in 'fiu' or angles.dtype.kind not in 'fiu'):
        raise ValueError('ranges and angles must be equal bounded numeric vectors')
    ranges, angles = ranges.astype(float, copy=False), angles.astype(float, copy=False)
    if (np.isnan(ranges).any() or np.isneginf(ranges).any() or np.any(ranges <= 0)
            or not np.isfinite(angles).all() or np.any(np.abs(angles) > 2 * math.pi)
            or np.any(ranges[np.isfinite(ranges)] > 100)):
        raise ValueError('positive ranges <=100m, optional +inf missing returns, and finite radian angles required')
    observed = np.isfinite(ranges)
    return ranges[observed], angles[observed], int((~observed).sum())


class CoverageAnalyzer:
    """Cache occupied-cell centers; analyze one actual projected scan at a time."""

    def __init__(self, grid):
        if not isinstance(grid, Grid) or grid.data.size > MAX_GRID_CELLS:
            raise ValueError('bounded Grid required')
        self.grid = grid
        rows, cols = np.where(grid.data == 100)
        if len(rows) > MAX_OCCUPIED_CELLS:
            raise ValueError('occupied-cell tree exceeds resource bound')
        local = np.column_stack([cols + .5, rows + .5]) * grid.resolution
        c, s = math.cos(grid.origin[2]), math.sin(grid.origin[2])
        world = local @ np.asarray([[c, s], [-s, c]]) + grid.origin[:2]
        self._tree = cKDTree(world) if len(world) else None

    def _evaluate(self, local_points, pose):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        world = local_points @ np.asarray([[c, s], [-s, c]]) + [x, y]
        ox, oy, map_yaw = self.grid.origin
        c, s = math.cos(map_yaw), math.sin(map_yaw)
        with np.errstate(over='ignore', invalid='ignore'):
            coordinates = (world - [ox, oy]) @ np.asarray([[c, -s], [s, c]]) / self.grid.resolution
        if not np.isfinite(coordinates).all():
            raise ValueError('endpoint-to-grid coordinate overflow')
        inside = ((coordinates[:, 0] >= 0) & (coordinates[:, 0] < self.grid.width)
                  & (coordinates[:, 1] >= 0) & (coordinates[:, 1] < self.grid.height))
        values = np.full(len(world), -128, dtype=np.int16)
        indices = np.floor(coordinates[inside]).astype(np.int64)
        values[inside] = self.grid.data[indices[:, 1], indices[:, 0]]
        known = (values == 0) | (values == 100)
        distances = np.full(len(world), math.inf)
        if self._tree is not None and known.any():
            distances[known] = self._tree.query(world[known])[0]
        support = known & (distances <= POLICY['support_distance_m'] + 1e-12)
        cost = np.full(len(world), POLICY['cost_cap_and_missing_penalty_m'])
        cost[known] = np.minimum(distances[known], POLICY['cost_cap_and_missing_penalty_m'])
        n = len(world)
        finite = distances[known & np.isfinite(distances)]
        counts = {'outside': int((~inside).sum()), 'unknown': int((values == -1).sum()),
                  'free': int((values == 0).sum()), 'occupied': int((values == 100).sum()),
                  'known': int(known.sum()), 'supported': int(support.sum())}
        return {'pose_x_y_yaw': list(pose), 'observed_endpoints': n,
                **{key + '_endpoints': value for key, value in counts.items()},
                **{key + '_fraction': value / n if n else None for key, value in counts.items()},
                'known_endpoint_distance_to_occupied_center_m': {
                    'median': float(np.median(finite)), 'p95': float(np.quantile(finite, .95)),
                    'max': float(finite.max())} if len(finite) else None,
                'mean_capped_cost_with_missing_penalty_m': float(cost.mean()) if n else None}

    def analyze(self, ranges, angles_rad, lidar_pose_xy_yaw, *, heading_yaw_rad=None):
        """Return coverage and ±0.5/1/2m same-heading hypotheses; never a PASS.

        Pose is the laser origin in map coordinates. The default perturbation
        direction is the laser yaw; supply the vehicle heading explicitly when
        the laser axes differ. Positive-infinite missing beams are excluded.
        """
        pose = _pose(lidar_pose_xy_yaw)
        heading = pose[2] if heading_yaw_rad is None else _number(heading_yaw_rad, 'heading yaw')
        observed, angles, missing = _scan(ranges, angles_rad)
        local = np.column_stack([observed * np.cos(angles), observed * np.sin(angles)])
        base = self._evaluate(local, pose)
        enough = (len(observed) >= POLICY['minimum_observed_beams'] and self._tree is not None
                  and base['known_fraction'] >= POLICY['minimum_known_fraction'])
        hypotheses = []
        for offset in OFFSETS_M:
            shifted = (pose[0] + offset * math.cos(heading), pose[1] + offset * math.sin(heading), pose[2])
            candidate = self._evaluate(local, shifted)
            comparable = (enough and base['supported_fraction'] >= POLICY['minimum_base_support_fraction']
                          and candidate['known_fraction'] >= POLICY['minimum_known_fraction'])
            similar = (comparable
                       and candidate['supported_fraction'] >= base['supported_fraction'] - POLICY['maximum_support_fraction_drop']
                       and candidate['mean_capped_cost_with_missing_penalty_m'] <=
                       base['mean_capped_cost_with_missing_penalty_m'] + POLICY['maximum_cost_increase_m'])
            hypotheses.append({'offset_along_heading_m': offset, 'coverage_comparable': bool(comparable),
                               'similar_or_better_endpoint_fit': bool(similar), **candidate})
        ambiguous_offsets = [row['offset_along_heading_m'] for row in hypotheses if row['similar_or_better_endpoint_fit']]
        flags = []
        if not enough:
            flags.append('INSUFFICIENT_MAP_COVERAGE_OR_OBSERVATIONS')
        if ambiguous_offsets:
            flags.append('SIMILAR_FIT_AT_ALONG_HEADING_OFFSETS')
        return {'schema_version': 1, 'status': 'REVIEW_REQUIRED' if flags else 'NO_TRIGGER_IN_TESTED_HYPOTHESES_UNVALIDATED',
                'flags': flags, 'coverage_insufficient': not enough,
                'along_heading_ambiguity_evidence': bool(ambiguous_offsets),
                'similar_offsets_m': ambiguous_offsets, 'missing_beams_excluded': missing,
                'base': base, 'heading_yaw_rad': heading, 'hypotheses': hypotheses, 'policy': dict(POLICY),
                'interpretation': 'ENDPOINT_MAP_FIT_AND_COVERAGE_NOT_PHYSICAL_TRUTH_OR_POSE_UNIQUENESS',
                'unknown_space_is_free': False, 'ray_visibility_tested': False,
                'hardware_started': False, 'physical_navigation_validated': False}
