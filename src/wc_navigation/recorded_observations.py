"""Recorded endpoint projection and causal delivery for shadow diagnostics.

No free space, static-object velocity, physical stationarity, or counterfactual
sensor observation is inferred here. The supplied planar mounting and ground
height are explicit experimental inputs, not calibration acceptance.
"""
from __future__ import annotations

import math
from numbers import Real

import numpy as np


MAX_POINTS = 100_000
MAX_SNAPSHOTS = 20_000
MAX_BUFFER_ARRAY_ELEMENTS = 10_000_000
MAX_NS = 2**63 - 1


def _number(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(name + ' must be a finite number')
    return float(value)


def _ns(value, name):
    if type(value) is not int or not 0 <= value <= MAX_NS:
        raise ValueError(name + ' must be a nonnegative bounded integer nanosecond time')
    return value


def _planar(value):
    try:
        matrix = np.asarray(value)
        if matrix.dtype.kind not in 'fiu':
            raise ValueError
        matrix = matrix.astype(float, copy=True)
    except (TypeError, ValueError) as exc:
        raise ValueError('base_from_lidar requires an explicit finite planar SE(2) matrix') from exc
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-10, rtol=0)
            or not np.allclose(matrix[2], [0, 0, 1, 0], atol=1e-10, rtol=0)
            or not np.allclose(matrix[:2, 2], [0, 0], atol=1e-10, rtol=0)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-8, rtol=0)
            or not math.isclose(float(np.linalg.det(matrix[:3, :3])), 1., abs_tol=1e-8)
            or np.max(np.abs(matrix[:2, 3])) > 10_000):
        raise ValueError('base_from_lidar must be explicit planar SE(2); no implicit Z/roll/pitch removal')
    return matrix


def project_observation(points_lidar, stamp_ns, base_from_lidar, ground_z_lidar,
                        obstacle_height_m=(.05, 1.6), voxel_m=.10,
                        horizontal_fov_rad=None):
    """Keep actual obstacle-height returns and cover each body-XY voxel by a disc.

    Range limits are in the supplied lidar reference. Height is relative to the
    explicit ground candidate in that same reference. The circumscribed voxel
    disc covers every retained endpoint; it does not assert an object extent or
    velocity. In-envelope returns are retained, including possible self returns.
    ``horizontal_fov_rad`` is optional declared configuration, never inferred
    from the presence or absence of returns.
    """
    stamp = _ns(stamp_ns, 'stamp_ns')
    fixed = _planar(base_from_lidar)
    ground = _number(ground_z_lidar, 'ground z')
    if not -10 < ground < 10:
        raise ValueError('ground z exceeds the experimental metric bound')
    try:
        if isinstance(obstacle_height_m, (str, bytes, dict)) or len(obstacle_height_m) != 2:
            raise ValueError
        low, high = [_number(v, 'obstacle height') for v in obstacle_height_m]
    except (TypeError, ValueError) as exc:
        raise ValueError('two ordered obstacle height bounds required') from exc
    if not 0 <= low < high <= 5:
        raise ValueError('obstacle height must satisfy 0 <= low < high <= 5 m')
    voxel = _number(voxel_m, 'voxel size')
    if not .01 <= voxel <= .5:
        raise ValueError('voxel size must be within [.01,.5] m')
    fov = None
    if horizontal_fov_rad is not None:
        try:
            if isinstance(horizontal_fov_rad, (str, bytes, dict)) or len(horizontal_fov_rad) != 2:
                raise ValueError
            fov = [_number(v, 'declared FOV') for v in horizontal_fov_rad]
        except (TypeError, ValueError) as exc:
            raise ValueError('two finite declared horizontal FOV bounds required') from exc
        if not -math.pi <= fov[0] < fov[1] <= math.pi:
            raise ValueError('declared FOV must be ordered inside [-pi,pi]')
    points = np.asarray(points_lidar)
    if (points.ndim != 2 or points.shape[1] != 3 or len(points) > MAX_POINTS
            or points.dtype.kind not in 'fiu'):
        raise ValueError('bounded numeric Nx3 lidar endpoints required')
    points = points.astype(float, copy=False)
    finite = np.isfinite(points).all(axis=1)
    candidates = points[finite]
    ranges = np.hypot(candidates[:, 0], candidates[:, 1])
    heights = candidates[:, 2] - ground
    accepted = ((ranges >= .3-1e-12) & (ranges <= 8.+1e-12)
                & (heights >= low-1e-12) & (heights <= high+1e-12))
    kept = candidates[accepted]
    near = candidates[(ranges < .3-1e-12) & (heights >= low-1e-12) & (heights <= high+1e-12)]
    near_body = near @ fixed[:3, :3].T + fixed[:3, 3]
    body = kept @ fixed[:3, :3].T + fixed[:3, 3]
    # Prepared lidar axes are planar; preserve the actual retained Z values.
    cells = np.floor(body[:, :2] / voxel).astype(np.int64)
    unique, counts = np.unique(cells, axis=0, return_counts=True)
    centers = (unique.astype(float) + .5) * voxel
    azimuths = np.arctan2(kept[:, 1], kept[:, 0]) if len(kept) else np.empty(0)
    for array in [body, near_body, centers, counts]:
        array.setflags(write=False)
    summary = {
        'input_rows': len(points), 'finite_rows': int(finite.sum()),
        'nonfinite_rows_excluded': int((~finite).sum()),
        'retained_rows': len(body), 'range_or_height_rows_excluded': int(len(candidates)-len(kept)),
        'voxel_count': len(centers), 'range_min_m': .3, 'range_max_m': 8.,
        'near_range_excluded_points': len(near_body),
        'near_range_height_candidate_rows': len(near_body),
        'near_range_counter_meaning': 'FINITE_HEIGHT_CANDIDATE_RETURNS_BELOW_PROCESSING_RANGE_MINIMUM',
        'range_limits_basis': 'EXPLICIT_PROCESSING_CANDIDATE_NOT_VERIFIED_SENSOR_BLIND_ZONE',
        'near_field_clearance_validated': False,
        'ground_z_lidar_m': ground, 'obstacle_height_above_ground_m': [low, high],
        'ground_basis': 'EXPLICIT_FROZEN_CONFIGURATION_CANDIDATE_NOT_MEASURED_FLOOR',
        'base_from_lidar': fixed.tolist(), 'voxel_m': voxel,
        'horizontal_reference': 'CONFIGURED_PLANAR_AXLE_UNVALIDATED',
        'vertical_coordinate': 'PREPARED_LIDAR_Z_NOT_AXLE_Z',
        'voxel_coverage': 'CIRCUMSCRIBED_BODY_XY_VOXEL_DISC_NOT_ESTIMATED_OBJECT_SIZE',
        'observed_azimuth_min_max_rad': [float(azimuths.min()), float(azimuths.max())] if len(azimuths) else None,
        'observed_azimuth_is_free_space_or_fov_evidence': False,
        'declared_horizontal_fov_rad': fov,
        'self_return_mask_applied': False, 'in_envelope_returns_removed': False,
        'object_velocity_known': False, 'static_obstacles_proven': False,
        'missing_returns_are_free': False, 'empty_observation_is_clear': False,
        'source_points_modified': False, 'physical_navigation_validated': False,
    }
    return {'stamp_ns': stamp, 'points_body_xy_lidar_z': body,
            'near_range_points_body_xy_lidar_z': near_body, 'voxel_centers_body_xy': centers,
            'voxel_radius_m': voxel / math.sqrt(2.), 'voxel_counts': counts,
            'sensor_origin_body_xy': fixed[:2, 3].tolist(),
            'sensor_yaw_body_rad': math.atan2(fixed[1, 0], fixed[0, 0]),
            'horizontal_fov_rad': fov, 'summary': summary}


def _copy_payload(value, budget, depth=0):
    """Bounded defensive copies keep future scheduled payloads private."""
    budget['nodes'] += 1
    if depth > 12 or budget['nodes'] > 1_000_000:
        raise ValueError('observation metadata resource bound exceeded')
    if isinstance(value, np.ndarray):
        if value.dtype.kind not in 'fiub' or not np.isfinite(value).all():
            raise ValueError('observation arrays must be finite numeric values')
        budget['elements'] += value.size
        if budget['elements'] > MAX_BUFFER_ARRAY_ELEMENTS:
            raise ValueError('observation array resource bound exceeded')
        result = value.copy(); result.setflags(write=False); return result
    if isinstance(value, dict):
        if any(not isinstance(key, str) or len(key) > 200 for key in value):
            raise ValueError('observation metadata requires bounded string keys')
        return {key: _copy_payload(item, budget, depth+1) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_copy_payload(item, budget, depth+1) for item in value]
    if value is None or type(value) in (bool, int):
        return value
    if isinstance(value, (Real, np.bool_)):
        result = value.item() if isinstance(value, np.generic) else value
        if isinstance(result, float) and not math.isfinite(result):
            raise ValueError('observation metadata must be finite')
        return result
    if isinstance(value, str) and len(value) <= 20_000:
        return value
    raise ValueError('unsupported or oversized observation payload')


class CausalObservationBuffer:
    """Deliver only snapshots whose explicit receipt time has passed.

    Input schedule is capture ordered with unique increasing ``stamp_ns``;
    receipt order may differ. Late old packets never replace a newer capture.
    Query times must strictly increase. Stale observations are retained for
    diagnosis, with ``usable=False``; neither dropout nor emptiness is clearance.
    """

    def __init__(self, schedule):
        if not isinstance(schedule, (tuple, list)) or len(schedule) > MAX_SNAPSHOTS:
            raise ValueError('bounded explicit observation schedule required')
        events=[]; previous=-1; budget={'nodes':0,'elements':0}
        for item in schedule:
            if not isinstance(item, dict) or set(item) != {'stamp_ns','received_time_ns','observation'}:
                raise ValueError('schedule requires capture, explicit receipt and observation')
            stamp=_ns(item['stamp_ns'],'capture time'); received=_ns(item['received_time_ns'],'receipt time')
            if stamp <= previous:
                raise ValueError('capture timestamps must strictly increase')
            if received < stamp:
                raise ValueError('capture cannot be in the future of its receipt')
            observation=item['observation']
            if (not isinstance(observation,dict) or type(observation.get('stamp_ns')) is not int
                    or observation.get('stamp_ns') != stamp):
                raise ValueError('payload capture timestamp must match schedule')
            previous=stamp
            events.append((received,stamp,_copy_payload(observation,budget)))
        self._events=sorted(events,key=lambda item:(item[0],item[1]))
        self._cursor=0; self._latest=None; self._now=None

    def latest(self, now_ns, max_age_ns):
        now=_ns(now_ns,'now'); age_limit=_ns(max_age_ns,'maximum age')
        if self._now is not None and now <= self._now:
            raise ValueError('query time must strictly increase')
        self._now=now
        while self._cursor < len(self._events) and self._events[self._cursor][0] <= now:
            received,stamp,observation=self._events[self._cursor]; self._cursor+=1
            if self._latest is None or stamp > self._latest[1]:
                self._latest=(received,stamp,observation)
        if self._latest is None:
            return {'status':'UNAVAILABLE','usable':False,'observation':None,'stamp_ns':None,
                    'received_time_ns':None,'age_ns':None,'delivery_latency_ns':None,
                    'dropout_is_clear':False,'physical_navigation_validated':False}
        received,stamp,observation=self._latest
        age=now-stamp
        return {'status':'FRESH' if age <= age_limit else 'STALE','usable':age <= age_limit,
                'observation':_copy_payload(observation,{'nodes':0,'elements':0}),
                'stamp_ns':stamp,'received_time_ns':received,'age_ns':age,
                'delivery_latency_ns':received-stamp,'dropout_is_clear':False,
                'physical_navigation_validated':False}
