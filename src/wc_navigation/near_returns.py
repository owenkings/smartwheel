"""Retrospective near-return witnesses; never remove returns or certify clearance.

XY is the configured planar body reference. Z remains prepared lidar Z. A
persistently occupied body cell is only a candidate: no point association,
verified pose/time, measured envelope, or self-return identity is supplied.
"""
from __future__ import annotations

import math
from numbers import Real

import numpy as np


MAX_POINTS = 100_000
MAX_FRAMES = 20_000


def _number(value, name, *, lower=None, upper=None):
    if (isinstance(value, (bool, np.bool_)) or not isinstance(value, Real)
            or not math.isfinite(value)):
        raise ValueError(name + ' must be finite numeric')
    value = float(value)
    if (lower is not None and value < lower) or (upper is not None and value > upper):
        raise ValueError(name + ' outside diagnostic bounds')
    return value


def _array(value, columns, name):
    raw = np.asarray(value)
    if (raw.ndim != 2 or raw.shape[1] != columns or len(raw) > MAX_POINTS
            or raw.dtype.kind not in 'fiu'):
        raise ValueError(name + ' requires bounded numeric rows')
    result = raw.astype(float, copy=False)
    if not np.isfinite(result).all() or (result.size and np.max(np.abs(result)) > 100):
        raise ValueError(name + ' exceeds finite metric bounds')
    return result


def _stats(points, ground):
    if not len(points):
        return {'count': 0, 'body_xy_lidar_z_min_m': None, 'body_xy_lidar_z_max_m': None,
                'body_radius_min_max_m': None, 'candidate_floor_height_min_max_m': None}
    radii = np.linalg.norm(points[:, :2], axis=1)
    heights = points[:, 2] - ground
    return {'count': len(points), 'body_xy_lidar_z_min_m': points.min(axis=0).tolist(),
            'body_xy_lidar_z_max_m': points.max(axis=0).tolist(),
            'body_radius_min_max_m': [float(radii.min()), float(radii.max())],
            'candidate_floor_height_min_max_m': [float(heights.min()), float(heights.max())]}


def analyze_near_frame(observation, *, radius_m=.65, context_radius_m=1.,
                       ground_z_lidar=-.7635, z_bin_m=.1):
    """Explain raw-point and circumscribed-voxel overlap in one frozen capture.

    Input is ``project_observation`` output, including its retained points.
    Witness indices address that retained array, not the original cloud. All
    members of an overlapping voxel are reported even outside context_radius.
    Outputs are JSON-safe and all source arrays are read without modification.
    """
    if not isinstance(observation, dict):
        raise ValueError('observation dictionary required')
    stamp = observation.get('stamp_ns')
    if type(stamp) is not int or not 0 <= stamp < 2**63:
        raise ValueError('bounded integer stamp_ns required')
    radius = _number(radius_m, 'radius', lower=.01, upper=5.)
    context = _number(context_radius_m, 'context radius', lower=radius, upper=10.)
    ground = _number(ground_z_lidar, 'ground z', lower=-10., upper=10.)
    z_bin = _number(z_bin_m, 'Z bin size', lower=.01, upper=.5)
    summary = observation.get('summary')
    if not isinstance(summary, dict):
        raise ValueError('projection summary required')
    voxel = _number(summary.get('voxel_m'), 'voxel size', lower=.01, upper=.5)
    disc_radius = _number(observation.get('voxel_radius_m'), 'voxel radius', lower=0., upper=1.)
    if not math.isclose(disc_radius, voxel/math.sqrt(2.), rel_tol=0., abs_tol=1e-10):
        raise ValueError('circumscribed voxel radius inconsistent with voxel size')
    configured_ground = summary.get('ground_z_lidar_m')
    if configured_ground is not None and not math.isclose(
            _number(configured_ground, 'summary ground'), ground, rel_tol=0., abs_tol=1e-10):
        raise ValueError('ground argument differs from projection candidate')
    points = _array(observation.get('points_body_xy_lidar_z'), 3, 'retained points')
    centers = _array(observation.get('voxel_centers_body_xy'), 2, 'voxel centers')
    if summary.get('retained_rows') != len(points):
        raise ValueError('retained point count differs from projection summary')
    cells = np.floor(points[:, :2]/voxel).astype(np.int64)
    unique, inverse, counts = np.unique(cells, axis=0, return_inverse=True, return_counts=True)
    computed_centers = (unique.astype(float)+.5)*voxel
    if centers.shape != computed_centers.shape or not np.allclose(centers, computed_centers, rtol=0., atol=1e-10):
        raise ValueError('supplied voxel centers differ from retained point membership')
    supplied_counts = observation.get('voxel_counts')
    if supplied_counts is not None:
        supplied_counts = np.asarray(supplied_counts)
        if supplied_counts.shape != counts.shape or not np.array_equal(supplied_counts, counts):
            raise ValueError('supplied voxel counts differ from retained membership')
    radii = np.linalg.norm(points[:, :2], axis=1)
    inside = radii <= radius
    center_gaps = np.linalg.norm(computed_centers, axis=1)-radius-disc_radius
    overlapping = center_gaps <= 1e-12
    member_overlap = overlapping[inverse]

    def witness(index):
        return {'retained_point_index': int(index), 'body_xy_lidar_z_m': points[index].tolist(),
                'body_radius_m': float(radii[index]),
                'height_above_candidate_floor_m': float(points[index, 2]-ground),
                'cell_xy': cells[index].tolist(), 'inside_assumed_circle': bool(inside[index]),
                'member_of_overlap_voxel': bool(member_overlap[index])}

    voxel_rows = []
    for cell_index in np.flatnonzero(overlapping):
        indices = np.flatnonzero(inverse == cell_index)
        voxel_rows.append({'cell_xy': unique[cell_index].tolist(),
                           'center_body_xy_m': computed_centers[cell_index].tolist(),
                           'disc_radius_m': disc_radius,
                           'initial_capture_gap_m': float(center_gaps[cell_index]),
                           'raw_inside_rows': int(inside[indices].sum()),
                           'voxel_only_overlap': not bool(inside[indices].any()),
                           'member_stats': _stats(points[indices], ground),
                           'members': [witness(i) for i in indices]})
    context_mask = radii <= context
    context_cells = []
    for cell_index in np.unique(inverse[context_mask]):
        indices = np.flatnonzero((inverse == cell_index) & context_mask)
        context_cells.append({'cell_xy': unique[cell_index].tolist(),
                              'retained_rows': len(indices),
                              'mean_body_xy_m': points[indices, :2].mean(axis=0).tolist()})
    cells_xyz = np.column_stack([cells, np.floor(points[:, 2]/z_bin).astype(np.int64)])
    context_cells_xyz = []
    unique_xyz, xyz_inverse = np.unique(cells_xyz[context_mask], axis=0, return_inverse=True)
    context_points = points[context_mask]
    for cell_index, cell in enumerate(unique_xyz):
        indices = np.flatnonzero(xyz_inverse == cell_index)
        context_cells_xyz.append({'cell_xyz': cell.tolist(), 'retained_rows': len(indices),
                                  'mean_body_xy_lidar_z_m': context_points[indices].mean(axis=0).tolist()})
    input_rows = summary.get('input_rows')
    finite_rows = summary.get('finite_rows')
    return {'schema_version': 1, 'stamp_ns': stamp,
            'status': 'RETROSPECTIVE_NEAR_RETURN_DIAGNOSTIC_NO_MASK',
            'settings': {'radius_m': radius, 'context_radius_m': context,
                         'voxel_m': voxel, 'voxel_disc_radius_m': disc_radius,
                         'ground_z_lidar_candidate_m': ground, 'z_bin_m': z_bin,
                         'z_bin_basis': 'PREPARED_LIDAR_Z_NOT_AXLE_Z'},
            'coordinates': 'CONFIGURED_BODY_XY_WITH_PREPARED_LIDAR_Z',
            'original_cloud_row_indices_available': False,
            'witness_index_basis': 'RETAINED_PROJECTION_POINT_ARRAY_NOT_ORIGINAL_CLOUD',
            'source_counts': {'input_rows': input_rows, 'finite_rows': finite_rows,
                              'retained_rows': len(points),
                              'rows_not_retained_by_upstream_projection': input_rows-len(points) if type(input_rows) is int else None,
                              'near_range_height_candidate_rows': summary.get('near_range_height_candidate_rows'),
                              'scope': 'RETAINED_POINTS_ONLY_UPSTREAM_EXCLUSIONS_NOT_RECLASSIFIED'},
            'raw_inside': _stats(points[inside], ground),
            'raw_inside_low_height_rows': int(np.sum(inside & (points[:, 2]-ground <= .15))),
            'overlap_voxel_count': len(voxel_rows),
            'voxel_only_overlap_count': sum(row['voxel_only_overlap'] for row in voxel_rows),
            'voxel_overlaps': voxel_rows,
            'context_points': [witness(i) for i in np.flatnonzero(context_mask | member_overlap)],
            'context_cells': context_cells,
            'context_cells_xyz': context_cells_xyz,
            'offline_retrospective_only': True, 'candidate_floor_measured': False,
            'envelope_measured': False, 'self_return_identified': False,
            'points_removed': 0, 'source_points_modified': False,
            'clearance_proven': False, 'physical_navigation_validated': False}


def _circular_span(angles):
    ordered = np.sort(np.mod(angles, 2*math.pi))
    if len(ordered) < 2:
        return 0.
    gaps = np.diff(np.r_[ordered, ordered[0]+2*math.pi])
    return float(2*math.pi-gaps.max())


def summarize_body_fixed_candidates(frame_reports, *, min_frames=3, min_time_span_s=1.,
                                    min_yaw_span_rad=.35, min_translation_span_m=.3,
                                    min_moving_frames=2, moving_speed_mps=.05):
    """Report cell persistence and motion diversity, never a self-return mask.

    Each frame report may be augmented by ``pose_xy_yaw`` (frozen unvalidated
    prior) and ``velocity`` with ``forward_mps``/``yaw_radps``. Missing evidence
    prevents a candidate label. Retrospective future support is not causal.
    No cross-frame endpoint identity or visibility is inferred from cell reuse.
    """
    if not isinstance(frame_reports, (list, tuple)) or len(frame_reports) > MAX_FRAMES:
        raise ValueError('bounded frame reports required')
    for value, name in [(min_frames, 'min_frames'), (min_moving_frames, 'min_moving_frames')]:
        if type(value) is not int or not 1 <= value <= MAX_FRAMES:
            raise ValueError(name + ' must be positive bounded integer')
    time_limit = _number(min_time_span_s, 'time span', lower=.01, upper=3600.)
    yaw_limit = _number(min_yaw_span_rad, 'yaw span', lower=.01, upper=2*math.pi)
    translation_limit = _number(min_translation_span_m, 'translation span', lower=.01, upper=100.)
    speed_limit = _number(moving_speed_mps, 'moving speed', lower=.001, upper=2.)
    previous = -1
    by_cell = {}
    pose_frames = moving_frames = stationary_frames = unknown_speed_frames = 0
    voxel_sizes = set()
    for index, report in enumerate(frame_reports):
        if not isinstance(report, dict) or report.get('status') != 'RETROSPECTIVE_NEAR_RETURN_DIAGNOSTIC_NO_MASK':
            raise ValueError('near-frame reports required')
        stamp = report.get('stamp_ns')
        if type(stamp) is not int or stamp <= previous:
            raise ValueError('strictly increasing integer report stamps required')
        previous = stamp
        voxel_sizes.add((report['settings']['voxel_m'], report['settings']['z_bin_m']))
        pose = report.get('pose_xy_yaw')
        pose_valid = False
        if pose is not None:
            pose = np.asarray(pose)
            if pose.shape != (3,) or pose.dtype.kind not in 'fiu' or not np.isfinite(pose).all():
                raise ValueError('pose_xy_yaw must be finite numeric triple when supplied')
            pose = pose.astype(float)
            pose_valid = True
            pose_frames += 1
        velocity = report.get('velocity')
        speed = velocity.get('forward_mps') if isinstance(velocity, dict) else None
        moving = (speed is not None and abs(_number(speed, 'recorded forward speed', lower=-2., upper=2.)) >= speed_limit)
        moving_frames += bool(moving)
        unknown_speed_frames += speed is None
        stationary_frames += speed is not None and not moving
        seen = set()
        for cell in report['context_cells_xyz']:
            key = tuple(cell['cell_xyz'])
            if len(key) != 3 or any(type(v) is not int for v in key) or key in seen:
                raise ValueError('unique integer context cells required')
            seen.add(key)
            mean = np.asarray(cell['mean_body_xy_lidar_z_m'], dtype=float)
            if mean.shape != (3,) or not np.isfinite(mean).all():
                raise ValueError('finite context-cell mean required')
            by_cell.setdefault(key, []).append((index, stamp, pose if pose_valid else None, bool(moving), mean))
    if len(voxel_sizes) > 1:
        raise ValueError('body-cell persistence requires one common voxel grid')
    rows = []
    for key, support in sorted(by_cell.items()):
        if len(support) < min_frames:
            continue
        pose_support = [s for s in support if s[2] is not None]
        span = (support[-1][1]-support[0][1])*1e-9
        pose_span = (pose_support[-1][1]-pose_support[0][1])*1e-9 if pose_support else 0.
        yaw_span = _circular_span([s[2][2] for s in pose_support])
        positions = np.array([s[2][:2] for s in pose_support])
        # Bounding-box diagonal is an upper bound, not a measured translation
        # pair. Use distance from first supporting pose: a conservative lower
        # bound on maximum pairwise recorded translation, in linear memory.
        translation_span = float(np.linalg.norm(positions-positions[0], axis=1).max()) if len(positions) else 0.
        moving_count = sum(s[3] for s in pose_support)
        candidate = (len(pose_support) >= min_frames and pose_span >= time_limit and yaw_span >= yaw_limit
                     and translation_span >= translation_limit and moving_count >= min_moving_frames)
        body_means = np.array([s[4] for s in support])
        prior_world_means = np.array([
            s[2][:2] + np.array([[math.cos(s[2][2]), -math.sin(s[2][2])],
                                  [math.sin(s[2][2]), math.cos(s[2][2])]]) @ s[4][:2]
            for s in pose_support])
        rows.append({'cell_xyz': list(key), 'support_frames': len(support),
                     'support_frame_indices': [s[0] for s in support],
                     'support_stamp_ns': [s[1] for s in support],
                     'pose_available_support_frames': len(pose_support), 'moving_support_frames': moving_count,
                     'time_span_s': span, 'pose_available_time_span_s': pose_span,
                     'circular_yaw_span_rad': yaw_span,
                     'recorded_translation_lower_bound_m': translation_span,
                     'body_xy_lidar_z_cell_mean_spread_max_from_first_m': float(np.linalg.norm(body_means-body_means[0], axis=1).max()),
                     'prior_world_xy_cell_mean_spread_max_from_first_m': float(np.linalg.norm(prior_world_means-prior_world_means[0], axis=1).max()) if len(prior_world_means) else None,
                     'label': 'BODY_FIXED_CELL_PERSISTENCE_CANDIDATE' if candidate else 'REPEATED_BODY_CELL_INSUFFICIENT_DIVERSITY',
                     'self_return_proven': False, 'point_association_proven': False})
    return {'schema_version': 1, 'status': 'RETROSPECTIVE_BODY_CELL_PERSISTENCE_DIAGNOSTIC',
            'frame_denominator': len(frame_reports), 'pose_available_frames': pose_frames,
            'moving_frame_denominator': int(moving_frames),
            'stationary_below_speed_threshold_frame_denominator': int(stationary_frames),
            'unknown_speed_frame_denominator': int(unknown_speed_frames),
            'settings': {'min_frames': min_frames, 'min_time_span_s': time_limit,
                         'min_yaw_span_rad': yaw_limit, 'min_translation_span_m': translation_limit,
                         'min_moving_frames': min_moving_frames, 'moving_speed_mps': speed_limit,
                         'cell_basis': 'BODY_XY_VOXEL_AND_EXPLICIT_PREPARED_LIDAR_Z_BIN',
                         'xy_voxel_and_z_bin_m': list(next(iter(voxel_sizes))) if voxel_sizes else None},
            'repeated_cells': rows,
            'candidate_cell_count': sum(r['label'] == 'BODY_FIXED_CELL_PERSISTENCE_CANDIDATE' for r in rows),
            'pose_basis': 'SUPPLIED_FROZEN_PRIOR_UNVALIDATED_NOT_GROUND_TRUTH',
            'time_basis': 'SUPPLIED_FRONTEND_REFERENCE_NOT_VALIDATED_ACQUISITION_TIME',
            'visibility_denominator_known': False, 'point_association_proven': False,
            'self_return_identified': False, 'mask_generated': False,
            'future_support_used_retrospectively': True, 'causal_detector': False,
            'physical_navigation_validated': False}
