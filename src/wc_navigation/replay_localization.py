"""Training-only wall maps and bounded, hardware-free single-lidar localization.

The matcher reuses the existing experimental SE(2) registration policy without
relaxing its observability gates. Reference poses are consumed only after every
estimate has been calculated. This module has no ROS, source-file or device I/O.
"""
from collections import Counter
from collections.abc import Mapping
from dataclasses import asdict
import hashlib
import math

import numpy as np
from scipy.spatial import cKDTree

from wc_runtime.offline_planar_registration import Policy, align, rigid


MAX_FRAMES = 50_000
MAX_FRAME_POINTS = 200_000
MAX_TOTAL_POINTS = 50_000_000
MAX_COORDINATE_M = 1_000_000.0


def _number(value, name, *, positive=False):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.integer, np.floating)):
        raise ValueError(name + ' must be a finite number')
    result = float(value)
    if not math.isfinite(result) or (positive and result <= 0):
        raise ValueError(name + ' must be finite' + (' and positive' if positive else ''))
    return result


def _stamp(value):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)) or value < 0:
        raise ValueError('stamp_ns must be a nonnegative integer')
    return int(value)


def _points(value, *, limit=MAX_FRAME_POINTS, label='points'):
    array = np.asarray(value)
    if array.ndim != 2 or array.shape[1] != 3 or len(array) > limit or array.dtype.kind not in 'fiu':
        raise ValueError(label + ' must be a bounded finite numeric Nx3 array')
    array = np.asarray(array, dtype=np.float64)
    if not np.isfinite(array).all() or (array.size and np.max(np.abs(array)) > MAX_COORDINATE_M):
        raise ValueError(label + ' must be finite and within the coordinate resource bound')
    return array


def _pose(value):
    pose = rigid(value)
    if np.max(np.abs(pose[:3, 3])) > MAX_COORDINATE_M:
        raise ValueError('odometry/pose exceeds the coordinate resource bound')
    return pose


def _frames(values):
    result = []
    previous = None
    point_count = 0
    for row in values:
        if not isinstance(row, Mapping):
            raise ValueError('each frame must provide stamp_ns, points and odometry_T')
        stamp = _stamp(row['stamp_ns'])
        if previous is not None and stamp <= previous:
            raise ValueError('frame stamps must be unique and strictly increasing')
        points = _points(row['points'])
        pose = _pose(row['odometry_T'])
        point_count += len(points)
        if len(result) >= MAX_FRAMES or point_count > MAX_TOTAL_POINTS:
            raise ValueError('frame stream exceeds the declared resource bound')
        result.append({'stamp_ns': stamp, 'points': points, 'odometry_T': pose})
        previous = stamp
    if not result:
        raise ValueError('at least one frame is required')
    return result


def _voxel_average(points, voxel):
    if len(points) == 0:
        return np.empty((0, 3), dtype=np.float64)
    scaled = np.floor(points / voxel)
    if not np.isfinite(scaled).all() or np.max(np.abs(scaled)) > 2 ** 52:
        raise ValueError('voxel keys exceed the exact integer resource bound')
    keys = scaled.astype(np.int64)
    _, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    return np.column_stack([np.bincount(inverse, weights=points[:, axis]) / counts for axis in range(3)])


def _array_digest(array):
    # This identifies the prepared derived array, not the original recording.
    value = np.ascontiguousarray(array, dtype='<f8')
    return hashlib.sha256(value.tobytes()).hexdigest()


def build_wall_map(training_frames, ground_z=-0.7635, min_height_m=0.25,
                   max_height_m=1.6, voxel=0.08):
    """Transform and voxelize only the explicitly supplied training frames.

    ``odometry_T`` maps each prepared FLU scan into a new replay-map frame.
    Floor height and the retained above-floor interval are explicit experimental
    inputs. No trajectory corridor is painted free and no reference is fitted.
    The returned ``points`` remains an ndarray for direct matcher consumption.
    """
    ground = _number(ground_z, 'ground_z')
    low = _number(min_height_m, 'min_height_m')
    high = _number(max_height_m, 'max_height_m')
    voxel = _number(voxel, 'voxel', positive=True)
    if not 0 <= low < high:
        raise ValueError('above-ground wall height interval must satisfy 0 <= min < max')
    frames = _frames(training_frames)
    prepared = []
    frame_counts = []
    for frame in frames:
        pose, source = frame['odometry_T'], frame['points']
        moved = source @ pose[:3, :3].T + pose[:3, 3]
        _points(moved)
        height = moved[:, 2] - ground
        selected = moved[(height >= low) & (height <= high)]
        prepared.append(selected)
        frame_counts.append({'stamp_ns': frame['stamp_ns'], 'input_points': len(source),
                             'retained_height_points': len(selected)})
    all_points = np.concatenate(prepared, axis=0)
    if not len(all_points):
        raise ValueError('training frames contain no points in the declared wall height interval')
    wall = _voxel_average(all_points, voxel)
    wall.setflags(write=False)
    stamps = [frame['stamp_ns'] for frame in frames]
    summary = {
        'schema_version': 1, 'frame_id': 'replay_map',
        'mode': 'TRAINING_ONLY_LEFT_LIDAR_XYZ_WALL_MAP',
        'pose_source': 'CALLER_SUPPLIED_CAUSAL_ODOMETRY',
        'training_frames': len(frames), 'training_stamps_ns': stamps,
        'height_policy': {'ground_z_m': ground, 'min_above_ground_m': low,
                          'max_above_ground_m': high,
                          'ground_status': 'EXPLICIT_EXPERIMENTAL_ESTIMATE_NOT_SURVEYED'},
        'voxel_m': voxel, 'input_points': sum(row['input_points'] for row in frame_counts),
        'retained_height_points': len(all_points), 'voxel_points': len(wall),
        'prepared_wall_points_sha256': _array_digest(wall),
        'bounds_min_m': wall.min(axis=0).tolist(), 'bounds_max_m': wall.max(axis=0).tolist(),
        'frame_counts': frame_counts, 'query_frames_added': False,
        'external_truth': False, 'navigation_validated': False,
    }
    return {'points': wall, 'training_stamps_ns': stamps, 'summary': summary}


def _statistics(values):
    if not values:
        return None
    values = np.asarray(values, dtype=float)
    return {'count': len(values), 'median': float(np.median(values)),
            'p95': float(np.quantile(values, 0.95)), 'max': float(np.max(values))}


def _reference_evaluation(rows, reference_poses):
    result = {'status': 'NOT_SUPPLIED', 'external_truth': False,
              'used_by_matcher': False, 'navigation_validated': False,
              'interpretation': 'Post-hoc same-recording reference consistency; not independent accuracy'}
    if reference_poses is None:
        return result
    if not isinstance(reference_poses, Mapping):
        result.update(status='INVALID_REFERENCE', errors=['reference_poses must map integer stamps to SE2 poses'])
        return result
    reference, errors = {}, []
    for key, value in reference_poses.items():
        try:
            stamp = _stamp(key)
            reference[stamp] = _pose(value)
        except (ValueError, TypeError) as exc:
            errors.append(str(exc))
    groups = {}
    compared = []
    for row in rows:
        if row['estimated_T'] is None or row['stamp_ns'] not in reference:
            continue
        estimated, target = np.asarray(row['estimated_T']), reference[row['stamp_ns']]
        delta = estimated[:2, 3] - target[:2, 3]
        relative = estimated[:3, :3].T @ target[:3, :3]
        error = {'stamp_ns': row['stamp_ns'], 'status': row['status'],
                 'position_difference_m': float(np.linalg.norm(delta)),
                 'yaw_difference_deg': abs(math.degrees(math.atan2(relative[1, 0], relative[0, 0])))}
        compared.append(error)
        group = groups.setdefault(row['status'], {'position': [], 'yaw': []})
        group['position'].append(error['position_difference_m'])
        group['yaw'].append(error['yaw_difference_deg'])
    result.update(status='INVALID_REFERENCE' if errors else 'COMPARED', errors=errors,
                  compared_frames=len(compared), frames=compared,
                  by_status={key: {'position_difference_m': _statistics(value['position']),
                                   'yaw_difference_deg': _statistics(value['yaw'])}
                             for key, value in groups.items()})
    return result


def localize_replay(map_points, training_stamps, frames, initial_pose, policy=None,
                    max_gap_s=1.0, reference_poses=None):
    """Localize holdout frames without reference-pose feedback or map updates.

    The caller supplies one initial pose. All later predictions use the prior
    estimate and consecutive recorded odometry increments. Rejected matches are
    explicitly ``ODOM_ONLY``. After an input gap every remaining frame is
    ``LOST`` until a separate invocation provides an explicit new initialization.
    """
    wall = _points(map_points, limit=MAX_TOTAL_POINTS, label='map_points')
    if not len(wall):
        raise ValueError('map_points must contain training wall observations')
    training_values = [_stamp(value) for value in training_stamps]
    if (not training_values or len(training_values) > MAX_FRAMES or
            len(set(training_values)) != len(training_values)):
        raise ValueError('training_stamps must be bounded, nonempty and unique')
    training = set(training_values)
    queries = _frames(frames)
    overlap = training.intersection(frame['stamp_ns'] for frame in queries)
    if overlap:
        raise ValueError('training/query timestamps overlap; holdout frames must be disjoint')
    # A whole contiguous temporal holdout sits outside the training time extent.
    qmin, qmax = queries[0]['stamp_ns'], queries[-1]['stamp_ns']
    if min(training) <= qmax and max(training) >= qmin:
        raise ValueError('query interval intersects the training interval; use a contiguous temporal holdout')
    p = policy if isinstance(policy, Policy) else Policy(**(policy or {}))
    gap_limit = _number(max_gap_s, 'max_gap_s', positive=True)
    estimate = _pose(initial_pose)
    initial = estimate.copy()
    tree = cKDTree(wall[:, :2])
    rows, counts, rejected = [], Counter(), Counter()
    last_odom = last_stamp = None
    lost = False
    streak = longest_streak = 0
    for frame in queries:
        current_odom, stamp = frame['odometry_T'], frame['stamp_ns']
        gap = None if last_stamp is None else (stamp - last_stamp) * 1e-9
        if gap is not None and gap > gap_limit:
            lost = True
        row = {'stamp_ns': stamp, 'status': None, 'localization_available': False,
               'predicted_T': None, 'estimated_T': None, 'map_updated': False,
               'pose_covariance': None, 'covariance_calibrated': False,
               'input_gap_s': gap, 'odometry_T': current_odom.tolist()}
        if lost:
            row.update(status='LOST', rejection_reasons=['INPUT_GAP_REQUIRES_EXPLICIT_REINITIALIZATION'],
                       registration=None)
            streak = 0
        else:
            prediction = estimate if last_odom is None else estimate @ np.linalg.inv(last_odom) @ current_odom
            prediction = _pose(prediction)
            indices = tree.query_ball_point(prediction[:2, 3], p.max_range_m)
            target = wall[indices]
            local_target = (target - prediction[:3, 3]) @ prediction[:3, :3]
            registration = align(frame['points'], local_target, np.eye(4), policy=p)
            accepted = registration['accepted']
            if accepted:
                estimate = _pose(prediction @ np.asarray(registration['T_target_source']))
                streak += 1
                longest_streak = max(longest_streak, streak)
            else:
                estimate = prediction
                streak = 0
            row.update(status='MATCHED' if accepted else 'ODOM_ONLY',
                       localization_available=bool(accepted), predicted_T=prediction.tolist(),
                       estimated_T=estimate.tolist(), registration=registration,
                       target_crop_points=len(target), rejection_reasons=registration['rejection_reasons'])
        counts[row['status']] += 1
        rejected.update(row['rejection_reasons'])
        rows.append(row)
        last_odom, last_stamp = current_odom, stamp
    # No references were accessed until the complete estimate stream existed.
    reference = _reference_evaluation(rows, reference_poses)
    matched = [row['registration'] for row in rows if row['status'] == 'MATCHED']
    return {
        'schema_version': 1, 'status': 'COMPLETE',
        'outcome': ('LOST_AFTER_GAP' if lost else
                    ('MATCHES_AVAILABLE' if counts['MATCHED'] else 'NO_OBSERVABLE_MAP_MATCH')),
        'mode': 'OFFLINE_SINGLE_LIDAR_STATIC_WALL_LOCALIZATION',
        'frame_id': 'replay_map', 'initial_pose': initial.tolist(),
        'initialization': 'EXPLICIT_CALLER_POSE_ONCE_ONLY',
        'prediction': 'last_estimate @ inverse(previous_odometry) @ current_odometry',
        'reference_pose_feedback': False, 'map_updated': False,
        'global_relocalization_performed': False, 'place_uniqueness_validated': False,
        'training_stamps_ns': sorted(training),
        'query_stamps_ns': [row['stamp_ns'] for row in rows],
        'temporal_holdout': 'DISJOINT_CONTIGUOUS_QUERY_INTERVAL_OUTSIDE_TRAINING_EXTENT',
        'prepared_map_points_sha256': _array_digest(wall), 'map_points': len(wall),
        'policy': asdict(p), 'max_gap_s': gap_limit,
        'summary': {'query_frames': len(rows), 'status_counts': dict(counts),
                    'matched_frames': counts['MATCHED'], 'longest_matched_streak': longest_streak,
                    'matched_fraction': counts['MATCHED'] / len(rows),
                    'rejection_counts': dict(rejected),
                    'matched_correction_translation_m': _statistics([row['correction_translation_m'] for row in matched]),
                    'matched_correction_yaw_deg': _statistics([row['correction_yaw_deg'] for row in matched]),
                    'matched_residual_rmse_m': _statistics([row['final_quality']['rmse_m'] for row in matched])},
        'frames': rows, 'reference_evaluation': reference,
        'covariance_interpretation': 'Registration information measures geometry observability, not calibrated pose covariance',
        'external_truth': False, 'hardware_started': False, 'navigation_validated': False,
    }
