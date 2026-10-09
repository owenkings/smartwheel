"""Bounded single-scene ICP exploration; never an independently validated calibration.

Input rows are finite metre coordinates in each lidar frame. The supplied rigid
transform maps right to left. Exported NN indices refer to these original rows.
No files, device interfaces, formal calibration gates or live transforms are used.
"""
import copy
import math

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from .core import CalibrationError, _se3_exp, _transform, validate_transform


POLICY = {
    'role': 'exploratory_thresholds_not_sensor_accuracy_or_validation',
    'max_input_points_per_side': 20000, 'max_absolute_coordinate_m': 1000.0,
    'levels': [
        {'voxel_m': .10, 'max_distance_m': .35, 'iterations': 20},
        {'voxel_m': .05, 'max_distance_m': .20, 'iterations': 25},
        {'voxel_m': .025, 'max_distance_m': .12, 'iterations': 30}],
    'max_optimization_points': 5000, 'normal_neighbors': 20,
    'normal_max_curvature': .08, 'normal_min_second_eigenvalue_m2': 1e-8,
    'normal_min_second_to_third_ratio': .025, 'normal_angle_abs_dot_min': .5,
    'minimum_points': 60, 'minimum_correspondences': 45,
    'minimum_overlap_fraction': .20, 'trim_keep_fraction': .80,
    'huber_m': .02, 'minimum_information_eigenvalue_ratio': 1e-5,
    'minimum_normal_direction_eigenvalue': .015,
    'maximum_step_translation_m': .06, 'maximum_step_rotation_deg': 3.0,
    'maximum_total_translation_change_m': .50, 'maximum_total_rotation_change_deg': 20.0,
    'convergence_translation_m': 1e-4, 'convergence_rotation_rad': 1e-4,
    'line_search_attempts': 6, 'metric_max_distance_m': .12,
    'scale_fitted': False, 'independent_validation_performed': False,
}


def _points(value):
    try:
        raw = np.asarray(value)
        if raw.size == 0 and raw.shape == (0,):
            raw = np.empty((0, 3))
        if raw.ndim != 2 or raw.shape[1] != 3 or len(raw) > POLICY['max_input_points_per_side']:
            raise CalibrationError('finite Nx3 input with at most 20000 original rows is required')
        if raw.dtype.kind not in 'fiu':
            raise CalibrationError('coordinates must be real numbers; booleans and text are not coordinates')
        points = np.asarray(raw, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as error:
        raise CalibrationError('invalid point array: '+str(error)) from error
    if not np.isfinite(points).all() or (np.abs(points) > POLICY['max_absolute_coordinate_m']).any():
        raise CalibrationError('finite coordinates within the 1000-m preview budget are required')
    return points.copy()


def _nearest_metrics(left, right, transform):
    limit = POLICY['metric_max_distance_m']
    metrics = {'source_points': len(right), 'target_points': len(left),
        'max_correspondence_distance_m': limit, 'forward_inlier_count': 0, 'reverse_inlier_count': 0,
        'forward_overlap_fraction': 0.0, 'reverse_overlap_fraction': 0.0,
        'nn_rmse_m': None, 'nn_median_m': None, 'nn_p95_m': None}
    if not len(left) or not len(right):
        return metrics, []
    moved = _transform(right, transform)
    distances, indices = cKDTree(left).query(moved, k=1)
    reverse_distances = cKDTree(moved).query(left, k=1)[0]
    keep, reverse_keep = distances <= limit, reverse_distances <= limit
    values = distances[keep]
    metrics.update(forward_inlier_count=int(keep.sum()), reverse_inlier_count=int(reverse_keep.sum()),
        forward_overlap_fraction=float(keep.mean()), reverse_overlap_fraction=float(reverse_keep.mean()))
    if len(values):
        metrics.update(nn_rmse_m=float(np.sqrt(np.mean(values**2))),
                       nn_median_m=float(np.median(values)), nn_p95_m=float(np.quantile(values, .95)))
    pairs = [{'left_index': int(indices[i]), 'right_index': int(i),
              'distance_m': float(distances[i]), 'verified': False} for i in np.flatnonzero(keep)]
    return metrics, pairs


def evaluate_preview(left_xyz, right_xyz, T):
    """Fixed 12-cm NN gate, both directions; residuals describe only forward inliers."""
    return _nearest_metrics(_points(left_xyz), _points(right_xyz), _rigid(T))[0]


def _rigid(value):
    transform = validate_transform(value)
    if (np.abs(transform[:3, 3]) > POLICY['max_absolute_coordinate_m']).any():
        raise CalibrationError('initial translation exceeds the bounded offline preview range')
    return transform


def _downsample(points, size):
    _, inverse = np.unique(np.floor(points/size).astype(np.int64), axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    points = np.column_stack([np.bincount(inverse, weights=points[:, a])/counts for a in range(3)])
    budget = POLICY['max_optimization_points']
    if len(points) > budget:
        points = points[np.linspace(0, len(points)-1, budget, dtype=int)]
    return points


def _normal_cloud(points, voxel):
    tree = cKDTree(points)
    distances, neighbors = tree.query(points, k=min(POLICY['normal_neighbors'], len(points)))
    patches = points[neighbors]
    centered = patches-patches.mean(axis=1, keepdims=True)
    values, vectors = np.linalg.eigh(np.einsum('nki,nkj->nij', centered, centered)/patches.shape[1])
    valid = (values[:, 1] > POLICY['normal_min_second_eigenvalue_m2']) & \
        (values[:, 1]/np.maximum(values[:, 2], 1e-15) >= POLICY['normal_min_second_to_third_ratio']) & \
        (values[:, 0]/np.maximum(values.sum(axis=1), 1e-15) <= POLICY['normal_max_curvature']) & \
        (distances[:, -1] <= max(.30, voxel*4))
    return vectors[:, :, 0], valid, tree


def _pairs(target, source, target_normal, target_valid, source_normal, source_valid, tree, T, gate):
    moved = _transform(source, T)
    distance, indices = tree.query(moved, k=1)
    rotated = source_normal @ T[:3, :3].T
    compatible = np.abs(np.einsum('ij,ij->i', rotated, target_normal[indices])) >= POLICY['normal_angle_abs_dot_min']
    mask = (distance <= gate) & target_valid[indices] & source_valid & compatible
    chosen = np.flatnonzero(mask)
    count = len(chosen)
    # Euclidean trimming precedes a Huber point-to-plane loss; no nearest pair is a known match.
    chosen = chosen[np.argsort(distance[chosen], kind='stable')[:int(math.ceil(count*POLICY['trim_keep_fraction']))]]
    q, p, n = moved[chosen], target[indices[chosen]], target_normal[indices[chosen]]
    residual = np.einsum('ij,ij->i', n, q-p)
    return q, p, n, residual, count


def _huber(values):
    absolute = np.abs(values)
    h = POLICY['huber_m']
    return np.where(absolute <= h, .5*values**2, h*(absolute-.5*h))


def _change(T, initial):
    return {'translation_m': float(np.linalg.norm(T[:3, 3]-initial[:3, 3])),
        'rotation_deg': float(np.degrees(Rotation.from_matrix(T[:3, :3] @ initial[:3, :3].T).magnitude()))}


def refine_preview(left_xyz, right_xyz, initial_T_left_right):
    """Locally refine one explicit initial transform; all outcomes remain exploratory."""
    left, right = _points(left_xyz), _points(right_xyz)
    initial = _rigid(initial_T_left_right)
    T = initial.copy()
    initial_metrics, _ = _nearest_metrics(left, right, T)
    history, reasons = [], []
    status, finest_converged = 'NOT_CONVERGED', False
    if min(len(left), len(right)) < POLICY['minimum_points']:
        status, reasons = 'INSUFFICIENT_OVERLAP', ['INSUFFICIENT_INPUT_POINTS']
    elif min(len(np.unique(left, axis=0)), len(np.unique(right, axis=0))) < 3:
        status, reasons = 'DEGENERATE', ['INSUFFICIENT_DISTINCT_GEOMETRY']
    else:
        for level_index, level in enumerate(POLICY['levels']):
            target, source = _downsample(left, level['voxel_m']), _downsample(right, level['voxel_m'])
            if min(len(target), len(source)) < POLICY['minimum_points']:
                # Finer levels can retain small but useful scenes erased by coarse voxels.
                history.append({'level': level_index, 'voxel_m': level['voxel_m'],
                    'status': 'SKIPPED_INSUFFICIENT_DOWNSAMPLED_POINTS',
                    'target_points': len(target), 'source_points': len(source)})
                continue
            distance = cKDTree(target).query(_transform(source, T), k=1)[0]
            initial_overlap = distance <= level['max_distance_m']
            if initial_overlap.sum() < POLICY['minimum_correspondences'] or \
                    initial_overlap.mean() < POLICY['minimum_overlap_fraction']:
                status, reasons = 'INSUFFICIENT_OVERLAP', ['INSUFFICIENT_DISTANCE_GATED_OVERLAP']
                history.append({'level': level_index, 'voxel_m': level['voxel_m'],
                    'source_points': len(source), 'target_points': len(target),
                    'distance_gated_correspondences': int(initial_overlap.sum())})
                break
            tn, tv, tree = _normal_cloud(target, level['voxel_m'])
            sn, sv, _ = _normal_cloud(source, level['voxel_m'])
            if min(int(tv.sum()), int(sv.sum())) < POLICY['minimum_correspondences']:
                status, reasons = 'DEGENERATE', ['INSUFFICIENT_RELIABLE_NORMALS']
                break
            for iteration in range(level['iterations']):
                q, p, n, residual, before_trim = _pairs(target, source, tn, tv, sn, sv, tree, T, level['max_distance_m'])
                detail = {'level': level_index, 'iteration': iteration, 'voxel_m': level['voxel_m'],
                    'max_distance_m': level['max_distance_m'], 'source_points': len(source),
                    'target_points': len(target), 'valid_target_normals': int(tv.sum()),
                    'valid_source_normals': int(sv.sum()), 'correspondences_before_trim': before_trim,
                    'correspondences': len(residual)}
                history.append(detail)
                if len(residual) < POLICY['minimum_correspondences'] or \
                        before_trim/len(source) < POLICY['minimum_overlap_fraction']:
                    status, reasons = 'INSUFFICIENT_OVERLAP', ['INSUFFICIENT_NORMAL_COMPATIBLE_OVERLAP']
                    break
                center = q.mean(axis=0)
                radius = max(.10, float(np.sqrt(np.mean(np.sum((q-center)**2, axis=1)))))
                J = np.column_stack((np.cross(q-center, n)/radius, n))
                weights = np.minimum(1.0, POLICY['huber_m']/np.maximum(np.abs(residual), 1e-12))
                # Mixed PCA neighborhoods along intersecting planes can invent tiny
                # components in their unconstrained direction. Six numerical singular
                # values alone therefore do not establish useful geometric constraints.
                normal_information = (n.T*weights)@n/weights.sum()
                normal_values, normal_vectors = np.linalg.eigh(normal_information)
                detail.update(normal_direction_eigenvalues=normal_values.tolist(),
                    weakest_translation_direction_left=normal_vectors[:, 0].tolist())
                weighted = J*np.sqrt(weights[:, None])
                u, singular, vt = np.linalg.svd(weighted, full_matrices=False)
                ratio = float((singular[-1]/max(singular[0], 1e-15))**2)
                rank = int(np.count_nonzero(singular > singular[0]*math.sqrt(POLICY['minimum_information_eigenvalue_ratio'])))
                detail.update(rank=rank, information_eigenvalue_ratio=ratio,
                    normalization_radius_m=radius, point_to_plane_rmse_m=float(np.sqrt(np.mean(residual**2))))
                if normal_values[0] < POLICY['minimum_normal_direction_eigenvalue']:
                    status, reasons = 'DEGENERATE', ['SURFACE_NORMAL_DIRECTIONS_DO_NOT_CONSTRAIN_3D_TRANSLATION']
                    break
                if rank < 6 or ratio < POLICY['minimum_information_eigenvalue_ratio']:
                    status, reasons = 'DEGENERATE', ['POINT_TO_PLANE_SYSTEM_NOT_FULLY_OBSERVABLE']
                    break
                normalized_delta = vt.T @ ((u.T @ (-residual*np.sqrt(weights)))/singular)
                delta = normalized_delta.copy(); delta[:3] /= radius
                scale = max(1.0, np.linalg.norm(delta[:3])/math.radians(POLICY['maximum_step_rotation_deg']),
                    np.linalg.norm(delta[3:])/POLICY['maximum_step_translation_m'])
                delta /= scale
                old_loss = float(np.mean(_huber(residual)))
                accepted, proposed = False, T
                for attempt in range(POLICY['line_search_attempts']):
                    step = delta/(2**attempt)
                    increment = _se3_exp(step)
                    increment[:3, 3] += center-increment[:3, :3]@center
                    candidate = increment@T
                    candidate_loss = float(np.mean(_huber(np.einsum('ij,ij->i', n, _transform(q, increment)-p))))
                    change = _change(candidate, initial)
                    if change['translation_m'] > POLICY['maximum_total_translation_change_m'] or \
                            change['rotation_deg'] > POLICY['maximum_total_rotation_change_deg']:
                        continue
                    if candidate_loss <= old_loss+1e-14:
                        # Reassignment must not discard much of the previous overlap to lower its loss.
                        new_count = _pairs(target, source, tn, tv, sn, sv, tree, candidate, level['max_distance_m'])[-1]
                        if new_count >= .90*before_trim:
                            accepted, proposed = True, candidate
                            break
                if not accepted:
                    status, reasons = 'NOT_CONVERGED', ['NO_BOUNDED_DESCENDING_STEP']
                    break
                T = proposed
                detail.update(rotation_step_rad=float(np.linalg.norm(step[:3])),
                    translation_step_m=float(np.linalg.norm(step[3:])),
                    fixed_correspondence_huber_loss_before=old_loss,
                    fixed_correspondence_huber_loss_after=candidate_loss)
                # Backtracking/total-change bounds can make the accepted step tiny
                # while a substantial descent direction remains. That is not convergence.
                if np.linalg.norm(delta[:3]) < POLICY['convergence_rotation_rad'] and \
                        np.linalg.norm(delta[3:]) < POLICY['convergence_translation_m']:
                    detail['numerically_converged'] = True
                    if level_index == len(POLICY['levels'])-1:
                        finest_converged = True
                    break
            if reasons:
                break
    final_metrics, correspondences = _nearest_metrics(left, right, T)
    if not reasons:
        if min(final_metrics['forward_overlap_fraction'], final_metrics['reverse_overlap_fraction']) < POLICY['minimum_overlap_fraction']:
            status, reasons = 'INSUFFICIENT_OVERLAP', ['FINAL_BIDIRECTIONAL_COVERAGE_TOO_LOW']
        elif not finest_converged:
            status, reasons = 'NOT_CONVERGED', ['FINEST_LEVEL_DID_NOT_CONVERGE_WITHIN_BUDGET']
        elif initial_metrics['nn_rmse_m'] is not None and final_metrics['nn_rmse_m'] > initial_metrics['nn_rmse_m']*1.15+.001 and \
                final_metrics['forward_overlap_fraction'] <= initial_metrics['forward_overlap_fraction']+.02:
            status, reasons = 'NOT_CONVERGED', ['FINAL_NN_ERROR_INCREASED_WITHOUT_COVERAGE_GAIN']
        else:
            status = 'CANDIDATE'
    return {'schema_version': 1, 'kind': 'offline_icp_candidate', 'status': status,
        'live_eligible': False, 'independent_validation_performed': False,
        'initial_T_left_right': initial.tolist(), 'refined_T_left_right': T.tolist(),
        'initial_metrics': initial_metrics, 'final_metrics': final_metrics,
        'transform_change': _change(T, initial), 'policy': copy.deepcopy(POLICY),
        'history': history, 'reasons': reasons, 'correspondences': correspondences,
        'correspondence_semantics': 'Unverified right-to-left nearest neighbors within the fixed metric gate; '
            'no normal gate, trimming or Huber rejection is applied to this display list. '
            'Indices refer to original provided finite arrays, not voxel centroids or known physical matches.',
        'transform_definition': 'p_left = R_left_right * p_right + t_left_right; metres; no scale fitted',
        'limitations': ['Single-scene local exploration; convergence is not independent validation.',
            'Wrong manual matches or repeated geometry can yield a wrong local optimum.',
            'No time synchronization, sensor scale, motion, installation stability or formal calibration is certified.']}
