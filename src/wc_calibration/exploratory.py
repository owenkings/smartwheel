"""Bounded single-scene exploration, never a production calibration or TF source.

Only numerical primitives are reused from core. Its formal dataset, holdout and
promotion gates remain unchanged. All transforms here are hypotheses in metres.
"""
import argparse
from dataclasses import asdict
import hashlib
import itertools
import json
from pathlib import Path
import sys

import numpy as np
import scipy
from scipy.spatial.transform import Rotation

from . import core
from .cli import save_version


def _integer(value, name, minimum, maximum):
    if type(value) is not int or not minimum <= value <= maximum:
        raise core.CalibrationError(f'{name} must be an integer in [{minimum}, {maximum}]')
    return value


def _input(data, profile):
    if not isinstance(data, dict):
        raise core.CalibrationError('prepared input must be an object')
    if type(data.get('schema_version')) is not int or data['schema_version'] != 1 or data.get('status') != 'PREPARED_NOT_VALIDATED':
        raise core.CalibrationError('explicit prepared v1 input required')
    if data.get('source_mode') not in ('real', 'synthetic'):
        raise core.CalibrationError('explicit real/synthetic source_mode required')
    ids = data.get('sensor_ids', {})
    if not isinstance(ids, dict) or set(ids) != {'left', 'right'} or any(not isinstance(x, str) or not x.strip() for x in ids.values()) or ids['left'] == ids['right']:
        raise core.CalibrationError('two explicit distinct sensor identities required')
    training = data.get('training')
    if not isinstance(training, list) or len(training) != 1 or data.get('validation') != []:
        raise core.CalibrationError('exploration requires exactly one training scene and an empty validation list')
    if data.get('initial_T_left_right') is not None:
        raise core.CalibrationError('exploration uses data-derived hypotheses, not an imposed transform')
    scene = training[0]
    if not isinstance(scene, dict) or not isinstance(scene.get('id'), str) or not scene['id'].strip():
        raise core.CalibrationError('explicit scene identity required')
    origin = scene.get('selected_raw_frames', {})
    if not isinstance(origin, dict) or set(origin) != {'left', 'right'} or any(not isinstance(v, dict) for v in origin.values()):
        raise core.CalibrationError('prepared SourceFrame provenance required for both sides')
    preparation = data.get('input_preparation', {})
    if not isinstance(preparation, dict) or preparation.get('kind') != 'rosbag2_source_frame' or not isinstance(preparation.get('bag_uri'), str) or not preparation['bag_uri'].strip():
        raise core.CalibrationError('explicit bag preparation provenance required')
    policy = {'max_input_points_per_side': 200000, 'max_solver_points_per_side': 12000,
              'refine_candidates': 6}
    supplied = data.get('exploration_policy', {})
    if not isinstance(supplied, dict) or set(supplied)-set(policy):
        raise core.CalibrationError('unknown exploration budget')
    policy.update(supplied)
    _integer(policy['max_input_points_per_side'], 'input point budget', profile.min_scene_points, 200000)
    _integer(policy['max_solver_points_per_side'], 'solver point budget', profile.min_scene_points, 20000)
    _integer(policy['refine_candidates'], 'refined hypothesis budget', 4, 6)
    if policy['max_solver_points_per_side'] > policy['max_input_points_per_side']:
        raise core.CalibrationError('solver budget exceeds input budget')
    conventions = data.get('coordinate_conventions') or {s: origin[s].get('coordinate_convention') for s in ids}
    units = data.get('units')
    if units is None and origin['left'].get('units') == origin['right'].get('units'):
        units = origin['left'].get('units')
    if units != 'm':
        raise core.CalibrationError('prepared point arrays must explicitly be in metres')
    if not isinstance(conventions, dict) or set(conventions) != set(ids) or any(not isinstance(v, str) or not v.strip() for v in conventions.values()) or conventions['left'] != conventions['right']:
        raise core.CalibrationError('matching explicit coordinate conventions required')
    points, budgets = {}, {}
    for side in ids:
        meta = origin[side]
        key = meta.get('raw_key')
        if not isinstance(key, list) or len(key) != 4 or key[1] != ids[side] or \
                any(not isinstance(key[i], str) or not key[i] for i in (0, 1, 2)) or \
                type(key[3]) is not int or key[3] < 0:
            raise core.CalibrationError('authoritative raw key does not match sensor identity')
        if meta.get('sensor_id') != ids[side] or meta.get('coordinate_convention') != conventions[side] or meta.get('units') != units:
            raise core.CalibrationError('point units/axes/identity contradict source provenance')
        for field in ('source_config_hash', 'cloud_data_sha256'):
            value = meta.get(field)
            if not isinstance(value, str) or len(value) != 64 or any(c not in '0123456789abcdefABCDEF' for c in value):
                raise core.CalibrationError('explicit source and content SHA256 required')
        if not isinstance(scene.get(side), list) or len(scene[side]) > policy['max_input_points_per_side']:
            raise core.CalibrationError('input exceeds explicit point budget')
        full = core._points(scene[side], profile.min_scene_points)
        if np.max(np.abs(full)) > 10000:
            raise core.CalibrationError('coordinate magnitude exceeds numerical exploration bound')
        order = np.lexsort((full[:, 2], full[:, 1], full[:, 0]))
        count = min(len(full), policy['max_solver_points_per_side'])
        indices = order[np.linspace(0, len(full)-1, count, dtype=int)]
        points[side] = full[indices].copy()
        budgets[side] = {'input_count': len(full), 'solver_count': count,
                         'sampling': 'lexicographic_xyz_then_even_indices',
                         'sampled_indices_sha256': hashlib.sha256(np.asarray(indices, dtype='<i8').tobytes()).hexdigest()}
    if origin['left']['raw_key'][0] != origin['right']['raw_key'][0]:
        raise core.CalibrationError('pair spans different recording sessions')
    scene_out = {'id': scene['id'], **points}
    provenance = {'scene_id': scene['id'], 'sensor_ids': ids, 'units': units,
                  'coordinate_conventions': conventions, 'selected_raw_frames': origin,
                  'input_preparation': preparation, 'pair_dt_ns': scene.get('pair_dt_ns'),
                  'time_quality': scene.get('time_quality'),
                  'bag_record_window_ns': scene.get('bag_record_window_ns'),
                  'static_suggestion': scene.get('static_suggestion'),
                  'operator_provenance': data.get('provenance')}
    return scene_out, policy, budgets, provenance


def _hypotheses(scene):
    left, right = scene['left'], scene['right']
    lc, rc = np.median(left, axis=0), np.median(right, axis=0)
    lv, lvec = np.linalg.eigh(np.cov((left-lc).T))
    rv, rvec = np.linalg.eigh(np.cov((right-rc).T))
    transforms = [np.eye(4)]
    transforms[0][:3, 3] = lc-rc
    for permutation in itertools.permutations(range(3)):
        for signs in itertools.product((-1, 1), repeat=3):
            rotation = lvec @ (np.eye(3)[:, permutation] @ np.diag(signs)) @ rvec.T
            if np.linalg.det(rotation) < 0:
                continue
            transform = np.eye(4)
            transform[:3, :3], transform[:3, 3] = rotation, lc-rotation @ rc
            transforms.append(core.validate_transform(transform))
    return transforms, {'left_covariance_eigenvalues_m2': lv.tolist(),
                        'right_covariance_eigenvalues_m2': rv.tolist()}


def _score(forward, backward, transform):
    values = []
    for prepared, value in ((forward, transform), (backward, np.linalg.inv(transform))):
        for _, _, source, _, _, tree in prepared:
            distances, _ = tree.query(core._transform(source, value))
            values.append(float(np.mean(np.sort(distances)[:max(1, int(len(distances)*.7))])))
    score = float(np.mean(values))
    if not np.isfinite(score):
        raise core.CalibrationError('nonfinite bidirectional score')
    return score


def _compare(reference, candidate):
    delta = candidate @ np.linalg.inv(reference)
    return {'translation_difference_m': float(np.linalg.norm(delta[:3, 3])),
            'rotation_difference_rad': float(Rotation.from_matrix(delta[:3, :3]).magnitude())}


def explore(data):
    profile = core.QualityProfile()
    scene, budget, counts, provenance = _input(data, profile)
    result = {'schema_version': 1, 'kind': 'single_scene_exploration', 'status': 'EXPLORATION_ONLY',
              'source_mode': data['source_mode'], 'computation_status': 'UNAVAILABLE',
              'live_eligible': False, 'independent_validation_performed': False,
              'time_validated': False, 'ground_validated': False, 'navigation_validated': False,
              'global_uniqueness_validated': False, 'scale_fitted': False,
              'input_hash': core._hash(data), 'input_provenance': provenance,
              'point_budget': counts, 'policy': {'budget': budget, 'quality': asdict(profile),
                  'score': 'mean_bidirectional_trimmed_70_percent_euclidean_metres',
                  'near_optimal_score_gap_m': .03, 'distinct_translation_m': .10, 'distinct_rotation_rad': .08,
                  'threshold_role': 'experimental_diagnostics_not_accuracy_claims'},
              'hypotheses': [], 'candidates': [], 'failed_hypotheses': [], 'best_candidate_id': None,
              'ambiguity': {'near_optimal_distinct': False, 'comparisons': []},
              'versions': {'algorithm': 'wc_calibration/single_scene_exploration_v1',
                           'numpy': np.__version__, 'scipy': scipy.__version__},
              'limitations': ['One cabinet scene has no independent holdout; never promote this result.',
                  'Arrival pairing and apparent stillness do not prove exposure synchronization.',
                  'Low plane residual can coexist with unconstrained motion or repeated-object ambiguity.',
                  'Data-derived initialization is not global registration or proof of correct object correspondence.',
                  'No measured baseline, installation tilt, optical height or physical scale was assumed.']}
    try:
        transforms, result['pca_diagnostics'] = _hypotheses(scene)
        result['hypotheses'] = [{'id': f'h{index:02d}', 'initial_T_left_right': transform.tolist(),
            'coarse_score_m': None, 'selected_for_refinement': False} for index, transform in enumerate(transforms)]
        # Cache voxelization/normals once per scale, shared by all bounded solves.
        scales = [(core._prepared([scene], voxel, profile), core._prepared([scene], voxel, profile, reverse=True))
                  for voxel in profile.voxel_sizes]
        result['scale_geometry'] = [{'voxel_m': voxel, 'left_points': len(f[0][1]),
            'right_points': len(b[0][1]), 'left_valid_normals': int(f[0][4].sum()),
            'right_valid_normals': int(b[0][4].sum())}
            for voxel, (f, b) in zip(profile.voxel_sizes, scales)]
        for hypothesis, transform in zip(result['hypotheses'], transforms):
            hypothesis['coarse_score_m'] = _score(*scales[0], transform)
        selected = sorted(result['hypotheses'], key=lambda h: (h['coarse_score_m'], h['id']))[:budget['refine_candidates']]
        for hypothesis in selected:
            hypothesis['selected_for_refinement'] = True
            try:
                transform = np.asarray(hypothesis['initial_T_left_right'])
                stages = []
                for voxel, distance, prepared in zip(profile.voxel_sizes, profile.correspondence_distances, scales):
                    transform, detail = core._optimize(prepared[0], transform, distance, profile)
                    transform = core.validate_transform(transform)
                    stages.append({'voxel_m': voxel, 'max_correspondence_m': distance, **detail})
                forward = core._correspondences(scales[-1][0], transform, profile.correspondence_distances[-1])
                backward = core._correspondences(scales[-1][1], np.linalg.inv(transform), profile.correspondence_distances[-1])
                info = core._information(forward[0], forward[2], forward[3], forward[1], profile)
                reverse_info = core._information(backward[0], backward[2], backward[3], backward[1], profile)
                reasons = []
                for direction, values, information in (('forward', forward, info), ('backward', backward, reverse_info)):
                    stats = values[4][0]
                    if stats['correspondences'] < profile.min_correspondences or stats['overlap_ratio'] < profile.min_overlap_ratio:
                        reasons.append(direction+':LOW_OVERLAP')
                    if stats['abs_residual_quantiles_m']['p95'] is None or stats['abs_residual_quantiles_m']['p95'] > profile.validation_p95_m:
                        reasons.append(direction+':RESIDUAL_LIMIT')
                    if not information['observable']:
                        reasons.append(direction+':DEGENERATE_GEOMETRY')
                if stages[-1].get('reason'):
                    reasons.append(stages[-1]['reason'])
                if not stages[-1]['converged']:
                    reasons.append('FINAL_SCALE_NOT_CONVERGED')
                candidate = {'id': hypothesis['id'], 'candidate_T_left_right': transform.tolist(),
                    'candidate_T_right_left': np.linalg.inv(transform).tolist(),
                    'score_m': _score(*scales[-1], transform), 'geometry_usable': not reasons,
                    'rejection_reasons': reasons, 'diagnostics': {'forward': forward[4], 'backward': backward[4],
                        'information': info, 'reverse_information': reverse_info, 'stages': stages},
                    'orientation_observation': {'right_x_axis_dot_left_x_axis': float(transform[0, 0]),
                        'axis_semantics': 'input coordinate convention; +X is not assumed to be physical forward',
                        'estimated_origin_distance_m': float(np.linalg.norm(transform[:3, 3])),
                        'used_as_measured_prior': False}}
                result['candidates'].append(candidate)
            except (core.CalibrationError, np.linalg.LinAlgError, FloatingPointError, ValueError) as error:
                result['failed_hypotheses'].append({'id': hypothesis['id'], 'error': type(error).__name__+': '+str(error)})
        result['candidates'].sort(key=lambda c: (c['score_m'], c['id']))
        if result['candidates']:
            result['computation_status'] = 'CANDIDATES_AVAILABLE'
            best = result['candidates'][0]
            result['best_candidate_id'] = best['id']
            for candidate in result['candidates'][1:]:
                difference = _compare(np.asarray(best['candidate_T_left_right']), np.asarray(candidate['candidate_T_left_right']))
                gap = candidate['score_m']-best['score_m']
                distinct = difference['translation_difference_m'] > .10 or difference['rotation_difference_rad'] > .08
                near = distinct and gap <= .03
                result['ambiguity']['comparisons'].append({'candidate_id': candidate['id'], **difference,
                    'score_gap_m': gap, 'distinct_pose': distinct, 'near_optimal': near})
                result['ambiguity']['near_optimal_distinct'] |= bool(near)
    except (core.CalibrationError, np.linalg.LinAlgError, FloatingPointError, ValueError) as error:
        result['numerical_error'] = type(error).__name__+': '+str(error)
    # Reject accidental NaN serialization; do not replace unavailable numbers by zero.
    json.dumps(result, allow_nan=False)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output-root', required=True, type=Path)
    parser.add_argument('--version', required=True)
    args = parser.parse_args(argv)
    try:
        if args.input.stat().st_size > 128_000_000:
            raise core.CalibrationError('input JSON exceeds 128 MB limit')
        with args.input.open(encoding='utf-8') as stream:
            data = json.load(stream)
        result = explore(data)
        path = save_version(result, args.output_root, args.version)
        print(json.dumps({'result_path': str(path), 'status': result['status'],
            'computation_status': result['computation_status'], 'live_eligible': False}))
        return 0 if result['computation_status'] == 'CANDIDATES_AVAILABLE' else 2
    except (core.CalibrationError, OSError, ValueError, TypeError, KeyError) as error:
        print(json.dumps({'error': type(error).__name__, 'detail': str(error)}), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
