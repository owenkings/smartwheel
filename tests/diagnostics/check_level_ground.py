#!/usr/bin/env python3
"""Bounded offline checks of gravity display rotation and unlabelled plane patches.

No device, TF, calibration, picker state or input file is modified. A below-origin
plane closest to IMU up is a candidate, not a user-identified ground surface.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from inspect_scene_geometry import clean, plane_patches
from wc_calibration.core import CalibrationError
from wc_calibration.level_reference import validate_level_reference
from wc_runtime.prepare_picker_input import project_path, read_json


def sha256(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def require(condition, message):
    if not condition:
        raise CalibrationError(message)


def distribution(xyz):
    finite = np.isfinite(xyz).all(axis=1)
    cloud = xyz[finite]
    ranges = np.linalg.norm(cloud, axis=1)
    return {'original_rows': len(xyz), 'finite_rows': int(finite.sum()),
            'zero_range_rows': int(np.count_nonzero(ranges == 0)),
            'finite_point_ids_sha256': hashlib.sha256(np.flatnonzero(finite).astype('<i8').tobytes()).hexdigest(),
            'xyz_q01_q50_q99_m': np.quantile(cloud, [.01, .5, .99], axis=0) if len(cloud) else None,
            'range_q01_q50_q99_m': np.quantile(ranges, [.01, .5, .99]) if len(cloud) else None}


def rotate_and_check(xyz, rotation):
    finite = np.isfinite(xyz).all(axis=1)
    output = np.full_like(xyz, np.nan, dtype=float)
    output[finite] = xyz[finite] @ rotation.T
    transformed_finite = np.isfinite(output).all(axis=1)
    require(np.array_equal(finite, transformed_finite), 'Rotation changed finite source point IDs')
    before, after = xyz[finite], output[finite]
    delta_range = np.abs(np.linalg.norm(before, axis=1) - np.linalg.norm(after, axis=1))
    delta_pair = np.abs(np.linalg.norm(np.diff(before, axis=0), axis=1) -
                        np.linalg.norm(np.diff(after, axis=0), axis=1))
    maximum = lambda values: float(np.max(values)) if len(values) else 0.0
    require(maximum(delta_range) <= 1e-9 and maximum(delta_pair) <= 1e-9,
            'Rotation failed numerical distance invariance')
    report = {'status': 'NUMERICAL_ROTATION_CHECK_PASS', 'finite_point_ids_and_count_preserved': True,
              'finite_rows': int(finite.sum()), 'source_row_order_preserved': True,
              'max_origin_range_change_m': maximum(delta_range),
              'max_consecutive_source_point_distance_change_m': maximum(delta_pair),
              'comparison_tolerance_m': 1e-9,
              'negative_level_z_finite_rows': int(np.count_nonzero(output[finite, 2] < 0)),
              'level_distribution': distribution(output),
              'note': 'Numerical invariants of this rotation only; not sensor metric accuracy.'}
    return output, report


def left_planes(original, leveled, rotation):
    ranges = np.linalg.norm(original, axis=1)
    mask = np.isfinite(original).all(axis=1) & (ranges > .2) & (ranges < 8) & (original[:, 0] > .05)
    cloud, level_cloud = original[mask], leveled[mask]
    patches = []
    for index, patch in enumerate(plane_patches(cloud)):
        normal = np.asarray(patch['normal'], dtype=float)
        level_normal = rotation @ normal
        offset = float(patch['offset_m'])
        centroid = np.asarray(patch['centroid_m']) @ rotation.T
        if level_normal[2] < 0:
            normal, level_normal, offset = -normal, -level_normal, -offset
        residual_original = cloud @ normal + offset
        residual_level = level_cloud @ level_normal + offset
        change = float(np.max(np.abs(residual_original - residual_level))) if len(cloud) else 0.0
        require(change <= 1e-9, 'Plane residual changed under display rotation')
        patches.append({'patch_index': index, 'point_count': patch['count'],
            'normal_original_left': normal, 'normal_level': level_normal,
            'normal_angle_to_up_deg': float(np.degrees(np.arccos(np.clip(level_normal[2], -1, 1)))),
            'offset_m': offset, 'centroid_level_m': centroid,
            'rmse_m': patch['rmse_m'], 'absolute_residual_p95_m': patch['p95_m'],
            'max_same_point_plane_residual_change_m': change,
            'plane_equation': 'normal_level dot p_level + offset_m = 0',
            'below_left_lidar_origin': bool(centroid[2] < 0)})
    below = [p for p in patches if p['below_left_lidar_origin']]
    candidate = min(below, key=lambda p: (p['normal_angle_to_up_deg'], -p['point_count'])) if below else None
    return {'analysis_points': int(mask.sum()),
            'analysis_mask': 'Finite points, original range 0.2..8 m, original sensor X > 0.05 m; descriptive ROI.',
            'ransac_selection_threshold_m': .03, 'unlabelled_plane_patches': patches,
            'ground_candidate': candidate,
            'ground_candidate_rule': 'Among extracted patches with level centroid Z < 0, choose smallest normal angle to IMU up.',
            'ground_verified': False,
            'limitations': ['No user-labelled ground ROI or surveyed reference; candidate may be furniture or a wall.',
                'Three-centimetre RANSAC selects subsets; small residual does not prove a complete flat ground.',
                'Plane offset is relative to left lidar origin and is not a verified mounting height.',
                'Planes are fitted once and transformed analytically, preserving the same points for residual comparison.']}


def inspect(root, prepared_path):
    prepared_path = project_path(root, prepared_path)
    before_digest = sha256(prepared_path)
    prepared = read_json(root, prepared_path, 128_000_000)
    require(prepared.get('source_mode') == 'real', 'Expected real prepared scene')
    require(len(prepared.get('training', [])) == 1, 'Choose a prepared input with exactly one explicit scene')
    scene = prepared['training'][0]
    metadata = scene['input_files']
    ref = validate_level_reference(scene['level_reference'], scene['id'], metadata)
    rotation = np.asarray(ref['T_level_left'], dtype=float)[:3, :3]
    up = np.asarray(ref['gravity']['up_unit_in_left'], dtype=float)
    require(up.shape == (3,) and np.isfinite(up).all() and abs(np.linalg.norm(up)-1) <= 1e-9,
            'Gravity up must be a finite unit vector')
    leveled_up = rotation @ up
    up_error = float(np.linalg.norm(leveled_up - np.array([0., 0., 1.])))
    require(up_error <= 1e-9, 'Level rotation does not send recorded up to +Z')
    result = {'prepared': str(prepared_path), 'prepared_sha256': before_digest, 'scene_id': scene['id'],
              'level_reference_payload_sha256': ref['payload_sha256'],
              'rotation_check': {'R_times_up': leveled_up, 'up_to_z_error_norm': up_error,
                  'determinant': float(np.linalg.det(rotation)),
                  'orthogonality_max_error': float(np.max(np.abs(rotation.T@rotation-np.eye(3)))),
                  'R_level_left': rotation, 'up_unit_in_left': up,
                  'tilt_from_left_z_deg': float(np.degrees(np.arccos(np.clip(up[2], -1, 1)))),
                  'native_imu_roll_pitch': ref.get('imu_provenance', {}).get('stages', {}).get('left_single', {}).get('native_tilt'),
                  'mounting_rotation_validated': False}, 'sides': {}}
    for side in ('left', 'right'):
        meta = metadata[side]
        npz_path = project_path(root, meta['sources']['npz']['path'])
        file_digest = sha256(npz_path)
        require(file_digest == meta['sources']['npz']['sha256'], side+': NPZ source hash mismatch')
        require(npz_path.stat().st_size <= 80_000_000, 'NPZ exceeds prepared source budget')
        index = meta['frame_index']
        require(type(index) is int and index >= 0, 'Explicit nonnegative source frame index required')
        item = {'source_npz': str(npz_path), 'source_npz_sha256': file_digest,
                'selected_frame_index': index, 'source_session': meta['capture_session'], 'representations': {}}
        with np.load(npz_path, allow_pickle=False) as archive:
            for rep in ('raw', 'filtered'):
                values = archive[rep+'_xyz']
                require(values.ndim == 3 and values.shape[1:] == (9600, 3) and index < len(values),
                        side+': invalid source array shape/index')
                xyz = values[index].astype(float)
                del values
                intensity = archive[rep+'_intensity'][index]
                payload = np.empty((len(xyz), 4), dtype='<f4')
                payload[:, :3], payload[:, 3] = xyz, intensity
                require(hashlib.sha256(payload.tobytes()).hexdigest() == meta[rep+'_frame']['payload_sha256'],
                        side+'/'+rep+': selected frame payload hash mismatch')
                report = {'original_distribution': distribution(xyz)}
                if rep == 'filtered':
                    displayed = np.asarray(scene[side], dtype=float)
                    expected = xyz.copy()
                    expected[(expected == 0).all(axis=1)] = np.nan
                    expected[~np.isfinite(expected)] = np.nan
                    require(displayed.shape == expected.shape and np.allclose(displayed, expected, atol=0, rtol=0, equal_nan=True),
                            side+': prepared coordinates differ from selected filtered source rows')
                    report['prepared_rows_match_selected_source'] = True
                if side == 'left':
                    leveled, report['rotation_invariants'] = rotate_and_check(xyz, rotation)
                    report['planes'] = left_planes(xyz, leveled, rotation)
                else:
                    report['leveling_applied'] = False
                    report['note'] = 'Right sensor native coordinates only; left IMU does not define right sensor up without an extrinsic.'
                item['representations'][rep] = report
        require(sha256(npz_path) == file_digest, 'Source NPZ changed during diagnostic read')
        result['sides'][side] = item
    require(sha256(prepared_path) == before_digest, 'Prepared input changed during diagnostic read')
    return result


def compare(current, previous):
    a, b = current['rotation_check'], previous['rotation_check']
    result = {'status': 'DESCRIPTIVE_DIFFERENCE_ONLY',
              'current_scene': current['scene_id'], 'previous_scene': previous['scene_id'],
              'left_up_direction_change_deg': float(np.degrees(np.arccos(np.clip(
                  np.asarray(a['up_unit_in_left']) @ np.asarray(b['up_unit_in_left']), -1, 1)))),
              'left_tilt_from_z_change_deg': a['tilt_from_left_z_deg']-b['tilt_from_left_z_deg'],
              'ground_candidate_changes': {},
              'note': 'Sensor reinstallation changes its frame; unidentified plane candidates across recordings may be different surfaces.'}
    for rep in ('raw', 'filtered'):
        now = current['sides']['left']['representations'][rep]['planes']['ground_candidate']
        old = previous['sides']['left']['representations'][rep]['planes']['ground_candidate']
        result['ground_candidate_changes'][rep] = None if now is None or old is None else {
            'normal_angle_to_up_change_deg': now['normal_angle_to_up_deg']-old['normal_angle_to_up_deg'],
            'offset_change_m': now['offset_m']-old['offset_m'],
            'p95_residual_change_m': now['absolute_residual_p95_m']-old['absolute_residual_p95_m'],
            'point_count_change': now['point_count']-old['point_count']}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared', type=Path, required=True)
    parser.add_argument('--previous', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    output = project_path(root, args.output)
    require(output.is_relative_to(root/'reports') and not output.exists(), 'New output JSON under project reports required')
    output.parent.mkdir(parents=True, exist_ok=True)
    result = {'schema_version': 1, 'status': 'OFFLINE_NUMERICAL_AND_DESCRIPTIVE_CHECKS',
              'ground_truth_available': False, 'hardware_started': False, 'calibration_or_picker_modified': False,
              'limitations': ['Numerical invariance verifies this display rotation, not hardware or SDK measurement accuracy.',
                  'Ground candidate identity is unverified without user-labelled ROI; residual gates are not accuracy tolerances.',
                  'Raw is before host filters but includes device processing, SDK decoding and lens projection.',
                  'New and previous scene differences do not isolate a causal hardware or software change.']}
    try:
        result['current'] = inspect(root, args.prepared)
        if args.previous is not None:
            result['previous'] = inspect(root, args.previous)
            result['comparison'] = compare(result['current'], result['previous'])
    except (ValueError, OSError, KeyError, IndexError, TypeError, CalibrationError, np.linalg.LinAlgError) as error:
        result.update(status='DIAGNOSTIC_FAILED', error=type(error).__name__+': '+str(error))
    with output.open('x', encoding='utf-8') as stream:
        json.dump(clean(result), stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'status':result['status'], 'output':str(output), 'error':result.get('error')}, ensure_ascii=False))
    return 1 if result['status'] == 'DIAGNOSTIC_FAILED' else 0


if __name__ == '__main__':
    raise SystemExit(main())
