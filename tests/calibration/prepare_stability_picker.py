#!/usr/bin/env python3
"""Prepare actual sequential single-lidar frames for offline manual picking only.

The rig and cabinet must have remained unchanged between captures. No median
cloud, common session, synchronized time, TF or production calibration is made.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import zipfile

import numpy as np

from wc_calibration.core import CalibrationError

PIXELS = 9600
ARRAYS = {'raw_xyz', 'filtered_xyz', 'raw_intensity', 'filtered_intensity',
          'frame_sequence', 'host_ns', 'device_raw', 'sdk_frame_id'}
TIME_QUALITY = 'NON_SIMULTANEOUS_STATIC_SCENE_ASSUMPTION'


def require(condition, message):
    if not condition:
        raise CalibrationError(message)


def sha256(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            result.update(chunk)
    return result.hexdigest()


def read_json(path):
    require(path.is_file() and path.stat().st_size <= 10_000_000, 'missing or oversized metadata: '+str(path))
    raw = path.read_bytes()
    value = json.loads(raw.decode('utf-8'))
    json.dumps(value, allow_nan=False)
    return value, {'path': str(path.resolve()), 'sha256': hashlib.sha256(raw).hexdigest(), 'bytes': len(raw)}


def is_sha(value):
    return isinstance(value, str) and len(value) == 64 and all(c in '0123456789abcdef' for c in value)


def payload_hash(xyz, intensity):
    payload = np.empty((PIXELS, 4), dtype='<f4')
    payload[:, :3], payload[:, 3] = xyz, intensity
    return hashlib.sha256(payload.tobytes()).hexdigest()


def prepare_side(study, side, expected_id):
    directory = study/(side+'_single')
    capture, capture_source = read_json(directory/'capture.json')
    frames, frames_source = read_json(directory/(side+'_frames.json'))
    require(isinstance(capture, dict) and capture.get('status') == 'CAPTURE_AUDIT_PASS' and capture.get('errors') == [],
            side+': a successful capture audit is required, without implying static-scene verification')
    require(isinstance(capture.get('session'), str) and bool(capture['session']), side+': missing capture session')
    require(isinstance(capture.get('sides'), dict) and set(capture['sides']) == {side}, side+': expected a single-side capture')
    require(isinstance(frames, dict) and set(frames) == {'raw', 'filtered'} and
            all(isinstance(frames[rep], list) for rep in frames), side+': paired raw/filtered metadata required')
    path = directory/(side+'.npz')
    require(path.is_file() and path.stat().st_size <= 80_000_000, side+': missing or oversized NPZ')
    with zipfile.ZipFile(path) as archive:
        entries = archive.infolist()
        require({entry.filename for entry in entries} == {name+'.npy' for name in ARRAYS} and len(entries) == len(ARRAYS)
                and sum(entry.file_size for entry in entries) <= 80_000_000, side+': NPZ layout/expanded byte budget mismatch')
    digest = sha256(path)
    with np.load(path, allow_pickle=False) as archive:
        arrays = {key: archive[key] for key in ARRAYS}
    require(sha256(path) == digest, side+': NPZ changed during preparation')
    xyz = arrays['filtered_xyz']
    require(xyz.ndim == 3 and xyz.shape[1:] == (PIXELS, 3) and 20 <= len(xyz) <= 500,
            side+': expected 20..500 actual frames with all 9600 XYZ slots')
    count = len(xyz)
    for rep in ('raw', 'filtered'):
        for field, shape in (('xyz', (count, PIXELS, 3)), ('intensity', (count, PIXELS))):
            value = arrays[rep+'_'+field]
            require(value.shape == shape and value.dtype.kind == 'f' and value.dtype.itemsize == 4,
                    side+': float32 XYZ/intensity capture representation required')
        require(len(frames[rep]) == count, side+': missing paired frame metadata')
    for key in ('frame_sequence', 'host_ns', 'device_raw', 'sdk_frame_id'):
        require(arrays[key].shape == (count,) and arrays[key].dtype.kind in 'iu', side+': integer source vectors required')
    require(capture['sides'][side].get('paired') == count, side+': capture count differs from exact paired arrays')
    matching = ('session_id', 'side', 'sensor_id', 'sequence', 'epoch', 'host_ns', 'monotonic_ns',
        'device_raw', 'device_unit', 'time_source', 'common_time_valid', 'units', 'coordinate_convention',
        'sdk_frame_id', 'point_count')
    for index, (raw, filtered) in enumerate(zip(frames['raw'], frames['filtered'])):
        require(isinstance(raw, dict) and isinstance(filtered, dict) and all(k in raw and k in filtered for k in matching),
                side+': incomplete actual source frame metadata')
        require(all(raw[k] == filtered[k] for k in matching), side+': raw/filtered source identity or time mismatch')
        require(raw['session_id'] == capture['session'] and raw['side'] == side and raw['sensor_id'] == expected_id,
                side+': capture/session/serial mismatch')
        require(raw['units'] == 'm' and raw['coordinate_convention'] == 'FLU' and raw['point_count'] == PIXELS,
                side+': source units/coordinates/layout mismatch')
        require(isinstance(raw['epoch'], str) and bool(raw['epoch']) and
                is_sha(raw.get('config_hash')) and is_sha(filtered.get('config_hash')) and
                is_sha(raw.get('payload_sha256')) and is_sha(filtered.get('payload_sha256')),
                side+': missing epoch or actual source hashes')
        # xt_source_node.cpp deliberately hashes the two representations
        # separately. The filtered record carries the explicit raw-config link.
        require(isinstance(raw.get('flags'), dict) and isinstance(filtered.get('flags'), dict) and
                raw['flags'].get('representation') == 'before_host_filter' and
                filtered['flags'].get('representation') == 'host_filtered' and
                filtered['flags'].get('raw_source_config_hash') == raw['config_hash'],
                side+': representation or raw-source configuration link missing/mismatched')
        require(filtered['flags'].get('before_host_filter_cloud_sha256') == raw['payload_sha256'],
                side+': raw/filtered pairing hash missing or mismatched')
        for vector, field in (('frame_sequence', 'sequence'), ('host_ns', 'host_ns'), ('device_raw', 'device_raw'), ('sdk_frame_id', 'sdk_frame_id')):
            require(type(raw[field]) is int and raw[field] == int(arrays[vector][index]), side+': source vector differs from metadata')
    require(capture['sides'][side].get('first') == frames['raw'][0] and
            capture['sides'][side].get('last') == frames['raw'][-1], side+': capture endpoint metadata mismatch')
    require(len({row['epoch'] for row in frames['raw']}) == 1, side+': stream epoch changed during capture')
    for rep in ('raw', 'filtered'):
        require(len({row['config_hash'] for row in frames[rep]}) == 1,
                side+': '+rep+' configuration changed during capture')
    times = [int(value) for value in arrays['host_ns']]
    sequences = [int(value) for value in arrays['frame_sequence']]
    require(times[0] > 0 and all(b > a for a, b in zip(times, times[1:])) and
            sequences[0] >= 0 and all(b > a for a, b in zip(sequences, sequences[1:])), side+': nonmonotonic frame order')
    # Integer twice-distance avoids rounding Unix nanoseconds to float. Ties use
    # the earlier original array index; this always selects one actual frame.
    chosen = min(range(count), key=lambda i: (abs(2*times[i]-times[0]-times[-1]), i))
    for rep in ('raw', 'filtered'):
        require(payload_hash(arrays[rep+'_xyz'][chosen], arrays[rep+'_intensity'][chosen]) == frames[rep][chosen]['payload_sha256'],
                side+': selected NPZ frame does not match its recorded payload hash')
    selected = xyz[chosen]
    finite = np.isfinite(selected).all(axis=1)
    zero_range = finite & (selected == 0).all(axis=1)
    selectable = finite & ~zero_range
    points = [[None, None, None] if zero_range[index] else
              [float(value) if np.isfinite(value) else None for value in row]
              for index, row in enumerate(selected)]
    require(selectable.any(), side+': selected frame has no finite nonzero-range points')
    metadata = {'sensor_id': expected_id, 'units': 'm', 'coordinate_convention': 'FLU',
        'representation': 'actual_filtered_frame_not_temporal_median', 'frame_index': chosen,
        'original_point_count': PIXELS, 'finite_point_count': int(finite.sum()),
        'selectable_point_count': int(selectable.sum()), 'zero_range_sentinel_count': int(zero_range.sum()),
        'nonfinite_point_count': int((~finite).sum()),
        'selection_method': 'nearest_integer_host_time_midpoint_tie_earlier_frame',
        'capture_session': capture['session'], 'raw_frame': frames['raw'][chosen], 'filtered_frame': frames['filtered'][chosen],
        'sources': {'npz': {'path': str(path.resolve()), 'sha256': digest, 'bytes': path.stat().st_size},
                    'frames': frames_source, 'capture': capture_source}}
    return points, metadata


def prepare_study(study, expected_sensor_ids, *, identity_reference=None):
    study = Path(study).resolve(strict=True)
    require(isinstance(expected_sensor_ids, dict) and set(expected_sensor_ids) == {'left', 'right'} and
            all(isinstance(v, str) and v for v in expected_sensor_ids.values()) and
            expected_sensor_ids['left'] != expected_sensor_ids['right'], 'explicit distinct expected serial identities required')
    points, metadata = {}, {}
    for side in ('left', 'right'):
        points[side], metadata[side] = prepare_side(study, side, expected_sensor_ids[side])
    require(metadata['left']['capture_session'] != metadata['right']['capture_session'],
            'independent sequential captures must retain their different sessions')
    return {'schema_version': 1, 'status': 'PREPARED_NOT_VALIDATED', 'source_mode': 'real',
        'sensor_ids': dict(expected_sensor_ids), 'units': 'm', 'coordinate_conventions': {'left': 'FLU', 'right': 'FLU'},
        'live_eligible': False, 'time_validated': False, 'independent_validation_performed': False,
        'training': [{'id': study.name+'_sequential_single_frames', **points, 'input_files': metadata,
            'time_quality': TIME_QUALITY, 'pair_dt_ns': None,
            'static_suggestion': {'status': 'ASSUMPTION_ONLY', 'static_validated': False, 'segments': []}}],
        'validation': [], 'initial_T_left_right': None,
        'input_preparation': {'kind': 'sequential_stability_npz_actual_frames', 'study_path': str(study),
            'capture_sessions': {s: metadata[s]['capture_session'] for s in metadata},
            'identity_reference': identity_reference, 'time_quality': TIME_QUALITY,
            'static_review_required': True, 'automatic_static_evidence': False,
            'precondition': 'Rig, sensor mounts and cabinet must have remained unchanged between these captures.',
            'limitations': ['Sequential frames are not simultaneous observations or time-calibration evidence.',
                'Only manual correspondence initialization is supported; no independent geometry validation is inferred.',
                'All 9600 original point indices are preserved; nonfinite coordinates become JSON null and zero-range sentinel rows become [null,null,null].']}}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--study', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        config, source = read_json(Path(__file__).resolve().parents[2]/'config/live_unvalidated.json')
        require(config.get('source_mode') == 'real', 'expected identity configuration must describe real sources')
        data = prepare_study(args.study, config['sensor_ids'], identity_reference=source)
        output = args.output.absolute()
        require(not any(p.is_symlink() for p in (output, *output.parents)), 'output must not traverse symlinks')
        with output.open('x', encoding='utf-8') as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
        print(json.dumps({'status': data['status'], 'output': str(output), 'live_eligible': False,
            'time_quality': TIME_QUALITY, 'selected_frame_indices': {s: data['training'][0]['input_files'][s]['frame_index'] for s in ('left', 'right')}}))
        return 0
    except (CalibrationError, OSError, ValueError, KeyError, TypeError, zipfile.BadZipFile) as error:
        print(json.dumps({'error': type(error).__name__, 'detail': str(error)}), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
