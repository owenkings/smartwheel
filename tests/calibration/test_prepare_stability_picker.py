"""Synthetic capture-format fixtures only; these tests are not hardware evidence."""
import copy
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

from wc_calibration.core import CalibrationError
from wc_calibration.picker import PickerSession

spec = importlib.util.spec_from_file_location('prepare_stability_picker', Path(__file__).with_name('prepare_stability_picker.py'))
preparation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preparation)
IDS = {'left': 'SYN-L', 'right': 'SYN-R'}


@pytest.fixture
def study(tmp_path):
    root = tmp_path/'synthetic_study'; root.mkdir()
    for side, sensor in IDS.items():
        directory = root/(side+'_single'); directory.mkdir()
        count, pixels = 20, 9600
        shape = np.zeros((count, pixels, 3), dtype='<f4')
        shape[:, :, 0] = np.arange(pixels, dtype=np.float32)[None, :]/1000
        shape[:, :, 1] = np.arange(count, dtype=np.float32)[:, None]*.017
        shape[:, :, 2] = np.arange(count, dtype=np.float32)[:, None]**2*.001+2
        raw = shape.copy(); filtered = shape.copy()
        filtered[9, 17, 0] = np.nan
        filtered[9, 201, 2] = np.inf
        filtered[9, 300, 1] = -np.inf
        filtered[9, 400, :] = 0  # SDK zero-range sentinel, not a real origin point.
        raw_intensity = np.full((count, pixels), 128, dtype='<f4')
        filtered_intensity = raw_intensity.copy()
        times = np.arange(count, dtype=np.int64)*100_000_000+10**18+(0 if side == 'left' else 10_000_000_000)
        sequence = np.arange(count, dtype=np.int64)+5
        arrays = {'raw_xyz': raw, 'filtered_xyz': filtered, 'raw_intensity': raw_intensity,
            'filtered_intensity': filtered_intensity, 'frame_sequence': sequence, 'host_ns': times,
            'device_raw': sequence*100, 'sdk_frame_id': sequence+1000}
        np.savez_compressed(directory/(side+'.npz'), **arrays)
        frames = {'raw': [], 'filtered': []}
        for i in range(count):
            record = {'session_id': side+'-actual-capture-name', 'side': side, 'sensor_id': sensor,
                'sequence': int(sequence[i]), 'epoch': side+'-epoch', 'host_ns': int(times[i]),
                'monotonic_ns': 50000+i*100, 'device_raw': int(sequence[i]*100), 'device_unit': 'unspecified',
                'time_source': 'arrival_only', 'common_time_valid': False, 'units': 'm',
                'coordinate_convention': 'FLU', 'config_hash': ('a' if side == 'left' else 'b')*64,
                'sdk_frame_id': int(sequence[i]+1000), 'point_count': pixels, 'flags': {}}
            raw_record = dict(copy.deepcopy(record), payload_sha256=preparation.payload_hash(raw[i], raw_intensity[i]))
            filtered_record = dict(copy.deepcopy(record), payload_sha256=preparation.payload_hash(filtered[i], filtered_intensity[i]))
            raw_record['flags']['representation'] = 'before_host_filter'
            filtered_record['flags']['representation'] = 'host_filtered'
            filtered_record['config_hash'] = ('c' if side == 'left' else 'd')*64
            filtered_record['flags']['raw_source_config_hash'] = raw_record['config_hash']
            filtered_record['flags']['before_host_filter_cloud_sha256'] = raw_record['payload_sha256']
            frames['raw'].append(raw_record); frames['filtered'].append(filtered_record)
        (directory/(side+'_frames.json')).write_text(json.dumps(frames))
        capture = {'session': record['session_id'], 'status': 'CAPTURE_AUDIT_PASS', 'errors': [],
            'physical_static_independently_verified': False, 'starts_hardware': False, 'starts_motion': False,
            'sides': {side: {'paired': count, 'first': frames['raw'][0], 'last': frames['raw'][-1]}}}
        (directory/'capture.json').write_text(json.dumps(capture))
    return root


def test_exact_midpoint_frames_preserve_nonfinite_slots_and_source_hashes(study, tmp_path):
    result = preparation.prepare_study(study, IDS)
    scene = result['training'][0]
    assert result['status'] == 'PREPARED_NOT_VALIDATED' and result['source_mode'] == 'real'
    assert result['live_eligible'] is False and result['time_validated'] is False
    assert result['validation'] == [] and scene['pair_dt_ns'] is None
    assert scene['time_quality'] == 'NON_SIMULTANEOUS_STATIC_SCENE_ASSUMPTION'
    assert 'selected_raw_frames' not in scene
    assert result['input_preparation']['kind'] == 'sequential_stability_npz_actual_frames'
    assert 'bag_uri' not in result['input_preparation']
    assert scene['static_suggestion']['segments'] == []
    for side in IDS:
        directory = study/(side+'_single')
        with np.load(directory/(side+'.npz'), allow_pickle=False) as arrays:
            actual = arrays['filtered_xyz'][9]
            points = np.asarray(scene[side], dtype=float)
            assert points.shape == (9600, 3)
            excluded = ~np.isfinite(actual) | np.repeat((actual == 0).all(axis=1)[:, None], 3, axis=1)
            assert np.array_equal(points[~excluded], actual[~excluded])
            assert np.array_equal(np.isnan(points), excluded)
            assert scene[side][400] == [None, None, None]
            assert not np.allclose(points[0], np.median(arrays['filtered_xyz'], axis=0)[0])
        meta = scene['input_files'][side]
        assert meta['frame_index'] == 9 and meta['finite_point_count'] == 9597
        assert meta['selectable_point_count'] == 9596 and meta['zero_range_sentinel_count'] == 1
        assert meta['nonfinite_point_count'] == 3
        assert meta['filtered_frame']['sequence'] == 14
        assert meta['raw_frame']['config_hash'] != meta['filtered_frame']['config_hash']
        assert meta['filtered_frame']['flags']['raw_source_config_hash'] == meta['raw_frame']['config_hash']
        for source in meta['sources'].values():
            assert source['sha256'] == preparation.sha256(Path(source['path']))
    assert scene['input_files']['left']['capture_session'] != scene['input_files']['right']['capture_session']
    encoded = json.dumps(result, allow_nan=False)
    source = tmp_path/'prepared.json'; source.write_text(encoded)
    picker = PickerSession(source, tmp_path/'exports')
    shown = picker.scene()
    ids = [p['id'] for p in shown['clouds']['left']]
    assert 16 in ids and 18 in ids and 17 not in ids and 201 not in ids and 300 not in ids and 400 not in ids
    assert ids[-1] == 9599
    assert picker.selection({'input_hash': picker.input_hash, 'scene_id': scene['id'],
        'pairs': [{'left_id': 18, 'right_id': 18}]})['left'][0] == scene['left'][18]


@pytest.mark.parametrize('fault', ['serial', 'missing_metadata', 'pair_hash', 'sequence', 'session', 'units',
    'raw_config_change', 'filtered_config_change', 'capture_failed', 'capture_count', 'payload', 'missing_file',
    'raw_link_missing', 'raw_link_wrong', 'raw_representation', 'filtered_representation', 'bad_filtered_hash'])
def test_mismatched_capture_or_paired_metadata_cannot_be_prepared(study, fault):
    directory = study/'left_single'; path = directory/'left_frames.json'
    frames = json.loads(path.read_text())
    if fault == 'serial':
        frames['raw'][9]['sensor_id'] = frames['filtered'][9]['sensor_id'] = 'OTHER-SERIAL'
    elif fault == 'missing_metadata': frames['filtered'].pop()
    elif fault == 'pair_hash': frames['filtered'][9]['flags'].pop('before_host_filter_cloud_sha256')
    elif fault == 'sequence': frames['raw'][9]['sequence'] = frames['filtered'][9]['sequence'] = 999
    elif fault == 'session': frames['raw'][9]['session_id'] = frames['filtered'][9]['session_id'] = 'UNRELATED'
    elif fault == 'units': frames['raw'][9]['units'] = frames['filtered'][9]['units'] = 'mm'
    elif fault == 'raw_config_change':
        frames['raw'][9]['config_hash'] = 'e'*64
        frames['filtered'][9]['flags']['raw_source_config_hash'] = 'e'*64
    elif fault == 'filtered_config_change': frames['filtered'][9]['config_hash'] = 'e'*64
    elif fault == 'raw_link_missing': frames['filtered'][9]['flags'].pop('raw_source_config_hash')
    elif fault == 'raw_link_wrong': frames['filtered'][9]['flags']['raw_source_config_hash'] = 'f'*64
    elif fault == 'raw_representation': frames['raw'][9]['flags']['representation'] = 'host_filtered'
    elif fault == 'filtered_representation': frames['filtered'][9]['flags']['representation'] = 'before_host_filter'
    elif fault == 'bad_filtered_hash': frames['filtered'][9]['config_hash'] = 'not-a-config-hash'
    elif fault in ('capture_failed', 'capture_count'):
        capture_path = directory/'capture.json'; capture = json.loads(capture_path.read_text())
        if fault == 'capture_failed': capture['status'] = 'FAIL'
        else: capture['sides']['left']['paired'] = 99
        capture_path.write_text(json.dumps(capture))
    elif fault == 'payload':
        npz = directory/'left.npz'
        with np.load(npz, allow_pickle=False) as archive: arrays = {k: archive[k] for k in archive.files}
        arrays['filtered_xyz'][9, 0, 0] += 1
        np.savez_compressed(npz, **arrays)
    else:
        path.unlink()
        with pytest.raises(CalibrationError): preparation.prepare_study(study, IDS)
        return
    path.write_text(json.dumps(frames))
    with pytest.raises(CalibrationError): preparation.prepare_study(study, IDS)


def test_cli_uses_project_identity_reference_and_never_overwrites(study, tmp_path, monkeypatch):
    project = tmp_path/'fake_project'
    (project/'config').mkdir(parents=True)
    config_path = project/'config/live_unvalidated.json'
    config_path.write_text(json.dumps({'source_mode': 'real', 'sensor_ids': IDS}))
    monkeypatch.setattr(preparation, '__file__', str(project/'tests/calibration/prepare_stability_picker.py'))
    output = tmp_path/'prepared.json'
    assert preparation.main(['--study', str(study), '--output', str(output)]) == 0
    saved = output.read_bytes()
    data = json.loads(saved)
    assert data['input_preparation']['identity_reference']['sha256'] == preparation.sha256(config_path)
    assert preparation.main(['--study', str(study), '--output', str(output)]) == 2
    assert output.read_bytes() == saved
    config_path.write_text(json.dumps({'source_mode': 'real', 'sensor_ids': dict(IDS, left='WRONG')}))
    rejected = tmp_path/'rejected.json'
    assert preparation.main(['--study', str(study), '--output', str(rejected)]) == 2
    assert not rejected.exists()
