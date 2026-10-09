import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from wc_calibration.core import CalibrationError
from wc_calibration.picker import PickerSession
from wc_runtime import prepare_organized_picker as prep


@pytest.fixture
def capture(tmp_path):
    root = tmp_path/'project'
    folder = root/'reports/capture'
    folder.mkdir(parents=True)
    (folder/'capture.json').write_text(json.dumps({'session': 'synthetic-session',
        'status': 'CAPTURE_AUDIT_PASS', 'physical_static_independently_verified': False,
        'errors': [], 'sides': {'left': {}, 'right': {}}}))
    for side, offset in [('left', 0), ('right', 10)]:
        xyz = np.array([[[1, 2, 3], [4, 5, 6], [0, 0, 0], [np.nan, 8, 9]]]*3, dtype=np.float32)
        amp = np.array([[1, 2, 3, np.nan]]*3, dtype=np.float32)
        host = np.array([1000000000, 2000000000, 3000000000], dtype=np.int64) + offset
        sequence = np.array([4, 5, 6], dtype=np.int64)
        device = np.array([44, 45, 46], dtype=np.int64)
        sdk = np.array([144, 145, 146], dtype=np.int64)
        frames = []
        for index in range(3):
            frames.append({'session_id': 'synthetic-session', 'side': side, 'sensor_id': side+'-SN',
                'epoch': 'epoch-'+side, 'sequence': int(sequence[index]), 'host_ns': int(host[index]),
                'sdk_frame_id': int(sdk[index]), 'device_raw': int(device[index]), 'units': 'm',
                'coordinate_convention': 'FLU', 'point_count': 4,
                'payload_sha256': prep.payload_hash(xyz[index], amp[index]),
                'flags': {'sdk_width': '2', 'sdk_height': '2', 'sdk_point_order': 'row_major',
                    'representation': 'before_host_filter'}})
        (folder/(side+'_frames.json')).write_text(json.dumps({'raw': frames, 'filtered': []}))
        np.savez(folder/(side+'.npz'), raw_xyz=xyz, raw_intensity=amp, host_ns=host,
            frame_sequence=sequence, device_raw=device, sdk_frame_id=sdk)
    return root, folder


def test_prepare_roundtrip_hashes_and_original_indices(capture):
    root, source = capture
    before = {p.name: p.read_bytes() for p in source.iterdir()}
    result = prep.prepare(root, 'reports/capture', 'reports/prepared')
    output = Path(result['prepared_input'])
    prepared = json.loads(output.read_text(encoding='utf-8'))
    assert result['prepared_file_sha256'] == hashlib.sha256(output.read_bytes()).hexdigest()
    assert len(prepared['training']) == 3 and prepared['initial_T_left_right'] is None
    assert prepared['validation'] == []
    scene = prepared['training'][0]
    assert scene['pair_dt_ns'] == 10 and scene['time_quality'] == 'ARRIVAL_NEAREST_NOT_SYNCHRONIZED'
    assert scene['left'][3] == [None, 8., 9.]
    assert scene['organized']['left']['amplitude'] == [1., 2., 3., None]
    session = PickerSession(output, root/'reports/export')
    assert [p['id'] for p in session.scene()['clouds']['left']] == [0, 1]
    assert before == {p.name: p.read_bytes() for p in source.iterdir()}
    for name, record in result['source_files'].items():
        assert record['sha256'] == hashlib.sha256(before[name]).hexdigest()
    with pytest.raises(CalibrationError, match='must not exist'):
        prep.prepare(root, source, output.parent)


@pytest.mark.parametrize('fault', ['session', 'axis', 'unit', 'side', 'payload', 'dimension', 'raw', 'sensor'])
def test_mixed_or_mislabeled_sources_rejected_without_output(capture, fault):
    root, source = capture
    path = source/'right_frames.json'
    value = json.loads(path.read_text()); frame = value['raw'][0]
    if fault == 'session': frame['session_id'] = 'different-session'
    elif fault == 'axis': frame['coordinate_convention'] = 'FRU'
    elif fault == 'unit': frame['units'] = 'mm'
    elif fault == 'side': frame['side'] = 'left'
    elif fault == 'payload': frame['payload_sha256'] = '0'*64
    elif fault == 'dimension': frame['flags']['sdk_height'] = '3'
    elif fault == 'raw': frame['flags']['representation'] = 'filtered'
    elif fault == 'sensor': frame['sensor_id'] = 'other'
    path.write_text(json.dumps(value))
    with pytest.raises(CalibrationError): prep.prepare(root, source, 'reports/rejected')
    assert not (root/'reports/rejected/prepared.json').exists()


def test_npz_time_identity_cannot_be_reordered(capture):
    root, source = capture; path = source/'right.npz'
    with np.load(path) as z: data = {name: z[name] for name in z.files}
    data['host_ns'] = data['host_ns'][::-1]
    np.savez(path, **data)
    with pytest.raises(CalibrationError, match='identity differs'):
        prep.prepare(root, source, 'reports/rejected')


def test_input_mutation_rejected_before_success_file(capture, monkeypatch):
    root, source = capture
    original = prep.read_selected_npz
    def change_input(*args, **kwargs):
        result = original(*args, **kwargs)
        with (source/'capture.json').open('a') as stream: stream.write(' ')
        return result
    monkeypatch.setattr(prep, 'read_selected_npz', change_input)
    with pytest.raises(CalibrationError, match='input changed'):
        prep.prepare(root, source, 'reports/rejected')
    assert not (root/'reports/rejected/prepared.json').exists()


def test_explicit_index_and_time_bound(capture):
    root, source = capture
    result = prep.prepare(root, source, 'reports/single', left_frame_index=1)
    prepared = json.loads(Path(result['prepared_input']).read_text(encoding='utf-8'))
    assert len(prepared['training']) == 1
    assert prepared['training'][0]['organized']['left']['array_frame_index'] == 1
    with pytest.raises(CalibrationError, match='no frame pair'):
        prep.prepare(root, source, 'reports/notime', max_pair_dt_ms=0.)


def test_output_never_overlaps_input_or_code(capture):
    root, source = capture
    for output in ['src/result', source/'nested', root/'reports']:
        with pytest.raises(CalibrationError): prep.prepare(root, source, output)


def test_all_invalid_selected_frame_cannot_claim_ready(capture):
    root, source = capture
    path = source/'left.npz'
    with np.load(path) as z: arrays = {name: z[name] for name in z.files}
    arrays['raw_xyz'][1] = 0
    arrays['raw_xyz'][1, 0] = np.nan
    np.savez(path, **arrays)
    meta_path = source/'left_frames.json'
    meta = json.loads(meta_path.read_text())
    meta['raw'][1]['payload_sha256'] = prep.payload_hash(arrays['raw_xyz'][1], arrays['raw_intensity'][1])
    meta_path.write_text(json.dumps(meta))
    with pytest.raises(CalibrationError, match='no finite nonzero XYZ'):
        prep.prepare(root, source, 'reports/no_valid', left_frame_index=1)
    assert not (root/'reports/no_valid').exists()
