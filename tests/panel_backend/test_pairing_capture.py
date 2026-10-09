"""Immutable actual-byte fixture preparation; acquisition is never started."""
import hashlib
import json
from pathlib import Path
import sqlite3
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import pytest

from wc_panel import pairing_capture as module


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding='utf-8')


def capture(root, *, organized=True, complete=True, count=4):
    source = root/'data/experiments/static_scene'
    bag = source/'bag'; bag.mkdir(parents=True)
    manifest = dict(session_id='static_scene', status='COMPLETE' if complete else 'PARTIAL',
                    recording_complete=complete, bag_path='bag')
    write_json(source/'capture_manifest.json', manifest)
    decoded, journals = {}, {}
    db = sqlite3.connect(bag/'data.db3')
    db.execute('CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT, type TEXT)')
    db.execute('CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB)')
    for side_index, side in enumerate(('left', 'right')):
        db.execute('INSERT INTO topics VALUES(?,?,?)', (side_index + 1,
            '/wc_mapping/lidar_' + side + '/source_frame', 'wc_interfaces/msg/SourceFrame'))
        rows = []
        for index in range(count):
            points = np.array([[1, 2, 3, 10], [1, 1, 2, 20], [3, 1, 2, 30],
                [1, 3, 2, 40], [4, 2, 3, 50], [0, 0, 0, -1]], dtype='<f4')
            points[:, 0] += side_index * .1
            raw = points.tobytes()
            cloud = NS(width=6, height=1, point_step=16, row_step=96, is_bigendian=False, data=raw,
                fields=[NS(name=name, offset=4*i, datatype=7, count=1) for i, name in enumerate(('x', 'y', 'z', 'intensity'))])
            host = 1_000_000_000 + index * 100_000_000 + side_index * 2_000_000
            mono = host + 10_000_000_000
            flags = ['representation=before_host_filter']
            if organized:
                flags += ['sdk_width=3', 'sdk_height=2', 'sdk_point_order=row_major']
            msg = NS(session_id='static_scene', side=side, sensor_id=side+'-serial', stream_epoch='epoch',
                frame_sequence=index, host_receive_time=NS(sec=host//10**9, nanosec=host%10**9),
                host_monotonic_ns=mono, units='m', coordinate_convention='FLU', diagnostic_flags=flags,
                source_config_hash='a'*64, cloud=cloud)
            token = (side + str(index)).encode()
            decoded[token] = msg
            db.execute('INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)', (side_index+1, host+20_000_000, token))
            rows.append(dict(sensor_id=msg.sensor_id, stream_epoch='epoch', sequence=index,
                host_receive_ns=host, host_monotonic_ns=mono, raw_valid_count=5,
                raw_sha256=hashlib.sha256(raw).hexdigest()))
        journal = source/'sources/lidar'/side/'epoch/source_frames.jsonl'
        journal.parent.mkdir(parents=True)
        journal.write_text('\n'.join(map(json.dumps, rows))+'\n', encoding='utf-8')
        journals[side] = journal
    db.commit(); db.close()
    return source, decoded, journals


@pytest.fixture(autouse=True)
def guards(monkeypatch):
    # Mount parsing is platform-specific; immutable SQLite/data checks remain real.
    monkeypatch.setattr(module, 'resolve_user_destination',
        lambda root, value: (Path(value), mock.Mock(check=lambda: None)))


def tree_hashes(root):
    return {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in root.rglob('*') if p.is_file()}


def test_completed_recording_produces_picker_input_preserves_all_sources(tmp_path):
    source, messages, _ = capture(tmp_path)
    before = tree_hashes(source)
    result = module.prepare_recording(tmp_path, source, decoder=messages.__getitem__)
    assert result['status'] == 'READY_FOR_OFFLINE_PICKING'
    assert result['organized_amplitude_available'] is True
    assert result['static_quality'] == 'MANUAL_STATIC_SCENE_ASSUMPTION'
    assert tree_hashes(source) == before
    path = Path(result['prepared_input'])
    prepared = json.loads(path.read_text(encoding='utf-8'))
    assert len(prepared['training']) == 3 and prepared['initial_T_left_right'] is None
    assert prepared['training'][0]['pair_dt_ns'] == 2_000_000
    assert prepared['training'][0]['left'][-1] == [None, None, None]
    assert prepared['input_preparation']['coordinate_transform_applied'] is False
    assert not (tmp_path/'state/point_picker_input.json').exists()
    from wc_calibration.picker import PickerSession
    (tmp_path/'data/candidates').mkdir()
    session = PickerSession(path, tmp_path/'data/candidates')
    assert session.scene()['source_mode'] == 'real'
    assert session.scene()['organized']['left']['width'] == 3


@pytest.mark.parametrize('status,complete', [('PARTIAL', False), ('RECORDING', False), ('COMPLETE', False), ('PARTIAL', True)])
def test_partial_or_active_capture_never_prepares(tmp_path, status, complete):
    source, messages, _ = capture(tmp_path)
    manifest = source/'capture_manifest.json'
    value = json.loads(manifest.read_text()); value.update(status=status, recording_complete=complete)
    write_json(manifest, value)
    with pytest.raises(ValueError, match='COMPLETE'):
        module.prepare_recording(tmp_path, source, decoder=messages.__getitem__)
    assert not (tmp_path/'reports').exists()


def test_missing_pixel_order_keeps_xyz_without_inventing_amplitude_image(tmp_path):
    source, messages, _ = capture(tmp_path, organized=False)
    result = module.prepare_recording(tmp_path, source, decoder=messages.__getitem__)
    data = json.loads(Path(result['prepared_input']).read_text(encoding='utf-8'))
    assert result['organized_amplitude_available'] is False
    assert all('organized' not in scene and scene['left'] for scene in data['training'])
    assert 'UNAVAILABLE' in data['training'][0]['amplitude_availability']


def test_selected_payload_corruption_rejects_without_output(tmp_path):
    source, messages, _ = capture(tmp_path)
    messages[b'left0'].cloud.data = b'0'*96
    with pytest.raises(ValueError, match='校验值'):
        module.prepare_recording(tmp_path, source, decoder=messages.__getitem__)
    assert not (tmp_path/'reports').exists()


def test_input_change_during_decode_is_detected_before_publication(tmp_path):
    source, messages, journals = capture(tmp_path)
    changed = []
    def decode(raw):
        if not changed:
            with journals['left'].open('a', encoding='utf-8') as stream:
                stream.write('\n')
            changed.append(True)
        return messages[raw]
    with pytest.raises(ValueError, match='发生改变'):
        module.prepare_recording(tmp_path, source, decoder=decode)
    assert not (tmp_path/'reports').exists()


def test_absent_right_raw_messages_rejected(tmp_path):
    source, messages, _ = capture(tmp_path)
    with sqlite3.connect(source/'bag/data.db3') as db:
        db.execute('DELETE FROM messages WHERE topic_id=2')
    with pytest.raises(ValueError, match='right'):
        module.prepare_recording(tmp_path, source, decoder=messages.__getitem__)


def test_same_device_identity_rejected(tmp_path):
    source, messages, journals = capture(tmp_path)
    journals['right'].write_text(journals['right'].read_text().replace('right-serial', 'left-serial'), encoding='utf-8')
    with pytest.raises(ValueError, match='不同的设备身份'):
        module.prepare_recording(tmp_path, source, decoder=messages.__getitem__)


def test_wrong_session_rejected(tmp_path):
    source, messages, _ = capture(tmp_path)
    messages[b'left0'].session_id = 'another-recording'
    with pytest.raises(ValueError, match='会话'):
        module.prepare_recording(tmp_path, source, decoder=messages.__getitem__)


def test_open_database_and_overlapping_destination_rejected(tmp_path):
    source, messages, _ = capture(tmp_path)
    wal = source/'bag/data.db3-wal'; wal.touch()
    with pytest.raises(ValueError, match='尚未关闭'):
        module.prepare_recording(tmp_path, source, decoder=messages.__getitem__)
    wal.unlink()
    with pytest.raises(ValueError, match='覆盖或包含'):
        module.prepare_recording(tmp_path, source, output=source/'prepared', decoder=messages.__getitem__)


def test_disconnected_volume_never_falls_back(tmp_path):
    source, messages, _ = capture(tmp_path)
    with mock.patch.object(module.StoragePolicy, 'check', side_effect=ValueError('USB unavailable')):
        with pytest.raises(ValueError, match='USB unavailable'):
            module.prepare_recording(tmp_path, source, decoder=messages.__getitem__)
    assert not (tmp_path/'reports').exists()


def test_oversized_cdr_rejected_before_loading_or_decoding(tmp_path, monkeypatch):
    source, messages, _ = capture(tmp_path)
    monkeypatch.setattr(module, 'MAX_CDR_BYTES', 8)
    with sqlite3.connect(source/'bag/data.db3') as db:
        db.execute('UPDATE messages SET data=zeroblob(9) WHERE id=1')
    decoder = mock.Mock(side_effect=messages.__getitem__)
    with pytest.raises(ValueError, match='CDR'):
        module.prepare_recording(tmp_path, source, decoder=decoder)
    decoder.assert_not_called()
