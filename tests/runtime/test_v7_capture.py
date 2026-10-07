import hashlib
from contextlib import closing
import json
from pathlib import Path
import sqlite3
import struct
import threading

import pytest

from wc_runtime.source_archive import SourceJournal, CameraArchive, atomic_json, digest, read_camera_frame
from wc_runtime.capture import capacity, source_commands, RESERVE_BYTES
from wc_runtime.capture_audit import audit_capture
from wc_runtime.ultrasonic_capture import crc16, decode, request, validate_config


def test_journal_drains_and_hashes_tail(tmp_path):
    journal = SourceJournal(tmp_path/'imu', 'imu')
    for index in range(53):
        journal.append({'event': 'frame', 'sequence': index})
    summary = journal.close()
    assert summary['synchronized'] and summary['closed_normally']
    assert summary['event_counts'] == {'frame': 53}
    assert summary['events_sha256'] == digest(tmp_path/'imu/events.jsonl')
    assert list((tmp_path/'imu/events.jsonl').read_text().splitlines())[-1].endswith('52}')


def test_source_overflow_is_not_complete(tmp_path):
    release = threading.Event()
    class SlowJournal(SourceJournal):
        def _run(self):
            release.wait(5)
            super()._run()
    journal = SlowJournal(tmp_path/'source', 'source', max_items=1)
    journal.append({'sequence': 1})
    with pytest.raises(RuntimeError, match='QUEUE_FULL'):
        journal.append({'sequence': 2})
    release.set()
    with pytest.raises(RuntimeError):
        journal.close()
    summary = json.loads((tmp_path/'source/summary.json').read_text())
    assert not summary['synchronized'] and summary['rejected_records'] == 1


def test_camera_preserves_every_unrotated_frame_and_tail(tmp_path):
    archive = CameraArchive(tmp_path/'camera', 'left_front', 'epoch', chunk_bytes=15)
    payloads = [bytes(range(i, i+12)) for i in range(7)]
    for i, payload in enumerate(payloads):
        archive.append(payload, width=2, height=2, sequence=i+1, preview_rotation_deg=180)
    result = archive.close()
    assert result['captured_frames'] == result['persisted_frames'] == 7
    assert not result['exposure_completeness_verified']
    rows = [json.loads(line) for line in (tmp_path/'camera/frames.jsonl').read_text().splitlines()]
    assert len(rows) == 7
    for expected, row in zip(payloads, rows):
        data = read_camera_frame(tmp_path/'camera',row)
        assert data == expected
        assert hashlib.sha256(data).hexdigest() == row['payload_sha256']
    assert result['index_sha256'] == digest(tmp_path/'camera/frames.jsonl')


def test_ultrasound_never_maps_no_response_to_zero():
    assert request(1).hex() == '010300010001d5ca'
    assert decode(b'', 1) == {'status': 'RESPONSE_TIMEOUT', 'distance_m': None}
    body = bytes.fromhex('010302012c')
    good = body+struct.pack('<H', crc16(body))
    assert decode(good, 1)['response_value_u16'] == 300
    assert decode(good, 1)['distance_m'] is None
    assert decode(good, 2)['status'] == 'ADDRESS_MISMATCH'
    assert decode(good[:-1]+b'\0', 1)['status'] == 'CRC_ERROR'
    body = bytes.fromhex('018302')
    assert decode(body+struct.pack('<H', crc16(body)), 1)['status'] == 'MODBUS_EXCEPTION'
    with pytest.raises(ValueError):
        request(0)


def test_capacity_and_capture_do_not_instantiate_motion(tmp_path):
    assert not capacity('mapping_core', 30, RESERVE_BYTES)['sufficient']
    core = capacity('mapping_core', 10, 10**10)
    all_sensors = capacity('all_sensors', 10, 10**10)
    assert all_sensors['estimated_recording_bytes'] > core['estimated_recording_bytes'] + 250_000_000
    project = Path(__file__).resolve().parents[2]
    bindings = json.loads((project/'config/device_bindings.json').read_text(encoding='utf-8'))
    config = tmp_path/'data/configuration'
    config.mkdir(parents=True)
    atomic_json(config/'device_bindings.json', bindings)
    commands = source_commands(tmp_path, tmp_path/'run', tmp_path/'data', 'sample', 'all_sensors')
    assert set(commands) == {'lidar', 'imu', 'wheel', 'ultrasonic', 'camera_left_front',
                             'camera_right_front', 'camera_left_side', 'camera_right_side'}
    serialized = json.dumps(commands)
    assert 'mapping_wheel' not in serialized and 'slam' not in serialized and 'mapping_prior' not in serialized
    assert '--archive-dir' in serialized and '--journal-dir' in serialized
    assert 'preserve_current' in serialized


def write_rows(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows), encoding='utf-8')


def fixture_capture(root):
    records = []
    for side in ('left', 'right'):
        directory = root/'sources/lidar'/side/'epoch'
        write_rows(directory/'source_frames.jsonl', [dict(sensor_id=side, stream_epoch='epoch', sequence=0,
                   raw_sha256='raw'+side, filtered_sha256='filtered'+side)])
        atomic_json(directory/'source_summary.json', dict(synchronized=True, closed_normally=True,
                    published_frames=1, journal_frames=1))
        for representation in ('raw', 'filtered'):
            records.append(dict(source_id=side, stream_epoch='epoch', sequence=0, representation=representation,
                payload_sha256=representation+side, topic='/lidar_'+side+'/'+representation))
    journal = SourceJournal(root/'sources/imu', 'imu')
    journal.append(dict(event='byte_batch', batch_id=1, bytes_hex='01', sha256=hashlib.sha256(b'\x01').hexdigest()))
    journal.append(dict(event='frame', batch_id=1, sensor_id='imu', stream_epoch='epoch', sequence=1, packet_sha256='packet'))
    journal.close()
    records.append(dict(source_id='imu', stream_epoch='epoch', sequence=1, representation='packet', payload_sha256='packet', topic='/imu'))
    wheel_hash = hashlib.sha256(bytes.fromhex('01030400000000fa33')).hexdigest()
    transaction = dict(event='transaction_complete', device_id='wheel', stream_epoch='epoch', sequence=0,
                       response_sha256=wheel_hash, status='RESPONSE_VALID', request_hex='010320ab0002be2b',
                       response_hex='01030400000000fa33', register_words_u16=[0,0])
    write_rows(root/'sources/wheel_feedback.jsonl', [transaction, dict(event='capture_complete', completed=1), dict(event='lease_closed')])
    atomic_json(root/'sources/wheel_summary.json',dict(synchronized=True,closed_normally=True,completed=1,
                journal={'final_fsync_complete':True},events_sha256=digest(root/'sources/wheel_feedback.jsonl')))
    records.append(dict(source_id='wheel', stream_epoch='epoch', sequence=0, representation='transaction', payload_sha256=wheel_hash, topic='/wheel'))
    (root/'bag').mkdir()
    (root/'bag/metadata.yaml').write_text('closed')
    with closing(sqlite3.connect(root/'bag/bag_0.db3')) as conn, conn:
        conn.execute('CREATE TABLE topics(id INTEGER, name TEXT)')
        conn.execute('CREATE TABLE messages(topic_id INTEGER,timestamp INTEGER,data BLOB)')
        for i, row in enumerate(records):
            cdr = bytes([i])
            row.update(storage_stamp_ns=i, cdr_sha256=hashlib.sha256(cdr).hexdigest())
            conn.execute('INSERT INTO topics VALUES (?,?)', (i, row['topic']))
            conn.execute('INSERT INTO messages VALUES (?,?,?)', (i, i, cdr))
    write_rows(root/'records.jsonl', records)
    atomic_json(root/'recorder_summary.json', dict(synchronized=True, closed_normally=True, received=6, persisted=6,
                                                  index_sha256=digest(root/'records.jsonl')))


def test_recording_source_index_and_bag_all_match(tmp_path):
    fixture_capture(tmp_path)
    assert audit_capture(tmp_path, 'mapping_core', dict.fromkeys(('recorder','lidar','imu','wheel'),0))['recording_complete']


@pytest.mark.parametrize('fault', ['bag_tail', 'imu_tail', 'source_close', 'record_hash', 'process_exit', 'missing_devices', 'wheel_payload'])
def test_recording_faults_cannot_pass(tmp_path, fault):
    fixture_capture(tmp_path)
    result_codes = dict.fromkeys(('recorder','lidar','imu','wheel'),0)
    profile = 'mapping_core'
    if fault == 'bag_tail':
        with closing(sqlite3.connect(tmp_path/'bag/bag_0.db3')) as conn, conn:
            conn.execute('DELETE FROM messages WHERE topic_id=5')
    elif fault == 'imu_tail':
        journal = tmp_path/'sources/imu/events.jsonl'
        write_rows(journal, [dict(event='byte_batch', batch_id=1, bytes_hex='01', sha256=hashlib.sha256(b'\x01').hexdigest())])
    elif fault == 'source_close':
        (tmp_path/'sources/lidar/right/epoch/source_summary.json').unlink()
    elif fault == 'record_hash':
        with (tmp_path/'records.jsonl').open('a') as stream:
            stream.write('{}\n')
    elif fault == 'process_exit':
        result_codes['recorder'] = -9
    elif fault == 'missing_devices':
        profile = 'all_sensors'
    elif fault == 'wheel_payload':
        path = tmp_path/'sources/wheel_feedback.jsonl'
        path.write_text(path.read_text().replace('01030400000000fa33','01030400000001fa33'))
        summary = json.loads((tmp_path/'sources/wheel_summary.json').read_text())
        summary['events_sha256'] = digest(path)
        atomic_json(tmp_path/'sources/wheel_summary.json',summary)
    result = audit_capture(tmp_path, profile, result_codes)
    assert not result['recording_complete'] and result['issues']


def test_recorder_recomputes_transaction_bytes():
    from types import SimpleNamespace
    from wc_runtime.source_recorder import message_index
    value = dict(device_id='wheel',stream_epoch='x',sequence=1,response_hex='010203',response_sha256='fake')
    with pytest.raises(ValueError,match='declared source hash'):
        message_index('/wc_mapping/wheel/feedback_raw',SimpleNamespace(data=json.dumps(value)))


def test_reaudit_detects_config_change_and_preserves_original_failure(tmp_path):
    from wc_runtime.capture_audit import verify_archive
    fixture_capture(tmp_path)
    (tmp_path/'config.json').write_text('{"value":1}')
    manifest=dict(profile='mapping_core',recording_complete=True,status='COMPLETE',
                  process_exit_codes=dict.fromkeys(('recorder','lidar','imu','wheel'),0),
                  input_hashes={'config.json':digest(tmp_path/'config.json')})
    atomic_json(tmp_path/'capture_manifest.json',manifest)
    assert verify_archive(tmp_path)['recording_complete']
    (tmp_path/'config.json').write_text('{"value":2}')
    assert not verify_archive(tmp_path)['recording_complete']
    manifest.update(recording_complete=False,input_hashes={})
    atomic_json(tmp_path/'capture_manifest.json',manifest)
    assert not verify_archive(tmp_path)['recording_complete']


def test_all_sensor_preflight_keeps_missing_roles_and_uses_actual_profile(monkeypatch):
    from types import SimpleNamespace
    from wc_runtime import cli, capture, ultrasonic_capture
    from wc_motion import feedback_transport
    from wc_cameras import capture as camera_capture
    monkeypatch.setattr(cli,'device_preflight',lambda: None)
    monkeypatch.setattr(cli,'imu_preflight',lambda: {'mock':True})
    monkeypatch.setattr(feedback_transport,'verify_identity',lambda config:(Path('/mock/wheel'), 'identity'))
    monkeypatch.setattr(feedback_transport,'require_unoccupied',lambda path:None)
    def camera_identity(camera):
        if camera['role'].startswith('left'):
            raise FileNotFoundError('expected USB port absent')
        return {'resolved_node':'/mock/camera'}
    monkeypatch.setattr(camera_capture,'verify_device',camera_identity)
    monkeypatch.setattr(camera_capture,'require_unoccupied',lambda path:None)
    monkeypatch.setattr(ultrasonic_capture,'identity',lambda config:(Path('/mock/ultrasonic'),None))
    monkeypatch.setattr(capture.shutil,'disk_usage',lambda path:SimpleNamespace(free=10**10))
    result=capture.preflight(Path(__file__).resolve().parents[2],'all_sensors',10)
    assert result['camera_left_front']['status']=='BLOCKED'
    assert result['camera_left_side']['status']=='BLOCKED'
    assert result['camera_right_front']['status']=='AVAILABLE'
    assert result['ultrasonic']['evidence']['probe_response']=='NOT_QUERIED'
    assert result['camera_configuration']['profile']=='monitor_320'
    assert result['capacity']['estimated_recording_bytes']==capacity('all_sensors',10,10**10,
        initialization_budget_s=capture.ALL_SOURCE_READY_TIMEOUT_S)['estimated_recording_bytes']
    assert result['capacity']['source_initialization_budget_s']==capture.ALL_SOURCE_READY_TIMEOUT_S
