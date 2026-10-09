"""Read-only, identity-checked inputs and evidence for offline refinement."""
import copy
import hashlib
import json
import os
from pathlib import Path
import re
import sqlite3

import numpy as np

from .offline_motion_calibration import build_candidate, select_windows, validate_for_session


SELECTED_SUFFIX = '#IMU_and_wheel_selected_serialized_messages'
HASH_DEFINITION = ('uint64 little-endian bag message ID then serialized payload, only selected '
                   'IMU and wheel topics, message ID order within lexicographically sorted bag parts; '
                   'not entire bag hash')
TOPICS = {'/wc_mapping/imu/source_frame': 'wc_interfaces/msg/H30Frame',
          '/wc_mapping/wheel/feedback_raw': 'std_msgs/msg/String'}


def _require(value, reason):
    if not value:
        raise ValueError(reason)


def _path(value):
    p = Path(value).absolute()
    _require('..' not in p.parts and not any(v.is_symlink() for v in (p, *p.parents)),
             'evidence paths must not contain traversal or symlinks')
    return p


def _digest(path):
    h = hashlib.sha256()
    with _path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


def _write(path, value):
    with _path(path).open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    return _digest(path)


def _expected_sha(value):
    if isinstance(value, dict):
        value = value.get('sha256')
    _require(isinstance(value, str) and re.fullmatch('[0-9a-f]{64}', value),
             'evidence SHA-256 must be a lowercase 64-digit digest')
    return value


def recording_session(source, runtime=None):
    """Bind to the frozen capture identity, independent of the display folder.

    Old evidence-only callers without archived configuration keep their former
    folder-name contract; a candidate cannot declare its own new identity.
    """
    source = _path(source)
    manifest_path = source/'capture_manifest.json'
    def read(path):
        path = _path(path)
        _require(path.stat().st_size <= 2*1024*1024, 'oversized recording identity configuration')
        value = json.loads(path.read_text(encoding='utf-8'))
        _require(isinstance(value,dict), 'recording identity configuration must be an object')
        return value
    manifest = read(manifest_path) if manifest_path.is_file() else None
    relative = Path(manifest.get('runtime_config_path','runtime_config.json') if manifest else 'runtime_config.json')
    _require(not relative.is_absolute() and '..' not in relative.parts, 'archived runtime path escapes dataset')
    frozen_path = _path(source/relative)
    frozen = read(frozen_path) if frozen_path.is_file() else None
    session = manifest.get('session_id') if manifest else (frozen.get('session_id',source.name) if frozen else source.name)
    _require(isinstance(session,str) and re.fullmatch('[A-Za-z0-9][A-Za-z0-9_.-]{0,119}',session),
             'valid archived capture session identity required')
    # Capture snapshots historically preserve mapping defaults without an ID;
    # load_dataset establishes it from the actual recorded SourceFrame stream.
    if frozen is not None and 'session_id' in frozen:
        _require(frozen.get('session_id') == session, 'frozen runtime and capture manifest session mismatch')
    if runtime is not None:
        _require(runtime.get('session_id') == session, 'runtime session differs from archived recording')
    return session


def selected_message_evidence(source, *, packets=None):
    """Hash original serialized bytes, never retimed/deserialized packet objects."""
    source = _path(source)
    bags = sorted((source/'bag').glob('*.db3'))
    _require(bags, 'source bag parts unavailable')
    h = hashlib.sha256(); counts = dict.fromkeys(TOPICS, 0); metadata = []
    indexed = None
    if packets is not None:
        indexed = {(e['bag'], e['message_id']): e for e in packets.events
                   if e['category'] in ('imu', 'wheel')}
        _require(len(indexed) == sum(e['category'] in ('imu', 'wheel') for e in packets.events),
                 'duplicate selected recorded event identity')
    seen = set()
    for bag in bags:
        bag = _path(bag); before = bag.stat()
        with sqlite3.connect(bag.as_uri()+'?mode=ro', uri=True) as db:
            db.execute('PRAGMA query_only=ON')
            definitions = list(db.execute('SELECT id,name,type FROM topics'))
            chosen = {i: name for i, name, typ in definitions if name in TOPICS}
            for name in TOPICS:
                found = [(i, typ) for i, n, typ in definitions if n == name]
                _require(len(found) <= 1, 'duplicate selected topic definition')
                _require(not found or found[0][1] == TOPICS[name], 'selected topic type mismatch')
            if chosen:
                query = ('SELECT id,topic_id,data FROM messages WHERE topic_id IN ('+
                         ','.join('?' for _ in chosen)+') ORDER BY id')
                for mid, tid, payload in db.execute(query, list(chosen)):
                    _require(type(mid) is int and 0 <= mid < 2**64, 'invalid bag message ID')
                    key = (bag.name, mid); seen.add(key)
                    if indexed is not None:
                        event = indexed.get(key)
                        _require(event is not None and event['topic'] == chosen[tid] and
                                 event['cdr_sha256'] == hashlib.sha256(payload).hexdigest(),
                                 'selected original message differs from verified event index')
                    h.update(mid.to_bytes(8, 'little')); h.update(payload)
                    counts[chosen[tid]] += 1
        after = bag.stat()
        _require((before.st_size, before.st_mtime_ns, before.st_ino) ==
                 (after.st_size, after.st_mtime_ns, after.st_ino), 'bag changed during evidence read')
        metadata.append({'path': str(bag), 'size': before.st_size,
                         'mtime_ns': before.st_mtime_ns, 'read_only': True})
    _require(all(counts.values()), 'both original IMU and wheel topics required')
    if indexed is not None:
        _require(seen == set(indexed), 'selected event index does not cover this exact source bag')
    return {'path': str(source/'bag')+SELECTED_SUFFIX, 'sha256': h.hexdigest(),
            'hash_definition': HASH_DEFINITION}, metadata, counts


def _runtime_identity(runtime):
    prior = runtime.get('prior_template', runtime)
    imu = runtime.get('imu_sensor_id', prior.get('imu_sensor_id'))
    wheel = runtime.get('wheel_device_id', prior.get('wheel_device_id'))
    conversion = runtime.get('wheel_candidate', prior.get('wheel_candidate'))
    R = runtime.get('imu_mount', {}).get('R_axle_imu')
    if R is None and 'R_base_imu' in prior:
        _require('T_base_axle' in prior or
                 (prior.get('mount_model') == 'forward_aligned_axle' and prior.get('same_forward') is True),
                 'explicit axle rotation or forward-aligned mounting declaration required')
        base_axle = np.asarray(prior.get('T_base_axle', np.eye(4)), dtype=float)
        R = base_axle[:3, :3].T @ np.asarray(prior['R_base_imu'], dtype=float)
    R = np.asarray(R, dtype=float)
    _require(isinstance(imu, str) and isinstance(wheel, str) and isinstance(conversion, dict),
             'frozen runtime identities and wheel conversion required')
    _require(R.shape == (3, 3) and np.isfinite(R).all() and
             np.allclose(R.T @ R, np.eye(3), atol=1e-8) and
             np.isclose(np.linalg.det(R), 1., atol=1e-8), 'explicit proper runtime IMU rotation required')
    return imu, wheel, conversion, R


def motion_sample_time_s(event, clock):
    """Use the replay's exact IMU model; wheel and legacy IMU keep host time."""
    if event['category'] == 'imu' and 'imu_measurement_stamp_ns' in event:
        model = clock.report().get('imu_time_model')
        _require(model is not None and event.get('imu_time_model_sha256') == model['model_sha256'],
                 'calibration IMU event differs from frozen timing model')
        return (event['imu_measurement_stamp_ns']-clock.ros_origin_ns)*1e-9
    _require(event['category'] != 'imu' or 'imu_time_model' not in clock.report(),
             'device-relative calibration requires every derived IMU stamp')
    return (event['monotonic_ns']-clock.monotonic_origin_ns)*1e-9


def extract_and_calibrate(source, runtime, packets, output, *, input_hashes=None):
    """Freeze evidence, select disjoint windows, and save candidate or rejection.

    output is a new calibration evidence directory outside the original session.
    This performs no ROS publication or device access. Input hashes are the
    caller's already verified input manifest; original CDR is checked again.
    """
    from wc_motion.history_preview import HistoryPreview
    source, output = _path(source), _path(output)
    _require(not output.is_relative_to(source) and not source.is_relative_to(output),
             'calibration output overlaps original recording')
    output.mkdir(parents=True, exist_ok=False)
    try:
        session = recording_session(source, runtime)
        imu_id, wheel_id, conversion, R = _runtime_identity(runtime)
        _require(runtime.get('confirmed_gyro_bias') is None and
                 runtime.get('prior_template', {}).get('confirmed_gyro_bias') is None and
                 not runtime.get('offline_motion_correction'), 'raw uncorrected runtime required')
        original, bag_meta, counts = selected_message_evidence(source, packets=packets)
        runtime_path = output/'input_runtime_snapshot.json'
        runtime_sha = _write(runtime_path, runtime)
        manifest_path = output/'verified_input_hashes.json'
        manifest = {str(k): _expected_sha(v) for k, v in (input_hashes or {}).items()}
        manifest_sha = _write(manifest_path, manifest)
        provenance = {'session_id': session, 'imu_device_id': imu_id, 'wheel_device_id': wheel_id,
            'sources': [original, {'path': str(runtime_path), 'sha256': runtime_sha},
                        {'path': str(manifest_path), 'sha256': manifest_sha}],
            'R_reference_imu': R.tolist(), 'time_mapping': packets.clock.report(),
            'bag_read_evidence': bag_meta, 'wheel_conversion': copy.deepcopy(conversion),
            'extractor_code_sha256': _digest(__file__), 'selected_message_counts': counts}
        hp = HistoryPreview(conversion, continuous=True)
        imu, wheel = [], []; previous = {}; epochs = {}
        origin = packets.clock.monotonic_origin_ns
        for event in packets.events:
            kind = event['category']
            if kind not in ('imu', 'wheel'):
                continue
            key = (event['monotonic_ns'], event['sequence'])
            _require(key[0] >= origin and (kind not in previous or
                     (key[0] >= previous[kind][0] and key[1] > previous[kind][1])),
                     'motion event time/sequence discontinuity')
            previous[kind] = key
            msg = packets.get(event)  # Checks original CDR hash; only returned copy is retimed.
            t = motion_sample_time_s(event, packets.clock)
            if kind == 'imu':
                epoch = msg.stream_epoch
                _require(isinstance(epoch, str) and epoch and
                         (kind not in epochs or epochs[kind] == epoch), 'IMU stream epoch changed')
                epochs[kind] = epoch
                _require(msg.sensor_id == imu_id and msg.session_id == session and
                         msg.angular_velocity_valid and msg.linear_acceleration_valid and
                         msg.coordinate_convention == 'H30_NATIVE_UNVALIDATED' and
                         msg.time_source == 'arrival_only' and msg.common_time_valid is False and
                         int(msg.host_monotonic_ns) == key[0] and int(msg.frame_sequence) == key[1],
                         'original IMU identity/validity/time mismatch')
                g, a = msg.imu.angular_velocity, msg.imu.linear_acceleration
                imu.append({'t_s': t, 'gyro_native_rad_s': [g.x, g.y, g.z],
                            'accel_native_m_s2': [a.x, a.y, a.z], 'device_id': imu_id,
                            'session_id': session})
            else:
                row = json.loads(msg.data)
                _require(row['device_id'] == wheel_id and row['session_id'] == session and
                         row['receive_monotonic_ns'] == key[0] and row['sequence'] == key[1],
                         'original wheel identity/time mismatch')
                sample = hp.update(row)
                wheel.append({'t_s': t, 'v_m_s': sample['linear_velocity_m_s'],
                              'w_rad_s': sample['angular_velocity_rad_s'], 'device_id': wheel_id,
                              'session_id': session})
        provenance['raw_samples'] = {'imu': len(imu), 'wheel': len(wheel)}
        windows = select_windows(imu, wheel, provenance=provenance)
        _write(output/'windows.json', windows)
        candidate = build_candidate(imu, wheel, provenance=provenance, windows=windows)
        path = output/'candidate.json'; digest = _write(path, candidate)
        _require(candidate.get('validation', {}).get('passed') is True,
                 'offline candidate rejected: '+str(candidate.get('validation')))
        verify_candidate(candidate, path, source, input_hashes or {}, runtime=runtime, packets=packets)
        return candidate, path, digest
    except Exception as error:
        _write(output/'rejection.json', {'status': 'REJECTED', 'error_type': type(error).__name__,
            'error': str(error), 'source': str(source), 'hardware_started': False,
            'original_recording_modified': False})
        raise


def verify_candidate(candidate, path, source, input_hashes, *, runtime=None, packets=None):
    """Verify a candidate file and every declared source; reject arbitrary fragments."""
    source, path = _path(source), _path(path)
    _require(json.loads(path.read_text(encoding='utf-8')) == candidate,
             'candidate object differs from archived candidate file')
    p = candidate.get('provenance', {})
    imu_id, wheel_id = p.get('imu_device_id'), p.get('wheel_device_id')
    session = recording_session(source, runtime)
    if runtime is not None:
        imu_id, wheel_id, conversion, R = _runtime_identity(runtime)
        _require(p.get('wheel_conversion') == conversion and
                 np.allclose(np.asarray(p.get('R_reference_imu')), R, atol=1e-9),
                 'candidate wheel conversion or IMU axes differ from frozen runtime')
    validate_for_session(candidate, session_id=session, imu_device_id=imu_id, wheel_device_id=wheel_id)
    if packets is not None:
        _require(p.get('time_mapping') == packets.clock.report(),
                 'candidate time origin differs from recorded packet clock')
    known = {str(_path(k)): _expected_sha(v) for k, v in input_hashes.items()}
    verified = []; selected = None; identities = set()
    expected_pseudo = str(source/'bag')+SELECTED_SUFFIX
    for evidence in p['sources']:
        value, expected = evidence['path'], _expected_sha(evidence['sha256'])
        _require(value not in identities, 'duplicate candidate evidence path'); identities.add(value)
        if '#' in value:
            _require(value == expected_pseudo, 'unsupported pseudo-path in candidate evidence')
            selected, _, _ = selected_message_evidence(source, packets=packets)
            actual = selected['sha256']
            declaration = evidence.get('hash_definition', '')
            _require('uint64 little-endian bag message ID then serialized payload' in declaration and
                     'only selected IMU and wheel topics' in declaration,
                     'selected-message evidence hash definition missing or unsupported')
        else:
            actual_path = _path(value)
            _require(actual_path.is_file(), 'candidate source file missing: '+value)
            if str(actual_path) in known:
                _require(known[str(actual_path)] == expected, 'candidate differs from verified input manifest')
            actual = _digest(actual_path)
        _require(actual == expected, 'candidate source SHA-256 mismatch: '+value)
        verified.append({'path': value, 'sha256': actual})
    _require(selected is not None, 'candidate must bind this dataset selected original IMU and wheel messages')
    return {'status': 'VERIFIED', 'candidate_path': str(path), 'candidate_sha256': _digest(path),
            'session_id': session, 'verified_sources': verified,
            'physical_calibration': False, 'original_recording_modified': False}
