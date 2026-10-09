"""Synthetic device-time replay checks; no hardware or physical-sync claims."""
import copy
import hashlib
import json
from pathlib import Path
import sqlite3
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

from wc_runtime.compare_replay import RecordedPackets, stamp
from wc_runtime.offline_imu_time import build_model, configure_runtime, prior_arguments, resolved_mode
from wc_runtime.source_time import SourceClock
from wc_runtime.mapping_prior import MotionPrior, PriorError
from wc_runtime.offline_refinement_inputs import motion_sample_time_s

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'operations'))
from test_mapping_prior import config as prior_config, imu


def freeze_profile(source, **changes):
    directory = source/'configuration'; directory.mkdir(exist_ok=True)
    evidence = b'SYNTHETIC TEST EVIDENCE ONLY'
    (directory/'evidence.bin').write_bytes(evidence)
    profile = dict(schema_version=1, status='VERIFIED_DEVICE_RELATIVE_TIME',
        sensor_id='H30-0000000015', hardware_serial='0000000015', product_version='SYNTHETIC',
        device_timestamp_unit='us', tick_ns=1000,
        required_fields=['sample_timestamp_raw','dataready_timestamp_raw'],
        backward_policy='guarded_uint32_wrap', duplicate_policy='reject', read_mode='event',
        max_gap_ticks=2_000_000, wrap_host_tolerance_ns=100_000_000,
        evidence=[dict(path='evidence.bin', sha256=hashlib.sha256(evidence).hexdigest())])
    profile.update(changes)
    raw = json.dumps(profile).encode()
    (directory/'imu_timing.json').write_bytes(raw)
    return hashlib.sha256(raw).hexdigest()


def runtime(source, mode='device_relative'):
    return dict(session_id=source.name, mode='all', cloud_source='raw', offline_imu_time_mode=mode,
        prior_template=dict(imu_sensor_id='H30-0000000015', imu_topic='/imu', wheel_topic='/wheel'))


def events(profile_sha, *, samples=None, host=None):
    samples = list(range(0, 40000, 5000)) if samples is None else samples
    host = [1_010_000_000+(i//2)*10_000_000 for i in range(len(samples))] if host is None else host
    return [dict(category='imu', sequence=i, monotonic_ns=mono, stamp_ns=100_000_000_000+mono-1_000_000_000,
        cdr_sha256=hashlib.sha256(str(i).encode()).hexdigest(),
        imu_device_time=dict(sensor_id='H30-0000000015', session_id='original', stream_epoch='epoch',
            device_timestamp_unit='us', device_time_status='VALID_DEVICE_RELATIVE_TIME',
            timing_profile_sha256=profile_sha, sample_timestamp_valid=True, sample_timestamp_raw=sample,
            dataready_timestamp_valid=True, dataready_timestamp_raw=(sample+627)%2**32))
        for i, (sample, mono) in enumerate(zip(samples, host))]


@pytest.fixture
def original(tmp_path):
    source = tmp_path/'original'; source.mkdir()
    return source


def test_batched_arrivals_become_real_sample_intervals_and_zero_is_valid(original):
    sha = freeze_profile(original); rows = events(sha); before = copy.deepcopy(rows)
    model = build_model(rows, original, runtime(original))
    assert np.diff([row['imu_measurement_stamp_ns'] for row in rows]).tolist() == [5_000_000]*7
    assert all(row['imu_measurement_stamp_ns'] <= row['stamp_ns'] for row in rows)
    for row, old in zip(rows, before):
        assert row['imu_device_time'] == old['imu_device_time']
        assert row['stamp_ns'] == old['stamp_ns'] and row['monotonic_ns'] == old['monotonic_ns']
    assert model['common_measurement_time_validated'] is False
    assert model['first_sample_timestamp_raw'] == 0
    assert model['intersensor_fixed_latency'] == 'UNKNOWN'


def test_wrap_is_shared_guarded_and_counters_can_cross_on_different_frames(original):
    sha = freeze_profile(original)
    samples = [2**32-6000, 2**32-1000, 4000]
    rows = events(sha, samples=samples, host=[1_000_000_000,1_005_000_000,1_010_000_000])
    # Ready crosses one frame before sample; neither counter borrows the other's epoch.
    rows[0]['imu_device_time']['dataready_timestamp_raw'] = 2**32-4000
    rows[1]['imu_device_time']['dataready_timestamp_raw'] = 1000
    rows[2]['imu_device_time']['dataready_timestamp_raw'] = 6000
    model = build_model(rows, original, runtime(original))
    assert model['counter_wrap_counts'] == dict(sample_timestamp_raw=1,dataready_timestamp_raw=1)
    assert np.diff([row['imu_measurement_stamp_ns'] for row in rows]).tolist() == [5_000_000]*2


@pytest.mark.parametrize('change', ['missing', 'duplicate', 'backward', 'reset', 'wrong_hash', 'epoch', 'unit', 'forward_jump'])
def test_invalid_timing_never_falls_back_or_interpolates(original, change):
    sha = freeze_profile(original); rows = events(sha)
    meta = rows[2]['imu_device_time']
    if change == 'missing': meta['sample_timestamp_valid'] = False
    elif change == 'duplicate': meta['sample_timestamp_raw'] = rows[1]['imu_device_time']['sample_timestamp_raw']
    elif change == 'backward': meta['sample_timestamp_raw'] = 4000
    elif change == 'reset': meta['sample_timestamp_raw'] = 0
    elif change == 'wrong_hash': meta['timing_profile_sha256'] = '0'*64
    elif change == 'epoch': meta['stream_epoch'] = 'new'
    elif change == 'unit': meta['device_timestamp_unit'] = '100us'
    else: meta['sample_timestamp_raw'] = 3_000_000
    with pytest.raises(ValueError): build_model(rows, original, runtime(original))


def test_auto_only_uses_present_verified_profile_and_arrival_is_explicit(original):
    cfg = runtime(original, 'auto')
    assert resolved_mode(original, cfg) == 'arrival'
    sha = freeze_profile(original)
    assert resolved_mode(original, cfg) == 'device_relative'
    cfg['offline_imu_time_mode'] = 'arrival'
    assert resolved_mode(original, cfg) == 'arrival'
    (original/'configuration/evidence.bin').write_bytes(b'changed')
    with pytest.raises(ValueError, match='HASH_MISMATCH'):
        build_model(events(sha), original, runtime(original, 'auto'))


def test_model_binding_changes_with_any_derived_sequence_or_profile(original):
    sha = freeze_profile(original); rows = events(sha)
    first = build_model(rows, original, runtime(original))
    changed = events(sha); changed[-1]['monotonic_ns'] += 1000; changed[-1]['stamp_ns'] += 1000
    second = build_model(changed, original, runtime(original))
    assert first['model_sha256'] != second['model_sha256']
    clock = SourceClock(rows); old_report = clock.report()
    assert 'imu_time_model' not in old_report
    clock.bind_imu_time_model(first)
    assert clock.report() != old_report
    cfg = runtime(original); configure_runtime(cfg, clock)
    assert cfg['prior_template']['offline_imu_time_model'] == first


def test_calibration_consumes_identical_derived_time_and_rejects_mixed_model(original):
    sha = freeze_profile(original); rows = events(sha)
    model = build_model(rows, original, runtime(original)); clock = SourceClock(rows)
    clock.bind_imu_time_model(model)
    times = [motion_sample_time_s(row, clock) for row in rows]
    np.testing.assert_allclose(np.diff(times), .005, atol=1e-15)
    bad = dict(rows[0], imu_time_model_sha256='0'*64)
    with pytest.raises(ValueError, match='frozen timing model'): motion_sample_time_s(bad, clock)
    bad = dict(rows[0]); del bad['imu_measurement_stamp_ns']
    with pytest.raises(ValueError, match='every derived'): motion_sample_time_s(bad, clock)
    wheel_event = dict(category='wheel', monotonic_ns=rows[0]['monotonic_ns']+1_000_000)
    assert motion_sample_time_s(wheel_event, clock) == .001


def test_refine_cli_defaults_auto_and_preserves_explicit_arrival(monkeypatch):
    from wc_runtime import mapping_refine
    seen = {}
    monkeypatch.setattr(mapping_refine, 'run', lambda args: seen.update(vars(args)) or 0)
    assert mapping_refine.main(['--dataset','original','--output','new']) == 0
    assert seen['imu_time_mode'] == 'auto'
    assert mapping_refine.main(['--dataset','original','--output','new','--imu-time-mode','arrival']) == 0
    assert seen['imu_time_mode'] == 'arrival'


def test_motion_prior_integrates_derived_time_but_preserves_host_clock_validation(original):
    sha = freeze_profile(original); rows = events(sha); model = build_model(rows, original, runtime(original))
    cfg = prior_config(); cfg.update(offline_experiment=True, offline_imu_time_mode='device_relative',
        offline_imu_time_model=model)
    prior = MotionPrior(cfg, 'original', clock=lambda:1.04)
    for row in rows:
        imu(prior, row['sequence'], row['stamp_ns'], monotonic_ns=row['monotonic_ns'],
            session_id='original', stream_epoch='epoch', **prior_arguments(row))
    assert [row['stamp'] for row in prior.imu] == [row['imu_measurement_stamp_ns'] for row in rows]
    assert [row['arrival_stamp'] for row in prior.imu] == [row['stamp_ns'] for row in rows]
    # A discontinuous arrival clock cannot be concealed by a valid device dt.
    with pytest.raises(PriorError, match='host arrival clock'):
        imu(prior, 8, rows[-1]['stamp_ns']+1_000_000_000,
            monotonic_ns=rows[-1]['monotonic_ns']+5_000_000, session_id='original', stream_epoch='epoch',
            measurement_stamp_ns=rows[-1]['imu_measurement_stamp_ns']+5_000_000,
            imu_time_model_sha256=model['model_sha256'])


def test_live_or_unbound_measurement_time_is_rejected(original):
    sha = freeze_profile(original); model = build_model(events(sha), original, runtime(original))
    cfg = prior_config(); cfg.update(offline_imu_time_mode='device_relative', offline_imu_time_model=model)
    with pytest.raises(PriorError, match='offline only'): MotionPrior(cfg, 'original')
    prior = MotionPrior(prior_config(), 'synthetic')
    with pytest.raises(PriorError, match='frozen offline model'):
        imu(prior, 0, 1_000_000_000, measurement_stamp_ns=999_000_000)


def test_sqlite_replay_keeps_header_host_consistency_and_original_cdr(original, monkeypatch):
    sha = freeze_profile(original); rows = events(sha)
    serialization = ModuleType('rclpy.serialization')
    serialization.deserialize_message = lambda data, kind: json.loads(data, object_hook=lambda d: SimpleNamespace(**d))
    utilities = ModuleType('rosidl_runtime_py.utilities'); utilities.get_message = lambda kind: kind
    monkeypatch.setitem(sys.modules, 'rclpy.serialization', serialization)
    monkeypatch.setitem(sys.modules, 'rosidl_runtime_py.utilities', utilities)
    def ros_stamp(ns): return dict(zip(('sec','nanosec'), divmod(ns, 10**9)))
    messages = [(1, dict(data=json.dumps(dict(stamp_ns=100_000_000_000, receive_monotonic_ns=1_000_000_000, sequence=0))))]
    for row in rows:
        host = ros_stamp(row['stamp_ns']+5_000_000_000)
        meta = row['imu_device_time']
        messages.append((2, dict(meta, frame_sequence=row['sequence'], host_monotonic_ns=row['monotonic_ns'],
            header=dict(stamp=host), host_receive_time=host, imu=dict(header=dict(stamp=host)),
            time_source='arrival_only', common_time_valid=False, common_time_ns=0,
            diagnostic_flags=['DEVICE_RELATIVE_TIME_VALIDATED','TIMING_PROFILE_SHA256:'+sha])))
    for topic_id, side in ((3,'left'),(4,'right')):
        host = ros_stamp(100_000_000_000)
        messages.append((topic_id, dict(side=side, frame_sequence=0, host_monotonic_ns=1_000_000_000,
            header=dict(stamp=host), host_receive_time=host, cloud=dict(header=dict(stamp=host)),
            time_source='arrival_only', common_time_valid=False, common_time_ns=100_000_000_000)))
    (original/'bag').mkdir(); path = original/'bag/source.db3'
    with sqlite3.connect(path) as db:
        db.execute('CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT)')
        db.execute('CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB)')
        for values in [(1,'/wheel','std_msgs/msg/String'),(2,'/imu','wc_interfaces/msg/H30Frame'),
                       (3,'/wc_mapping/lidar_left/source_frame','wc_interfaces/msg/SourceFrame'),
                       (4,'/wc_mapping/lidar_right/source_frame','wc_interfaces/msg/SourceFrame')]:
            db.execute('INSERT INTO topics VALUES(?,?,?)', values)
        for i, (topic, message) in enumerate(messages):
            db.execute('INSERT INTO messages VALUES(?,?,?,?)',(i+1,topic,100_000_000_000+i,json.dumps(message).encode()))
    original_sha = hashlib.sha256(path.read_bytes()).hexdigest()
    packets = RecordedPackets(original, runtime(original, 'auto'))
    try:
        imu_rows = [row for row in packets.events if row['category']=='imu']
        first = imu_rows[0]; msg = packets.get(first)
        assert stamp(msg.header.stamp) == stamp(msg.host_receive_time) == stamp(msg.imu.header.stamp) == first['stamp_ns']
        assert msg.host_monotonic_ns == first['monotonic_ns']
        assert msg.common_time_valid is False and msg.common_time_ns == 0
        assert first['imu_measurement_stamp_ns'] < stamp(msg.header.stamp)
        assert first['original_source_stamp_ns'] == first['stamp_ns']+5_000_000_000
    finally: packets.close()
    assert hashlib.sha256(path.read_bytes()).hexdigest() == original_sha
    legacy = RecordedPackets(original, runtime(original, 'arrival'))
    try:
        assert 'imu_time_model' not in legacy.clock.report()
        assert not any('imu_measurement_stamp_ns' in row for row in legacy.events)
        assert len([row for row in legacy.events if row['category']=='imu']) == len(rows)
    finally: legacy.close()
