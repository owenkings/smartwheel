"""SYNTHETIC continuous mapping: no devices, ROS or fabricated calibration."""
import json

import numpy as np
import pytest

from wc_runtime import mapping_input as m
from wc_runtime import single_mapping_input as single
from test_mapping_input import (START, WALL, config, deliver, imu, owner, serialize,
                                settle, source, tmp_path, wheel)


def continuous_config(mode='left'):
    candidate = config(mode)
    candidate['continuous_mapping'] = True
    return candidate


@pytest.mark.parametrize('mode', ['left', 'right', 'all'])
def test_fixed_reference_is_created_before_any_imu_wheel_or_static_window(owner, mode):
    value = owner(mode, continuous_mapping=True)
    settle(value, START)
    assert value.transforms is not None
    assert value.bootstrap.last_imu is value.bootstrap.last_wheel is None
    document = json.loads((value.output/'bootstrap.json').read_text())
    assert document['reference_mode'] == 'FIXED_MOUNT_REFERENCE_NO_STATIC_CALIBRATION'
    assert document['gravity_observation_available'] is document['stationary_window_required'] is False
    assert document['mean_acceleration_imu_m_s2'] is document['imu_roll_pitch_yaw_rad'] is None
    assert document['window_host_span_ns'] is None
    assert document['imu_samples_used'] == document['wheel_zero_records_used'] == []
    assert document['imu_stream_epoch'] is document['wheel_stream_epoch'] is None
    assert document['gravity_used_to_define_mounts'] is False
    prior = json.loads((value.output.parent/'prior_config.json').read_text())
    assert prior['continuous_mapping'] is True
    assert prior['initial_reference_mode'] == document['reference_mode']
    expected = m.mount_transforms(config(mode), [0, 0, 9.81])
    for key in ('T_base_axle', 'R_base_imu'):
        assert np.allclose(value.transforms[key], expected[key])


def test_moving_start_and_old_health_samples_do_not_require_static_or_freshness(owner):
    value = owner(continuous_mapping=True)
    settle(value, START)
    now = START+60_000_000_000
    value.receive_wheel(wheel(0, START, words=(400, 500)), now)
    assert not value.receive_imu(imu(0, START, acceleration=(10, 3, 2), gyro=(1, 2, 3)), now)
    value.check_stale(now+24*3600*1_000_000_000)
    assert value.bootstrap.window == [] and value.failure_reason is None
    document = json.loads((value.output/'bootstrap.json').read_text())
    assert document['mean_acceleration_imu_m_s2'] is None
    assert document['imu_samples_used'] == []


@pytest.mark.parametrize('publication_mode', ['written_before_publish', 'queued_realtime'])
def test_first_frame_after_deadline_and_long_pause_resume_original_stamps_without_health(owner, publication_mode):
    value = owner(publication_mode=publication_mode, checkpoint_policy='on_close', continuous_mapping=True)
    settle(value, START)
    published = []
    after_day = START+24*3600*1_000_000_000
    value.check_stale(after_day)
    assert value.status(after_day)['not_yet_received_sources'] == ['left']
    # The first accepted source is itself old: no callback-age or startup gate.
    first = source(host=START+1_000_000_000)
    assert deliver(value, first, published, at=after_day)
    assert m._stamp(published[-1].header.stamp) == WALL+first.host_monotonic_ns
    next_day = after_day+24*3600*1_000_000_000
    value.check_stale(next_day)
    second = source(sequence=1, host=after_day)
    assert deliver(value, second, published, at=next_day)
    assert len(published) == 2 and value.failure_reason is None
    assert [m._stamp(item.header.stamp) for item in published] == [WALL+first.host_monotonic_ns, WALL+after_day]
    assert value.pending_bytes == 0


def test_delayed_reference_config_write_has_no_fifteen_second_deadline(owner):
    value = owner(continuous_mapping=True)
    # Results are applied by poll_io; elapsed host time alone cannot fail it.
    value.check_stale(START+24*3600*1_000_000_000)
    settle(value, START+24*3600*1_000_000_000)
    assert value.transforms is not None and value.failure_reason is None


def test_unmatched_dual_source_wait_is_bounded_and_recovers_with_two_sources(owner):
    value = owner('all', checkpoint_policy='on_close', continuous_mapping=True)
    settle(value, START)
    published = []
    for index in range(100):
        host = START+index*200_000_000
        assert not deliver(value, source(sequence=index, host=host), published)
        value.check_stale(host+60_000_000_000)
    assert len(value.buffers['left']) == 32 and not value.buffers['right']
    assert not published and value.unpaired_discarded == 68
    assert value.status(START+60_000_000_000)['input_wait_state'] == 'WAITING_MATCHING_PAIR'
    # A late first right source still must pair within the original 50 ms.
    now = START+120_000_000_000
    assert not deliver(value, source('right', 0, now), published)
    assert not deliver(value, source('left', 100, now+60_000_000), published)
    assert not published
    assert deliver(value, source('right', 1, now+80_000_000), published)
    assert len(published) == 1 and value.last_pair_delta_ns == 20_000_000
    value.close()
    pair = json.loads((value.output/'pairs.jsonl').read_text().strip())
    assert [row['side'] for row in pair['sources']] == ['left', 'right']


def test_source_gate_accepts_more_than_twelve_thousand_real_validated_envelopes():
    gate = single.SourceGate('mapping_input_test', 'left', 'lidar-L', started_ns=START, continuous_mapping=True)
    for index in range(12003):
        host = START+index*200_000_000
        metadata = gate.inspect(source(sequence=index, host=host), host+120_000_000_000)
        assert metadata['frame_sequence'] == index
        gate.archived_frames += 1
        gate.last_forwarded_host_ns = host
    assert gate.archived_frames == gate.validated_frames == 12003
    assert gate.status(host)['max_archived_frames'] is None
    gate.check_stale(host+24*3600*1_000_000_000)


@pytest.mark.parametrize('archive_type', [single.FrameArchive, m.FlushedFrameArchive])
def test_archive_writes_beyond_legacy_boundary_without_changing_max_constant(tmp_path, archive_type):
    archive = archive_type(tmp_path, tmp_path/'archive', continuous_mapping=True)
    try:
        archive.count = 12000  # Boundary fixture; no claim that earlier files exist.
        row = archive.append(b'SYNTHETIC source cdr', {'frame_sequence': 12000})
        assert row['archive_index'] == 12001
        assert (archive.root/row['file']).read_bytes() == b'SYNTHETIC source cdr'
        assert single.MAX_FRAMES == m.MAX_FRAMES == 12000
    finally:
        archive.close()


@pytest.mark.parametrize('publication_mode', ['written_before_publish', 'queued_realtime'])
def test_mapping_owner_and_archive_both_use_unlimited_profile(owner, monkeypatch, publication_mode):
    monkeypatch.setattr(m, 'MAX_FRAMES', 2)
    monkeypatch.setattr(single, 'MAX_FRAMES', 2)
    value = owner(publication_mode=publication_mode, checkpoint_policy='on_close', continuous_mapping=True)
    settle(value, START)
    published = []
    for index in range(4):
        assert deliver(value, source(sequence=index, host=START+index*200_000_000), published)
    assert value.enqueued_outputs == value.archived_outputs == len(published) == 4
    assert value.status(START)['max_archived_outputs'] is None
    assert value.archives['left'].continuous_mapping


def test_continuous_profile_still_bounds_pending_storage_memory(owner, monkeypatch):
    value = owner(publication_mode='queued_realtime', checkpoint_policy='on_close', continuous_mapping=True)
    settle(value, START)
    monkeypatch.setattr(m, 'MAX_PENDING_BYTES', 1)
    published = []
    with pytest.raises(m.InputFailure, match='ARCHIVE_QUEUE_FULL'):
        deliver(value, source(), published)
    assert not published
    assert value.enqueued_outputs == value.pending_bytes == 0
    assert not value.pending


@pytest.mark.parametrize('kind', ['lidar_identity', 'lidar_future', 'lidar_sequence', 'imu_identity',
                                  'imu_future', 'wheel_identity', 'wheel_future'])
def test_continuous_profile_preserves_identity_future_and_order_failures(owner, kind):
    value = owner(continuous_mapping=True)
    settle(value, START)
    with pytest.raises(m.InputFailure):
        if kind.startswith('lidar'):
            message = source()
            if kind == 'lidar_identity': message.sensor_id = 'different-sensor'
            if kind == 'lidar_sequence':
                assert deliver(value, message, [])
            deliver(value, message, [], at=START-1 if kind == 'lidar_future' else START)
        elif kind.startswith('imu'):
            message = imu()
            if kind == 'imu_identity': message.sensor_id = 'different-imu'
            value.receive_imu(message, START-1 if kind == 'imu_future' else START)
        else:
            message = wheel()
            if kind == 'wheel_identity': message['device_id'] = 'different-wheel'
            value.receive_wheel(message, START-1 if kind == 'wheel_future' else START)
    assert value.failure_reason is not None and value.state == 'FAILED'


@pytest.mark.parametrize('bad', [None, 1, 'true', []])
def test_continuous_profile_requires_explicit_boolean(bad):
    candidate = config()
    candidate['continuous_mapping'] = bad
    with pytest.raises(m.InputFailure, match='INVALID_CONTINUOUS_MAPPING_PROFILE'):
        m.validate_config(candidate)
