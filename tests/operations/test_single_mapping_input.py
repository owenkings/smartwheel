"""Synthetic SourceFrame gate/archive tests; no ROS, devices, TF or network."""
import copy
import hashlib
import json
from pathlib import Path
import struct
import shutil
from types import SimpleNamespace as NS
import uuid

import pytest

from wc_runtime import single_mapping_input as module


@pytest.fixture
def tmp_path():
    # pytest's mode-0700 Windows temp directories are unreadable in this managed
    # workspace. A normal-permission, uniquely named owner directory is portable.
    parent = Path(__file__).resolve().parent
    directory = parent / ('.single_input_test_' + uuid.uuid4().hex)
    directory.mkdir()
    try:
        yield directory
    finally:
        resolved = directory.resolve()
        assert resolved.parent == parent and resolved.name.startswith('.single_input_test_')
        shutil.rmtree(resolved)


def stamp(value):
    return NS(sec=value // 1_000_000_000, nanosec=value % 1_000_000_000)


def message(sequence=0, monotonic=10_000_000_000, wall=1_700_000_000_000_000_000):
    header = NS(stamp=stamp(wall), frame_id='lidar_right')
    cloud = NS(header=copy.deepcopy(header), width=3, height=1, point_step=16, row_step=48,
               is_bigendian=False, is_dense=False,
               fields=[NS(name=name, offset=4 * i, count=1, datatype=7)
                       for i, name in enumerate(('x', 'y', 'z', 'intensity'))],
               data=struct.pack('<12f', 1, 2, 3, 8, 4, 5, 6, 9, float('nan'), 0, 0, 10))
    return NS(header=header, cloud=cloud, session_id='test_single', side='right', sensor_id='sensor-right',
              stream_epoch='epoch-1', frame_sequence=sequence, units='m', coordinate_convention='FLU',
              time_source='arrival_only', common_time_valid=False, uncertainty_valid=False,
              clock_model_id='', common_time_ns=wall, host_receive_time=stamp(wall),
              host_monotonic_ns=monotonic, source_config_hash='a' * 64, raw_count=3, valid_count=2)


def gate():
    return module.SourceGate('test_single', 'right', 'sensor-right', started_ns=0)


@pytest.fixture
def archive(tmp_path, monkeypatch):
    monkeypatch.setattr(module.shutil, 'disk_usage', lambda _: NS(free=4_000_000_000))
    root = tmp_path / 'project'
    root.mkdir()
    result = module.FrameArchive(root, root / 'session' / 'input')
    yield result
    result.close()


def test_valid_authoritative_frame_keeps_invalid_rows_and_source_metadata():
    value = message()
    original_data, original_stamp = bytes(value.cloud.data), copy.deepcopy(value.header.stamp)
    result = gate().inspect(value, value.host_monotonic_ns)
    assert result['raw_key'] == ['test_single', 'sensor-right', 'epoch-1', 0]
    assert result['raw_count'] == 3 and result['valid_count'] == 2 and result['invalid_xyz_count'] == 1
    assert result['cloud_data_sha256'] == hashlib.sha256(original_data).hexdigest()
    assert value.cloud.data == original_data and value.header.stamp == original_stamp
    assert result['common_time_valid'] is False and result['uncertainty_valid'] is False


@pytest.mark.parametrize('field,bad', [
    ('session_id', 'another'), ('side', 'left'), ('sensor_id', 'sensor-left'),
    ('units', 'mm'), ('coordinate_convention', 'RDF'), ('time_source', 'validated_model'),
    ('common_time_valid', True), ('uncertainty_valid', True), ('clock_model_id', 'guessed'),
    ('stream_epoch', ''), ('source_config_hash', 'unknown'), ('frame_sequence', True),
    ('raw_count', 2), ('valid_count', 3), ('common_time_ns', 1),
])
def test_bad_identity_timing_or_counts_latch_even_when_next_frame_would_be_valid(field, bad):
    source, validator = message(), gate()
    setattr(source, field, bad)
    with pytest.raises(module.InputFailure):
        validator.inspect(source, source.host_monotonic_ns)
    reason = validator.failure_reason
    with pytest.raises(module.InputFailure):
        validator.inspect(message(), source.host_monotonic_ns)
    assert validator.failure_reason == reason and validator.state == 'FAILED'
    assert validator.published_frames == 0


@pytest.mark.parametrize('mutation', [
    lambda m: setattr(m.header, 'frame_id', 'map'),
    lambda m: setattr(m.cloud.header, 'frame_id', 'lidar_left'),
    lambda m: setattr(m.cloud.header.stamp, 'nanosec', 1),
    lambda m: setattr(m.host_receive_time, 'sec', 1),
    lambda m: setattr(m.cloud, 'row_step', 47),
    lambda m: setattr(m.cloud.fields[1], 'offset', 0),
    lambda m: setattr(m.cloud.fields[0], 'datatype', 6),
    lambda m: setattr(m.cloud.fields[3], 'name', 'not_intensity'),
    lambda m: setattr(m.cloud, 'data', m.cloud.data[:-1]),
    lambda m: setattr(m.cloud, 'is_dense', True),
    lambda m: setattr(m.cloud, 'width', 20001),
])
def test_header_and_point_schema_corruption_is_rejected(mutation):
    source, validator = message(), gate()
    mutation(source)
    with pytest.raises(module.InputFailure):
        validator.inspect(source, source.host_monotonic_ns)
    assert validator.state == 'FAILED'


@pytest.mark.parametrize('change', ['epoch', 'config', 'sequence', 'host_time', 'wall_time'])
def test_stream_resets_duplicates_and_time_reversals_fail(change):
    validator = gate(); first = message(); validator.inspect(first, first.host_monotonic_ns)
    second = message(1, first.host_monotonic_ns + 100_000_000, first.common_time_ns + 100_000_000)
    if change == 'epoch': second.stream_epoch = 'epoch-2'
    if change == 'config': second.source_config_hash = 'b' * 64
    if change == 'sequence': second.frame_sequence = 0
    if change == 'host_time': second.host_monotonic_ns = first.host_monotonic_ns
    if change == 'wall_time':
        second.header.stamp = second.cloud.header.stamp = second.host_receive_time = stamp(first.common_time_ns)
        second.common_time_ns = first.common_time_ns
    with pytest.raises(module.InputFailure):
        validator.inspect(second, first.host_monotonic_ns + 100_000_000)


def test_startup_grace_and_stale_stream_latch():
    validator = gate()
    validator.check_stale(module.STARTUP_NS)
    with pytest.raises(module.InputFailure, match='STARTUP_TIMEOUT'):
        validator.check_stale(module.STARTUP_NS + 1)
    validator = gate(); first = message(); validator.inspect(first, first.host_monotonic_ns)
    validator.check_stale(first.host_monotonic_ns + module.STALE_NS)
    with pytest.raises(module.InputFailure, match='SOURCE_STALE'):
        validator.check_stale(first.host_monotonic_ns + module.STALE_NS + 1)


def test_first_frame_after_startup_deadline_is_not_allowed_to_reset_the_grace():
    source = message(monotonic=module.STARTUP_NS + 1)
    with pytest.raises(module.InputFailure, match='STARTUP_TIMEOUT'):
        gate().inspect(source, source.host_monotonic_ns)


def test_left_source_is_supported_without_substituting_right_identity():
    source = message()
    source.side, source.sensor_id = 'left', 'sensor-left'
    source.header.frame_id = source.cloud.header.frame_id = 'lidar_left'
    validator = module.SourceGate('test_single', 'left', 'sensor-left', started_ns=0)
    metadata = validator.inspect(source, source.host_monotonic_ns)
    assert metadata['frame_id'] == 'lidar_left'
    assert validator.status(source.host_monotonic_ns)['output_topic'] == '/wc_mapping/single_left/scan_cloud'


@pytest.mark.parametrize('offset', [-1, module.STALE_NS + 1])
def test_callback_rejects_future_or_already_stale_source(offset):
    source = message()
    with pytest.raises(module.InputFailure, match='STALE_OR_FUTURE'):
        gate().inspect(source, source.host_monotonic_ns + offset)


def test_throttling_uses_source_monotonic_time_not_callback_bursts(archive):
    validator, published = gate(), []
    for i in range(5):
        source = message(i, 10_000_000_000 + i * 100_000_000, 1_700_000_000_000_000_000 + i * 100_000_000)
        # All callbacks arrive as a burst; source stamps still determine 5 Hz.
        module.forward_frame(validator, archive, source, lambda m: b'CDR' + m.cloud.data,
                             published.append, clock=lambda: 11_000_000_000)
    assert validator.received_frames == validator.validated_frames == 5
    assert validator.published_frames == validator.archived_frames == 3
    assert validator.throttled_frames == 2 and len(published) == 3
    rows = [json.loads(line) for line in (archive.root / 'frames.jsonl').read_text().splitlines()]
    assert [row['frame_sequence'] for row in rows] == [0, 2, 4]


@pytest.mark.parametrize('offline', [False,True])
def test_zero_rate_requires_explicit_offline_source_gate(offline):
    if not offline:
        with pytest.raises(module.InputFailure,match='INVALID_RATE_HZ'):
            module.SourceGate('test_single','right','sensor-right',0,started_ns=0)
        return
    validator=module.SourceGate('test_single','right','sensor-right',0,started_ns=0,
                                offline_experiment=True)
    first=message()
    validator.inspect(first,first.host_monotonic_ns)
    validator.last_forwarded_host_ns=first.host_monotonic_ns
    second=message(1,first.host_monotonic_ns+1,first.common_time_ns+1)
    assert validator.inspect(second,second.host_monotonic_ns)['frame_sequence'] == 1
    assert validator.throttled_frames == 0
    assert validator.status(second.host_monotonic_ns)['rate_policy'] == 'NO_OFFLINE_THROTTLE'
    third=message(2,first.host_monotonic_ns+2,first.common_time_ns+2)
    third.sensor_id='wrong'
    with pytest.raises(module.InputFailure,match='SOURCE_IDENTITY_MISMATCH'):
        validator.inspect(third,third.host_monotonic_ns)


def test_all_incoming_frames_are_validated_before_throttling(archive):
    validator = gate(); first = message()
    module.forward_frame(validator, archive, first, lambda m: b'CDR', lambda _: None, clock=lambda: first.host_monotonic_ns)
    second = message(1, first.host_monotonic_ns + 1, first.common_time_ns + 1)
    second.units = 'mm'
    with pytest.raises(module.InputFailure):
        module.forward_frame(validator, archive, second, lambda m: b'CDR', lambda _: pytest.fail('bad cloud published'), clock=lambda: second.host_monotonic_ns)
    assert validator.published_frames == 1 and validator.throttled_frames == 0


def test_cdr_and_index_exist_and_match_before_identical_cloud_is_published(archive):
    source, validator = message(), gate()
    serialized = b'FULL_SOURCEFRAME_CDR' + source.cloud.data
    def publish(cloud):
        assert cloud is source.cloud and cloud.header == source.header
        row = json.loads((archive.root / 'frames.jsonl').read_text())
        assert (archive.root / row['file']).read_bytes() == serialized
        assert row['cdr_sha256'] == hashlib.sha256(serialized).hexdigest()
        assert row['ros_type'] == 'wc_interfaces/msg/SourceFrame'
        assert row['coordinate_transform_applied'] is False
        assert validator.archived_frames == 1 and validator.published_frames == 0
    assert module.forward_frame(validator, archive, source, lambda m: serialized, publish, clock=lambda: source.host_monotonic_ns)
    assert validator.published_frames == 1


def test_archive_fsync_failure_prevents_publication_and_latches(archive, monkeypatch):
    def failed_fsync(_): raise OSError('injected disk persistence failure')
    monkeypatch.setattr(module.os, 'fsync', failed_fsync)
    validator, source = gate(), message()
    with pytest.raises(module.InputFailure, match='persistence failure'):
        module.forward_frame(validator, archive, source, lambda _: b'CDR', lambda _: pytest.fail('unarchived cloud published'), clock=lambda: source.host_monotonic_ns)
    assert validator.state == 'FAILED' and validator.archived_frames == validator.published_frames == 0


def test_archive_delay_cannot_publish_stale_cloud(archive):
    validator, source = gate(), message()
    times = iter([source.host_monotonic_ns, source.host_monotonic_ns + module.STALE_NS + 1])
    with pytest.raises(module.InputFailure, match='SOURCE_STALE'):
        module.forward_frame(validator, archive, source, lambda _: b'CDR', lambda _: pytest.fail('stale cloud published'), clock=lambda: next(times))
    assert validator.archived_frames == 1 and validator.published_frames == 0


def test_publication_failure_leaves_archive_evidence_but_no_success_count(archive):
    source, validator = message(), gate()
    def failed_publish(_): raise RuntimeError('publisher rejected the cloud')
    with pytest.raises(module.InputFailure, match='publisher rejected'):
        module.forward_frame(validator, archive, source, lambda _: b'CDR', failed_publish, clock=lambda: source.host_monotonic_ns)
    assert validator.archived_frames == 1 and validator.published_frames == 0
    assert validator.state == 'FAILED'


def test_disk_margin_and_frame_limit_stop_forwarding(archive, monkeypatch):
    source = message(); validator = gate()
    monkeypatch.setattr(module.shutil, 'disk_usage', lambda _: NS(free=module.MIN_FREE_BYTES))
    with pytest.raises(module.InputFailure, match='DISK_FREE_MARGIN'):
        module.forward_frame(validator, archive, source, lambda _: b'CDR', lambda _: pytest.fail('low-disk cloud published'), clock=lambda: source.host_monotonic_ns)
    assert validator.published_frames == 0
    validator = gate(); validator.archived_frames = module.MAX_FRAMES
    with pytest.raises(module.InputFailure, match='FRAME_ARCHIVE_LIMIT'):
        validator.inspect(source, source.host_monotonic_ns)


def test_existing_archive_and_outside_paths_are_rejected(archive, tmp_path):
    with pytest.raises(module.InputFailure, match='ALREADY_EXISTS'):
        module.FrameArchive(archive.project_root, archive.root)
    with pytest.raises(module.InputFailure, match='OUTSIDE_PROJECT'):
        module.project_path(archive.project_root, tmp_path / 'outside')
    with pytest.raises(module.InputFailure, match='PARENT_TRAVERSAL'):
        module.project_path(archive.project_root, 'session/../elsewhere')
    with pytest.raises(module.InputFailure, match='SESSION_DIRECTORY'):
        module.project_path(archive.project_root, archive.project_root)


def test_redirected_parent_is_rejected_before_writes(archive, monkeypatch):
    original = Path.is_symlink
    monkeypatch.setattr(Path, 'is_symlink', lambda self: self == archive.root or original(self))
    with pytest.raises(module.InputFailure, match='REDIRECTED_PATH'):
        module.project_path(archive.project_root, archive.root / 'new_file')


def test_status_preserves_failure_and_experimental_semantics(archive):
    validator = gate(); validator.fail('SOURCE_STALE')
    archive.status(validator.status(20_000_000_000))
    result = json.loads((archive.root / 'status.json').read_text())
    assert result['kind'] == 'SINGLE_LIDAR_EXPERIMENT' and result['source_mode'] == 'real'
    assert result['state'] == 'FAILED' and result['failure_reason'] == 'SOURCE_STALE'
    assert result['time_source'] == 'arrival_only' and result['formal_acceptance'] is False
    assert result['published_frames'] == result['archived_frames'] == 0
    assert result['coordinate_transform_applied'] is False and result['deskew_applied'] is False
    assert not (archive.root / 'status.json.partial').exists()
