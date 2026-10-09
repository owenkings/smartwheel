"""Software-only archive I/O tests; no ROS node, SDK or sensor is opened."""

import json
from pathlib import Path
import threading
import time

import pytest

import wc_fusion.archive as archive
from wc_fusion.archive import ArchiveCloseTimeout, ArchiveError, DurableArchive


def results(writer, count, timeout=3):
    output = []
    deadline = time.monotonic() + timeout
    while len(output) < count and time.monotonic() < deadline:
        output.extend(writer.poll())
        if len(output) < count:
            time.sleep(.002)
    assert len(output) == count, (output, writer.failure)
    return output


def held_writer(monkeypatch):
    entered, release = threading.Event(), threading.Event()
    original = archive._commit_bytes
    calls = []

    def hold(path, payload, stop):
        calls.append(path.name)
        entered.set()
        assert release.wait(5), 'test failed to release its own worker'
        original(path, payload, stop)

    monkeypatch.setattr(archive, '_commit_bytes', hold)
    return entered, release, calls


def test_timing_separates_sync_completion_from_poll(monkeypatch, tmp_path):
    entered, release = threading.Event(), threading.Event()
    real_sync = archive.os.fsync
    def held_sync(fd):
        if not entered.is_set():
            entered.set()
            assert release.wait(3)
        real_sync(fd)
    monkeypatch.setattr(archive.os, 'fsync', held_sync)
    writer = DurableArchive()
    try:
        writer.submit('timed', tmp_path/'raw.json', b'{}')
        assert entered.wait(2)
        assert writer.poll() == []
        before_release = time.monotonic_ns()
        release.set()
        result = results(writer, 1)[0]
        timing = dict(result.timing_ns)
        stages = ['submitted','dequeued','file_fsync_start','file_fsync_end',
                  'directory_fsync_start','directory_fsync_end','ack_ready','polled']
        assert [timing[s] for s in stages] == sorted(timing[s] for s in stages)
        assert timing['file_fsync_start'] <= before_release <= timing['file_fsync_end']
        assert result.error is None and (tmp_path/'raw.json').read_bytes() == b'{}'
    finally:
        release.set()
        writer.close()


def test_submit_nonblocking_fifo_and_total_capacity(monkeypatch, tmp_path):
    entered, release, calls = held_writer(monkeypatch)
    writer = DurableArchive(capacity=2)
    try:
        writer.submit('a', tmp_path/'a.json', b'{"a":1}')
        assert entered.wait(2)
        submitted = threading.Event()
        submitter = threading.Thread(target=lambda: (writer.submit('b', tmp_path/'b.json', b'{"b":2}'), submitted.set()))
        submitter.start()
        assert submitted.wait(.3), 'submit blocked on worker disk I/O'
        submitter.join(1)
        assert writer.pending_count == 2
        assert writer.poll() == []
        with pytest.raises(ArchiveError, match='ARCHIVE_QUEUE_FULL'):
            writer.submit('c', tmp_path/'c.json', b'{}')
        assert not list(tmp_path.iterdir())
        release.set()
        completed = results(writer, 2)
        assert [item.key for item in completed] == ['a', 'b']
        assert all(item.error is None for item in completed)
        assert calls == ['a.json', 'b.json']
        assert writer.pending_count == 0
        assert (tmp_path/'a.json').read_bytes() == b'{"a":1}'
        assert (tmp_path/'b.json').read_bytes() == b'{"b":2}'
    finally:
        release.set()
        writer.close()


def test_unpolled_completions_hold_capacity(tmp_path):
    writer = DurableArchive(capacity=1)
    try:
        writer.submit('a', tmp_path/'a', b'{}')
        # Wait for completion without acknowledging it via poll.
        deadline = time.monotonic() + 3
        while not (tmp_path/'a').exists() and time.monotonic() < deadline:
            time.sleep(.002)
        with pytest.raises(ArchiveError, match='ARCHIVE_QUEUE_FULL'):
            writer.submit('b', tmp_path/'b', b'{}')
        assert results(writer, 1)[0].error is None
        writer.submit('b', tmp_path/'b', b'{}')
        assert results(writer, 1)[0].key == 'b'
    finally:
        writer.close()


def test_failure_latches_and_cancels_later_jobs_in_fifo(monkeypatch, tmp_path):
    entered, release = threading.Event(), threading.Event()
    calls = []

    def fail(path, payload, stop):
        calls.append(path.name)
        entered.set()
        assert release.wait(5)
        raise OSError('synthetic disk full')

    monkeypatch.setattr(archive, '_commit_bytes', fail)
    writer = DurableArchive(capacity=3)
    try:
        writer.submit('a', tmp_path/'a', b'{}')
        assert entered.wait(2)
        writer.submit('b', tmp_path/'b', b'{}')
        writer.submit('c', tmp_path/'c', b'{}')
        release.set()
        completed = results(writer, 3)
        assert [item.key for item in completed] == ['a', 'b', 'c']
        assert all(item.error is not None for item in completed)
        assert 'synthetic disk full' in completed[0].error
        assert all('CANCELLED_AFTER_FAILURE' in item.error for item in completed[1:])
        assert not [item for item in completed if item.error is None]
        assert calls == ['a']
        assert not list(tmp_path.iterdir())
        with pytest.raises(ArchiveError, match='ARCHIVE_FAILED'):
            writer.submit('d', tmp_path/'d', b'{}')
    finally:
        release.set()
        writer.close()


def test_payload_is_immutable_encoded_snapshot(monkeypatch, tmp_path):
    entered, release, _ = held_writer(monkeypatch)
    original = {'points': [[1.0, 2.0, 3.0]], 'side': 'left'}
    payload = json.dumps(original, allow_nan=False).encode('utf-8')
    expected = payload
    writer = DurableArchive()
    try:
        with pytest.raises(ArchiveError, match='ARCHIVE_PAYLOAD'):
            writer.submit('mutable', tmp_path/'mutable', bytearray(payload))
        with pytest.raises(ArchiveError, match='ARCHIVE_PAYLOAD'):
            writer.submit('view', tmp_path/'view', memoryview(payload))
        writer.submit('a', tmp_path/'a.json', payload)
        assert entered.wait(2)
        original['points'][0][0] = 999
        payload = b'{"changed":true}'
        release.set()
        assert results(writer, 1)[0].error is None
        assert (tmp_path/'a.json').read_bytes() == expected
        assert json.loads((tmp_path/'a.json').read_bytes())['points'] == [[1.0, 2.0, 3.0]]
    finally:
        release.set()
        writer.close()


def test_existing_file_is_never_replaced_and_temp_is_cleaned(tmp_path):
    destination = tmp_path/'existing.json'
    destination.write_bytes(b'{"old":true}')
    identity = destination.stat().st_ino
    writer = DurableArchive()
    try:
        writer.submit('a', destination, b'{"new":true}')
        completed = results(writer, 1)
        assert 'FileExistsError' in completed[0].error
        assert destination.read_bytes() == b'{"old":true}'
        assert destination.stat().st_ino == identity
        assert list(tmp_path.iterdir()) == [destination]
    finally:
        writer.close()


def test_concurrent_destination_creation_is_not_overwritten(monkeypatch, tmp_path):
    original = archive.os.link

    def race(source, destination, **kwargs):
        (tmp_path/destination).write_bytes(b'competing existing file')
        return original(source, destination, **kwargs)

    monkeypatch.setattr(archive.os, 'link', race)
    writer = DurableArchive()
    try:
        writer.submit('a', tmp_path/'a', b'{}')
        assert 'FileExistsError' in results(writer, 1)[0].error
        assert (tmp_path/'a').read_bytes() == b'competing existing file'
        assert [p.name for p in tmp_path.iterdir()] == ['a']
    finally:
        writer.close()


def test_close_timeout_is_explicit_then_worker_cleans_own_temp(monkeypatch, tmp_path):
    original = archive.os.fsync
    entered, release = threading.Event(), threading.Event()

    def block_first_sync(descriptor):
        if not entered.is_set():
            entered.set()
            assert release.wait(5)
        return original(descriptor)

    monkeypatch.setattr(archive.os, 'fsync', block_first_sync)
    writer = DurableArchive(capacity=2)
    try:
        writer.submit('active', tmp_path/'active', b'{}')
        assert entered.wait(2)
        writer.submit('waiting', tmp_path/'waiting', b'{}')
        assert len(list(tmp_path.glob('.*.archive-*.tmp'))) == 1
        begin = time.monotonic()
        with pytest.raises(ArchiveCloseTimeout, match='cleanup is not confirmed'):
            writer.close(timeout=.02)
        assert time.monotonic() - begin < .3
        assert writer.alive
        with pytest.raises(ArchiveError, match='ARCHIVE_CLOSED'):
            writer.submit('after-close', tmp_path/'after-close', b'{}')
        release.set()
        writer.close(timeout=2)
        assert not writer.alive
        completed = results(writer, 2)
        assert [item.key for item in completed] == ['active', 'waiting']
        assert all(item.error is not None for item in completed)
        assert not list(tmp_path.iterdir())
        writer.close(timeout=0)  # Confirmed termination is idempotent.
    finally:
        release.set()
        writer.close()


def test_file_sync_failure_has_no_success_or_final_and_cleans_temp(monkeypatch, tmp_path):
    def fail(_):
        raise OSError('file fsync failed')

    monkeypatch.setattr(archive.os, 'fsync', fail)
    writer = DurableArchive()
    try:
        writer.submit('a', tmp_path/'a', b'{}')
        assert 'file fsync failed' in results(writer, 1)[0].error
        assert not list(tmp_path.iterdir())
    finally:
        writer.close()


def test_directory_sync_failure_never_acknowledges_success(monkeypatch, tmp_path):
    original, calls = archive.os.fsync, []

    def fail_directory(descriptor):
        calls.append(descriptor)
        if len(calls) == 2:
            raise OSError('directory fsync failed')
        return original(descriptor)

    monkeypatch.setattr(archive.os, 'fsync', fail_directory)
    writer = DurableArchive()
    try:
        writer.submit('a', tmp_path/'a', b'{}')
        completed = results(writer, 1)
        assert 'directory fsync failed' in completed[0].error
        # The committed file is preserved for diagnosis, never published as a
        # durable success or silently overwritten in an attempted retry.
        assert (tmp_path/'a').read_bytes() == b'{}'
        assert not list(tmp_path.glob('.*.archive-*.tmp'))
        assert not [item for item in completed if item.error is None]
    finally:
        writer.close()


def test_visible_final_is_not_acknowledged_before_directory_sync(monkeypatch, tmp_path):
    original = archive.os.fsync
    entered, release = threading.Event(), threading.Event()
    count = 0

    def hold_directory(descriptor):
        nonlocal count
        count += 1
        if count == 2:
            entered.set()
            assert release.wait(5)
        return original(descriptor)

    monkeypatch.setattr(archive.os, 'fsync', hold_directory)
    writer = DurableArchive()
    try:
        writer.submit('a', tmp_path/'a', b'{}')
        assert entered.wait(2)
        assert (tmp_path/'a').read_bytes() == b'{}'
        assert writer.poll() == []  # Visibility alone never authorizes publish.
        release.set()
        assert results(writer, 1)[0].error is None
    finally:
        release.set()
        writer.close()


def test_duplicate_unpolled_keys_and_paths_are_rejected(monkeypatch, tmp_path):
    entered, release, _ = held_writer(monkeypatch)
    writer = DurableArchive()
    try:
        writer.submit('a', tmp_path/'a', b'{}')
        assert entered.wait(2)
        with pytest.raises(ArchiveError, match='ARCHIVE_DUPLICATE'):
            writer.submit('a', tmp_path/'other', b'{}')
        with pytest.raises(ArchiveError, match='ARCHIVE_DUPLICATE'):
            writer.submit('other', tmp_path/'a', b'{}')
        release.set()
        assert results(writer, 1)[0].error is None
    finally:
        release.set()
        writer.close()


@pytest.mark.parametrize('capacity', [0, 9, True, 1.5])
def test_capacity_cannot_exceed_eight(capacity):
    with pytest.raises(ArchiveError, match='ARCHIVE_CAPACITY'):
        DurableArchive(capacity=capacity)


def test_invalid_paths_and_empty_payload_are_rejected_without_io(tmp_path):
    writer = DurableArchive()
    try:
        for destination in [Path('relative.json'), tmp_path/'..'/'outside']:
            with pytest.raises(ArchiveError, match='ARCHIVE_PATH'):
                writer.submit('a', destination, b'{}')
        with pytest.raises(ArchiveError, match='ARCHIVE_PAYLOAD'):
            writer.submit('a', tmp_path/'a', b'')
        assert writer.pending_count == 0
        assert writer.poll() == []
    finally:
        writer.close()


def test_symlink_parent_rejected_without_writing_target(tmp_path):
    real, alias = tmp_path/'real', tmp_path/'alias'
    real.mkdir()
    alias.symlink_to(real, target_is_directory=True)
    writer = DurableArchive()
    try:
        writer.submit('a', alias/'a', b'{}')
        assert 'ARCHIVE_PATH' in results(writer, 1)[0].error
        assert not list(real.iterdir())
    finally:
        writer.close()
