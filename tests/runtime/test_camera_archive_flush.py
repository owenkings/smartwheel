"""Camera archive durability barriers, without camera/ROS/hardware access."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import threading
import time
import uuid

import pytest

from wc_runtime import source_archive
from wc_runtime.source_archive import CameraArchive


@pytest.fixture
def archive_temp():
    # Keep the test's artifacts inside this isolated checkout on Windows, where
    # the inherited pytest temporary directory may be inaccessible.
    parent = Path(__file__).resolve().parent
    root = parent / ('.camera_archive_test_' + uuid.uuid4().hex)
    root.mkdir()
    try:
        yield root
    finally:
        assert root.resolve().parent == parent
        assert root.name.startswith('.camera_archive_test_')
        shutil.rmtree(root)


def wait_for(predicate, timeout=3.):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError('camera writer did not reach expected state')
        time.sleep(.005)


def append_frame(archive, sequence, data=b'abcdefghijkl'):
    archive.append(data, width=2, height=2, sequence=sequence,
                   preview_rotation_deg=180, timestamp_ns=sequence * 1000)


def track_sync(monkeypatch):
    """Record the actual descriptor's file, including POSIX directory fsync."""
    names = {}
    events = []
    original_open = Path.open
    original_os_open = os.open
    original_fsync = os.fsync

    def path_open(path, *args, **kwargs):
        stream = original_open(path, *args, **kwargs)
        names[stream.fileno()] = str(path)
        return stream

    def os_open(path, *args, **kwargs):
        descriptor = original_os_open(path, *args, **kwargs)
        names[descriptor] = str(path)
        return descriptor

    def fsync(descriptor):
        events.append(Path(names.get(descriptor, 'unknown')))
        original_fsync(descriptor)

    monkeypatch.setattr(Path, 'open', path_open)
    monkeypatch.setattr(os, 'open', os_open)
    monkeypatch.setattr(os, 'fsync', fsync)
    return events, names, original_fsync


def test_rotation_never_fsyncs_live_and_close_commits_after_all_data(archive_temp, monkeypatch):
    events, _, _ = track_sync(monkeypatch)
    # CameraArchive's hashes must be streamed, not reread from disk at rotation
    # or close. Tests independently read the final artifacts below.
    monkeypatch.setattr(source_archive, 'digest', lambda path: pytest.fail('archive reread for hash'))
    archive = CameraArchive(archive_temp / 'camera', 'right_front', 'epoch', chunk_bytes=15)
    payloads = [bytes(range(sequence, sequence + 12)) for sequence in range(8)]
    try:
        for sequence, payload in enumerate(payloads):
            append_frame(archive, sequence, payload)
            wait_for(lambda: archive.persisted == sequence + 1)
        assert archive.queue.maxsize == 256
        assert len(archive.chunks) == 7  # Last buffered chunk closes with capture.
        assert events == []
        assert not (archive.root / 'summary.json').exists()
        assert not archive.final_fsync_complete

        result = archive.close()
        expected_data = [archive.root / ('frames-%06d.bgr' % i) for i in range(8)]
        expected_data.append(archive.root / 'frames.jsonl')
        assert events[:9] == expected_data
        assert events[9] == archive.root / 'summary.json.tmp'
        assert result['status'] == 'COMPLETE'
        assert result['synchronized'] and result['closed_normally']
        assert result['final_fsync_complete'] and result['index_synchronized']
        assert result['chunks_synchronized'] == 8
        assert result['captured_frames'] == result['persisted_frames'] == 8
        assert result['dropped_frames'] == 0
        assert result['queue_max_items'] == 256
        assert result['estimated_queued_bytes'] == 256 * 12
        assert not result['exposure_completeness_verified']
        assert result['index_sha256'] == hashlib.sha256((archive.root / 'frames.jsonl').read_bytes()).hexdigest()
        rows = [json.loads(row) for row in (archive.root / 'frames.jsonl').read_text(encoding='utf-8').splitlines()]
        for payload, row, chunk in zip(payloads, rows, result['chunks']):
            blob = (archive.root / row['file']).read_bytes()
            assert source_archive.read_camera_frame(archive.root, row) == payload
            assert row['stored_sha256'] == hashlib.sha256(blob[row['offset']:row['offset'] + row['length']]).hexdigest()
            assert row['preview_rotation_deg'] == 180
            assert row['representation'] == 'DECODED_BGR8_BEFORE_PREVIEW_ROTATION'
            assert row['payload_sha256'] == hashlib.sha256(payload).hexdigest()
            assert chunk['bytes'] == len(blob)
            assert chunk['sha256'] == hashlib.sha256(blob).hexdigest()
        assert json.loads((archive.root / 'summary.json').read_text(encoding='utf-8')) == result
        assert archive.close() == result  # No second flush or duplicate commit.
    finally:
        if archive.thread.is_alive():
            archive.closing = True
            archive.thread.join(3.)


def test_no_summary_is_visible_while_final_data_fsync_is_blocked(archive_temp, monkeypatch):
    events, names, real_fsync = track_sync(monkeypatch)
    entered = threading.Event()
    release = threading.Event()
    outcome = {}
    archive = CameraArchive(archive_temp / 'camera', 'right_front', 'epoch', chunk_bytes=15)

    def delayed_fsync(descriptor):
        path = Path(names[descriptor])
        events.append(path)
        if path.name == 'frames-000000.bgr':
            entered.set()
            if not release.wait(3.):
                raise OSError('test barrier release timeout')
        real_fsync(descriptor)

    def close_archive():
        try:
            outcome['result'] = archive.close(timeout=5.)
        except Exception as error:
            outcome['error'] = error

    monkeypatch.setattr(os, 'fsync', delayed_fsync)
    closer = threading.Thread(target=close_archive)
    try:
        append_frame(archive, 1)
        append_frame(archive, 2)
        wait_for(lambda: archive.persisted == 2)
        assert events == []
        closer.start()
        assert entered.wait(3.)
        assert not archive.final_fsync_complete
        assert not (archive.root / 'summary.json').exists()
        assert not (archive.root / 'partial_summary.json').exists()
        release.set()
        closer.join(5.)
        assert not closer.is_alive()
        assert 'error' not in outcome
        assert outcome['result']['status'] == 'COMPLETE'
    finally:
        release.set()
        if closer.ident is not None:
            closer.join(5.)
        archive.closing = True
        archive.thread.join(5.)


@pytest.mark.parametrize('failed_file,chunk_count', [('frames-000001.bgr', 1), ('frames.jsonl', 2)])
def test_data_fsync_failure_has_only_partial_diagnostic(archive_temp, monkeypatch, failed_file, chunk_count):
    events, names, real_fsync = track_sync(monkeypatch)
    archive = CameraArchive(archive_temp / 'camera', 'right_front', 'epoch', chunk_bytes=15)

    def failing_fsync(descriptor):
        path = Path(names[descriptor])
        events.append(path)
        if path.name == failed_file:
            raise OSError('injected data sync failure: ' + failed_file)
        real_fsync(descriptor)

    monkeypatch.setattr(os, 'fsync', failing_fsync)
    append_frame(archive, 1)
    append_frame(archive, 2)
    with pytest.raises(RuntimeError, match='injected data sync failure'):
        archive.close()
    assert not (archive.root / 'summary.json').exists()
    diagnostic = json.loads((archive.root / 'partial_summary.json').read_text(encoding='utf-8'))
    assert diagnostic['status'] == 'PARTIAL'
    assert not diagnostic['synchronized'] and not diagnostic['closed_normally']
    assert not diagnostic['final_fsync_complete'] and not diagnostic['index_synchronized']
    assert diagnostic['chunks_synchronized'] == chunk_count
    assert diagnostic['captured_frames'] == diagnostic['persisted_frames'] == 2
    assert failed_file in diagnostic['error']


def test_summary_sync_failure_cannot_leave_complete_marker(archive_temp, monkeypatch):
    original_atomic_json = source_archive.atomic_json

    def fail_after_rename(path, value):
        original_atomic_json(path, value)
        if Path(path).name == 'summary.json':
            raise OSError('injected summary directory sync failure')

    monkeypatch.setattr(source_archive, 'atomic_json', fail_after_rename)
    archive = CameraArchive(archive_temp / 'camera', 'right_front', 'epoch')
    append_frame(archive, 1)
    with pytest.raises(RuntimeError, match='CAMERA_ARCHIVE_SUMMARY_SYNC_FAILED'):
        archive.close()
    assert not (archive.root / 'summary.json').exists()
    diagnostic = json.loads((archive.root / 'partial_summary.json').read_text(encoding='utf-8'))
    assert diagnostic['status'] == 'PARTIAL'
    assert diagnostic['final_fsync_complete']
    assert not diagnostic['synchronized'] and not diagnostic['closed_normally']


def test_bounded_queue_overflow_remains_failed_after_drain(archive_temp):
    release = threading.Event()

    class PausedArchive(CameraArchive):
        def _run(self):
            if release.wait(3.):
                super()._run()
            else:
                self._fail('test writer start timeout')

    archive = PausedArchive(archive_temp / 'camera', 'right_front', 'epoch', max_items=1)
    try:
        append_frame(archive, 1)
        with pytest.raises(RuntimeError, match='CAMERA_ARCHIVE_QUEUE_FULL'):
            append_frame(archive, 2)
        assert archive.queue.maxsize == archive.queue.qsize() == 1
        release.set()
        with pytest.raises(RuntimeError, match='CAMERA_ARCHIVE_QUEUE_FULL'):
            archive.close()
        result = json.loads((archive.root / 'summary.json').read_text(encoding='utf-8'))
        assert result['status'] == 'PARTIAL'
        assert result['final_fsync_complete']
        assert not result['synchronized'] and not result['closed_normally']
        assert result['captured_frames'] == 2
        assert result['persisted_frames'] == 1
        assert result['dropped_frames'] == 1
        assert result['queue_max_items'] == result['queue_high_watermark_items'] == 1
    finally:
        release.set()
        archive.closing = True
        archive.thread.join(5.)


def test_default_256_frame_queue_absorbs_bounded_writer_stall_then_drains(archive_temp):
    release = threading.Event()
    class PausedArchive(CameraArchive):
        def _run(self):
            if release.wait(3.):
                super()._run()
            else:
                self._fail('test writer start timeout')
    archive = PausedArchive(archive_temp / 'camera', 'right_front', 'epoch')
    try:
        for sequence in range(256):
            append_frame(archive, sequence)
        assert archive.queue.maxsize == archive.queue.qsize() == 256
        release.set()
        result = archive.close()
        assert result['status'] == 'COMPLETE'
        assert result['captured_frames'] == result['persisted_frames'] == 256
        assert result['dropped_frames'] == 0 and result['final_fsync_complete']
        assert result['queue_max_items'] == result['queue_high_watermark_items'] == 256
        assert result['estimated_queued_bytes'] == result['queue_high_watermark_bytes'] == 256 * 12
        rows = [json.loads(row) for row in (archive.root / 'frames.jsonl').read_text(encoding='utf-8').splitlines()]
        assert [row['sequence'] for row in rows] == list(range(256))
    finally:
        release.set()
        archive.closing = True
        archive.thread.join(5.)


def test_close_timeout_never_commits_over_running_writer(archive_temp):
    release = threading.Event()

    class PausedArchive(CameraArchive):
        def _run(self):
            if release.wait(3.):
                super()._run()
            else:
                self._fail('test writer start timeout')

    archive = PausedArchive(archive_temp / 'camera', 'right_front', 'epoch')
    try:
        append_frame(archive, 1)
        with pytest.raises(RuntimeError, match='CAMERA_ARCHIVE_CLOSE_TIMEOUT'):
            archive.close(timeout=.01)
        assert archive.thread.is_alive()
        assert not (archive.root / 'summary.json').exists()
        assert not (archive.root / 'partial_summary.json').exists()
        release.set()
        archive.thread.join(5.)
        with pytest.raises(RuntimeError, match='CAMERA_ARCHIVE_CLOSE_TIMEOUT'):
            archive.close()
        diagnostic = json.loads((archive.root / 'summary.json').read_text(encoding='utf-8'))
        assert diagnostic['status'] == 'PARTIAL'
        assert not diagnostic['synchronized'] and not diagnostic['closed_normally']
    finally:
        release.set()
        archive.closing = True
        archive.thread.join(5.)


def test_source_failure_is_partial_even_with_complete_durable_archive(archive_temp):
    archive = CameraArchive(archive_temp / 'camera', 'right_front', 'epoch')
    append_frame(archive, 1)
    result = archive.close(source_error='CAMERA_DISCONNECTED')
    assert result['synchronized'] and result['final_fsync_complete']
    assert not result['closed_normally']
    assert result['status'] == 'PARTIAL'
    assert result['source_error'] == 'CAMERA_DISCONNECTED'


@pytest.mark.parametrize('kwargs', [{'max_items': 0}, {'max_items': -1}, {'chunk_bytes': 0}])
def test_invalid_archive_limits_cannot_create_unbounded_queue(archive_temp, kwargs):
    with pytest.raises(ValueError, match='positive'):
        CameraArchive(archive_temp / 'camera', 'right_front', 'epoch', **kwargs)
    assert not (archive_temp / 'camera').exists()
