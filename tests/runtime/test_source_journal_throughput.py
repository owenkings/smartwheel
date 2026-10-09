"""Deterministic FIFO/durability failure tests; no ROS, serial or hardware."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import threading
import uuid

import pytest

from wc_runtime import source_archive
from wc_runtime.source_archive import SourceJournal


@pytest.fixture
def journal_parent():
    # Avoid Windows pytest temporary-directory ACL behavior; only this unique
    # synthetic fixture directory is created and removed by the test.
    parent = Path(__file__).resolve().parent
    directory = parent/('.source_journal_test_'+uuid.uuid4().hex)
    directory.mkdir()
    try:
        yield directory
    finally:
        assert directory.resolve().parent == parent
        assert directory.name.startswith('.source_journal_test_')
        shutil.rmtree(directory)


class GatedJournal(SourceJournal):
    """Hold the writer until a deterministic source burst is queued."""
    def __init__(self, *args, **kwargs):
        self.gate = threading.Event()
        super().__init__(*args, **kwargs)

    def _run(self):
        if not self.gate.wait(5):
            self.error = 'SYNTHETIC_WRITER_GATE_TIMEOUT'
            return
        super()._run()


def encoded_bytes(rows):
    # Match the original text writer's newline behavior on each OS exactly.
    return ''.join(json.dumps(row, ensure_ascii=False, allow_nan=False, separators=(',', ':'))
                   +os.linesep for row in rows).encode('utf-8')


def test_burst_batches_fifo_without_reparsing_and_closes_tail(journal_parent, monkeypatch):
    rows = [dict(event='byte_batch' if i % 3 == 0 else 'frame', sequence=i, note='原始帧')
            for i in range(513)]
    journal = GatedJournal(journal_parent/'imu', 'imu', max_items=1024)
    for row in rows:
        journal.append(row)
    monkeypatch.setattr(source_archive.json, 'loads',
                        lambda *_args, **_kwargs: pytest.fail('writer re-parsed its own encoded JSON'))
    journal.gate.set()
    summary = journal.close()
    payload = (journal.root/'events.jsonl').read_bytes()
    assert payload == encoded_bytes(rows)
    assert summary['events_sha256'] == hashlib.sha256(payload).hexdigest()
    assert summary['accepted_records'] == summary['persisted_records'] == 513
    assert summary['rejected_records'] == 0 and summary['synchronized'] and summary['closed_normally']
    assert summary['event_counts'] == {'byte_batch': 171, 'frame': 342}
    metrics = summary['journal_writer']
    assert metrics['queue_max_items'] == 1024 and metrics['queue_high_watermark_items'] == 513
    assert metrics['queue_high_watermark_bytes'] == len(encoded_bytes(rows).replace(b'\r\n', b'\n'))
    assert metrics['queue_remaining_items'] == 0
    assert metrics['write_batches_completed'] == metrics['write_batch_attempts'] == 5
    assert metrics['write_batch_max_items'] == metrics['batch_max_items'] == 128
    assert metrics['write_batch_max_bytes'] <= metrics['batch_target_bytes']
    assert 0 <= metrics['write_batch_max_ns'] <= metrics['write_batch_total_ns']
    with pytest.raises(RuntimeError, match='already closing'):
        journal.append({'event': 'frame', 'sequence': 514})


def test_byte_target_lookahead_preserves_final_record_and_default_kind(journal_parent, monkeypatch):
    monkeypatch.setattr(SourceJournal, 'BATCH_MAX_BYTES', 150)
    rows = [dict(sequence=i, payload='x'*80) for i in range(11)]
    journal = GatedJournal(journal_parent/'imu', 'imu', max_items=32)
    for row in rows:
        journal.append(row)
    journal.gate.set()
    summary = journal.close()
    assert (journal.root/'events.jsonl').read_bytes() == encoded_bytes(rows)
    assert summary['event_counts'] == {'frame': 11}
    assert summary['accepted_records'] == summary['persisted_records'] == 11
    assert summary['journal_writer']['write_batches_completed'] == 11
    assert summary['journal_writer']['write_batch_max_bytes'] <= 150
    assert summary['synchronized'] and summary['closed_normally']


def test_single_large_record_is_unchanged_without_unbounded_batching(journal_parent, monkeypatch):
    monkeypatch.setattr(SourceJournal, 'BATCH_MAX_BYTES', 64)
    rows = [dict(event='frame', sequence=i, payload='x'*256) for i in range(3)]
    journal = GatedJournal(journal_parent/'imu', 'imu', max_items=4)
    for row in rows:
        journal.append(row)
    journal.gate.set()
    summary = journal.close()
    assert (journal.root/'events.jsonl').read_bytes() == encoded_bytes(rows)
    assert summary['journal_writer']['write_batch_max_items'] == 1
    assert summary['journal_writer']['write_batches_completed'] == 3
    assert summary['synchronized']


def test_queue_full_still_fails_after_accepted_prefix_is_drained(journal_parent):
    journal = GatedJournal(journal_parent/'imu', 'imu', max_items=2)
    rows = [dict(event='frame', sequence=i) for i in range(2)]
    for row in rows:
        journal.append(row)
    with pytest.raises(RuntimeError, match='SOURCE_JOURNAL_QUEUE_FULL'):
        journal.append(dict(event='frame', sequence=2))
    assert journal.queue.qsize() == journal.queue.maxsize == 2
    journal.gate.set()
    with pytest.raises(RuntimeError, match='SOURCE_JOURNAL_QUEUE_FULL'):
        journal.close(source_error='SOURCE_JOURNAL_QUEUE_FULL')
    summary = json.loads((journal.root/'summary.json').read_text(encoding='utf-8'))
    assert summary['accepted_records'] == summary['persisted_records'] == 2
    assert summary['rejected_records'] == 1
    assert not summary['synchronized'] and not summary['closed_normally']
    assert summary['source_error'] == summary['error'] == 'SOURCE_JOURNAL_QUEUE_FULL'
    assert summary['journal_writer']['queue_high_watermark_items'] == 2
    assert (journal.root/'events.jsonl').read_bytes() == encoded_bytes(rows)


@pytest.mark.parametrize('failure', ['write', 'short_write', 'flush', 'fsync'])
def test_writer_failures_never_claim_synchronized_completion(journal_parent, monkeypatch, failure):
    original_open = Path.open
    class FailingStream:
        def __init__(self, stream):
            self.stream = stream
        def __enter__(self):
            self.stream.__enter__()
            return self
        def __exit__(self, *args):
            return self.stream.__exit__(*args)
        def write(self, value):
            if failure == 'write':
                raise OSError('SYNTHETIC_WRITE_FAILED')
            if failure == 'short_write':
                self.stream.write(value[:len(value)//2])
                return len(value)//2
            return self.stream.write(value)
        def flush(self):
            if failure == 'flush':
                raise OSError('SYNTHETIC_FLUSH_FAILED')
            return self.stream.flush()
        def fileno(self):
            return self.stream.fileno()
    def opened(path, *args, **kwargs):
        stream = original_open(path, *args, **kwargs)
        return FailingStream(stream) if path.name == 'events.jsonl' and args and args[0] == 'x' else stream
    monkeypatch.setattr(Path, 'open', opened)
    original_fsync = source_archive.os.fsync
    def fsync(descriptor):
        if failure == 'fsync' and threading.current_thread().name == 'source-evidence':
            raise OSError('SYNTHETIC_FSYNC_FAILED')
        return original_fsync(descriptor)
    monkeypatch.setattr(source_archive.os, 'fsync', fsync)
    journal = GatedJournal(journal_parent/'imu', 'imu', max_items=32)
    for i in range(9):
        journal.append(dict(event='frame', sequence=i))
    journal.gate.set()
    with pytest.raises(RuntimeError):
        journal.close()
    summary = json.loads((journal.root/'summary.json').read_text(encoding='utf-8'))
    assert summary['error'] and not summary['synchronized'] and not summary['closed_normally']
    assert summary['accepted_records'] == 9
    assert summary['persisted_records'] == (0 if failure in ('write', 'short_write') else 9)
    assert summary['journal_writer']['write_batch_attempts'] == 1
    assert summary['journal_writer']['write_batches_completed'] == (0 if failure in ('write', 'short_write') else 1)
    assert summary['events_sha256'] == hashlib.sha256((journal.root/'events.jsonl').read_bytes()).hexdigest()


@pytest.mark.parametrize('max_items', [0, -1, True, 1.5])
def test_unbounded_or_invalid_capacity_is_rejected_before_directory_creation(journal_parent, max_items):
    directory = journal_parent/'imu'
    with pytest.raises(ValueError, match='positive bounded'):
        SourceJournal(directory, 'imu', max_items=max_items)
    assert not directory.exists()
