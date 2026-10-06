"""Bounded FIFO durability acknowledgements for immutable raw observations.

The caller encodes a complete JSON document with ``allow_nan=False`` before
``submit``. This module never changes/decimates points or serializes objects.
Only ``poll`` results with ``error is None`` authorize downstream publication.

Capacity includes the active write, waiting writes and unpolled results. The
single worker owns disk I/O; submit/poll do not stat, create or sync files.
Raw destinations are immutable: same-directory hard-link publication provides
atomic create-if-absent, followed by directory fsync. os.replace is deliberately
not used because it would overwrite an existing raw observation.

close requests cancellation and joins for a bounded time. Python cannot cancel
a kernel-blocked fsync: a timeout is an explicit error, not proof that the worker
or its temporary file is gone. The daemon worker cleans its own temporary file
when the I/O returns, and a subsequent close can confirm it has terminated.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import os
from pathlib import Path
import threading
import time
import uuid

_commit_timing = threading.local()


def _mark(stage):
    timing = getattr(_commit_timing, 'values', None)
    if timing is not None:
        timing[stage] = time.monotonic_ns()


class ArchiveError(RuntimeError):
    def __init__(self, code: str, detail: str = ''):
        self.code = code
        super().__init__(code + (': ' + detail if detail else ''))


class ArchiveCloseTimeout(ArchiveError):
    def __init__(self):
        super().__init__('ARCHIVE_CLOSE_TIMEOUT', 'worker still owns pending I/O; cleanup is not confirmed')


@dataclass(frozen=True)
class ArchiveResult:
    key: str
    path: Path
    error: str | None = None
    timing_ns: tuple[tuple[str, int], ...] = ()


@dataclass(frozen=True)
class _Job:
    key: str
    path: Path
    payload: bytes
    submitted_ns: int


def _cancelled(stop: threading.Event) -> None:
    if stop.is_set():
        raise ArchiveError('ARCHIVE_CANCELLED', 'close or preceding write failure requested cancellation')


def _commit_bytes(path: Path, payload: bytes, stop: threading.Event) -> None:
    """Worker-only file transaction; never remove or replace a final path."""
    _cancelled(stop)
    parent = path.parent
    if parent.resolve(strict=True) != parent:
        raise ArchiveError('ARCHIVE_PATH', 'destination directory contains a symlink')
    directory = os.open(parent, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW)
    temporary = '.' + path.name + '.archive-' + uuid.uuid4().hex + '.tmp'
    created = False
    try:
        try:
            _cancelled(stop)
            descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL |
                                 os.O_CLOEXEC | os.O_NOFOLLOW, 0o600, dir_fd=directory)
            created = True
            try:
                output = os.fdopen(descriptor, 'wb')
            except BaseException:
                os.close(descriptor)
                raise
            with output:
                if output.write(payload) != len(payload):
                    raise ArchiveError('ARCHIVE_SHORT_WRITE')
                output.flush()
                _mark('file_fsync_start')
                os.fsync(output.fileno())
                _mark('file_fsync_end')
            _cancelled(stop)
            # Atomic no-replace publication, including an existing symlink.
            # Both names are relative to the same held directory descriptor.
            try:
                os.link(temporary, path.name, src_dir_fd=directory,
                        dst_dir_fd=directory, follow_symlinks=False)
            except OSError as error:
                import errno
                if error.errno not in (errno.EPERM, errno.EOPNOTSUPP, errno.ENOSYS):
                    raise
                # The USB archive may be exFAT: preserve the held-directory
                # identity, then use the same configured-volume commit lock as
                # mapping JSON and map packages. Unknown destinations fail.
                info, held = parent.stat(), os.fstat(directory)
                if (info.st_dev, info.st_ino) != (held.st_dev, held.st_ino):
                    raise ArchiveError('ARCHIVE_PATH', 'directory changed before USB publication')
                from wc_runtime.storage_atomic import rename_on_configured_archive
                rename_on_configured_archive(parent/temporary, path)
                created = False  # rename consumed our temporary name.
        finally:
            if created:
                os.unlink(temporary, dir_fd=directory)
        _mark('directory_fsync_start')
        os.fsync(directory)
        _mark('directory_fsync_end')
    finally:
        os.close(directory)


def _error_text(error: BaseException) -> str:
    # Include cleanup exceptions and their original cause, rather than hiding
    # the first write failure behind a later unlink/close failure.
    parts, seen = [], set()
    while error is not None and id(error) not in seen:
        seen.add(id(error))
        parts.append(type(error).__name__ + ': ' + str(error))
        error = error.__cause__ or error.__context__
    return ' <- '.join(parts)


class DurableArchive:
    """One fail-latching worker with at most eight unacknowledged jobs.

    ``submit(key, absolute_path, immutable_bytes)`` accepts without disk I/O or
    raises ArchiveError synchronously. ``poll()`` drains ordered ArchiveResult
    items; only a result with error=None means the final file and directory were
    fsynced. Any failure latches and rejects future submit calls; queued jobs get
    explicit cancellation results in their original order. After a disk error a
    final file can exist without an acknowledgement (e.g. directory fsync failed);
    do not publish it or retry by overwriting it.

    ``close(timeout=2.0)`` stops admission, requests cancellation, and joins. It
    raises ArchiveCloseTimeout if still alive; it does not discard results. Call
    poll after close to retrieve terminal outcomes for every accepted job. Keys
    and destinations must be unique among all currently unpolled jobs; existing
    destination files additionally enforce persistent no-overwrite semantics.
    """

    def __init__(self, capacity: int = 8):
        if type(capacity) is not int or not 1 <= capacity <= 8:
            raise ArchiveError('ARCHIVE_CAPACITY', 'capacity must be an integer from 1 through 8')
        self.capacity = capacity
        self._condition = threading.Condition()
        self._waiting: deque[_Job] = deque()
        self._results: deque[ArchiveResult] = deque()
        self._outstanding: dict[str, Path] = {}
        self._paths: set[Path] = set()
        self._closing = False
        self._failure: str | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._work, name='wc-raw-archive', daemon=True)
        self._thread.start()

    def submit(self, key: str, path: Path, payload: bytes) -> None:
        if type(key) is not str or not key:
            raise ArchiveError('ARCHIVE_KEY', 'nonempty string required')
        if type(payload) is not bytes or not payload:
            raise ArchiveError('ARCHIVE_PAYLOAD', 'nonempty immutable bytes required; encode JSON before submit')
        try:
            path = Path(path)
        except (TypeError, ValueError) as error:
            raise ArchiveError('ARCHIVE_PATH', 'path-like destination required') from error
        if not path.is_absolute() or '..' in path.parts or not path.name:
            raise ArchiveError('ARCHIVE_PATH', 'absolute destination without parent traversal required')
        with self._condition:
            if self._closing:
                raise ArchiveError('ARCHIVE_CLOSED')
            if self._failure is not None:
                raise ArchiveError('ARCHIVE_FAILED', self._failure)
            if key in self._outstanding or path in self._paths:
                raise ArchiveError('ARCHIVE_DUPLICATE', 'key or path already has an unpolled job')
            if len(self._outstanding) >= self.capacity:
                raise ArchiveError('ARCHIVE_QUEUE_FULL', 'active, waiting and unpolled total reached capacity')
            self._outstanding[key] = path
            self._paths.add(path)
            self._waiting.append(_Job(key, path, payload, time.monotonic_ns()))
            self._condition.notify()

    def poll(self) -> list[ArchiveResult]:
        with self._condition:
            polled = time.monotonic_ns()
            results = [ArchiveResult(r.key, r.path, r.error, r.timing_ns+(('polled',polled),)) for r in self._results]
            self._results.clear()
            for result in results:
                self._outstanding.pop(result.key)
                self._paths.remove(result.path)
            return results

    @property
    def pending_count(self) -> int:
        """Active + waiting + completed but unpolled jobs, always <= capacity."""
        with self._condition:
            return len(self._outstanding)

    @property
    def failure(self) -> str | None:
        with self._condition:
            return self._failure

    @property
    def alive(self) -> bool:
        return self._thread.is_alive()

    def close(self, timeout: float = 2.0) -> None:
        if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or \
                not math.isfinite(timeout) or not 0 <= timeout <= 60:
            raise ArchiveError('ARCHIVE_CLOSE_TIMEOUT_ARGUMENT', 'finite timeout from 0 through 60 seconds required')
        with self._condition:
            self._closing = True
            self._stop.set()
            self._condition.notify_all()
        self._thread.join(timeout)
        if self._thread.is_alive():
            raise ArchiveCloseTimeout()

    def _work(self) -> None:
        while True:
            with self._condition:
                self._condition.wait_for(lambda: bool(self._waiting) or self._closing)
                if not self._waiting:
                    return
                job = self._waiting.popleft()
                prior = self._failure
                closing = self._closing
            error = None
            timing = dict(submitted=job.submitted_ns, dequeued=time.monotonic_ns())
            _commit_timing.values = timing
            if prior is not None:
                error = 'ARCHIVE_CANCELLED_AFTER_FAILURE: ' + prior
            elif closing:
                error = 'ARCHIVE_CANCELLED: writer is closing'
            else:
                try:
                    _commit_bytes(job.path, job.payload, self._stop)
                except BaseException as failure:
                    # Every accepted job must have an explicit terminal result,
                    # including unexpected exceptions raised inside this worker.
                    error = _error_text(failure)
            with self._condition:
                if error is not None and self._failure is None:
                    self._failure = error
                    self._stop.set()
                timing['ack_ready'] = time.monotonic_ns()
                self._results.append(ArchiveResult(job.key, job.path, error, tuple(timing.items())))
            _commit_timing.values = None
