"""Read-only, bounded metadata accounting for a capture's remaining disk demand.

This monitor stops new acquisition before capacity is exhausted; it does not
reserve filesystem blocks or certify that another writer cannot consume them.
Only stopped source/recorder owners permit ``check(closing=True)``.
"""
import json
import math
import os
from pathlib import Path
import shutil
import stat
import time


MAX_SCAN_ENTRIES = 10_000
MAX_SCAN_DEPTH = 20
CLOSE_GROWTH_SECONDS = 135
FINAL_REPORT_HEADROOM_BYTES = 16 * 1024**2


def _unlinked_ancestors(path):
    """Check missing future children too: an existing parent may not be linked."""
    for candidate in (path, *path.parents):
        try:
            mode = os.lstat(candidate).st_mode
        except FileNotFoundError:
            continue
        if stat.S_ISLNK(mode):
            raise ValueError('linked capture storage path: ' + str(candidate))


def _tree_bytes(path, state, *, allow_missing=False, depth=0, allowed_socket=None):
    """Count logical bytes, since transfer copies content rather than sparse blocks."""
    if depth == 0:
        _unlinked_ancestors(path)
    try:
        mode = os.lstat(path).st_mode
    except FileNotFoundError:
        if allow_missing:
            return 0
        raise
    if stat.S_ISLNK(mode) or not stat.S_ISDIR(mode):
        raise ValueError('capture storage requires an ordinary directory: ' + str(path))
    total = 0
    with os.scandir(path) as entries:
        for entry in entries:
            state['entries_scanned'] += 1
            if state['entries_scanned'] > MAX_SCAN_ENTRIES:
                raise ValueError('capture storage entry limit exceeded')
            if depth + 1 > MAX_SCAN_DEPTH:
                raise ValueError('capture storage depth limit exceeded')
            if entry.is_symlink():
                raise ValueError('linked capture storage entry: ' + str(Path(path)/entry.name))
            try:
                metadata = entry.stat(follow_symlinks=False)
            except FileNotFoundError:
                # Atomic ready/manifest replacements can disappear between
                # readdir and lstat. They are not unexplained permission/I/O
                # failures, and the next bounded sample discovers the new name.
                state['vanished_entries'] += 1
                continue
            if stat.S_ISLNK(metadata.st_mode):
                raise ValueError('linked capture storage entry: ' + str(Path(path)/entry.name))
            if stat.S_ISDIR(metadata.st_mode):
                total += _tree_bytes(Path(path)/entry.name, state, depth=depth + 1,
                                     allowed_socket=allowed_socket)
            elif stat.S_ISREG(metadata.st_mode):
                total += metadata.st_size
            elif stat.S_ISSOCK(metadata.st_mode) and Path(path)/entry.name == allowed_socket:
                # The live owner exposes exactly this endpoint. It carries no
                # archive payload and must be unlinked before stopped transfer.
                state['manual_socket_count'] += 1
            else:
                raise ValueError('nonregular capture storage entry: ' + str(Path(path)/entry.name))
    return total


def _disk_path(path):
    _unlinked_ancestors(path)
    candidate = path
    while True:
        try:
            metadata = os.lstat(candidate)
        except FileNotFoundError:
            if candidate == candidate.parent:
                raise
            candidate = candidate.parent
            continue
        if not stat.S_ISDIR(metadata.st_mode):
            raise ValueError('final capture storage requires a directory: ' + str(candidate))
        return candidate


class StorageMonitor:
    """Account pending copies plus observed growth without a duration estimate.

    ``remaining_after_bytes`` is free disk after pending copies and headroom,
    before the protected reserve. ``margin_bytes`` additionally subtracts that
    reserve. First observation establishes a baseline, never a snapshot rate.
    """
    def __init__(self, staging_directory, final_directory, source_session_directory,
                 *, memory_staging, reserve_bytes=2 * 1024**3,
                 close_headroom_bytes=256 * 1024**2, interval_s=1.0,
                 close_growth_seconds=CLOSE_GROWTH_SECONDS, destination_guard=None):
        if type(memory_staging) is not bool:
            raise ValueError('memory_staging must be boolean')
        for name, value in (('reserve_bytes', reserve_bytes), ('close_headroom_bytes', close_headroom_bytes)):
            if type(value) is not int or value < 0:
                raise ValueError(name + ' must be nonnegative integer bytes')
        if type(interval_s) not in (int, float) or not math.isfinite(interval_s) or interval_s <= 0:
            raise ValueError('interval_s must be finite and positive')
        if (type(close_growth_seconds) not in (int, float) or not math.isfinite(close_growth_seconds)
                or close_growth_seconds <= 0):
            raise ValueError('close_growth_seconds must be finite and positive')
        self.staging_directory = Path(staging_directory).absolute()
        self.final_directory = Path(final_directory).absolute()
        self.source_session_directory = Path(source_session_directory).absolute()
        self.memory_staging = memory_staging
        self.reserve_bytes = reserve_bytes
        self.close_headroom_bytes = close_headroom_bytes
        self.interval_s = float(interval_s)
        self.close_growth_seconds = float(close_growth_seconds)
        self.last_report = None
        self._previous = None
        self.peak_growth_bytes_per_s = 0.0
        self.destination_guard = destination_guard
        self.peak_source_growth_bytes_per_s = 0.0

    def check(self, force=False, closing=False):
        now = time.monotonic()
        state = {'entries_scanned': 0, 'vanished_entries': 0,
                 'manual_socket_count': 0, 'manual_socket_bytes': 0}
        try:
            # Mount identity is checked even when the more expensive scan is throttled.
            if self.destination_guard is not None:
                self.destination_guard.check()
            if (not force and not closing and self._previous is not None
                    and now - self._previous['monotonic_s'] < self.interval_s):
                return None
            disk_path = _disk_path(self.final_directory)
            if closing:
                _unlinked_ancestors(self.staging_directory)
                try:
                    socket_mode = os.lstat(self.staging_directory/'manual.sock').st_mode
                except FileNotFoundError:
                    socket_mode = None
                if socket_mode is not None and stat.S_ISSOCK(socket_mode):
                    raise ValueError('manual socket remains after stopped capture')
            allowed_socket = self.staging_directory/'manual.sock' if not closing else None
            ram_bytes = (_tree_bytes(self.staging_directory, state, allowed_socket=allowed_socket)
                         if self.memory_staging else 0)
            _unlinked_ancestors(self.source_session_directory)
            lidar_bytes = sum(_tree_bytes(self.source_session_directory/side, state, allow_missing=True)
                              for side in ('left', 'right'))
            free_bytes = shutil.disk_usage(disk_path).free
            if type(free_bytes) is not int or free_bytes < 0:
                raise ValueError('disk free bytes must be a nonnegative integer')
            source_free_bytes = None
            if self.destination_guard is not None:
                source_path = _disk_path(self.source_session_directory)
                if source_path.stat().st_dev != disk_path.stat().st_dev:
                    source_free_bytes = shutil.disk_usage(source_path).free
                    if type(source_free_bytes) is not int or source_free_bytes < 0:
                        raise ValueError('source disk free bytes must be a nonnegative integer')
        except (OSError, ValueError) as error:
            self.last_report = dict(
                status='UNKNOWN', monotonic_s=now, error=type(error).__name__ + ': ' + str(error),
                memory_staging=self.memory_staging, closing=bool(closing),
                free_bytes=None, pending_bytes=None, ram_bytes=None, lidar_bytes=None,
                headroom_bytes=None, reserve_bytes=self.reserve_bytes, remaining_after_bytes=None,
                margin_bytes=None, peak_growth_bytes_per_s=self.peak_growth_bytes_per_s, **state)
            raise RuntimeError('CAPTURE_STORAGE_MEASUREMENT_FAILED: '
                               + json.dumps(self.last_report, sort_keys=True)) from error
        pending_bytes = ram_bytes + lidar_bytes
        previous = self._previous
        if previous is not None and now > previous['monotonic_s']:
            # Final-disk consumption already happened; pending copies are still
            # to happen. Both reduce the same remaining commitment margin.
            growth = (max(0, pending_bytes - previous['pending_bytes'])
                      + max(0, previous['free_bytes'] - free_bytes))
            rate = growth / (now - previous['monotonic_s'])
            self.peak_growth_bytes_per_s = max(self.peak_growth_bytes_per_s, rate)
            if source_free_bytes is not None and previous.get('source_free_bytes') is not None:
                source_rate = max(0, previous['source_free_bytes'] - source_free_bytes) / (now - previous['monotonic_s'])
                self.peak_source_growth_bytes_per_s = max(self.peak_source_growth_bytes_per_s, source_rate)
        headroom_bytes = (FINAL_REPORT_HEADROOM_BYTES if closing else max(
            self.close_headroom_bytes, math.ceil(self.peak_growth_bytes_per_s * self.close_growth_seconds)))
        remaining_after = free_bytes - pending_bytes - headroom_bytes
        source_headroom = (FINAL_REPORT_HEADROOM_BYTES if closing else max(
            self.close_headroom_bytes, math.ceil(self.peak_source_growth_bytes_per_s * self.close_growth_seconds)))
        source_margin = None if source_free_bytes is None else source_free_bytes - source_headroom - self.reserve_bytes
        sufficient = remaining_after >= self.reserve_bytes and (source_margin is None or source_margin >= 0)
        report = dict(status='AVAILABLE' if sufficient else 'BLOCKED',
            monotonic_s=now, memory_staging=self.memory_staging, closing=bool(closing),
            free_bytes=free_bytes, pending_bytes=pending_bytes, ram_bytes=ram_bytes,
            lidar_bytes=lidar_bytes, headroom_bytes=headroom_bytes, reserve_bytes=self.reserve_bytes,
            remaining_after_bytes=remaining_after, margin_bytes=remaining_after-self.reserve_bytes,
            peak_growth_bytes_per_s=self.peak_growth_bytes_per_s,
            source_free_bytes=source_free_bytes, source_margin_bytes=source_margin,
            source_headroom_bytes=source_headroom if source_free_bytes is not None else None,
            peak_source_growth_bytes_per_s=self.peak_source_growth_bytes_per_s,
            close_growth_budget_s=self.close_growth_seconds, interval_s=self.interval_s, **state)
        self._previous = dict(monotonic_s=now, pending_bytes=pending_bytes, free_bytes=free_bytes,
                              source_free_bytes=source_free_bytes)
        self.last_report = report
        if report['status'] == 'BLOCKED':
            raise RuntimeError('CAPTURE_STORAGE_RESERVE_REACHED: ' + json.dumps(report, sort_keys=True))
        return report
