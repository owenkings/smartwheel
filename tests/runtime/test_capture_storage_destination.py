"""External archive mount loss and independent local-journal capacity checks."""
from pathlib import Path
from types import SimpleNamespace

import pytest

from wc_runtime import capture_storage as storage


class Guard:
    error = None
    calls = 0

    def check(self):
        self.calls += 1
        if self.error:
            raise ValueError(self.error)


def test_mount_loss_is_detected_during_throttled_scan(tmp_path, monkeypatch):
    staged, final, source = [tmp_path / x for x in ('ram', 'usb', 'journals')]
    for path in (staged, final, source):
        path.mkdir()
    clock = [1.0]
    monkeypatch.setattr(storage.time, 'monotonic', lambda: clock[0])
    guard = Guard()
    monitor = storage.StorageMonitor(staged, final, source, memory_staging=True,
        reserve_bytes=0, close_headroom_bytes=0, destination_guard=guard)
    assert monitor.check()['status'] == 'AVAILABLE'
    clock[0] += 0.1
    assert monitor.check() is None
    guard.error = 'required USB no longer mounted'
    with pytest.raises(RuntimeError, match='required USB no longer mounted'):
        monitor.check()
    assert monitor.last_report['status'] == 'UNKNOWN'
    assert guard.calls == 3


def test_large_usb_does_not_hide_full_local_journal_disk(tmp_path, monkeypatch):
    staged, final, source = [tmp_path / x for x in ('ram', 'usb', 'journals')]
    for path in (staged, final, source):
        path.mkdir()
    real_stat = Path.stat
    def fake_stat(path, *args, **kwargs):
        info = real_stat(path, *args, **kwargs)
        if path == source:
            return SimpleNamespace(st_dev=info.st_dev + 1)
        return info
    monkeypatch.setattr(Path, 'stat', fake_stat)
    monkeypatch.setattr(storage.shutil, 'disk_usage', lambda path:
        SimpleNamespace(free=299 if Path(path) == source else 10**12))
    monitor = storage.StorageMonitor(staged, final, source, memory_staging=True,
        reserve_bytes=200, close_headroom_bytes=100, destination_guard=Guard())
    with pytest.raises(RuntimeError, match='CAPTURE_STORAGE_RESERVE_REACHED'):
        monitor.check()
    assert monitor.last_report['free_bytes'] == 10**12
    assert monitor.last_report['source_free_bytes'] == 299
    assert monitor.last_report['source_margin_bytes'] == -1
