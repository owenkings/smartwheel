"""Actual storage accounting with temporary files and simulated clocks/capacity."""
import json
import os
from pathlib import Path
import socket
from types import SimpleNamespace

import pytest

from wc_runtime import capture_storage as storage


@pytest.fixture
def capture_paths(tmp_path):
    staged = tmp_path/'ram'/'session'
    final = tmp_path/'disk'/'session'
    sources = tmp_path/'run'/'sessions'/'session'
    staged.mkdir(parents=True)
    final.mkdir(parents=True)
    return staged, final, sources


def write(path, size):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b'x' * size)


def monitor(capture_paths, monkeypatch, *, free=10**12, memory=True, reserve=200, headroom=100,
            growth_budget=135):
    clock, capacity = [10.0], [free]
    monkeypatch.setattr(storage.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(storage.shutil, 'disk_usage', lambda path: SimpleNamespace(free=capacity[0]))
    value = storage.StorageMonitor(*capture_paths, memory_staging=memory,
        reserve_bytes=reserve, close_headroom_bytes=headroom, close_growth_seconds=growth_budget)
    return value, clock, capacity


def failed_report(value, **kwargs):
    with pytest.raises(RuntimeError, match='^CAPTURE_STORAGE_RESERVE_REACHED: ') as caught:
        value.check(**kwargs)
    return json.loads(str(caught.value).split(': ', 1)[1])


def test_first_snapshot_is_pending_content_not_a_growth_rate(capture_paths, monkeypatch):
    staged, final, sources = capture_paths
    write(staged/'software/snapshot.tar.gz', 1200)
    write(staged/'bag/capture.db3', 400)
    write(sources/'left/epoch/source_frames.jsonl', 300)
    write(sources/'right/epoch/source_frames.jsonl', 500)
    write(sources/'unrelated/raw', 900)
    value, _, _ = monitor(capture_paths, monkeypatch)
    report = value.check()
    assert report['ram_bytes'] == 1600 and report['lidar_bytes'] == 800
    assert report['pending_bytes'] == 2400
    assert report['headroom_bytes'] == 100 and report['peak_growth_bytes_per_s'] == 0
    assert report['remaining_after_bytes'] == 10**12 - 2500
    assert report['margin_bytes'] == 10**12 - 2700
    assert not (staged/'storage_usage.json').exists() and list(final.iterdir()) == []


def test_lidar_source_copy_is_required_even_when_staging_already_on_disk(capture_paths, monkeypatch):
    staged, _, sources = capture_paths
    write(staged/'bag/capture.db3', 5000)  # Already included in reported disk consumption.
    write(sources/'left/epoch/journal', 400)
    value, _, _ = monitor(capture_paths, monkeypatch, memory=False, free=600)
    report = failed_report(value)
    assert report['ram_bytes'] == 0 and report['pending_bytes'] == report['lidar_bytes'] == 400
    assert report['remaining_after_bytes'] == 100 and report['reserve_bytes'] == 200


def test_missing_future_source_directories_are_zero(capture_paths, monkeypatch):
    value, _, _ = monitor(capture_paths, monkeypatch)
    report = value.check()
    assert report['lidar_bytes'] == 0 and report['pending_bytes'] == 0


def test_scan_is_throttled_and_force_rechecks_changed_space(capture_paths, monkeypatch):
    value, clock, capacity = monitor(capture_paths, monkeypatch, free=1000)
    original = storage.os.scandir
    scans = []
    def counted(path):
        scans.append(path)
        return original(path)
    monkeypatch.setattr(storage.os, 'scandir', counted)
    assert value.check()['status'] == 'AVAILABLE'
    first_count = len(scans)
    capacity[0] = 299
    clock[0] += .25
    for _ in range(20):
        assert value.check() is None
    assert len(scans) == first_count
    report = failed_report(value, force=True)
    assert len(scans) > first_count and report['free_bytes'] == 299


def test_growth_peak_includes_ram_and_final_disk_consumption_and_never_decays(capture_paths, monkeypatch):
    staged, _, _ = capture_paths
    value, clock, capacity = monitor(capture_paths, monkeypatch)
    value.check()
    write(staged/'records.jsonl', 10)
    capacity[0] -= 20
    clock[0] += 2
    peak = value.check()
    assert peak['peak_growth_bytes_per_s'] == 15
    assert peak['headroom_bytes'] == 2025 and peak['close_growth_budget_s'] == 135
    clock[0] += 5
    quiet = value.check()
    assert quiet['peak_growth_bytes_per_s'] == 15 and quiet['headroom_bytes'] == 2025


def test_actual_free_space_loss_stops_disk_capture(capture_paths, monkeypatch):
    value, clock, capacity = monitor(capture_paths, monkeypatch, memory=False, free=100_000)
    value.check()
    capacity[0] = 90_000
    clock[0] += 1
    report = failed_report(value)
    assert report['peak_growth_bytes_per_s'] == 10_000
    assert report['headroom_bytes'] == 1_350_000 and report['pending_bytes'] == 0


def test_closing_bypasses_throttle_and_uses_only_report_headroom(capture_paths, monkeypatch):
    staged, _, sources = capture_paths
    write(staged/'bag/db3', 100)
    write(sources/'right/epoch/journal', 200)
    value, clock, capacity = monitor(capture_paths, monkeypatch)
    value.check()
    clock[0] += 1
    write(staged/'bag/db3', 1_000_100)
    assert value.check()['headroom_bytes'] == 135_000_000
    # All producers stopped by the caller; no growth budget is needed.
    clock[0] += .01
    capacity[0] = 1_000_300 + storage.FINAL_REPORT_HEADROOM_BYTES + 200
    report = value.check(closing=True)
    assert report['closing'] and report['pending_bytes'] == 1_000_300
    assert report['headroom_bytes'] == storage.FINAL_REPORT_HEADROOM_BYTES
    assert report['remaining_after_bytes'] == 200 and report['margin_bytes'] == 0
    capacity[0] -= 1
    assert failed_report(value, closing=True)['margin_bytes'] == -1


@pytest.mark.parametrize('root', ['ram', 'left', 'right'])
def test_link_entries_are_rejected_without_scanning_the_target(capture_paths, monkeypatch, root):
    staged, _, sources = capture_paths
    base = staged if root == 'ram' else sources/root
    base.mkdir(parents=True, exist_ok=True)
    target = sources.parent/'foreign'
    write(target/'secret', 10)
    link = base/'linked'
    try:
        link.symlink_to(target, target_is_directory=True)
    except OSError:
        pytest.skip('creating symlinks requires OS permission')
    original = storage.os.scandir
    def no_target_scan(path):
        assert Path(path) != target and Path(path) != link
        return original(path)
    monkeypatch.setattr(storage.os, 'scandir', no_target_scan)
    value, _, _ = monitor(capture_paths, monkeypatch)
    with pytest.raises(RuntimeError, match='CAPTURE_STORAGE_MEASUREMENT_FAILED.*linked'):
        value.check()


def test_linked_source_ancestor_is_rejected_even_when_side_directories_missing(capture_paths, monkeypatch):
    staged, final, sources = capture_paths
    target = sources.parent/'foreign'
    target.mkdir(parents=True)
    try:
        sources.symlink_to(target, target_is_directory=True)
    except OSError:
        pytest.skip('creating symlinks requires OS permission')
    value, _, _ = monitor(capture_paths, monkeypatch)
    with pytest.raises(RuntimeError, match='CAPTURE_STORAGE_MEASUREMENT_FAILED.*linked'):
        value.check()


def test_entry_budget_is_shared_across_ram_and_both_lidar_trees(capture_paths, monkeypatch):
    staged, _, sources = capture_paths
    write(staged/'raw', 1)
    write(sources/'left/raw', 1)
    write(sources/'right/raw', 1)
    monkeypatch.setattr(storage, 'MAX_SCAN_ENTRIES', 2)
    value, _, _ = monitor(capture_paths, monkeypatch)
    with pytest.raises(RuntimeError, match='CAPTURE_STORAGE_MEASUREMENT_FAILED.*entry limit'):
        value.check()


def test_depth_budget_prevents_unbounded_walk(capture_paths, monkeypatch):
    staged, _, _ = capture_paths
    write(staged/'nested/raw', 1)
    monkeypatch.setattr(storage, 'MAX_SCAN_DEPTH', 1)
    value, _, _ = monitor(capture_paths, monkeypatch)
    with pytest.raises(RuntimeError, match='CAPTURE_STORAGE_MEASUREMENT_FAILED.*depth limit'):
        value.check()


def test_metadata_failure_is_not_silently_zero_pending_bytes(capture_paths, monkeypatch):
    value, _, _ = monitor(capture_paths, monkeypatch)
    def unavailable(path):
        raise PermissionError('simulated metadata access denied')
    monkeypatch.setattr(storage.os, 'scandir', unavailable)
    with pytest.raises(RuntimeError, match='CAPTURE_STORAGE_MEASUREMENT_FAILED.*PermissionError'):
        value.check()
    assert value.last_report['status'] == 'UNKNOWN'
    assert value.last_report['pending_bytes'] is None
    assert 'PermissionError' in value.last_report['error']


def test_future_final_directory_uses_existing_filesystem_parent(capture_paths, monkeypatch):
    staged, final, sources = capture_paths
    future = final/'not_created'
    calls = []
    monkeypatch.setattr(storage.shutil, 'disk_usage',
        lambda path: (calls.append(path) or SimpleNamespace(free=10**12)))
    value = storage.StorageMonitor(staged, future, sources, memory_staging=True)
    assert value.check()['status'] == 'AVAILABLE'
    assert calls == [final] and not future.exists()


def test_disappearing_atomic_entry_is_counted_as_a_race_not_read_as_content(capture_paths, monkeypatch):
    staged, _, _ = capture_paths
    write(staged/'stable', 20)
    class Vanished:
        name = '.ready.tmp'
        def is_symlink(self): return False
        def stat(self, **kwargs): raise FileNotFoundError('atomic rename')
    original = storage.os.scandir
    class Snapshot:
        def __enter__(self):
            with original(staged) as entries:
                return iter([*entries, Vanished()])
        def __exit__(self, *args): pass
    monkeypatch.setattr(storage.os, 'scandir', lambda path: Snapshot() if path == staged else original(path))
    value, _, _ = monitor(capture_paths, monkeypatch)
    report = value.check()
    assert report['vanished_entries'] == 1 and report['ram_bytes'] == 20


@pytest.mark.skipif(os.name != 'posix', reason='real AF_UNIX filesystem type requires target POSIX')
@pytest.mark.parametrize('memory', [True, False])
def test_live_manual_socket_has_no_archive_bytes_but_must_be_removed_before_closing(capture_paths, monkeypatch, memory):
    staged, _, _ = capture_paths
    write(staged/'raw', 20)
    endpoint = staged/'manual.sock'
    value, _, _ = monitor(capture_paths, monkeypatch, memory=memory)
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as owner:
        owner.bind(str(endpoint))
        report = value.check()
        assert report['ram_bytes'] == (20 if memory else 0)
        assert report['manual_socket_count'] == (1 if memory else 0)
        assert report['manual_socket_bytes'] == 0
        with pytest.raises(RuntimeError, match='CAPTURE_STORAGE_MEASUREMENT_FAILED.*manual socket remains'):
            value.check(closing=True)
        assert value.last_report['status'] == 'UNKNOWN'
    endpoint.unlink()
    assert value.check(closing=True)['status'] == 'AVAILABLE'


@pytest.mark.skipif(os.name != 'posix', reason='real AF_UNIX filesystem type requires target POSIX')
@pytest.mark.parametrize('location', ['other_name', 'nested', 'lidar'])
def test_no_other_socket_is_ignored_as_capture_data(capture_paths, monkeypatch, location):
    staged, _, sources = capture_paths
    endpoint = {'other_name': staged/'other.sock', 'nested': staged/'sub/manual.sock',
                'lidar': sources/'left/manual.sock'}[location]
    endpoint.parent.mkdir(parents=True, exist_ok=True)
    value, _, _ = monitor(capture_paths, monkeypatch)
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as owner:
        owner.bind(str(endpoint))
        with pytest.raises(RuntimeError, match='CAPTURE_STORAGE_MEASUREMENT_FAILED.*nonregular'):
            value.check()


def test_manual_capture_uses_explicit_longer_shutdown_growth_budget(capture_paths, monkeypatch):
    staged, _, _ = capture_paths
    value, clock, _ = monitor(capture_paths, monkeypatch, growth_budget=230)
    value.check()
    write(staged/'records.jsonl', 20)
    clock[0] += 2
    report = value.check()
    assert report['peak_growth_bytes_per_s'] == 10
    assert report['headroom_bytes'] == 2300 and report['close_growth_budget_s'] == 230


@pytest.mark.parametrize('invalid', [0, -1, float('inf'), float('nan'), True, '230'])
def test_shutdown_growth_budget_rejects_nonfinite_or_nonpositive_values(capture_paths, invalid):
    with pytest.raises(ValueError, match='close_growth_seconds must be finite and positive'):
        storage.StorageMonitor(*capture_paths, memory_staging=True, close_growth_seconds=invalid)
