"""A blocked disposable cache must not block sensor submission or grow memory."""
from pathlib import Path
import threading
import pytest

from wc_fusion.cache import LatestCacheWriter, replace_bytes


def test_blocked_writer_coalesces_two_paths_and_drains_latest(tmp_path):
    entered, release = threading.Event(), threading.Event()
    writes = []
    first, second = tmp_path/'frontend.json', tmp_path/'tracking.json'
    def held(path, payload):
        if not writes:
            entered.set()
            assert release.wait(3)
        writes.append((path, payload))
    writer = LatestCacheWriter([first, second], write=held)
    try:
        writer.submit(first, b'initial')
        assert entered.wait(2)
        for i in range(10000):
            writer.submit(first, str(i).encode())
            writer.submit(second, str(i).encode())
        assert writer.pending_count == 2
        with pytest.raises(TimeoutError, match='cache may be stale'):
            writer.close(timeout=0.01)
        assert writer.error is None
    finally:
        release.set()
        writer.close(timeout=3)
    assert writes == [(first,b'initial'),(first,b'9999'),(second,b'9999')]


def test_cache_error_is_observable_and_later_payload_can_replace(tmp_path):
    entered, release = threading.Event(), threading.Event()
    calls = []
    path = tmp_path/'status.json'
    def fail_once(path, data):
        calls.append(data)
        if len(calls) == 1:
            entered.set()
            assert release.wait(3)
            raise OSError('storage unavailable')
        replace_bytes(path, data)
    writer = LatestCacheWriter([path], write=fail_once)
    writer.submit(path, b'old')
    assert entered.wait(2)
    writer.submit(path, b'latest')
    release.set()
    writer.close(timeout=3)
    assert 'storage unavailable' in writer.error
    assert path.read_bytes() == b'latest'
    assert list(tmp_path.iterdir()) == [path]


def test_cache_rejects_unbounded_mutable_or_unregistered_payload(tmp_path):
    path = tmp_path/'status.json'
    writer = LatestCacheWriter([path], max_bytes=3)
    try:
        for location, payload in ((path,b'long'), (path,bytearray(b'x')), (tmp_path/'other',b'x')):
            with pytest.raises(ValueError): writer.submit(location, payload)
    finally:
        writer.close()
    with pytest.raises(RuntimeError): writer.submit(path,b'x')


def test_replace_does_not_follow_existing_target_symlink(tmp_path):
    original, link = tmp_path/'original', tmp_path/'status.json'
    original.write_bytes(b'preserved')
    link.symlink_to(original)
    replace_bytes(link, b'cache')
    assert original.read_bytes() == b'preserved'
    assert link.read_bytes() == b'cache' and not link.is_symlink()
