"""Bounded disposable status files; never an acceptance or recovery authority."""
from collections import OrderedDict
import os
from pathlib import Path
import threading
import uuid


def replace_bytes(path, payload):
    temporary = path.with_name(path.name + '.cache-' + uuid.uuid4().hex)
    fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    try:
        view = memoryview(payload)
        while view:
            count = os.write(fd, view)
            if count <= 0:
                raise OSError('short cache write')
            view = view[count:]
        os.close(fd)
        fd = -1
        os.replace(temporary, path)
    finally:
        if fd >= 0:
            os.close(fd)
        temporary.unlink(missing_ok=True)


class LatestCacheWriter:
    """One writer and at most one pending immutable payload per allowed path.

    Linux may throttle even non-fsync writes. No filesystem operation therefore
    runs in submit(), which is called by the sensor/ICP executor. Cache errors
    remain observable, but cannot authorize or reject mapping data.
    """
    def __init__(self, paths, *, write=replace_bytes, max_bytes=262144):
        self.paths = frozenset(Path(p) for p in paths)
        if not 1 <= len(self.paths) <= 2:
            raise ValueError('one or two explicit cache paths required')
        self.max_bytes, self._write = max_bytes, write
        self._condition = threading.Condition()
        self._pending = OrderedDict()
        self._closing = False
        self._error = None
        self._thread = threading.Thread(target=self._run, name='wc-disposable-cache', daemon=True)
        self._thread.start()

    @property
    def error(self):
        with self._condition:
            return self._error

    @property
    def pending_count(self):
        with self._condition:
            return len(self._pending)

    def submit(self, path, payload):
        path = Path(path)
        if path not in self.paths or not isinstance(payload, bytes) or len(payload) > self.max_bytes:
            raise ValueError('cache requires an allowed path and bounded immutable bytes')
        with self._condition:
            if self._closing:
                raise RuntimeError('cache writer is closing')
            self._pending[path] = payload
            self._condition.notify()

    def _run(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._pending or self._closing)
                if not self._pending:
                    return
                path, payload = self._pending.popitem(last=False)
            try:
                self._write(path, payload)
            except Exception as error:
                with self._condition:
                    self._error = f'{type(error).__name__}: {error}'[:4096]

    def close(self, timeout=2.0):
        with self._condition:
            self._closing = True
            self._condition.notify()
        self._thread.join(timeout=timeout)
        if self._thread.is_alive():
            raise TimeoutError('disposable cache write still blocked; cache may be stale')
