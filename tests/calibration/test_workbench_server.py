"""Browser loads multiple static assets concurrently on a fresh page or reload."""
import concurrent.futures
import http.client
import threading
from types import SimpleNamespace

from wc_calibration import picker


def test_all_five_workbench_assets_survive_simultaneous_browser_requests(tmp_path, monkeypatch):
    assets = ['/workbench.css', '/amplitude.js', '/picker.js', '/alignment.js', '/workbench.js']
    for route in assets:
        (tmp_path/route[1:]).write_text('asset '+route)
    monkeypatch.setattr(picker, 'ASSET_ROOT', tmp_path)
    barrier = threading.Barrier(len(assets))
    original = picker.PickerHandler._reply
    def synchronized_reply(self, status, content, content_type='application/json; charset=utf-8'):
        barrier.wait(timeout=2)
        return original(self, status, content, content_type)
    monkeypatch.setattr(picker.PickerHandler, '_reply', synchronized_reply)
    server = picker.PickerServer(SimpleNamespace(token='synthetic-only'), 0)
    worker = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .01}, daemon=True)
    worker.start()
    def get(route):
        connection = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=4)
        try:
            connection.request('GET', route)
            response = connection.getresponse()
            return response.status, response.read().decode()
        finally:
            connection.close()
    try:
        with concurrent.futures.ThreadPoolExecutor(max_workers=len(assets)) as pool:
            responses = list(pool.map(get, assets))
        assert responses == [(200, 'asset '+route) for route in assets]
    finally:
        server.shutdown(); server.server_close(); worker.join(2)
