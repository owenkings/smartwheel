"""Dry-run owner for window D: simulated driver frames, and the real owner loop end to end over auto.sock."""
import json
from pathlib import Path
import socket
import struct
import tempfile
import threading
import time

import pytest

from wc_motion.auto_bridge import BridgeCore
from wc_motion.protocol import crc16
from wc_runtime import auto_dry_run as d
from wc_runtime import mapping_wheel as m

ROOT = Path(__file__).resolve().parents[2]


class Clock:
    def __init__(self): self.value = 100.0
    def __call__(self): return self.value


def feedback(driver):
    reply = driver.answer(m.QUERY)
    return struct.unpack('>hh', reply[3:7])


def test_simulated_driver_follows_enabled_speed_writes_and_coasts_after_release():
    clock = Clock()
    driver = d.SimulatedDriver(None, 0.5, tau_s=0.1, clock=clock)
    assert feedback(driver) == (0, 0)
    assert driver.answer(m.speed_write(-200, 200)) == m.speed_write(-200, 200)[:6] + crc16(m.speed_write(-200, 200)[:6])
    clock.value += 1.0
    assert feedback(driver) == (0, 0)                       # not enabled: no motion
    for request in m.INITIALIZATION_WRITES:
        assert driver.answer(request) == request          # FC06 is echoed
    clock.value += 0.1
    left, right = feedback(driver)
    assert -70 < left < -55 and 55 < right < 70           # one time constant: about 63 % of 100
    clock.value += 1.0
    assert feedback(driver) == (-100, 100)
    driver.answer(m.RELEASE_REQUEST)
    clock.value += 1.0
    assert feedback(driver) == (0, 0) and not driver.enabled
    assert driver.answer(m.read_request(1, 0x2000, 1)) is None      # not the feedback register: no answer
    driver.buffer.extend(m.QUERY[:-1] + b'\x00' + m.QUERY)          # a damaged frame clears the buffer
    assert driver._frames() == []


def gate(seq, linear):
    return json.dumps({'schema': 'wc_gate_command_v1', 'gate_id': 'gate_dry', 'seq': seq, 'stamp_ns': time.time_ns(),
                       'mono_ns': time.monotonic_ns(), 'state': 'ACTIVE', 'reasons': [], 'linear': linear,
                       'angular': 0.0, 'config_sha256': None,
                       'limits': {'max_linear': 0.10, 'max_angular': 0.30, 'linear_ceiling': 0.10,
                                  'allow_reverse': False}})


def test_dry_run_owner_drives_the_simulated_driver_through_auto_sock_and_stops_on_stale_gate():
    with tempfile.TemporaryDirectory(prefix='wc-dry-', dir='/tmp') as tmp:
        run = d.DryRun(ROOT, Path(tmp)/'session', 'dryrun_test', start='once')
        stop, errors = threading.Event(), []

        def loop():
            try:
                while not stop.is_set():
                    run.tick()
                    time.sleep(0.005)
            except Exception as error:  # noqa: BLE001 - reported by the assertion below
                errors.append(error)
        thread = threading.Thread(target=loop)
        thread.start()
        bridge = BridgeCore('dryrun_test', 'bridge_dry', require_dry_run=True)
        seen = []
        try:
            bridge.connect(run.socket_path)
            started, seq = time.monotonic(), 0
            while time.monotonic() - started < 4.5:
                bridge.pump()
                if time.monotonic() - started < 3.0:
                    seq += 1
                    bridge.on_gate_command(gate(seq, 0.10))
                if bridge.status:
                    seen.append((bridge.status['auto_state'], (bridge.status['feedback'] or {}).get('linear_m_s')))
                time.sleep(0.05)
        finally:
            bridge.close()
            stop.set()
            thread.join(timeout=5)
            status = run.auto.status()
            run.close()
        assert not errors and bridge.dry_run_confirmed and status['dry_run'] is True
        moving = [speed for state, speed in seen if state == 'AUTO' and speed is not None and speed > 0.09]
        assert moving and max(moving) < 0.105, seen
        assert status['auto_state'] == 'NONE' and 'DECISION_STALE' in status['reason']
        records = [json.loads(line) for line in
                   (Path(tmp)/'session/sources/wheel_feedback.jsonl').read_text().splitlines()]
        writes = [r for r in records if r.get('event') == 'wheel_io_complete' and r['request_hex'][2:4] == '10']
        assert writes and records[-1]['event'] == 'journal_finalization'
        assert {r['state'] for r in records if r.get('event') == 'auto_state'} >= {'INITIALIZING', 'AUTO', 'NONE'}


def test_dry_run_refuses_a_used_session_root(tmp_path):
    (tmp_path/'old').write_text('x')
    with pytest.raises(SystemExit, match='new or empty'):
        d.main(['--project-root', str(ROOT), '--session-root', str(tmp_path)])
