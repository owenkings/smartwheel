"""0.10 m/s test command source and stop report, against the dry-run owner (simulated driver, real time)."""
import datetime
from pathlib import Path
import tempfile
import threading
import time

import pytest

from wc_motion import auto_stop_report as r
from wc_motion import auto_test_source as t
from wc_motion.auto_bridge import BridgeCore, BridgeError
from wc_runtime import auto_dry_run as d

ROOT = Path(__file__).resolve().parents[2]


class Owner:
    """The dry-run owner in a thread; ``hold`` keeps the loop going after the source is gone."""
    def __init__(self, tmp):
        self.run = d.DryRun(ROOT, Path(tmp)/'session', 'dryrun_source', start='once')
        self.stop, self.errors = threading.Event(), []
        self.thread = threading.Thread(target=self.loop)
        self.thread.start()

    def loop(self):
        try:
            while not self.stop.is_set():
                self.run.tick()
                time.sleep(0.005)
        except Exception as error:  # noqa: BLE001 - the owner's own fault path, inspected by the tests
            self.errors.append(error)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=5)
        self.run.close()


def trial(tmp, stop, *, during=None, seconds=12.0, **options):
    owner = Owner(tmp)
    core = BridgeCore('dryrun_source', 'test_source', require_dry_run=True)
    source = t.TestSource(core, stop=stop, dry_run=True, out=lambda text: None, **options)
    failure = None
    try:
        core.connect(owner.run.socket_path)
        started = time.monotonic()
        while time.monotonic() - started < seconds:
            if during is not None and source.phase == 'FORWARD' and time.monotonic() - source.since > 1.5:
                if during(owner, core) == 'gone':
                    break
                during = None
            if not source.step():
                break
            time.sleep(t.PERIOD_S)
    except BridgeError as error:
        failure = str(error)
    finally:
        core.close()
        time.sleep(1.5)   # the owner brakes after the source is gone and the journal batch is written
        owner.close()
    return source, failure, r.report(owner.run.session_root)['trials'], owner


def test_software_zero_trial_reports_speed_conversion_stop_time_and_distance():
    with tempfile.TemporaryDirectory(prefix='wc-src-', dir='/tmp') as tmp:
        source, failure, trials, owner = trial(tmp, 'software_zero', forward_s=2.0)
        assert failure is None and source.phase == 'DONE' and not owner.errors
        assert [e['phase'] for e in source.events] == ['FORWARD', 'ZERO', 'DONE']
        row = trials[0]
        assert row['kind'] == 'software_zero' and row['commanded_m_s'] == pytest.approx(0.10, abs=0.002)
        assert 0.95 < row['speed_ratio'] < 1.02
        assert 0 < row['stop_time_s'] < 1.0 and 0 < row['stop_distance_m'] < 0.05
        assert abs(row['heading_change_deg']) < 0.5
        assert any('software_zero' in line for line in r.lines({'trials': trials}))


def test_emergency_stop_on_the_driver_is_reported_from_the_slowing_wheels():
    def press(owner, core):
        owner.run.driver.estop = True
    with tempfile.TemporaryDirectory(prefix='wc-src-', dir='/tmp') as tmp:
        source, failure, trials, owner = trial(tmp, 'external', during=press)
        assert failure is None and [e['phase'] for e in source.events][:2] == ['FORWARD', 'OWNER_STOPPED']
        row = trials[0]
        assert row['kind'] == 'estop' and row['owner_exit_reason'] == 'WHEELS_STOPPED_WHILE_COMMANDED'
        assert 0 < row['stop_time_s'] < 1.5 and 0 < row['stop_distance_m'] < 0.1


def test_killed_source_and_pulled_usb_adapter():
    def kill(owner, core):
        core.sock.close()   # what kill -9 does to the socket
        core.sock = None
        return 'gone'
    with tempfile.TemporaryDirectory(prefix='wc-src-', dir='/tmp') as tmp:
        _, _, trials, owner = trial(tmp, 'external', during=kill)
        assert trials[0]['kind'] == 'source_lost' and 0 < trials[0]['stop_time_s'] < 1.0

    def unplug(owner, core):
        owner.run.far.close()   # the driver end of the line disappears
    with tempfile.TemporaryDirectory(prefix='wc-src-', dir='/tmp') as tmp:
        _, failure, trials, owner = trial(tmp, 'external', during=unplug, seconds=4.0)
        assert owner.errors and trials[0]['kind'] == 'feedback_lost'
        assert trials[0]['stop'].startswith('UNKNOWN') and trials[0]['measured_m_s'] > 0.09


def test_cli_needs_todays_approval_and_one_running_session(tmp_path):
    with pytest.raises(SystemExit, match='APPROVED_BY_USER_20261111'):
        t.main(['--project-root', str(ROOT), '--stop', 'software_zero'], environ={},
               today=datetime.date(2026, 11, 11))
    with pytest.raises(SystemExit, match='--forward-s'):
        t.main(['--project-root', str(ROOT), '--stop', 'software_zero', '--forward-s', '30', '--dry-run'])
    with pytest.raises(SystemExit, match='together'):
        t.main(['--project-root', str(ROOT), '--stop', 'external', '--session-root', str(tmp_path), '--dry-run'])


def test_stop_report_finds_the_newest_recording_of_the_checkout(tmp_path, monkeypatch, capsys):
    import json
    import os
    from wc_runtime import mapping_auto, project_paths
    monkeypatch.setattr(project_paths, 'capture_staging_root', lambda: tmp_path)
    for index, (name, root, status) in enumerate((('old', ROOT, 'COMPLETE'), ('new', ROOT, 'RECORDING'),
                                                  ('other', tmp_path, 'RECORDING'))):
        manifest = tmp_path/name/'capture_manifest.json'
        manifest.parent.mkdir()
        manifest.write_text(json.dumps({'session_id': name, 'status': status, 'project_root': str(root)}))
        os.utime(manifest, ns=(10**18 + index, 10**18 + index))
    (tmp_path/'broken').mkdir()
    (tmp_path/'broken/capture_manifest.json').write_text('{')
    assert [s[1:] for s in mapping_auto.recording_sessions(ROOT)] == [('new', 'RECORDING'), ('old', 'COMPLETE')]
    assert mapping_auto.running_auto_sessions(ROOT) == []   # no auto.sock in either
    with pytest.raises(FileNotFoundError, match='no wheel_feedback.jsonl'):
        r.main(['--project-root', str(ROOT)])
    assert str(tmp_path/'new') in capsys.readouterr().out
    with pytest.raises(SystemExit, match='--project-root'):
        r.main([])


def test_vehicle_source_refuses_outside_the_stand_and_ground_stages():
    class Core:
        status = {'evidence_stage': 'SUPERVISED_CLOSED_LOOP', 'auto_state': 'NONE'}
        def pump(self): pass
        def fresh(self): return True
        def clock_ns(self): return time.monotonic_ns()
        def on_gate_command(self, text): pass
    source = t.TestSource(Core(), stop='software_zero', out=lambda text: None)
    with pytest.raises(BridgeError, match='STAND_TEST or GROUND_TEST'):
        source.step()
    Core.status = {'evidence_stage': 'STAND_TEST', 'auto_state': 'NONE'}
    assert source.step() and source.phase == 'WAIT_AUTO'
