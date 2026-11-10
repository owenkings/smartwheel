"""Command-line "start / stop automatic" over operator.sock, against the dry-run owner (simulated driver, real time)."""
from pathlib import Path
import tempfile
import threading
import time

import pytest

from wc_motion import auto_test_source as t
from wc_motion.auto_bridge import BridgeCore
from wc_runtime import auto_dry_run as d
from wc_runtime import auto_operator as o

ROOT = Path(__file__).resolve().parents[2]


def test_start_status_and_stop_from_the_command_line(capsys):
    with tempfile.TemporaryDirectory(prefix='wc-op-', dir='/tmp') as tmp:
        run = d.DryRun(ROOT, Path(tmp)/'session', 'dryrun_operator', start='never')
        stop, errors = threading.Event(), []

        def owner():
            try:
                while not stop.is_set():
                    run.tick()
                    time.sleep(0.005)
            except Exception as error:  # noqa: BLE001 - inspected below
                errors.append(error)
        core = BridgeCore('dryrun_operator', 'test_source', require_dry_run=True)
        source = t.TestSource(core, stop='external', max_s=8.0, dry_run=True, out=lambda text: None)

        def drive():
            while source.step():
                time.sleep(t.PERIOD_S)
        threads = [threading.Thread(target=owner), threading.Thread(target=drive)]
        args = ['--project-root', str(ROOT), '--session-root', str(Path(tmp)/'session'), '--session-id',
                'dryrun_operator']
        try:
            threads[0].start()
            core.connect(run.socket_path)
            threads[1].start()
            time.sleep(1.0)                                   # the wheels stand still, decisions are fresh
            assert o.main(['status', *args]) == 0
            assert 'auto NONE' in capsys.readouterr().out
            assert o.main(['start', *args]) == 0              # the dry run's interface always has the focus
            out = capsys.readouterr().out
            assert 'auto AUTO (AUTO_STARTED)' in out and 'bridge or test source connected' in out
            time.sleep(1.0)
            assert o.main(['stop', *args]) == 0
            assert 'USER_STOPPED_AUTO' in capsys.readouterr().out
            threads[1].join(timeout=10)
            assert [e['phase'] for e in source.events] == ['FORWARD', 'OWNER_STOPPED', 'DONE']
            core.close()                                      # the test source has gone
            time.sleep(0.3)
            assert o.main(['start', *args]) == 1              # refused at once
            assert 'refused: AUTO_REFUSED: BRIDGE_DISCONNECTED' in capsys.readouterr().out
        finally:
            core.close()
            stop.set()
            for thread in threads:
                thread.join(timeout=10)
            run.close()
        assert not errors


def test_command_line_needs_one_running_session(tmp_path, monkeypatch):
    from wc_runtime import project_paths
    monkeypatch.setattr(project_paths, 'capture_staging_root', lambda: tmp_path)
    with pytest.raises(SystemExit, match='expected one running recording'):
        o.main(['status', '--project-root', str(ROOT)])
    with pytest.raises(SystemExit, match='together'):
        o.main(['status', '--project-root', str(ROOT), '--session-id', 'x'])
    with pytest.raises(SystemExit, match='no answer from the wheel owner'):
        o.main(['stop', '--project-root', str(ROOT), '--session-root', str(tmp_path), '--session-id', 'nobody'])
