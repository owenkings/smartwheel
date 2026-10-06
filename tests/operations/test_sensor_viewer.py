"""No-device tests of viewer dispatch, startup failures and owned cleanup."""
import io
import json
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from contextlib import ExitStack, redirect_stdout

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT/'src'))
from wc_runtime import sensor_viewer as viewer


class Clock:
    def __init__(self):
        self.value = 0
    def now(self):
        return self.value
    def sleep(self, value):
        self.value += value


class FakeWindow:
    def __init__(self, runtime, side):
        self.runtime, self.side = runtime, side
        self.exit_code = 0 if side == runtime.close_side else None
        self.stop_called = self.finished = False
    def start(self):
        self.runtime.events.append(('window_start', self.side))
        if self.side == self.runtime.fail_window:
            raise OSError('synthetic failure after child creation')
    def metadata(self):
        return {'side': self.side, 'pid': 1001 if self.side == 'left' else 1002,
                'start_ticks': 'fixture', 'pidfd_acquired': True}
    def poll(self):
        return self.exit_code
    def request_stop(self):
        self.stop_called = True
        self.runtime.events.append(('window_stop', self.side))
        self.exit_code = 0
    def finish(self):
        self.finished = True
        return {'side': self.side, 'status': 'PASS', 'exit_code': self.exit_code, 'alive_after_cleanup': False}


class FakeRuntime:
    def __init__(self, root):
        self.root = root
        (root/'config/rviz').mkdir(parents=True)
        for side in ('left', 'right'):
            (root/'config/rviz'/('sensor_'+side+'.rviz')).write_text('SYNTHETIC_NO_ROS', encoding='utf-8')
        self.events = []
        self.windows = []
        self.close_side = self.fail_window = None
        self.gui_failure = False
        self.start_code = 0
        self.started = False
        self.value = None
        self.alive_checks = 0
        self.driver_dies = False
        self.late_manifest_reads = 0
        self.stop_failure = False
    def gui(self):
        self.events.append(('gui',))
        if self.gui_failure:
            raise RuntimeError('no GUI')
        return {'DISPLAY': ':fixture', 'XAUTHORITY': '/fixture/no-cookie'}
    def start_drivers(self, session, mode, duration, report_dir):
        self.started = True
        self.events.append(('drivers', session, mode, duration))
        self.value = {'session_id': session, 'role': 'drivers', 'state': 'RUNNING'}
        return self.start_code
    def manifest_path(self, session):
        return self.root/'.phase1_runtime/sessions'/session/'drivers/manifest.json'
    def manifest(self, session):
        if self.late_manifest_reads:
            self.late_manifest_reads -= 1
            return None
        return self.value
    def driver_alive(self, manifest):
        self.alive_checks += 1
        return not self.driver_dies or self.alive_checks == 1
    def stop_drivers(self, session, report_dir):
        self.events.append(('stop', session))
        self.value = {**self.value, 'state': 'FAILED' if self.stop_failure else 'STOPPED',
                      'exit_code': 1 if self.stop_failure else 0}
        return 1 if self.stop_failure else 0
    def new_window(self, side, session, env, report_dir):
        window = FakeWindow(self, side)
        self.windows.append(window)
        return window


class ViewerLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='viewer-fixture-', dir=PROJECT/'tests/operations')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.runtime = FakeRuntime(self.root)
        self.clock = Clock()
    def run_view(self, mode='dual', stop=lambda: False):
        with redirect_stdout(io.StringIO()):
            code = viewer.run_view(self.runtime, mode, 1, stop, clock=self.clock.now, sleep=self.clock.sleep)
        paths = list((self.root/'reports/sensor_views').glob('*/result.json'))
        self.assertEqual(len(paths), 1)
        return code, json.loads(paths[0].read_text(encoding='utf-8'))
    def test_gui_failure_never_calls_drivers_or_opens_rviz(self):
        self.runtime.gui_failure = True
        code, report = self.run_view()
        self.assertEqual(code, 1)
        self.assertEqual(self.runtime.events, [('gui',)])
        self.assertFalse(report['driver_start_attempted'])
        self.assertEqual(report['driver_cleanup']['status'], 'NOT_STARTED')
    def test_dual_opens_two_distinct_views_and_stops_only_its_unique_session(self):
        code, report = self.run_view()
        self.assertEqual(code, 0)
        self.assertEqual([window.side for window in self.runtime.windows], ['left', 'right'])
        session = report['session_id']
        self.assertEqual([row for row in self.runtime.events if row[0] in ('drivers', 'stop')],
                         [('drivers', session, 'dual', 1), ('stop', session)])
        self.assertEqual(report['stop_reason'], 'DURATION_REACHED')
        self.assertEqual(len(report['rviz_processes']), 2)
        self.assertFalse(report['recording_started'])
        self.assertFalse(report['imu_encoder_or_motor_started'])
    def test_single_right_opens_only_right(self):
        code, report = self.run_view('single_right')
        self.assertEqual(code, 0)
        self.assertEqual([window.side for window in self.runtime.windows], ['right'])
        self.assertEqual(report['rviz_views'], ['right'])
    def test_second_window_partial_start_failure_cleans_both_owned_children(self):
        self.runtime.fail_window = 'right'
        code, report = self.run_view()
        self.assertEqual(code, 1)
        self.assertTrue(all(w.stop_called and w.finished for w in self.runtime.windows))
        self.assertEqual(report['cleanup_status'], 'PASS')
        self.assertEqual(len([e for e in self.runtime.events if e[0] == 'stop']), 1)
    def test_uncertain_driver_start_waits_for_own_late_manifest_and_stops_it(self):
        self.runtime.start_code = 2
        self.runtime.late_manifest_reads = 2
        code, report = self.run_view()
        self.assertEqual(code, 1)
        self.assertFalse(self.runtime.windows)
        self.assertEqual(report['driver_cleanup']['status'], 'PASS')
        self.assertEqual([e for e in self.runtime.events if e[0] == 'stop'], [('stop', report['session_id'])])
        self.assertGreater(self.clock.value, 0)
    def test_missing_manifest_does_not_guess_another_session_or_report_cleanup_pass(self):
        self.runtime.start_code = 2
        self.runtime.late_manifest_reads = 1000
        code, report = self.run_view()
        self.assertEqual(code, 1)
        self.assertEqual(report['driver_cleanup']['status'], 'UNKNOWN')
        self.assertEqual(report['cleanup_status'], 'FAIL')
        self.assertFalse(any(e[0] == 'stop' for e in self.runtime.events))
        self.assertGreaterEqual(self.clock.value, 8)
        self.assertLess(self.clock.value, 9)
    def test_closing_either_window_stops_own_driver_and_other_window(self):
        self.runtime.close_side = 'right'
        code, report = self.run_view()
        self.assertEqual(code, 0)
        self.assertEqual(report['stop_reason'], 'RVIZ_WINDOW_EXIT')
        self.assertTrue(all(w.stop_called and w.finished for w in self.runtime.windows))
        events = self.runtime.events
        stop_index = next(i for i, event in enumerate(events) if event[0] == 'stop')
        self.assertTrue(all(events.index(('window_stop', side)) < stop_index for side in ('left', 'right')))
    def test_early_driver_exit_closes_windows_and_is_not_a_successful_duration(self):
        self.runtime.driver_dies = True
        code, report = self.run_view()
        self.assertEqual(code, 1)
        self.assertTrue(all(w.finished for w in self.runtime.windows))
        self.assertEqual(report['cleanup_status'], 'PASS')
        self.assertIn('driver exited', report['errors'][0])
    def test_signal_during_driver_start_prevents_window_start_but_stops_driver(self):
        code, report = self.run_view(stop=lambda: self.runtime.started)
        self.assertEqual(code, 0)
        self.assertFalse(self.runtime.windows)
        self.assertTrue(report['driver_cleanup']['stop_called'])
        self.assertEqual(report['stop_reason'], 'USER_SIGNAL')
    def test_failed_driver_cleanup_is_reported_even_after_clean_window_exit(self):
        self.runtime.stop_failure = True
        code, report = self.run_view()
        self.assertEqual(code, 1)
        self.assertEqual(report['cleanup_status'], 'FAIL')
        self.assertEqual(report['driver_cleanup']['manifest']['exit_code'], 1)
    def test_report_write_failure_during_viewing_cannot_skip_driver_cleanup(self):
        original = viewer.write_report
        def write(path, report):
            if report['state'] in ('VIEWING', 'STOPPING'):
                raise OSError('synthetic full report device')
            return original(path, report)
        with patch.object(viewer, 'write_report', side_effect=write):
            code, report = self.run_view()
        self.assertEqual(code, 1)
        self.assertEqual(report['cleanup_status'], 'PASS')
        self.assertTrue(report['driver_cleanup']['stop_called'])
        self.assertTrue(all(w.finished for w in self.runtime.windows))
    def test_invalid_input_cannot_cross_gui_or_driver_boundary(self):
        for mode, duration in (('triple', 1), ('dual', 0), ('dual', 3601), ('dual', True)):
            with self.assertRaises(ValueError):
                viewer.run_view(self.runtime, mode, duration, lambda: False)
        self.assertEqual(self.runtime.events, [])


class ViewerBoundariesTests(unittest.TestCase):
    def test_dispatch_uses_only_existing_drivers_stop_and_correct_rviz_configs(self):
        with tempfile.TemporaryDirectory(prefix='viewer-dispatch-', dir=PROJECT/'tests/operations') as temporary:
            runtime = viewer.Runtime.__new__(viewer.Runtime)
            runtime.root = Path(temporary)
            calls = []
            runtime.cli = SimpleNamespace(main=lambda args: calls.append(args) or 0, ros_command=lambda args: list(map(str, args)))
            self.assertEqual(runtime.start_drivers('owned', 'single_left', 900, runtime.root), 0)
            self.assertEqual(runtime.stop_drivers('owned', runtime.root), 0)
            self.assertEqual(calls, [['drivers', '--session', 'owned', '--mode', 'single_left', '--duration', '900'],
                                     ['stop', '--session', 'owned']])
            window = runtime.new_window('right', 'owned', {}, runtime.root)
            self.assertEqual(window.command[:3], ['rviz2', '-d', str(runtime.root/'config/rviz/sensor_right.rviz')])
            self.assertIn('__node:=wc_lidar_view_owned_right', window.command)
    def test_graphical_environment_requires_same_uid_and_real_session_values(self):
        with tempfile.TemporaryDirectory(prefix='viewer-gui-', dir=PROJECT/'tests/operations') as temporary:
            folder = Path(temporary)
            proc = folder/'proc/123'
            proc.mkdir(parents=True)
            authority = folder/'synthetic-authority'
            authority.write_text('NOT_A_REAL_COOKIE', encoding='utf-8')
            (proc/'comm').write_text('gnome-shell\n', encoding='utf-8')
            (proc/'environ').write_bytes(('DISPLAY=:1\0XAUTHORITY='+str(authority)+'\0').encode())
            uid = (proc/'comm').stat().st_uid
            env = viewer.desktop_environment(environ={'DISPLAY': 'localhost:10.0'}, proc_root=folder/'proc', uid=uid)
            self.assertEqual(env['DISPLAY'], ':1')
            self.assertEqual(env['XAUTHORITY'], str(authority))
            with self.assertRaises(RuntimeError):
                viewer.desktop_environment(environ={}, proc_root=folder/'proc', uid=uid+1)
    def test_half_started_rviz_is_owned_and_cleaned_when_pidfd_acquisition_fails(self):
        with tempfile.TemporaryDirectory(prefix='viewer-pidfd-', dir=PROJECT/'tests/operations') as temporary:
            root = Path(temporary)
            child = Mock(pid=123)
            state = {'code': None}
            child.poll.side_effect = lambda: state['code']
            child.wait.side_effect = lambda timeout: state.update(code=0) or 0
            with patch.object(viewer.subprocess, 'Popen', return_value=child) as popen, \
                 patch.object(viewer, 'process_identity', return_value={'start_ticks': '42'}), \
                 patch.object(viewer.os, 'pidfd_open', side_effect=OSError('synthetic pidfd failure'), create=True), \
                 patch.object(viewer.signal, 'SIGKILL', 9, create=True):
                window = viewer.OwnedRviz('left', ['rviz2'], {}, root, root/'rviz.log')
                with self.assertRaises(OSError):
                    window.start()
                self.assertEqual(window.metadata()['pid'], 123)
                self.assertEqual(window.metadata()['start_ticks'], '42')
                self.assertTrue(popen.call_args.kwargs['start_new_session'])
                window.request_stop()
                report = window.finish()
            child.send_signal.assert_called_once_with(signal.SIGINT)
            self.assertEqual(report['status'], 'PASS')
            self.assertFalse(report['alive_after_cleanup'])
    def test_terminal_hangup_requests_same_owned_cleanup_path_and_restores_handlers(self):
        installed = {}
        restored = []
        def set_signal(signum, handler):
            if callable(handler):
                installed[signum] = handler
            else:
                restored.append(signum)
        def run(runtime, mode, duration, stopped):
            self.assertFalse(stopped())
            installed[viewer.signal.SIGHUP](viewer.signal.SIGHUP, None)
            self.assertTrue(stopped())
            return 0
        with patch.object(viewer, 'Runtime', return_value=object()), \
             patch.object(viewer, 'run_view', side_effect=run), \
             patch.object(viewer.signal, 'SIGHUP', 1, create=True), \
             patch.object(viewer.signal, 'getsignal', return_value=signal.SIG_DFL), \
             patch.object(viewer.signal, 'signal', side_effect=set_signal):
            self.assertEqual(viewer.main(['dual', '30']), 0)
        self.assertEqual(set(installed), set(restored))
        self.assertIn(1, installed)
    def test_driver_pid_reuse_or_zombie_is_not_running(self):
        runtime = viewer.Runtime.__new__(viewer.Runtime)
        manifest = {'supervisor_pid': 123, 'supervisor_start_ticks': 'a',
                    'children': [{'pid': 124, 'start_ticks': 'b'}]}
        for replacement in ({'state': 'Z', 'start_ticks': 'a'}, {'state': 'S', 'start_ticks': 'reused'}):
            with patch.object(viewer, 'process_identity', return_value=replacement):
                self.assertFalse(runtime.driver_alive(manifest))


class OwnedRvizStopTests(unittest.TestCase):
    """Synthetic Popen/pidfd only; no process, display, log or device is opened."""

    def cleanup(self, code=-signal.SIGINT, *, already_exited=False, pidfd=False,
                signal_error=None, wait_timeouts=0, never_exits=False, prior_errors=()):
        window = viewer.OwnedRviz('right', ['synthetic-rviz-no-execution'], {},
                                 PROJECT, PROJECT/'synthetic-rviz-unused.log')
        child = Mock(pid=123)
        state = {'code': code if already_exited else None, 'send_attempts': [], 'waits': 0}
        child.poll.side_effect = lambda: state['code']

        def send(signum):
            state['send_attempts'].append(signum)
            if signal_error is not None:
                raise signal_error

        def wait(timeout):
            state['waits'] += 1
            if never_exits or state['waits'] <= wait_timeouts:
                raise subprocess.TimeoutExpired('synthetic-rviz', timeout)
            state['code'] = code
            return code

        child.send_signal.side_effect = send
        child.wait.side_effect = wait
        window.child, window.start_ticks = child, '42'
        window.pidfd = 71 if pidfd else None
        window.errors.extend(prior_errors)
        with ExitStack() as patches:
            # SIGKILL/pidfd functions are absent on Windows. These mocks model
            # Linux ownership and successful/failed sends without real signals.
            patches.enter_context(patch.object(viewer.signal, 'SIGKILL', 9, create=True))
            patches.enter_context(patch.object(viewer.signal, 'Signals', side_effect=lambda value:
                SimpleNamespace(name={int(signal.SIGINT): 'SIGINT', int(signal.SIGTERM): 'SIGTERM', 9: 'SIGKILL'}[int(value)])))
            sender = patches.enter_context(patch.object(viewer.signal, 'pidfd_send_signal',
                side_effect=lambda descriptor, signum: send(signum), create=True))
            close = patches.enter_context(patch.object(viewer.os, 'close'))
            # This is the real OwnedRviz stop/finish path, not a fixture whose
            # finish method always returns PASS.
            window.request_stop()
            report = window.finish()
            if pidfd:
                close.assert_called_once_with(71)
                self.assertIsNone(window.pidfd)
            else:
                close.assert_not_called()
            if pidfd and not already_exited:
                self.assertGreater(sender.call_count, 0)
        return report, state

    def test_owner_sigint_to_live_child_is_expected_when_reaped_before_rviz_initializes(self):
        for pidfd in (False, True):
            with self.subTest(pidfd=pidfd):
                report, state = self.cleanup(pidfd=pidfd)
                self.assertEqual(report['status'], 'PASS')
                self.assertEqual(report['exit_code'], -signal.SIGINT)
                self.assertEqual(report['termination_kind'], 'OWNED_SIGINT')
                self.assertTrue(report['wait_completed'])
                self.assertFalse(report['alive_after_cleanup'])
                self.assertFalse(report['forced_termination'])
                self.assertEqual(report['signals'], ['SIGINT'])
                self.assertEqual(report['errors'], [])
                self.assertEqual(state['send_attempts'], [signal.SIGINT])

    def test_external_sigint_already_exited_is_not_reclassified_as_owner_stop(self):
        report, state = self.cleanup(already_exited=True)
        self.assertEqual(state['send_attempts'], [])
        self.assertEqual(report['signals'], [])
        self.assertEqual(report['exit_code'], -signal.SIGINT)
        self.assertEqual(report['termination_kind'], 'SIGNAL_EXIT')
        self.assertEqual(report['status'], 'FAIL')

    def test_failed_sigint_send_cannot_supply_owner_evidence(self):
        for error in (PermissionError('synthetic send denied'), ProcessLookupError('synthetic child vanished')):
            with self.subTest(error=type(error).__name__):
                report, state = self.cleanup(signal_error=error)
                self.assertEqual(state['send_attempts'], [signal.SIGINT])
                self.assertEqual(report['signals'], [])
                self.assertEqual(report['status'], 'FAIL')
                self.assertEqual(report['exit_code'], -signal.SIGINT)

    def test_other_negative_signals_and_nonzero_exits_stay_failed(self):
        for code in (-9, -signal.SIGTERM, 1, 139, 245):
            with self.subTest(exit_code=code):
                report, _ = self.cleanup(code)
                self.assertEqual(report['status'], 'FAIL')
                self.assertEqual(report['exit_code'], code)
                self.assertEqual(report['termination_kind'], 'SIGNAL_EXIT' if code < 0 else 'NONZERO_EXIT')

    def test_forced_escalation_does_not_become_expected_even_if_sigint_is_eventual_exit(self):
        report, _ = self.cleanup(wait_timeouts=1)
        self.assertEqual(report['exit_code'], -signal.SIGINT)
        self.assertEqual(report['status'], 'FAIL')
        self.assertTrue(report['forced_termination'])
        self.assertEqual(report['signals'], ['SIGINT', 'SIGTERM'])
        self.assertEqual(report['termination_kind'], 'FORCED_TERMINATION')

    def test_sigkill_after_deadline_and_unreaped_child_stay_failed(self):
        report, _ = self.cleanup(-9, wait_timeouts=2)
        self.assertEqual(report['status'], 'FAIL')
        self.assertEqual(report['exit_code'], -9)
        self.assertEqual(report['signals'], ['SIGINT', 'SIGTERM', 'SIGKILL'])
        self.assertEqual(report['termination_kind'], 'FORCED_TERMINATION')
        alive, _ = self.cleanup(never_exits=True)
        self.assertEqual(alive['status'], 'FAIL')
        self.assertIsNone(alive['exit_code'])
        self.assertFalse(alive['wait_completed'])
        self.assertTrue(alive['alive_after_cleanup'])
        self.assertEqual(alive['termination_kind'], 'NOT_REAPED')

    def test_prior_errors_are_preserved_when_owner_sigint_closes_child(self):
        report, _ = self.cleanup(prior_errors=['OpenGL rendering context failure'])
        self.assertEqual(report['status'], 'FAIL')
        self.assertEqual(report['exit_code'], -signal.SIGINT)
        self.assertIn('OpenGL rendering context failure', report['errors'])
        self.assertNotEqual(report['termination_kind'], 'OWNED_SIGINT')

    def test_normal_zero_exit_retains_normal_kind(self):
        report, _ = self.cleanup(0, already_exited=True)
        self.assertEqual(report['status'], 'PASS')
        self.assertEqual(report['exit_code'], 0)
        self.assertEqual(report['termination_kind'], 'CLEAN_EXIT')


if __name__ == '__main__':
    unittest.main()
