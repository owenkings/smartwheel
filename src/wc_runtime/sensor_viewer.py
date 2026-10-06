"""Bounded XT-M60 preview; each sensor has its own RViz fixed frame.

Only the existing drivers/stop commands are dispatched. No recording, mapping,
IMU, encoder or motor operation is started by this entry point.
"""
import argparse
import getpass
import json
import os
from pathlib import Path
import platform
import re
import shutil
import signal
import socket
import subprocess
import sys
import time
import uuid
from contextlib import redirect_stderr, redirect_stdout


ROOT = Path('/home/nvidia/wheelchair')
MODES = {'dual': ('left', 'right'), 'single_left': ('left',), 'single_right': ('right',)}


def require_target():
    if (os.name != 'posix' or platform.machine() != 'aarch64' or
            socket.gethostname() != 'ubuntu' or getpass.getuser() != 'nvidia'):
        raise RuntimeError('Requires the verified Orin target ubuntu/nvidia/aarch64')
    import pwd
    if pwd.getpwuid(os.getuid()).pw_name != 'nvidia':
        raise RuntimeError('Current UID is not nvidia')
    if (not ROOT.is_dir() or ROOT.is_symlink() or Path.cwd().resolve() != ROOT or
            Path(__file__).resolve().parents[2] != ROOT):
        raise RuntimeError('Run the installed entry point in /home/nvidia/wheelchair')
    if not hasattr(os, 'pidfd_open') or not hasattr(signal, 'pidfd_send_signal'):
        raise RuntimeError('Owned-process pidfd support is required')


def desktop_environment(*, environ=None, proc_root=Path('/proc'), uid=None):
    """Use only this UID's GNOME session, not another user's display/cookies."""
    env = dict(os.environ if environ is None else environ)
    uid = os.getuid() if uid is None else uid
    candidates = {}
    for comm in sorted(Path(proc_root).glob('[0-9]*/comm')):
        try:
            if comm.stat().st_uid != uid or comm.read_text().strip() != 'gnome-shell':
                continue
            values = {}
            for item in (comm.parent/'environ').read_bytes().split(b'\0'):
                if item.startswith((b'DISPLAY=', b'XAUTHORITY=', b'DBUS_SESSION_BUS_ADDRESS=')):
                    key, value = item.decode('utf-8').split('=', 1)
                    values[key] = value
            display, auth = values.get('DISPLAY', ''), values.get('XAUTHORITY', '')
            if not re.fullmatch(r':\d+(?:\.\d+)?', display) or not auth:
                continue
            auth_path = Path(auth)
            if (not auth_path.is_absolute() or not auth_path.is_file() or
                    auth_path.stat().st_uid != uid or not os.access(auth_path, os.R_OK)):
                continue
            candidates[(display, auth)] = values
        except (OSError, UnicodeError):
            continue
    if not candidates:
        raise RuntimeError('No accessible graphical GNOME session for the current UID; no sensors started')
    selected = candidates.get((env.get('DISPLAY'), env.get('XAUTHORITY')))
    if selected is None:
        if len(candidates) != 1:
            raise RuntimeError('Multiple graphical sessions found; select this desktop terminal explicitly')
        selected = next(iter(candidates.values()))
    env.update(selected)
    return env


def confirm_display(env):
    executable = shutil.which('xprop', path=env.get('PATH'))
    if executable is None:
        raise RuntimeError('xprop is required for the read-only display connectivity check')
    result = subprocess.run([executable, '-root', '_NET_SUPPORTING_WM_CHECK'], env=env,
                            stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=5)
    if result.returncode:
        raise RuntimeError('Authorized display is not reachable: '+result.stderr[-1000:])


def write_report(path, value):
    temporary = path.with_name(path.name+'.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def process_identity(pid):
    value = Path('/proc')/str(pid)/'stat'
    text = value.read_text()
    fields = text[text.rindex(')')+2:].split()
    return {'pid': pid, 'state': fields[0], 'start_ticks': fields[19]}


class OwnedRviz:
    """A direct Popen child, retained even if pidfd acquisition fails halfway."""
    def __init__(self, side, command, env, root, log_path):
        self.side, self.command, self.env, self.root, self.log_path = side, command, env, root, log_path
        self.child = self.pidfd = self.log = None
        self.start_ticks = None
        self.stop_started = None
        self.signals = []
        self.errors = []

    def start(self):
        self.log = self.log_path.open('xb', buffering=0)
        self.child = subprocess.Popen(self.command, env=self.env, cwd=self.root,
                                      stdin=subprocess.DEVNULL, stdout=self.log, stderr=subprocess.STDOUT,
                                      start_new_session=True)
        self.start_ticks = process_identity(self.child.pid)['start_ticks']
        self.pidfd = os.pidfd_open(self.child.pid)

    def metadata(self):
        return {'side': self.side, 'pid': self.child.pid if self.child is not None else None,
                'start_ticks': self.start_ticks, 'pidfd_acquired': self.pidfd is not None,
                'argv': list(map(str, self.command)), 'log_path': str(self.log_path),
                'start_new_session': True}

    def poll(self):
        return self.child.poll() if self.child is not None else None

    def _signal(self, signum):
        if self.child is None or self.child.poll() is not None:
            return
        try:
            if self.pidfd is not None:
                signal.pidfd_send_signal(self.pidfd, signum)
            else:
                # Startup may fail between Popen and pidfd_open. This Popen is
                # still our direct, unreaped child; never look up a PID by name.
                self.child.send_signal(signum)
            self.signals.append(signal.Signals(signum).name)
        except ProcessLookupError:
            pass
        except OSError as error:
            self.errors.append('signal: '+str(error))

    def request_stop(self):
        self.stop_started = time.monotonic()
        self._signal(signal.SIGINT)

    def finish(self):
        if self.stop_started is None:
            self.request_stop()
        forced = False
        wait_completed = expected_owner_sigint = False
        termination_kind = 'NOT_STARTED'
        try:
            if self.child is not None:
                for wait_s, next_signal in ((max(0, self.stop_started+10-time.monotonic()), signal.SIGTERM),
                                             (3, signal.SIGKILL), (2, None)):
                    try:
                        self.child.wait(timeout=wait_s)
                        wait_completed = True
                        break
                    except subprocess.TimeoutExpired:
                        forced = True
                        self.errors.append('RViz cleanup deadline exceeded')
                        if next_signal is not None:
                            self._signal(next_signal)
                code = self.child.poll()
                alive = code is None
                # _signal records SIGINT only after a successful send to our
                # still-live direct child/pidfd. An already-exited child or a
                # failed send supplies no such evidence. A startup cancellation
                # can be reaped as -SIGINT before RViz installs its handler.
                expected_owner_sigint = (wait_completed and not alive and code == -signal.SIGINT
                    and self.stop_started is not None and 'SIGINT' in self.signals
                    and all(name == 'SIGINT' for name in self.signals)
                    and not forced and not self.errors)
                if alive or (code != 0 and not expected_owner_sigint):
                    self.errors.append('RViz did not exit normally: '+str(code))
                termination_kind = ('NOT_REAPED' if alive else 'FORCED_TERMINATION' if forced else
                    'OWNED_SIGINT' if expected_owner_sigint else 'CLEAN_EXIT' if code == 0 else
                    'SIGNAL_EXIT' if code < 0 else 'NONZERO_EXIT')
            else:
                code, alive = None, False
            return {'side': self.side, 'pid': self.child.pid if self.child is not None else None,
                    'start_ticks': self.start_ticks,
                    'pidfd_acquired': self.pidfd is not None, 'exit_code': code,
                    'termination_kind': termination_kind, 'wait_completed': wait_completed,
                    'alive_after_cleanup': alive, 'signals': self.signals,
                    'forced_termination': forced, 'errors': list(self.errors),
                    'status': 'FAIL' if self.errors else ('PASS' if self.child is not None else 'NOT_STARTED')}
        finally:
            if self.pidfd is not None:
                os.close(self.pidfd)
                self.pidfd = None
            if self.log is not None:
                self.log.close()


class Runtime:
    def __init__(self):
        require_target()  # Before importing target-only fcntl/ROS runtime code.
        from . import cli
        self.cli, self.root = cli, ROOT

    def gui(self):
        env = desktop_environment()
        env.update(self.cli.environment())
        confirm_display(env)
        return env

    def start_drivers(self, session, mode, duration, report_dir):
        with (report_dir/'driver-start.log').open('x', encoding='utf-8') as log:
            with redirect_stdout(log), redirect_stderr(log):
                return self.cli.main(['drivers', '--session', session, '--mode', mode, '--duration', str(duration)])

    def manifest_path(self, session):
        return self.root/'.phase1_runtime/sessions'/session/'drivers/manifest.json'

    def manifest(self, session):
        path = self.manifest_path(session)
        if not path.exists():
            return None
        if path.is_symlink() or not path.resolve().is_relative_to((self.root/'.phase1_runtime/sessions'/session).resolve()):
            raise RuntimeError('Owned driver manifest path was redirected')
        data = path.read_bytes()
        if len(data) > 1024*1024:
            raise RuntimeError('Driver manifest exceeds bounded read size')
        value = json.loads(data)
        if value.get('session_id') != session or value.get('role') != 'drivers':
            raise RuntimeError('Driver manifest identity does not match this viewer session')
        return value

    def driver_alive(self, manifest):
        owners = [(manifest.get('supervisor_pid'), manifest.get('supervisor_start_ticks'))]
        owners += [(row.get('pid'), row.get('start_ticks')) for row in manifest.get('children', [])]
        if len(owners) < 2:
            return False
        try:
            for pid, expected in owners:
                if type(pid) is not int or pid <= 0:
                    return False
                identity = process_identity(pid)
                if identity['start_ticks'] != expected or identity['state'] in ('Z', 'X'):
                    return False
            return True
        except (OSError, ValueError):
            return False

    def stop_drivers(self, session, report_dir):
        with (report_dir/'driver-stop.log').open('x', encoding='utf-8') as log:
            with redirect_stdout(log), redirect_stderr(log):
                return self.cli.main(['stop', '--session', session])

    def new_window(self, side, session, env, report_dir):
        command = self.cli.ros_command(['rviz2', '-d', self.root/'config/rviz'/('sensor_'+side+'.rviz'),
                                        '--ros-args', '-r', '__node:=wc_lidar_view_'+session+'_'+side])
        return OwnedRviz(side, command, env, self.root, report_dir/('rviz-'+side+'.log'))


def cleanup_driver(runtime, session, report_dir, *, clock=time.monotonic, sleep=time.sleep):
    """A failed/uncertain begin can still have created the owned supervisor."""
    try:
        deadline = clock()+8
        manifest = runtime.manifest(session)
        while manifest is None and clock() < deadline:
            sleep(.2)
            manifest = runtime.manifest(session)
        if manifest is None:
            return {'status': 'UNKNOWN', 'error': 'Driver start was attempted but no owned manifest appeared; inspect runtime evidence',
                    'manifest_path': str(runtime.manifest_path(session)), 'stop_called': False}
        stop_code = None
        if manifest.get('state') == 'RUNNING':
            stop_code = runtime.stop_drivers(session, report_dir)
            manifest = runtime.manifest(session)
        clean = (manifest is not None and manifest.get('state') == 'STOPPED' and
                 manifest.get('exit_code') == 0 and not manifest.get('cleanup_errors') and stop_code in (None, 0))
        return {'status': 'PASS' if clean else 'FAIL', 'stop_called': stop_code is not None,
                'stop_return_code': stop_code, 'manifest': manifest,
                'manifest_path': str(runtime.manifest_path(session))}
    except Exception as error:
        return {'status': 'FAIL', 'error': str(error), 'manifest_path': str(runtime.manifest_path(session))}


def run_view(runtime, mode, duration, stop_requested, *, clock=time.monotonic, sleep=time.sleep):
    if mode not in MODES or type(duration) is not int or not 1 <= duration <= 3600:
        raise ValueError('Mode must be dual/single_left/single_right; duration must be 1..3600 seconds')
    session = 'lidar_view_'+time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())+'_'+uuid.uuid4().hex[:12]
    from .storage_policy import resolve_storage_path
    report_dir = resolve_storage_path(runtime.root, Path('reports/sensor_views')/session)
    if (runtime.root/'.phase1_runtime/sessions'/session).exists():
        raise RuntimeError('Generated session already exists; refusing to reuse another session')
    report_dir.mkdir(parents=True, exist_ok=False)
    report_path = report_dir/'result.json'
    report = {'session_id': session, 'mode': mode, 'requested_duration_s': duration,
              'project_root': str(runtime.root), 'started_unix_ns': time.time_ns(),
              'state': 'PREFLIGHT', 'errors': [], 'recording_started': False,
              'imu_encoder_or_motor_started': False, 'fusion_or_mapping_started': False,
              'point_cloud_quality_verified': False, 'driver_start_attempted': False,
              'rviz_views': list(MODES[mode]), 'driver_cleanup': {'status': 'NOT_STARTED'}, 'rviz_cleanup': []}
    windows = []
    write_report(report_path, report)
    started = clock()
    try:
        env = runtime.gui()  # Must finish before touching the drivers command.
        report['graphical_session'] = {'DISPLAY': env['DISPLAY'], 'XAUTHORITY': env['XAUTHORITY']}
        for side in MODES[mode]:
            config = runtime.root/'config/rviz'/('sensor_'+side+'.rviz')
            if not config.is_file() or config.is_symlink():
                raise RuntimeError('Reviewed single-sensor RViz configuration missing: '+str(config))
        if stop_requested():
            report['stop_reason'] = 'USER_STOP_BEFORE_ACQUISITION'
        else:
            report['driver_start_attempted'] = True
            write_report(report_path, report)
            code = runtime.start_drivers(session, mode, duration, report_dir)
            report['driver_start_return_code'] = code
            if code != 0:
                raise RuntimeError('Driver startup failed or is uncertain; checking only the owned session during cleanup')
            manifest = runtime.manifest(session)
            if manifest is None or manifest.get('state') != 'RUNNING' or not runtime.driver_alive(manifest):
                raise RuntimeError('Owned driver session did not remain running after startup')
            if not stop_requested():
                for side in MODES[mode]:
                    if stop_requested():
                        break
                    window = runtime.new_window(side, session, env, report_dir)
                    windows.append(window)  # Track before Popen/pidfd can fail halfway.
                    window.start()
            report['rviz_processes'] = [window.metadata() for window in windows]
            report['state'] = 'VIEWING'
            write_report(report_path, report)
            print(json.dumps({'event': 'LIDAR_VIEW_STARTED', 'session_id': session,
                              'mode': mode, 'report': str(report_path)}, ensure_ascii=False), flush=True)
            while True:
                if stop_requested():
                    report['stop_reason'] = 'USER_SIGNAL'
                    break
                if clock()-started >= duration:
                    report['stop_reason'] = 'DURATION_REACHED'
                    break
                current = runtime.manifest(session)
                if current is None or current.get('state') != 'RUNNING' or not runtime.driver_alive(current):
                    raise RuntimeError('Owned driver exited before viewer duration; closing the owned windows')
                closed = []
                for window in windows:
                    exit_code = window.poll()
                    if exit_code is not None:
                        closed.append((window.side, exit_code))
                if closed:
                    report['stop_reason'] = 'RVIZ_WINDOW_EXIT'
                    report['window_exit_trigger'] = closed
                    if any(code != 0 for _, code in closed):
                        raise RuntimeError('An owned RViz process exited with an error')
                    break
                sleep(.2)
    except Exception as error:
        report['errors'].append(str(error))
        report['stop_reason'] = report.get('stop_reason', 'STARTUP_OR_RUNTIME_FAILURE')
    finally:
        report['state'] = 'STOPPING'
        report['rviz_processes'] = [window.metadata() for window in windows]
        try:
            write_report(report_path, report)
        except OSError as error:
            report['errors'].append('Could not write stopping report: '+str(error))
        # Signal every owned window first; sensor cleanup and RViz exit can then
        # proceed concurrently rather than leaving another window open for 30s.
        for window in windows:
            try:
                window.request_stop()
            except Exception as error:
                report['errors'].append('RViz stop request: '+str(error))
        if report['driver_start_attempted']:
            report['driver_cleanup'] = cleanup_driver(runtime, session, report_dir, clock=clock, sleep=sleep)
        for window in windows:
            try:
                report['rviz_cleanup'].append(window.finish())
            except Exception as error:
                report['rviz_cleanup'].append({'side': window.side, 'status': 'FAIL', 'error': str(error)})
        clean = (report['driver_cleanup']['status'] in ('PASS', 'NOT_STARTED') and
                 all(row['status'] in ('PASS', 'NOT_STARTED') for row in report['rviz_cleanup']))
        report['cleanup_status'] = 'PASS' if clean else 'FAIL'
        report['status'] = 'PASS' if clean and not report['errors'] else 'FAIL'
        report['state'] = 'FINISHED'
        report['finished_unix_ns'] = time.time_ns()
        report['host_total_elapsed_s'] = clock()-started
        write_report(report_path, report)
        print(json.dumps({'event': 'LIDAR_VIEW_FINISHED', 'status': report['status'],
                          'cleanup_status': report['cleanup_status'], 'session_id': session,
                          'report': str(report_path), 'errors': report['errors']}, ensure_ascii=False), flush=True)
    return 0 if report['status'] == 'PASS' else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', nargs='?', choices=tuple(MODES), default='dual')
    parser.add_argument('duration', nargs='?', type=int, default=900)
    args = parser.parse_args(argv)
    if not 1 <= args.duration <= 3600:
        parser.error('duration must be 1..3600 seconds')
    previous = {}
    requested = False
    def stop(signum, frame):
        nonlocal requested
        requested = True
    try:
        runtime = Runtime()
        stop_signals = (signal.SIGINT, signal.SIGTERM)
        if hasattr(signal, 'SIGHUP'):
            stop_signals += (signal.SIGHUP,)
        for signum in stop_signals:
            previous[signum] = signal.getsignal(signum)
            signal.signal(signum, stop)
        return run_view(runtime, args.mode, args.duration, lambda: requested)
    except (OSError, RuntimeError, ValueError) as error:
        print(json.dumps({'status': 'FAIL', 'reason': str(error)}, ensure_ascii=False), file=sys.stderr)
        return 2
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)


if __name__ == '__main__':
    raise SystemExit(main())
