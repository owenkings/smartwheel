"""Test command source for the stand and ground tests at 0.10 m/s (velocity_channel.md 7.3).

Takes the base bridge's place on the session's auto.sock while the gate and the bridge are not running, and sends
the wheel owner one decision every 50 ms in the gate's format (state ACTIVE, limits 0.10 m/s and 0.30 rad/s, no
reverse): zero until the owner reports AUTO, then straight ahead at 0.10 m/s, the only speed it sends. It never
starts automatic control: the operator does, as for the closed loop (wc_runtime.auto_operator start, then a click
into the RViz window).

--stop software_zero: after --forward-s seconds at 0.10 m/s it sends zero; the owner brakes at its approved
deceleration. --stop external: it keeps 0.10 m/s until something else stops the wheels (the emergency stop, the
USB adapter pulled, kill -9 of this process: the owner then brakes because its bridge is gone), and after --max-s
seconds at the latest it sends zero itself. It ends when the wheels have stood still for 1 s after the stop (10 s
after the stop at the latest), or when the owner closes the socket, and prints the stop time and distance from the
session's wheel journal (wc_motion.auto_stop_report).

--stop kill is the kill -9 stop without anyone at a terminal, where a click out of RViz would already stop
automatic control: a child test source in external mode is killed with SIGKILL --forward-s seconds after
automatic control started, and this process prints the stop report.

Refuses unless the owner's evidence stage is STAND_TEST or GROUND_TEST (the closed loop uses the real gate), or the
owner is the dry run (--dry-run). Needs WC_AUTO_DRIVE_APPROVAL=APPROVED_BY_USER_<today>, like the bridge, except
with --dry-run.
"""
import argparse
import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import uuid

from .auto_bridge import APPROVAL_ENV, BridgeCore, BridgeError
from .auto_protocol import GATE_COMMAND_SCHEMA

SPEED_M_S = 0.10
MAX_ANGULAR_RAD_S = 0.30
PERIOD_S = 0.05
STAGES = ('STAND_TEST', 'GROUND_TEST')
STANDSTILL_S = 1.0     # zero wheel feedback this long after the stop ends the trial
STOP_WAIT_S = 10.0     # ... and without it the trial ends this long after the stop, still sending zero until then
WAIT_AUTO_S = 60.0     # the operator has this long to start automatic control


class TestSource:
    """One trial. ``step`` sends one decision and returns False once the trial is over."""

    def __init__(self, core, *, stop, forward_s=3.0, max_s=8.0, dry_run=False, clock=time.monotonic, out=print):
        if stop not in ('software_zero', 'external'):
            raise ValueError('stop must be software_zero or external')
        self.core, self.stop, self.forward_s, self.max_s, self.dry_run = core, stop, forward_s, max_s, dry_run
        self.clock, self.out = clock, out
        self.gate_id = 'test_source_' + uuid.uuid4().hex[:12]
        self.seq = 0
        self.phase, self.since = 'WAIT_AUTO', clock()
        self.zero_since, self.last_sequence = None, None
        self.events = []

    def _phase(self, phase, note):
        self.phase, self.since = phase, self.clock()
        self.events.append({'mono_s': round(self.since, 3), 'phase': phase, 'note': note})
        self.out('%s %s: %s' % (datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3], phase, note))

    def _gate(self, linear):
        self.seq += 1
        return json.dumps({'schema': GATE_COMMAND_SCHEMA, 'gate_id': self.gate_id, 'seq': self.seq,
                           'stamp_ns': time.time_ns(), 'mono_ns': self.core.clock_ns(), 'state': 'ACTIVE',
                           'reasons': ['TEST_SOURCE_' + self.phase], 'linear': linear, 'angular': 0.0,
                           'limits': {'max_linear': SPEED_M_S, 'max_angular': MAX_ANGULAR_RAD_S,
                                      'linear_ceiling': SPEED_M_S, 'allow_reverse': False},
                           'config_sha256': None})

    def _check(self, status):
        if self.dry_run:
            return
        if status.get('evidence_stage') not in STAGES:
            raise BridgeError('the test source runs only at the STAND_TEST or GROUND_TEST evidence stage, not %s'
                              % status.get('evidence_stage'))

    def step(self):
        now = self.clock()
        self.core.pump()
        status = self.core.status if self.core.fresh() else None
        if status is not None:
            self._check(status)
        state = None if status is None else status.get('auto_state')
        linear = 0.0
        if self.phase == 'WAIT_AUTO':
            if state == 'AUTO':
                self._phase('FORWARD', 'automatic control started: 0.10 m/s ahead')
            elif now - self.since > WAIT_AUTO_S:
                raise BridgeError('automatic control was not started within %.0f s' % WAIT_AUTO_S)
        if self.phase == 'FORWARD':
            elapsed = now - self.since
            if state != 'AUTO':
                self._phase('OWNER_STOPPED', 'the owner left AUTO: %s' % (None if status is None else
                                                                          status.get('immediate_reason') or
                                                                          status.get('reason')))
            elif self.stop == 'software_zero' and elapsed >= self.forward_s:
                self._phase('ZERO', 'software zero sent after %.1f s' % elapsed)
            elif self.stop == 'external' and elapsed >= self.max_s:
                self._phase('ZERO', 'nothing stopped the wheels within %.1f s: software zero sent' % self.max_s)
            else:
                linear = SPEED_M_S
        if self.phase in ('ZERO', 'OWNER_STOPPED'):
            feedback = (status or {}).get('feedback') or {}
            if feedback.get('sequence') is not None and feedback['sequence'] != self.last_sequence:
                self.last_sequence = feedback['sequence']
                standing = feedback.get('left_m_s') == 0 and feedback.get('right_m_s') == 0
                self.zero_since = (self.zero_since or now) if standing else None
            if self.zero_since is not None and now - self.zero_since >= STANDSTILL_S:
                self._phase('DONE', 'wheels stood still for %.1f s' % STANDSTILL_S)
            elif now - self.since >= STOP_WAIT_S:
                self._phase('DONE', 'no standstill in the wheel feedback within %.0f s of the stop' % STOP_WAIT_S)
        self.core.on_gate_command(self._gate(linear))
        return self.phase != 'DONE'


def main(argv=None, *, environ=None, today=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--project-root', type=Path, required=True)
    parser.add_argument('--stop', choices=('software_zero', 'external', 'kill'), required=True)
    parser.add_argument('--forward-s', type=float, default=3.0,
                        help='software_zero and kill: seconds at 0.10 m/s before the stop (0.5-10)')
    parser.add_argument('--max-s', type=float, default=8.0, help='external: zero after this many seconds (1-20)')
    parser.add_argument('--session-root', type=Path, help='default: the one running recording with an auto.sock')
    parser.add_argument('--session-id')
    parser.add_argument('--dry-run', action='store_true', help='against wc_runtime.auto_dry_run only')
    args = parser.parse_args(argv)
    if not 0.5 <= args.forward_s <= 10 or not 1 <= args.max_s <= 20:
        raise SystemExit('--forward-s must be 0.5-10 s and --max-s 1-20 s')
    environ = os.environ if environ is None else environ
    today = today or datetime.date.today()
    if not args.dry_run and environ.get(APPROVAL_ENV) != 'APPROVED_BY_USER_' + today.strftime('%Y%m%d'):
        raise SystemExit('the test source needs %s=APPROVED_BY_USER_%s' % (APPROVAL_ENV, today.strftime('%Y%m%d')))
    from wc_runtime.mapping_auto import auto_socket_path, running_auto_sessions
    root = args.project_root.resolve()
    if (args.session_root is None) != (args.session_id is None):
        raise SystemExit('give --session-root and --session-id together, or neither')
    if args.session_root is None:
        sessions = running_auto_sessions(root)
        if len(sessions) != 1:
            raise SystemExit('expected one running recording with an automatic control socket, found %d' % len(sessions))
        session_root, session_id = sessions[0]
    else:
        session_root, session_id = args.session_root.absolute(), args.session_id
    if args.stop == 'kill':
        return kill_trial(root, session_root, session_id, args.forward_s, args.dry_run, environ)
    core = BridgeCore(session_id, 'test_source_' + uuid.uuid4().hex[:12], require_dry_run=args.dry_run)
    source = TestSource(core, stop=args.stop, forward_s=args.forward_s, max_s=args.max_s, dry_run=args.dry_run)
    print('test source pid %d, session %s, stop %s; start automatic control now (auto_operator start, then click '
          'into RViz)' % (os.getpid(), session_id, args.stop), flush=True)
    started, failure = time.monotonic(), None
    try:
        core.connect(auto_socket_path(root, session_root, session_id))
        while source.step():
            time.sleep(PERIOD_S)
    except (BridgeError, OSError) as error:
        failure = str(error)
        print('test source stopped: ' + failure, flush=True)
    finally:
        core.close()
    time.sleep(0.5)   # the owner's journal is written in batches of up to 50 ms
    from .auto_stop_report import lines, report
    print_report(session_root, started)
    return 1 if failure and source.phase not in ('OWNER_STOPPED', 'ZERO') else 0


def print_report(session_root, started):
    from .auto_stop_report import lines, report
    try:
        print('\n'.join(lines(report(session_root, since_mono_s=started))), flush=True)
    except (OSError, ValueError, KeyError) as error:
        print('no stop report: %s' % error, flush=True)


def kill_trial(root, session_root, session_id, forward_s, dry_run, environ):
    """--stop kill: a child test source in external mode, killed with SIGKILL forward_s after AUTO started."""
    command = [sys.executable, '-u', '-m', 'wc_motion.auto_test_source', '--project-root', str(root),
               '--stop', 'external', '--max-s', '%.1f' % min(20.0, forward_s + 5.0),
               '--session-root', str(session_root), '--session-id', session_id] + (['--dry-run'] if dry_run else [])
    started = time.monotonic()
    child = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=dict(environ))
    print('test source pid %d is killed with SIGKILL %.1f s after automatic control starts' % (child.pid, forward_s),
          flush=True)

    def kill():
        if child.poll() is None:
            os.kill(child.pid, signal.SIGKILL)
            print('%s KILLED: kill -9 %d' % (datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3], child.pid),
                  flush=True)
    timer = None
    try:
        for text in child.stdout:
            print(text, end='', flush=True)
            if timer is None and ' FORWARD: ' in text:
                timer = threading.Timer(forward_s, kill)
                timer.start()
    finally:
        if timer is not None:
            timer.cancel()
        if child.poll() is None:
            child.kill()   # this process was interrupted: the child never outlives it
        child.wait()
    time.sleep(1.5)   # the owner brakes once its bridge is gone; its journal is written in batches
    print_report(session_root, started)
    return 0 if child.returncode == -signal.SIGKILL else 1


if __name__ == '__main__':
    raise SystemExit(main())
