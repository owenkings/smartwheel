"""User-operated WASD dry-run. Real motor dispatch is deliberately unavailable.

The reviewed old driver does not establish this firmware's disconnect-stop
semantics. A host key watchdog cannot stop a controller after the serial link
or host fails. No configuration flag can bypass that missing hardware contract.
"""

import argparse
import json
from pathlib import Path
import select
import signal
import sys
import time

from .feedback_transport import bounded
from .protocol import FeedbackError


REAL_BLOCK_REASON = 'REAL_WASD_DISABLED_UNVERIFIED_CONTROLLER_DISCONNECT_STOP_AND_BRAKE_SEMANTICS'


class DryRunBackend:
    """Simulation sink only. No serial, ROS Twist, enable or brake operations."""
    def __init__(self):
        self.available = True
        self.commands = []

    def intent(self, linear, angular):
        if not self.available:
            raise FeedbackError('simulated command link unavailable')
        self.commands.append((linear, angular))
        return {'simulation_acknowledged': True, 'hardware_command_sent': False}


class ManualController:
    def __init__(self, config, backend=None, observer=None, *, mode='dry-run'):
        if mode != 'dry-run':
            raise FeedbackError(REAL_BLOCK_REASON)
        if config.get('schema') != 'wc_manual_dry_run_v1' or config.get('hardware_enabled') is not False:
            raise FeedbackError('manual configuration must explicitly disable hardware')
        self.linear = bounded(config.get('linear_speed_m_s'), 'manual linear speed', 0, .15)
        self.angular = bounded(config.get('angular_speed_rad_s'), 'manual angular speed', 0, .3)
        self.key_timeout = bounded(config.get('key_timeout_s'), 'key timeout', .05, .5)
        self.link_timeout = bounded(config.get('link_timeout_s'), 'link timeout', .05, .5)
        self.backend = backend or DryRunBackend()
        if not isinstance(self.backend, DryRunBackend):
            raise FeedbackError('only the dry-run backend is implemented and reviewed')
        self.observer = observer or (lambda record: None)
        self.state = 'DISARMED'
        self.keys = set()
        self.last_now = self.last_key = self.last_link = None
        self.target = (0.0, 0.0)
        self.reason = 'STARTUP_DISABLED'
        self.last_stop_simulated = None
        self.journal_error = None
        self.closed = False

    def _now(self, now):
        try:
            bounded(now, 'manual monotonic timestamp', 0, 1e15)
        except FeedbackError:
            self._stop('INVALID_MONOTONIC_TIME', fault=True)
            raise
        if self.last_now is not None and now < self.last_now:
            self._stop('MONOTONIC_TIME_REVERSED', fault=True)
            raise FeedbackError('manual monotonic time moved backwards')
        self.last_now = now

    def status(self):
        return {'schema': 'wc_manual_status_v1', 'mode': 'DRY_RUN', 'state': self.state,
            'reason': self.reason, 'keys': sorted(self.keys), 'target_linear_m_s': self.target[0],
            'target_angular_rad_s': self.target[1], 'hardware_enabled': False,
            'hardware_command_sent': False, 'hardware_stop_confirmed': False,
            'last_stop_simulation_acknowledged': self.last_stop_simulated,
            'journal_error': self.journal_error,
            'real_entry_block': REAL_BLOCK_REASON}

    def _observe(self, record):
        try:
            self.observer(record)
            return True
        except Exception as error:
            self.journal_error = str(error)
            return False

    def heartbeat(self, now, connected=True):
        self._now(now)
        if self.closed:
            raise FeedbackError('manual session is closed')
        self.backend.available = connected is True
        if connected is not True:
            self._stop('COMMAND_LINK_DISCONNECTED', fault=True)
            return
        self.last_link = now

    def arm(self, now, *, operator_confirmed=False):
        self._now(now)
        if operator_confirmed is not True:
            raise FeedbackError('manual operator arming event is required')
        if self.closed or self.state == 'FAULT':
            raise FeedbackError('fault/closed session cannot rearm; start a new session')
        if self.state == 'ARMED':
            raise FeedbackError('already armed; stop before requesting a new arm')
        if self.last_link is None or now-self.last_link > self.link_timeout or not self.backend.available:
            raise FeedbackError('fresh command-link health required before arm')
        self.keys.clear()
        self.last_key = now
        self.state, self.reason = 'ARMED', 'EXPLICIT_OPERATOR_ARM_DRY_RUN'
        if not self._observe(self.status()):
            self._stop('JOURNAL_WRITE_FAILED', fault=True)
            raise FeedbackError('manual journal write failed')

    def _send(self, target, reason):
        if target == self.target:
            return
        if not self._observe({'schema': 'wc_manual_intent_v1', 'linear_m_s': target[0], 'angular_rad_s': target[1],
            'reason': reason, 'mode': 'DRY_RUN', 'hardware_command_sent': False}):
            self._stop('JOURNAL_WRITE_FAILED', fault=True)
            raise FeedbackError('manual journal write failed')
        try:
            self.backend.intent(*target)
            self.target = target
        except Exception:
            self._stop('SIMULATED_COMMAND_WRITE_FAILED', fault=True, force_attempt=True)
            raise

    def _stop(self, reason, *, fault=False, force_attempt=False):
        moving = self.target != (0.0, 0.0) or force_attempt
        self.keys.clear()
        self.state = 'FAULT' if fault or self.state == 'FAULT' else 'DISARMED'
        self.reason = reason
        if moving:
            if not self._observe({'schema': 'wc_manual_intent_v1', 'linear_m_s': 0.0, 'angular_rad_s': 0.0,
                'reason': reason, 'mode': 'DRY_RUN', 'hardware_command_sent': False}):
                self.state = 'FAULT'
            try:
                ack = self.backend.intent(0.0, 0.0)
                self.last_stop_simulated = ack.get('simulation_acknowledged') is True
            except Exception:
                self.last_stop_simulated = False
                self.state = 'FAULT'
            # Requested state is zero; ack is recorded independently. This does
            # not claim the physical chair stopped when a command link failed.
            self.target = (0.0, 0.0)
        if not self._observe(self.status()):
            self.state = 'FAULT'

    def key_event(self, key, pressed, now):
        self._now(now)
        if self.closed:
            raise FeedbackError('manual session is closed')
        if key == ' ':
            self._stop('OPERATOR_STOP')
            return
        if key not in {'w', 'a', 's', 'd'} or type(pressed) is not bool:
            self._stop('INVALID_KEY_EVENT', fault=True)
            raise FeedbackError('explicit WASD key event required')
        self.tick(now)
        if self.state != 'ARMED':
            return
        if pressed:
            self.keys.add(key)
        else:
            self.keys.discard(key)
        self.last_key = now
        linear = (int('w' in self.keys)-int('s' in self.keys))*self.linear
        angular = (int('a' in self.keys)-int('d' in self.keys))*self.angular
        self._send((linear, angular), 'OPERATOR_KEY_EVENT')

    def tick(self, now):
        self._now(now)
        if self.state != 'ARMED':
            return
        if not self.backend.available or self.last_link is None or now-self.last_link > self.link_timeout:
            self._stop('COMMAND_LINK_STALE', fault=True)
        elif self.last_key is None or now-self.last_key > self.key_timeout:
            self._stop('KEY_TIMEOUT')

    def close(self, now):
        self._now(now)
        self._stop('OPERATOR_EXIT', fault=self.state == 'FAULT')
        self.closed = True


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--mode', choices=('dry-run', 'real'), default='dry-run')
    parser.add_argument('--duration', type=float, default=60)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    bounded(args.duration, 'manual session duration', .1, 300)
    config = json.loads(args.config.read_text(encoding='utf-8'))
    controller = ManualController(config, mode=args.mode)
    if not sys.stdin.isatty():
        raise FeedbackError('manual WASD requires the user interactive terminal; no piped command playback')
    stopped = False

    def stop(signum, frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    with args.output.open('x', encoding='utf-8') as output:
        def observe(record):
            output.write(json.dumps({'monotonic_ns': time.monotonic_ns(), **record}, allow_nan=False)+'\n')
            output.flush()
            print(json.dumps(record, allow_nan=False), flush=True)
        controller.observer = observe
        previous = None
        try:
            if sys.platform != 'win32':
                import termios
                import tty
                previous = termios.tcgetattr(sys.stdin.fileno())
                tty.setcbreak(sys.stdin.fileno())
            print('DRY RUN ONLY. E: arm simulation; W/A/S/D: short motion intent; Space: disarm; Q: exit. No motor/ROS command output.', flush=True)
            print('Terminal keys have no release event: input expires after the configured key timeout; arm again after timeout.', flush=True)
            started = time.monotonic()
            while not stopped and time.monotonic()-started < args.duration:
                now = time.monotonic()
                controller.heartbeat(now)
                if sys.platform == 'win32':
                    import msvcrt
                    key = msvcrt.getwch() if msvcrt.kbhit() else None
                else:
                    readable, _, _ = select.select([sys.stdin], [], [], .02)
                    key = sys.stdin.read(1) if readable else None
                    if key == '':
                        controller.heartbeat(time.monotonic(), connected=False)
                        break
                now = time.monotonic()
                if key is not None:
                    key = key.lower()
                    if key in ('q', '\x03'):
                        break
                    if key == 'e':
                        controller.arm(now, operator_confirmed=True)
                    elif key in 'wasd ':
                        controller.key_event(key, True, now)
                controller.tick(now)
                time.sleep(.01)
        finally:
            controller.close(time.monotonic())
            if previous is not None:
                termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, previous)
    return 1 if controller.state == 'FAULT' else 0


if __name__ == '__main__':
    raise SystemExit(main())
