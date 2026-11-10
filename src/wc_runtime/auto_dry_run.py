"""Closed-loop dry run for window D: the wheel owner and its automatic control source on a simulated driver.

No serial port, ROS node or device is opened. The owner (MappingWheel), its channel (WheelChannel), the automatic
control source (AutoDrive) and auto.sock (AutoSocket) are the ones the vehicle runs; only the descriptor under the
channel is one end of a socket pair, answered by SimulatedDriver like a ZLAC8030D at slave 1. The RViz panel is
replaced by an interface heartbeat every 50 ms. The operator's "start automatic" is simulated by --start: ``once``
(until the first entry), ``always`` (again whenever automatic control is idle) or ``never``. The limits are the
first closed loop's: 0.10 m/s and 0.30 rad/s. The owner status carries ``dry_run: true``.

Run the base bridge with ``--dry-run`` and the same --project-root, --session-root and --session-id; it finds
auto.sock at ``auto_socket_path(project_root, session_root, session_id)``. The owner's journal is
<session-root>/sources/wheel_feedback.jsonl next to configuration/manual_runtime.json, the same layout and records as
a recording session, so wc_motion.auto_stop_report reads it too. Ctrl-C ends the run: the owner writes
zero and closes as on the vehicle.
"""
import argparse
import datetime
import json
import math
import os
from pathlib import Path
import select
import signal
import socket
import stat
import struct
import threading
import time

from wc_motion.feedback_transport import create_journal
from wc_motion.protocol import crc16
from .mapping_auto import AutoDrive, AutoSocket, auto_socket_path
from .mapping_wheel import FeedbackError, MappingWheel, WheelChannel

DEVICE_ID = 'ZLAC8030D-0000000014'   # the tracked placeholder controller of the tests; never a real one
LIMITS = {'max_linear_m_s': 0.10, 'max_angular_rad_s': 0.30, 'max_wheel_rpm': 27.0, 'max_linear_accel_m_s2': 0.30,
          'max_linear_decel_m_s2': 0.60, 'max_angular_accel_rad_s2': 0.80}
HEARTBEAT_S = 0.05   # the RViz panel's interface heartbeat
START_RETRY_S = 1.0
FEEDBACK_ADDRESS, SPEED_ADDRESS, CONTROL_WORD = 0x20AB, 0x2088, 0x200E


class SimulatedDriver:
    """Answers RTU frames on one end of a socket pair like a ZLAC8030D at slave 1; not the real register map.

    FC03 20ABh x2 returns the two wheel speeds. FC10 2088h x2 sets the speed targets, applied while the servo is
    enabled. FC06 200Eh: 8 enables, 7 releases (the targets are dropped and the wheels coast to zero); other FC06
    writes are acknowledged. The wheels follow their targets with a first-order lag of ``tau_s``; a feedback word is
    the command word times ``follow`` (the chair's command and feedback register scales). Any other or damaged frame
    gets no answer, so the owner times out as it would on the bus.
    """

    def __init__(self, sock, follow, *, tau_s=0.15, clock=time.monotonic):
        self.sock, self.follow, self.tau_s, self.clock = sock, follow, tau_s, clock
        self.enabled, self.target, self.speed, self.updated = False, (0, 0), [0.0, 0.0], clock()
        # The vehicle's emergency stop acts on the driver only: while it is pressed the wheels coast to zero
        # whatever is written; released, they follow the last target again (the case section 7.3 step 5 checks).
        self.estop = False
        self.buffer = bytearray()
        self.frames = []   # (function code, register) of every well-formed frame, for tests
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self._run, name='simulated-zlac8030d', daemon=True)

    def start(self):
        self.thread.start()
        return self

    def _advance(self):
        now = self.clock()
        share = 1.0 - math.exp(-max(0.0, now - self.updated) / self.tau_s)
        goal = [word * self.follow if self.enabled and not self.estop else 0.0 for word in self.target]
        self.speed = [value + (aim - value) * share for value, aim in zip(self.speed, goal)]
        self.updated = now

    def answer(self, frame):
        function, register = frame[1], struct.unpack('>H', frame[2:4])[0]
        self.frames.append((function, register))
        self._advance()
        if function == 3:
            if register != FEEDBACK_ADDRESS or struct.unpack('>H', frame[4:6])[0] != 2:
                return None
            payload = struct.pack('>BBBHH', 1, 3, 4, *[int(round(value)) & 0xffff for value in self.speed])
            return payload + crc16(payload)
        if function == 6:
            if register == CONTROL_WORD:
                value = struct.unpack('>H', frame[4:6])[0]
                if value == 7:
                    self.enabled, self.target = False, (0, 0)
                elif value == 8:
                    self.enabled = True
            return frame
        if function == 0x10 and register == SPEED_ADDRESS:
            self.target = struct.unpack('>hh', frame[7:11])
            return frame[:6] + crc16(frame[:6])
        return None

    def _frames(self):
        out = []
        while len(self.buffer) >= 8:
            length = 13 if self.buffer[1] == 0x10 else 8
            if len(self.buffer) < length:
                break
            frame = bytes(self.buffer[:length])
            del self.buffer[:length]
            if frame[0] != 1 or crc16(frame[:-2]) != frame[-2:]:
                self.buffer.clear()
                break
            out.append(frame)
        return out

    def _run(self):
        while not self.stopped.is_set():
            try:
                if not select.select([self.sock], [], [], 0.05)[0]:
                    continue
                data = self.sock.recv(256)
            except (OSError, ValueError):
                return
            if not data:
                return
            self.buffer.extend(data)
            for frame in self._frames():
                reply = self.answer(frame)
                if reply is not None:
                    try:
                        self.sock.sendall(reply)
                    except OSError:
                        return

    def close(self):
        self.stopped.set()
        self.thread.join(timeout=1.0)


class SimulatedLease:
    """What WheelChannel uses of FeedbackSerialLease: the descriptor, the failed flag and the configuration."""

    def __init__(self, fd):
        self.fd, self.failed = fd, False
        self.config = {'device_id': DEVICE_ID}
        self.host_configuration = {'device': 'socket pair to SimulatedDriver', 'access': 'DRY_RUN_NO_DEVICE'}


def owner_config(project_root, session_id):
    wheel = json.loads((Path(project_root) / 'config/wheel_history_calibration.json').read_text(encoding='utf-8'))
    return {'session_id': session_id, 'source_mode': 'dry_run', 'status': 'EXPERIMENT', 'duration_s': 0,
            'continuous_mapping': True, 'wheel_device_id': DEVICE_ID, 'feedback_timeout_s': 0.08,
            'wheel_candidate': dict(wheel, device_id=DEVICE_ID),
            'manual_controls': {'arm_allowed': True, 'interaction_policy': 'hybrid_manual',
                                'reason': 'dry run: simulated driver, no device',
                                'operation_evidence': {'status': 'USER_REPORTED_LEGACY_OPERATION',
                                                       'note': 'DRY RUN, simulated driver, no hardware'}}}


class DryRun:
    """The owner loop of mapping_wheel.main with the simulated driver and panel; ``tick`` is one loop pass."""

    def __init__(self, project_root, session_root, session_id, *, start='once', clock=time.monotonic):
        if start not in ('once', 'always', 'never'):
            raise FeedbackError('start must be once, always or never')
        self.start_mode, self.clock = start, clock
        self.session_root = Path(session_root).absolute()
        # The layout of a recording session: the stop report and other readers find the same files.
        for folder in (self.session_root, self.session_root / 'sources', self.session_root / 'configuration'):
            folder.mkdir(mode=0o700, parents=True, exist_ok=True)
        config = owner_config(project_root, session_id)
        (self.session_root / 'configuration/manual_runtime.json').write_text(
            json.dumps(config, indent=1, ensure_ascii=False) + '\n', encoding='utf-8')
        self.near, self.far = socket.socketpair()
        self.journal = create_journal(self.session_root / 'sources/wheel_feedback.jsonl', asynchronous=True)
        try:
            self.channel = WheelChannel(SimulatedLease(self.near.fileno()), self.journal)
            self.owner = MappingWheel(config, self.channel, self.journal)
            follow = 1 / (self.owner.control['command_rpm_to_register_scale'] * self.owner.wheel['register_to_wheel_rpm'])
            self.driver = SimulatedDriver(self.far, follow).start()
            self.auto = AutoDrive(self.owner, LIMITS, hold_to_run_switch='NOT_INSTALLED',
                                  readback=lambda request: [0], evidence={'stage': 'DRY_RUN'})
            self.auto.dry_run = True
            path = auto_socket_path(project_root, self.session_root, session_id)
            path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            info = path.parent.lstat()
            if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.geteuid() or stat.S_IMODE(info.st_mode) != 0o700:
                raise FeedbackError('control socket directory ownership/permissions invalid')
            self.socket_path = path
            self.auto_socket = AutoSocket(path, self.auto)
        except BaseException:
            self.journal.close()
            raise
        self.owner.before_control = self.auto_socket.pump
        self.owner.connected()   # the simulated RViz panel
        self.owner._log({'event': 'dry_run_started', 'auto_socket': str(path), 'limits': LIMITS,
                         'start': start, 'device': 'SIMULATED_DRIVER'})
        self.next_heartbeat = self.next_start = 0.0
        self.closed = False

    def _panel(self, kind, **fields):
        owner = self.owner
        owner.command({'type': kind, 'session_id': owner.session_id, 'sequence': owner.last_key_sequence + 1,
                       'arm_generation': owner.arm_generation, 'foreground': True, **fields})

    def tick(self):
        now = self.clock()
        self.auto_socket.pump()
        if now >= self.next_heartbeat:
            self.next_heartbeat = now + HEARTBEAT_S
            self._panel('keys', keys=[])
        if self.start_mode != 'never' and now >= self.next_start and self.auto.state == 'NONE' and \
                self.auto.bridge_connected and self.auto.decision is not None and \
                (self.start_mode == 'always' or self.auto.counters['entries'] == 0):
            self.next_start = now + START_RETRY_S
            self._panel('auto', enabled=True)
        self.owner.step()
        if self.owner.failure:
            raise FeedbackError(self.owner.failure)

    def line(self):
        status = self.auto.status()
        feedback = status['feedback'] or {}
        return ('%s %-9s %-34s v=%.3f w=%.3f fb=%.3f bridge=%s gate=%s%s' % (
            datetime.datetime.now().strftime('%H:%M:%S'), status['auto_state'], str(status['reason'])[:34],
            status['command']['linear'], status['command']['angular'], feedback.get('linear_m_s') or 0.0,
            'up' if status['bridge_connected'] else 'down', status['gate_state'],
            '  ' + status['refusal'] if status['refusal'] else ''))

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            self.auto_socket.close()
        finally:
            try:
                self.owner.close()   # writes zero through the channel, as on the vehicle
            finally:
                self.driver.close()
                self.near.close()
                self.far.close()
                self.journal.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--project-root', type=Path, required=True)
    parser.add_argument('--session-root', type=Path, required=True,
                        help='new or empty directory for this dry run; the journal is written there')
    parser.add_argument('--session-id', default='dryrun_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    parser.add_argument('--start', choices=('once', 'always', 'never'), default='once',
                        help='simulated operator start: until the first entry, whenever idle, or never')
    parser.add_argument('--duration', type=float, default=0.0, help='seconds; 0 runs until Ctrl-C')
    args = parser.parse_args(argv)
    if args.session_root.exists() and any(args.session_root.iterdir()):
        raise SystemExit('--session-root must be new or empty')
    stopped = []
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, lambda *_: stopped.append(True))
    run = DryRun(args.project_root.resolve(), args.session_root, args.session_id, start=args.start)
    print('dry run %s: auto.sock %s; start the bridge with --dry-run --project-root %s --session-root %s '
          '--session-id %s' % (args.session_id, run.socket_path, args.project_root.resolve(),
                               run.session_root, args.session_id), flush=True)
    failure, started, printed = None, time.monotonic(), 0.0
    try:
        while not stopped and (args.duration <= 0 or time.monotonic() - started < args.duration):
            run.tick()
            if time.monotonic() - printed >= 1.0:
                printed = time.monotonic()
                print(run.line(), flush=True)
            time.sleep(0.005)
    except Exception as error:  # noqa: BLE001 - reported below; close() still writes zero
        failure = str(error)
    finally:
        run.close()
    print(json.dumps({'state': 'FAILED' if failure else 'STOPPED', 'failure': failure,
                      'auto': run.auto.status()['counters'],
                      'journal': str(run.session_root / 'sources/wheel_feedback.jsonl')}, ensure_ascii=False), flush=True)
    return 1 if failure else 0


if __name__ == '__main__':
    raise SystemExit(main())
