"""Base bridge (docs/architecture/velocity_channel.md, sections 3 and 9).

Runs in the navigation domain (ROS_DOMAIN_ID=100, localhost only). Subscribes the safety gate's decisions on
``/wc_nav/gate_command`` and hands each one to the wheel serial owner over the session's ``auto.sock``; publishes the
owner's status on ``/wc_nav/base_status`` (10 Hz) and derives ``/wc_nav/enable`` and ``/wc_nav/estop`` from it, as
agreed with window D on 2026-11-09:

- enable: ``true`` at 10 Hz while a fresh owner status reports the hold-to-run switch held and no emergency stop;
  one ``false`` the moment a fresh status reports it released (or the emergency stop pressed), then nothing; nothing
  while the owner status is stale or the switch input is missing, so the gate's enable times out on its own.
  With the switch not installed (``enable_source: OPERATOR_START``) the same rules follow the owner's
  ``enable_request``: true from the operator's accepted "start automatic" until any stop.
- estop: ``true`` at 10 Hz while pressed, one ``false`` when released; reliable and transient local.

A malformed gate command (unknown schema, missing or extra field, non-finite number) or one stamped in the future is
a fault: the bridge stops and the owner, seeing the socket close, ends automatic control. A stale gate command
(older than 0.3 s) is dropped. The bridge never opens a serial port, never publishes in the panel domain, does not
change any velocity and does not reconnect. Starting it requires today's user approval
``WC_AUTO_DRIVE_APPROVAL=APPROVED_BY_USER_<YYYYMMDD>``; without an available automatic control source in the owner
there is no socket to connect to.
"""
import argparse
import datetime
import os
from pathlib import Path
import socket
import time
import uuid

from .auto_protocol import (DECISION_STALE_S, STATUS_STALE_S, ProtocolError, decision_packet, decode_line,
                            encode_line, hello_packet, parse_gate_command, parse_status)

NAV_DOMAIN = '100'
APPROVAL_ENV = 'WC_AUTO_DRIVE_APPROVAL'
FUTURE_TOLERANCE_NS = 50_000_000
RELAY_SCHEMA = 'wc_base_status_relay_v1'
SWITCH_PERIOD_S = 0.1          # enable heartbeat and estop repetition, same rate as base_status


class BridgeError(RuntimeError):
    """The owner's socket is gone or misbehaves, or the gate sent something that must not be acted upon."""


class BridgeCore:
    """Socket side of the bridge, without ROS. ``clock_ns`` must be the same monotonic clock as the gate's mono_ns."""

    def __init__(self, session_id, bridge_id, *, clock_ns=time.monotonic_ns):
        self.session_id, self.bridge_id, self.clock_ns = session_id, bridge_id, clock_ns
        self.sock = None
        self.sequence = 0
        self.pending, self.received = bytearray(), bytearray()
        self.status = self.status_ns = None
        self.enable_state = self.estop_state = None   # what the bridge last told the gate
        self.counters = {name: 0 for name in ('gate_commands', 'gate_stale', 'forwarded', 'status', 'status_rejected',
                                              'enable_true', 'enable_false', 'estop_true', 'estop_false')}

    def connect(self, path, timeout=1.0):
        client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            client.settimeout(timeout)
            client.connect(str(path))
            client.setblocking(False)
        except BaseException:
            client.close()
            raise
        self.sock = client
        self._queue(hello_packet(self.session_id, self.bridge_id))
        self.flush()

    def _queue(self, value):
        data = encode_line(value)
        if len(self.pending) + len(data) > 65536:
            raise BridgeError('wheel owner is not reading decisions')
        self.pending.extend(data)

    def flush(self):
        if self.sock is None:
            raise BridgeError('not connected to the wheel owner')
        while self.pending:
            try:
                sent = self.sock.send(self.pending)
            except BlockingIOError:
                return
            except OSError as error:
                raise BridgeError('wheel owner socket failed: ' + str(error)) from error
            if sent <= 0:
                raise BridgeError('wheel owner socket closed')
            del self.pending[:sent]

    def on_gate_command(self, text):
        """Forward one gate decision. Malformed or future-dated: fault. Older than 0.3 s: dropped, never repaired."""
        now = self.clock_ns()
        self.counters['gate_commands'] += 1
        try:
            gate = parse_gate_command(text)
        except ProtocolError as error:
            raise BridgeError('gate command rejected: ' + str(error)) from error
        if gate['mono_ns'] > now + FUTURE_TOLERANCE_NS:
            raise BridgeError('gate command stamped %.3f s in the future: not the same monotonic clock'
                              % ((gate['mono_ns'] - now) * 1e-9))
        if (now - gate['mono_ns']) * 1e-9 > DECISION_STALE_S:
            self.counters['gate_stale'] += 1
            return False
        self.sequence += 1
        self._queue(decision_packet(gate, self.session_id, self.sequence, now))
        self.counters['forwarded'] += 1
        self.flush()
        return True

    def pump(self):
        self.flush()
        while True:
            try:
                data = self.sock.recv(8192)
            except BlockingIOError:
                break
            except OSError as error:
                raise BridgeError('wheel owner socket failed: ' + str(error)) from error
            if not data:
                raise BridgeError('wheel owner closed the automatic control socket')
            self.received.extend(data)
            if len(self.received) > 65536:
                raise BridgeError('wheel owner status overflow')
        while b'\n' in self.received:
            line, _, rest = self.received.partition(b'\n')
            self.received[:] = rest
            try:
                self.status = parse_status(decode_line(bytes(line)), self.session_id)
                self.status_ns = self.clock_ns()
                self.counters['status'] += 1
            except ProtocolError:
                self.counters['status_rejected'] += 1

    def status_age_s(self):
        return None if self.status_ns is None else (self.clock_ns() - self.status_ns) * 1e-9

    def fresh(self):
        age = self.status_age_s()
        return age is not None and age <= STATUS_STALE_S

    def switches(self):
        """(dead_man, estop) from a fresh owner status; None for each value that is unknown."""
        if not self.fresh():
            return None, None
        return self.status.get('dead_man'), self.status.get('estop')

    def enable_decision(self):
        """True (enable), False (released or stopped: say so once) or None (unknown: say nothing)."""
        if not self.fresh():
            return None
        status = self.status
        dead_man, estop = status.get('dead_man'), status.get('estop')
        if estop is True:
            return False
        if status.get('enable_source') == 'OPERATOR_START':
            return status.get('enable_request') is True
        if dead_man is True and estop is False:
            return True
        return False if dead_man is False else None

    def enable(self):
        """True only while a fresh owner status says the gate may be enabled (switch held, or operator start)."""
        return self.enable_decision() is True

    def switch_outputs(self, heartbeat):
        """Messages for this tick as (``'enable'`` or ``'estop'``, value); ``heartbeat`` marks the 10 Hz tick.

        Called on every pump for the immediate transitions and every 0.1 s with ``heartbeat`` for the repetitions."""
        out = []
        enabled = self.enable_decision()
        if enabled is True:
            if heartbeat or self.enable_state is not True:
                out.append(('enable', True))
        elif enabled is False and self.enable_state is not False:
            out.append(('enable', False))       # released or stopped: tell the gate at once, then stay silent
        self.enable_state = enabled             # None: unknown, publish nothing, the gate's enable goes stale
        _, estop = self.switches()
        if estop is True:
            if heartbeat or self.estop_state is not True:
                out.append(('estop', True))
            self.estop_state = True
        elif estop is False and self.estop_state is not False:
            out.append(('estop', False))        # released, or the first known state: keeps transient local current
            self.estop_state = False
        for name, value in out:
            self.counters['%s_%s' % (name, 'true' if value else 'false')] += 1
        return out

    def base_status(self):
        """``/wc_nav/base_status``: the owner's status plus what the gate acts on (``base_auto``, ``reason``)."""
        age = self.status_age_s()
        fresh = self.fresh()
        owner = self.status if fresh else None
        base_auto = owner is not None and owner.get('auto_state') == 'AUTO'
        reason = None if base_auto else ('BASE_NOT_AUTO' if fresh else 'BASE_STATUS_STALE')
        dead_man, estop = self.switches()
        return {'schema': RELAY_SCHEMA, 'stamp_mono_ns': self.clock_ns(), 'session_id': self.session_id,
                'bridge_id': self.bridge_id, 'connected': self.sock is not None,
                'owner_status_age_s': None if age is None else round(age, 3), 'owner_status_stale': not fresh,
                'base_auto': base_auto, 'reason': reason,
                'source': None if owner is None else owner.get('source'),
                'auto_state': None if owner is None else owner.get('auto_state'),
                'enable_source': None if owner is None else owner.get('enable_source'),
                'enable_request': None if owner is None else owner.get('enable_request'),
                'dead_man': dead_man, 'estop': estop, 'enable': self.enable(),
                'feedback': None if owner is None else owner.get('feedback'),
                'owner': self.status, 'counters': dict(self.counters)}

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None


def check_environment(environ, today):
    """Refuse outside the navigation domain or without today's explicit user approval."""
    if environ.get('ROS_DOMAIN_ID') != NAV_DOMAIN or environ.get('ROS_LOCALHOST_ONLY') != '1' or \
            environ.get('ROS_DISCOVERY_SERVER'):
        raise SystemExit('the base bridge runs only with ROS_DOMAIN_ID=%s and ROS_LOCALHOST_ONLY=1' % NAV_DOMAIN)
    if environ.get(APPROVAL_ENV) != 'APPROVED_BY_USER_' + today.strftime('%Y%m%d'):
        raise SystemExit('the base bridge needs %s=APPROVED_BY_USER_<today, YYYYMMDD>' % APPROVAL_ENV)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--project-root', type=Path, required=True)
    parser.add_argument('--session-root', type=Path, required=True, help='directory of the running panel session')
    parser.add_argument('--session-id', required=True)
    parser.add_argument('--bridge-id', default='bridge_' + uuid.uuid4().hex[:12])
    args = parser.parse_args(argv)
    check_environment(os.environ, datetime.date.today())
    from wc_runtime.mapping_auto import auto_socket_path
    core = BridgeCore(args.session_id, args.bridge_id)
    core.connect(auto_socket_path(args.project_root, args.session_root, args.session_id))
    import json
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String
    rclpy.init(args=[])
    node = rclpy.create_node('wc_auto_bridge')
    failure = []
    try:
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        status_pub = node.create_publisher(String, '/wc_nav/base_status', 10)
        publishers = {'enable': node.create_publisher(Bool, '/wc_nav/enable', 10),
                      'estop': node.create_publisher(Bool, '/wc_nav/estop', latched)}

        def guarded(action):
            def run(*args):
                if failure:
                    return
                try:
                    action(*args)
                except BridgeError as error:
                    failure.append(str(error))
            return run

        def publish_switches(heartbeat):
            for name, value in core.switch_outputs(heartbeat):
                publishers[name].publish(Bool(data=value))

        def fast():
            core.pump()
            publish_switches(False)

        def slow():
            publish_switches(True)
            status_pub.publish(String(data=json.dumps(core.base_status(), allow_nan=False)))

        node.create_subscription(String, '/wc_nav/gate_command', guarded(lambda m: core.on_gate_command(m.data)), latest)
        node.create_timer(0.02, guarded(fast))
        node.create_timer(SWITCH_PERIOD_S, guarded(slow))
        while rclpy.ok() and not failure:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        core.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if failure:
        print(json.dumps({'state': 'STOPPED', 'failure': failure[0], 'counters': core.counters}), flush=True)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
