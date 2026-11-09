"""Base bridge (docs/architecture/velocity_channel.md, sections 3 and 9).

Runs in the navigation domain (ROS_DOMAIN_ID=100, localhost only). Subscribes the safety gate's decisions on
``/wc_nav/gate_command`` and hands each one to the wheel serial owner over the session's ``auto.sock``; publishes the
owner's status on ``/wc_nav/base_status`` and derives ``/wc_nav/enable`` (only while the hold-to-run switch is held)
and ``/wc_nav/estop`` from it. It never opens a serial port, never publishes in the panel domain, does not change any
velocity and does not reconnect: when the owner's socket goes away the bridge exits. Starting it requires today's
user approval ``WC_AUTO_DRIVE_APPROVAL=APPROVED_BY_USER_<YYYYMMDD>``; without an available automatic control source
in the owner there is no socket to connect to.
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


class BridgeError(RuntimeError):
    """The owner's socket is gone or misbehaves; the bridge stops."""


class BridgeCore:
    """Socket side of the bridge, without ROS. ``clock_ns`` must be the same monotonic clock as the gate's mono_ns."""

    def __init__(self, session_id, bridge_id, *, clock_ns=time.monotonic_ns):
        self.session_id, self.bridge_id, self.clock_ns = session_id, bridge_id, clock_ns
        self.sock = None
        self.sequence = 0
        self.pending, self.received = bytearray(), bytearray()
        self.status = self.status_ns = None
        self.counters = {name: 0 for name in ('gate_commands', 'gate_rejected', 'gate_stale', 'forwarded', 'status',
                                              'status_rejected')}

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
        """Forward one gate decision; malformed, stale or future-dated ones are dropped, never repaired."""
        now = self.clock_ns()
        self.counters['gate_commands'] += 1
        try:
            gate = parse_gate_command(text)
        except ProtocolError:
            self.counters['gate_rejected'] += 1
            return False
        if gate['mono_ns'] > now + FUTURE_TOLERANCE_NS or (now - gate['mono_ns']) * 1e-9 > DECISION_STALE_S:
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

    def enable(self):
        """True only while a fresh owner status reports the hold-to-run switch held and no emergency stop."""
        return self.fresh() and self.status.get('dead_man') is True and self.status.get('estop') is False

    def estop(self):
        return self.status.get('estop') if self.fresh() else None

    def base_status(self):
        age = self.status_age_s()
        return {'schema': 'wc_base_status_relay_v1', 'session_id': self.session_id, 'bridge_id': self.bridge_id,
                'connected': self.sock is not None, 'owner_status_age_s': None if age is None else round(age, 3),
                'owner_status_stale': not self.fresh(), 'reason': None if self.fresh() else 'BASE_STATUS_STALE',
                'enable': self.enable(), 'owner': self.status,
                'counters': dict(self.counters)}

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
        status_pub = node.create_publisher(String, '/wc_nav/base_status', 10)
        enable_pub = node.create_publisher(Bool, '/wc_nav/enable', 10)
        estop_pub = node.create_publisher(Bool, '/wc_nav/estop', 10)
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        last_estop = [None]

        def guarded(action):
            def run(*args):
                if failure:
                    return
                try:
                    action(*args)
                except BridgeError as error:
                    failure.append(str(error))
            return run

        node.create_subscription(String, '/wc_nav/gate_command', guarded(lambda m: core.on_gate_command(m.data)), latest)
        node.create_timer(0.02, guarded(core.pump))
        node.create_timer(0.1, guarded(lambda: status_pub.publish(String(data=json.dumps(core.base_status())))))

        def switches():
            if core.enable():
                enable_pub.publish(Bool(data=True))   # no message when released: the gate's enable goes stale
            estop = core.estop()
            if estop is not None and estop != last_estop[0]:
                estop_pub.publish(Bool(data=estop))
                last_estop[0] = estop
        node.create_timer(0.25, guarded(switches))
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
