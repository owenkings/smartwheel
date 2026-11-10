"""Start and stop automatic control from the command line until the RViz panel has its buttons (velocity_channel.md).

    python3 -m wc_runtime.auto_operator --project-root ~/wheelchair start|stop|status|session

start: asks the wheel owner of the running recording to start automatic control. Typing the command took the focus
away from the RViz window, and automatic control only runs while RViz has it: click into the RViz window within
10 s. The start is then checked like the panel's "start automatic" (the bridge or the test source connected, the
wheels standing still, the gate not latched, no keys held, ...). The command prints the state until AUTO or a
refusal. Stop with Space or WASD in RViz, the emergency stop, or ``stop`` here; clicking out of RViz (into this
terminal, for example) already stops automatic control at once.
stop: the panel's "stop automatic": decelerate to zero and leave automatic control; also cancels a waiting start.
status: the owner's automatic control state in one line.
session: prints ``--session-root <dir> --session-id <id>`` of the running recording, for the base bridge's command
line: ``python3 -m wc_motion.auto_bridge --project-root ~/wheelchair $(python3 -m wc_runtime.auto_operator
--project-root ~/wheelchair session)``.

The socket is operator.sock next to the session's manual.sock and auto.sock (same user only). Only this command
uses it: the base bridge and navigation never request automatic control. --session-root and --session-id select a
session explicitly, for example the dry run of wc_runtime.auto_dry_run.
"""
import argparse
import json
from pathlib import Path
import socket
import time

from wc_motion.auto_protocol import ProtocolError, encode_line

from .mapping_auto import OPERATOR_WAIT_S, operator_socket_path, recording_sessions, running_auto_sessions

ANSWER_BYTES = 65536
POLL_S = 0.2


def ask(path, session_id, message, *, timeout_s=2.0):
    """One request on operator.sock; returns the owner's automatic control status."""
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(timeout_s)
        client.connect(str(path))
        client.sendall(encode_line(dict(message, session_id=session_id)))
        data = bytearray()
        while not data.endswith(b'\n'):
            chunk = client.recv(ANSWER_BYTES)
            if not chunk:
                raise ProtocolError('the wheel owner closed the operator socket without an answer')
            data.extend(chunk)
            if len(data) > ANSWER_BYTES:
                raise ProtocolError('operator answer too long')
    value = json.loads(bytes(data))
    if not isinstance(value, dict) or value.get('type') != 'operator_status' or value.get('session_id') != session_id \
            or not isinstance(value.get('status'), dict):
        raise ProtocolError('unexpected operator answer')
    return value['status']


def line(status):
    text = 'auto %s (%s)' % (status.get('auto_state'), status.get('reason'))
    if status.get('refusal'):
        text += ', refused: ' + str(status['refusal'])
    if status.get('operator_pending'):
        text += ', waiting for a click into the RViz window'
    if status.get('start_pending'):
        text += ', waiting for the gate to unlatch'
    text += ', bridge or test source %s' % ('connected' if status.get('bridge_connected') else 'not connected')
    feedback = status.get('feedback') or {}
    if feedback.get('linear_m_s') is not None:
        text += ', wheels %.3f m/s' % feedback['linear_m_s']
    limits = status.get('limits') or {}
    text += ', limits %s m/s %s rad/s, evidence %s' % (limits.get('max_linear_m_s'), limits.get('max_angular_rad_s'),
                                                     status.get('evidence_stage'))
    return text


def attach_reason(directory, *, lines=5000):
    """The wheel owner's account of its automatic control source (state: reason), from the start of its journal."""
    try:
        with (Path(directory) / 'sources/wheel_feedback.jsonl').open(encoding='utf-8') as stream:
            for number, text in enumerate(stream):
                if number >= lines:
                    break
                if '"auto_attached"' in text:
                    value = json.loads(text)
                    return '%s: %s' % (value.get('state'), value.get('reason'))
    except (OSError, ValueError):
        pass
    return None


def session(args):
    root = args.project_root.resolve()
    if (args.session_root is None) != (args.session_id is None):
        raise SystemExit('give --session-root and --session-id together, or neither')
    if args.session_root is not None:
        return root, args.session_root.absolute(), args.session_id
    sessions = running_auto_sessions(root)
    if len(sessions) != 1:
        hint = ''
        running = [row for row in recording_sessions(root) if row[2] == 'RECORDING']
        if not sessions and running:
            reason = attach_reason(running[0][0])
            hint = '; the wheel owner of %s reports %s' % (running[0][1], reason or (
                'nothing about automatic control: is auto_controls in the hardware setup?'))
        elif not sessions:
            hint = '; no recording of this checkout is running'
        raise SystemExit('expected one running recording with automatic control, found %d%s'
                         % (len(sessions), hint))
    return (root, *sessions[0])


def start(path, session_id, *, clock=time.monotonic, sleep=time.sleep, out=print):
    status = ask(path, session_id, {'type': 'auto_request', 'enabled': True})
    if status.get('operator_pending'):
        out('click into the RViz window now (within %.0f s): automatic control runs only while RViz has the focus'
            % OPERATOR_WAIT_S, flush=True)
    deadline, last = clock() + OPERATOR_WAIT_S + 5.0, None
    while True:
        text = line(status)
        if text != last:
            out(time.strftime('%H:%M:%S ') + text, flush=True)
            last = text
        state = status.get('auto_state')
        if state == 'AUTO':
            return 0
        if state in ('STOPPING', 'FAULT', 'UNAVAILABLE') or (
                state == 'NONE' and status.get('refusal') and not status.get('operator_pending')
                and not status.get('start_pending')):
            return 1
        if clock() > deadline:
            out('automatic control did not start', flush=True)
            return 1
        sleep(POLL_S)
        status = ask(path, session_id, {'type': 'status'})


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command', choices=('start', 'stop', 'status', 'session'))
    parser.add_argument('--project-root', type=Path, required=True)
    parser.add_argument('--session-root', type=Path, help='default: the one running recording with automatic control')
    parser.add_argument('--session-id')
    args = parser.parse_args(argv)
    root, session_root, session_id = session(args)
    if args.command == 'session':
        print('--session-root %s --session-id %s' % (session_root, session_id), flush=True)
        return 0
    path = operator_socket_path(root, session_root, session_id)
    try:
        if args.command == 'start':
            return start(path, session_id)
        status = ask(path, session_id, {'type': 'auto_request', 'enabled': False} if args.command == 'stop'
                     else {'type': 'status'})
        print(line(status), flush=True)
        return 0
    except (OSError, ProtocolError, ValueError) as error:
        raise SystemExit('no answer from the wheel owner on %s: %s' % (path, error))


if __name__ == '__main__':
    raise SystemExit(main())
