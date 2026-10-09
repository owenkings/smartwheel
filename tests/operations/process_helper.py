"""Bounded synthetic process trees for supervisor tests; no ROS or device I/O."""

import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def record(path, role, token):
    data = Path(f'/proc/{os.getpid()}/stat').read_text()
    start = data[data.rindex(')')+2:].split()[19]
    value = {'pid': os.getpid(), 'start_ticks': start, 'role': role, 'token': token,
             'ppid': os.getppid(), 'pgid': os.getpgrp()}
    with Path(path).open('a', encoding='utf-8') as stream:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
        stream.write(json.dumps(value)+'\n')
        stream.flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('stay', 'spawn-stay', 'spawn-exit', 'exit-on-release', 'forwarder', 'signal-aware'), default='stay')
    parser.add_argument('--pid-log', required=True)
    parser.add_argument('--token', required=True)
    parser.add_argument('--role', default='component')
    parser.add_argument('--exit-code', type=int, default=0)
    parser.add_argument('--release')
    parser.add_argument('--grandchild-new-session', action='store_true')
    parser.add_argument('--ignore-int', action='store_true')
    parser.add_argument('--ignore-term', action='store_true')
    parser.add_argument('--signal-log')
    args, _ = parser.parse_known_args()
    stopped = False
    child = None
    interrupted = 0
    signal_exit_at = None

    def interrupt(signum, frame):
        nonlocal interrupted, signal_exit_at
        if args.mode == 'forwarder':
            # Separate delivery times expose duplicate parent+child broadcasts
            # reliably, rather than relying on Linux standard-signal coalescing.
            time.sleep(.15)
            if child is not None and child.poll() is None:
                child.send_signal(signal.SIGINT)
        elif args.mode == 'signal-aware':
            interrupted += 1
            signal_exit_at = time.monotonic()+.4
            with Path(args.signal_log).open('a', encoding='utf-8') as stream:
                fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
                stream.write(json.dumps({'pid': os.getpid(), 'token': args.token, 'sigint_count': interrupted})+'\n')
                stream.flush()
        else:
            stop(signum, frame)

    def stop(_signum, _frame):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGINT, signal.SIG_IGN if args.ignore_int else interrupt)
    signal.signal(signal.SIGTERM, signal.SIG_IGN if args.ignore_term else stop)
    record(args.pid_log, args.role, args.token)
    if args.mode.startswith('spawn-'):
        command = [sys.executable, str(Path(__file__).resolve()), '--mode', 'stay', '--role', 'grandchild',
                   '--pid-log', args.pid_log, '--token', args.token, '--ignore-int', '--ignore-term']
        subprocess.Popen(command, start_new_session=args.grandchild_new_session)
    elif args.mode == 'forwarder':
        if not args.signal_log:
            raise ValueError('forwarder test requires its own signal log')
        command = [sys.executable, str(Path(__file__).resolve()), '--mode', 'signal-aware', '--role', 'forwarded-child',
                   '--pid-log', args.pid_log, '--token', args.token, '--signal-log', args.signal_log]
        child = subprocess.Popen(command)
    deadline = time.monotonic()+90  # Hard upper bound even if the production supervisor fails.
    earliest_exit = time.monotonic()+.35
    while not stopped and time.monotonic() < deadline:
        if args.mode == 'forwarder' and child.poll() is not None:
            return child.returncode
        if args.mode == 'signal-aware' and signal_exit_at is not None and time.monotonic() >= signal_exit_at:
            return 0 if interrupted == 1 else 19
        if args.mode in ('spawn-exit', 'exit-on-release') and time.monotonic() >= earliest_exit and \
                (not args.release or Path(args.release).exists()):
            return args.exit_code
        time.sleep(.02)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
