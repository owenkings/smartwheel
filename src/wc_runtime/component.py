"""Own a component's descendant tree, including when its supervisor dies.

Linux subreaping retains double-forked/setsid descendants. Cleanup uses pidfds
and current ancestry, never process-name matching or a global process signal.
"""

import argparse
import ctypes
from dataclasses import dataclass
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


DEFAULT_SIGINT_GRACE_S = 5.0
RECORD_SIGINT_GRACE_S = 30.0
MAX_SIGINT_GRACE_S = 120.0  # Mapping may drain eMMC after acquisition has stopped.
TERM_GRACE_S = 3.0
KILL_GRACE_S = 2.0


def validate_sigint_grace(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= MAX_SIGINT_GRACE_S:
        raise ValueError(f'SIGINT grace must be finite and between 0 and {MAX_SIGINT_GRACE_S:g} seconds')
    return float(value)


@dataclass(frozen=True)
class Process:
    pid: int
    parent: int
    start_ticks: str
    state: str


def process(pid):
    try:
        value = Path(f'/proc/{pid}/stat').read_text()
        fields = value[value.rindex(')')+2:].split()
        return Process(pid, int(fields[1]), fields[19], fields[0])
    except (FileNotFoundError, ProcessLookupError):
        return None


def descendants(owner):
    """Find candidates; ancestry is checked again after pidfd_open before use."""
    snapshot = {}
    for path in Path('/proc').glob('[0-9]*'):
        try:
            record = process(int(path.name))
        except PermissionError:
            continue  # Unreadable unrelated processes cannot be candidates.
        if record is not None:
            snapshot[record.pid] = record
    owned = {owner.pid}
    pending = [owner.pid]
    by_parent = {}
    for record in snapshot.values():
        by_parent.setdefault(record.parent, []).append(record)
    result = []
    while pending:
        for record in by_parent.get(pending.pop(), ()):
            if record.pid not in owned:
                owned.add(record.pid)
                pending.append(record.pid)
                result.append(record)
    return result


def belongs_to(record, owner):
    """Reject stale snapshots/PID reuse and recheck each parent link."""
    chain = []
    seen = set()
    current = process(record.pid)
    if current is None or current.start_ticks != record.start_ticks:
        return False
    while current.pid != owner.pid:
        if current.pid in seen or current.parent <= 1:
            return False
        seen.add(current.pid)
        chain.append(current)
        current = process(current.parent)
        if current is None:
            return False
    if current.start_ticks != owner.start_ticks:
        return False
    # A parent may die/reparent while /proc is scanned. Such candidates wait
    # for the next scan, when the still-live wrapper has adopted the orphan.
    for previous in reversed(chain):
        observed = process(previous.pid)
        if observed is None or (observed.start_ticks, observed.parent) != (previous.start_ticks, previous.parent):
            return False
    return True


def signal_descendant(record, owner, signum):
    descriptor = None
    try:
        descriptor = os.pidfd_open(record.pid)
        if not belongs_to(record, owner):
            return False
        signal.pidfd_send_signal(descriptor, signum)
        if record.state in {'T', 't'} and signum != signal.SIGKILL:
            signal.pidfd_send_signal(descriptor, signal.SIGCONT)
        return True
    except (FileNotFoundError, ProcessLookupError):
        return False
    finally:
        if descriptor is not None:
            os.close(descriptor)


def reap(child):
    """Reap adopted orphans without losing the direct component exit status."""
    if child is not None:
        child.poll()
    while True:
        try:
            pid, status = os.waitpid(-1, os.WNOHANG)
        except ChildProcessError:
            return
        if pid == 0:
            return
        if child is not None and pid == child.pid:
            child.returncode = os.waitstatus_to_exitcode(status)


def cleanup(owner, child, sigint_grace_s=DEFAULT_SIGINT_GRACE_S):
    # Remain alive through SIGKILL so setsid descendants stay adopted here and
    # zombies can be reaped. Never SIGKILL the wrapper's own process group.
    sigint_grace_s = validate_sigint_grace(sigint_grace_s)
    errors = set()
    for signum, seconds in ((signal.SIGINT, sigint_grace_s), (signal.SIGTERM, TERM_GRACE_S), (signal.SIGKILL, KILL_GRACE_S)):
        sent = set()
        deadline = time.monotonic()+seconds
        while True:
            reap(child)
            remaining = descendants(owner)
            if not remaining:
                reap(child)
                if not descendants(owner):
                    return True
            for record in remaining:
                key = (record.pid, record.start_ticks)
                if record.state in {'Z', 'X'} or key in sent:
                    continue
                # A launcher forwards SIGINT to its own children. Broadcasting
                # simultaneously to those children can interrupt destructors
                # twice and lose the closed-database barrier. Notify only the
                # direct command first; TERM/KILL later cover every survivor,
                # including orphaned children that called setsid.
                if signum == signal.SIGINT and (child is None or record.pid != child.pid):
                    continue
                try:
                    if signal_descendant(record, owner, signum):
                        sent.add(key)
                except OSError as exc:
                    message = f'component cleanup pid={record.pid}: {exc}'
                    if message not in errors:
                        print(message, file=sys.stderr, flush=True)
                        errors.add(message)
            if time.monotonic() >= deadline:
                break
            time.sleep(.05)
    reap(child)
    remaining = descendants(owner)
    if remaining:
        print('component cleanup incomplete: '+repr([(p.pid, p.start_ticks, p.state) for p in remaining]),
              file=sys.stderr, flush=True)
    return not remaining


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--parent', type=int, required=True)
    parser.add_argument('--sigint-grace-s', type=float, default=DEFAULT_SIGINT_GRACE_S)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    args.sigint_grace_s = validate_sigint_grace(args.sigint_grace_s)
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command:
        raise ValueError('component command is empty')
    if not hasattr(os, 'pidfd_open') or not hasattr(signal, 'pidfd_send_signal'):
        raise RuntimeError('component ownership requires Linux pidfd support')
    stopped = False

    def stop(signum, frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(1, signal.SIGTERM, 0, 0, 0) != 0:  # PR_SET_PDEATHSIG
        raise OSError(ctypes.get_errno(), 'prctl PDEATHSIG')
    if libc.prctl(36, 1, 0, 0, 0) != 0:  # PR_SET_CHILD_SUBREAPER
        raise OSError(ctypes.get_errno(), 'prctl CHILD_SUBREAPER')
    if os.getppid() != args.parent or stopped:
        return 1
    if os.getpgrp() != os.getpid():
        raise RuntimeError('component wrapper must own its new process group')
    owner = process(os.getpid())
    child = None
    rc = None
    clean = False
    try:
        child = subprocess.Popen(command, stdin=subprocess.DEVNULL)
        while not stopped:
            reap(child)
            rc = child.returncode
            if rc is not None:
                break
            time.sleep(.05)
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        clean = cleanup(owner, child, args.sigint_grace_s)
    if not clean:
        return 1
    if rc is None and child is not None:
        rc = child.returncode  # Preserve failures while writing the final bag.
    return rc if rc is not None else 0


if __name__ == '__main__':
    raise SystemExit(main())
