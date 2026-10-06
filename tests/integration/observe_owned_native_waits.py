#!/usr/bin/env python3
"""Read-only /proc sampling of this session's native processing descendants."""
import argparse
import json
import os
from pathlib import Path
import signal
import time

from wc_runtime.cli import ROOT, target, name
from wc_runtime.component import process, descendants, belongs_to


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-name', type=name, required=True)
    parser.add_argument('--duration', type=float, default=230)
    args = parser.parse_args()
    if not 1 <= args.duration <= 300: raise ValueError('Bounded sampling required')
    target()
    output = ROOT/'reports/map_six_issues_20260914'/('native_waits_'+args.session_name+'.json')
    if output.exists(): raise RuntimeError('Existing evidence preserved')
    manifest = ROOT/'.phase1_runtime/sessions'/args.session_name/'mapping_app/manifest.json'
    stop = [False]
    for sig in (signal.SIGINT, signal.SIGTERM): signal.signal(sig, lambda *_: stop.__setitem__(0, True))
    started = time.monotonic(); records = []; errors = []; identities = {}
    while not stop[0] and time.monotonic()-started < args.duration:
        try:
            if not manifest.exists(): time.sleep(.5); continue
            value = json.loads(manifest.read_text())
            owner = process(value['supervisor_pid'])
            if owner is None or owner.start_ticks != value['supervisor_start_ticks']: break
            now = time.time_ns(); snapshot = []
            for child in descendants(owner):
                base = Path('/proc')/str(child.pid)
                if base.stat().st_uid != os.getuid() or not belongs_to(child, owner): continue
                argv = (base/'cmdline').read_bytes().split(b'\0')
                if not argv or Path(os.fsdecode(argv[0])).name not in ('icp_odometry', 'rtabmap'): continue
                identities[str(child.pid)] = {'start_ticks': child.start_ticks, 'executable': os.fsdecode(argv[0])}
                threads = []
                for folder in (base/'task').iterdir():
                    fields = (folder/'stat').read_text().rsplit(')', 1)[1].split()
                    threads.append({'tid': int(folder.name), 'state': fields[0],
                                    'wchan': (folder/'wchan').read_text().strip()})
                if belongs_to(child, owner): snapshot.append({'pid': child.pid, 'threads': threads})
            records.append({'wall_ns': now, 'monotonic_ns': time.monotonic_ns(), 'processes': snapshot})
        except (OSError, ValueError, KeyError) as error:
            if len(errors) < 30: errors.append(str(error))
        time.sleep(.5)
    result = {'scope': 'Read-only kernel wait-state samples, not a causal backtrace',
              'session_id': args.session_name, 'hardware_started': False,
              'identities': identities, 'records': records, 'sampling_errors': errors}
    with output.open('x') as stream: json.dump(result, stream)
    print(json.dumps({'samples': len(records), 'processes': identities, 'errors': errors}))


if __name__ == '__main__': main()
