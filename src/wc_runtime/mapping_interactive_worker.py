"""Owned, cancellable finalization of one already-stopped map, without devices."""
import argparse
import os
from pathlib import Path
import signal

from .mapping_interactive_control import read_document, atomic_document, ordinary
from .supervisor import ticks


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--control-directory', required=True, type=Path)
    parser.add_argument('--job', required=True)
    args = parser.parse_args(argv)
    directory = ordinary(args.control_directory, directory=True)
    owner, _ = read_document(directory/'owner.json')
    root = Path(owner['project_root'])
    if directory != root/'.phase1_runtime/mapping_windows'/owner['window_id']:
        raise ValueError('Foreign control directory')
    if ticks(owner['owner_pid']) != owner['owner_start_ticks']:
        raise ValueError('Window owner is no longer alive')
    if not args.job.isdigit():
        raise ValueError('Invalid save job identity')
    request, _ = read_document(directory/('save-'+args.job+'.json'))
    if any(request.get(k) != owner[k] for k in ('window_id', 'owner_pid', 'owner_start_ticks')):
        raise ValueError('Save owner mismatch')
    if request.get('job') != args.job:
        raise ValueError('Save job mismatch')
    result_path = directory/('save-'+args.job+'.result.json')
    if result_path.exists():
        raise ValueError('Save result already exists')
    def interrupted(_signum, _frame):
        raise KeyboardInterrupt
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, interrupted)
    from .mapping_app import load_backend
    from .map_save_app import session_handle, recover_session
    _, handle = session_handle(request['directory'], project_root=root)
    if handle['session_id'] != request['session_id'] or handle['mapping_enabled'] is not True:
        raise ValueError('Save session mismatch')
    result, code = recover_session(request['directory'], project_root=root, backend=load_backend())
    atomic_document(result_path, dict(schema_version=1, window_id=owner['window_id'], job=args.job,
                                      session_id=handle['session_id'], worker_pid=os.getpid(), result=result))
    return code


if __name__ == '__main__':
    raise SystemExit(main())
