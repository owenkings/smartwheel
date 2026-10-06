#!/usr/bin/env python3
"""Review or restore only this upgrade's exact files. Never git reset or delete evidence."""
import argparse
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import stat
import tarfile
import time
import sys

try:
    import fcntl
except ImportError:  # A read-only preview is also useful on a copied Windows tree.
    fcntl = None

ROOT = Path('/home/nvidia/wheelchair')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from wc_runtime.storage_policy import StoragePolicy
SOURCE_PREFIXES = {'src', 'scripts', 'config', 'tests', 'docs'}
FIXED_LOCKS = (
    'sensor_owner.lock', 'domain-83-source.lock', 'domain-83-processing.lock',
    'domain-83-encoder.lock', 'domain-83-cameras.lock', 'domain-83-wheel.lock',
    'mapping-app.lock', 'single-mapping.lock', 'domain-84-offline-review.lock',
    'domain-89-geometry-review.lock', 'heavy_build.lock',
)


def _unlinked(path):
    path = Path(path).absolute()
    for item in (path, *path.parents):
        if item.is_symlink() or getattr(item, 'is_junction', lambda: False)():
            raise ValueError('linked path refused: '+str(item))
    return path


def _relative(value):
    if not isinstance(value, str) or '\\' in value:
        raise ValueError('canonical project-relative path required')
    path = PurePosixPath(value)
    if path.is_absolute() or str(path) != value or not path.parts or \
            any(part in ('.', '..') for part in path.parts) or path.parts[0] not in SOURCE_PREFIXES:
        raise ValueError('unreviewed path '+value)
    return value


def _sha(value):
    if not isinstance(value, str) or len(value) != 64 or any(c not in '0123456789abcdef' for c in value):
        raise ValueError('invalid SHA-256 in deployment/baseline manifest')
    return value


def _document(path):
    path = _unlinked(path)
    if not path.is_file():
        raise ValueError('regular manifest file required: '+str(path))
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError('duplicate manifest key '+key)
            result[key] = value
        return result
    value = json.loads(path.read_text(encoding='utf-8'), object_pairs_hook=unique)
    if not isinstance(value, dict):
        raise ValueError('manifest object required: '+str(path))
    return value


def _verify_current(relative, item):
    path = _unlinked(ROOT/relative)
    if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256']:
        raise ValueError('changed since deployment; reconcile before rollback: '+relative)
    return path


def _sync_directory(path):
    if os.name == 'posix':
        descriptor = os.open(path, os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0))
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


def _atomic_json(path, value):
    temporary = path.with_name(path.name+'.new')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    _sync_directory(path.parent)


def _lock_project(stack):
    if fcntl is None:
        raise RuntimeError('--apply requires the project Linux host and flock')
    directory = _unlinked(ROOT/'.phase1_runtime/locks')
    directory.mkdir(parents=True, exist_ok=True)
    names = set(FIXED_LOCKS)
    # Session, serial, camera-port, viewer-port and nondefault comparison-domain locks.
    names.update(path.name for path in directory.iterdir() if path.name.endswith('.lock'))
    for name in sorted(names):
        path = _unlinked(directory/name)
        descriptor = os.open(path, os.O_CREAT | os.O_RDWR | getattr(os, 'O_NOFOLLOW', 0), 0o600)
        stream = stack.enter_context(os.fdopen(descriptor, 'a+'))
        if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
            raise ValueError('regular lock required: '+str(path))
        try:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError('project is busy; stop the owner before rollback: '+name) from error


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--apply', action='store_true', help='Default is a read-only rollback preview')
    args = parser.parse_args(argv)
    storage = StoragePolicy(ROOT)
    report = _unlinked(storage.resolve(args.report)).resolve(strict=True)
    if not report.is_dir() or not report.is_relative_to(storage.resolve('reports')):
        raise ValueError('reviewed project report directory required')
    state = _document(report/'deployed_state.json')
    baseline = _document(report/'baseline/source_manifest.json')
    archive_path = _unlinked(report/'baseline/source.tar.gz')
    if not archive_path.is_file():
        raise ValueError('regular baseline archive required')
    for relative, item in state.items():
        _relative(relative)
        if not isinstance(item, dict):
            raise ValueError('deployment file entry required: '+relative)
        _sha(item.get('sha256'))
        if relative in baseline:
            original = baseline[relative]
            if not isinstance(original, dict):
                raise ValueError('baseline file entry required: '+relative)
            _sha(original.get('sha256'))
            if type(original.get('mode')) is not int or not 0 <= original['mode'] <= 0o177777 or \
                    stat.S_IFMT(original['mode']) not in (0, stat.S_IFREG):
                raise ValueError('integer permission mode required: '+relative)
    with ExitStack() as stack:
        if args.apply:
            _lock_project(stack)
        archive = stack.enter_context(tarfile.open(archive_path))
        members = {}
        for member in archive.getmembers():
            if member.name in state and member.name in baseline:
                if member.name in members or not member.isfile():
                    raise ValueError('unique regular baseline member required: '+member.name)
                members[member.name] = member
        payload = {}
        for relative, item in state.items():
            _verify_current(relative, item)
            if relative in baseline:
                if relative not in members:
                    raise ValueError('missing baseline member '+relative)
                stream = archive.extractfile(members[relative])
                if stream is None:
                    raise ValueError('missing baseline content '+relative)
                with stream:
                    data = stream.read()
                if hashlib.sha256(data).hexdigest() != baseline[relative]['sha256']:
                    raise ValueError('baseline hash mismatch '+relative)
                payload[relative] = data
        print(json.dumps({'status':'REVIEWED_ROLLBACK', 'files':len(state),
                          'restore_existing':len(payload), 'quarantine_new':len(state)-len(payload),
                          'apply':args.apply, 'independent_user_changes':'REFUSE_OVERWRITE'}))
        if not args.apply:
            return 0
        # Restoring code uses same-filesystem atomic moves. Backups are read
        # from USB, but this rollback safety journal must remain with the code.
        retention = ROOT/'.phase1_runtime/rollback_retained' if storage.enabled else report
        retention = _unlinked(retention)
        retention.mkdir(parents=True, exist_ok=True)
        quarantine = retention/('rollback_retained_'+str(time.time_ns()))
        quarantine.mkdir()
        _sync_directory(retention)
        if quarantine.stat().st_dev != ROOT.stat().st_dev:
            raise ValueError('rollback retention must share the project filesystem for atomic rename')
        journal = {'status':'STAGING', 'files':list(state), 'completed':[], 'active_file':None,
                   'retained_new_files':[relative for relative in state if relative not in payload]}
        try:
            _atomic_json(quarantine/'rollback_status.json', journal)
            # Finish every old-file write before moving any deployed file. A full disk
            # during preparation therefore leaves the running source untouched.
            for relative, data in payload.items():
                prepared = quarantine/'.baseline_ready'/relative
                prepared.parent.mkdir(parents=True, exist_ok=True)
                with prepared.open('xb') as stream:
                    stream.write(data)
                    os.chmod(prepared, stat.S_IMODE(baseline[relative]['mode']))
                    stream.flush()
                    os.fsync(stream.fileno())
                _sync_directory(prepared.parent)
            for relative, item in state.items():
                _verify_current(relative, item)
            journal['status'] = 'RESTORING'
            for relative, item in state.items():
                path = _verify_current(relative, item)
                retained = quarantine/relative
                retained.parent.mkdir(parents=True, exist_ok=True)
                journal['active_file'] = relative
                _atomic_json(quarantine/'rollback_status.json', journal)
                os.replace(path, retained)
                _sync_directory(path.parent)
                _sync_directory(retained.parent)
                if relative in payload:
                    try:
                        os.replace(quarantine/'.baseline_ready'/relative, path)
                    except BaseException:
                        # The prior atomic move retained the complete deployed file.
                        # Put it back if replacement failed, rather than leaving a gap.
                        if not path.exists():
                            os.replace(retained, path)
                            _sync_directory(path.parent)
                        raise
                    _sync_directory(path.parent)
                journal['completed'].append(relative)
                journal['active_file'] = None
                _atomic_json(quarantine/'rollback_status.json', journal)
            journal['status'] = 'SOURCE_RESTORED_REBUILD_REQUIRED'
            _atomic_json(quarantine/'rollback_status.json', journal)
        except BaseException as error:
            journal['status'] = 'INTERRUPTED_DO_NOT_START_PROJECT'
            journal['error'] = type(error).__name__+': '+str(error)
            try:
                _atomic_json(quarantine/'rollback_status.json', journal)
            except OSError:
                pass  # Preserve the last durable journal and all retained files.
            raise
        print('Source restored. Retained files and journal: '+str(quarantine))
        print('Rebuild wc_bringup wc_xt_driver wc_camera_panel before next use; all acquisition evidence retained.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
