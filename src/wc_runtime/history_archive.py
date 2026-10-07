"""Verified, reversible lossless archival of closed historical experiment files."""
import argparse
import hashlib
import json
import lzma
import os
from pathlib import Path
from .project_paths import project_root
import shutil
import subprocess
import time

from .source_archive import atomic_json, digest

RESERVE = 2 * 1024**3
ALLOWED = ('reports/maps', 'reports/single_mapping')


def sync_dir(path):
    fd = os.open(path, os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def sync_ancestry(path, root):
    """Persist newly created directory links up to the existing project root."""
    path,root=Path(path),Path(root)
    if not path.is_relative_to(root): raise ValueError('directory sync escaped project')
    while True:
        sync_dir(path)
        if path==root: break
        path=path.parent


def hash_decoded(path):
    value, count = hashlib.sha256(), 0
    with lzma.open(path, 'rb') as stream:
        while True:
            block = stream.read(1024**2)
            if not block:
                break
            count += len(block)
            value.update(block)
    return value.hexdigest(), count


def checked(root, relative):
    path = root / relative
    if Path(relative).is_absolute() or '..' in Path(relative).parts:
        raise ValueError('archive path must stay under project')
    if path.resolve() != path.absolute() or not path.resolve().is_relative_to(root):
        raise ValueError('symlink or escaping archive path')
    return path


def candidates(root):
    rows = []
    for base in ALLOWED:
        directory = checked(root, base)
        if not directory.exists():
            continue
        for path in directory.rglob('*'):
            if not path.is_file() or path.is_symlink() or path.suffix not in ('.db3', '.cdr', '.npz', '.jsonl'):
                continue
            relative = path.relative_to(root).as_posix()
            checked(root, relative)
            info = path.stat()
            if info.st_size < 1024**2:
                continue
            rows.append(dict(path=relative, bytes=info.st_size, mtime_ns=info.st_mtime_ns))
    return sorted(rows, key=lambda row: row['bytes'], reverse=True)


def open_project_files(root, approved_gaps=None):
    """Fail closed if an unrelated process's descriptors cannot be inspected."""
    opened = set()
    # fuser/lsof are not required. Same-user processes are the project owners;
    # kernel/system processes cannot own these user-exclusive device sessions.
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            if proc.stat().st_uid != os.getuid():
                continue
            for fd in (proc/'fd').iterdir():
                try:
                    target = fd.resolve(strict=True)
                except FileNotFoundError:
                    continue
                if target.is_relative_to(root):
                    opened.add(str(target))
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError:
            # Only the exact PID/name coverage exceptions explicitly accepted
            # by the user for this reviewed session. New gaps still fail closed.
            if not approved_gaps or approved_gaps.get(proc.name)!=(proc/'comm').read_text().strip():
                # Brief SSH probe processes can disappear during a scan. Wait
                # for that PID to cease existing; never waive a surviving gap.
                for _ in range(20):
                    if not proc.exists(): break
                    time.sleep(.1)
                if proc.exists(): raise
    return opened


def validate_review(root, review, *, configuration_root=None):
    """Only the explicitly reviewed, closed session is eligible for replacement."""
    session = checked(root,review['session'])
    if session.parent.relative_to(root).as_posix() not in ALLOWED:
        raise ValueError('review must identify exactly one historical session')
    if not review.get('closure_evidence') or not review.get('files'):
        raise ValueError('closure evidence and exact file hashes are required')
    for evidence in review['closure_evidence']:
        if digest(checked(root,evidence['path'])) != evidence['sha256']:
            raise ValueError('reviewed session closure evidence changed')
    for pid in review.get('owner_pids',[]):
        if Path('/proc',str(pid)).exists():
            raise ValueError('reviewed session owner PID exists; re-review required')
    for row in review['files']:
        source=checked(root,row['path'])
        if not source.is_relative_to(session) or not row.get('sha256') or not row.get('inode'):
            raise ValueError('review allowlist has missing identity or escapes the single session')
    for base in ('config','state'):
        for path in ((configuration_root or root)/base).rglob('*'):
            if path.is_file() and not path.is_symlink() and path.stat().st_size<=2*1024**2:
                if session.name.encode() in path.read_bytes():
                    raise ValueError('active configuration references reviewed session: '+str(path))
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name)==os.getpid(): continue
        try:
            if proc.stat().st_uid != os.getuid(): continue
            if str(session).encode() in (proc/'cmdline').read_bytes():
                raise ValueError('process command references reviewed session: '+proc.name)
        except (FileNotFoundError,ProcessLookupError): continue
    return review['files']


def verify_receipt(row, target, root, record):
    """A prior receipt is evidence only for this exact reviewed replacement."""
    if (record.get('path') != row['path']
            or record.get('archive_path') != target.relative_to(root).as_posix()
            or record.get('codec') != 'xz'
            or record.get('original_bytes') != row['bytes']
            or record.get('original_mtime_ns') != row['mtime_ns']
            or (row.get('sha256') and record.get('original_sha256') != row['sha256'])):
        raise ValueError('prior receipt differs from reviewed allowlist; recovery review required')
    if not target.is_file():
        raise ValueError('prior compressed archive missing; recovery review required')
    before = target.stat()
    if (before.st_size != record.get('archive_bytes')
            or digest(target) != record.get('archive_sha256')
            or hash_decoded(target) != (record.get('original_sha256'), record.get('original_bytes'))):
        raise ValueError('prior compressed archive verification failed; original retained if present')
    after = target.stat()
    if (before.st_size, before.st_mtime_ns, before.st_ino) != (after.st_size, after.st_mtime_ns, after.st_ino):
        raise ValueError('prior compressed archive changed during verification; recovery review required')


def verify_reviewed_source(root, row, expected_hash=None):
    """Repeat identity checks immediately before removing a closed original."""
    source = checked(root, row['path'])
    before = source.stat()
    identity = dict(bytes=before.st_size, mtime_ns=before.st_mtime_ns,
                    inode=before.st_ino, device=before.st_dev, nlink=before.st_nlink)
    if any(key in row and row[key] != value for key, value in identity.items()):
        raise ValueError('source identity differs from reviewed allowlist; original retained')
    current_hash = digest(source)
    if ((row.get('sha256') and current_hash != row['sha256'])
            or (expected_hash is not None and current_hash != expected_hash)):
        raise ValueError('source hash differs from reviewed allowlist; original retained')
    after = source.stat()
    if (before.st_size, before.st_mtime_ns, before.st_ino, before.st_dev, before.st_nlink) != (
            after.st_size, after.st_mtime_ns, after.st_ino, after.st_dev, after.st_nlink):
        raise ValueError('source changed during identity verification; original retained')
    return before, current_hash


def archive_one(root, row, receipt_dir, *, compress_only=False, guard=None, approved_gaps=None):
    if guard: guard()
    def opened():
        return open_project_files(root,approved_gaps) if approved_gaps else open_project_files(root)
    source = checked(root, row['path'])
    target = checked(root, 'data/history_archive/' + row['path'] + '.xz')
    receipt = receipt_dir / (hashlib.sha256(row['path'].encode()).hexdigest()+'.json')
    if receipt.exists():
        record = json.loads(receipt.read_text())
        if record.get('status') == 'ARCHIVED' and not source.exists():
            verify_receipt(row, target, root, record)
            return record
        if record.get('status') == 'VERIFIED_ORIGINAL_RETAINED':
            verify_receipt(row, target, root, record)
            verify_reviewed_source(root, row, record['original_sha256'])
            if str(source) in opened():
                raise ValueError('original changed/open; retained')
            if compress_only: return record
            if guard: guard()
            sync_ancestry(target.parent,root)
            sync_ancestry(receipt.parent,root)
            if digest(target) != record['archive_sha256'] or str(source) in opened():
                raise ValueError('archive changed or original opened before cleanup; original retained')
            verify_reviewed_source(root, row, record['original_sha256'])
            source.unlink();sync_dir(source.parent)
            record.update(status='ARCHIVED',released_bytes=record['original_bytes']-record['archive_bytes'])
            atomic_json(receipt,record)
            return record
    before = source.stat()
    if (before.st_size, before.st_mtime_ns) != (row['bytes'], row['mtime_ns']):
        raise ValueError('source changed since inventory: '+row['path'])
    if str(source) in opened():
        raise ValueError('source currently open: '+row['path'])
    if shutil.disk_usage(root).free < before.st_size + RESERVE + 16*1024**2:
        return dict(status='SKIPPED_TEMP_CAPACITY', path=row['path'])
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(target.name+'.partial')
    if target.exists():
        raise ValueError('archive target already exists without matching receipt; recovery review required: '+str(target))
    if str(temporary) in opened():
        raise ValueError('archive temporary is still being written: '+str(temporary))
    _, original_hash = verify_reviewed_source(root, row)
    recovered_partial=temporary.exists()
    if not recovered_partial:
        with temporary.open('xb') as output:
            result = subprocess.run(['xz', '-1', '-T2', '--stdout', '--', str(source)], stdout=output, check=False)
            output.flush()
            os.fsync(output.fileno())
        if result.returncode:
            raise RuntimeError('xz failed; source and partial retained')
    else:
        # A prior process may have stopped after compression but before receipt
        # creation. Reuse only after the full decoded hash check below passes.
        with temporary.open('rb') as stream: os.fsync(stream.fileno())
    if temporary.stat().st_size >= before.st_size * .98:
        temporary.unlink()  # Our just-created compressed temporary, never original data.
        return dict(status='SKIPPED_LOW_GAIN', path=row['path'])
    restored_hash, restored_bytes = hash_decoded(temporary)
    if (restored_hash, restored_bytes) != (original_hash, before.st_size):
        raise ValueError('archive round trip differs; original retained')
    after = source.stat()
    if (after.st_size, after.st_mtime_ns, after.st_ino) != (before.st_size, before.st_mtime_ns, before.st_ino):
        raise ValueError('source changed during compression; original retained')
    if str(source) in opened() or digest(source) != original_hash:
        raise ValueError('source changed or opened before cleanup; original retained')
    temporary.replace(target)
    sync_ancestry(target.parent,root)
    record = dict(status='VERIFIED_ORIGINAL_RETAINED', path=row['path'],
                  recovered_verified_partial=recovered_partial,
                  archive_path=target.relative_to(root).as_posix(), codec='xz',
                  original_sha256=original_hash, original_bytes=before.st_size,
                  archive_sha256=digest(target), archive_bytes=target.stat().st_size,
                  original_mode=before.st_mode & 0o777, original_mtime_ns=before.st_mtime_ns,
                  checked_unix_ns=time.time_ns())
    atomic_json(receipt, record)
    sync_ancestry(receipt.parent,root)
    if compress_only:
        return record
    if guard: guard()
    if digest(target) != record['archive_sha256'] or str(source) in opened():
        raise ValueError('archive changed or original opened before cleanup; original retained')
    verify_reviewed_source(root, row, original_hash)
    source.unlink()
    sync_dir(source.parent)
    record.update(status='ARCHIVED', released_bytes=before.st_size-target.stat().st_size)
    atomic_json(receipt, record)
    return record


def restore(root, record, *, guard=None):
    if guard: guard()
    source, target = checked(root, record['archive_path']), checked(root, record['path'])
    if target.exists():
        raise ValueError('restore never overwrites an existing file')
    if digest(source) != record['archive_sha256']:
        raise ValueError('stored archive hash mismatch')
    if shutil.disk_usage(root).free < record['original_bytes'] + RESERVE:
        raise ValueError('insufficient restore space; archive retained')
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(target.name+'.restoring')
    with lzma.open(source, 'rb') as reader, temporary.open('xb') as output:
        shutil.copyfileobj(reader, output, 1024**2)
        output.flush(); os.fsync(output.fileno())
    if digest(temporary) != record['original_sha256'] or temporary.stat().st_size != record['original_bytes']:
        raise ValueError('restored bytes differ; archive retained')
    os.chmod(temporary, record['original_mode'])
    os.utime(temporary, ns=(record['original_mtime_ns'], record['original_mtime_ns']))
    if guard: guard()
    temporary.replace(target)
    sync_ancestry(target.parent,root)
    return str(target)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=project_root())
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--review',type=Path,help='Exact reviewed allowlist for one closed historical session')
    parser.add_argument('--compress-only',action='store_true',help='Keep original even after verified compression')
    parser.add_argument('--restore-receipt', type=Path)
    args = parser.parse_args(argv)
    from .storage_policy import StoragePolicy
    code_root = args.root.resolve()
    storage = StoragePolicy(code_root)
    storage.check()
    root = storage.archive_root if storage.enabled else code_root
    if args.restore_receipt:
        receipt = storage.resolve(args.restore_receipt)
        print(restore(root, json.loads(receipt.read_text()), guard=storage.check))
        return 0
    review=json.loads(storage.resolve(args.review).read_text()) if args.review else None
    rows = validate_review(root,review,configuration_root=code_root) if review else candidates(root)
    if not args.apply:
        print(json.dumps(dict(files=len(rows), bytes=sum(row['bytes'] for row in rows), candidates=rows), indent=2))
        return 0
    if not review:
        raise ValueError('--apply requires a reviewed single-session allowlist; broad cleanup is disabled')
    report = storage.resolve('reports/v7_recording_20261006/storage')
    receipts = report/'receipts'
    receipts.mkdir(parents=True, exist_ok=True)
    atomic_json(report/'inventory.json', dict(files=rows, created_unix_ns=time.time_ns()))
    result = []
    def guard():
        storage.check()
        return validate_review(root,review,configuration_root=code_root)
    for row in rows:
        value = archive_one(root, row, receipts,compress_only=args.compress_only,
                            guard=guard,
                            approved_gaps=review.get('user_approved_process_gaps'))
        result.append(value)
        atomic_json(report/'progress.json', dict(status='RUNNING', results=result,
                    released_bytes=sum(r.get('released_bytes', 0) for r in result)))
        print(json.dumps({k:v for k,v in value.items() if k not in ('original_sha256','archive_sha256')}), flush=True)
    atomic_json(report/'progress.json', dict(status='COMPLETE', results=result,
                released_bytes=sum(r.get('released_bytes', 0) for r in result)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
