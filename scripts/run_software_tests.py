#!/usr/bin/env python3
"""Run target software tests; archive this run's POSIX scratch on configured storage."""
import argparse
import datetime
import getpass
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import signal
import stat
import subprocess
import sys
import tarfile
import tempfile
import uuid


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def tree_manifest(root):
    """Do not follow symlinks; unsupported special files prevent scratch removal."""
    result = {}
    for base, dirs, files in os.walk(root, followlinks=False):
        for name in sorted(dirs + files):
            path = Path(base) / name
            st = path.lstat()
            row = {'mode': stat.S_IMODE(st.st_mode)}
            if stat.S_ISLNK(st.st_mode):
                row.update(type='symlink', target=os.readlink(path))
            elif stat.S_ISDIR(st.st_mode):
                row.update(type='directory')
            elif stat.S_ISREG(st.st_mode):
                row.update(type='file', size=st.st_size, sha256=sha256(path))
            else:
                raise ValueError('unsupported scratch special file: ' + str(path))
            result[path.relative_to(root).as_posix()] = row
    return result


def archive_scratch(scratch, destination):
    expected = tree_manifest(scratch)
    partial = destination.with_suffix(destination.suffix + '.partial')
    with partial.open('xb') as raw:
        with tarfile.open(fileobj=raw, mode='w:gz', dereference=False) as archive:
            for name in sorted(expected):
                archive.add(scratch / name, arcname='scratch/' + name, recursive=False)
        raw.flush()
        os.fsync(raw.fileno())
    found = {}
    with tarfile.open(partial, 'r:gz') as archive:
        for member in archive:
            if not member.name.startswith('scratch/'):
                raise ValueError('unexpected archive member')
            name = member.name[len('scratch/'):]
            if name in found:
                raise ValueError('duplicate archive member')
            row = {'mode': member.mode}
            if member.isdir():
                row.update(type='directory')
            elif member.issym():
                row.update(type='symlink', target=member.linkname)
            elif member.isfile() or member.islnk():
                stream = archive.extractfile(member)
                digest = hashlib.sha256()
                size = 0
                for block in iter(lambda: stream.read(1024 * 1024), b''):
                    digest.update(block)
                    size += len(block)
                row.update(type='file', size=size, sha256=digest.hexdigest())
            else:
                raise ValueError('unsupported archive member')
            found[name] = row
    if found != expected or tree_manifest(scratch) != expected:
        raise ValueError('scratch/archive content mismatch or scratch changed during archive')
    os.replace(partial, destination)
    descriptor = os.open(destination.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return {'status': 'VERIFIED', 'path': str(destination), 'sha256': sha256(destination),
            'members': expected, 'verification': 'all regular bytes, symlink targets and modes; no extraction'}


def source_hashes(root):
    return {str(path.relative_to(root)): sha256(path)
            for folder in ('src', 'tests', 'config')
            for path in sorted((root / folder).rglob('*'))
            if path.is_file() and '__pycache__' not in path.parts and '.pytest_cache' not in path.parts}


def run_tests(project_root, pytest_args, *, pytest_prefix=None, check_target=True, storage_check=None):
    root = Path(project_root).resolve(strict=True)
    if check_target and (getpass.getuser() != 'nvidia' or platform.machine() != 'aarch64'):
        raise RuntimeError('Run on the target nvidia/aarch64 Orin; workstation tests are not target evidence.')
    environment = dict(os.environ, PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1',
                       ROS_DOMAIN_ID='89', ROS_LOCALHOST_ONLY='1')
    environment['PYTHONPATH'] = str(root / 'src') + (os.pathsep + environment['PYTHONPATH']
                                                  if environment.get('PYTHONPATH') else '')
    storage_command = [sys.executable, '-s', '-m', 'wc_runtime.storage_policy',
                       '--project-root', str(root), 'reports/test_runs']
    resolved = subprocess.run(storage_command, cwd=root, env=environment, check=True,
                              text=True, capture_output=True).stdout.strip()
    report_root = Path(resolved)
    if not report_root.is_absolute() or '\n' in resolved:
        raise ValueError('storage policy returned an invalid report path')
    if storage_check is None:
        # One policy object retains the initial mount/device identity for this run.
        # Reconstructing it at shutdown would silently accept a replacement mount.
        sys.path.insert(0, str(root / 'src'))
        from wc_runtime.storage_policy import StoragePolicy
        policy = StoragePolicy(root)
        if policy.resolve('reports/test_runs') != report_root:
            raise ValueError('storage configuration changed during runner initialization')
        storage_check = policy.check
    storage_checks = []
    def verify_storage(stage):
        storage_check()
        storage_checks.append(stage)
    verify_storage('INITIAL_BINDING')
    fingerprint = source_hashes(root)
    run_id = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex
    report = report_root / run_id
    report.mkdir(parents=True, exist_ok=False)
    # AF_UNIX paths include nested pytest fixture names; keep this component short.
    # The run manifest and inode/uid checks bind ownership, not the run-id prefix.
    scratch = Path(tempfile.mkdtemp(prefix='wct-', dir='/tmp'))
    scratch_identity = (scratch.stat().st_dev, scratch.stat().st_ino)
    environment.update(TMPDIR=str(scratch), TEMP=str(scratch), TMP=str(scratch),
                       MPLCONFIGDIR=str(report / 'matplotlib'), PYTHONPYCACHEPREFIX=str(scratch / 'pycache'))
    for name in ('matplotlib', 'pytest_cache'):
        (report / name).mkdir()
    command = list(pytest_prefix or [sys.executable, '-s', '-m', 'pytest']) + list(pytest_args or ['tests'])
    command += ['-v', '--junitxml=' + str(report / 'junit.xml'),
                '-o', 'cache_dir=' + str(report / 'pytest_cache'), '--basetemp=' + str(scratch / 'pytest')]
    record = {'schema_version': 1, 'run_id': run_id, 'project_root': str(root),
              'status': 'RUNNING', 'level': 'SYNTHETIC', 'command_argv': command,
              'storage_command_argv': storage_command, 'log': str(report / 'pytest.log'),
              'scratch': str(scratch), 'scratch_filesystem': 'system /tmp (POSIX)',
              'source_sha256': fingerprint, 'scratch_cleanup': 'NOT_ATTEMPTED',
              'ros_domain_id': 89, 'storage_checks': storage_checks}
    (report / 'run.json').write_text(json.dumps(record, indent=2), encoding='utf-8')
    print(json.dumps({'stage': 'SOFTWARE_TESTS', 'report': str(report), 'command_argv': command}), flush=True)
    test_rc = 1
    child = None
    try:
        with (report / 'pytest.log').open('xb') as log:
            child = subprocess.Popen(command, cwd=root, env=environment, stdout=log, stderr=subprocess.STDOUT)
            try:
                test_rc = child.wait()
            except KeyboardInterrupt:
                record['interrupted'] = True
                if child.poll() is None:
                    child.send_signal(signal.SIGINT)
                    try:
                        child.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        child.terminate()
                        try:
                            child.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            child.kill()
                            child.wait()
                test_rc = 130
            log.flush()
            os.fsync(log.fileno())
    except Exception as exc:
        record['test_error'] = repr(exc)
    finally:
        record['test_exit_code'] = test_rc
        record['status'] = 'PASS' if test_rc == 0 else 'FAIL'
        try:
            verify_storage('BEFORE_ARCHIVE')
            record['scratch_archive'] = archive_scratch(scratch, report / 'scratch.tar.gz')
            if scratch.is_symlink() or scratch.parent != Path('/tmp') or not scratch.name.startswith('wct-'):
                raise ValueError('owned scratch identity changed; retained')
            st = scratch.stat()
            if (st.st_dev, st.st_ino) != scratch_identity or st.st_uid != os.getuid():
                raise ValueError('owned scratch identity changed; retained')
            verify_storage('BEFORE_SCRATCH_REMOVAL')
            shutil.rmtree(scratch)
            record['scratch_cleanup'] = 'REMOVED_AFTER_VERIFIED_ARCHIVE'
        except Exception as exc:
            record['scratch_cleanup'] = 'RETAINED'
            record['archive_or_cleanup_error'] = repr(exc)
            record['status'] = 'ARTIFACT_FAILURE'
        record['exit_code'] = test_rc if record['status'] != 'ARTIFACT_FAILURE' else 74
        record['finished_utc'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
        try:
            verify_storage('BEFORE_FINAL_REPORT')
            with (report / 'run.json').open('w', encoding='utf-8') as output:
                json.dump(record, output, indent=2)
                output.flush()
                os.fsync(output.fileno())
        except Exception as exc:
            record.update(status='ARTIFACT_FAILURE', exit_code=74, report_persistence_error=repr(exc))
            # Preserve diagnostics beside this run's retained POSIX fixtures. Never
            # recreate the configured archive tree after a detach or remount.
            if scratch.is_dir() and not scratch.is_symlink() and (scratch.stat().st_dev, scratch.stat().st_ino) == scratch_identity:
                (scratch / 'runner_recovery.json').write_text(json.dumps(record, indent=2), encoding='utf-8')
            print(json.dumps({'stage': 'REPORT_PERSISTENCE_FAILED', 'scratch': str(scratch),
                              'error': repr(exc)}), file=sys.stderr, flush=True)
    print(json.dumps({'stage': 'SOFTWARE_TESTS_FINISHED', 'status': record['status'],
                      'report': str(report), 'exit_code': record['exit_code'],
                      'scratch_cleanup': record['scratch_cleanup']}), flush=True)
    return record['exit_code'], report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('pytest_args', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    selected = args.pytest_args[1:] if args.pytest_args[:1] == ['--'] else args.pytest_args
    return run_tests(args.project_root, selected)[0]


if __name__ == '__main__':
    raise SystemExit(main())
