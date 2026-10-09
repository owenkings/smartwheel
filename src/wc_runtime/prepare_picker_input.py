"""Capture four static lidar stages, or prepare an explicit completed study offline."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import uuid

from wc_calibration.picker import PickerSession, _json_loads
from .storage_policy import resolve_storage_path, logical_path
from .picker_state import POINTER, pointer_lock, migrate_pointer_locked

STAGES = {'dual_A': ('left', 'right'), 'left_single': ('left',),
          'right_single': ('right',), 'dual_B': ('left', 'right')}


def project_path(root, value):
    """Resolve code locally and migrated data on the verified configured volume."""
    return resolve_storage_path(root, value)


def read_json(root, path, limit=10_000_000):
    path = project_path(root, path)
    if not path.is_file() or path.stat().st_size > limit:
        raise ValueError('Missing or oversized JSON: '+str(path))
    raw = path.read_bytes()
    if len(raw) > limit:
        raise ValueError('JSON grew beyond its byte budget')
    return _json_loads(raw.decode('utf-8'))


def write_new(path, data):
    with path.open('xb') as output:
        output.write(data)
        output.flush()
        os.fsync(output.fileno())


def json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode('utf-8')


def completed_study(root, value):
    study = project_path(root, value)
    if study.parent != project_path(root, 'reports/lidar_stability') or not study.is_dir() or \
            re.fullmatch(r'[A-Za-z0-9_-]{1,20}_\d{8}T\d{6}Z', study.name) is None:
        raise ValueError('Use an explicit reports/lidar_stability/SCENE_YYYYMMDDTHHMMSSZ directory')
    stages = {}
    for label, sides in STAGES.items():
        audit_path = study/label/'capture.json'
        audit = read_json(root, audit_path)
        if not isinstance(audit, dict) or audit.get('status') != 'CAPTURE_AUDIT_PASS' or \
                audit.get('errors') != [] or set(audit.get('sides', {})) != set(sides):
            raise ValueError(label+': four successful capture audits are required')
        if not isinstance(audit.get('session'), str) or not audit['session']:
            raise ValueError(label+': explicit capture session is required')
        for side in sides:
            row = audit['sides'][side]
            if type(row.get('paired')) is not int or row['paired'] < 20:
                raise ValueError(label+'/'+side+': insufficient paired frames')
            for endpoint in ('first', 'last'):
                if row.get(endpoint, {}).get('flags', {}).get('device_config_policy') != 'preserve_current':
                    raise ValueError(label+'/'+side+': preserve_current metadata is required')
            for filename in (side+'.npz', side+'_frames.json'):
                if not project_path(root, study/label/filename).is_file():
                    raise ValueError(label+': missing '+filename)
        # The existing recorder writes this only after its own stop command succeeds.
        after = read_json(root, study/(label+'_after.json'))
        if not isinstance(after, dict) or not isinstance(after.get('registered_live_processes'), list):
            raise ValueError(label+': missing post-stop resource snapshot')
        if any('/sessions/'+audit['session']+'/' in str(row.get('manifest', ''))
               for row in after['registered_live_processes']):
            raise ValueError(label+': capture session still had registered live processes after stop')
        stages[label] = {'session': audit.get('session'),
            'paired': {side: audit['sides'][side]['paired'] for side in sides},
            'capture_sha256': hashlib.sha256(audit_path.read_bytes()).hexdigest()}
    final = read_json(root, study/'after.json')
    if not isinstance(final, dict) or not isinstance(final.get('registered_live_processes'), list):
        raise ValueError('Missing final resource snapshot')
    return study, stages


def study_from_output(root, output, scene_name):
    matches = re.findall(r'^STATIC_COMPARISON_COMPLETE ([^\r\n]+)$', output, re.MULTILINE)
    if len(matches) != 1:
        raise ValueError('Recorder did not return exactly one completion directory; latest pointer is never used')
    study = project_path(root, matches[0])
    if study.parent != project_path(root, 'reports/lidar_stability') or \
            re.fullmatch(re.escape(scene_name)+r'_\d{8}T\d{6}Z', study.name) is None:
        raise ValueError('Recorder completion directory does not match this invocation')
    return study


def capture_study(root, scene_name, log_path):
    """Run only the existing recorder; forward interrupts to its owned group."""
    recorder = project_path(root, 'scripts/record_lidar_comparison.sh')
    project_path(root, 'reports/lidar_stability').mkdir(parents=True, exist_ok=True)
    command = ['bash', str(recorder), scene_name,
        'Operator-requested scene '+scene_name+'; rig, mounts and scene must remain unchanged. '
        'Static status is an assumption, not independently verified.']
    interrupted, previous = [], {}
    child, forwarded = None, False
    def request_stop(signum, _frame):
        nonlocal forwarded
        if not interrupted:
            interrupted.append((signum, time.monotonic()))
        if child is not None and not forwarded and child.poll() is None:
            try:
                os.killpg(child.pid, signal.SIGINT)
                forwarded = True
            except ProcessLookupError:
                pass
    try:
        # Install before Popen: an interrupt during spawn is remembered and forwarded afterwards.
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            previous[sig] = signal.signal(sig, request_stop)
        with log_path.open('xb') as log:
            if interrupted:
                raise RuntimeError('Recorder interrupted before spawn; no capture started')
            child = subprocess.Popen(command, cwd=root, stdout=log, stderr=subprocess.STDOUT,
                                     start_new_session=True)
            if interrupted:
                request_stop(interrupted[0][0], None)
            started = time.monotonic()
            while child.poll() is None:
                if not interrupted and time.monotonic()-started > 900:
                    request_stop(signal.SIGINT, None)
                if interrupted and time.monotonic()-interrupted[0][1] > 110:
                    raise RuntimeError('Recorder cleanup wait expired; inspect owned PID '+str(child.pid)+
                                       ' and '+str(log_path)+'; default input remains unchanged')
                try:
                    child.wait(timeout=.25)
                except subprocess.TimeoutExpired:
                    pass
        if interrupted or child.returncode != 0:
            raise RuntimeError('Recorder interrupted or failed (exit '+str(child.returncode)+'); see '+str(log_path))
    except BaseException:
        if child is not None and child.poll() is None:
            request_stop(signal.SIGINT, None)
            # Preserve the original 110-second cleanup deadline on exceptional paths too.
            while child.poll() is None and time.monotonic()-interrupted[0][1] <= 110:
                try:
                    child.wait(timeout=.25)
                except subprocess.TimeoutExpired:
                    pass
        raise
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    if log_path.stat().st_size > 8_000_000:
        raise ValueError('Recorder output exceeds its byte budget')
    return study_from_output(root, log_path.read_text(encoding='utf-8', errors='replace'), scene_name)


def run_preparer(root, study, output, log_path):
    with log_path.open('xb') as log:
        result = subprocess.run([sys.executable, '-s', '-m', 'wc_calibration.prepare_stability_input',
            '--project-root', str(root), '--study', str(study),
            '--output', str(output)], cwd=root, stdout=log, stderr=subprocess.STDOUT, timeout=120)
    if result.returncode:
        raise RuntimeError('Existing preparation script failed; see '+str(log_path))


def prepare_input(root, study_value, *, preparer=run_preparer):
    """No hardware access. Validate and journal everything before replacing the pointer."""
    with pointer_lock(root):
        migrate_pointer_locked(root)
        return _prepare_input_locked(root, study_value, preparer=preparer)


def _prepare_input_locked(root, study_value, *, preparer):
    root = Path(root).absolute()
    study, stages = completed_study(root, study_value)
    operation = 'prepare_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:12]
    base = project_path(root, Path('reports/calibration')/study.name)
    directory = project_path(root, base/operation)
    directory.mkdir(parents=True, exist_ok=False)
    pointer = project_path(root, POINTER)
    pointer.parent.mkdir(parents=True, exist_ok=True)
    temporary = project_path(root, pointer.parent/('.point_picker_input_'+operation+'.tmp'))
    result_path = directory/'result.json'
    try:
        # Exclusive writes preserve earlier prepared inputs, including an existing base file.
        output = project_path(root, base/'prepared.json')
        if output.exists():
            output = directory/'prepared.json'
        preparer(root, study, output, directory/'preparation.log')
        project_path(root, output)
        session = PickerSession(output, project_path(root, 'data/calibration/manual_points'))
        scene = session.scene()
        prepared = read_json(root, output, 128_000_000)
        if scene['source_mode'] != 'real' or scene['time_quality'] != 'NON_SIMULTANEOUS_STATIC_SCENE_ASSUMPTION' or \
                prepared.get('input_preparation', {}).get('study_path') != str(study.resolve()):
            raise ValueError('Prepared data does not belong to this explicit real study')
        digest = hashlib.sha256(output.read_bytes()).hexdigest()
        old = None
        if pointer.exists():
            if not pointer.is_file() or pointer.stat().st_size > 16384:
                raise ValueError('Existing pointer is not a bounded regular file')
            old = pointer.read_bytes()
            write_new(directory/'previous_point_picker_input.json', old)
        new_pointer = {'schema_version': 1, 'status': 'READY_FOR_OFFLINE_PICKING',
            'prepared_input': logical_path(root, output), 'prepared_file_sha256': digest,
            'scene_id': scene['scene_id'], 'live_eligible': False, 'time_quality': scene['time_quality'],
            'operation_id': operation, 'preparation_result': logical_path(root, result_path)}
        result = {'schema_version': 1, 'status': 'PREPARED_VALIDATED', 'operation_id': operation,
            'study': str(study), 'prepared_input': str(output), 'prepared_file_sha256': digest,
            'scene_id': scene['scene_id'], 'point_counts': scene['point_counts'], 'stages': stages,
            'device_config_policy': 'preserve_current', 'policy_evidence': 'capture first/last raw metadata in each stage',
            'live_eligible': False, 'time_quality': scene['time_quality'],
            'previous_pointer_backup': str(directory/'previous_point_picker_input.json') if old is not None else None,
            'proposed_pointer': new_pointer, 'pointer_commit_evidence': str(pointer),
            'commit_note': 'This journal precedes atomic commit. Matching operation_id in the pointer is commit evidence; '
                           'a later operation may legitimately replace that pointer.',
            'picker_command': 'bash scripts/pick_lidar_points.sh --input '+new_pointer['prepared_input']}
        write_new(temporary, json_bytes(new_pointer))
        write_new(result_path, json_bytes(result))
        # Detect another writer while this operation was being prepared. Never use a latest-study pointer.
        project_path(root, pointer)
        if (pointer.read_bytes() if pointer.exists() else None) != old:
            raise RuntimeError('Default pointer changed concurrently; prepared files retained, no pointer update')
        if hashlib.sha256(output.read_bytes()).hexdigest() != digest:
            raise RuntimeError('Prepared input changed before commit')
        os.replace(temporary, pointer)  # Final required mutation: any earlier failure leaves the old pointer intact.
        return dict(result, status='READY_FOR_OFFLINE_PICKING', result_json=str(result_path), pointer_updated=True)
    except BaseException as error:
        try:
            write_new(directory/'failure.json', json_bytes({'status': 'FAILED_NO_POINTER_UPDATE',
                'error': type(error).__name__, 'detail': str(error), 'study': str(study)}))
        except OSError:
            pass
        raise
    finally:
        if temporary.exists():
            temporary.unlink()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene_name', nargs='?', default=None, help='ASCII letters/digits/_/-, 1..20; default manual')
    parser.add_argument('--prepare-only', type=Path, metavar='STUDY', help='Explicit completed study; no sensor access')
    args = parser.parse_args(argv)
    if args.prepare_only is not None and args.scene_name is not None:
        parser.error('--prepare-only does not take a scene name')
    scene_name = args.scene_name or 'manual'
    if re.fullmatch('[A-Za-z0-9_-]{1,20}', scene_name) is None:
        parser.error('scene_name must be 1..20 ASCII letters, digits, underscores or hyphens')
    from .cli import ROOT, target
    target()
    if Path.cwd().resolve() != ROOT:
        raise RuntimeError('Run from the selected project root')
    operation = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:12]
    directory = project_path(ROOT, Path('reports/point_picker_operations')/operation)
    directory.mkdir(parents=True, exist_ok=False)
    try:
        study = args.prepare_only
        if study is None:
            print('四阶段采集约需 3～4 分钟，请保持轮椅与场景静止；日志：'+str(directory/'capture.log'), flush=True)
            study = capture_study(ROOT, scene_name, directory/'capture.log')
        result = prepare_input(ROOT, study)
        print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)
        print('选点数据已准备；未启动浏览器。请运行：\n'+result['picker_command'], flush=True)
        return 0
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        write_new(directory/'failure.json', json_bytes({'status': 'FAILED', 'error': type(error).__name__,
            'detail': str(error), 'requested_study': str(args.prepare_only) if args.prepare_only else None,
            'requested_scene': scene_name if args.prepare_only is None else None}))
        print(json.dumps({'status': 'FAILED', 'detail': str(error)}, ensure_ascii=False), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
