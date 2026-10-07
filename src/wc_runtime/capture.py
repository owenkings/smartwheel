"""Independent V7 capture; optional user-operated driving, never EKF or SLAM.

Without --manual-drive this remains a read-only acquisition tool. The explicit
mode authorizes this session's human keyboard/hand-push operation through one
wheel serial owner, which also journals/publishes every feedback transaction.
"""
import argparse
import copy
from contextlib import ExitStack
import json
import os
from pathlib import Path
import shutil
import signal
import stat
import subprocess
import sys
import time

from .source_archive import atomic_json, digest

RESERVE_BYTES = 2*1024**3
MANUAL_READY_TIMEOUT_S = 20
ALL_SOURCE_READY_TIMEOUT_S = 30
SOFTWARE_SNAPSHOT_BUDGET_BYTES = 256*1024**2
PROFILES = ('mapping_core', 'mapping_cameras', 'all_sensors')
CAMERA_PROFILES = ('mapping_cameras', 'all_sensors')
MEMORY_STAGING_ROOT = Path('/dev/shm/wc_capture')
MEMORY_RESERVE_BYTES = 4*1024**3
RECORDER_ITEM_ESTIMATE_BYTES = 2*1024**2


def memory_budget(estimated_recording_bytes, recorder_queue_size, *, camera_profile=None,
                  tmpfs_free_bytes, available_memory_bytes, total_memory_bytes):
    """Conservative acquisition estimate, not a hardware throughput guarantee."""
    from .source_archive import CameraArchive
    camera_items = getattr(CameraArchive, 'DEFAULT_MAX_ITEMS', 256)
    camera_queue = (4*camera_items*camera_profile['width']*camera_profile['height']*3
                    if camera_profile else 0)
    recorder_queue = recorder_queue_size*RECORDER_ITEM_ESTIMATE_BYTES
    # Reliable DDS history, SDK buffers, IMU/journal queues and Python overhead.
    other_queues = 2*1024**3
    queue_bytes = camera_queue+recorder_queue+other_queues
    reserve = max(MEMORY_RESERVE_BYTES, total_memory_bytes//10)
    required_memory = estimated_recording_bytes+queue_bytes+reserve
    required_tmpfs = estimated_recording_bytes+RESERVE_BYTES
    sufficient = tmpfs_free_bytes>=required_tmpfs and available_memory_bytes>=required_memory
    startup_memory = queue_bytes+reserve+SOFTWARE_SNAPSHOT_BUDGET_BYTES
    startup_tmpfs = RESERVE_BYTES+SOFTWARE_SNAPSHOT_BUDGET_BYTES
    startup_allowed = tmpfs_free_bytes>startup_tmpfs and available_memory_bytes>startup_memory
    return dict(status=('AVAILABLE' if sufficient else 'WARNING') if startup_allowed else 'BLOCKED',
        sufficient=sufficient, startup_allowed=startup_allowed,
        forecast_policy='ADVISORY_ONLY', startup_required_memory_bytes=startup_memory,
        startup_required_tmpfs_bytes=startup_tmpfs,
        estimated_recording_bytes=estimated_recording_bytes, camera_queue_items_per_source=camera_items,
        camera_queue_estimated_bytes=camera_queue, recorder_queue_estimated_bytes=recorder_queue,
        recorder_item_estimate_bytes=RECORDER_ITEM_ESTIMATE_BYTES, other_queue_overhead_estimated_bytes=other_queues,
        bounded_queue_estimated_bytes=queue_bytes, memory_reserve_bytes=reserve,
        tmpfs_reserve_bytes=RESERVE_BYTES, tmpfs_free_bytes=tmpfs_free_bytes,
        available_memory_bytes=available_memory_bytes, required_memory_bytes=required_memory,
        required_tmpfs_bytes=required_tmpfs,
        estimate_only=True, runtime_capacity_checks=True)


def host_memory():
    values = {}
    for line in Path('/proc/meminfo').read_text().splitlines():
        name, _, value = line.partition(':')
        if name in ('MemTotal', 'MemAvailable'): values[name] = int(value.split()[0])*1024
    if len(values)!=2: raise ValueError('MemAvailable/MemTotal required for memory staging')
    return values


def memory_preflight(checks, recorder_queue_size, profile):
    # /dev/shm must be the actual tmpfs mount, never a similarly named disk dir.
    mount = MEMORY_STAGING_ROOT.parent
    mount_ok = any(row.split(' - ',1)[1].split()[0]=='tmpfs' and row.split()[4]==str(mount)
                   for row in Path('/proc/self/mountinfo').read_text().splitlines() if ' - ' in row)
    if not mount_ok or mount.is_symlink(): raise ValueError('memory staging requires verified /dev/shm tmpfs')
    memory = host_memory()
    camera = checks.get('camera_configuration', {}).get('capture_profile') if profile in CAMERA_PROFILES else None
    if profile in CAMERA_PROFILES and camera is None: camera = dict(width=320,height=240,capture_fps=30)
    return memory_budget(checks['capacity']['estimated_recording_bytes'], recorder_queue_size,
        camera_profile=camera, tmpfs_free_bytes=shutil.disk_usage(mount).free,
        available_memory_bytes=memory['MemAvailable'], total_memory_bytes=memory['MemTotal'])


def checked_staging_path(session):
    if not isinstance(session,str) or not session or Path(session).name!=session or session in ('.','..') or '/' in session or '\\' in session:
        raise ValueError('one explicit session path component required')
    root = MEMORY_STAGING_ROOT.absolute()
    path = root/session
    if root != root.resolve() or any(p.is_symlink() for p in (path,*path.parents)):
        raise ValueError('memory staging path must be absolute and unlinked')
    if path.resolve().parent!=root.resolve(): raise ValueError('memory staging path escapes session root')
    return path


def prepare_staging_directory(session):
    """Create the RAM root and session with the wheel owner's exact contract.

    mkdir(parents=True, mode=0700) protects only the leaf. A newly implicit
    parent instead inherits umask and can become group-writable. Validate the
    owned root through a no-follow descriptor before narrowing its write bits.
    Existing session directories and their data are never reused or changed.
    """
    directory=checked_staging_path(session)
    root=directory.parent
    root.mkdir(mode=0o700,exist_ok=True)
    flags=os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW
    descriptor=os.open(root,flags)
    try:
        before=os.fstat(descriptor)
        if not stat.S_ISDIR(before.st_mode) or before.st_uid!=os.geteuid():
            raise ValueError('RAM capture root must be an ordinary directory owned by this user: '+str(root))
        if stat.S_IMODE(before.st_mode)&0o022:
            os.fchmod(descriptor,stat.S_IMODE(before.st_mode)&~0o022)
        checked_staging_path(session)
        current=root.lstat()
        if (current.st_dev,current.st_ino)!=(before.st_dev,before.st_ino):
            raise ValueError('RAM capture root changed while preparing the session')
        os.mkdir(session,mode=0o700,dir_fd=descriptor)
        child=os.open(session,flags,dir_fd=descriptor)
        try:
            info=os.fstat(child)
            if info.st_uid!=os.geteuid() or stat.S_IMODE(info.st_mode)&0o022:
                raise ValueError('RAM capture session ownership/permissions invalid')
        finally:
            os.close(child)
    finally:
        os.close(descriptor)
    return directory


def sync_capture_directory(directory):
    descriptor=os.open(directory,os.O_RDONLY)
    try: os.fsync(descriptor)
    finally: os.close(descriptor)


def transfer_capture(staged, final_directory, manifest, *, destination_guard=None):
    """Stopped sources only: one writer, byte hashes and explicit disk fsync."""
    staged, final_directory = Path(staged), Path(final_directory)
    if destination_guard is not None: destination_guard.check()
    report = dict(status='TRANSFERRING', staging_directory=str(staged), final_directory=str(final_directory),
                  files={}, source_retained=True)
    checkpoint = dict(manifest, status='PARTIAL', recording_complete=False, finalization_state='STAGING_TRANSFER',
                      staging_transfer=report)
    atomic_json(final_directory/'capture_manifest.json', checkpoint)
    atomic_json(staged/'capture_manifest.json', checkpoint)
    total_bytes = sum(p.stat().st_size for p in staged.rglob('*') if p.is_file())
    copied_bytes, reported_at = 0, time.monotonic()
    for source in sorted(staged.rglob('*')):
        if destination_guard is not None: destination_guard.check()
        relative = source.relative_to(staged)
        if relative == Path('capture_manifest.json'): continue
        if source.is_symlink(): raise ValueError('staging transfer refuses symlink: '+str(relative))
        destination = final_directory/relative
        if source.is_dir(): destination.mkdir(exist_ok=False); continue
        before = source.stat()
        if not stat.S_ISREG(before.st_mode): raise ValueError('staging transfer requires ordinary files: '+str(relative))
        if shutil.disk_usage(final_directory).free < before.st_size+RESERVE_BYTES:
            raise RuntimeError('STAGING_TRANSFER_DISK_RESERVE_REACHED')
        with source.open('rb') as reader, destination.open('xb') as writer:
            while True:
                block = reader.read(1024**2)
                if not block:
                    break
                if destination_guard is not None: destination_guard.check()
                writer.write(block)
                copied_bytes += len(block)
                if time.monotonic()-reported_at >= 2:
                    print(json.dumps(dict(stage='TRANSFERRING',copied_bytes=copied_bytes,
                        total_bytes=total_bytes,file=str(relative))),flush=True)
                    reported_at = time.monotonic()
            writer.flush(); os.fsync(writer.fileno())
        after = source.stat()
        if (before.st_size,before.st_mtime_ns)!=(after.st_size,after.st_mtime_ns) or digest(source)!=digest(destination):
            raise ValueError('staging transfer byte/hash mismatch: '+str(relative))
        report['files'][str(relative)] = dict(bytes=before.st_size, sha256=digest(destination))
    # Persist directory entries only after all producers have stopped.
    for directory in sorted([p for p in final_directory.rglob('*') if p.is_dir()]+[final_directory],
                            key=lambda p:len(p.parts),reverse=True):
        sync_capture_directory(directory)
    report['status']='PERSISTED_AWAITING_AUDIT'
    manifest.update(status='PARTIAL',recording_complete=False,finalization_state='PERSISTED_AWAITING_AUDIT',
                    staging_transfer=report)
    if destination_guard is not None: destination_guard.check()
    atomic_json(final_directory/'capture_manifest.json',manifest)
    return report


def retain_staging_failure(staged, final_directory, manifest, error, *, destination_guard=None):
    value = dict(manifest, status='PARTIAL', recording_complete=False, finalization_state='STAGING_RECOVERY_REQUIRED',
        staging_transfer=dict(status='FAILED', staging_directory=str(staged), final_directory=str(final_directory),
            source_retained=True, error=type(error).__name__+': '+str(error),
            recovery='Retain both paths. Copy into a new archive and re-audit before any removal.'))
    for directory in (final_directory,staged):
        try:
            if Path(directory)==Path(final_directory) and destination_guard is not None:
                destination_guard.check()
            atomic_json(Path(directory)/'capture_manifest.json',value)
        except Exception as failure: print('STAGING_FAILURE_REPORT_WRITE_FAILED: '+str(failure),file=sys.stderr)
    print(json.dumps(dict(status='PARTIAL',output=str(final_directory),staging_directory=str(staged),
                         error=value['staging_transfer']['error']),ensure_ascii=False))
    return 2


def remove_memory_staging(staged, session):
    expected=checked_staging_path(session)
    if Path(staged).absolute()!=expected or not expected.is_dir():
        raise ValueError('cleanup only accepts this session RAM directory')
    for path in expected.rglob('*'):
        if path.is_symlink() or not path.resolve().is_relative_to(expected):
            raise ValueError('staging cleanup refuses linked/outside path')
    shutil.rmtree(expected)


def source_selection(profile):
    """Explicit requested evidence scope; excluded hardware is never acquired."""
    if profile not in PROFILES:
        raise ValueError('unknown capture profile: '+str(profile))
    from wc_cameras.config import ROLES
    core = ['lidar_left', 'lidar_right', 'imu', 'wheel']
    cameras = ['camera_'+role for role in ROLES]
    selected = core+(cameras if profile in CAMERA_PROFILES else [])
    if profile == 'all_sensors': selected.append('ultrasonic')
    excluded = [name for name in core+cameras+['ultrasonic'] if name not in selected]
    return dict(selected_sources=selected, excluded_sources=excluded,
                selection_basis='EXPLICIT_CAPTURE_PROFILE',
                completeness_scope='selected_sources_only',
                excluded_source_policy='NOT_PREFLIGHTED_STARTED_SUBSCRIBED_OR_VALIDATED')


def capacity(profile, duration, free_bytes, camera_profile=None, *, initialization_budget_s=0):
    if profile not in PROFILES or type(duration) is not int or not 1 <= duration <= 3600:
        raise ValueError('explicit profile and duration 1..3600 required')
    if type(initialization_budget_s) is not int or not 0 <= initialization_budget_s <= 3600:
        raise ValueError('source initialization budget must be an integer 0..3600')
    # Observed historic per-side raw+filtered recording, plus headroom. Cameras
    # store all captured BGR frames, never the 8 Hz preview count.
    rate = 2*6_600_000 + 500_000
    if profile in CAMERA_PROFILES:
        camera_profile = camera_profile or dict(width=320, height=240, capture_fps=30)
        rate += 4*camera_profile['width']*camera_profile['height']*3*camera_profile['capture_fps']
    expected_window = duration+initialization_budget_s
    expected = int(rate*expected_window*1.25) + SOFTWARE_SNAPSHOT_BUDGET_BYTES
    sufficient = free_bytes >= expected+RESERVE_BYTES
    startup_required = RESERVE_BYTES+SOFTWARE_SNAPSHOT_BUDGET_BYTES
    startup_allowed = free_bytes > startup_required
    return dict(free_bytes=free_bytes, estimated_recording_bytes=expected, reserve_bytes=RESERVE_BYTES,
                status=('AVAILABLE' if sufficient else 'WARNING') if startup_allowed else 'BLOCKED',
                startup_allowed=startup_allowed, startup_required_bytes=startup_required,
                forecast_policy='ADVISORY_ONLY',
                runtime_capacity_checks=True,
                requested_duration_s=duration, source_initialization_budget_s=initialization_budget_s,
                estimated_source_window_s=expected_window,
                sufficient=sufficient,
                maximum_duration_s=max(0,int((free_bytes-RESERVE_BYTES-SOFTWARE_SNAPSHOT_BUDGET_BYTES)/(rate*1.25))-initialization_budget_s),
                software_snapshot_budget_bytes=SOFTWARE_SNAPSHOT_BUDGET_BYTES,
                camera_codec='zlib_level_1_lossless', compression_savings_assumed=False,
                estimate_basis='historical SDK output + configured decoded camera payload + 25% headroom',
                actual_rate_may_differ=True)


def source_commands(root, run, directory, session, profile, *, manual_drive=False):
    source_selection(profile)
    from .cli import ros_command, H30_DEVICE, H30_BY_ID, H30_SERIAL
    py = sys.executable
    config = directory/'configuration'
    commands = {
        'lidar': ros_command(['ros2', 'launch', 'wc_xt_driver', 'dual_sources.launch.py',
            'source_mode:=dual', 'session_id:='+session, 'run_root:='+str(run), 'allow_hardware:=true',
            'config_directory:='+str(config/'lidar'),
            'require_recorder:=true',
            'read_only_probe:=false', 'device_config_policy:=preserve_current',
            'publish_cloud_mirror:=false', 'max_runtime_seconds:=0', 'source_stale_seconds:=3']),
        'imu': ros_command([py, '-m', 'wc_imu.ros_node', '--device', H30_DEVICE,
            '--expected-by-id', H30_BY_ID, '--hardware-serial', H30_SERIAL, '--sensor-id', 'H30-'+H30_SERIAL,
            '--session-id', session, '--run-root', run, '--duration', '0', '--stale-timeout', '2',
            '--require-recorder', '--journal-dir', directory/'sources/imu']),
        'wheel': ros_command([py, '-m', 'wc_motion.feedback_transport', '--config', config/'wheel_feedback.json',
            '--output', directory/'sources/wheel_feedback.jsonl', '--run-root', run,
            '--summary-path', directory/'sources/wheel_summary.json',
            '--samples', '0', '--max-duration', '0', '--rate-hz', '10', '--allow-read-queries',
            '--publish-ros', '--require-recorder', '--async-journal']),
    }
    if manual_drive:
        # Replace the read-only wheel reader; never acquire a second descriptor
        # or run a second feedback process beside the manual wheel owner.
        commands['wheel'] = ros_command([py, '-m', 'wc_runtime.mapping_wheel',
            '--session-root', directory, '--config', config/'manual_runtime.json',
            '--raw-output', directory/'sources/wheel_feedback.jsonl',
            '--summary-path', directory/'sources/wheel_summary.json',
            '--require-recorder', '--close-timeout-s', '30'])
        # Execute the same installed binary checked by preflight. The component
        # owner delivers SIGINT to its direct child; ros2 run would swallow that
        # signal while waiting for a UI child which never received it.
        commands['manual_ui'] = ros_command([root/'install/main/wc_bringup/lib/wc_bringup/manual_capture_ui',
            '--session-root', directory, '--session-id', session])
    if profile in CAMERA_PROFILES:
        from wc_cameras.config import ROLES
        for role in ROLES:
            commands['camera_'+role] = ros_command([py, '-m', 'wc_cameras.node', '--config', config/'cameras.json',
                '--role', role, '--profile', 'monitor_320', '--session-id', session, '--run-root', run,
                '--duration', '0', '--archive-dir', directory/'sources/cameras'/role])
    if profile == 'all_sensors':
        commands['ultrasonic'] = ros_command([py, '-m', 'wc_runtime.ultrasonic_capture',
            '--config', config/'ultrasonic.json', '--output', directory/'sources/ultrasonic',
            '--run-root', run, '--session', session, '--publish-ros', '--allow-read-queries'])
    return commands


def manual_runtime_configuration(setup, wheel_hardware, directory, session):
    """Compose only measured/declared control settings, independent of extrinsics.

    Geometry may remain UNKNOWN. --manual-drive is the current-session user
    authorization, not evidence of calibration or a tested hardware watchdog.
    Existing command scale and wheel candidate still require explicit sources.
    """
    from .hardware_setup import resolve_hardware_setup
    from .mapping_wheel import validate_manual_controls
    from wc_motion.feedback_transport import validate_config
    resolved = resolve_hardware_setup(setup)
    hardware = validate_config(wheel_hardware)
    controls = copy.deepcopy(resolved.get('manual_controls'))
    if not isinstance(controls, dict) or 'command_rpm_to_register_scale' not in setup.get('manual_controls', {}):
        raise ValueError('MANUAL_DRIVE_SETTINGS_UNAVAILABLE: explicit control scale/evidence required')
    controls.update(arm_allowed=True, interaction_policy='hybrid_manual',
        reason='本会话显式 --manual-drive：用户键盘驾驶及自动手推授权；不证明物理断线停车。')
    controls = validate_manual_controls(controls)
    return dict(session_id=session, source_mode='real', status='EXPERIMENT', duration_s=0,
                continuous_mapping=True, wheel_device_id=hardware['device_id'],
                wheel_hardware_config=str(Path(directory)/'configuration/wheel_feedback.json'),
                wheel_candidate=resolved['wheel_candidate'], manual_controls=controls,
                control_source='capture_manual_drive', capture_manual_drive=True,
                motion_estimation=False, slam=False,
                manual_authorization={'source': 'EXPLICIT_CAPTURE_MANUAL_DRIVE_ARGUMENT',
                    'session_id': session, 'scope': 'human WASD and hybrid hand-push for this session only',
                    'hardware_stop_validated': False})


def preflight(root, profile, duration, *, manual_drive=False, capacity_path=None):
    source_selection(profile)
    from .cli import device_preflight, imu_preflight
    from wc_motion.feedback_transport import validate_config, verify_identity, require_unoccupied
    checks = {}
    def check(name, callback):
        try:
            checks[name] = dict(status='AVAILABLE', evidence=callback())
        except Exception as exc:
            checks[name] = dict(status='BLOCKED', reason=str(exc))
    check('lidar', device_preflight)
    check('imu', imu_preflight)
    def wheel():
        cfg = validate_config(json.loads((root/'config/wheel_feedback_current.json').read_text()))
        path, identity = verify_identity(cfg)
        require_unoccupied(path)
        return {'path': str(path), 'identity': str(identity)}
    check('wheel', wheel)
    if manual_drive:
        def manual_settings():
            manual_runtime_configuration(
                json.loads((root/'config/hardware_setup.json').read_text(encoding='utf-8')),
                json.loads((root/'config/wheel_feedback_current.json').read_text(encoding='utf-8')),
                root/'data/experiments/preflight', 'preflight')
            return {'authorization': 'EXPLICIT_CAPTURE_MANUAL_DRIVE_ARGUMENT',
                    'geometry_required': False, 'wheel_owner': 'wc_runtime.mapping_wheel'}
        check('manual_drive_configuration', manual_settings)
        def manual_ui():
            from .sensor_viewer import desktop_environment, confirm_display
            executable = root/'install/main/wc_bringup/lib/wc_bringup/manual_capture_ui'
            if not executable.is_file():
                raise FileNotFoundError('standalone capture UI not installed: '+str(executable))
            env = desktop_environment()
            confirm_display(env)
            return {'executable': str(executable), 'display': env.get('DISPLAY'),
                    'ekf_started': False, 'slam_started': False}
        check('manual_ui', manual_ui)
    if profile in CAMERA_PROFILES:
        from wc_cameras.config import load_config
        from wc_cameras.capture import verify_device, require_unoccupied as camera_free
        cameras, camera_config_hash = load_config(root/'config/cameras.json')
        for camera in cameras['cameras']:
            def verify(camera=camera):
                info = verify_device(camera)
                camera_free(info['resolved_node'])
                return info
            check('camera_'+camera['role'], verify)
    if profile == 'all_sensors':
        from .ultrasonic_capture import identity, validate_config as ultrasound_config
        def ultrasound():
            cfg = ultrasound_config(json.loads((root/'config/ultrasonic_capture.json').read_text()))
            path, _ = identity(cfg)
            require_unoccupied(path)
            return {'path': str(path), 'probe_response': 'NOT_QUERIED', 'physical_roles': 'UNVERIFIED'}
        check('ultrasonic', ultrasound)
    if profile in CAMERA_PROFILES:
        checks['camera_configuration'] = dict(status='CONFIG_VALID', sha256=camera_config_hash,
                                               profile='monitor_320', capture_profile=copy.deepcopy(cameras['profiles']['monitor_320']))
    checks['capacity'] = capacity(profile, duration, shutil.disk_usage(capacity_path or root).free,
                                  cameras['profiles']['monitor_320'] if profile in CAMERA_PROFILES else None,
                                  initialization_budget_s=ALL_SOURCE_READY_TIMEOUT_S)
    return checks


def stop_children(children, grace=40):
    """Notify only owned wrappers; their pidfd cleanup reaps the descendant tree."""
    for process in children.values():
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
    deadline = time.monotonic()+grace
    while time.monotonic() < deadline and any(p.poll() is None for p in children.values()):
        time.sleep(.1)
    for process in children.values():
        if process.poll() is None:
            process.send_signal(signal.SIGTERM)
    deadline = time.monotonic()+5
    while time.monotonic() < deadline and any(p.poll() is None for p in children.values()):
        time.sleep(.1)
    for process in children.values():
        if process.poll() is None:
            process.kill()
    result = {}
    for name, process in children.items():
        try:
            result[name] = process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            result[name] = 'UNREAPED'
    return result


def stop_capture_children(children, recorder, *, manual_drive=False):
    """Revoke human intent/control before other sources, then drain the recorder."""
    results = {}
    remaining = dict(children)
    if manual_drive:
        for name in ('manual_ui', 'wheel'):
            if name in remaining:
                # The wheel wrapper allows 40 s for transport/journal close;
                # its supervisor must allow that complete budget before TERM.
                results.update(stop_children({name: remaining.pop(name)},
                                             grace=45 if name == 'wheel' else 40))
    results.update(stop_children(remaining))
    # DDS callbacks may remain in flight after SDK stop and source drain.
    time.sleep(.5)
    if recorder is not None:
        results.update(stop_children({'recorder': recorder}, grace=60))
    return results


def wheel_ready(directory, session):
    path = Path(directory)/'wheel_ready.json'
    if not path.exists():
        return False
    value = json.loads(path.read_text(encoding='utf-8'))
    if (not isinstance(value, dict) or value.get('session_id') != session or value.get('ready') is not True
            or value.get('raw_recorder_discovered') is not True
            or value.get('manual_socket_ready') is not True
            or type(value.get('feedback_samples')) is not int or value['feedback_samples'] < 1
            or value.get('source_type') != 'HYBRID_MANUAL_CAPTURE'):
        raise ValueError('MANUAL_WHEEL_READY_INVALID: '+str(path))
    return True


def final_control_evidence(directory, *, manual_drive=False):
    """Report actual final owner counters; absent evidence is unknown, never zero."""
    path = Path(directory)/'sources/wheel_summary.json'
    result = dict(control_transmissions=None, control_count_status='UNVERIFIED',
                  control_evidence='sources/wheel_summary.json')
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
        if not isinstance(value, dict):
            raise ValueError('wheel source summary must be an object')
        count = value.get('control_transmissions')
        if type(count) is not int or count < 0:
            raise ValueError('missing/invalid actual control transmission count')
        result.update(control_transmissions=count, control_count_status='SOURCE_OWNER_FINAL_COUNTER',
                      control_bytes_observed=value.get('control_bytes_observed'),
                      control_write_attempts=value.get('control_write_attempts'))
        if not manual_drive and count != 0:
            raise ValueError('read-only capture reported control transmissions')
        if manual_drive and (value.get('session_id') != Path(directory).name
                             or value.get('ready_observed') is not True
                             or value.get('recorder_discovered') is not True):
            raise ValueError('manual wheel source session/readiness evidence incomplete')
        journal = value.get('journal')
        if (value.get('synchronized') is not True or value.get('closed_normally') is not True
                or not isinstance(journal, dict) or journal.get('final_fsync_complete') is not True):
            raise ValueError('wheel final close/persistence evidence incomplete')
        result['control_count_status'] = 'SOURCE_OWNER_FINAL_SYNCHRONIZED_COUNTER'
    except (OSError, ValueError, KeyError, TypeError) as error:
        result['error'] = str(error)
    return result


def validate_capture_audit(value):
    """Reject incomplete/malformed audit results before any complete commit."""
    if (not isinstance(value, dict) or type(value.get('recording_complete')) is not bool
            or value.get('status') != ('COMPLETE' if value['recording_complete'] else 'PARTIAL')
            or not isinstance(value.get('issues'), list) or not isinstance(value.get('source_accounting'), dict)):
        raise ValueError('invalid capture audit result schema/status')
    for row in value['issues']:
        if not isinstance(row, dict) or any(not isinstance(row.get(key), str) for key in ('code','detail','evidence')):
            raise ValueError('invalid capture audit issue schema')
    if value['recording_complete'] and value['issues']:
        raise ValueError('capture audit complete flag contradicts issues')
    if any(not isinstance(name, str) or not isinstance(counts, dict)
           for name, counts in value['source_accounting'].items()):
        raise ValueError('invalid capture source accounting schema')
    # Serialization failures and NaN must be discovered before final commit.
    json.dumps(value, ensure_ascii=False, allow_nan=False)
    return value


def capture_summary(manifest):
    selection = source_selection(manifest['profile'])
    summary = ['# V7 采集结果', '', '状态：'+manifest['status'],
               '配置：'+manifest['profile'],
               '本次请求来源：'+ '、'.join(selection['selected_sources']),
               '本次排除来源：'+ ('、'.join(selection['excluded_sources']) or '无')+'；未预检、启动、订阅或校验。',
               '完整性只针对本次请求来源；mapping_cameras 不包含超声波，不宣称全部传感器完整。',
               'EKF/SLAM：未启动。用户驾驶/手推：'+('本会话 --manual-drive。' if manifest['manual_drive'] else '未启用，只读采集。'),
               '真实控制发送次数：'+str(manifest['control_transmissions'])+'；来源：'+manifest['control_count_status'],
               '原始档案：保留；后续对照仅覆盖同一 SDK 输出后的算法。',
               '绝对定位精度：待评估。', '', '|来源|成功接收|发布|落盘/留存|', '|---|---:|---:|---:|']
    for name, counts in manifest['source_accounting'].items():
        summary.append('|%s|%s|%s|%s|' % (name, counts.get('successfully_received'),
            counts.get('published', '不适用'), counts.get('retained', '见逐条源端与录包对账')))
    summary.extend(['', '## 待处理项']+[f"- {i['code']}：{i['detail']}；证据 {i['evidence']}" for i in manifest['issues']])
    return '\n'.join(summary)+'\n'


def finalize_capture(directory, run, manifest, hashes, results, failures, stop_reason, *, destination_guard=None):
    """Finalize stopped sources only; every file-stage failure retains PARTIAL.

    The durable FINALIZING checkpoint precedes copy/audit/hash/report work. The
    health writer inspects a prospective in-memory result while the on-disk
    manifest remains PARTIAL, so COMPLETE is committed only after all stages.
    """
    if destination_guard is not None: destination_guard.check()
    issues = [dict(code='CAPTURE_INTERRUPTED', evidence='logs', detail=f) for f in failures]
    def issue(code, evidence, error):
        issues.append(dict(code=code, evidence=str(evidence), detail=type(error).__name__+': '+str(error)))
    try:
        controls = final_control_evidence(directory, manual_drive=manifest['manual_drive'])
    except Exception as error:
        controls = dict(control_transmissions=None, control_count_status='UNVERIFIED',
                        control_evidence='sources/wheel_summary.json', error=str(error))
    if controls.get('error'):
        issue('CONTROL_FINAL_EVIDENCE_INVALID', 'sources/wheel_summary.json', ValueError(controls['error']))
    manifest.update(status='PARTIAL', recording_complete=False, finalization_state='FINALIZING',
                    source_accounting={}, issues=issues, input_hashes=hashes, process_exit_codes=results,
                    stop_reason=stop_reason, control_evidence=controls,
                    control_transmissions=controls['control_transmissions'],
                    control_count_status=controls['control_count_status'],
                    ended_wall_ns=time.time_ns(), ended_monotonic_ns=time.monotonic_ns())
    manifest_path = directory/'capture_manifest.json'
    contract_path = directory/'configuration/capture_contract.json'
    contract = json.loads(contract_path.read_text()) if contract_path.is_file() else None
    def persist(value, stage):
        try:
            if destination_guard is not None: destination_guard.check()
            atomic_json(manifest_path, value)
            return True
        except Exception as error:
            issue('CAPTURE_MANIFEST_WRITE_FAILED', 'capture_manifest.json', RuntimeError(stage+': '+str(error)))
            value.update(status='PARTIAL', recording_complete=False, issues=issues,
                         manifest_persistence_failed=True)
            print('CAPTURE_MANIFEST_WRITE_FAILED '+stage+': '+str(error)+'; retained '+str(directory), file=sys.stderr)
            # atomic_json can fail after rename (directory fsync). Attempt one
            # explicit downgrade; never claim that this failed commit succeeded.
            try:
                if destination_guard is not None: destination_guard.check()
                atomic_json(manifest_path, value)
            except Exception as fallback_error:
                print('CAPTURE_PARTIAL_MANIFEST_WRITE_FAILED: '+str(fallback_error), file=sys.stderr)
            return False
    if not persist(manifest, 'FINALIZING_CHECKPOINT'):
        return 2

    try:
        # The journals live outside RAM and are copied here after transfer.
        # Recheck their real size against current disk space before duplication.
        from .capture_storage import StorageMonitor
        StorageMonitor(directory,directory,run/'sessions'/manifest['session_id'],
                       memory_staging=False,destination_guard=destination_guard).check(force=True,closing=True)
        (directory/'sources/lidar').mkdir()
    except Exception as error:
        issue('CAPTURE_SOURCE_COPY_FAILED', 'sources/lidar', error)
    else:
        for side in ('left', 'right'):
            try:
                if destination_guard is not None: destination_guard.check()
                source = run/'sessions'/manifest['session_id']/side
                if source.is_dir():
                    shutil.copytree(source, directory/'sources/lidar'/side)
            except Exception as error:
                issue('CAPTURE_SOURCE_COPY_FAILED', 'sources/lidar/'+side, error)
    if contract:
        try:
            if destination_guard is not None: destination_guard.check()
            from .capture_support import create_source_indexes
            create_source_indexes(directory,contract)
        except Exception as error:
            issue('CAPTURE_SOURCE_INDEX_FAILED','sources',error)
    audit = dict(recording_complete=False, status='PARTIAL', issues=[], source_accounting={})
    try:
        from .capture_audit import audit_capture
        audit = validate_capture_audit(audit_capture(directory, manifest['profile'], results))
    except Exception as error:
        issue('CAPTURE_AUDIT_FAILED', 'sources/index.jsonl; bag; sources', error)
    issues.extend(audit['issues'])
    if any(type(code) is not int or code != 0 for name,code in results.items() if name != 'preview'):
        issue('CAPTURE_PROCESS_INCOMPLETE', 'logs', ValueError(str(results)))
    try:
        for relative,expected_hash in list(hashes.items()):
            if not (directory/relative).is_file():
                issue('CAPTURE_FROZEN_INPUT_MISSING',relative,ValueError('immutable snapshot file is missing'))
        for path in directory.rglob('*'):
            if path.is_file() and path.name not in ('capture_manifest.json','health.json','diagnosis_zh.md','summary_zh.md'):
                relative = str(path.relative_to(directory))
                try:
                    actual_hash = digest(path)
                    if relative in hashes and hashes[relative] != actual_hash:
                        issue('CAPTURE_FROZEN_INPUT_CHANGED',relative,ValueError('immutable snapshot hash changed'))
                    else:
                        hashes[relative] = actual_hash
                except Exception as error:
                    issue('CAPTURE_HASH_FAILED', relative, error)
    except Exception as error:
        issue('CAPTURE_HASH_SCAN_FAILED', '.', error)
    prospective = {**manifest, **audit, 'input_hashes':hashes, 'issues':issues,
                   'finalization_state':'FINALIZED'}
    # SDK journals are copied after the RAM transfer. Persist these and all
    # added indexes before committing any successful manifest.
    try:
        if destination_guard is not None: destination_guard.check()
        from .capture_support import synchronize_tree
        evidence = synchronize_tree(directory)
        prospective.update(durability_evidence=evidence,durability_complete=True)
    except Exception as error:
        prospective.update(durability_evidence=dict(complete=False,error=str(error)),durability_complete=False)
        issue('CAPTURE_DURABILITY_FAILED','sources; bag; configuration; software',error)
    def qualify():
        evidence_ok = (audit.get('transport_complete') is True and audit.get('window_complete') is True
                       if contract else audit['recording_complete'])
        complete = evidence_ok and prospective.get('durability_complete') is True and not issues
        prospective['completion_pending'] = ([name for name,ok in
            [('transport',audit.get('transport_complete')),('window',audit.get('window_complete')),
             ('durability',prospective.get('durability_complete'))] if ok is not True] if contract else [])
        prospective.update(recording_complete=complete, status='COMPLETE' if complete else 'PARTIAL')
    qualify()
    summary_status = health_status = None
    health_attempts = summary_attempts = 0
    def write_summary(refresh=False):
        nonlocal summary_status, summary_attempts
        summary_attempts += 1
        try:
            if destination_guard is not None: destination_guard.check()
            (directory/'summary_zh.md').write_text(capture_summary(prospective), encoding='utf-8')
            summary_status = prospective['status']
        except Exception as error:
            issue('CAPTURE_SUMMARY_REFRESH_FAILED' if refresh else 'CAPTURE_SUMMARY_WRITE_FAILED',
                  'summary_zh.md', error)
            qualify()
    def write_health(refresh=False):
        nonlocal health_status, health_attempts
        health_attempts += 1
        try:
            if destination_guard is not None: destination_guard.check()
            from .runtime_health import write_health_report
            write_health_report(directory, capture_manifest=prospective)
            health_status = prospective['status']
            return True
        except Exception as error:
            issue('CAPTURE_HEALTH_REPORT_REFRESH_FAILED' if refresh else 'CAPTURE_HEALTH_REPORT_FAILED',
                  'health.json; diagnosis_zh.md', error)
            qualify()
            return False
    write_summary()
    if not write_health():
        # A report writer can fail after replacing health.json but before the
        # diagnosis. Try once with PARTIAL, then retain any stale report under
        # an explicit failed name instead of leaving a COMPLETE health marker.
        if not write_health(refresh=True):
            for name in ('health.json', 'diagnosis_zh.md'):
                try:
                    if destination_guard is not None: destination_guard.check()
                    path = directory/name
                    if path.exists():
                        failed = path.with_name(path.stem+'.failed_finalization_'+str(time.time_ns())+path.suffix)
                        os.replace(path, failed)
                except Exception as error:
                    issue('CAPTURE_HEALTH_REPORT_RETENTION_FAILED', name, error)
        if summary_status == 'COMPLETE' and summary_attempts < 2:
            write_summary(refresh=True)
    try:
        if destination_guard is not None: destination_guard.check()
        prospective['durability_evidence'] = synchronize_tree(directory)
    except Exception as error:
        prospective.update(durability_complete=False,durability_evidence=dict(complete=False,error=str(error)))
        issue('CAPTURE_REPORT_DURABILITY_FAILED','health.json; summary_zh.md',error)
        qualify()
        write_summary(refresh=True)
        write_health(refresh=True)
    committed = persist(prospective, 'FINAL_RESULT')
    if not committed:
        # The final manifest fsync can fail after all reports were generated.
        # Downgrade those reports too; retries are bounded to one per report.
        issue_count = len(issues)
        if health_status == 'COMPLETE' and health_attempts < 2:
            if not write_health(refresh=True):
                try:
                    if destination_guard is not None: destination_guard.check()
                    path = directory/'health.json'
                    if path.exists():
                        os.replace(path, directory/('health.failed_finalization_'+str(time.time_ns())+'.json'))
                except Exception as error:
                    issue('CAPTURE_HEALTH_REPORT_RETENTION_FAILED', 'health.json', error)
        if summary_status == 'COMPLETE' and summary_attempts < 2:
            write_summary(refresh=True)
        if len(issues) != issue_count:
            # Persist new report-refresh failures when the storage is usable;
            # an unsuccessful write is reported, without recursively retrying.
            try:
                if destination_guard is not None: destination_guard.check()
                atomic_json(manifest_path, prospective)
            except Exception as error:
                print('CAPTURE_PARTIAL_MANIFEST_REFRESH_FAILED: '+str(error), file=sys.stderr)
    print(json.dumps(dict(status=prospective['status'], recording_complete=prospective['recording_complete'],
                         output=str(directory), issues=issues), ensure_ascii=False, indent=2))
    return 0 if committed and prospective['recording_complete'] else 2


def snapshot(root, directory, *, profile='all_sensors', storage_policy=None):
    selection = source_selection(profile)
    cfg = directory/'configuration'
    cfg.mkdir()
    paths = {'runtime_config.json': 'config/mapping_live.json', 'hardware_setup.json': 'config/hardware_setup.json',
             'wheel_feedback.json': 'config/wheel_feedback_current.json'}
    if profile in CAMERA_PROFILES: paths['cameras.json'] = 'config/cameras.json'
    if profile == 'all_sensors': paths['ultrasonic.json'] = 'config/ultrasonic_capture.json'
    from .storage_policy import StoragePolicy
    hashes = (storage_policy or StoragePolicy(root)).snapshot_configuration(cfg)
    atomic_json(cfg/'source_selection.json', dict(profile=profile, **selection))
    hashes['configuration/source_selection.json'] = digest(cfg/'source_selection.json')
    from .capture_support import data_capabilities
    atomic_json(cfg/'data_capabilities.json',data_capabilities(profile))
    hashes['configuration/data_capabilities.json'] = digest(cfg/'data_capabilities.json')
    for destination, source in paths.items():
        shutil.copyfile(root/source, cfg/destination)
        hashes[str((cfg/destination).relative_to(directory))] = digest(cfg/destination)
    # Preserve referenced calibration evidence and source code identities.
    shutil.copytree(root/'config/calibration', cfg/'calibration')
    shutil.copytree(root/'install/main/wc_xt_driver/share/wc_xt_driver/config', cfg/'lidar')
    for path in (cfg/'lidar').rglob('*'):
        if path.is_file():
            hashes[str(path.relative_to(directory))] = digest(path)
    for path in (cfg/'calibration').rglob('*'):
        if path.is_file():
            hashes[str(path.relative_to(directory))] = digest(path)
    software = {}
    for base in ('src', 'scripts', 'install/main/wc_bringup/lib/wc_bringup', 'install/main/wc_xt_driver/lib/wc_xt_driver',
                 'install/main/wc_estimation/lib/wc_estimation'):
        for path in (root/base).rglob('*'):
            if path.is_file() and not path.is_symlink() and '__pycache__' not in path.parts:
                software[str(path.relative_to(root))] = digest(path)
    atomic_json(cfg/'software_hashes.json', software)
    hashes['configuration/software_hashes.json'] = digest(cfg/'software_hashes.json')
    from .hardware_setup import resolve_hardware_setup
    resolved = resolve_hardware_setup(json.loads((cfg/'hardware_setup.json').read_text(encoding='utf-8')))
    atomic_json(directory/'capability_assessment.json', resolved.get('geometry_report', {}))
    from .capture_support import software_snapshot
    hashes.update(software_snapshot(root,directory))
    return hashes


def run_capture(args):
    recorder_queue_size = getattr(args, 'recorder_queue_size', 256)
    if type(recorder_queue_size) is not int or not 1 <= recorder_queue_size <= 8192:
        raise ValueError('recorder-queue-size must be an integer 1..8192')
    from .cli import ROOT, RUN, target, environment, ros_command, H30_SERIAL
    target()
    manual_drive = bool(getattr(args, 'manual_drive', False))
    preview = bool(getattr(args, 'preview', False))
    staging = getattr(args, 'staging', 'disk') # Existing library callers keep their disk contract.
    if staging not in ('memory','disk'): raise ValueError('staging must be memory or disk')
    from .capture_destination import capture_destination
    output_root, destination_guard = capture_destination(ROOT, getattr(args,'output_root',None),
                                                         getattr(args,'required_output_uuid',None))
    destination_options = {'destination_guard':destination_guard} if destination_guard is not None else {}
    if (destination_guard is not None and manual_drive and staging=='disk'
            and destination_guard.metadata['filesystem'] in ('exfat','fuseblk','fuse.exfat')):
        raise ValueError('manual-drive on exFAT requires --staging memory for its Unix socket')
    if destination_guard is not None and not output_root.is_dir():
        raise ValueError('CAPTURE_DESTINATION_UNAVAILABLE: external output-root must already exist')
    checks = preflight(ROOT, args.profile, args.duration, manual_drive=manual_drive,
                       **({'capacity_path':destination_guard.mount_root} if destination_guard is not None else {}))
    if destination_guard is not None: checks['destination']=destination_guard.check()
    if preview:
        try:
            from .sensor_viewer import desktop_environment, confirm_display
            preview_environment = desktop_environment()
            confirm_display(preview_environment)
            checks['preview'] = dict(status='AVAILABLE',mode='SUBSCRIBER_ONLY_NATIVE_LEFT_RIGHT',
                                     display=preview_environment.get('DISPLAY'))
        except Exception as error:
            checks['preview'] = dict(status='BLOCKED',reason=str(error))
    final_directory = output_root/args.session
    directory = checked_staging_path(args.session) if staging=='memory' else final_directory
    if staging=='memory':
        try: checks['memory_capacity']=memory_preflight(checks,recorder_queue_size,args.profile)
        except Exception as error: checks['memory_capacity']=dict(status='BLOCKED',sufficient=False,reason=str(error))
    result = dict(profile=args.profile, duration_s=args.duration, output=str(final_directory), preflight=checks,
                  startup_policy='DIRECT_CAPTURE_WITH_RUNTIME_STORAGE_GUARD',
                  storage_staging=dict(mode=staging,live_directory=str(directory),final_directory=str(final_directory),
                      durable_archive=False,cleanup='NOT_ATTEMPTED'),
                  **source_selection(args.profile),
                  recorder_queue_size=recorder_queue_size,
                  manual_drive=manual_drive, preview=preview, control_transmissions=0 if args.dry_run else None,
                  control_count_status='NO_PROCESSES_STARTED' if args.dry_run else 'AWAITING_SOURCE_OWNER',
                  motion_estimation=False, slam=False)
    if args.dry_run:
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if all(v.get('status')!='BLOCKED' for v in checks.values()) else 2
    if not checks['capacity'].get('startup_allowed',checks['capacity']['sufficient']):
        raise RuntimeError('CAPTURE_STORAGE_RESERVE_REACHED: '+json.dumps(checks['capacity']))
    if staging=='memory' and not checks['memory_capacity'].get('startup_allowed',checks['memory_capacity']['sufficient']):
        raise RuntimeError('CAPTURE_MEMORY_CAPACITY_INSUFFICIENT: '+json.dumps(checks['memory_capacity']))
    if manual_drive and any(checks.get(name, {}).get('status') != 'AVAILABLE'
                            for name in ('wheel', 'manual_drive_configuration', 'manual_ui')):
        raise RuntimeError('MANUAL_DRIVE_PREFLIGHT_BLOCKED: '+json.dumps(checks, ensure_ascii=False))
    if not args.diagnostic and any(v.get('status')=='BLOCKED' for v in checks.values()):
        raise RuntimeError('CAPTURE_DEVICE_MISSING: '+json.dumps(checks, ensure_ascii=False))
    for name in ('capacity','memory_capacity'):
        if checks.get(name,{}).get('status')=='WARNING':
            print(json.dumps(dict(stage='CAPACITY_ESTIMATE_WARNING',resource=name,
                message='预计空间可能不足以录满请求时长；允许开始，按实际占用监测并收尾。',
                **checks[name]),ensure_ascii=False),flush=True)
    # Resolve the GUI environment before creating a RECORDING manifest. A
    # display lost since preflight must fail without leaving a phantom session.
    env = dict(os.environ, **environment())
    env.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    if manual_drive or preview:
        from .sensor_viewer import desktop_environment, confirm_display
        ui_env = desktop_environment()
        confirm_display(ui_env)
        env.update({key: ui_env[key] for key in ('DISPLAY', 'XAUTHORITY', 'DBUS_SESSION_BUS_ADDRESS')
                    if key in ui_env})
    import fcntl
    with ExitStack() as stack:
        if destination_guard is not None: destination_guard.check()
        (RUN/'locks').mkdir(parents=True, exist_ok=True)
        for name in ('sensor_owner.lock', 'domain-83-source.lock'):
            stream = stack.enter_context((RUN/'locks'/name).open('a+'))
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        if destination_guard is not None: destination_guard.check()
        final_directory.mkdir(parents=destination_guard is None, exist_ok=False)
        if destination_guard is not None: destination_guard.check()
        if staging=='memory':
            directory=prepare_staging_directory(args.session)
            atomic_json(final_directory/'capture_manifest.json',dict(result,schema_version=2,session_id=args.session,
                status='PARTIAL',recording_complete=False,finalization_state='RAM_INITIALIZING'))
        (directory/'sources').mkdir()
        (directory/'logs').mkdir()
        (directory/'ready').mkdir()
        from .capture_support import progress
        progress(directory,'PREPARING')
        hashes = snapshot(ROOT, directory, profile=args.profile,
                          storage_policy=getattr(destination_guard, 'storage_policy', None))
        from .capture_contract import build_capture_contract, read_source_readiness
        contract = build_capture_contract(directory/'configuration',args.profile,args.session,
                                           imu_sensor_id='H30-'+H30_SERIAL)
        atomic_json(directory/'configuration/capture_contract.json',contract)
        hashes['configuration/capture_contract.json'] = digest(directory/'configuration/capture_contract.json')
        if manual_drive:
            runtime = manual_runtime_configuration(
                json.loads((directory/'configuration/hardware_setup.json').read_text(encoding='utf-8')),
                json.loads((directory/'configuration/wheel_feedback.json').read_text(encoding='utf-8')),
                directory, args.session)
            atomic_json(directory/'configuration/manual_runtime.json', runtime)
            hashes['configuration/manual_runtime.json'] = digest(directory/'configuration/manual_runtime.json')
        commands = source_commands(ROOT, RUN, directory, args.session, args.profile, manual_drive=manual_drive)
        if preview:
            commands['preview'] = ros_command([sys.executable,'-m','wc_runtime.capture_preview',
                '--session-root',directory,'--session-id',args.session,'--preview-hz','3','--cloud-source','raw'])
        recorder_command = ros_command([sys.executable, '-m', 'wc_runtime.source_recorder',
                                       '--output', directory, '--profile', args.profile,
                                       '--queue-size', str(recorder_queue_size)])
        manifest = dict(result, schema_version=2, session_id=args.session, status='RECORDING', recording_complete=False,
                        diagnostic_capture=args.diagnostic, bag_path='bag', mode='all',
                        runtime_config_path='configuration/runtime_config.json',
                        hardware_setup_path='configuration/hardware_setup.json', imu_sensor_id='H30-'+H30_SERIAL,
                        input_hashes=hashes, commands=commands, recorder_command=recorder_command,
                        capture_contract_required=True,capture_contract_path='configuration/capture_contract.json',
                        time_policy='RECORDER_RECEIPT_WALL_TIME_WITH_ORIGINAL_SOURCE_MONOTONIC_AND_SEQUENCE_RETAINED',
                        coverage_time_policy='ORIGINAL_SOURCE_HOST_MONOTONIC_NS',
                        bag_storage_time_policy='RECORDER_CALLBACK_WALL_TIME_NS',
                        recording_transport={'reliability': 'RELIABLE', 'durability': 'VOLATILE',
                                             'subscriber_history': {'imu': 2048, 'other_topics': 128},
                                             'writer_queue_items': recorder_queue_size,
                                             'completeness_authority': 'independent source/index/bag reconciliation'},
                        started_wall_ns=time.time_ns(), started_monotonic_ns=time.monotonic_ns(),
                        requested_source_duration_s=args.duration, raw_retention='RETAINED',
                        source_initialization_budget_s=checks['capacity'].get('source_initialization_budget_s',0),
                        duration_meaning='full requested common window begins only after every selected source is ready; preparation retained',
                        source_scope='SDK_OUTPUT_ONWARD_NOT_COMPLETE_OLD_SDK_REPRODUCTION')
        if manual_drive:
            manifest.update(wheel_owner='wc_runtime.mapping_wheel',
                            control_authorization='EXPLICIT_CAPTURE_MANUAL_DRIVE_ARGUMENT',
                            manual_interaction_policy='hybrid_manual',
                            manual_runtime_path='configuration/manual_runtime.json',
                            duration_meaning='requested manual window begins after every source is ready; earlier initialization is retained')
        atomic_json(directory/'capture_manifest.json', manifest)
        def start(name, command):
            if destination_guard is not None: destination_guard.check()
            log = stack.enter_context((directory/'logs'/(name+'.log')).open('xb'))
            wrapped = [sys.executable, '-m', 'wc_runtime.component', '--parent', str(os.getpid()),
                       '--sigint-grace-s', '55' if name == 'recorder' else '40' if name == 'wheel' and manual_drive else '30',
                       '--', *map(str,command)]
            return subprocess.Popen(wrapped, env=env, cwd=ROOT, stdin=subprocess.DEVNULL,
                                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        stopped = [False]
        old_handlers = {}
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            old_handlers[sig] = signal.signal(sig, lambda *_: stopped.__setitem__(0, True))
        children, recorder, failures = {}, None, []
        results, stop_reason = {}, 'REQUESTED_DURATION_REACHED'
        from .capture_storage import StorageMonitor
        storage = StorageMonitor(directory,final_directory,RUN/'sessions'/args.session,
                                 memory_staging=staging=='memory',
                                 close_growth_seconds=230 if manual_drive else 135,**destination_options)
        def check_storage(*, force=False, closing=False):
            try:
                report=storage.check(force=force,closing=closing)
            except RuntimeError:
                if storage.last_report:
                    if staging!='memory' and destination_guard is not None: destination_guard.check()
                    atomic_json(directory/'storage_usage.json',storage.last_report)
                raise
            if report is not None:
                atomic_json(directory/'storage_usage.json',report)
                if staging=='memory' and not closing:
                    budget=checks['memory_capacity']
                    if shutil.disk_usage(directory).free<=budget['tmpfs_reserve_bytes'] or host_memory()['MemAvailable']<=budget['memory_reserve_bytes']:
                        raise RuntimeError('CAPTURE_MEMORY_RESERVE_REACHED')
        try:
            check_storage(force=True)
            recorder = start('recorder', recorder_command)
            deadline = time.monotonic()+20
            while not (directory/'recorder_ready.json').exists():
                check_storage()
                if stopped[0] or recorder.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('RECORDER_NOT_READY; no sources started')
                time.sleep(.05)
            progress(directory,'STARTING_SOURCES',selected_sources=result['selected_sources'])
            for name, command in commands.items():
                if name == 'manual_ui':
                    continue
                children[name] = start(name, command)
            ready_deadline = time.monotonic()+ALL_SOURCE_READY_TIMEOUT_S
            if manual_drive:
                progress(directory,'WAITING_FOR_WHEEL',message='正在等待轮反馈，成功后打开手动驾驶窗口')
                manual_ready_deadline = time.monotonic()+MANUAL_READY_TIMEOUT_S
                while not wheel_ready(directory, args.session):
                    check_storage()
                    if stopped[0] or recorder.poll() is not None or children['wheel'].poll() is not None or time.monotonic()>manual_ready_deadline:
                        detail=dict(wheel_exit_code=children['wheel'].poll(),recorder_exit_code=recorder.poll(),
                                    user_stop_requested=stopped[0],evidence='logs/wheel.log')
                        try:
                            with (directory/'logs/wheel.log').open('rb') as log:
                                log.seek(0,os.SEEK_END)
                                log.seek(max(0,log.tell()-4096))
                                detail['wheel_error_tail']=log.read(4096).decode('utf-8',errors='replace').splitlines()[-4:]
                        except OSError as error:
                            detail['log_read_error']=str(error)
                        raise RuntimeError('MANUAL_WHEEL_NOT_READY; UI not started; '+json.dumps(detail,ensure_ascii=False))
                    time.sleep(.05)
                children['manual_ui'] = start('manual_ui', commands['manual_ui'])
            progress(directory,'WAITING_FOR_ALL_SOURCES',required_sources=list(contract['sources']))
            readiness = None
            while not stopped[0]:
                check_storage()
                readiness = read_source_readiness(directory,contract)
                if readiness['all_ready']:
                    break
                exited = [name for name,child in children.items() if name!='preview' and child.poll() is not None]
                if recorder.poll() is not None or (exited and (not args.diagnostic or manual_drive and 'wheel' in exited)):
                    raise RuntimeError('ALL_SOURCES_NOT_READY: '+json.dumps(dict(
                        missing=readiness['missing'],invalid=readiness['invalid'],exited=exited)))
                if time.monotonic()>ready_deadline:
                    if args.diagnostic:
                        failures.append('DIAGNOSTIC_MISSING_SOURCES: '+json.dumps(readiness['missing']))
                        break
                    raise RuntimeError('ALL_SOURCES_NOT_READY: '+json.dumps(dict(
                        missing=readiness['missing'],invalid=readiness['invalid'],exited=exited)))
                time.sleep(.1)
            if stopped[0]:
                raise RuntimeError('USER_STOPPED_DURING_PREPARATION')
            start_ns = time.monotonic_ns()
            end_ns = start_ns+int(args.duration*1e9)
            manifest['acquisition_window'] = dict(start_monotonic_ns=start_ns,end_monotonic_ns=end_ns,
                requested_duration_ns=int(args.duration*1e9),policy='ALL_SOURCES_READY_THEN_COMMON_WINDOW',
                observed_common_ready_ns=readiness['observed_common_ready_ns'])
            atomic_json(directory/'capture_manifest.json',manifest)
            progress(directory,'RECORDING',window=manifest['acquisition_window'])
            deadline = end_ns/1e9
            preview_reported = False
            while time.monotonic() < deadline and not stopped[0]:
                check_storage()
                if recorder.poll() is not None:
                    raise RuntimeError('RECORDER_EXITED')
                if manual_drive and children['manual_ui'].poll() is not None:
                    if children['manual_ui'].poll() != 0:
                        raise RuntimeError('MANUAL_UI_FAILED: '+str(children['manual_ui'].poll()))
                    stop_reason = 'MANUAL_UI_WINDOW_CLOSED'
                    break
                if preview and children['preview'].poll() is not None and not preview_reported:
                    progress(directory,'PREVIEW_EXITED',exit_code=children['preview'].poll(),recording_continues=True)
                    preview_reported = True
                exited = [name for name, process in children.items() if name!='preview' and process.poll() is not None]
                if exited and (not args.diagnostic or manual_drive and 'wheel' in exited):
                    raise RuntimeError('SOURCE_EXITED: '+','.join(exited))
                time.sleep(.1)
            if stopped[0]:
                stop_reason = 'USER_STOP_REQUEST'
                failures.append('USER_INTERRUPTED_BEFORE_REQUESTED_DURATION')
            if stop_reason == 'REQUESTED_DURATION_REACHED':
                # Retain the first sample at/after the target end for each
                # source, so sparse polling is not mistaken for a short window.
                tail_deadline = time.monotonic()+max(s['max_gap_ns'] for s in contract['sources'].values())/1e9+1.
                while True:
                    check_storage()
                    readiness = read_source_readiness(directory,contract)
                    if readiness['all_ready'] and all(v['last_valid_monotonic_ns']>=end_ns for v in readiness['sources'].values()):
                        break
                    if stopped[0] or recorder.poll() is not None or time.monotonic()>tail_deadline:
                        raise RuntimeError('CAPTURE_TAIL_WINDOW_NOT_COVERED')
                    time.sleep(.05)
        except BaseException as exc:
            stop_reason = 'CAPTURE_FAILURE'
            failures.append(type(exc).__name__+': '+str(exc))
            print('采集启动或运行失败：'+str(exc)+'；日志将保存在 '+str(final_directory/'logs'),
                  file=sys.stderr,flush=True)
        finally:
            try:
                if staging!='memory' and destination_guard is not None: destination_guard.check()
                progress(directory,'STOPPING_SOURCES',reason=stop_reason)
            except Exception as error:
                failures.append('CAPTURE_PROGRESS_WRITE_FAILED: '+str(error))
            try:
                results.update(stop_capture_children(children, recorder, manual_drive=manual_drive))
            finally:
                for sig, handler in old_handlers.items():
                    signal.signal(sig, handler)
        for name in commands:
            results.setdefault(name, 'NOT_STARTED')
        results.setdefault('recorder', 'NOT_STARTED')
        if staging=='memory':
            # No COMPLETE marker is emitted from RAM. Transfer occurs after all
            # source/recorder owners have drained, with shared RUN unchanged.
            manifest.update(process_exit_codes=results,stop_reason=stop_reason,
                            status='PARTIAL',recording_complete=False)
            try:
                check_storage(force=True,closing=True)
                progress(directory,'TRANSFERRING')
                transfer_capture(directory,final_directory,manifest,**destination_options)
                manifest['storage_staging']['durable_archive']=True
                progress(final_directory,'VERIFYING')
                code=finalize_capture(final_directory,RUN,manifest,hashes,results,failures,stop_reason,**destination_options)
            except BaseException as error:
                return retain_staging_failure(directory,final_directory,manifest,error,**destination_options)
            if code==0:
                final_manifest=json.loads((final_directory/'capture_manifest.json').read_text(encoding='utf-8'))
                if final_manifest.get('recording_complete') is True and final_manifest.get('status')=='COMPLETE':
                    try:
                        if destination_guard is not None: destination_guard.check()
                        remove_memory_staging(directory,args.session)
                        final_manifest['storage_staging']['cleanup']='REMOVED_AFTER_DURABLE_AUDIT'
                    except Exception as error:
                        final_manifest['storage_staging'].update(cleanup='RETAINED_CLEANUP_FAILED',cleanup_error=str(error))
                    if destination_guard is not None: destination_guard.check()
                    atomic_json(final_directory/'capture_manifest.json',final_manifest)
            return code
        if destination_guard is not None:
            try: destination_guard.check()
            except ValueError as error:
                print(str(error)+'; stopped capture retained without writing an unverified destination',file=sys.stderr)
                return 2
        return finalize_capture(final_directory, RUN, manifest, hashes, results, failures, stop_reason,**destination_options)


def main(argv=None):
    from .cli import name
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=PROFILES, required=True)
    parser.add_argument('--session', type=name, required=True)
    parser.add_argument('--duration', type=int, required=True)
    parser.add_argument('--staging',choices=('memory','disk'),default='memory',
                        help='memory: tmpfs capture, then stopped-source durable transfer and audit (default); disk: direct archive')
    parser.add_argument('--output-root',type=Path,
                        help='override config/storage.json archive root; requires --required-output-uuid; no local fallback')
    parser.add_argument('--required-output-uuid',
                        help='filesystem UUID required for the external archive for the entire session')
    parser.add_argument('--recorder-queue-size', type=int, default=256,
                        help='bounded recorder writer queue items, 1..8192 (default: 256)')
    parser.add_argument('--dry-run', action='store_true', help='optional identity diagnostics and advisory capacity estimate; no device opened')
    parser.add_argument('--preview',action='store_true',help='Independent subscribed native lidar and camera views; never another device reader')
    parser.add_argument('--diagnostic', action='store_true', help='explicit incomplete diagnostic capture; missing sources remain errors')
    parser.add_argument('--manual-drive', action='store_true',
                        help='authorize this session user WASD/hybrid hand-push; one wheel owner; no EKF/SLAM')
    args = parser.parse_args(argv)
    if (args.output_root is None)!=(args.required_output_uuid is None):
        parser.error('--output-root and --required-output-uuid must be supplied together')
    if not 1 <= args.duration <= 3600:
        parser.error('--duration must be 1..3600 seconds')
    if not 1 <= args.recorder_queue_size <= 8192:
        parser.error('--recorder-queue-size must be 1..8192')
    return run_capture(args)


if __name__ == '__main__':
    raise SystemExit(main())
