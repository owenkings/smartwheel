"""Bounded OpenCV acquisition in an owned child process, with shared latest frame.

Only the child opens V4L2. Explicit child join waits have deadlines; a child stuck in the
kernel may outlive them and must be reported as a cleanup failure. The child
retains the common port flock until its device closes.
"""
import ctypes
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import shutil
import signal
import stat
import subprocess
import time

from .config import CameraError, port_path


def verify_properties(camera, properties):
    expected = {'ID_VENDOR_ID': camera['vid'], 'ID_MODEL_ID': camera['pid'],
                'ID_SERIAL_SHORT': camera['serial'],
                'ID_PATH': f"platform-3610000.usb-usb-0:3.{camera['port']}:1.0"}
    if any(properties.get(key) != val for key, val in expected.items()) or ':capture:' not in properties.get('ID_V4L_CAPABILITIES', ''):
        raise CameraError('USB port identity or capture capability mismatch')
    return expected


def verify_device(camera):
    path = port_path(camera['port'])
    resolved = path.resolve(strict=True)
    device_stat = resolved.stat()
    if not stat.S_ISCHR(device_stat.st_mode):
        raise CameraError('camera target is not a character device')
    output = subprocess.check_output(['udevadm', 'info', '--query=property', '--name', str(path)],
                                     text=True, timeout=5)
    properties = dict(row.split('=', 1) for row in output.splitlines() if '=' in row)
    expected = verify_properties(camera, properties)
    return {'device': str(path), 'resolved_node': str(resolved), 'identity': expected,
            'st_rdev': device_stat.st_rdev, 'st_ino': device_stat.st_ino}


def require_unoccupied(device):
    executable = shutil.which('fuser')
    if executable is None or not Path(device).is_absolute():
        raise CameraError('fuser and an absolute verified device path are required')
    result = subprocess.run([executable, str(device)], capture_output=True, text=True, timeout=5)
    if result.returncode != 1 or result.stdout.strip() or result.stderr.strip():
        raise CameraError('camera is occupied or occupancy check is inconclusive: '+result.stdout.strip()+result.stderr.strip())


class CameraLease:
    def __init__(self, camera, run_root, session_id):
        self.camera, self.run_root, self.session_id = camera, Path(run_root), session_id
        self.lock = None

    def __enter__(self):
        import fcntl
        if not self.run_root.is_absolute():
            raise CameraError('run-root must be an absolute shared path')
        locks = self.run_root.resolve() / 'locks'
        locks.mkdir(parents=True, exist_ok=True)
        name = locks / f"camera-usb3_{self.camera['port']}.lock"
        descriptor = os.open(str(name), os.O_RDWR | os.O_CREAT | os.O_CLOEXEC | os.O_NOFOLLOW, 0o600)
        self.lock = os.fdopen(descriptor, 'a+', encoding='utf-8')
        try:
            fcntl.flock(self.lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            identity = verify_device(self.camera)
            # Alternate V4L2 indices can belong to the same physical camera.
            # Do not open index0 while another application owns its sibling.
            by_path = port_path(self.camera['port'])
            siblings = sorted(by_path.parent.glob(by_path.name.rsplit('index', 1)[0]+'index[0-9]'))
            if by_path not in siblings:
                raise CameraError('verified camera by-path disappeared')
            for sibling in siblings:
                require_unoccupied(str(sibling.resolve(strict=True)))
            if verify_device(self.camera) != identity:
                raise CameraError('camera identity changed during preflight')
            self.lock.seek(0)
            self.lock.truncate()
            json.dump({'pid': os.getpid(), 'session_id': self.session_id,
                       'role': self.camera['role'], **identity}, self.lock)
            self.lock.flush()
            return identity
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def __exit__(self, *_):
        if self.lock is not None:
            self.lock.close()
            self.lock = None


def fourcc_name(value):
    if not math.isfinite(value) or value < 0 or value > 0xffffffff:
        raise CameraError('invalid FOURCC readback')
    return ''.join(chr((int(value) >> (8*i)) & 255) for i in range(4))


def configure_capture(cap, cv, profile):
    requests = [('fourcc', cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*profile['fourcc'])),
                ('width', cv.CAP_PROP_FRAME_WIDTH, profile['width']),
                ('height', cv.CAP_PROP_FRAME_HEIGHT, profile['height']),
                ('capture_fps', cv.CAP_PROP_FPS, profile['capture_fps'])]
    setter_results = {key: bool(cap.set(prop, value)) for key, prop, value in requests}
    actual = {key: float(cap.get(prop)) for key, prop, _ in requests}
    if not all(math.isfinite(value) and value > 0 for value in actual.values()):
        raise CameraError('camera capture property readback is unavailable/nonfinite')
    actual['fourcc'] = fourcc_name(actual['fourcc'])
    if actual['fourcc'] != profile['fourcc'] or actual['width'] != profile['width'] or actual['height'] != profile['height']:
        raise CameraError('capture format/shape differs from requested profile: '+json.dumps(actual))
    if not 1 <= actual['capture_fps'] <= 240:
        raise CameraError('negotiated camera FPS is outside supported diagnostic bounds')
    # Some UVC devices negotiate another cadence. Report it, never secretly
    # change dimensions, format, or the selected profile to make acquisition pass.
    actual['fps_matches_request'] = abs(actual['capture_fps']-profile['capture_fps']) <= .05
    actual['backend'] = cap.getBackendName()
    actual['setters_returned'] = setter_results
    return actual


class SharedFrame:
    """One bounded, overwrite-latest BGR buffer; no image file or feeder thread."""
    def __init__(self, context, width, height):
        self.capacity = width*height*3
        self.pixels = context.RawArray(ctypes.c_ubyte, self.capacity)
        # capture sequence, host wall/monotonic ns, read duration ns, width, height, byte count
        self.meta = context.RawArray(ctypes.c_int64, 7)
        self.lock = context.Lock()

    def put(self, data, sequence, arrival_ns, mono_ns, read_ns, width, height):
        if len(data) != width*height*3 or len(data) > self.capacity:
            raise CameraError('decoded BGR frame size exceeds the requested bounded buffer')
        with self.lock:
            ctypes.memmove(ctypes.addressof(self.pixels), data, len(data))
            self.meta[:] = (sequence, arrival_ns, mono_ns, read_ns, width, height, len(data))

    def snapshot(self, after_sequence):
        # A child can be killed during copying. Never wait on its mutex forever.
        if not self.lock.acquire(timeout=.01):
            return None
        try:
            sequence, arrival, mono, read_ns, width, height, length = self.meta[:]
            if sequence <= after_sequence:
                return None
            if length != width*height*3 or not 0 < length <= self.capacity:
                raise CameraError('shared frame metadata is inconsistent')
            return {'capture_sequence': sequence, 'host_arrival_ns': arrival,
                    'host_monotonic_ns': mono, 'host_read_elapsed_ns': read_ns,
                    'width': width, 'height': height,
                    'data': ctypes.string_at(ctypes.addressof(self.pixels), length)}
        finally:
            self.lock.release()


def arm_parent_death(parent_pid):
    if os.name != 'posix' or not Path('/proc/self').exists():
        raise CameraError('real camera acquisition requires Linux')
    libc = ctypes.CDLL(None, use_errno=True)
    libc.prctl.argtypes = [ctypes.c_int, ctypes.c_ulong, ctypes.c_ulong, ctypes.c_ulong, ctypes.c_ulong]
    libc.prctl.restype = ctypes.c_int
    if libc.prctl(1, signal.SIGKILL, 0, 0, 0) != 0:  # PR_SET_PDEATHSIG
        raise CameraError('could not arm owned camera child cleanup')
    if os.getppid() != parent_pid:
        raise CameraError('camera parent exited before child startup')
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)


def capture_worker(camera, profile, run_root, session_id, shared, stop, status, parent_pid, archive_root=None):
    cap = None
    archive = None
    source_error = None
    stream_epoch = session_id+'-'+str(os.getpid())
    try:
        arm_parent_death(parent_pid)
        import cv2
        cv2.setNumThreads(1)
        with CameraLease(camera, run_root, session_id) as identity:
            try:
                cap = cv2.VideoCapture(identity['resolved_node'], cv2.CAP_V4L2)
                if not cap.isOpened():
                    raise CameraError('V4L2 camera failed to open')
                if verify_device(camera) != identity:
                    raise CameraError('camera identity changed while opening')
                actual = configure_capture(cap, cv2, profile)
                if archive_root is not None:
                    from wc_runtime.source_archive import CameraArchive
                    archive = CameraArchive(archive_root, camera['role'], stream_epoch)
                    from wc_runtime.source_archive import atomic_json
                    atomic_json(archive.root/'capture_configuration.json', dict(
                        requested_profile=profile, actual=actual, identity=identity,
                        camera=camera, opencv_version=cv2.__version__,
                        preview_rotation_applied_to_archive=False))
                status.send({'event': 'READY', 'stream_epoch': stream_epoch, 'actual': actual, 'device': identity,
                             'opencv_version': cv2.__version__})
                sequence = 0
                while not stop.is_set():
                    begin = time.monotonic_ns()
                    ok, frame = cap.read()
                    mono, wall = time.monotonic_ns(), time.time_ns()
                    if not ok or frame is None or frame.size == 0:
                        raise CameraError('V4L2 read failed; no automatic reopen or profile fallback')
                    if frame.dtype.name != 'uint8' or frame.shape != (profile['height'], profile['width'], 3):
                        raise CameraError('decoded frame shape/type differs from requested BGR8')
                    sequence += 1
                    if archive is not None:
                        archive.append(frame.tobytes(), sequence=sequence, host_receive_ns=wall,
                            host_monotonic_ns=mono, read_start_monotonic_ns=begin,
                            width=frame.shape[1], height=frame.shape[0],
                            device=identity['device'], preview_rotation_deg=camera['rotate_deg'])
                    if camera['rotate_deg'] == 180:
                        frame = cv2.rotate(frame, cv2.ROTATE_180)
                    shared.put(frame.tobytes(), sequence, wall, mono, mono-begin, frame.shape[1], frame.shape[0])
            finally:
                # Keep the device lock across release(), including if release blocks.
                if cap is not None:
                    release_begin = time.monotonic_ns()
                    cap.release()
                    release_complete = time.monotonic_ns()
                    cap = None
                    status.send({'event': 'RELEASE_COMPLETE',
                                 'host_release_start_monotonic_ns': release_begin,
                                 'host_release_complete_monotonic_ns': release_complete,
                                 'host_release_elapsed_ns': release_complete-release_begin})
    except BaseException as error:
        source_error = str(error)
        try:
            status.send({'event': 'ERROR', 'error': str(error)[:1600]})
        except (BrokenPipeError, EOFError, OSError):
            pass
        raise SystemExit(1)
    finally:
        if archive is not None:
            try:
                status.send({'event': 'ARCHIVE_COMPLETE', 'archive': archive.close(source_error=source_error)})
            except Exception as error:
                try:
                    status.send({'event': 'ERROR', 'error': 'camera archive: '+str(error)})
                except (OSError, EOFError):
                    pass
        status.close()


class CaptureProcess:
    def __init__(self, camera, profile, run_root, session_id, *, context=None, worker=None, archive_root=None):
        self.context = context or mp.get_context('spawn')
        self.shared = SharedFrame(self.context, profile['width'], profile['height'])
        self.stop_event = self.context.Event()
        self.receiver, sender = self.context.Pipe(duplex=False)
        worker_args = (camera, profile, str(run_root), session_id, self.shared, self.stop_event, sender, os.getpid())
        if archive_root is not None:
            worker_args += (str(archive_root),)
        self.process = self.context.Process(target=worker or capture_worker,
            args=worker_args,
            name='camera-usb3_'+str(camera['port']), daemon=True)
        self.sender = sender
        self.started = False
        self.cleanup_result = None
        self.normal_close_budget_s = 25 if archive_root is not None else 5

    def start(self):
        self.process.start()
        self.started = True
        self.sender.close()

    def events(self):
        result = []
        while True:
            try:
                if not self.receiver.poll():
                    break
                result.append(self.receiver.recv())
            except (EOFError, BrokenPipeError):
                break
        return result

    def close(self):
        if self.cleanup_result is not None and not self.started:
            return self.cleanup_result
        result = {'status': 'NOT_STARTED', 'termination': 'not_started',
                  'final_exit_code': None, 'alive_after_cleanup': False,
                  'late_events': [], 'errors': [], 'owned_pid': self.process.pid,
                  'process_observations': [], 'late_event_drain': 'NOT_STARTED',
                  'normal_join_budget_s': self.normal_close_budget_s, 'terminate_join_budget_s': 1, 'kill_join_budget_s': 1,
                  'stop_requested_host_monotonic_ns': None, 'reap_observed_host_monotonic_ns': None,
                  'stop_to_reap_observed_elapsed_ns': None, 'stop_to_cleanup_observed_elapsed_ns': None}
        try:
            if self.started:
                result['termination'] = 'normal'
                result['stop_requested_host_monotonic_ns'] = time.monotonic_ns()
                self.stop_event.set()
                # Observed camera shutdown waited in usb_unlocked_enable_lpm and
                # outlived the former 0.5 s allowance. Allow normal close time
                # without weakening forced-termination failure classification.
                self.process.join(self.normal_close_budget_s)
                if self.process.is_alive():
                    result['process_observations'].append(process_observation(self.process.pid, 'before_terminate'))
                    result['termination'] = 'terminate'
                    self.process.terminate()
                    self.process.join(1)
                if self.process.is_alive():
                    result['process_observations'].append(process_observation(self.process.pid, 'before_kill'))
                    result['termination'] = 'kill'
                    self.process.kill()
                    self.process.join(1)
                result['alive_after_cleanup'] = self.process.is_alive()
                result['final_exit_code'] = self.process.exitcode
                cleanup_observed = time.monotonic_ns()
                result['stop_to_cleanup_observed_elapsed_ns'] = cleanup_observed-result['stop_requested_host_monotonic_ns']
                if not result['alive_after_cleanup']:
                    # This is when the parent observed/reaped exit, not an
                    # exposure timestamp or the exact kernel exit instant.
                    result['reap_observed_host_monotonic_ns'] = cleanup_observed
                    result['stop_to_reap_observed_elapsed_ns'] = result['stop_to_cleanup_observed_elapsed_ns']
                # A still-live sender could be partway through a message. poll()
                # alone does not make recv() bounded, so do not drain it here.
                # This is already FAIL and must not claim complete late events.
                if result['alive_after_cleanup']:
                    result['process_observations'].append(process_observation(self.process.pid, 'after_kill_still_alive'))
                    result['late_event_drain'] = 'SKIPPED_UNREAPED_CHILD'
                else:
                    # Drain after join: read/release can fail after the last ROS tick.
                    result['late_event_drain'] = 'DRAINED_AFTER_CHILD_EXIT'
                    result['late_events'] = self.events()
                result['errors'] = [str(event.get('error', 'unknown capture error'))[:1600]
                                    for event in result['late_events'] if event.get('event') == 'ERROR']
                if result['termination'] != 'normal':
                    result['errors'].append('forced_'+result['termination'])
                if result['final_exit_code'] != 0:
                    result['errors'].append('capture_child_exit_code='+str(result['final_exit_code']))
                if result['alive_after_cleanup']:
                    result['errors'].append('owned camera child cleanup did not complete')
                result['status'] = 'FAIL' if result['errors'] else 'PASS'
                if not result['alive_after_cleanup']:
                    self.process.close()
                    self.started = False
            self.cleanup_result = result
            if result['alive_after_cleanup']:
                raise CameraError('owned camera child cleanup did not complete')
            return result
        finally:
            self.cleanup_result = result
            self.sender.close()
            self.receiver.close()


def process_observation(pid, stage, *, proc_root=Path('/proc')):
    """Read only small status/wchan files for this owned child, without sudo.

    Capture the state before escalation and immediately after an unsuccessful
    kill; a later inspection may miss a transient kernel wait. Missing or denied
    proc entries are evidence limitations, never proof that cleanup succeeded.
    """
    result = {'stage': stage, 'pid': pid, 'host_monotonic_ns': time.monotonic_ns(),
              'host_wall_ns': time.time_ns(), 'status_fields': {}, 'wchan': None, 'read_errors': []}
    if type(pid) is not int or pid <= 0:
        result['read_errors'].append('invalid owned child pid')
        return result
    wanted = {'Name', 'State', 'Tgid', 'Pid', 'PPid', 'TracerPid', 'Threads',
              'SigPnd', 'ShdPnd', 'SigBlk', 'SigIgn', 'SigCgt'}
    for name, limit in (('status', 16384), ('wchan', 1024)):
        try:
            with (Path(proc_root)/str(pid)/name).open('r', encoding='utf-8', errors='replace') as stream:
                value = stream.read(limit+1)
            if len(value) > limit:
                raise ValueError('proc entry exceeds bounded diagnostic size')
            if name == 'status':
                result['status_fields'] = {key: text.strip() for row in value.splitlines()
                                           if ':' in row for key, text in [row.split(':', 1)] if key in wanted}
            else:
                result['wchan'] = value.strip()
        except (OSError, ValueError) as error:
            result['read_errors'].append(name+': '+type(error).__name__+': '+str(error)[:300])
    return result
