#!/usr/bin/env python3
"""Bounded sequential V4L2 frame reads on verified Orin USB camera ports.

No image files, ROS publishers, control bus access, camera property setters or
persistent camera changes. USB port labels do not claim physical directions.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path('/home/nvidia/wheelchair')


def node(port):
    return Path('/dev/v4l/by-path') / f'platform-3610000.usb-usb-0:3.{port}:1.0-video-index0'


def verify(port):
    path = node(port)
    properties = subprocess.check_output(['udevadm', 'info', '--query=property', '--name', str(path)], text=True, timeout=5)
    values = dict(line.split('=', 1) for line in properties.splitlines() if '=' in line)
    expected = {'ID_VENDOR_ID': '0bda', 'ID_MODEL_ID': '5858',
                'ID_PATH': f'platform-3610000.usb-usb-0:3.{port}:1.0',
                'ID_SERIAL_SHORT': '000000000011'}
    if any(values.get(k) != v for k, v in expected.items()) or ':capture:' not in values.get('ID_V4L_CAPABILITIES', ''):
        raise RuntimeError('Camera port identity/capture capability changed')
    return path, expected


def read_frames(port):
    import cv2
    path, identity = verify(port)
    cv2.setNumThreads(1)
    cap = cv2.VideoCapture(str(path), cv2.CAP_V4L2)
    try:
        if not cap.isOpened():
            raise RuntimeError('V4L2 camera could not be opened')
        samples = []
        for _ in range(8):
            ok, frame = cap.read()
            if not ok or frame is None or frame.size == 0:
                raise RuntimeError('Camera did not return a decoded frame')
            samples.append({'shape': list(frame.shape), 'dtype': str(frame.dtype),
                            'sha256': hashlib.sha256(frame.tobytes()).hexdigest(),
                            'mean': float(frame.mean()), 'stddev': float(frame.std()),
                            'host_read_done_ns': time.time_ns()})
        return {'status': 'DECODED_FRAMES_READ', 'port': port, 'device': str(path),
                'resolved_node': str(path.resolve()), 'identity': identity,
                'opencv_version': cv2.__version__, 'backend': cap.getBackendName(),
                'reported_fps': cap.get(cv2.CAP_PROP_FPS), 'samples': samples,
                'physical_direction': 'UNKNOWN', 'images_saved': False,
                'visual_quality_verified': False, 'ros_or_rviz_verified': False}
    finally:
        cap.release()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, choices=range(1, 5), help=argparse.SUPPRESS)
    args = parser.parse_args()
    if (Path.cwd().resolve() != ROOT or subprocess.check_output(['hostname'], text=True).strip() != 'ubuntu'
            or subprocess.check_output(['id', '-un'], text=True).strip() != 'nvidia'
            or os.uname().machine != 'aarch64'):
        raise RuntimeError('This test requires verified ubuntu/nvidia/aarch64 in the project root')
    if args.port:
        print(json.dumps(read_frames(args.port), allow_nan=False))
        return 0
    report = {'test': 'sequential_four_usb_camera_read', 'started_ns': time.time_ns(),
              'capture_scope': '8_DECODED_FRAMES_PER_PORT_SEQUENTIAL_NO_IMAGE_STORAGE',
              'simultaneous_streaming_verified': False, 'cameras': []}
    for port in range(1, 5):
        with (ROOT / f'.phase1_runtime/locks/camera-usb3_{port}.lock').open('a') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                path, _ = verify(port)
                busy = subprocess.run(['fuser', str(path.resolve())], capture_output=True, text=True, timeout=5)
                if busy.returncode != 1 or busy.stdout.strip() or busy.stderr.strip():
                    raise RuntimeError('Camera is occupied or occupancy check is inconclusive')
                child = subprocess.run([sys.executable, '-s', __file__, '--port', str(port)],
                                       capture_output=True, text=True, timeout=12)
                if child.returncode:
                    raise RuntimeError(child.stderr[-1600:] or 'camera reader failed')
                report['cameras'].append(json.loads(child.stdout))
            except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as error:
                report['cameras'].append({'port': port, 'status': 'FAIL', 'error': str(error)})
    report['finished_ns'] = time.time_ns()
    report['status'] = 'PASS' if all(row['status'] == 'DECODED_FRAMES_READ' for row in report['cameras']) else 'FAIL'
    folder = ROOT / 'reports/cameras'
    folder.mkdir(exist_ok=True)
    output = folder / ('read_' + time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()) + '.json')
    with output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    print(json.dumps({'status': report['status'], 'report': str(output), 'cameras': [
        {k: v for k, v in row.items() if k != 'samples'} | {'frame_count': len(row.get('samples', [])),
            'first_shape': row['samples'][0]['shape'] if row.get('samples') else None} for row in report['cameras']]}, indent=2))
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
