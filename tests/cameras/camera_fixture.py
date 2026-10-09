"""Synthetic worker for process-lifecycle tests; never imports OpenCV or ROS."""
import time


def blocked_worker(camera, profile, run_root, session_id, shared, stop, status, parent_pid):
    status.send({'event': 'READY', 'fixture': 'BLOCKED_SYNTHETIC_NO_DEVICE'})
    time.sleep(60)  # Intentionally ignores stop; owning test must terminate it.


def frame_worker(camera, profile, run_root, session_id, shared, stop, status, parent_pid):
    status.send({'event': 'READY', 'fixture': 'SYNTHETIC_NO_DEVICE'})
    data = bytes([19, 31, 47])*profile['width']*profile['height']
    shared.put(data, 1, time.time_ns(), time.monotonic_ns(), 1234, profile['width'], profile['height'])
    while not stop.wait(.01):
        pass
    status.close()
