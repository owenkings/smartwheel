"""Select a machine timing policy before freezing it into a recording.

Replay consumers must load the archived policy directly, never this selector.
A present local policy is authoritative: invalid local evidence cannot fall
back to another sensor's base profile.
"""
from pathlib import Path

from wc_imu.device_time import load_timing_profile


def load_project_timing_profile(root, *, sensor_id):
    configuration = Path(root)/'config'
    for name in ('imu_timing.local.json', 'imu_timing.json'):
        path = configuration/name
        if (path.exists() or path.is_symlink() or
                getattr(path, 'is_junction', lambda: False)()):
            return load_timing_profile(path, sensor_id=sensor_id)
    return None
