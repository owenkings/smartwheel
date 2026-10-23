"""Read a bounded completed capture for offline multi-pose plane calibration.

No sensor or motion owner is started.  Arrival pairing and temporal averaging
do not establish physical stillness, hardware synchronization, or accuracy.
The original bag and journals are opened read-only and retained in place.
"""
from contextlib import ExitStack
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import warnings

import numpy as np


SIDES = ('left', 'right')
WIDTH, HEIGHT = 160, 60
POINTS = WIDTH * HEIGHT
MAX_FRAMES = 1000
PAIR_COUNT = 20
MIN_PAIRS = 5
MAX_PAIR_DT_NS = 50_000_000
MIN_VALID_FRACTION = 0.90
MAX_RAY_CHANGE_DEG = 1e-4
MAX_MANIFEST_BYTES = 32 * 1024**2
MAX_BAG_BYTES = 4 * 1024**3


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _ordinary(value):
    path = Path(value).absolute()
    _require('..' not in path.parts, 'Source path must not contain parent traversal')
    for item in (path, *path.parents):
        _require(not item.is_symlink() and not getattr(item, 'is_junction', lambda: False)(),
                 'Source paths must not contain links: ' + str(item))
    return path


def _helpers():
    from wc_panel import pairing_capture
    return pairing_capture


def _policy(project_root):
    from wc_runtime.storage_policy import StoragePolicy
    policy = StoragePolicy(_ordinary(project_root))
    policy.check()
    return policy


def _configuration_hashes(policy):
    return {name: hashlib.sha256(raw).hexdigest()
            for name, raw in policy.configuration_files.items()}


def _dataset(policy, dataset):
    source = _ordinary(policy.resolve(dataset))
    experiments = _ordinary(policy.resolve('data/experiments'))
    _require(source.is_relative_to(experiments) and source != experiments and source.is_dir(),
             'Only an explicit existing capture under current storage data/experiments is allowed')
    return source


def _read_manifest(path):
    _require(path.is_file() and path.stat().st_size <= MAX_MANIFEST_BYTES,
             'Missing or oversized capture manifest')
    def invalid_constant(value):
        raise ValueError('Nonfinite manifest value: ' + value)
    value = json.loads(path.read_bytes(), parse_constant=invalid_constant)
    _require(isinstance(value, dict) and value.get('status') == 'COMPLETE'
             and value.get('recording_complete') is True,
             'Only a finalized COMPLETE recording is accepted')
    _require(isinstance(value.get('session_id'), str) and value['session_id'],
             'Explicit capture session identity is required')
    window = value.get('acquisition_window', {})
    start, end = window.get('start_monotonic_ns'), window.get('end_monotonic_ns')
    _require(type(start) is int and type(end) is int and start > 0 and end > start,
             'A formal monotonic acquisition window is required')
    _require(10_000_000_000 <= end - start <= 60_000_000_000,
             'Use a completed 10 to 60 second static calibration capture')
    requested = window.get('requested_duration_ns')
    if requested is not None:
        _require(type(requested) is int and requested == end - start,
                 'Formal window and requested duration disagree')
    return value


def _source_files(source, manifest, helpers):
    relative = manifest.get('bag_path', 'bag')
    _require(isinstance(relative, str) and relative and not Path(relative).is_absolute()
             and '..' not in Path(relative).parts and '\\' not in relative,
             'Invalid bag path in capture manifest')
    bag = _ordinary(source / relative)
    _require(bag.is_relative_to(source) and bag.is_dir(), 'Bag directory is outside capture')
    bags = sorted(_ordinary(path) for path in bag.glob('*.db3'))
    _require(0 < len(bags) <= 16, 'Expected one bounded closed SQLite capture')
    _require(sum(path.stat().st_size for path in bags) <= MAX_BAG_BYTES,
             'SQLite capture exceeds the 4 GiB offline plane-calibration read budget')
    _require(not any(path.with_name(path.name + suffix).exists()
                     for path in bags for suffix in ('-wal', '-journal', '-shm')),
             'SQLite capture is not closed; WAL, SHM or journal is present')
    # The shared journal helper permits much longer captures. Bound its input
    # before it materializes rows, rather than rejecting 60000 rows afterwards.
    for side in SIDES:
        streams = [_ordinary(path) for path in (source / 'sources/lidar' / side).glob('*')
                   if path.is_dir()]
        _require(len(streams) == 1, 'Each sensor needs one closed source stream: ' + side)
        journal = _ordinary(streams[0] / 'source_frames.jsonl')
        _require(journal.is_file() and journal.stat().st_size <= MAX_MANIFEST_BYTES,
                 'Missing or oversized source journal: ' + side)
        with journal.open('rb') as stream:
            for index, line in enumerate(stream, 1):
                _require(index <= MAX_FRAMES and len(line) <= 64 * 1024,
                         'Capture exceeds the 1000 raw frames per side journal bound')
    journals, journal_paths = helpers._journals(source)
    _require(all(0 < len(journals[side]) <= MAX_FRAMES for side in SIDES),
             'Capture exceeds the 1000 raw frames per side limit')
    paths = [source / 'capture_manifest.json', *bags, *journal_paths]
    metadata = _ordinary(bag / 'metadata.yaml')
    if metadata.exists():
        paths.append(metadata)
    records = {path.relative_to(source).as_posix(): helpers._source(path) for path in paths}
    return bags, journals, records


def _check_raw_message(message, metadata):
    from wc_sensors.pointcloud import validate_pointcloud2
    flags = metadata['flags']
    _require(flags.get('sdk_point_order') == 'row_major'
             and flags.get('sdk_width') == str(WIDTH) and flags.get('sdk_height') == str(HEIGHT),
             'Every raw frame must explicitly declare 160 x 60 row-major SDK pixels')
    cloud = message.cloud
    _require(type(cloud.width) is int and type(cloud.height) is int
             and cloud.width > 0 and cloud.height > 0 and cloud.width * cloud.height == POINTS,
             'Raw PointCloud2 count differs from the declared SDK layout')
    layout = validate_pointcloud2(cloud)
    _require(layout['point_count'] == POINTS and layout['fields'].get('intensity', {}).get('count') == 1,
             'Every raw frame needs valid scalar XYZ and amplitude PointCloud2 fields')
    _require(hashlib.sha256(bytes(cloud.data)).hexdigest() == metadata['payload_sha256'],
             'Raw payload SHA256 differs from the source journal')


def _pairs(frames, midpoint, end):
    # Pair only within the latter half. Unique nearest-arrival matches retain
    # provenance; this is explicitly not an exposure synchronization claim.
    filtered = {side: [row for row in frames[side]
                       if midpoint <= row['monotonic_ns'] <= end] for side in SIDES}
    _require(all(filtered.values()), 'Both sensors need frames in the latter formal half-window')
    result, used = [], set()
    right_times = np.asarray([row['monotonic_ns'] for row in filtered['right']], dtype=np.int64)
    for left in filtered['left']:
        at = int(np.searchsorted(right_times, left['monotonic_ns']))
        candidates = [j for j in (at - 1, at) if 0 <= j < len(right_times) and j not in used]
        if not candidates:
            continue
        selected = min(candidates, key=lambda j: (abs(int(right_times[j]) - left['monotonic_ns']), j))
        delta = abs(int(right_times[selected]) - left['monotonic_ns'])
        if delta <= MAX_PAIR_DT_NS:
            used.add(selected)
            result.append((left, filtered['right'][selected], delta))
    _require(len(result) >= MIN_PAIRS, 'Fewer than 5 distinct pairs within 50 ms in the latter window')
    count = min(PAIR_COUNT, len(result))
    indices = np.rint(np.linspace(0, len(result) - 1, count)).astype(int)
    return [result[int(index)] for index in indices]


def _summary(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    return dict(count=int(a.size), median=float(np.median(a)) if a.size else None,
                p95=float(np.percentile(a, 95)) if a.size else None,
                maximum=float(np.max(a)) if a.size else None)


def _aggregate(frames, amplitude):
    xyz = np.asarray(frames, dtype=np.float64)
    amp = np.asarray(amplitude, dtype=np.float64)
    _require(xyz.ndim == 3 and xyz.shape[1:] == (POINTS, 3)
             and amp.shape == xyz.shape[:2] and MIN_PAIRS <= len(xyz) <= PAIR_COUNT,
             'Invalid bounded organized temporal input')
    ranges = np.linalg.norm(xyz, axis=2)
    valid = np.isfinite(xyz).all(axis=2) & np.isfinite(ranges) & (ranges > 0)
    reference_index = len(xyz) // 2
    reference_valid = valid[reference_index]
    unit = np.full_like(xyz, np.nan)
    np.divide(xyz, ranges[:, :, None], out=unit, where=valid[:, :, None])
    reference_rays = unit[reference_index]
    cross_norm = np.linalg.norm(np.cross(unit, reference_rays[None, :, :]), axis=2)
    dot = np.einsum('kij,ij->ki', unit, reference_rays)
    angle_deg = np.degrees(np.arctan2(cross_norm, dot))
    ray_changed = np.any(valid & (angle_deg > MAX_RAY_CHANGE_DEG), axis=0)
    minimum = int(math.ceil(MIN_VALID_FRACTION * len(xyz)))
    enough = valid.sum(axis=0) >= minimum
    usable = reference_valid & enough & ~ray_changed
    ranges[~valid] = np.nan
    amp[~valid | ~np.isfinite(amp) | (amp < 0)] = np.nan
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        radial_median = np.nanmedian(ranges, axis=0)
        radial_std = np.nanstd(ranges, axis=0, ddof=0)
        amplitude_median = np.nanmedian(amp, axis=0)
    median_xyz = reference_rays * radial_median[:, None]
    median_xyz[~usable] = np.nan
    amplitude_median[~usable | (np.isfinite(amp).sum(axis=0) < minimum)] = np.nan
    report = dict(method='PER_PIXEL_MEDIAN_RADIAL_RANGE_ON_REFERENCE_RAW_UNIT_RAY',
                  reference_selected_frame_index=reference_index,
                  selected_frame_count=len(xyz), minimum_valid_samples=minimum,
                  minimum_valid_fraction=MIN_VALID_FRACTION,
                  ray_change_limit_deg=MAX_RAY_CHANGE_DEG,
                  ray_changed_pixel_count=int(ray_changed.sum()),
                  insufficient_valid_pixel_count=int((~enough).sum()),
                  reference_invalid_pixel_count=int((~reference_valid).sum()),
                  output_valid_pixel_count=int(usable.sum()),
                  radial_population_std_m=_summary(radial_std[usable]),
                  interpretation='TEMPORAL_REPEATABILITY_NOT_ACCURACY_OR_STATIC_PROOF',
                  spatial_pixel_averaging=False)
    return median_xyz, amplitude_median, report


def _json_values(values):
    a = np.asarray(values)
    return [[float(x) if np.isfinite(x) else None for x in row] for row in a] if a.ndim == 2 else [
        float(x) if np.isfinite(x) else None for x in a]


def load_capture(project_root, dataset, *, decoder=None):
    """Return a source-bound temporal preview and in-memory original frames.

    ``decoder`` is a test seam for serialized SourceFrame fixtures. It changes
    no source identity, time-window, payload hash, storage, or layout checks.
    ``frames`` contains only 5..20 selected pairs, never the complete recording.
    """
    helpers = _helpers()
    policy = _policy(project_root)
    source = _dataset(policy, dataset)
    manifest_path = source / 'capture_manifest.json'
    manifest_before = helpers._source(manifest_path)
    manifest = _read_manifest(manifest_path)
    session = manifest['session_id']
    _require(source.name == session, 'Capture directory and session identity disagree')
    bags, journals, source_records = _source_files(source, manifest, helpers)
    _require(source_records['capture_manifest.json'] == manifest_before,
             'Capture manifest changed before loading')
    start = manifest['acquisition_window']['start_monotonic_ns']
    end = manifest['acquisition_window']['end_monotonic_ns']
    midpoint = (start + end) // 2
    decode = decoder or helpers._ros_decoder()
    frames = {side: [] for side in SIDES}
    seen = {side: set() for side in SIDES}
    connections = {}
    with ExitStack() as stack:
        for path in bags:
            policy.check()
            db = sqlite3.connect(path.as_uri() + '?mode=ro', uri=True)
            stack.callback(db.close)
            db.execute('PRAGMA query_only=ON')
            connections[path.name] = db
            for topic_id, topic, kind in db.execute('SELECT id,name,type FROM topics'):
                side = next((side for side in SIDES
                             if topic == '/wc_mapping/lidar_' + side + '/source_frame'), None)
                if side is None:
                    continue
                _require(kind == 'wc_interfaces/msg/SourceFrame', 'Unexpected raw topic message type')
                for message_id, length in db.execute(
                        'SELECT id,length(data) FROM messages WHERE topic_id=? ORDER BY id', (topic_id,)):
                    _require(type(length) is int and 0 < length <= helpers.MAX_CDR_BYTES,
                             'Raw message exceeds the CDR byte bound')
                    stored = db.execute('SELECT data FROM messages WHERE id=? AND length(data)=?',
                                        (message_id, length)).fetchone()
                    _require(stored is not None, 'Raw message disappeared during read')
                    cdr = stored[0]
                    message = decode(cdr)
                    metadata = helpers._metadata(message, side, session, journals[side])
                    key = (metadata['sensor_id'], metadata['epoch'], metadata['sequence'])
                    _require(key not in seen[side], 'Duplicate raw frame identity in capture')
                    seen[side].add(key)
                    _require(len(seen[side]) <= MAX_FRAMES, 'Raw frame bound exceeded')
                    _check_raw_message(message, metadata)
                    frames[side].append(dict(metadata, bag=path.name, message_id=message_id,
                                             cdr_sha256=hashlib.sha256(cdr).hexdigest()))
        for side in SIDES:
            _require(seen[side] == set(journals[side]), 'Bag and source journal frames disagree: ' + side)
            frames[side].sort(key=lambda row: (row['monotonic_ns'], row['sequence']))
            _require(len({row['source_config_hash'] for row in frames[side]}) == 1
                     and frames[side][0]['source_config_hash'],
                     'Source configuration identity changes or is absent: ' + side)
        selected = _pairs(frames, midpoint, end)
        selected_metadata = {side: [] for side in SIDES}
        xyz_by_side, amplitude_by_side = {side: [] for side in SIDES}, {side: [] for side in SIDES}
        for left, right, delta in selected:
            for side, metadata in (('left', left), ('right', right)):
                stored = connections[metadata['bag']].execute(
                    'SELECT data FROM messages WHERE id=? AND length(data)<=?',
                    (metadata['message_id'], helpers.MAX_CDR_BYTES)).fetchone()
                _require(stored and hashlib.sha256(stored[0]).hexdigest() == metadata['cdr_sha256'],
                         'Selected CDR bytes changed while reading')
                message = decode(stored[0])
                repeated = helpers._metadata(message, side, session, journals[side])
                _require(repeated == {key: value for key, value in metadata.items()
                                     if key not in ('bag', 'message_id', 'cdr_sha256')},
                         'Selected raw metadata changed while reading')
                cloud, organized = helpers._cloud(message, metadata)
                _require(organized is not None and organized.get('width') == WIDTH
                         and organized.get('height') == HEIGHT and organized.get('order') == 'row_major'
                         and len(cloud) == POINTS and len(organized['amplitude']) == POINTS,
                         'Selected frame lacks the exact organized raw XYZ/amplitude layout')
                xyz = np.asarray(cloud, dtype=float)
                xyz[~np.isfinite(xyz).all(axis=1) | ~np.any(xyz != 0, axis=1)] = np.nan
                xyz_by_side[side].append(xyz)
                amplitude_by_side[side].append(np.asarray(organized['amplitude'], dtype=float))
                selected_metadata[side].append(metadata)
        policy.check()
    result = dict(capture_id=session, source_mode='real', units='m',
                  sensor_ids={side: frames[side][0]['sensor_id'] for side in SIDES},
                  coordinate_conventions={side: 'FLU' for side in SIDES},
                  width=WIDTH, height=HEIGHT, clouds={}, amplitude={}, frames={}, frame_metadata=selected_metadata,
                  provenance=dict(project_root=str(policy.project_root), dataset=str(source), sources=source_records,
                                  storage_configuration_sha256=_configuration_hashes(policy),
                                  window=dict(manifest['acquisition_window']),
                                  formal_window=dict(start_monotonic_ns=start, end_monotonic_ns=end),
                                  selected_window=dict(start_monotonic_ns=midpoint, end_monotonic_ns=end),
                                  source_frame_counts={side: len(frames[side]) for side in SIDES},
                                  source_config_hashes={side: frames[side][0]['source_config_hash'] for side in SIDES},
                                  pair_selection=dict(method='UNIQUE_NEAREST_MONOTONIC_ARRIVAL_THEN_UNIFORM_SPREAD',
                                                      pair_count=len(selected), max_pair_dt_ns=MAX_PAIR_DT_NS,
                                                      actual_pair_dt_ns=[int(pair[2]) for pair in selected]),
                                  static_quality='MANUAL_STATIC_SCENE_ASSUMPTION',
                                  physical_static_independently_verified=False,
                                  hardware_synchronization_claimed=False,
                                  original_inputs_modified=False,
                                  same_capture_frames_are_not_independent_poses=True,
                                  aggregation={}))
    for side in SIDES:
        xyz = np.stack(xyz_by_side[side])
        amp = np.stack(amplitude_by_side[side])
        median_xyz, median_amp, report = _aggregate(xyz, amp)
        _require(report['output_valid_pixel_count'] > 0,
                 'No pixels survive temporal validity and unit-ray consistency checks: ' + side)
        result['frames'][side] = xyz
        result['clouds'][side] = _json_values(median_xyz)
        result['amplitude'][side] = _json_values(median_amp)
        result['provenance']['aggregation'][side] = report
    assert_sources(result)
    return result


def assert_sources(capture):
    """Fail if current storage identity/config or any bound original bytes change."""
    helpers = _helpers()
    provenance = capture['provenance']
    policy = _policy(provenance['project_root'])
    _require(_configuration_hashes(policy) == provenance['storage_configuration_sha256'],
             'Storage configuration changed since capture load')
    source = _dataset(policy, provenance['dataset'])
    _require(source.name == capture['capture_id'], 'Capture identity changed since load')
    records = provenance['sources']
    _require(isinstance(records, dict) and 'capture_manifest.json' in records
             and any(name.endswith('.db3') for name in records), 'Source provenance is incomplete')
    for name, expected in records.items():
        _require(isinstance(name, str), 'Invalid bound source path')
        relative = Path(name)
        _require(not relative.is_absolute() and '..' not in relative.parts
                 and '\\' not in name, 'Invalid bound source path')
        path = _ordinary(source / relative)
        _require(path.is_relative_to(source) and helpers._source(path) == expected,
                 'Bound original source changed: ' + name)
        if path.suffix == '.db3':
            _require(not any(path.with_name(path.name + suffix).exists()
                             for suffix in ('-wal', '-journal', '-shm')),
                     'Bound SQLite source is no longer closed')
    manifest = _read_manifest(source / 'capture_manifest.json')
    _require(manifest['session_id'] == capture['capture_id'], 'Bound session identity changed')
    # Detect a new/split bag or a newly introduced journal as well as changes
    # to already-bound files, while touching only this explicit capture.
    _, journals, current_records = _source_files(source, manifest, helpers)
    _require(current_records == records, 'Bound source file set changed since capture load')
    _require({side: next(iter(journals[side]))[0] for side in SIDES} == capture['sensor_ids'],
             'Bound sensor identities changed since capture load')
    policy.check()
