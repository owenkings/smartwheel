"""Prepare offline amplitude/XYZ picking from one audited dual-lidar NPZ capture."""
import argparse
import bisect
import hashlib
import json
import math
import os
from pathlib import Path
import zipfile

import numpy as np

from wc_calibration.core import CalibrationError
from wc_calibration.organized_input import MAX_ORGANIZED_POINTS, validate_organized
from wc_calibration.picker import MAX_INPUT_BYTES, _json_bytes, _json_loads
from .project_paths import project_root
from .storage_policy import StoragePolicy

SOURCE_NAMES = ('capture.json', 'left_frames.json', 'right_frames.json', 'left.npz', 'right.npz')
MAX_SOURCE_BYTES = 1024 * 1024 * 1024
MAX_EXPANDED_BYTES = 512 * 1024 * 1024


def source_record(path):
    """Hash exact bytes and reject a concurrent modification during the read."""
    if not path.is_file() or path.is_symlink() or path.stat().st_size > MAX_SOURCE_BYTES:
        raise CalibrationError('source must be a bounded ordinary file: ' + str(path))
    before = path.stat()
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    after = path.stat()
    signature = lambda s: (s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns)
    if signature(before) != signature(after):
        raise CalibrationError('input changed while hashing: ' + str(path))
    return {'path': str(path), 'bytes': after.st_size, 'sha256': digest.hexdigest()}


def _read_json(path):
    if path.stat().st_size > 32 * 1024 * 1024:
        raise CalibrationError('source metadata exceeds 32 MiB')
    return _json_loads(path.read_text(encoding='utf-8'))


def _dimension(value):
    if type(value) is str and value.isdecimal():
        value = int(value)
    if type(value) is not int or value < 1:
        raise CalibrationError('explicit positive SDK dimensions required')
    return value


def validate_frames(value, side, session):
    frames = value.get('raw') if isinstance(value, dict) else None
    if not isinstance(frames, list) or not frames or len(frames) > 100000:
        raise CalibrationError('nonempty bounded raw frame metadata required')
    identities, keys, dimensions, hosts = set(), set(), set(), []
    for frame in frames:
        if not isinstance(frame, dict) or frame.get('session_id') != session or frame.get('side') != side:
            raise CalibrationError('mixed session or side in source frame metadata')
        sensor = frame.get('sensor_id')
        if not isinstance(sensor, str) or not sensor.strip() or not isinstance(frame.get('epoch'), str) or not frame['epoch']:
            raise CalibrationError('explicit sensor and stream epoch required')
        if frame.get('units') != 'm' or frame.get('coordinate_convention') != 'FLU':
            raise CalibrationError('source requires metres and FLU; no automatic axis or unit conversion')
        for field in ('sequence', 'host_ns', 'sdk_frame_id', 'device_raw'):
            if type(frame.get(field)) is not int or frame[field] < (1 if field == 'host_ns' else 0):
                raise CalibrationError('invalid source frame integer: ' + field)
        flags = frame.get('flags', {})
        if not isinstance(flags, dict) or flags.get('representation') != 'before_host_filter' or flags.get('sdk_point_order') != 'row_major':
            raise CalibrationError('organized input requires explicit raw row_major layout')
        width, height = _dimension(flags.get('sdk_width')), _dimension(flags.get('sdk_height'))
        if width * height > MAX_ORGANIZED_POINTS or frame.get('point_count') != width * height:
            raise CalibrationError('point count differs from SDK layout or exceeds budget')
        payload = frame.get('payload_sha256')
        if not isinstance(payload, str) or len(payload) != 64 or any(c not in '0123456789abcdef' for c in payload):
            raise CalibrationError('source payload SHA256 required')
        key = (sensor, frame['epoch'], frame['sequence'])
        if key in keys:
            raise CalibrationError('duplicate raw frame identity')
        keys.add(key); identities.add(sensor); dimensions.add((width, height)); hosts.append(frame['host_ns'])
    if len(identities) != 1 or len(dimensions) != 1 or any(a > b for a, b in zip(hosts, hosts[1:])):
        raise CalibrationError('source identity/layout changed or host timestamps are unordered')
    return frames, next(iter(identities)), next(iter(dimensions))


def select_pairs(left, right, count=3, max_dt_ns=100000000, left_frame_index=None):
    if type(count) is not int or not 1 <= count <= 16 or type(max_dt_ns) is not int or max_dt_ns < 0:
        raise CalibrationError('pair count must be 1..16 and time bound nonnegative')
    if left_frame_index is not None and (type(left_frame_index) is not int or not 0 <= left_frame_index < len(left)):
        raise CalibrationError('left-frame-index outside the recorded raw frames')
    right_times = [frame['host_ns'] for frame in right]
    candidates, used = [], set()
    indices = range(len(left)) if left_frame_index is None else [left_frame_index]
    for i in indices:
        at = bisect.bisect_left(right_times, left[i]['host_ns'])
        options = [j for j in (at - 1, at) if 0 <= j < len(right)]
        j = min(options, key=lambda j: (abs(left[i]['host_ns'] - right_times[j]), j))
        dt = abs(left[i]['host_ns'] - right_times[j])
        if dt <= max_dt_ns and j not in used:
            used.add(j); candidates.append((i, j, dt))
    if not candidates:
        raise CalibrationError('no frame pair satisfies the host-arrival time bound')
    wanted = min(count if left_frame_index is None else 1, len(candidates))
    # Spread frames across the available overlap; they remain one physical scene.
    positions = [len(candidates) // 2] if wanted == 1 else [round(k * (len(candidates) - 1) / (wanted - 1)) for k in range(wanted)]
    return [candidates[k] for k in positions]


def payload_hash(xyz, intensity):
    """SourceFrame payload is little-endian packed float32 XYZ then amplitude per point."""
    packed = np.empty((len(xyz), 4), dtype='<f4')
    packed[:, :3] = xyz
    packed[:, 3] = intensity
    return hashlib.sha256(packed.tobytes(order='C')).hexdigest()


def read_selected_npz(path, frames, selected_indices, dimensions):
    with zipfile.ZipFile(path) as archive:
        members = archive.infolist()
        if len({item.filename for item in members}) != len(members) or \
                sum(item.file_size for item in members) > MAX_EXPANDED_BYTES:
            raise CalibrationError('NPZ duplicate members or expanded byte budget exceeded')
    needed = {'raw_xyz', 'raw_intensity', 'frame_sequence', 'host_ns', 'device_raw', 'sdk_frame_id'}
    n, points = len(frames), dimensions[0] * dimensions[1]
    result = {}
    with np.load(path, allow_pickle=False) as archive:
        if not needed.issubset(archive.files):
            raise CalibrationError('NPZ lacks raw XYZ/amplitude or frame identity arrays')
        for array_key, field in (('frame_sequence', 'sequence'), ('host_ns', 'host_ns'),
                                 ('device_raw', 'device_raw'), ('sdk_frame_id', 'sdk_frame_id')):
            values = archive[array_key]
            if values.shape != (n,) or values.dtype.kind not in 'iu' or \
                    any(int(values[i]) != frame[field] for i, frame in enumerate(frames)):
                raise CalibrationError('NPZ frame identity differs from metadata: ' + array_key)
        xyz, intensity = archive['raw_xyz'], archive['raw_intensity']
        if xyz.shape != (n, points, 3) or intensity.shape != (n, points) or \
                xyz.dtype.kind != 'f' or intensity.dtype.kind != 'f' or xyz.dtype.itemsize != 4 or intensity.dtype.itemsize != 4:
            raise CalibrationError('raw arrays must have exact frame/layout dimensions and float32 values')
        for index in sorted(set(selected_indices)):
            if payload_hash(xyz[index], intensity[index]) != frames[index]['payload_sha256']:
                raise CalibrationError('selected raw XYZ/amplitude payload hash differs from source frame')
            rows = [[float(v) if np.isfinite(v) else None for v in row] for row in xyz[index]]
            amp = [float(v) if np.isfinite(v) and v >= 0 else None for v in intensity[index]]
            result[index] = (rows, amp)
    return result


def _fsync_dir(path):
    if os.name == 'posix':
        descriptor = os.open(str(path), os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


def prepare(root, capture_root, output, *, pair_count=3, max_pair_dt_ms=100., left_frame_index=None):
    policy = StoragePolicy(root)
    policy.check()
    source = policy.resolve(capture_root)
    destination = policy.resolve(output)
    allowed = policy.data_roots()
    if not any(destination.is_relative_to(base) and destination != base for base in allowed):
        raise CalibrationError('output must be a new directory under configured data, reports or maps')
    if destination.exists() or destination.is_relative_to(source) or source.is_relative_to(destination):
        raise CalibrationError('output must not exist or overlap the source capture')
    if not math.isfinite(max_pair_dt_ms) or not 0 <= max_pair_dt_ms <= 10000:
        raise CalibrationError('max-pair-dt-ms must be finite in 0..10000')
    sources = {name: policy.resolve(source/name) for name in SOURCE_NAMES}
    records = {name: source_record(path) for name, path in sources.items()}
    capture = _read_json(sources['capture.json'])
    session = capture.get('session') if isinstance(capture, dict) else None
    if not isinstance(session, str) or not session.strip() or capture.get('status') != 'CAPTURE_AUDIT_PASS':
        raise CalibrationError('an explicitly identified CAPTURE_AUDIT_PASS capture is required')
    if capture.get('errors') != [] or not isinstance(capture.get('sides'), dict) or set(capture['sides']) != {'left', 'right'}:
        raise CalibrationError('capture audit must have no errors and explicit left/right sources')
    frames, ids, dimensions = {}, {}, {}
    for side in ('left', 'right'):
        frames[side], ids[side], dimensions[side] = validate_frames(_read_json(sources[side+'_frames.json']), side, session)
    if ids['left'] == ids['right']:
        raise CalibrationError('left and right must have distinct sensor identities')
    pairs = select_pairs(frames['left'], frames['right'], pair_count, round(max_pair_dt_ms * 1000000), left_frame_index)
    clouds = {side: read_selected_npz(sources[side+'.npz'], frames[side],
        [pair[index] for pair in pairs], dimensions[side]) for index, side in enumerate(('left', 'right'))}
    scenes = []
    for left, right, dt in pairs:
        scene = {'id': '%s_raw_L%d_R%d' % (session, left, right),
            'pair_dt_ns': dt, 'time_quality': 'ARRIVAL_NEAREST_NOT_SYNCHRONIZED',
            'physical_static_independently_verified': capture.get('physical_static_independently_verified') is True,
            'organized': {}, 'selected_raw_frames': {}}
        for side, index in (('left', left), ('right', right)):
            rows, amplitude = clouds[side][index]
            if not any(all(v is not None and math.isfinite(v) for v in row)
                       and any(v != 0 for v in row) for row in rows):
                raise CalibrationError('selected %s frame has no finite nonzero XYZ observations' % side)
            frame = frames[side][index]
            scene[side] = rows
            scene['organized'][side] = {'width': dimensions[side][0], 'height': dimensions[side][1],
                'order': 'row_major', 'amplitude': amplitude, 'quantity': 'return_amplitude',
                'array_frame_index': index, 'source_array': 'raw_intensity',
                'source_npz_sha256': records[side+'.npz']['sha256']}
            scene['selected_raw_frames'][side] = dict(frame, array_frame_index=index,
                raw_key=[session, ids[side], frame['epoch'], frame['sequence']],
                npz=records[side+'.npz'], frame_metadata=records[side+'_frames.json'])
        validate_organized(scene['organized'], scene)
        scenes.append(scene)
    prepared = {'schema_version': 1, 'status': 'PREPARED_NOT_VALIDATED', 'source_mode': 'real',
        'units': 'm', 'coordinate_conventions': {'left': 'FLU', 'right': 'FLU'}, 'sensor_ids': ids,
        'initial_T_left_right': None, 'training': scenes, 'validation': [],
        'input_preparation': {'kind': 'audited_npz_organized_raw_v1', 'session_id': session,
            'sources': records, 'selection': {'method': 'nearest_host_ns_then_spread', 'requested_pair_count': pair_count,
                'selected_pair_count': len(pairs), 'max_pair_dt_ms': max_pair_dt_ms, 'left_frame_index': left_frame_index},
            'coordinate_transform_applied': False, 'mechanical_initial_applied': False,
            'array_order_changed': False, 'hardware_synchronization_claimed': False,
            'invalid_json_value_policy': 'Nonfinite XYZ and nonfinite/negative amplitude become null; original NPZ retained',
            'independent_validation_performed': False,
            'same_capture_multiple_frames_are_not_independent_scenes': True},
        'retention': {'original_inputs_retained': True, 'original_inputs_modified': False,
            'prepared_is_selected_frame_copy_not_full_capture': True}, 'live_eligible': False}
    content = _json_bytes(prepared)
    if len(content) > MAX_INPUT_BYTES:
        raise CalibrationError('prepared scenes exceed picker byte budget; reduce --pair-count')
    for name, path in sources.items():
        if source_record(path) != records[name]:
            raise CalibrationError('input changed during preparation: ' + name)
    policy.check()
    destination.mkdir(parents=True, exist_ok=False)
    reserved_identity = (destination.stat().st_dev, destination.stat().st_ino)
    manifest = {'schema_version': 1, 'status': 'READY_FOR_OFFLINE_PICKING', 'prepared_input': 'prepared.json',
        'prepared_file_sha256': hashlib.sha256(content).hexdigest(), 'bytes': len(content),
        'source_files': records, 'scene_ids': [scene['id'] for scene in scenes],
        'original_inputs_retained': True, 'initial_transform_applied': False,
        'independent_validation_performed': False, 'live_eligible': False}
    files = {'prepared.json': content, 'manifest.json': _json_bytes(manifest),
        'README.md': ('# 幅度图与点云联动选点输入\n\n'
            '这是同一次静态场景录制中的原始帧副本，不是标定结果。幅度图不等于深度图。\n'
            'XYZ 单位为米，轴为前 X、左 Y、上 Z；未应用安装外参或镜像。\n'
            '按主机到达时间就近选帧，不表示硬同步；不同帧仍属于同一场景。\n'
            '像素编号=row*width+column；无效 XYZ 和零原点占位不可选。\n'
            '原始录制保留在 manifest.json 所列路径，文件哈希已冻结。\n'
            '默认初值未知；选点求解和 ICP 只产生候选，不能自动替代正式外参。\n').encode('utf-8')}
    try:
        for name, value in files.items():
            policy.check()
            with (destination/('.'+name+'.partial')).open('xb') as stream:
                stream.write(value); stream.flush(); os.fsync(stream.fileno())
        # Recheck every source immediately before publishing prepared.json.
        for name, path in sources.items():
            if source_record(path) != records[name]:
                raise CalibrationError('input changed before commit: ' + name)
        policy.check()
        for name in ('README.md', 'prepared.json', 'manifest.json'):
            os.rename(destination/('.'+name+'.partial'), destination/name)
        _fsync_dir(destination); _fsync_dir(destination.parent)
    except BaseException:
        # A removed USB must never redirect cleanup into a new mount or home path.
        # If identity cannot be proved, retain partials for inspection.
        try:
            policy.check()
            same_path = policy.resolve(destination) == destination
            current = destination.stat()
            if same_path and not destination.is_symlink() and (current.st_dev, current.st_ino) == reserved_identity:
                for name in files:
                    for path in (destination/name, destination/('.'+name+'.partial')):
                        if path.is_file() and not path.is_symlink():
                            path.unlink()
                destination.rmdir()
        except (OSError, ValueError, RuntimeError):
            pass
        raise
    return dict(manifest, output=str(destination), prepared_input=str(destination/'prepared.json'))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture-root', type=Path, required=True, help='Directory containing capture.json and left/right NPZ + frame metadata')
    parser.add_argument('--output', type=Path, required=True, help='New logical reports/... or configured absolute data directory; never overwritten')
    parser.add_argument('--pair-count', type=int, default=3, help='1..16 time-spread pairs from this same capture (default 3)')
    parser.add_argument('--left-frame-index', type=int, help='Choose exactly this zero-based left raw-array frame and its nearest right frame')
    parser.add_argument('--max-pair-dt-ms', type=float, default=100., help='Maximum difference of host arrival times, not a hardware sync tolerance')
    args = parser.parse_args(argv)
    try:
        result = prepare(project_root(), args.capture_root, args.output, pair_count=args.pair_count,
            max_pair_dt_ms=args.max_pair_dt_ms, left_frame_index=args.left_frame_index)
    except (CalibrationError, ValueError, OSError, KeyError, zipfile.BadZipFile) as error:
        parser.exit(2, 'Preparation rejected: ' + str(error) + '\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
