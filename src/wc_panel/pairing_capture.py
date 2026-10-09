"""Prepare bounded immutable raw frames from a completed ordinary capture.

Only offline SQLite reads occur here. Sensor acquisition belongs to the existing
capture supervisor; COMPLETE is a transport/durability result, not proof that
the rig or scene was physically stationary.
"""
from contextlib import ExitStack
from datetime import datetime
import bisect
import hashlib
import json
import math
from pathlib import Path
import secrets
import sqlite3

import numpy as np

from .storage import ordinary, resolve_user_destination
from wc_runtime.storage_policy import StoragePolicy


MAX_SOURCE_BYTES = 64 * 1024**3
MAX_FRAMES = 60000
MAX_METADATA_BYTES = 32 * 1024**2
MAX_CDR_BYTES = 8 * 1024**2
MAX_POINTS = 20000


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _json(path, limit=MAX_METADATA_BYTES):
    path = ordinary(path)
    _require(path.is_file() and path.stat().st_size <= limit, '缺少或过大的录制清单：' + str(path))
    raw = path.read_bytes()
    _require(len(raw) <= limit, '读取期间清单超过大小上限')
    return json.loads(raw)


def _source(path):
    path = ordinary(path)
    _require(path.is_file() and path.stat().st_size <= MAX_SOURCE_BYTES, '源文件缺失或超过 64 GiB：' + str(path))
    before = path.stat()
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024**2), b''):
            digest.update(block)
    after = path.stat()
    signature = lambda row: (row.st_dev, row.st_ino, row.st_size, row.st_mtime_ns)
    _require(signature(before) == signature(after), '读取期间原始文件发生改变：' + str(path))
    return dict(path=str(path), bytes=after.st_size, sha256=digest.hexdigest())


def _relative(root, name):
    _require(isinstance(name, str) and name and not Path(name).is_absolute()
             and '..' not in Path(name).parts and '\\' not in name,
             '录制清单包含无效的相对路径')
    result = ordinary(root/name)
    _require(result.is_relative_to(root), '录制清单路径超出录制目录')
    return result


def _journals(source):
    result, paths = {}, []
    for side in ('left', 'right'):
        directories = [ordinary(path) for path in (source/'sources/lidar'/side).glob('*') if path.is_dir()]
        _require(len(directories) == 1, side + ' 雷达必须有唯一的已结束原始流')
        path = ordinary(directories[0]/'source_frames.jsonl')
        _require(path.is_file() and path.stat().st_size <= MAX_METADATA_BYTES,
                 side + ' 雷达缺少原始帧记录或记录过大')
        rows = {}
        with path.open(encoding='utf-8') as stream:
            for line in stream:
                row = json.loads(line)
                key = (row.get('sensor_id'), row.get('stream_epoch'), row.get('sequence'))
                _require(isinstance(key[0], str) and key[0] and isinstance(key[1], str) and key[1]
                         and type(key[2]) is int and key[2] >= 0 and key not in rows,
                         side + ' 原始流身份无效或重复')
                _require(type(row.get('host_monotonic_ns')) is int and row['host_monotonic_ns'] > 0,
                         '缺少单调到达时间，无法建立可核查的同次录制帧对')
                rows[key] = row
                _require(len(rows) <= MAX_FRAMES, '标定准备最多读取每侧 60000 帧；请使用短静态录制')
        _require(rows and len({key[:2] for key in rows}) == 1, side + ' 原始流为空或混合设备/流标识')
        result[side] = rows
        paths.append(path)
    _require(next(iter(result['left']))[0] != next(iter(result['right']))[0], '左右雷达必须具有不同的设备身份')
    return result, paths


def _ros_decoder():
    from rclpy.serialization import deserialize_message
    from wc_interfaces.msg import SourceFrame
    return lambda raw: deserialize_message(raw, SourceFrame)


def _metadata(message, side, session, journals):
    flags = dict(item.split('=', 1) for item in message.diagnostic_flags if '=' in item)
    key = (message.sensor_id, message.stream_epoch, int(message.frame_sequence))
    _require(message.session_id == session and message.side == side and key in journals,
             '录包帧身份与录制会话、雷达或原始流不一致')
    _require(message.units == 'm' and message.coordinate_convention == 'FLU',
             '需要米制 FLU 原始点云；不会自动猜测单位或坐标轴')
    _require(flags.get('representation') == 'before_host_filter', '点云不是记录的 raw 原始帧')
    row = journals[key]
    host = int(message.host_receive_time.sec) * 10**9 + int(message.host_receive_time.nanosec)
    _require(host == row.get('host_receive_ns') and int(message.host_monotonic_ns) == row['host_monotonic_ns'],
             '原始流与录包中的到达时间不一致')
    _require(isinstance(row.get('raw_sha256'), str) and len(row['raw_sha256']) == 64,
             '原始流缺少 raw 点云校验值')
    return dict(session_id=session, side=side, sensor_id=key[0], epoch=key[1], sequence=key[2],
                host_ns=host, monotonic_ns=row['host_monotonic_ns'], flags=flags,
                units='m', coordinate_convention='FLU', source_config_hash=message.source_config_hash,
                payload_sha256=row['raw_sha256'], raw_valid_count=row.get('raw_valid_count'))


def _cloud(message, metadata):
    from wc_sensors.pointcloud import decode_pointcloud2, validate_pointcloud2
    cloud = message.cloud
    layout = validate_pointcloud2(cloud)
    _require(0 < layout['point_count'] <= MAX_POINTS, '所选帧点数超出标定工具上限')
    raw = bytes(cloud.data)
    _require(hashlib.sha256(raw).hexdigest() == metadata['payload_sha256'], '原始点云与采集端记录的校验值不一致')
    xyz = decode_pointcloud2(cloud)
    valid = np.isfinite(xyz).all(axis=1) & np.any(xyz != 0, axis=1)
    _require(valid.any(), '所选帧没有有效点；请录制有静态物体且视野重叠的场景')
    rows = [[float(value) if np.isfinite(value) and np.any(point != 0) else None for value in point] for point in xyz]
    flags, organized = metadata['flags'], None
    intensity = layout['fields'].get('intensity')
    try:
        width, height = int(flags.get('sdk_width', '0')), int(flags.get('sdk_height', '0'))
    except (ValueError, TypeError):
        width = height = 0
    if (flags.get('sdk_point_order') == 'row_major' and width > 0 and height > 0
            and width * height == len(rows) and intensity and intensity['count'] == 1):
        dtype = np.dtype(('>' if layout['is_bigendian'] else '<') + intensity['dtype'])
        values = np.ndarray((layout['height'], layout['width']), dtype=dtype, buffer=raw,
            offset=intensity['offset'], strides=(layout['row_step'], layout['point_step'])).reshape(-1)
        organized = dict(width=width, height=height, order='row_major', quantity='return_amplitude',
            amplitude=[float(value) if np.isfinite(value) and value >= 0 else None for value in values],
            source='recorded_raw_SourceFrame.cloud.intensity', payload_sha256=metadata['payload_sha256'])
    return rows, organized


def select_pairs(left,right,count=3,max_dt_ns=100_000_000):
    """Nearest distinct arrival pairs, spread across this one static capture."""
    _require(left and right,'双雷达必须各有有效原始帧')
    times=[row['host_ns'] for row in right]
    candidates=[];used=set()
    for i,row in enumerate(left):
        at=bisect.bisect_left(times,row['host_ns'])
        options=[j for j in (at-1,at) if 0<=j<len(times)]
        j=min(options,key=lambda k:(abs(row['host_ns']-times[k]),k))
        delta=abs(row['host_ns']-times[j])
        if delta<=max_dt_ns and j not in used:
            used.add(j);candidates.append((i,j,delta))
    _require(candidates,'没有满足主机到达时间差上限的左右帧对')
    wanted=min(count,len(candidates))
    indices=[len(candidates)//2] if wanted==1 else [round(k*(len(candidates)-1)/(wanted-1)) for k in range(wanted)]
    return [candidates[k] for k in indices]


def prepare_recording(project_root, dataset, output=None, *, pair_count=3, max_pair_dt_ms=100., decoder=None):
    """Return the exact new prepared file; never update a latest/default pointer.

    A decoder injection permits byte-level SQLite fixture tests without ROS.
    Production uses the installed SourceFrame message definition. Host monotonic
    proximity is explicitly not exposure synchronization or a static guarantee.
    """
    from wc_calibration.organized_input import validate_organized
    _require(type(pair_count) is int and 1 <= pair_count <= 6, '请选择 1 至 6 组原始帧对')
    _require(isinstance(max_pair_dt_ms, (int, float)) and not isinstance(max_pair_dt_ms, bool)
             and math.isfinite(max_pair_dt_ms) and 0 <= max_pair_dt_ms <= 1000,
             '到达时间差上限必须在 0 至 1000 ms 范围内')
    project = ordinary(Path(project_root).absolute())
    policy = StoragePolicy(project)
    policy.check()
    source, guard = resolve_user_destination(project, dataset)
    manifest_path = source/'capture_manifest.json'
    manifest_record = _source(manifest_path)
    manifest = _json(manifest_path)
    _require(manifest.get('status') == 'COMPLETE' and manifest.get('recording_complete') is True,
             '只允许准备已正常收尾、完整性为 COMPLETE 的录制；PARTIAL 数据保持原样')
    session = manifest.get('session_id')
    _require(isinstance(session, str) and session, '录制缺少明确会话身份')
    bag = _relative(source, manifest.get('bag_path', 'bag'))
    bags = sorted(ordinary(path) for path in bag.glob('*.db3'))
    _require(bags and not any(path.with_name(path.name + suffix).exists()
                            for path in bags for suffix in ('-wal', '-journal')),
             '原始 SQLite 录包缺失或尚未关闭数据库')
    journals, journal_paths = _journals(source)
    source_paths = [manifest_path, *bags, *journal_paths]
    records = {str(path.relative_to(source)): _source(path) for path in source_paths}
    _require(records['capture_manifest.json'] == manifest_record and _json(manifest_path) == manifest,
             '读取期间录制清单改变，尚不能准备点云')
    _require(_journals(source)[0] == journals, '读取期间原始流记录改变')
    destination = policy.resolve(output or ('reports/calibration/panel_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + secrets.token_hex(4)))
    _require(any(destination.is_relative_to(root) and destination != root for root in policy.data_roots()),
             '准备文件必须位于配置的数据或报告目录')
    _require(not destination.exists() and not destination.is_relative_to(source) and not source.is_relative_to(destination),
             '准备目录必须是新的目录，且不能覆盖或包含原始录制')
    decode = decoder or _ros_decoder()
    frames, seen, connections = {side: [] for side in journals}, {side: set() for side in journals}, {}
    with ExitStack() as stack:
        for path in bags:
            guard.check(); policy.check()
            db = sqlite3.connect(path.resolve().as_uri() + '?mode=ro', uri=True)
            stack.callback(db.close); db.execute('PRAGMA query_only=ON')
            connections[path.name] = db
            for topic_id, name, kind in db.execute('SELECT id,name,type FROM topics'):
                side = next((side for side in journals if name == '/wc_mapping/lidar_' + side + '/source_frame'), None)
                if side is None:
                    continue
                _require(kind == 'wc_interfaces/msg/SourceFrame', '原始雷达话题类型不符')
                for message_id, length in db.execute('SELECT id,length(data) FROM messages WHERE topic_id=? ORDER BY id', (topic_id,)):
                    _require(type(length) is int and 0 < length <= MAX_CDR_BYTES, '单帧 CDR 数据过大或为空')
                    stored = db.execute('SELECT data FROM messages WHERE id=? AND length(data)=? AND length(data)<=?',
                                        (message_id, length, MAX_CDR_BYTES)).fetchone()
                    _require(stored is not None, '原始帧大小在读取期间改变')
                    cdr = stored[0]
                    message = decode(cdr)
                    metadata = _metadata(message, side, session, journals[side])
                    key = (metadata['sensor_id'], metadata['epoch'], metadata['sequence'])
                    _require(key not in seen[side], '录包存在重复的原始帧身份')
                    seen[side].add(key)
                    _require(len(seen[side]) <= MAX_FRAMES, '标定准备帧数超过上限；请使用短静态录制')
                    if metadata['raw_valid_count'] is not None and metadata['raw_valid_count'] <= 0:
                        continue
                    frames[side].append(dict(metadata, bag=path.name, message_id=message_id,
                        cdr_sha256=hashlib.sha256(cdr).hexdigest()))
            guard.check()
        for side in journals:
            _require(seen[side] == set(journals[side]) and frames[side],
                     side + ' 雷达录包与原始流记录不完整，或没有有效原始帧')
            frames[side].sort(key=lambda row: (row['monotonic_ns'], row['sequence']))
        pairs = select_pairs(*[[dict(row, host_ns=row['monotonic_ns']) for row in frames[side]]
                             for side in ('left', 'right')], count=pair_count, max_dt_ns=round(max_pair_dt_ms * 1_000_000))
        scenes = []
        for li, ri, delta in pairs:
            scene = dict(id=session+'_raw_L'+str(frames['left'][li]['sequence'])+'_R'+str(frames['right'][ri]['sequence']),
                pair_dt_ns=delta, time_quality='ARRIVAL_NEAREST_NOT_SYNCHRONIZED',
                static_quality='MANUAL_STATIC_SCENE_ASSUMPTION', physical_static_independently_verified=False,
                static_suggestion=dict(status='MANUAL_STATIC_SCENE_ASSUMPTION', static_validated=False, segments=[]),
                selected_raw_frames={}, organized={})
            for side, index in (('left', li), ('right', ri)):
                metadata = frames[side][index]
                stored = connections[metadata['bag']].execute('SELECT data FROM messages WHERE id=? AND length(data)<=?',
                                                               (metadata['message_id'], MAX_CDR_BYTES)).fetchone()
                _require(stored and hashlib.sha256(stored[0]).hexdigest() == metadata['cdr_sha256'], '选定原始帧在读取期间改变')
                message = decode(stored[0])
                scene[side], layout = _cloud(message, metadata)
                if layout:
                    scene['organized'][side] = layout
                scene['selected_raw_frames'][side] = metadata
            if set(scene['organized']) != {'left', 'right'}:
                scene.pop('organized')
                scene['amplitude_availability'] = 'UNAVAILABLE: recording lacks both explicit SDK row-major layouts and amplitude fields'
            else:
                validate_organized(scene['organized'], scene)
            scenes.append(scene)
    prepared = dict(schema_version=1, status='PREPARED_NOT_VALIDATED', source_mode='real', units='m',
        coordinate_conventions=dict(left='FLU', right='FLU'),
        sensor_ids={side: frames[side][0]['sensor_id'] for side in frames},
        initial_T_left_right=None, training=scenes, validation=[], live_eligible=False,
        time_validated=False, independent_validation_performed=False,
        input_preparation=dict(kind='completed_capture_raw_sqlite_v1', dataset=str(source), session_id=session,
            sources=records, coordinate_transform_applied=False, mechanical_initial_applied=False,
            selection=dict(method='nearest_host_monotonic_then_spread', requested_pair_count=pair_count,
                           selected_pair_count=len(scenes), max_pair_dt_ms=max_pair_dt_ms),
            static_quality='MANUAL_STATIC_SCENE_ASSUMPTION', hardware_synchronization_claimed=False,
            same_capture_multiple_frames_are_not_independent_scenes=True),
        retention=dict(original_inputs_retained=True, original_inputs_modified=False,
                       prepared_is_selected_frame_copy_not_full_capture=True))
    raw = (json.dumps(prepared, ensure_ascii=False, allow_nan=False, separators=(',', ':')) + '\n').encode('utf-8')
    _require(len(raw) <= 128_000_000, '准备文件超出工具上限，请减少帧对')
    for path in source_paths:
        _require(_source(path) == records[str(path.relative_to(source))], '准备期间原始录制发生改变，未发布点云输入')
    guard.check(); policy.check()
    destination.mkdir(parents=True, exist_ok=False)
    # Exclusive partial files are retained for inspection if the volume fails;
    # prepared.json is only published after all original bytes are rechecked.
    import os
    with (destination/'prepared.json.partial').open('xb') as stream:
        stream.write(raw); stream.flush(); os.fsync(stream.fileno())
    policy.check(); guard.check()
    os.replace(destination/'prepared.json.partial', destination/'prepared.json')
    result = dict(schema_version=1, status='READY_FOR_OFFLINE_PICKING', prepared_input=str(destination/'prepared.json'),
        prepared_file_sha256=hashlib.sha256(raw).hexdigest(), dataset=str(source), scene_ids=[row['id'] for row in scenes],
        source_files=records, organized_amplitude_available=all('organized' in row for row in scenes),
        static_quality='MANUAL_STATIC_SCENE_ASSUMPTION', original_inputs_modified=False,
        output=str(destination), live_eligible=False)
    with (destination/'manifest.json').open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2); stream.write('\n')
    return result
