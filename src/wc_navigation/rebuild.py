"""Derive a single-left, immutable-clock frontend for offline grid experiments.

No device, ROS node, clock publisher or vehicle control is started here. ROS
message codecs are imported only by the file-derivation entry point. The pure
PointCloud2 helpers accept normal messages or equivalent attribute objects.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import re

import numpy as np


VARIANTS = ('left_normals_ray', 'left_height_ray_05', 'left_height_ray_15')
MAX_POINTS = 2_000_000
MAX_CDR_BYTES = 64 * 1024 * 1024
_SIZES = {1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8}


class NonReferenceOriginError(ValueError):
    """Left points belong to an earlier arrival than this paired reference."""

    def __init__(self, metadata):
        super().__init__('selected left rows have nonzero time offsets; ray origin is not this reference origin')
        self.metadata = metadata


def _integer(value, name, minimum=0):
    if type(value) is not int or value < minimum:
        raise ValueError(name + ' must be an integer >= ' + str(minimum))
    return value


def _finite(value, name, *, positive=False):
    if type(value) not in (int, float) or not math.isfinite(value) or (positive and value <= 0):
        raise ValueError(name + ' must be finite' + (' and positive' if positive else ''))
    return float(value)


def _layout(cloud):
    width = _integer(cloud.width, 'cloud width', 1)
    height = _integer(cloud.height, 'cloud height', 1)
    step = _integer(cloud.point_step, 'cloud point_step', 1)
    row_step = _integer(cloud.row_step, 'cloud row_step', 1)
    if type(cloud.is_bigendian) is not bool or type(cloud.is_dense) is not bool:
        raise ValueError('cloud endian and density flags must be bool')
    count = width * height
    if count > MAX_POINTS or step > 256 or row_step != width * step:
        raise ValueError('bounded contiguous PointCloud2 rows required')
    raw = bytes(cloud.data)
    if len(raw) != row_step * height or len(raw) > MAX_CDR_BYTES:
        raise ValueError('PointCloud2 byte size does not match its contiguous layout')
    fields = {}
    occupied = set()
    for field in cloud.fields:
        if not isinstance(field.name, str) or not field.name or field.name in fields:
            raise ValueError('PointCloud2 fields require unique nonempty names')
        offset = _integer(field.offset, 'field offset')
        datatype = _integer(field.datatype, 'field datatype', 1)
        elements = _integer(field.count, 'field count', 1)
        if datatype not in _SIZES:
            raise ValueError('unsupported PointCloud2 field datatype')
        size = _SIZES[datatype] * elements
        positions = set(range(offset, offset + size))
        if offset + size > step or occupied.intersection(positions):
            raise ValueError('PointCloud2 fields overlap or exceed point_step')
        occupied.update(positions)
        fields[field.name] = field
    for name in ('x', 'y', 'z'):
        f = fields.get(name)
        if f is None or f.datatype not in (7, 8) or f.count != 1:
            raise ValueError('XYZ requires one FLOAT32/FLOAT64 field per axis')
    return raw, count, step, fields


def _field_values(raw, count, step, field, bigendian):
    codes = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
    dtype = np.dtype(('>' if bigendian else '<') + codes[field.datatype])
    return np.ndarray((count,), dtype=dtype, buffer=raw, offset=field.offset, strides=(step,))


def pointcloud_xyz(cloud):
    """Return a checked Nx3 float64 XYZ copy; preserve invalid rows as NaN."""
    raw, count, step, fields = _layout(cloud)
    return np.column_stack([_field_values(raw, count, step, fields[axis], cloud.is_bigendian)
                            for axis in ('x', 'y', 'z')]).astype(float, copy=False)


def _single_left_config(config):
    if not isinstance(config, dict) or config.get('mode') != 'left':
        return False
    selection = config.get('offline_source_selection')
    if not isinstance(selection, dict) or selection.get('selected_sides') != ['left']:
        return False
    other = config.get('offline_lidar_selection')
    return not isinstance(other, dict) or other.get('selected_sides') == ['left']


def select_left_cloud(cloud, *, require_zero_offset=True, source_config=None):
    """Select source_code=1 byte rows, preserving all fields and header metadata.

    A raytraced grid may only use a paired frame whose selected left offset is
    zero. Nonzero-offset left points were deskewed to another reference time;
    their true ray origin is not necessarily [0,0,0]. Callers extracting points
    solely for paired-time localization may explicitly disable that extra gate.
    """
    if type(require_zero_offset) is not bool:
        raise ValueError('require_zero_offset must be bool')
    raw, count, step, fields = _layout(cloud)
    code_field = fields.get('source_code')
    offset_field = fields.get('time_offset_s')
    if code_field is None and offset_field is None:
        if not _single_left_config(source_config):
            raise ValueError('missing source fields require frozen mode=left and selected_sides=[left] provenance')
        # A freshly replayed single-left frontend has its own canonical left
        # arrival stamp and reference origin. Its whole PointCloud2 is retained,
        # including organized dimensions, layout, invalid rows and all bytes.
        result = copy.deepcopy(cloud)
        xyz = pointcloud_xyz(result)
        return result, {
            'selection': 'VALIDATED_SINGLE_LEFT_FRONTEND', 'input_rows': count,
            'selected_rows': count, 'excluded_right_rows': 0,
            'left_time_offset_s': 0.0, 'ray_origin_zero_valid': True,
            'rows_preserved_byte_for_byte': True, 'fields_preserved': True,
            'header_preserved': True, 'coordinate_transform_applied': False,
            'source_data_sha256': _digest(raw), 'derived_data_sha256': _digest(raw),
            'selected_finite_xyz_rows': int(np.isfinite(xyz).all(axis=1).sum()),
            'side_provenance': 'FROZEN_MODE_LEFT_AND_OFFLINE_SOURCE_SELECTION_LEFT',
        }
    if code_field is None or code_field.datatype != 2 or code_field.count != 1:
        raise ValueError('source_code must be one UINT8 field')
    if offset_field is None or offset_field.datatype != 8 or offset_field.count != 1:
        raise ValueError('time_offset_s must be one FLOAT64 field')
    codes = _field_values(raw, count, step, code_field, cloud.is_bigendian)
    offsets = _field_values(raw, count, step, offset_field, cloud.is_bigendian)
    if not set(np.unique(codes).tolist()).issubset({1, 2}) or not (codes == 1).any():
        raise ValueError('source codes must be left/right only and include left')
    if not np.isfinite(offsets).all() or np.any(offsets > 0) or np.any(offsets < -.05):
        raise ValueError('source arrival offsets must be finite and within [-0.05,0] seconds')
    selected = np.flatnonzero(codes == 1)
    left_offsets = offsets[selected]
    if len(np.unique(left_offsets)) != 1:
        raise ValueError('one consistent arrival offset required for the left frame')
    meta = {
        'selection': 'EXPLICIT_SOURCE_CODE_EQUALS_1', 'input_rows': count,
        'selected_rows': int(len(selected)), 'excluded_right_rows': int((codes == 2).sum()),
        'left_time_offset_s': float(left_offsets[0]),
        'ray_origin_zero_valid': bool(np.all(left_offsets == 0.0)),
        'rows_preserved_byte_for_byte': True, 'fields_preserved': True,
        'header_preserved': True, 'coordinate_transform_applied': False,
        'source_data_sha256': hashlib.sha256(raw).hexdigest(),
    }
    if require_zero_offset and not meta['ray_origin_zero_valid']:
        raise NonReferenceOriginError(meta)
    records = np.frombuffer(raw, dtype=np.uint8).reshape(count, step)
    result = copy.deepcopy(cloud)
    result.width, result.height = int(len(selected)), 1
    result.row_step = result.width * step
    result.data = records[selected].tobytes()
    meta['derived_data_sha256'] = hashlib.sha256(bytes(result.data)).hexdigest()
    xyz = pointcloud_xyz(result)
    meta['selected_finite_xyz_rows'] = int(np.isfinite(xyz).all(axis=1).sum())
    return result, meta


def profile_overrides(config, variant):
    """Freeze a declared single-sensor grid variant, never a measured floor."""
    if variant not in VARIANTS:
        raise ValueError('unsupported predeclared rebuild variant: ' + str(variant))
    from wc_runtime.mapping_profile import validate_profile, resolve_native_profile
    frozen = copy.deepcopy(config)
    if frozen.get('motion_model') != 'planar_ekf' or not isinstance(frozen.get('map_profile'), dict):
        raise ValueError('explicit planar_ekf and frozen map_profile required')
    frozen['mode'] = 'left'
    profile = validate_profile(frozen['map_profile'])
    profile['ray_tracing'] = 'single_sensor_only'
    profile['max_ground_height_m'] = .05 if variant == 'left_height_ray_05' else .15
    profile = validate_profile(profile)
    frozen['map_profile'] = profile
    resolved = resolve_native_profile(frozen)
    # Heights and projection remain under the production map_profile authority.
    # Only these three native experiment switches need the driver's allowlist.
    overrides = {'Grid/3D': 'false', 'Grid/RayTracing': 'true',
                 'Grid/NormalsSegmentation': 'true' if variant == 'left_normals_ray' else 'false'}
    return {
        'variant': variant, 'map_profile': profile, 'native_parameter_overrides': overrides,
        'resolved_native_profile_parameters': copy.deepcopy(resolved['parameters']),
        'ground_model': {k: copy.deepcopy(resolved[k]) for k in
                         ('candidate_ground_z_in_reference_m', 'ground_basis', 'ground_height_validated',
                          'reference_sensor', 'ray_origin', 'cloud_xyz_retained')},
        'status': 'EXPERIMENT', 'navigation_validated': False,
        'low_obstacle_limit': 'The configured ground band is an unvalidated flat-floor assumption; '
                              'height-filter variants do not qualify low-obstacle or step detection.',
    }


def validate_frontend_row(row):
    """Check immutable frontend identity without loading ROS message libraries."""
    if not isinstance(row, dict):
        raise ValueError('frontend index row must be an object')
    _integer(row.get('stamp_ns'), 'frontend stamp_ns', 1)
    for name in ('cloud_cdr_file', 'odom_cdr_file'):
        value = row.get(name)
        if not isinstance(value, str) or Path(value).name != value or Path(value).suffix != '.cdr':
            raise ValueError('frontend CDR names must be ordinary basenames')
    for name in ('cloud_cdr_sha256', 'cloud_data_sha256', 'odom_cdr_sha256'):
        if not isinstance(row.get(name), str) or re.fullmatch('[0-9a-f]{64}', row[name]) is None:
            raise ValueError('frontend requires original SHA-256 for ' + name)
    if row.get('frame_id') != 'mapping_reference' or row.get('odom_child') != 'mapping_reference' or row.get('odom_parent') != 'mapping_odom':
        raise ValueError('unexpected frontend map/odometry frames')
    pose = np.asarray(row.get('T_prior_reference'), dtype=float)
    if (pose.shape != (4, 4) or not np.isfinite(pose).all()
            or not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-10, rtol=0)
            or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-5, rtol=0)
            or not math.isclose(float(np.linalg.det(pose[:3, :3])), 1.0, abs_tol=1e-5)):
        raise ValueError('frontend pose must be a finite rigid transform')
    return row['stamp_ns']


def _ordinary(path):
    path = Path(path).absolute()
    if '..' in path.parts or any(p.is_symlink() for p in (path, *path.parents)):
        raise ValueError('ordinary unlinked paths without parent traversal required')
    return path


def _digest(raw):
    return hashlib.sha256(raw).hexdigest()


def _read_bounded(path, limit=MAX_CDR_BYTES):
    if not path.is_file() or path.stat().st_size > limit:
        raise ValueError('missing or oversized source file: ' + str(path))
    return path.read_bytes()


def _stamp(message):
    sec = _integer(message.header.stamp.sec, 'message stamp seconds')
    ns = _integer(message.header.stamp.nanosec, 'message stamp nanoseconds')
    if ns >= 10**9:
        raise ValueError('message nanoseconds exceed one second')
    return sec * 10**9 + ns


def derive_left_frontend(source_cell_dir, destination_cell_dir, profile_variant):
    """Write a fresh derived frontend from checked original paired scan CDRs.

    All variants use the same eligibility rule: a frame is retained only when
    its left arrival is this paired reference (zero offset). Skipped timestamps
    are recorded, never retimed. The prior odometry CDR and pose stay unchanged;
    this is single-lidar map evidence, not a newly estimated single-lidar pose.
    """
    source = _ordinary(source_cell_dir)
    destination = _ordinary(destination_cell_dir)
    if not source.is_dir() or destination.exists() or destination.is_relative_to(source) or source.is_relative_to(destination):
        raise ValueError('fresh destination separate from immutable source cell required')
    config_path, index_path = source / 'runtime_config.json', source / 'frontend/index.jsonl'
    config_raw, index_raw = _read_bounded(config_path, 2 * 1024**2), _read_bounded(index_path, 128 * 1024**2)
    config = json.loads(config_raw)
    profile = profile_overrides(config, profile_variant)
    rows = []
    stamps = set()
    previous = -1
    for line in index_raw.decode('utf-8').splitlines():
        if not line or len(line) > 2 * 1024**2:
            raise ValueError('invalid bounded frontend index line')
        row = json.loads(line)
        stamp = validate_frontend_row(row)
        if stamp in stamps or stamp <= previous:
            raise ValueError('frontend timestamps must be unique and increasing')
        stamps.add(stamp)
        previous = stamp
        rows.append(row)
        if len(rows) > 200_000:
            raise ValueError('frontend frame resource bound exceeded')
    if not rows:
        raise ValueError('empty original frontend')
    # ROS codecs are lazy; importing this module and its pure helpers does not
    # initialize rclpy or start a node, sensor, process or clock publisher.
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    destination.mkdir(parents=True, exist_ok=False)
    frontend = destination / 'frontend'
    frontend.mkdir()
    manifest = {
        'schema_version': 1, 'status': 'DERIVING', 'source_cell': str(source),
        'source_runtime_sha256': _digest(config_raw), 'source_index_sha256': _digest(index_raw),
        'profile': profile, 'selected_side': 'left', 'source_code': None if _single_left_config(config) else 1,
        'eligible_rule': ('VALIDATED_SINGLE_LEFT_FRONTEND_OWN_ARRIVAL_STAMPS' if _single_left_config(config)
                          else 'LEFT_TIME_OFFSET_EXACTLY_ZERO_NO_RETIMING'),
        'source_frames': len(rows), 'retained_frames': 0, 'skipped_nonreference_frames': [],
        'source_cdrs': [], 'hardware_started': False, 'navigation_validated': False,
        'original_recording_modified': False, 'odometry_reestimated': False,
        'odometry_scope': 'UNCHANGED_ORIGINAL_WHEEL_IMU_PRIOR_WITH_ORIGINAL_SOURCE_REFERENCE_TIMES',
    }
    hashes = {}
    try:
        with (frontend / 'index.jsonl').open('x', encoding='utf-8', newline='\n') as stream:
            for original in rows:
                cloud_path = _ordinary(source / 'frontend' / original['cloud_cdr_file'])
                odom_path = _ordinary(source / 'frontend' / original['odom_cdr_file'])
                cloud_raw, odom_raw = _read_bounded(cloud_path), _read_bounded(odom_path)
                if _digest(cloud_raw) != original['cloud_cdr_sha256'] or _digest(odom_raw) != original['odom_cdr_sha256']:
                    raise ValueError('source CDR identity mismatch at ' + str(original['stamp_ns']))
                hashes[str(cloud_path)] = original['cloud_cdr_sha256']
                hashes[str(odom_path)] = original['odom_cdr_sha256']
                cloud = deserialize_message(cloud_raw, PointCloud2)
                if _stamp(cloud) != original['stamp_ns'] or cloud.header.frame_id != original['frame_id']:
                    raise ValueError('source cloud stamp/frame disagrees with index')
                if _digest(bytes(cloud.data)) != original['cloud_data_sha256']:
                    raise ValueError('source cloud data identity mismatch')
                odom = deserialize_message(odom_raw, Odometry)
                if (_stamp(odom) != original['stamp_ns'] or odom.header.frame_id != original['odom_parent']
                        or odom.child_frame_id != original['odom_child']):
                    raise ValueError('source odometry stamp/frame disagrees with index')
                try:
                    selected, selection = select_left_cloud(cloud, source_config=config)
                except NonReferenceOriginError as exc:
                    manifest['skipped_nonreference_frames'].append({
                        'stamp_ns': original['stamp_ns'], 'left_time_offset_s': exc.metadata['left_time_offset_s'],
                        'reason': 'LEFT_RAY_ORIGIN_NOT_PAIR_REFERENCE'})
                    continue
                selected_raw = serialize_message(selected)
                (frontend / original['cloud_cdr_file']).write_bytes(selected_raw)
                (frontend / original['odom_cdr_file']).write_bytes(odom_raw)
                row = copy.deepcopy(original)
                row.update(cloud_cdr_sha256=_digest(selected_raw), cloud_data_sha256=selection['derived_data_sha256'],
                           point_rows=selection['selected_rows'], finite_points=selection['selected_finite_xyz_rows'],
                           rebuild_source={'source_cloud_cdr_sha256': original['cloud_cdr_sha256'],
                                           'source_odom_cdr_sha256': original['odom_cdr_sha256'],
                                           'original_pair_reference_stamp_ns': original['stamp_ns'],
                                           'selection': selection, 'odometry_bytes_unchanged': True})
                if isinstance(row.get('input_pair_index_file'), str):
                    row['input_pair_index_file'] = str((source / 'frontend' / row['input_pair_index_file']).resolve())
                stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False, separators=(',', ':')) + '\n')
                manifest['retained_frames'] += 1
        if manifest['retained_frames'] < 2:
            raise ValueError('fewer than two eligible left-origin frames')
        runtime = copy.deepcopy(config)
        runtime['mode'] = 'left'
        runtime['map_profile'] = profile['map_profile']
        for key in ('offline_source_selection', 'offline_lidar_selection'):
            if isinstance(runtime.get(key), dict):
                runtime[key].update(mode='left', selected_mode='left', selected_sides=['left'],
                                    excluded_sides=['right'], single_lidar_diagnostic=True,
                                    selection_basis=manifest['eligible_rule'])
        runtime['navigation_rebuild'] = {
            'variant': profile_variant, 'native_parameter_overrides': profile['native_parameter_overrides'],
            'derived_frontend_only': True, 'source_runtime_sha256': manifest['source_runtime_sha256'],
            'source_index_sha256': manifest['source_index_sha256'], 'hardware_started': False,
            'odometry_reestimated': False, 'ground_model': profile['ground_model'],
        }
        (destination / 'runtime_config.json').write_text(json.dumps(runtime, ensure_ascii=False, allow_nan=False, indent=2) + '\n', encoding='utf-8')
        derived_index = (frontend / 'index.jsonl').read_bytes()
        eligible_stamps = [str(json.loads(line)['stamp_ns']) for line in derived_index.decode('utf-8').splitlines()]
        manifest['eligible_stamp_sha256'] = _digest(('\n'.join(eligible_stamps) + '\n').encode())
        manifest['derived_index_sha256'] = _digest(derived_index)
        manifest['runtime_config_sha256'] = _digest((destination / 'runtime_config.json').read_bytes())
        if _digest(config_path.read_bytes()) != manifest['source_runtime_sha256'] or _digest(index_path.read_bytes()) != manifest['source_index_sha256']:
            raise ValueError('source metadata changed during derivation')
        changed = [path for path, expected in hashes.items() if _digest(Path(path).read_bytes()) != expected]
        if changed:
            raise ValueError('source CDR changed during derivation')
        manifest.update(status='DERIVED_FRONTEND_COMPLETE', sources_unchanged=True,
                        source_cdrs=[{'path': p, 'sha256': h} for p, h in hashes.items()])
    except Exception as exc:
        manifest.update(status='FAILED', error=type(exc).__name__ + ': ' + str(exc))
        raise
    finally:
        (destination / 'derivation.json').write_text(json.dumps(manifest, ensure_ascii=False, allow_nan=False, indent=2) + '\n', encoding='utf-8')
    return manifest
