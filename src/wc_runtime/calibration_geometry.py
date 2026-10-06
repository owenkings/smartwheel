"""Read-only geometry validation and capability gates; no devices or ROS.

T_parent_child maps child data coordinates into parent coordinates. Mechanical
CAD reference points are a separate catalogue, never an implicit sensor origin.
Availability means a complete, explicitly sourced *experimental* configuration,
not hardware health, synchronization, motion authorization or physical accuracy.
"""
import argparse
from collections import deque
import copy
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import re

import numpy as np


DIRECTION = 'child_to_parent'
TRANSFORM_CONVENTION = 'T_parent_child maps child coordinates into parent'
USABLE_STATUSES = {'USER_MEASURED_EXPERIMENT', 'CALIBRATED'}
COMPONENT_STATUSES = USABLE_STATUSES | {'UNKNOWN', 'CAD_NOMINAL', 'LEGACY_CANDIDATE'}
SOURCE_TYPES = {'CAD_AND_MANUAL_MEASUREMENTS', 'GEOMETRY_MEASUREMENT',
                'EXTRINSIC_CALIBRATION', 'VENDOR_DATA_FRAME_DEFINITION', 'LEGACY_CONFIGURATION'}
MEASURED_SOURCE_TYPES = {'GEOMETRY_MEASUREMENT', 'EXTRINSIC_CALIBRATION'}
SENSOR_FRAMES = ('lidar_left', 'lidar_right', 'camera_left_front_optical',
                 'camera_right_front_optical', 'camera_left_side_optical',
                 'camera_right_side_optical', 'imu_native', 'double_hole_left_front_unbound',
                 'double_hole_right_front_unbound', 'double_hole_left_side_unbound',
                 'double_hole_right_side_unbound')
FRAMES = {'axle', 'mounting_M', *SENSOR_FRAMES}
POINT_KINDS = {'CAD_OUTWARD_FACE_CENTER', 'CAD_BOTTOM_FACE_CENTER', 'CAD_PACKAGE_CENTER'}
DOCUMENT_SENSOR_NAMES = {
    '左雷达 XT-M60': 'lidar_left', '右雷达 XT-M60': 'lidar_right',
    '左前摄像头': 'camera_left_front_optical', '右前摄像头': 'camera_right_front_optical',
    '左侧摄像头': 'camera_left_side_optical', '右侧摄像头': 'camera_right_side_optical',
    '左前传感器（双孔）': 'double_hole_left_front_unbound',
    '右前传感器（双孔）': 'double_hole_right_front_unbound',
    '左侧传感器（双孔）': 'double_hole_left_side_unbound',
    '右侧传感器（双孔）': 'double_hole_right_side_unbound', 'IMU H30（左滑盖上）': 'imu_native'}


def inspect_installation_document(path, *, expected_sha256=None):
    """Parse this project's explicitly mechanical Markdown, never infer data TF.

    Unknown/different document formats fail rather than assigning optical axes
    or origins. The original bytes/hash and source lines remain authoritative.
    """
    path = Path(path)
    if not path.is_file() or path.stat().st_size > 16*1024*1024:
        raise ValueError('installation document is missing or exceeds 16 MiB')
    raw = path.read_bytes(); sha = hashlib.sha256(raw).hexdigest()
    if expected_sha256 is not None and sha != expected_sha256:
        raise ValueError('installation document SHA-256 changed')
    text = raw.decode('utf-8-sig')
    if '本文所有坐标都在 M 系中，单位 mm。' not in text or '不是标定结果' not in text:
        raise ValueError('document must explicitly identify mechanical M/mm nominal values, not calibration')
    points, checks, cad, auxiliary, box_bounds = [], [], {}, [], {}
    triple = re.compile(r'\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)')
    for number, line in enumerate(text.splitlines(), 1):
        if '封装中心在' in line:
            center = triple.search(line.split('封装中心在', 1)[1])
            if center is None:
                raise ValueError('document package centre format changed')
            auxiliary.append({'sensor_frame': 'imu_native', 'point_kind': 'CAD_PACKAGE_CENTER',
                              'position_mm': [float(x) for x in center.groups()], 'line': number})
        if not line.strip().startswith('|'):
            continue
        cells = [cell.strip() for cell in line.strip().strip('|').split('|')]
        if cells[0] in DOCUMENT_SENSOR_NAMES:
            if len(cells) != 7:
                raise ValueError('mechanical point table format changed')
            position = [float(x) for x in cells[2:5]]
            dimensions = [float(x) for x in cells[5].split('×')]
            center = triple.search(cells[6])
            if len(dimensions) != 3 or center is None:
                raise ValueError('mechanical envelope format changed')
            finite_array(position, (3,), 'document position_mm')
            size = finite_array(dimensions, (3,), 'document envelope_dimensions_mm')
            if np.any(size <= 0):
                raise ValueError('document envelope dimensions must be positive')
            center_values = [float(x) for x in center.groups()]
            finite_array(center_values, (3,), 'document envelope_center_mm')
            view = {'+x': [1, 0, 0], '+y': [0, 1, 0], '-y': [0, -1, 0], '顶面': [0, 0, 1]}
            direction = next((vector for prefix, vector in view.items() if cells[1].startswith(prefix)), None)
            if direction is None:
                raise ValueError('document nominal outward direction is unsupported')
            points.append({'sensor_frame': DOCUMENT_SENSOR_NAMES[cells[0]], 'position_mm': position,
                'envelope_dimensions_mm': dimensions, 'envelope_center_mm': center_values,
                'nominal_view_direction_M': direction,
                'line': number, 'point_kind': 'CAD_BOTTOM_FACE_CENTER' if cells[0].startswith('IMU') else 'CAD_OUTWARD_FACE_CENTER',
                'status': 'CAD_NOMINAL', 'data_origin_validated': False})
        elif cells[0] == '盒体外墙':
            bounds = re.search(r'左盒 y = (-?\d+(?:\.\d+)?) ~ (-?\d+(?:\.\d+)?)，右盒 y = (-?\d+(?:\.\d+)?) ~ (-?\d+(?:\.\d+)?)', cells[1])
            if bounds is None:
                raise ValueError('document box wall bounds format changed')
            values = [float(x) for x in bounds.groups()]
            box_bounds = {'left': values[:2], 'right': values[2:]}
        elif len(cells) == 5 and '→' in cells[0]:
            try:
                values = [float(x) for x in cells[1:]]
            except ValueError:
                continue  # The A→B header contains labels, not measurements.
            finite_array(values, (4,), 'document distance row')
            calculated = float(np.linalg.norm(values[:3]))
            checks.append({'label': cells[0], 'line': number, 'delta_mm': values[:3],
                'declared_distance_mm': values[3], 'calculated_distance_mm': calculated,
                'rounding_consistent': abs(calculated-values[3]) <= .00500001})
        elif cells[0] in ('右盒 (X, Y, Z)', '左盒 (X, Y, Z)'):
            a = re.fullmatch(r'(-?\d+(?:\.\d+)?)\s*-\s*Y', cells[1])
            b = re.fullmatch(r'X\s*([+-])\s*(\d+(?:\.\d+)?)', cells[2])
            c = re.fullmatch(r'Z\s*([+-])\s*(\d+(?:\.\d+)?)', cells[3])
            if not (a and b and c):
                raise ValueError('unsupported CAD-to-M expressions; do not evaluate arbitrary expressions')
            signed = lambda match: float(match[2])*(1 if match[1]=='+' else -1)
            translation = [float(a[1]), signed(b), signed(c)]
            matrix = [[0,-1,0,translation[0]], [1,0,0,translation[1]], [0,0,1,translation[2]], [0,0,0,1]]
            cad['right' if cells[0].startswith('右') else 'left'] = {'T_M_CAD_mm': matrix, 'line': number}
    if len(points) != 11 or {p['sensor_frame'] for p in points} != set(SENSOR_FRAMES) or set(cad) != {'left','right'}:
        raise ValueError('document must contain all 11 mechanical reference points and both CAD transforms')
    if not checks or not all(row['rounding_consistent'] for row in checks):
        raise ValueError('document distance arithmetic is missing or inconsistent')
    return {'schema_version': 1, 'source_path': str(path), 'sha256': sha, 'frame_id': 'mounting_M',
            'unit': 'mm', 'status': 'CAD_NOMINAL', 'reference_points': points, 'cad_to_M': cad,
            'auxiliary_points': auxiliary, 'box_lateral_bounds_mm': box_bounds,
            'distance_checks': checks, 'data_extrinsics_created': False,
            'hardware_started': False, 'configuration_written': False}


def _object(value, required, label, optional=()):
    if not isinstance(value, dict):
        raise ValueError(label + ' must be an object')
    missing = set(required) - set(value)
    unknown = set(value) - set(required) - set(optional)
    if missing or unknown:
        raise ValueError(f'{label} has missing/unknown fields: missing={sorted(missing)}, unknown={sorted(unknown)}')
    return value


def _text(value, label):
    if not isinstance(value, str) or not value.strip() or len(value) > 4096:
        raise ValueError(label + ' requires non-empty bounded text')
    return value


def finite_array(value, shape, label):
    """Reject strings/bools as well as nonfinite, ragged and wrong-size arrays."""
    def numeric(item):
        if isinstance(item, (list, tuple)):
            return all(numeric(child) for child in item)
        return type(item) in (int, float) and math.isfinite(item)
    if not isinstance(value, (list, tuple)) or not numeric(value):
        raise ValueError(label + ' requires finite numeric values, not strings or booleans')
    try:
        result = np.asarray(value, dtype=float)
    except (ValueError, TypeError) as error:
        raise ValueError(label + ' has invalid shape') from error
    if result.shape != shape:
        raise ValueError(label + f' requires shape {shape}')
    return result


def proper_rotation(value, label='R_parent_child'):
    result = finite_array(value, (3, 3), label)
    if not np.allclose(result.T @ result, np.eye(3), atol=1e-6, rtol=0) or \
            abs(float(np.linalg.det(result))-1.) > 1e-6:
        raise ValueError(label + ' must be a proper rotation; reflection, scaling and skew are forbidden')
    return result


def rigid_transform(value, label='T_parent_child', *, translation_limit_m=3.):
    result = finite_array(value, (4, 4), label)
    if not np.allclose(result[3], [0, 0, 0, 1], atol=1e-8, rtol=0):
        raise ValueError(label + ' requires homogeneous bottom row [0,0,0,1]')
    proper_rotation(result[:3, :3].tolist(), label + '.rotation')
    if np.linalg.norm(result[:3, 3]) > translation_limit_m:
        raise ValueError(label + ' translation is outside the declared unit/plausibility bound')
    return result


def _safe_snapshot_path(value):
    _text(value, 'source.snapshot_path')
    path = PurePosixPath(value)
    if path.is_absolute() or '..' in path.parts or '\\' in value or ':' in value:
        raise ValueError('source.snapshot_path must stay inside the project using a relative POSIX path')
    return path


def validate_sources(values):
    if not isinstance(values, list) or not values or len(values) > 128:
        raise ValueError('source_documents requires 1 to 128 source records')
    result = {}
    for value in values:
        row = _object(value, {'id', 'snapshot_path', 'sha256', 'evidence_type', 'assembly_revision', 'note'},
                      'source_document', {'original_path', 'document_date'})
        key = _text(row['id'], 'source.id')
        if key in result:
            raise ValueError('duplicate source_document id: ' + key)
        _safe_snapshot_path(row['snapshot_path'])
        sha = row['sha256']
        if not isinstance(sha, str) or len(sha) != 64 or any(x not in '0123456789abcdef' for x in sha):
            raise ValueError('source.sha256 requires 64 lowercase hexadecimal digits')
        if row['evidence_type'] not in SOURCE_TYPES:
            raise ValueError('unsupported geometry source evidence_type')
        for field in ('assembly_revision', 'note'):
            _text(row[field], 'source.' + field)
        for field in ('original_path', 'document_date'):
            if field in row:
                _text(row[field], 'source.' + field)
        result[key] = copy.deepcopy(row)
    return result


def _same_geometry_value(actual, expected, label):
    if isinstance(expected, (list, tuple)):
        expected_array = np.asarray(expected, dtype=float)
        actual_array = finite_array(actual, expected_array.shape, label)
        if not np.allclose(actual_array, expected_array, atol=1e-8, rtol=0):
            raise ValueError(label + ' differs from the source-derived geometry')
    elif type(expected) in (int, float):
        if type(actual) not in (int, float) or not math.isfinite(actual) or not math.isclose(actual, expected, abs_tol=1e-8, rel_tol=0):
            raise ValueError(label + ' differs from the source-derived geometry')
    elif actual != expected:
        raise ValueError(label + ' differs from the source-derived geometry')


def _has_geometry_evidence(row, source_id):
    if not any(item.get('source_id') == source_id for item in row.get('evidence', [])):
        raise ValueError('current geometry requires explicit override evidence: ' + source_id)


def _check_layout_override(setup, source, confirmation, document, sources, checked):
    """Check a measured lateral update against the unchanged original CAD.

    Hashing a new JSON alone cannot authorize arbitrary modified CAD. Both its
    declared catalogue and the runtime catalogue must equal the original CAD
    plus the measured side translation, with unchanged within-box geometry.
    """
    _object(confirmation, {'schema', 'schema_version', 'date', 'operator', 'assembly_revision',
        'source', 'raw_document', 'latest_confirmations', 'layout_override',
        'current_mechanical_geometry', 'current_transforms', 'wheel_geometry_m',
        'historical_measurements', 'note'}, 'current geometry confirmation')
    if confirmation['schema'] != 'wc_geometry_confirmation_v1' or type(confirmation['schema_version']) is not int or confirmation['schema_version'] != 1:
        raise ValueError('unsupported current geometry confirmation schema')
    if source['evidence_type'] != 'GEOMETRY_MEASUREMENT' or \
            confirmation['assembly_revision'] != source['assembly_revision'] or \
            source['assembly_revision'] != setup['assembly']['revision']:
        raise ValueError('layout override requires an assembly-matching GEOMETRY_MEASUREMENT source')
    for field in ('date', 'operator', 'source', 'note'):
        _text(confirmation[field], 'confirmation.' + field)
    raw = _object(confirmation['raw_document'], {'snapshot_path', 'sha256'}, 'raw_document')
    _safe_snapshot_path(raw['snapshot_path'])
    raw_sources = [row for row in sources.values() if row['snapshot_path'] == raw['snapshot_path'] and
                   row['sha256'] == raw['sha256'] and row['evidence_type'] == 'GEOMETRY_MEASUREMENT' and
                   row['assembly_revision'] == source['assembly_revision']]
    if len(raw_sources) != 1 or checked[raw_sources[0]['id']]['status'] != 'PASS':
        raise ValueError('current confirmation requires its declared, hash-verified raw measurement snapshot')
    override = _object(confirmation['layout_override'], {'base_source_id', 'base_sha256', 'type',
        'old_corner_spacing_mm', 'new_corner_spacing_mm', 'delta_M_mm', 'same_box_geometry_preserved',
        'superseded_current_measurement_ids', 'current_inner_wall_gap_mm',
        'current_lidar_face_spacing_mm'}, 'layout_override')
    base = sources.get(override['base_source_id'])
    if base is None or base['evidence_type'] != 'CAD_AND_MANUAL_MEASUREMENTS' or \
            override['base_sha256'] != base['sha256'] or document['sha256'] != base['sha256']:
        raise ValueError('layout override base source identity/hash must match the original CAD snapshot')
    if checked[base['id']]['status'] != 'PASS':
        raise ValueError('layout override original CAD snapshot has not passed its hash check')
    if override['type'] != 'SYMMETRIC_LATERAL_TRANSLATION' or override['same_box_geometry_preserved'] is not True:
        raise ValueError('layout override must preserve same-box geometry with a symmetric lateral translation')
    bounds = document['box_lateral_bounds_mm']
    if set(bounds) != {'left', 'right'} or len(document['auxiliary_points']) != 1:
        raise ValueError('original CAD wall bounds/package centre required for a layout override')
    old_span = bounds['left'][1] - bounds['right'][0]
    widths = [bounds[side][1]-bounds[side][0] for side in ('left', 'right')]
    values = finite_array([override['old_corner_spacing_mm'], override['new_corner_spacing_mm']], (2,), 'corner spacings')
    if np.any(values <= 0):
        raise ValueError('corner spacings must be positive')
    _same_geometry_value(values[0].item(), old_span, 'old corner spacing')
    delta = float((values[1]-values[0])/2)
    shifts = _object(override['delta_M_mm'], {'left', 'right'}, 'layout_override.delta_M_mm')
    for side, sign in (('left', 1), ('right', -1)):
        _same_geometry_value(shifts[side], [0, sign*delta, 0], 'layout override '+side+' shift')
    latest = confirmation['latest_confirmations']
    if not isinstance(latest, dict):
        raise ValueError('latest_confirmations must be an object')
    _same_geometry_value(latest.get('outer_front_top_corner_spacing_mm'), values[1].item(), 'confirmed corner spacing')
    closure_inputs = finite_array([latest.get('outer_tire_span_mm'), latest.get('C4_inward_offset_mm'),
                                  latest.get('closure_mm')], (3,), 'outer tire/corner closure')
    if latest.get('C4_H_reference') != 'CENTER_OF_OUTER_TIRE_SIDE_FACE' or \
            not math.isclose(closure_inputs[0]-2*closure_inputs[1], values[1], abs_tol=1e-8) or \
            not math.isclose(closure_inputs[2], values[1], abs_tol=1e-8):
        raise ValueError('current outer tire/corner closure is inconsistent')
    geometry = _object(confirmation['current_mechanical_geometry'],
        {'cad_to_M', 'reference_points', 'auxiliary_points'}, 'current_mechanical_geometry')
    catalog = setup['mechanical_reference']
    expected_points = []
    for point in document['reference_points']:
        side = 'left' if point['sensor_frame'] == 'imu_native' or '_left_' in point['sensor_frame'] or point['sensor_frame'].endswith('_left') else 'right'
        current = copy.deepcopy(point)
        for field in ('position_mm', 'envelope_center_mm'):
            current[field] = (np.asarray(point[field])+np.asarray(shifts[side])).tolist()
        expected_points.append(current)
    fields = ('position_mm', 'envelope_dimensions_mm', 'envelope_center_mm', 'point_kind', 'nominal_view_direction_M')
    for label, points in (('override', geometry['reference_points']), ('mechanical catalogue', catalog['reference_points'])):
        if not isinstance(points, list) or len(points) != 11 or {p.get('sensor_frame') for p in points} != set(SENSOR_FRAMES):
            raise ValueError(label + ' must contain each of the original 11 points exactly once')
        indexed = {point['sensor_frame']: point for point in points}
        for expected in expected_points:
            current = indexed[expected['sensor_frame']]
            for field in fields:
                _same_geometry_value(current.get(field), expected[field], label+' '+expected['sensor_frame']+'.'+field)
            if label == 'mechanical catalogue':
                _has_geometry_evidence(current, source['id'])
    for side in ('left', 'right'):
        expected = np.asarray(document['cad_to_M'][side]['T_M_CAD_mm'], dtype=float)
        expected[:3, 3] += shifts[side]
        _same_geometry_value(geometry['cad_to_M'].get(side), expected.tolist(), 'override CAD-to-M '+side)
        _same_geometry_value(catalog['cad_to_M'][side]['T_M_CAD_mm'], expected.tolist(), 'mechanical CAD-to-M '+side)
        _has_geometry_evidence(catalog['cad_to_M'][side], source['id'])
    expected_aux = copy.deepcopy(document['auxiliary_points'][0])
    expected_aux['position_mm'] = (np.asarray(expected_aux['position_mm'])+np.asarray(shifts['left'])).tolist()
    for label, points in (('override', geometry['auxiliary_points']), ('mechanical catalogue', catalog['auxiliary_points'])):
        if not isinstance(points, list) or len(points) != 1:
            raise ValueError(label+' must preserve the original auxiliary package reference')
        for field in ('sensor_frame', 'point_kind', 'position_mm'):
            _same_geometry_value(points[0].get(field), expected_aux[field], label+' auxiliary.'+field)
        if label == 'mechanical catalogue':
            _has_geometry_evidence(points[0], source['id'])
    lidar_points = {point['sensor_frame']: point for point in expected_points}
    lidar_span = lidar_points['lidar_left']['position_mm'][1]-lidar_points['lidar_right']['position_mm'][1]
    inner_gap = values[1]-sum(widths)
    for field, expected in (('current_inner_wall_gap_mm', inner_gap), ('current_lidar_face_spacing_mm', lidar_span)):
        _same_geometry_value(override[field], float(expected), 'override '+field)
    measurements = {row['id']: row for row in catalog['measurements']}
    if len(measurements) != len(catalog['measurements']):
        raise ValueError('current mechanical measurement IDs must be unique')
    for key, expected in (('outer_front_top_corner_spacing', values[1]), ('inner_box_wall_gap', inner_gap),
                          ('nominal_lidar_face_spacing', lidar_span)):
        if key not in measurements:
            raise ValueError('current mechanical measurement missing: '+key)
        _same_geometry_value(measurements[key]['value_mm'], float(expected), 'measurement '+key)
        _has_geometry_evidence(measurements[key], source['id'])
    # Old independent observations stay in the raw snapshot/history; they are
    # not current checks of a layout translated by the new measurement.
    for key in ('clamp_seat_faces', 'clamp_plate_faces', 'lidar_window_spacing'):
        if key in measurements:
            raise ValueError('superseded old measurement remains active: '+key)
    transforms = _object(confirmation['current_transforms'], {'T_axle_M', 'R_M_imu', 't_M_imu'}, 'current_transforms')
    axle_M = rigid_transform(transforms['T_axle_M'], 'confirmation.T_axle_M')
    imu_R = proper_rotation(transforms['R_M_imu'], 'confirmation.R_M_imu')
    for parent, child, component, expected in (
            ('axle', 'mounting_M', 'translation', axle_M[:3, 3].tolist()),
            ('axle', 'mounting_M', 'rotation', axle_M[:3, :3].tolist()),
            ('mounting_M', 'imu_native', 'rotation', imu_R.tolist())):
        edge = next(row for row in setup['data_transforms'] if row['parent_frame'] == parent and row['child_frame'] == child)
        row = edge[component]
        if any(item['source_id'] == source['id'] for item in row['evidence']):
            key = 'value_m' if component == 'translation' else 'value_matrix'
            _same_geometry_value(row[key], expected, parent+' <- '+child+'.'+component)
            if row['status'] != 'USER_MEASURED_EXPERIMENT':
                raise ValueError('user confirmation data transforms must remain USER_MEASURED_EXPERIMENT')
    return {'override_source_id': source['id'], 'base_source_id': base['id'],
            'left_shift_mm': shifts['left'], 'right_shift_mm': shifts['right'],
            'reference_points_checked': len(expected_points), 'auxiliary_points_checked': 1,
            'same_box_geometry_preserved': True}


def verify_geometry_sources(setup, project_root, *, source_path_map=None):
    """Hash declared project snapshots, without writing or opening hardware.

    Callers should freeze these checked snapshots with each session. A missing,
    changed or escaped snapshot is a FAIL, never replaced by its original path.
    """
    sources = validate_sources(setup['source_documents'])
    root = Path(project_root).resolve()
    path_map = {} if source_path_map is None else copy.deepcopy(source_path_map)
    if not isinstance(path_map, dict):
        raise ValueError('source_path_map must explicitly map declared relative snapshot paths')
    declared_paths = {row['snapshot_path'] for row in sources.values()}
    for original, archived in path_map.items():
        _safe_snapshot_path(original)
        _safe_snapshot_path(archived)
        if original not in declared_paths:
            raise ValueError('source_path_map contains an undeclared source path: '+original)
    # Identity, evidence references and embedded paths retain their original
    # source bytes. Only the explicit filesystem lookup location is relocated.
    rows, snapshots, confirmations = [], {}, []
    for row in sources.values():
        located_path = path_map.get(row['snapshot_path'], row['snapshot_path'])
        item = {'id': row['id'], 'snapshot_path': located_path, 'expected_sha256': row['sha256']}
        if located_path != row['snapshot_path']:
            item['declared_snapshot_path'] = row['snapshot_path']
        try:
            from .storage_policy import resolve_storage_path
            path = resolve_storage_path(root, located_path)
            if not path.is_file() or path.stat().st_size > 16*1024*1024:
                raise ValueError('snapshot is missing or exceeds 16 MiB')
            item['observed_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
            item['status'] = 'PASS' if item['observed_sha256'] == row['sha256'] else 'FAIL'
            item['reason'] = None if item['status'] == 'PASS' else 'source snapshot hash mismatch'
            if item['status'] == 'PASS':
                snapshots[row['id']] = path
                if row['evidence_type'] == 'GEOMETRY_MEASUREMENT' and path.suffix.lower() == '.json':
                    payload = json.loads(path.read_text(encoding='utf-8-sig'))
                    if isinstance(payload, dict) and payload.get('schema') == 'wc_geometry_confirmation_v1':
                        confirmations.append((row, payload))
        except (OSError, ValueError) as error:
            item.update(status='FAIL', reason=str(error), observed_sha256=None)
        rows.append(item)
    checked = {row['id']: row for row in rows}
    for source, payload in confirmations:
        override = payload.get('layout_override')
        base = sources.get(override.get('base_source_id')) if isinstance(override, dict) else None
        if base is None or base['evidence_type'] != 'CAD_AND_MANUAL_MEASUREMENTS':
            checked[source['id']].update(status='FAIL', reason='layout override requires a declared original CAD source')
    for source in sources.values():
        item = checked[source['id']]
        if item['status'] != 'PASS' or source['evidence_type'] != 'CAD_AND_MANUAL_MEASUREMENTS':
            continue
        try:
            path = snapshots[source['id']]
            document = inspect_installation_document(path, expected_sha256=source['sha256'])
            overrides = [(row, payload) for row, payload in confirmations
                         if isinstance(payload.get('layout_override'), dict) and
                         payload['layout_override'].get('base_source_id') == source['id']]
            if len(overrides) > 1:
                raise ValueError('multiple current layout overrides for the same original CAD source')
            if overrides:
                row, payload = overrides[0]
                item['layout_override'] = _check_layout_override(setup, row, payload, document, sources, checked)
            else:
                declared = {point['sensor_frame']: point for point in setup['mechanical_reference']['reference_points']}
                for point in document['reference_points']:
                    current = declared.get(point['sensor_frame'])
                    if current is None or any(current.get(field) != point[field] for field in
                            ('position_mm', 'envelope_dimensions_mm', 'envelope_center_mm', 'point_kind')):
                        raise ValueError('mechanical catalogue differs from source document: '+point['sensor_frame'])
                for side in ('left', 'right'):
                    if setup['mechanical_reference']['cad_to_M'][side]['T_M_CAD_mm'] != document['cad_to_M'][side]['T_M_CAD_mm']:
                        raise ValueError('mechanical CAD-to-M transform differs from source document')
            item['mechanical_reference_points_checked'] = len(document['reference_points'])
            item['distance_rows_checked'] = len(document['distance_checks'])
        except (OSError, ValueError, KeyError, TypeError, AttributeError, StopIteration) as error:
            item.update(status='FAIL', reason=str(error))
    return {'status': 'PASS' if all(row['status'] == 'PASS' for row in rows) else 'FAIL',
            'sources': rows, 'source_path_map': path_map, 'hardware_started': False}


def _evidence(values, sources, label):
    if not isinstance(values, list) or not values or len(values) > 32:
        raise ValueError(label + ' requires explicit source evidence')
    ids = []
    for item in values:
        row = _object(item, {'source_id'}, label, {'line_start', 'line_end'})
        if row['source_id'] not in sources:
            raise ValueError(label + ' refers to an unknown source_id')
        if ('line_start' in row) != ('line_end' in row):
            raise ValueError(label + ' line range needs both ends')
        if 'line_start' in row and (type(row['line_start']) is not int or type(row['line_end']) is not int or
                                    not 1 <= row['line_start'] <= row['line_end']):
            raise ValueError(label + ' has invalid source lines')
        ids.append(row['source_id'])
    return ids


def validate_mechanical_catalog(value, sources):
    catalog = _object(value, {'frame_id', 'unit', 'status', 'origin_definition', 'assumptions',
                             'cad_to_M', 'reference_points', 'auxiliary_points', 'measurements', 'limitations'},
                      'mechanical_reference')
    if catalog['frame_id'] != 'mounting_M' or catalog['unit'] != 'mm' or catalog['status'] != 'CAD_NOMINAL':
        raise ValueError('mechanical_reference must declare mounting_M, mm and CAD_NOMINAL')
    origin = _object(catalog['origin_definition'], {'x', 'y', 'z'}, 'mechanical_reference.origin_definition')
    for axis in ('x', 'y', 'z'):
        _text(origin[axis], 'mechanical_reference.origin_definition.' + axis)
    for field in ('assumptions', 'limitations'):
        if not isinstance(catalog[field], list) or not catalog[field] or not all(isinstance(x, str) and x.strip() for x in catalog[field]):
            raise ValueError('mechanical_reference.' + field + ' requires explicit text records')
    cad = _object(catalog['cad_to_M'], {'left', 'right'}, 'mechanical_reference.cad_to_M')
    for side, entry in cad.items():
        row = _object(entry, {'T_M_CAD_mm', 'direction', 'evidence'}, 'cad_to_M.' + side)
        if row['direction'] != 'CAD_to_M':
            raise ValueError('CAD transform direction must be CAD_to_M')
        rigid_transform(row['T_M_CAD_mm'], 'T_M_CAD_mm', translation_limit_m=3000.)
        _evidence(row['evidence'], sources, 'cad_to_M.evidence')
    if not isinstance(catalog['reference_points'], list) or len(catalog['reference_points']) != len(SENSOR_FRAMES):
        raise ValueError('mechanical_reference requires all 11 sensor reference points')
    if not isinstance(catalog['auxiliary_points'], list):
        raise ValueError('mechanical_reference.auxiliary_points requires an explicit list')
    seen = set()
    for row in catalog['reference_points'] + catalog['auxiliary_points']:
        _object(row, {'id', 'sensor_frame', 'point_kind', 'position_mm', 'status', 'evidence', 'note'},
                'mechanical_reference_point', {'nominal_view_direction_M', 'envelope_dimensions_mm', 'envelope_center_mm'})
        key = _text(row['id'], 'reference_point.id')
        if key in seen:
            raise ValueError('duplicate mechanical reference point id')
        seen.add(key)
        if row['sensor_frame'] not in SENSOR_FRAMES or row['point_kind'] not in POINT_KINDS or row['status'] != 'CAD_NOMINAL':
            raise ValueError('mechanical points must remain explicitly CAD_NOMINAL, with a defined CAD point_kind')
        finite_array(row['position_mm'], (3,), key + '.position_mm')
        for field in ('envelope_dimensions_mm', 'envelope_center_mm', 'nominal_view_direction_M'):
            if field in row:
                vector = finite_array(row[field], (3,), key + '.' + field)
                if field == 'envelope_dimensions_mm' and np.any(vector <= 0):
                    raise ValueError('CAD envelope dimensions must be positive')
                if field == 'nominal_view_direction_M' and not math.isclose(float(np.linalg.norm(vector)), 1., abs_tol=1e-6):
                    raise ValueError('nominal CAD view direction must be a unit vector')
        _text(row['note'], key + '.note')
        _evidence(row['evidence'], sources, key + '.evidence')
    if {row['sensor_frame'] for row in catalog['reference_points']} != set(SENSOR_FRAMES):
        raise ValueError('mechanical_reference must cover each of the 11 sensor frames exactly once')
    if not isinstance(catalog['measurements'], list):
        raise ValueError('mechanical measurements must be an explicit list')
    for row in catalog['measurements']:
        _object(row, {'id', 'value_mm', 'status', 'evidence', 'note'}, 'mechanical_measurement')
        _text(row['id'], 'measurement.id')
        if type(row['value_mm']) not in (int, float) or not math.isfinite(row['value_mm']) or row['value_mm'] <= 0:
            raise ValueError('mechanical measurement requires a positive finite millimetre value')
        if row['status'] not in ('USER_REPORTED_MEASUREMENT', 'DERIVED_NOMINAL'):
            raise ValueError('mechanical measurement must retain its evidence class')
        _text(row['note'], 'measurement.note')
        _evidence(row['evidence'], sources, 'measurement.evidence')
    return copy.deepcopy(catalog)


def _component(value, name, sources, assembly_revision):
    field = 'value_m' if name == 'translation' else 'value_matrix'
    row = _object(value, {field, 'status', 'reference_kind', 'evidence', 'note'}, name)
    if row['status'] not in COMPONENT_STATUSES:
        raise ValueError(name + '.status is unsupported')
    expected = 'FRAME_ORIGIN' if name == 'translation' else 'DATA_AXES'
    if row['reference_kind'] != expected:
        raise ValueError(name + f' must reference {expected}; CAD face/package centres are not data extrinsics')
    _text(row['note'], name + '.note')
    ids = _evidence(row['evidence'], sources, name + '.evidence')
    if row['status'] == 'UNKNOWN':
        if row[field] is not None:
            raise ValueError(name + ' UNKNOWN must remain null, not a filled zero/identity')
        return None, name + ':UNKNOWN'
    if name == 'translation':
        result = finite_array(row[field], (3,), 'translation.value_m')
        if np.linalg.norm(result) > 3:
            raise ValueError('translation.value_m must be metres within 3 m')
    else:
        result = proper_rotation(row[field], 'rotation.value_matrix')
    if row['status'] not in USABLE_STATUSES:
        return None, name + ':' + row['status']
    applicable = [sources[key] for key in ids if sources[key]['assembly_revision'] == assembly_revision]
    kinds = {source['evidence_type'] for source in applicable}
    if not kinds.intersection(MEASURED_SOURCE_TYPES):
        raise ValueError(name + ' measured data extrinsic requires an assembly-matching geometry measurement/calibration source')
    if row['status'] == 'CALIBRATED' and 'EXTRINSIC_CALIBRATION' not in kinds:
        raise ValueError(name + ' CALIBRATED requires an EXTRINSIC_CALIBRATION source')
    return result, None


def _connected_values(graph, origin, dimension):
    result = {origin: np.eye(dimension)}
    pending = deque([origin])
    while pending:
        current = pending.popleft()
        for neighbor, current_from_neighbor in graph.get(current, []):
            origin_from_neighbor = result[current] @ current_from_neighbor
            if neighbor in result:
                if not np.allclose(result[neighbor], origin_from_neighbor, atol=1e-6, rtol=0):
                    raise ValueError('inconsistent transform paths/cycle for ' + origin + ' <- ' + neighbor)
            else:
                result[neighbor] = origin_from_neighbor
                pending.append(neighbor)
    return result


def resolve_data_graph(values, sources, assembly_revision):
    if not isinstance(values, list) or not values or len(values) > 128:
        raise ValueError('data_transforms requires 1 to 128 explicit transform records')
    full, rotations, unresolved, seen, frames_seen = {}, {}, [], set(), set()
    for row in values:
        _object(row, {'id', 'parent_frame', 'child_frame', 'direction', 'unit',
                      'translation', 'rotation', 'note'}, 'data_transform')
        key = _text(row['id'], 'data_transform.id')
        if key in seen:
            raise ValueError('duplicate data transform id')
        seen.add(key)
        parent, child = row['parent_frame'], row['child_frame']
        if parent not in FRAMES or child not in FRAMES or parent == child:
            raise ValueError('data transform requires two distinct declared frames')
        if row['direction'] != DIRECTION or row['unit'] != 'm':
            raise ValueError('data transform must declare child_to_parent and metres')
        _text(row['note'], 'data_transform.note')
        frames_seen.update((parent, child))
        translation, t_reason = _component(row['translation'], 'translation', sources, assembly_revision)
        rotation, r_reason = _component(row['rotation'], 'rotation', sources, assembly_revision)
        for component, reason in (('translation', t_reason), ('rotation', r_reason)):
            if reason:
                unresolved.append({'transform_id': key, 'parent_frame': parent, 'child_frame': child,
                                   'component': component, 'reason': reason, 'note': row[component]['note']})
        if rotation is not None:
            rotations.setdefault(parent, []).append((child, rotation))
            rotations.setdefault(child, []).append((parent, rotation.T))
            if translation is not None:
                matrix = np.eye(4); matrix[:3, :3] = rotation; matrix[:3, 3] = translation
                full.setdefault(parent, []).append((child, matrix))
                full.setdefault(child, []).append((parent, np.linalg.inv(matrix)))
    if frames_seen != FRAMES:
        raise ValueError('data_transforms must explicitly declare unknown or known relations for every sensor, M and axle')
    # Check each connected component, including components disconnected from axle.
    checked = set()
    for origin in FRAMES:
        if origin not in checked:
            checked.update(_connected_values(full, origin, 4))
    checked = set()
    for origin in FRAMES:
        if origin not in checked:
            checked.update(_connected_values(rotations, origin, 3))
    return {frame: _connected_values(full, frame, 4) for frame in FRAMES}, \
           {frame: _connected_values(rotations, frame, 3) for frame in FRAMES}, unresolved


def capability_assessment(full, rotations, *, assembly_confirmed, legacy=False):
    """Evaluate geometry requirements only; hardware/time/authorization remain separate."""
    result = {}
    def add(name, requirements=(), motion=False, reason=None):
        missing = [description for description, available in requirements if not available]
        reasons = [f'缺少已测量/标定的数据坐标关系：{name}' for name in missing]
        if motion and not assembly_confirmed:
            reasons.append('当前装配版本和状态尚未确认；历史安装不自动继承')
        if reason:
            reasons.append(reason)
        result[name] = {'status': 'BLOCKED' if reasons else 'AVAILABLE', 'reasons': reasons,
                        'requires': [description for description, _ in requirements], 'validation_level': 'CONFIG_ONLY'}
    for name in ('native_preview', 'native_capture', 'mechanical_layout', 'camera_intrinsic_calibration',
                 'imu_native_bias_calibration', 'lidar_relative_calibration', 'legacy_replay'):
        add(name)
    relative = 'lidar_right' in full['lidar_left']
    legacy_reason = 'schema1 仅保留历史候选与回放；不能自动用于当前 V7 装配' if legacy else None
    add('dual_lidar_static_fusion', [('T_lidar_left_lidar_right', relative)], reason=legacy_reason)
    imu = 'imu_native' in rotations['axle']
    for mode in ('left', 'right', 'all'):
        sides = ('left', 'right') if mode == 'all' else (mode,)
        mounts = [('T_axle_lidar_' + side, 'lidar_' + side in full['axle']) for side in sides]
        add('wheel_imu_motion_' + mode, mounts + [('R_axle_imu_native', imu)], motion=True, reason=legacy_reason)
        add('ground_height_' + mode, mounts, motion=True, reason=legacy_reason)
    for role in ('left_front', 'right_front', 'left_side', 'right_side'):
        frame = 'camera_' + role + '_optical'
        add('lidar_camera_geometry_' + role, [('T_lidar_left_' + frame, frame in full['lidar_left'])], reason=legacy_reason)
    # Geometry is not enough for optical projection or ultrasound ray assignment.
    add('lidar_camera_projection', reason='还需各相机内参/畸变、图像处理轴定义与时间对应；本文件不认证这些条件')
    add('ultrasonic_spatial_projection', reason='双孔模块型号、地址/物理角色、测距原点、方向和波束定义尚未绑定')
    return result


def motion_capability(mode):
    if mode not in ('left', 'right', 'all'):
        raise ValueError('geometry motion gate requires explicit left/right/all mode')
    return 'wheel_imu_motion_' + mode


def require_geometry_capability(resolved_or_report, name):
    """Return the AVAILABLE row or raise before any dependent device launch."""
    if not isinstance(resolved_or_report, dict):
        raise ValueError('geometry capability assessment must be an object')
    rows = resolved_or_report.get('geometry_capabilities', resolved_or_report.get('capabilities'))
    if not isinstance(rows, dict) or name not in rows:
        raise ValueError('缺少能力判定：' + str(name))
    row = rows[name]
    if row.get('status') != 'AVAILABLE':
        raise ValueError('外参能力阻塞 ' + name + '：' + '；'.join(row.get('reasons') or ['状态不是 AVAILABLE']))
    return copy.deepcopy(row)


def fit_rigid_transform(parent_points_m, child_points_m):
    """Fit paired measured points offline; return a candidate, never install it.

    Correspondences must refer to the same physical points in both frames.
    Non-collinear geometry is required. Low training residual alone does not
    validate origin/axis semantics, independent accuracy or time alignment.
    """
    if not isinstance(parent_points_m, (list, tuple)) or len(parent_points_m) < 3:
        raise ValueError('rigid fit requires at least three paired physical points')
    parent = finite_array(parent_points_m, (len(parent_points_m), 3), 'parent_points_m')
    child = finite_array(child_points_m, parent.shape, 'child_points_m')
    a, b = parent-parent.mean(axis=0), child-child.mean(axis=0)
    if np.linalg.matrix_rank(a, tol=1e-9) < 2 or np.linalg.matrix_rank(b, tol=1e-9) < 2:
        raise ValueError('rigid fit is unobservable from collinear/coincident correspondences')
    u, singular, vt = np.linalg.svd(b.T @ a)
    correction = np.eye(3); correction[2, 2] = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ correction @ u.T
    translation = parent.mean(axis=0)-rotation @ child.mean(axis=0)
    matrix = np.eye(4); matrix[:3, :3] = rotation; matrix[:3, 3] = translation
    rigid_transform(matrix.tolist())
    residuals = np.linalg.norm(child @ rotation.T + translation-parent, axis=1)
    return {'T_parent_child': matrix.tolist(), 'direction': DIRECTION, 'unit': 'm',
            'status': 'FIT_ONLY_NOT_VALIDATED', 'rmse_m': float(np.sqrt(np.mean(residuals**2))),
            'max_residual_m': float(residuals.max()), 'paired_points': len(parent),
            'singular_values': singular.tolist(), 'independent_validation': False,
            'hardware_started': False, 'configuration_written': False}


def main(argv=None):
    parser = argparse.ArgumentParser(description='只读外参资料与能力检查；不访问硬件、不写文件')
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--setup', type=Path)
    source.add_argument('--inspect-document', type=Path)
    parser.add_argument('--project-root', type=Path)
    parser.add_argument('--expected-sha256')
    parser.add_argument('--require', dest='require_capability')
    args = parser.parse_args(argv)
    from .hardware_setup import resolve_hardware_setup
    try:
        if args.inspect_document is not None:
            if args.require_capability:
                raise ValueError('mechanical document inspection cannot certify a data capability')
            report=inspect_installation_document(args.inspect_document,expected_sha256=args.expected_sha256)
            print(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False))
            return 0
        if args.project_root is None:
            raise ValueError('--setup requires --project-root for source snapshot verification')
        if args.setup.stat().st_size > 131072:
            raise ValueError('hardware setup exceeds 128 KiB')
        setup = json.loads(args.setup.read_text(encoding='utf-8'))
        resolved = resolve_hardware_setup(setup)
        report = copy.deepcopy(resolved['geometry_report'])
        if setup['schema_version'] == 2:
            report['source_verification'] = verify_geometry_sources(setup, args.project_root)
            if report['source_verification']['status'] != 'PASS':
                print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
                return 2
        if args.require_capability:
            require_geometry_capability(resolved, args.require_capability)
        print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
        return 0
    except (OSError, ValueError, KeyError) as error:
        print(json.dumps({'status': 'BLOCKED', 'reason': str(error), 'hardware_started': False,
                          'configuration_written': False}, ensure_ascii=False))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
