"""Human units for data-frame extrinsics, with explicit unknowns and provenance.

The calculations here are previews. They never certify measurements or replace
the runtime's source, assembly and capability validation.
"""
import copy
import math
from collections import deque

import numpy as np

from wc_runtime.calibration_geometry import finite_array, proper_rotation


FRAME_NAMES = {'axle': '轮轴中心', 'mounting_M': '安装基准 M',
               'lidar_left': '左雷达数据原点', 'lidar_right': '右雷达数据原点',
               'imu_native': 'IMU 数据原点'}
CORE_FRAMES = ('mounting_M', 'lidar_left', 'lidar_right', 'imu_native')
STATUS_NAMES = {'UNKNOWN': '未确定', 'CAD_NOMINAL': '机械标称（不可作实测外参）',
                'LEGACY_CANDIDATE': '候选（未核验）',
                'USER_MEASURED_EXPERIMENT': '人工实测（实验）', 'CALIBRATED': '已标定'}
USABLE_STATUSES = {'USER_MEASURED_EXPERIMENT', 'CALIBRATED'}


def frame_name(frame):
    return FRAME_NAMES.get(frame, frame)


def rpy_matrix(degrees):
    """R_parent_child = Rz(yaw) @ Ry(pitch) @ Rx(roll), degrees, right handed."""
    values = finite_array(list(degrees), (3,), '横滚/俯仰/偏航角')
    r, p, y = np.radians(values)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]


def matrix_rpy(matrix):
    """Canonical ZYX display angles; at gimbal lock roll is fixed to zero."""
    value = proper_rotation(matrix)
    pitch = math.atan2(-value[2, 0], math.hypot(value[0, 0], value[1, 0]))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(value[2, 1], value[2, 2])
        yaw = math.atan2(value[1, 0], value[0, 0])
    else:
        roll = 0.
        yaw = math.atan2(-value[0, 1], value[1, 1])
    return [float(v) for v in np.degrees([roll, pitch, yaw])]


def principal_values(row):
    translation, rotation = row.get('translation', {}), row.get('rotation', {})
    position = translation.get('value_m')
    matrix = rotation.get('value_matrix')
    return dict(position_mm=None if translation.get('status') == 'UNKNOWN' or position is None
                else (finite_array(position, (3,), '位置') * 1000).tolist(),
                orientation_deg=None if rotation.get('status') == 'UNKNOWN' or matrix is None else matrix_rpy(matrix))


def update_component(original, component, values, status, source_id=None, note=''):
    """Explicit form edit, not a status promotion or source approval.

    A changed known value must carry the user's declaration and a source binding;
    ParameterStore validates the actual source and assembly on save.
    """
    if component not in ('translation', 'rotation'):
        raise ValueError('未知外参分量')
    if status not in STATUS_NAMES:
        raise ValueError('未知外参状态')
    result = copy.deepcopy(original)
    key = 'value_m' if component == 'translation' else 'value_matrix'
    if status == 'UNKNOWN':
        if values is not None:
            raise ValueError('未确定的分量必须留空，不能用零代替')
        result.update({key: None, 'status': status})
        if note.strip():
            result['note'] = note.strip()
        return result
    if values is None or any(item is None or str(item).strip() == '' for item in values):
        raise ValueError('请完整填写三个数值；空白不能按零计算')
    numbers = finite_array(list(values), (3,), '位置/角度')
    if component == 'translation':
        numbers = numbers / 1000
        if np.linalg.norm(numbers) > 3:
            raise ValueError('相对位置超出 3 米，请检查毫米单位和参考点')
        result[key] = numbers.tolist()
    else:
        result[key] = rpy_matrix(numbers.tolist())
    if status in USABLE_STATUSES:
        if not isinstance(source_id, str) or not source_id.strip():
            raise ValueError('实测或标定值需要绑定测量依据')
        if not isinstance(note, str) or not note.strip():
            raise ValueError('请说明测量方法、参考原点及朝向')
        result['evidence'] = [dict(source_id=source_id)]
    else:
        # Entered candidate values cannot inherit an old measured declaration.
        # Schema2 requires provenance even for unknown/candidate components.
        # Retain the original provenance, while the changed status explicitly
        # prevents it from being used as a measured transform.
        result['evidence'] = copy.deepcopy(original.get('evidence', []))
        if source_id:
            result['evidence'] = [dict(source_id=source_id)]
    result.update(status=status, note=note.strip() or '面板中输入的未核验候选；未声明为实测外参。')
    return result


def _graph(transforms, rotation_only=False, usable_only=False):
    graph = {}
    for row in transforms:
        if row.get('unit') != 'm' or row.get('direction') != 'child_to_parent':
            continue
        rotation = row.get('rotation', {})
        translation = row.get('translation', {})
        if rotation.get('status') == 'UNKNOWN' or rotation.get('value_matrix') is None:
            continue
        statuses = [rotation.get('status')]
        value = proper_rotation(rotation['value_matrix'])
        if not rotation_only:
            if translation.get('status') == 'UNKNOWN' or translation.get('value_m') is None:
                continue
            statuses.append(translation.get('status'))
            matrix = np.eye(4)
            matrix[:3, :3] = value
            matrix[:3, 3] = finite_array(translation['value_m'], (3,), '相对位置')
            value = matrix
        if any(status not in STATUS_NAMES for status in statuses):
            continue
        if usable_only and any(status not in USABLE_STATUSES for status in statuses):
            continue
        parent, child = row['parent_frame'], row['child_frame']
        graph.setdefault(parent, []).append((child, value, statuses))
        graph.setdefault(child, []).append((parent, np.linalg.inv(value), statuses))
    return graph


def _connected(graph, origin, dimension):
    found = {origin: (np.eye(dimension), [])}
    queue = deque([origin])
    while queue:
        current = queue.popleft()
        parent, evidence = found[current]
        for child, matrix, statuses in graph.get(current, []):
            value = parent @ matrix
            if child in found:
                if not np.allclose(found[child][0], value, atol=1e-6, rtol=0):
                    raise ValueError('同一坐标系存在相互矛盾的转换路径：' + frame_name(child))
            else:
                found[child] = value, evidence + statuses
                queue.append(child)
    return found


def derived_transforms(transforms, usable_only=False):
    """Compute both absolute and relative previews, preserving unknown position.

    A known IMU rotation can be shown even if its position is not measured.
    """
    graph = _graph(transforms, usable_only=usable_only)
    rotations = _graph(transforms, rotation_only=True, usable_only=usable_only)
    results = []
    for parent, child in (('axle', 'lidar_left'), ('axle', 'lidar_right'),
                          ('lidar_left', 'lidar_right'), ('axle', 'imu_native')):
        full = _connected(graph, parent, 4).get(child)
        rotation = _connected(rotations, parent, 3).get(child)
        values = dict(parent_frame=parent, child_frame=child,
                      label=frame_name(child) + ' → ' + frame_name(parent),
                      matrix=None, position_mm=None, orientation_deg=None, status='UNKNOWN')
        if full:
            matrix, statuses = full
            values.update(matrix=matrix.tolist(), position_mm=(matrix[:3, 3]*1000).tolist())
        elif rotation:
            _, statuses = rotation
        else:
            results.append(values)
            continue
        if rotation:
            values['orientation_deg'] = matrix_rpy(rotation[0].tolist())
        values['status'] = 'DECLARED' if all(status in USABLE_STATUSES for status in statuses) else 'CANDIDATE'
        results.append(values)
    return results


def installation_summary(snapshot):
    """Current device values and the *separate* opt-in offline initial model."""
    groups = {row['id']: row for row in snapshot.get('groups', [])}
    rows = groups.get('extrinsics', {}).get('value', [])
    result = dict(rows=copy.deepcopy(rows), derived=[], offline_initial=None, offline_error='',
                  scope='当前设备配置；已有录包使用各自冻结的配置，修改这里不会重算或改写旧结果。')
    try:
        result['derived'] = derived_transforms(rows)
    except (ValueError, TypeError, KeyError) as error:
        result['error'] = str(error)
    setup = snapshot.get('installation')
    if isinstance(setup, dict):
        try:
            from wc_runtime.hardware_setup import resolve_hardware_setup
            from wc_runtime.compare_geometry import mechanical_initial_geometry
            resolved = resolve_hardware_setup(setup)
            if resolved.get('mounts'):
                result['offline_error'] = '当前配置已有可解析的数据外参，无需机械初值替代。'
            else:
                initial = mechanical_initial_geometry(setup, resolved)['offline_geometry_initialization']
                result['offline_initial'] = copy.deepcopy(initial)
        except (ValueError, TypeError, KeyError) as error:
            result['offline_error'] = str(error)
    else:
        result['offline_error'] = '未提供完整设备配置，不能推断离线初值。'
    return result
