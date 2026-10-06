"""Synthetic fixed-mount geometry and editable-file contract; no hardware."""
import copy
import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from wc_runtime.hardware_setup import resolve_hardware_setup, rpy_rotation


LEFT_RPY = [-2.962871493851409, 5.053386070879511, 0.]
RIGHT_RPY = [3.275047203719066, 3.496551404509943, 0.]


def setup():
    wheel = json.loads((Path(__file__).resolve().parents[2] /
                        'config/wheel_history_calibration.json').read_text())
    return {'schema_version': 1, 'status': 'EXPERIMENT', 'coordinate_convention': 'FLU',
            'rotation_convention': 'Rz(yaw) Ry(pitch) Rx(roll)',
            'calibration_reference': {'axle_rpy_deg': [0, 0, 0],
                                      'status': 'ASSUMED_LEVEL_NOT_MEASURED'},
            'lidars': {
                'left': {'position_m': [.34, .3125, .52], 'rpy_deg': LEFT_RPY[:],
                         'rotation_reference': 'calibration_level'},
                'right': {'position_m': [.34, -.3125, .52], 'rpy_deg': RIGHT_RPY[:],
                          'rotation_reference': 'calibration_level'}},
            'imu': {'parent_frame': 'lidar_left', 'position_m': None, 'rpy_deg': [0, 0, 0]},
            'wheel_odometry': wheel}


def test_measured_pair_is_rigid_rotation_not_angle_subtraction_or_scale():
    result = resolve_hardware_setup(setup())
    left = np.asarray(result['mounts']['left']['R_axle_lidar'])
    right = np.asarray(result['mounts']['right']['R_axle_lidar'])
    relative = left.T @ right
    assert np.allclose(relative, [[.9996308668862737, -.0015521183352409174, -.027124175505820542],
                                  [-.0014043101213260744, .9940803616590637, -.10863821830676382],
                                  [.027132229567071824, .108636207097166, .9937112340243827]])
    # A complete composition includes the nonzero relative yaw induced by roll/pitch.
    angles = Rotation.from_matrix(relative).as_euler('xyz', degrees=True)
    assert np.allclose(angles, [6.23901084488666, -1.554753040060112, -.08049070193185497])
    assert np.allclose(relative.T @ relative, np.eye(3), atol=1e-12)
    assert np.linalg.det(relative) == pytest.approx(1.)
    assert np.allclose(result['imu_mount']['R_axle_imu'], left)
    assert result['imu_mount']['t_axle_imu_m'] is None
    assert result['imu_mount']['translation_used'] is False


@pytest.mark.parametrize('axis,point,expected', [(0, [0, 1, 0], [0, 0, 1]),
    (1, [1, 0, 0], [0, 0, -1]), (2, [1, 0, 0], [0, 1, 0])])
def test_positive_axes_follow_right_hand_rule(axis, point, expected):
    angles = [0, 0, 0]; angles[axis] = 90
    assert np.allclose(rpy_rotation(angles) @ point, expected)


@pytest.mark.parametrize('parent', ['lidar_left', 'lidar_right', 'axle'])
def test_arbitrary_imu_parent_axes_and_position_compose_as_a_rigid_child(parent):
    value = setup(); value['imu'].update(parent_frame=parent, rpy_deg=[90, -35, 140], position_m=[.08, -.04, .12])
    value['lidars']['left'].update(rotation_reference='axle', rpy_deg=[11, -28, 70])
    value['lidars']['right'].update(rotation_reference='axle', rpy_deg=[-17, 31, -110])
    result = resolve_hardware_setup(value)
    parent_rpy = [0, 0, 0] if parent == 'axle' else value['lidars'][parent[6:]]['rpy_deg']
    parent_translation = np.zeros(3) if parent == 'axle' else np.array(value['lidars'][parent[6:]]['position_m'])
    r_axle_parent = Rotation.from_euler('xyz', parent_rpy, degrees=True).as_matrix()
    r_parent_imu = Rotation.from_euler('xyz', [90, -35, 140], degrees=True).as_matrix()
    assert np.allclose(result['imu_mount']['R_axle_imu'], r_axle_parent @ r_parent_imu)
    assert np.allclose(result['imu_mount']['t_axle_imu_m'],
                       parent_translation + r_axle_parent @ [.08, -.04, .12])
    assert result['imu_mount']['translation_used'] is False


def test_survey_frame_changes_cancel_for_sensor_and_axle_together():
    original = setup()
    before = resolve_hardware_setup(original)
    changed = copy.deepcopy(original)
    common = Rotation.from_euler('xyz', [9, -13, 23], degrees=True).as_matrix()
    changed['calibration_reference'].update(axle_rpy_deg=[9, -13, 23], status='USER_MEASURED_EXPERIMENT')
    for side in ('left', 'right'):
        world_sensor = common @ Rotation.from_euler('xyz', original['lidars'][side]['rpy_deg'], degrees=True).as_matrix()
        changed['lidars'][side]['rpy_deg'] = Rotation.from_matrix(world_sensor).as_euler('xyz', degrees=True).tolist()
    after = resolve_hardware_setup(changed)
    for side in ('left', 'right'):
        assert np.allclose(before['mounts'][side]['R_axle_lidar'], after['mounts'][side]['R_axle_lidar'])
        assert before['mounts'][side]['t_axle_lidar_m'] == after['mounts'][side]['t_axle_lidar_m']
    assert np.allclose(before['imu_mount']['R_axle_imu'], after['imu_mount']['R_axle_imu'])


def test_axle_reference_uncertainty_cancels_relative_rotation_but_not_position_basis():
    value = setup(); before = resolve_hardware_setup(value)
    value['calibration_reference'].update(axle_rpy_deg=[10, 4, 12], status='USER_MEASURED_EXPERIMENT')
    after = resolve_hardware_setup(value)
    def relative_rotation(resolved):
        return np.asarray(resolved['mounts']['left']['R_axle_lidar']).T @ resolved['mounts']['right']['R_axle_lidar']
    assert np.allclose(relative_rotation(before), relative_rotation(after))
    # Axle-measured translation is not a survey vector, so changing only the
    # assumed axle orientation must affect its expression in lidar coordinates.
    displacement_axle = np.array([0, -.625, 0])
    assert not np.allclose(np.asarray(before['mounts']['left']['R_axle_lidar']).T @ displacement_axle,
                           np.asarray(after['mounts']['left']['R_axle_lidar']).T @ displacement_axle)


def test_direct_axle_rotation_is_independent_of_survey_reference():
    value = setup()
    for mount in value['lidars'].values():
        mount['rotation_reference'] = 'axle'
    before = resolve_hardware_setup(value)
    value['calibration_reference'].update(axle_rpy_deg=[-21, 7, 8], status='USER_MEASURED_EXPERIMENT')
    assert resolve_hardware_setup(value) == before


@pytest.mark.parametrize('angles', [[0, 0], [0, 0, 0, 0], None, ['0', 0, 0],
    [True, 0, 0], [181, 0, 0], [0, -181, 0], [0, 0, float('inf')], [float('nan'), 0, 0]])
def test_rpy_rejects_missing_nonnumeric_nonfinite_or_out_of_range(angles):
    with pytest.raises(ValueError):
        rpy_rotation(angles)


def test_backward_or_upside_down_mounts_are_proper_rotations():
    assert np.allclose(rpy_rotation([180, 0, 0]), np.diag([1, -1, -1]))
    assert np.allclose(rpy_rotation([0, 0, -180]), np.diag([-1, -1, 1]))
    for angles in ([180, 0, 0], [0, 0, -180], [-180, 180, 180]):
        assert np.linalg.det(rpy_rotation(angles)) == pytest.approx(1.)


@pytest.mark.parametrize('path,replacement', [
    (('schema_version',), True), (('status',), 'VERIFIED'), (('coordinate_convention',), 'FRU'),
    (('rotation_convention',), 'Rx Ry Rz'), (('calibration_reference', 'status'), 'CALIBRATED'),
    (('calibration_reference', 'axle_rpy_deg'), [1, 0, 0]),
    (('lidars', 'left', 'position_m'), [34, 31.25, 52]),
    (('lidars', 'left', 'position_m'), [2, 2, 2]),
    (('lidars', 'right', 'position_m'), ['0.34', -.3125, .52]),
    (('lidars', 'right', 'rotation_reference'), 'gravity_live'),
    (('imu', 'parent_frame'), 'left'), (('imu', 'position_m'), [None, 0, 0]),
    (('imu', 'rpy_deg'), None), (('wheel_odometry', 'state'), 'VALIDATED'),
    (('wheel_odometry', 'formal_odometry_eligible'), True),
    (('wheel_odometry', 'wheel_radius_m'), True), (('wheel_odometry', 'provenance'), '  ')])
def test_invalid_configuration_is_rejected(path, replacement):
    value = setup(); current = value
    for key in path[:-1]:
        current = current[key]
    current[path[-1]] = replacement
    with pytest.raises(ValueError):
        resolve_hardware_setup(value)


@pytest.mark.parametrize('path', [(), ('calibration_reference',), ('lidars',), ('lidars', 'left'),
                                 ('lidars', 'right'), ('imu',), ('wheel_odometry',)])
def test_unknown_fields_rejected_instead_of_silently_ignoring_user_edits(path):
    value = setup(); current = value
    for key in path:
        current = current[key]
    current['R_axle_lidar_typo'] = np.diag([1, -1, 1]).tolist()
    with pytest.raises(ValueError, match='unknown'):
        resolve_hardware_setup(value)


@pytest.mark.parametrize('path', [('calibration_reference', 'axle_rpy_deg'), ('lidars', 'left', 'rpy_deg'),
                                 ('lidars', 'right', 'position_m'), ('imu', 'position_m'), ('imu', 'rpy_deg')])
def test_missing_geometry_never_defaults_to_identity_or_zero(path):
    value = setup(); current = value
    for key in path[:-1]:
        current = current[key]
    del current[path[-1]]
    with pytest.raises(ValueError, match='missing'):
        resolve_hardware_setup(value)


def test_composed_imu_position_is_bounded_even_if_parent_offset_is_individually_valid():
    value = setup()
    value['lidars']['left'].update(position_m=[2.9, 0, 0], rpy_deg=[0, 0, 0])
    value['imu']['position_m'] = [.2, 0, 0]
    with pytest.raises(ValueError, match='resolved IMU position'):
        resolve_hardware_setup(value)


def test_wheel_values_are_copied_from_one_setup_without_mutating_inputs():
    value = setup()
    value['wheel_odometry'].update(wheel_radius_m=.19, track_width_m=.61, register_to_wheel_rpm=.125,
                                   left_sign=1, right_sign=-1)
    value['_help'] = {'units': 'metres and degrees'}
    value['imu']['note'] = 'Explicitly unknown position, same orientation as parent'
    original = copy.deepcopy(value)
    result = resolve_hardware_setup(value)
    assert value == original
    assert result['wheel_candidate'] == value['wheel_odometry']
    result['wheel_candidate']['wheel_radius_m'] = .2
    assert value['wheel_odometry']['wheel_radius_m'] == .19
    assert result['mount_model'] == 'fixed_hardware_setup_v1'


def visualization():
    return {'wheelchair': {'schematic': True, 'seat_width_m': .46, 'seat_depth_m': .45,
                          'seat_height_above_axle_m': .35, 'back_height_m': .45, 'wheel_width_m': .05},
            'display_ground_projection': True, 'note': 'Display dimensions only; not measured or a collision model'}


def test_legacy_setup_without_visualization_never_invents_display_dimensions():
    value = setup()
    assert 'visualization' not in resolve_hardware_setup(value)
    assert 'visualization' not in value


def test_optional_visualization_is_copied_without_changing_hardware_geometry():
    value = setup(); hardware_only = resolve_hardware_setup(value)
    value['visualization'] = visualization(); before = copy.deepcopy(value)
    result = resolve_hardware_setup(value)
    assert value == before
    assert result.pop('visualization') == value['visualization']
    assert result == hardware_only
    result = resolve_hardware_setup(value)
    result['visualization']['wheelchair']['seat_width_m'] = 1.
    assert value['visualization']['wheelchair']['seat_width_m'] == .46


@pytest.mark.parametrize('mutation', [lambda x: x.update(visualization=None),
    lambda x: x['visualization'].update(ground_height_m=-.685),
    lambda x: x['visualization']['wheelchair'].update(schematic=False),
    lambda x: x['visualization']['wheelchair'].update(seat_width_m=float('nan')),
    lambda x: x['visualization']['wheelchair'].update(wheel_width_m=-.05),
    lambda x: x['visualization']['wheelchair'].update(seat_depth_m=True),
    lambda x: x['visualization'].update(display_ground_projection='true'),
    lambda x: x['visualization'].pop('note')])
def test_optional_visualization_is_strictly_validated(mutation):
    value = setup(); value['visualization'] = visualization(); mutation(value)
    with pytest.raises(ValueError):
        resolve_hardware_setup(value)


def test_current_editable_setup_contains_explicit_schematic_without_altering_mount_resolution():
    path = Path(__file__).resolve().parents[2]/'config/hardware_setup.json'
    value = json.loads(path.read_text(encoding='utf-8'))
    result = resolve_hardware_setup(value)
    assert result['visualization'] == value['visualization']
    assert result['visualization']['wheelchair']['schematic'] is True
    hardware_only = copy.deepcopy(value); hardware_only.pop('visualization')
    assert result['mounts'] == resolve_hardware_setup(hardware_only)['mounts']
