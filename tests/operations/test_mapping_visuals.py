"""SYNTHETIC display geometry checks; no ROS, files, devices or measured maps."""
import copy
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from wc_runtime import mapping_visuals as m


def settings():
    return {'wheelchair': {'schematic': True, 'seat_width_m': .46, 'seat_depth_m': .45,
                          'seat_height_above_axle_m': .35, 'back_height_m': .45, 'wheel_width_m': .05},
            'display_ground_projection': True, 'note': 'Display schematic; dimensions are not measured or a collision model'}


def fixture(mode='all', *, transform=None):
    mount = {'R_axle_lidar': np.eye(3).tolist(), 't_axle_lidar_m': [.34, .3125, .52]}
    right = copy.deepcopy(mount); right['t_axle_lidar_m'][1] *= -1
    config = {'session_id': 'synthetic_display_test', 'source_mode': 'real', 'mode': mode,
              'visualization': settings(), 'wheel_candidate': {'wheel_radius_m': .165, 'track_width_m': .58},
              'mounts': {'left': mount, 'right': right}, 'imu_mount': {'position_m': None}}
    if transform is None:
        transform = np.eye(4)
        transform[:3, 3] = [-.34, .3125 if mode == 'right' else -.3125, -.52]
    initialization = {'session_id': config['session_id'], 'source_mode': 'real', 'status': 'EXPERIMENT',
                      'gravity_input_base_frame': 'lidar_right' if mode == 'right' else 'lidar_left',
                      'T_reference_axle': transform.tolist()}
    return config, initialization


def grid():
    yaw = .3
    return NS(header=NS(frame_id='mapping_map', stamp=NS(sec=12345, nanosec=67890)),
              info=NS(width=3, height=2, resolution=.03, map_load_time=NS(sec=1, nanosec=2),
                      origin=NS(position=NS(x=-1.25, y=2.75, z=0.),
                                orientation=NS(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2)))),
              data=[-1, 0, 100, 25, 75, -1])


def quaternion_rotation(values):
    x, y, z, w = values
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def test_visual_settings_are_validated_and_independently_copied():
    original = settings(); result = m.validate_visualization(original)
    assert result == original and result is not original
    result['wheelchair']['seat_width_m'] = 1.
    assert original['wheelchair']['seat_width_m'] == .46


@pytest.mark.parametrize('key', ['seat_width_m', 'seat_depth_m', 'seat_height_above_axle_m', 'back_height_m', 'wheel_width_m'])
@pytest.mark.parametrize('value', [True, 0, -1, float('nan'), float('inf'), 3.])
def test_display_dimensions_reject_nonphysical_or_nonfinite_values(key, value):
    candidate = settings(); candidate['wheelchair'][key] = value
    with pytest.raises(ValueError):
        m.validate_visualization(candidate)


@pytest.mark.parametrize('mutation', [lambda x: x.update(extra=True),
    lambda x: x['wheelchair'].update(collision=True), lambda x: x['wheelchair'].update(schematic=False),
    lambda x: x.update(display_ground_projection=1), lambda x: x.update(note=''),
    lambda x: x.pop('wheelchair'), lambda x: x['wheelchair'].pop('seat_depth_m')])
def test_settings_require_explicit_schematic_contract(mutation):
    candidate = settings(); mutation(candidate)
    with pytest.raises(ValueError):
        m.validate_visualization(candidate)


@pytest.mark.parametrize('mode', ['left', 'right', 'all'])
def test_selected_sensor_origin_and_wheel_geometry_preserve_session_mounts(mode):
    config, initialization = fixture(mode)
    original = copy.deepcopy((config, initialization))
    scene = m.build_scene(config, initialization)
    assert (config, initialization) == original
    assert scene['ground_z_m'] == pytest.approx(-.685)
    assert not scene['collision_model'] and not scene['native_maps_modified'] and not scene['ground_height_validated']
    specs = {row['id']: row for row in scene['markers']}
    selected = 7 if mode == 'right' else 6
    assert np.allclose(specs[selected]['position_m'], [0, 0, 0])
    assert np.linalg.norm(np.array(specs[0]['position_m'])-specs[1]['position_m']) == pytest.approx(.58)
    assert specs[0]['scale_m'] == pytest.approx([.33, .33, .05])
    assert all(row['frame_id'] == 'mapping_reference' and row['frame_locked'] for row in specs.values())
    assert not any('imu' in row['text'].lower() for row in specs.values())
    assert 'NOT a collision model' in specs[10]['text']


def test_full_rigid_transform_rotates_wheels_seat_and_floor_candidate_without_scaling():
    angle = math.radians(30)
    rotation = np.array([[1, 0, 0], [0, math.cos(angle), -math.sin(angle)], [0, math.sin(angle), math.cos(angle)]])
    transform = np.eye(4); transform[:3, :3] = rotation; transform[:3, 3] = [-.2, .4, -.6]
    config, initialization = fixture(transform=transform)
    specs = {row['id']: row for row in m.build_scene(config, initialization)['markers']}
    assert np.allclose(specs[0]['position_m'], (transform@np.array([0, .29, 0, 1]))[:3])
    assert np.allclose(specs[3]['position_m'], (transform@np.array([.225, 0, .35, 1]))[:3])
    assert np.allclose(quaternion_rotation(specs[3]['orientation_xyzw']), rotation)
    wheel_axis = quaternion_rotation(specs[0]['orientation_xyzw'])@np.array([0, 0, 1])
    assert np.allclose(wheel_axis, rotation@np.array([0, -1, 0]))
    arrow_axis = quaternion_rotation(specs[5]['orientation_xyzw'])@np.array([1, 0, 0])
    assert np.allclose(arrow_axis, rotation@np.array([1, 0, 0]))
    assert m.build_scene(config, initialization)['ground_z_m'] == pytest.approx(-.6-.165*math.cos(angle))


@pytest.mark.parametrize('rotation', [np.eye(3), np.diag([1., -1., -1.]), np.diag([-1., 1., -1.]), np.diag([-1., -1., 1.])])
def test_marker_quaternions_cover_identity_and_half_turns(rotation):
    transform = np.eye(4); transform[:3, :3] = rotation
    config, initialization = fixture(transform=transform)
    for spec in m.build_scene(config, initialization)['markers']:
        assert np.linalg.norm(spec['orientation_xyzw']) == pytest.approx(1.)
    seat = m.build_scene(config, initialization)['markers'][3]
    assert np.allclose(quaternion_rotation(seat['orientation_xyzw']), rotation)


@pytest.mark.parametrize('failure', ['session', 'base', 'scale', 'reflection', 'nonfinite', 'bottom_row'])
def test_wrong_initialization_never_places_a_model(failure):
    config, initialization = fixture()
    if failure == 'session': initialization['session_id'] = 'different_session'
    elif failure == 'base': initialization['gravity_input_base_frame'] = 'lidar_right'
    elif failure == 'scale': initialization['T_reference_axle'][0][0] = 2.
    elif failure == 'reflection': initialization['T_reference_axle'][0][0] = -1.
    elif failure == 'nonfinite': initialization['T_reference_axle'][1][3] = float('nan')
    else: initialization['T_reference_axle'][3][0] = 1.
    with pytest.raises(ValueError):
        m.build_scene(config, initialization)


def test_display_grid_changes_only_z_and_has_no_shared_mutable_native_storage():
    original = grid(); before = copy.deepcopy(original)
    result = m.display_grid(original, -.685)
    assert original == before and result is not original
    assert result.header == original.header and result.info.map_load_time == original.info.map_load_time
    assert result.info.origin.orientation == original.info.origin.orientation
    assert result.info.resolution == original.info.resolution
    assert result.info.width == original.info.width and result.info.height == original.info.height
    assert result.data == original.data and result.data is not original.data
    assert result.info.origin.position.x == original.info.origin.position.x
    assert result.info.origin.position.y == original.info.origin.position.y
    assert result.info.origin.position.z == -.685
    result.data[0] = 100; result.header.stamp.sec = 0
    assert original == before


@pytest.mark.parametrize('failure', ['frame', 'size', 'resolution', 'cells', 'quaternion', 'height'])
def test_bad_grid_is_rejected_without_changing_original(failure):
    value = grid(); candidate = -.685
    if failure == 'frame': value.header.frame_id = 'foreign_map'
    elif failure == 'size': value.info.height = 10
    elif failure == 'resolution': value.info.resolution = float('nan')
    elif failure == 'cells': value.data[0] = 101
    elif failure == 'quaternion': value.info.origin.orientation.w = 0.
    else: candidate = float('inf')
    with pytest.raises(ValueError):
        m.display_grid(value, candidate)


class Marker:
    ADD = 0; CYLINDER = 3; CUBE = 1; SPHERE = 2; ARROW = 0; TEXT_VIEW_FACING = 9
    def __init__(self):
        self.header = NS(); self.pose = NS(position=NS(), orientation=NS())
        self.scale = NS(); self.color = NS(); self.lifetime = NS()


def test_marker_adapter_uses_existing_reference_tf_and_bounded_lifetime():
    config, initialization = fixture()
    stamp = NS(sec=123, nanosec=456)
    array = m.marker_array(m.build_scene(config, initialization), stamp, Marker, NS)
    assert len(array.markers) == 11
    assert len({row.id for row in array.markers}) == 11
    for marker in array.markers:
        assert marker.header.frame_id == 'mapping_reference' and marker.header.stamp == stamp
        assert marker.frame_locked and marker.action == Marker.ADD and marker.lifetime.sec == 2
        assert marker.header.stamp is not stamp
        # ROS generated Vector3/Quaternion/ColorRGBA setters require float,
        # including the text markers' otherwise unused zero X/Y scales.
        for vector in (marker.pose.position, marker.pose.orientation, marker.scale, marker.color):
            assert all(type(value) is float for value in vars(vector).values())


def test_projection_option_is_respected_without_removing_vehicle_schematic():
    config, initialization = fixture()
    config['visualization']['display_ground_projection'] = False
    scene = m.build_scene(config, initialization)
    assert scene['display_ground_projection'] is False and len(scene['markers']) == 11
