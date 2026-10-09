import copy
import importlib.util
from pathlib import Path

import pytest

from wc_runtime.mapping_profile import resolve_native_profile, validate_profile


def config(mode='right'):
    return {'motion_model': 'planar_ekf', 'mode': mode, 'map_profile': {},
            'mounts': {s: {'t_axle_lidar_m': [.34, y, .52]} for s, y in (('left', .3125), ('right', -.3125))},
            'wheel_candidate': {'wheel_radius_m': .165}}


def test_ground_bounds_translate_from_floor_to_sensor_without_flattening_cloud():
    row = resolve_native_profile(config())
    p = row['parameters']
    assert float(p['Grid/MinGroundHeight']) == pytest.approx(-.785)
    assert float(p['Grid/MaxGroundHeight']) == pytest.approx(-.535)
    assert float(p['Grid/MaxObstacleHeight']) == pytest.approx(1.115)
    assert p['Reg/Force3DoF'] == 'true'
    assert p['Grid/3D'] == 'true'
    assert row['ground_height_validated'] is False


def test_dual_cannot_cast_rays_from_fictitious_common_origin():
    assert resolve_native_profile(config('right'))['parameters']['Grid/RayTracing'] == 'true'
    assert resolve_native_profile(config('left'))['parameters']['Grid/RayTracing'] == 'true'
    assert resolve_native_profile(config('all'))['parameters']['Grid/RayTracing'] == 'false'


def test_changed_installation_height_changes_all_native_height_bounds():
    a = config(); b = copy.deepcopy(a)
    b['mounts']['right']['t_axle_lidar_m'][2] += .2
    pa, pb = [resolve_native_profile(v)['parameters'] for v in (a, b)]
    for key in ('Grid/MinGroundHeight', 'Grid/MaxGroundHeight', 'Grid/MaxObstacleHeight'):
        assert float(pb[key])-float(pa[key]) == pytest.approx(-.2)


def test_zero_native_height_sentinel_cannot_silently_disable_requested_bound():
    value = config(); value['map_profile']['max_ground_height_m'] = .685
    with pytest.raises(ValueError, match='zero'):
        resolve_native_profile(value)


def test_legacy_configuration_preserves_unconstrained_profile():
    assert resolve_native_profile({})['parameters'] == {'Reg/Force3DoF': 'false'}
    value = config(); value['motion_model'] = 'se3_gyro'
    with pytest.raises(ValueError, match='floor model'):
        resolve_native_profile(value)


@pytest.mark.parametrize('bad', [{'ray_tracing': 'all'}, {'cell_size_m': 0}, {'range_max_m': .1},
                               {'max_ground_height_m': float('nan')}, {'keep_unlinked_nodes': 1},
                               {'typo': True}, {'noise_min_neighbors': True}])
def test_invalid_profile_rejected_before_launch(bad):
    with pytest.raises(ValueError):
        validate_profile(bad)


def test_native_launch_consumes_actual_runtime_profile(tmp_path):
    p = Path(__file__).resolve().parents[2]/'src/wc_bringup/launch/mapping_app.launch.py'
    spec = importlib.util.spec_from_file_location('route1_native_launch', p)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    directory = tmp_path/'session'; (directory/'slam').mkdir(parents=True)
    value = config()
    result = module.build_spec(directory, project_root=tmp_path, runtime_config=value)
    native = result['nodes'][-1]['parameters'][0]
    assert native['Reg/Force3DoF'] == 'true'
    assert float(native['Grid/MaxGroundHeight']) == pytest.approx(-.535)
    assert native['RGBD/ProximityBySpace'] == 'false'
    assert native['Mem/NotLinkedNodesKept'] == 'false'
