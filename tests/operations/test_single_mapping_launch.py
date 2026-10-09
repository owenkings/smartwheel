"""Single-lidar processing contracts; no ROS nodes, devices or subprocesses."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType

import pytest


LAUNCH = Path(__file__).resolve().parents[2] / 'src/wc_bringup/launch/single_mapping.launch.py'


@pytest.fixture
def module():
    spec = importlib.util.spec_from_file_location('_single_mapping_launch_test', LAUNCH)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


@pytest.fixture
def session(tmp_path):
    directory = tmp_path / 'reports/single_mapping/SYNTHETIC'
    (directory / 'slam').mkdir(parents=True)
    return tmp_path, directory


@pytest.mark.parametrize('side', ['left', 'right'])
def test_only_native_processing_and_matching_frames(module, session, side):
    root, directory = session
    spec = module.build_spec(side, directory, project_root=root)
    odom, slam = spec['nodes']
    assert [(node['package'], node['executable']) for node in spec['nodes']] == [
        ('rtabmap_odom', 'icp_odometry'), ('rtabmap_slam', 'rtabmap')]
    namespace = '/wc_mapping/single_' + side
    for node in spec['nodes']:
        assert node['namespace'] == namespace
        assert 'arguments' not in node
        params = node['parameters'][0]
        assert params['frame_id'] == 'lidar_' + side
        assert params['Reg/Force3DoF'] == 'false'
        assert params['scan_cloud_is_2d'] is False
        assert params['qos'] == 1
        remaps = dict(node['remappings'])
        assert remaps['scan_cloud'] == namespace + '/scan_cloud'
        assert remaps['odom'] == namespace + '/odom'
        assert remaps['odom_info'] == namespace + '/odom_info'
        assert remaps['/tf'] == namespace + '/tf'
        assert remaps['/tf_static'] == namespace + '/tf_static'
        assert remaps['imu'] == namespace + '/unused_imu'
        assert all(target.startswith(namespace + '/') for target in remaps.values())
    odom_params, slam_params = odom['parameters'][0], slam['parameters'][0]
    assert odom_params['odom_frame_id'] == 'single_' + side + '_odom'
    assert odom_params['guess_frame_id'] == ''
    assert odom_params['wait_imu_to_init'] is False
    assert odom_params['deskewing'] is False
    assert odom_params['Odom/ResetCountdown'] == '0'
    assert slam_params['odom_frame_id'] == ''  # Otherwise native odom sync is disabled.
    assert slam_params['odom_frame_id_init'] == odom_params['odom_frame_id']
    assert slam_params['map_frame_id'] == 'single_' + side + '_map'
    assert slam_params['subscribe_odom'] is True
    assert slam_params['subscribe_odom_info'] is True
    assert slam_params['subscribe_scan_cloud'] is True
    assert slam_params['approx_sync'] is False
    for key in ('subscribe_depth', 'subscribe_rgb', 'subscribe_stereo',
                'subscribe_rgbd', 'subscribe_scan', 'subscribe_scan_descriptor',
                'subscribe_sensor_data', 'subscribe_user_data'):
        assert slam_params[key] is False
    assert slam_params['Optimizer/GravitySigma'] == '0'
    assert slam_params['RGBD/ProximityOdomGuess'] == 'true'
    assert slam_params['Grid/3D'] == 'true'
    assert slam_params['Grid/GroundIsObstacle'] == 'true'
    assert slam_params['Grid/Sensor'] == '0'
    assert slam_params['database_path'] == str(directory / 'slam/rtabmap.db')
    assert slam_params['delete_db_on_start'] is False
    assert list((directory / 'slam').iterdir()) == []  # Validation is read-only.


@pytest.mark.parametrize('side', ['dual', 'single_right', '', '../right'])
def test_rejects_ambiguous_side(module, session, side):
    root, directory = session
    with pytest.raises(ValueError, match='side'):
        module.build_spec(side, directory, project_root=root)


@pytest.mark.parametrize('suffix', ['', '-wal', '-shm', '-journal'])
def test_never_reuses_or_deletes_sqlite_files(module, session, suffix):
    root, directory = session
    existing = directory / ('slam/rtabmap.db' + suffix)
    existing.write_bytes(b'preserve original evidence')
    with pytest.raises(ValueError, match='existing database'):
        module.build_spec('right', directory, project_root=root)
    assert existing.read_bytes() == b'preserve original evidence'


def test_rejects_uncontained_relative_and_parent_traversal(module, session):
    root, directory = session
    for invalid in (root, root / 'reports', Path('reports/single_mapping/session'),
                    directory / '..' / 'SYNTHETIC', root / 'reports/single_mapping'):
        with pytest.raises(ValueError):
            module.build_spec('right', invalid, project_root=root)


def test_requires_controller_created_slam_directory(module, session):
    root, directory = session
    (directory / 'slam').rmdir()
    with pytest.raises(ValueError, match='controller must create'):
        module.build_spec('right', directory, project_root=root)
    assert not (directory / 'slam').exists()


def test_rejects_linked_session_even_if_target_is_inside_project(module, session):
    root, directory = session
    alias = directory.parent / 'ALIAS'
    try:
        alias.symlink_to(directory, target_is_directory=True)
    except OSError as exc:
        pytest.skip('platform cannot create symlinks: ' + str(exc))
    with pytest.raises(ValueError, match='symlink'):
        module.build_spec('right', alias, project_root=root)


def test_each_native_node_exit_shuts_down_launch(module, session, monkeypatch):
    root, directory = session
    monkeypatch.setattr(module, 'PROJECT_ROOT', root)

    class Action:
        def __init__(self, *args, **kwargs):
            self.args, self.kwargs = args, kwargs

    class Node(Action):
        pass

    class Configuration:
        def __init__(self, name):
            self.name = name

        def perform(self, context):
            return context[self.name]

    for name, values in {
        'launch': {}, 'launch_ros': {},
        'launch.actions': {'EmitEvent': Action, 'RegisterEventHandler': Action},
        'launch.event_handlers': {'OnProcessExit': Action},
        'launch.events': {'Shutdown': Action},
        'launch.substitutions': {'LaunchConfiguration': Configuration},
        'launch_ros.actions': {'Node': Node},
    }.items():
        stub = ModuleType(name)
        stub.__dict__.update(values)
        monkeypatch.setitem(sys.modules, name, stub)

    actions = module.configure({'side': 'right', 'session_root': str(directory), 'project_root': str(root)})
    handlers, nodes = actions[:2], actions[2:]
    assert len(handlers) == len(nodes) == 2
    assert all(isinstance(node, Node) for node in nodes)
    for handler, node in zip(handlers, nodes):
        event_handler = handler.args[0]
        assert event_handler.kwargs['target_action'] is node
        event = event_handler.kwargs['on_exit'][0].kwargs['event']
        assert event.kwargs['reason'] == 'A required single-lidar processing node exited'
        assert node.kwargs['sigterm_timeout'] == '20'
        assert node.kwargs['sigkill_timeout'] == '5'
