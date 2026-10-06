"""Offline processing, output preservation and RViz contracts; no ROS/devices."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
LAUNCH = ROOT / 'src/wc_bringup/launch/mapping_app.launch.py'
RVIZ = ROOT / 'config/rviz/mapping_live.rviz'


@pytest.fixture
def module():
    spec = importlib.util.spec_from_file_location('_mapping_app_launch_test', LAUNCH)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


@pytest.fixture
def session(tmp_path):
    directory = tmp_path / 'reports/maps/SYNTHETIC'
    (directory / 'slam').mkdir(parents=True)
    return tmp_path, directory


def test_native_six_dof_with_private_prior_tf_and_exact_odom_sync(module, session):
    root, directory = session
    spec = module.build_spec(directory, project_root=root, odometry_source='icp')
    assert spec['odometry_source'] == 'icp'
    assert [(node['package'], node['executable']) for node in spec['nodes']] == [
        ('rtabmap_odom', 'icp_odometry'), ('rtabmap_slam', 'rtabmap')]
    ns = '/wc_mapping/app'
    for node in spec['nodes']:
        assert node['namespace'] == ns
        # Only the reviewed ROS log-level switch is permitted. No application
        # CLI options such as --delete_db_on_start/-d may bypass DB preservation.
        assert node.get('arguments', []) == []
        assert node['ros_arguments'] == ['--log-level', 'warn']
        params = node['parameters'][0]
        assert params['frame_id'] == 'mapping_reference'
        assert params['Reg/Force3DoF'] == 'false'
        assert params['scan_cloud_is_2d'] is False
        assert params['qos'] == 1
        remaps = dict(node['remappings'])
        for topic in ('scan_cloud', 'odom'):
            assert remaps[topic] == ns + '/' + topic
        assert remaps['/tf'] == ns + '/tf'
        assert remaps['/tf_static'] == ns + '/tf_static'
        assert remaps['imu'] == ns + '/unused_imu'
        assert all(target.startswith(ns + '/') for target in remaps.values())
    odom, slam = [node['parameters'][0] for node in spec['nodes']]
    icp_remaps = dict(spec['nodes'][0]['remappings'])
    slam_remaps = dict(spec['nodes'][1]['remappings'])
    assert icp_remaps['odom_info_lite'] == ns + '/odom_info'
    assert icp_remaps['odom_info'] == ns + '/unused_odom_info_full'
    assert slam_remaps['odom_info'] == ns + '/odom_info'
    assert 'odom_info_lite' not in slam_remaps
    assert odom['odom_frame_id'] == 'mapping_odom'
    assert odom['guess_frame_id'] == 'prior_odom'
    assert odom['publish_tf'] is True
    assert odom['wait_imu_to_init'] is False
    assert odom['deskewing'] is False
    assert odom['Odom/ResetCountdown'] == '0'
    assert slam['odom_frame_id'] == ''  # Nonempty disables odom-message sync.
    assert slam['odom_frame_id_init'] == odom['odom_frame_id']
    assert slam['map_frame_id'] == 'mapping_map'
    assert slam['subscribe_odom'] is True
    assert slam['subscribe_odom_info'] is True
    assert slam['subscribe_scan_cloud'] is True
    assert slam['approx_sync'] is False
    for key in ('subscribe_depth', 'subscribe_rgb', 'subscribe_stereo',
                'subscribe_rgbd', 'subscribe_scan', 'subscribe_scan_descriptor',
                'subscribe_sensor_data', 'subscribe_user_data'):
        assert slam[key] is False
    assert slam['Optimizer/GravitySigma'] == '0'
    assert slam['RGBD/ProximityOdomGuess'] == 'true'
    # Native MapsManager publishes OccupancyGrid as "map", not "grid_map".
    assert dict(spec['nodes'][1]['remappings'])['map'] == ns + '/grid_map'
    assert 'map' not in dict(spec['nodes'][0]['remappings'])
    assert slam['database_path'] == str(directory / 'slam/rtabmap.db')
    assert slam['delete_db_on_start'] is False
    assert slam['DbSqlite3/InMemory'] == 'true'
    assert list((directory / 'slam').iterdir()) == []


def test_default_wheel_imu_uses_exact_scan_and_odom_without_icp_or_odom_info(module, session):
    root, directory = session
    spec = module.build_spec(directory, project_root=root)
    assert spec == module.build_spec(directory, project_root=root, odometry_source='wheel_imu')
    assert spec['odometry_source'] == 'wheel_imu'
    assert [(node['package'], node['executable']) for node in spec['nodes']] == [
        ('rtabmap_slam', 'rtabmap')]
    node = spec['nodes'][0]
    params = node['parameters'][0]
    assert params['frame_id'] == 'mapping_reference'
    assert params['odom_frame_id'] == ''
    assert params['odom_frame_id_init'] == 'mapping_odom'
    assert params['map_frame_id'] == 'mapping_map'
    assert params['subscribe_scan_cloud'] is True
    assert params['subscribe_odom'] is True
    assert params['subscribe_odom_info'] is False
    assert params['approx_sync'] is False
    assert not any(key.startswith(('Icp/', 'Odom/', 'OdomF2M/')) for key in params)
    assert 'guess_frame_id' not in params
    assert params['RGBD/Enabled'] == 'true'
    for key in ('RGBD/NeighborLinkRefining', 'RGBD/ProximityByTime', 'RGBD/ProximityBySpace'):
        assert params[key] == 'false'
    assert params['Rtabmap/LoopThr'] == params['RGBD/AggressiveLoopThr'] == '1'
    assert params['RGBD/ProximityPathMaxNeighbors'] == '0'
    remaps = dict(node['remappings'])
    for key in ('scan_cloud', 'odom'):
        assert remaps[key] == '/wc_mapping/app/' + key
    assert remaps['map'] == '/wc_mapping/app/grid_map'
    assert remaps['/tf'] == '/wc_mapping/app/tf'
    assert remaps['/tf_static'] == '/wc_mapping/app/tf_static'
    assert all(target.startswith('/wc_mapping/app/') for target in remaps.values())
    assert params['database_path'] == str(directory / 'slam/rtabmap.db')
    assert params['delete_db_on_start'] is False
    assert params['DbSqlite3/InMemory'] == 'true'
    assert list((directory / 'slam').iterdir()) == []


@pytest.mark.parametrize('source', ['wheel_imu', 'icp'])
def test_ground_classifier_does_not_assume_sensor_origin_is_floor(module, session, source):
    root, directory = session
    slam = module.build_spec(directory, project_root=root, odometry_source=source)['nodes'][-1]['parameters'][0]
    assert slam['Grid/3D'] == 'true'
    assert slam['Grid/Sensor'] == '0'
    assert slam['Grid/NormalsSegmentation'] == 'true'
    assert slam['Grid/GroundIsObstacle'] == 'false'
    assert slam['Grid/FlatObstacleDetected'] == 'true'
    for key in ('Grid/MinGroundHeight', 'Grid/MaxGroundHeight', 'Grid/MaxObstacleHeight'):
        assert slam[key] == '0'
    assert slam['Grid/MapFrameProjection'] == 'true'
    assert slam['Grid/RayTracing'] == 'false'  # No common origin for dual rays.
    assert 0 < float(slam['Grid/MaxGroundAngle']) < 45
    assert slam['RGBD/CreateOccupancyGrid'] == 'true'


@pytest.mark.parametrize('suffix', ['', '-wal', '-shm', '-journal'])
@pytest.mark.parametrize('source', ['wheel_imu', 'icp'])
def test_never_reuses_or_deletes_database_evidence(module, session, suffix, source):
    root, directory = session
    existing = directory / ('slam/rtabmap.db' + suffix)
    existing.write_bytes(b'preserve original evidence')
    with pytest.raises(ValueError, match='existing database'):
        module.build_spec(directory, project_root=root, odometry_source=source)
    assert existing.read_bytes() == b'preserve original evidence'


def test_rejects_relative_outside_root_and_metadata_paths(module, session):
    root, directory = session
    invalid = [root, root.parent / 'outside', Path('reports/maps/SYNTHETIC'),
               directory / '..' / 'SYNTHETIC']
    invalid += [root / name / 'session' for name in
                ('.git', '.codex', '.agents', '.phase1_runtime')]
    for path in invalid:
        with pytest.raises(ValueError):
            module.build_spec(path, project_root=root)


def test_accepts_explicit_custom_project_output(module, session):
    root, _ = session
    directory = root / 'custom_maps/room_a'
    (directory / 'slam').mkdir(parents=True)
    assert module.build_spec(directory, project_root=root)['database_path'] == str(
        directory / 'slam/rtabmap.db')


def test_requires_controller_created_slam_directory(module, session):
    root, directory = session
    (directory / 'slam').rmdir()
    with pytest.raises(ValueError, match='controller must create'):
        module.build_spec(directory, project_root=root)
    assert not (directory / 'slam').exists()


def test_rejects_linked_session_inside_project(module, session):
    root, directory = session
    alias = directory.parent / 'ALIAS'
    try:
        alias.symlink_to(directory, target_is_directory=True)
    except OSError as exc:
        pytest.skip('platform cannot create symlinks: ' + str(exc))
    with pytest.raises(ValueError, match='symlink'):
        module.build_spec(alias, project_root=root)


@pytest.mark.parametrize('source,node_count', [('wheel_imu', 1), ('icp', 2)])
def test_each_native_node_exit_closes_chain_within_owner_timeout(module, session, monkeypatch, source, node_count):
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

    actions = module.configure({'session_root': str(directory), 'odometry_source': source})
    handlers, nodes = actions[:node_count], actions[node_count:]
    assert len(handlers) == len(nodes) == node_count
    assert all(isinstance(node, Node) for node in nodes)
    for handler, node in zip(handlers, nodes):
        event_handler = handler.args[0]
        assert event_handler.kwargs['target_action'] is node
        event = event_handler.kwargs['on_exit'][0].kwargs['event']
        assert event.kwargs['reason'] == 'A required mapping application node exited'
        assert node.kwargs['sigterm_timeout'] == '105'
        assert node.kwargs['sigkill_timeout'] == '5'
        assert node.kwargs['ros_arguments'] == ['--log-level', 'warn']
        assert node.kwargs.get('arguments', []) == []


@pytest.mark.parametrize('source', ['', 'wheel', 'wheel_imu ', 'ICP', 'auto', None, 0, True])
def test_rejects_unknown_odometry_source_before_using_output_path(module, source):
    with pytest.raises(ValueError, match='odometry_source'):
        module.build_spec('/unused', odometry_source=source)


def test_launch_argument_defaults_to_wheel_imu_and_declares_only_two_authorities(module, monkeypatch):
    class Action:
        def __init__(self, *args, **kwargs):
            self.args, self.kwargs = args, kwargs

    launch = ModuleType('launch')
    launch.LaunchDescription = lambda actions: actions
    actions = ModuleType('launch.actions')
    actions.DeclareLaunchArgument = actions.OpaqueFunction = Action
    monkeypatch.setitem(sys.modules, 'launch', launch)
    monkeypatch.setitem(sys.modules, 'launch.actions', actions)
    declarations = module.generate_launch_description()
    source = next(action for action in declarations if action.args == ('odometry_source',))
    assert source.kwargs['default_value'] == 'wheel_imu'
    assert source.kwargs['choices'] == ['wheel_imu', 'icp']
    assert declarations[-1].kwargs['function'] is module.configure


def test_one_rviz_window_has_native_maps_and_real_motion_topics():
    config = yaml.safe_load(RVIZ.read_text(encoding='utf-8'))
    manager = config['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == 'mapping_map'
    displays = {display.get('Topic', {}).get('Value'): display
                for display in manager['Displays']}
    for topic, kind in [('cloud_map', 'PointCloud2'), ('view_grid', 'Map'),
                        ('odom', 'Odometry'), ('prior_odom', 'Odometry'),
                        ('mapPath', 'Path'), ('status_markers', 'MarkerArray')]:
        display = displays['/wc_mapping/app/' + topic]
        assert display['Class'] == 'rviz_default_plugins/' + kind
        assert display['Enabled'] is True
        assert display['Topic']['Reliability Policy'] == 'Reliable'
        durability = 'Transient Local' if topic in ('cloud_map', 'view_grid') else 'Volatile'
        assert display['Topic']['Durability Policy'] == durability
    assert all(not topic or topic.startswith('/wc_mapping/app/') for topic in displays)
    assert all(display['Class'].startswith('rviz_default_plugins/')
               for display in manager['Displays'])
    assert {view['Class'] for view in manager['Views']['Saved']} == {
        'rviz_default_plugins/Orbit', 'rviz_default_plugins/TopDownOrtho'}
    assert not any(tool['Class'].endswith(('SetGoal', 'SetInitialPose'))
                   for tool in manager['Tools'])
