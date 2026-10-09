"""RAM data containment never changes the production code/executable root."""
import json
from pathlib import Path
import shutil
import sys
from types import ModuleType, SimpleNamespace
import uuid

import pytest

from wc_runtime import compare_native, compare_replay


@pytest.fixture
def project_output():
    project = Path(compare_native.__file__).resolve().parents[2]
    parent = project/'tests/runtime'
    directory = parent/('.compare_storage_'+uuid.uuid4().hex)
    directory.mkdir()
    try:
        yield project, directory
    finally:
        assert directory.resolve().parent == parent and directory.name.startswith('.compare_storage_')
        shutil.rmtree(directory)


def native_directory(parent):
    directory = parent/'session/native/cell'
    (directory/'slam').mkdir(parents=True)
    return directory


def test_ram_native_loads_real_launch_but_places_database_under_storage_root(tmp_path):
    project = Path(compare_native.__file__).resolve().parents[2]
    storage = tmp_path/'ram'
    directory = native_directory(storage)
    # A data workspace must never become an alternative source-code tree.
    trap = storage/'src/wc_bringup/launch/mapping_app.launch.py'
    trap.parent.mkdir(parents=True)
    trap.write_text("raise AssertionError('data workspace code must not load')\n", encoding='utf-8')
    spec = compare_native.load_spec(project, directory, {}, storage_root=storage)
    parameters = spec['nodes'][0]['parameters'][0]
    assert parameters['database_path'] == str(directory/'slam/rtabmap.db')
    assert parameters['Rtabmap/WorkingDirectory'] == str(directory/'slam')
    assert parameters['RGBD/NeighborLinkRefining'] == 'false'
    assert parameters['use_sim_time'] is True
    assert not (directory/'slam/rtabmap.db').exists()


def test_default_native_scope_still_rejects_output_outside_real_project(tmp_path):
    project = Path(compare_native.__file__).resolve().parents[2]
    directory = native_directory(tmp_path/'ram')
    with pytest.raises(ValueError, match='inside the project'):
        compare_native.load_spec(project, directory, {})


def test_native_explicit_storage_root_does_not_allow_sibling_escape(tmp_path):
    project = Path(compare_native.__file__).resolve().parents[2]
    storage = tmp_path/'ram'
    storage.mkdir()
    directory = native_directory(tmp_path/'unrelated')
    with pytest.raises(ValueError, match='inside the project'):
        compare_native.load_spec(project, directory, {}, storage_root=storage)


def test_default_native_scope_matches_explicit_original_root(project_output):
    project, output = project_output
    directory = native_directory(output)
    assert compare_native.load_spec(project, directory, {}) == compare_native.load_spec(
        project, directory, {}, storage_root=project)


def stub_module(monkeypatch, name, **members):
    parts = name.split('.')
    for length in range(1, len(parts)+1):
        current = '.'.join(parts[:length])
        if current not in sys.modules:
            module = ModuleType(current)
            module.__path__ = []
            monkeypatch.setitem(sys.modules, current, module)
    module = ModuleType(name)
    module.__dict__.update(members)
    monkeypatch.setitem(sys.modules, name, module)


class ReachedOutputOwner(RuntimeError):
    pass


@pytest.mark.parametrize('ram', [False, True])
def test_replay_routes_only_mapping_input_guard_to_optional_storage_root(tmp_path, monkeypatch, ram):
    from wc_runtime import mapping_input
    stub_module(monkeypatch, 'rclpy.serialization', serialize_message=lambda _: b'')
    stub_module(monkeypatch, 'sensor_msgs.msg', PointCloud2=SimpleNamespace, PointField=SimpleNamespace)
    project = Path(compare_replay.__file__).resolve().parents[2]
    storage = tmp_path/'ram' if ram else None
    output = tmp_path/'outputs'
    output.mkdir()
    seen = []
    def owner(config, guard_root, output_root, **kwargs):
        seen.append((guard_root, output_root, config['wheel_imu_estimator']))
        raise ReachedOutputOwner()
    monkeypatch.setattr(mapping_input, 'MappingInput', owner)
    config = {'prior_template': {}, 'mode': 'all'}
    packets = SimpleNamespace(events=[{'monotonic_ns': 100}],
                              clock=SimpleNamespace(report=lambda: {'mapping': 'SYNTHETIC_ARRIVAL_ONLY'}))
    with pytest.raises(ReachedOutputOwner):
        compare_replay.replay_cell(project, output, config, config, packets,
            'cell', 'five_state', False, 0, 5., storage_root=storage)
    assert seen == [(storage if ram else project, output/'cell/input', 'five_state')]


@pytest.mark.parametrize('ram', [False, True])
def test_native_runner_forwards_data_scope_without_substituting_code_root(tmp_path, monkeypatch, ram):
    stub_module(monkeypatch, 'ament_index_python.packages', get_package_prefix=lambda _: '')
    stub_module(monkeypatch, 'rclpy.serialization', deserialize_message=lambda *_: None)
    stub_module(monkeypatch, 'rclpy.qos', QoSProfile=SimpleNamespace, ReliabilityPolicy=SimpleNamespace)
    for module, names in {
        'rcl_interfaces.srv': ('GetParameters',), 'sensor_msgs.msg': ('PointCloud2',),
        'nav_msgs.msg': ('Odometry',), 'geometry_msgs.msg': ('TransformStamped',),
        'tf2_msgs.msg': ('TFMessage',), 'rosgraph_msgs.msg': ('Clock',),
        'rtabmap_msgs.msg': ('Info', 'MapGraph')}.items():
        stub_module(monkeypatch, module, **dict.fromkeys(names, SimpleNamespace))
    project = Path(compare_native.__file__).resolve().parents[2]
    storage = tmp_path/'ram' if ram else None
    frontend, output = tmp_path/'frontend', tmp_path/'native'
    (frontend/'cell').mkdir(parents=True)
    output.mkdir()
    (frontend/'cell/runtime_config.json').write_text(json.dumps({'odometry_source': 'wheel_imu'}), encoding='utf-8')
    seen = []
    def load(real_root, directory, config, *, storage_root=None):
        seen.append((real_root, directory, storage_root))
        raise ReachedOutputOwner()
    monkeypatch.setattr(compare_native, 'load_spec', load)
    with pytest.raises(ReachedOutputOwner):
        compare_native.run_cell(project, frontend, output, 'cell', {}, [], None, None,
            storage_root=storage)
    assert seen == [(project, output/'cell', storage)]
