"""Exercise actual CLI/launch functions with all device/process boundaries replaced.

The source package on PYTHONPATH determines which checkout is tested. No ROS,
serial, UDP or sensor acquisition is performed, including on Windows.
"""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

import pytest

import wc_runtime


SOURCE_ROOT = Path(wc_runtime.__file__).resolve().parent.parent


def load_source(name, path, modules):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


@pytest.fixture
def cli_fixture(tmp_path, monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail('test crossed an actual supervisor/process/hardware boundary')
    # Isolate platform-specific supervision, not the parser or command builder.
    supervisor = ModuleType('wc_runtime.supervisor')
    supervisor.stop_registered = supervisor.write_json = supervisor.shutdown_policy = forbidden
    cli = load_source('wc_runtime._device_policy_test_cli', SOURCE_ROOT/'wc_runtime/cli.py',
                      {'wc_runtime.supervisor': supervisor})
    calls = []
    cli.ROOT, cli.RUN = tmp_path, tmp_path/'.phase1_runtime'
    cli.target = lambda: calls.append(('target',))
    cli.device_preflight = lambda: calls.append(('lidar_preflight',))
    def imu_preflight():
        calls.append(('imu_preflight',))
        return {'state': 'SYNTHETIC_PREFLIGHT_ONLY', 'serial_opened': False}
    cli.imu_preflight = imu_preflight
    cli.begin = lambda *args, **kwargs: calls.append(('begin', args, kwargs)) or 0
    for name in ('Popen', 'run', 'call', 'check_call', 'check_output'):
        monkeypatch.setattr(cli.subprocess, name, forbidden)
    return cli, calls


@pytest.fixture
def launch_fixture():
    """Run the real launch callback with faithful minimal substitution objects."""
    events = []
    class Description:
        def __init__(self, entities): self.entities = entities
    class Argument:
        def __init__(self, name, default_value, description=None):
            self.name, self.default_value = name, default_value
    class Opaque:
        def __init__(self, function): self.function = function
    class Configuration:
        def __init__(self, name): self.name = name
        def perform(self, context): return context[self.name]
    class Action:
        def __init__(self, *args, **kwargs): self.args, self.kwargs = args, kwargs
    class Node(Action):
        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            events.append(('node', self))
    def package(name):
        events.append(('package', name))
        return str(Path('/SYNTHETIC_PACKAGE')/name)
    definitions = {
        'ament_index_python': {},
        'ament_index_python.packages': {'get_package_share_directory': package},
        'launch': {'LaunchDescription': Description},
        'launch.actions': {'DeclareLaunchArgument': Argument, 'EmitEvent': Action,
                           'OpaqueFunction': Opaque, 'RegisterEventHandler': Action},
        'launch.events': {'Shutdown': Action},
        'launch.event_handlers': {'OnProcessExit': Action},
        'launch.substitutions': {'LaunchConfiguration': Configuration},
        'launch_ros': {}, 'launch_ros.actions': {'Node': Node},
    }
    modules = {}
    for name, values in definitions.items():
        module = ModuleType(name); module.__path__ = []
        module.__dict__.update(values); modules[name] = module
    launch = load_source('_wc_device_policy_test_launch', SOURCE_ROOT/'wc_xt_driver/launch/dual_sources.launch.py', modules)
    def execute(overrides=None):
        description = launch.generate_launch_description()
        context = {entity.name: entity.default_value for entity in description.entities if isinstance(entity, Argument)}
        context['session_id'] = 'SYNTHETIC-POLICY-TEST'
        context.update(overrides or {})
        callbacks = [entity.function for entity in description.entities if isinstance(entity, Opaque)]
        assert len(callbacks) == 1
        output = callbacks[0](context)
        nodes = [entity for entity in output if isinstance(entity, Node)]
        return context, nodes
    return execute, events


@pytest.mark.parametrize('seconds', ['0', '3'])
def test_lidar_source_silence_policy_reaches_each_selected_driver(launch_fixture, seconds):
    execute, _ = launch_fixture
    context, nodes = execute({'source_stale_seconds': seconds})
    assert context['source_stale_seconds'] == seconds and len(nodes) == 2
    for node in nodes:
        assert node.kwargs['parameters'][1]['source_stale_seconds'] == int(seconds)


@pytest.mark.parametrize('command,extra,with_imu', [
    ('drivers', [], False), ('drivers', ['--probe'], False),
    ('record', ['--lidar-only'], False), ('record', [], True)])
@pytest.mark.parametrize('policy', [None, 'preserve_current', 'apply_xtcfg'])
def test_actual_cli_policy_reaches_both_launch_nodes_and_record_metadata(cli_fixture, launch_fixture, command, extra, with_imu, policy):
    cli, calls = cli_fixture
    argv = [command, '--session', 'synthetic-policy', '--duration', '5', *extra]
    if policy is not None:
        argv += ['--device-config-policy', policy]
    assert cli.main(argv) == 0
    expected = policy or 'preserve_current'
    assert [item[0] for item in calls] == ['target', *(['imu_preflight'] if with_imu else []), 'lidar_preflight', 'begin']
    _, args, kwargs = calls[-1]
    commands = args[2]
    sensor_commands = [cmd for cmd in commands if 'dual_sources.launch.py' in cmd]
    assert len(sensor_commands) == 1
    sensor = sensor_commands[0]
    assert [token for token in sensor if token.startswith('device_config_policy:=')] == ['device_config_policy:='+expected]
    assert ('read_only_probe:=true' in sensor) == ('--probe' in extra)
    assert 'source_mode:=dual' in sensor
    execute, _ = launch_fixture
    launch_args = dict(token.split(':=', 1) for token in sensor if ':=' in token)
    _, nodes = execute(launch_args)
    assert {node.kwargs['name'] for node in nodes} == {'xt_left', 'xt_right'}
    for node in nodes:
        params = node.kwargs['parameters'][1]
        assert params['device_config_policy'] == expected
        assert params['read_only_probe'] is ('--probe' in extra)
        assert params['allow_hardware'] is True  # Construction only; fake Node never starts hardware.
        assert params['session_id'] == 'synthetic-policy'
        side = node.kwargs['name'].removeprefix('xt_')
        assert Path(params['source_config_path']).name == side+'-2026-09-11.xtcfg'
    if command == 'record':
        evidence = kwargs['recording']
        assert evidence['device_config_policy'] == expected
        assert evidence['sensor_mode'] == 'dual' and evidence['imu_included'] is with_imu
        assert evidence['explicit_lidar_only'] is (not with_imu)
        assert len(commands) == (3 if with_imu else 2)
        if not with_imu:
            assert evidence['imu_preflight']['state'] == 'NOT_REQUESTED'
            assert all(not topic.startswith('/wc_mapping/imu/') for topic in evidence['topics'])
            assert all('wc_imu.ros_node' not in cmd for cmd in commands)
    else:
        assert 'recording' not in kwargs


@pytest.mark.parametrize('command,extra', [('drivers', []), ('drivers', ['--probe']),
    ('record', ['--lidar-only']), ('record', [])])
@pytest.mark.parametrize('invalid', ['unknown', '', 'apply_xtcfg;ignored'])
def test_invalid_cli_policy_is_rejected_before_identity_preflight_and_dispatch(cli_fixture, command, extra, invalid):
    cli, calls = cli_fixture
    with pytest.raises(SystemExit) as error:
        cli.main([command, '--session', 'synthetic-invalid-policy', '--duration', '5', *extra,
                  '--device-config-policy', invalid])
    assert error.value.code == 2
    assert calls == [], 'invalid policy must be rejected before even target or device identity checks'
    assert not (cli.ROOT/'data').exists()


def test_direct_launch_defaults_preserve_and_read_only(launch_fixture):
    execute, _ = launch_fixture
    context, nodes = execute()
    assert context['device_config_policy'] == 'preserve_current'
    assert len(nodes) == 2
    for node in nodes:
        params = node.kwargs['parameters'][1]
        assert params['device_config_policy'] == 'preserve_current'
        assert params['allow_hardware'] is False and params['read_only_probe'] is True


@pytest.mark.parametrize('mode,count', [('dual', 2), ('single_left', 1), ('single_right', 1)])
@pytest.mark.parametrize('policy', ['preserve_current', 'apply_xtcfg'])
def test_direct_launch_policy_is_independent_of_source_mode(launch_fixture, mode, count, policy):
    execute, _ = launch_fixture
    _, nodes = execute({'source_mode': mode, 'device_config_policy': policy})
    assert len(nodes) == count
    assert all(node.kwargs['parameters'][1]['device_config_policy'] == policy for node in nodes)


@pytest.mark.parametrize('invalid', ['unknown', '', 'apply_xtcfg;ignored'])
def test_direct_launch_rejects_policy_before_package_lookup_or_node_creation(launch_fixture, invalid):
    execute, events = launch_fixture
    with pytest.raises(ValueError, match='device_config_policy'):
        execute({'device_config_policy': invalid})
    assert events == []


def test_driver_command_guard_rejects_policy_before_ros_wrapper(cli_fixture, monkeypatch):
    cli, _ = cli_fixture
    monkeypatch.setattr(cli, 'ros_command', lambda *a: pytest.fail('invalid direct command reached ROS wrapper'))
    args = SimpleNamespace(device_config_policy='unknown', mode='dual', session='SYN', duration=5)
    with pytest.raises(ValueError, match='configuration policy'):
        cli.driver_command(args, False)


def test_old_internal_driver_command_call_defaults_to_preserve(cli_fixture):
    cli, _ = cli_fixture
    args = SimpleNamespace(mode='dual', session='SYN', duration=5)
    command = cli.driver_command(args, False)
    assert 'device_config_policy:=preserve_current' in command
