"""Actual Linux children and proc fallback; no ROS, hardware or remote access.

Only the pytest caller's newly created process tree is sampled. Test cleanup
closes private stdin pipes so helpers close their children's pipes and reap.
"""

import importlib.util
import errno
import json
import os
from pathlib import Path
import select
import subprocess
import sys

import pytest


_SPEC = importlib.util.spec_from_file_location('resource_sampler_under_test',
                                             Path(__file__).with_name('sample_resources.py'))
sampler = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(sampler)


def wait_line(stream):
    ready, _, _ = select.select([stream], [], [], 5)
    assert ready, 'private test helper did not report its child within five seconds'
    line = stream.readline()
    assert line, 'private test helper exited before reporting its child'
    return json.loads(line)


def helper_code(child_code=None):
    if child_code is None:
        return 'import sys\nsys.stdin.buffer.read()\n'
    return (
        'import json,subprocess,sys\n'
        'child=subprocess.Popen([sys.executable,"-c",' + repr(child_code) + '],stdin=subprocess.PIPE,stdout=subprocess.PIPE)\n'
        'try:\n'
        ' nested=json.loads(child.stdout.readline()) if ' + repr('child=subprocess.Popen' in child_code) + ' else None\n'
        ' print(json.dumps({"child":child.pid,"nested":nested}),flush=True)\n'
        ' sys.stdin.buffer.read()\n'
        'finally:\n'
        ' child.stdin.close()\n'
        ' child.wait(timeout=5)\n'
    )


@pytest.fixture
def private_tree():
    assert sys.platform == 'linux', 'resource process tests require the target Linux proc filesystem'
    code = helper_code(helper_code(helper_code()))
    root = subprocess.Popen([sys.executable, '-c', code], stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    unrelated = subprocess.Popen([sys.executable, '-c', helper_code()], stdin=subprocess.PIPE,
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        message = wait_line(root.stdout)
        owner = sampler.process_stat(root.pid)
        component_pid = message['child']
        leaf_pid = message['nested']['child']
        component = sampler.process_stat(component_pid)
        yield dict(owner=owner, components={component_pid: component['start_ticks']},
                   expected={root.pid, component_pid, leaf_pid}, unrelated=unrelated.pid)
    finally:
        root.stdin.close()
        unrelated.stdin.close()
        # Every helper exits through its own finally and reaps the child it made.
        # No process-name matching, process group or global signal is used.
        root.wait(timeout=8)
        unrelated.wait(timeout=5)
        root.stdout.close()
        root.stderr.close()


def assert_only_private_tree(records, tree):
    assert {record['pid'] for record in records} == tree['expected']
    assert tree['unrelated'] not in {record['pid'] for record in records}
    for record in records:
        assert record['start_ticks'] > 0
        assert record['rss_bytes'] >= 0
        assert record['threads'] >= 1


def test_actual_linux_descendants_and_manifest_components(private_tree):
    records, _ = sampler.discover_tree(private_tree['owner'], {}, private_tree['components'])
    assert_only_private_tree(records, private_tree)


def test_missing_live_children_file_falls_back_without_unrelated_resource_reads(private_tree, monkeypatch):
    original_read = sampler.read_small
    original_stat = sampler.process_stat
    sampled = []

    def without_optional_children(path, *args, **kwargs):
        if path.name == 'children' and str(path).startswith('/proc/'):
            raise FileNotFoundError('emulated kernel without CONFIG_CHECKPOINT_RESTORE')
        return original_read(path, *args, **kwargs)

    def scoped_resource_stat(pid):
        sampled.append(pid)
        assert pid in private_tree['expected'], 'resource counters must not be read for unrelated processes'
        return original_stat(pid)

    monkeypatch.setattr(sampler, 'read_small', without_optional_children)
    monkeypatch.setattr(sampler, 'process_stat', scoped_resource_stat)
    records, events = sampler.discover_tree(private_tree['owner'], {}, private_tree['components'])
    assert_only_private_tree(records, private_tree)
    assert set(sampled) == private_tree['expected']
    assert any(event['event'] == 'proc_children_unavailable_identity_index_used' for event in events)


def test_actual_child_with_wrong_recorded_ticks_is_rejected(private_tree):
    components = {pid: ticks + 1 for pid, ticks in private_tree['components'].items()}
    with pytest.raises(sampler.Blocked, match='PID identity changed'):
        sampler.discover_tree(private_tree['owner'], {}, components)


def test_unrelated_sibling_cannot_be_injected_as_manifest_component(private_tree):
    unrelated = sampler.process_identity(private_tree['unrelated'])
    with pytest.raises(sampler.Blocked, match='no longer belongs'):
        sampler.discover_tree(private_tree['owner'], {}, {unrelated['pid']: unrelated['start_ticks']})


def test_same_pid_new_start_ticks_cannot_enter_a_later_sample(private_tree):
    known = {}
    sampler.discover_tree(private_tree['owner'], known, private_tree['components'])
    known[private_tree['owner']['pid']] += 1
    with pytest.raises(sampler.Blocked, match='previously owned PID identity changed'):
        sampler.discover_tree(private_tree['owner'], known, private_tree['components'])


def test_resource_report_publication_cannot_replace_prior_evidence(tmp_path):
    output = tmp_path / 'resources.json'
    sampler.write_atomic_new(output, dict(status='RECORDED', value=1))
    with pytest.raises(FileExistsError):
        sampler.write_atomic_new(output, dict(status='RECORDED', value=2))
    assert json.loads(output.read_text())['value'] == 1
    assert not list(tmp_path.glob('.resource-*.tmp'))


def test_unbuffered_kernel_reader_reports_eagain_with_path_and_closes_fd(tmp_path, monkeypatch):
    path = tmp_path / 'temp'
    path.write_bytes(b'45000\n')
    original_open, original_close = sampler.os.open, sampler.os.close
    opened, closed = [], []

    def recording_open(value, flags):
        assert flags & os.O_NONBLOCK
        descriptor = original_open(value, flags)
        opened.append(descriptor)
        return descriptor

    def unavailable(descriptor, count):
        assert descriptor in opened
        raise BlockingIOError(errno.EAGAIN, 'thermal temporarily unavailable')

    def recording_close(descriptor):
        closed.append(descriptor)
        original_close(descriptor)

    monkeypatch.setattr(sampler.os, 'open', recording_open)
    monkeypatch.setattr(sampler.os, 'read', unavailable)
    monkeypatch.setattr(sampler.os, 'close', recording_close)
    with pytest.raises(BlockingIOError) as caught:
        sampler.read_small(path)
    assert caught.value.errno == errno.EAGAIN
    assert caught.value.filename == str(path)
    assert opened == closed and len(opened) == 1


def test_temperature_eagain_is_unknown_while_other_zone_is_preserved(monkeypatch):
    zones = [Path('/sys/class/thermal/thermal_zone0'), Path('/sys/class/thermal/thermal_zone1')]

    def zone_glob(path, pattern):
        assert path == Path('/sys/class/thermal') and pattern == 'thermal_zone*'
        return iter(zones)

    def reads(path, limit=65536):
        if path.name == 'type':
            return 'gpu' if path.parent == zones[0] else 'cpu'
        if path.parent == zones[0]:
            raise BlockingIOError(errno.EAGAIN, 'sensor not powered', str(path))
        return '45250\n'

    monkeypatch.setattr(Path, 'glob', zone_glob)
    monkeypatch.setattr(sampler, 'read_small', reads)
    result = sampler.temperature_sample()
    assert result['status'] == 'READABLE'
    assert result['zones'][0]['status'] == 'UNKNOWN'
    assert result['zones'][0]['degrees_c'] is None
    assert result['zones'][0]['read_errors'][0]['errno'] == errno.EAGAIN
    assert result['zones'][1]['degrees_c'] == 45.25


def test_unexpected_collect_error_is_preserved_with_traceback(tmp_path, monkeypatch):
    output = tmp_path / 'resources.json'
    monkeypatch.setattr(sampler, 'target_identity', lambda: {})
    monkeypatch.setattr(sampler, 'scoped_path', lambda path, parent: Path(path))

    def broken(*args):
        raise TypeError('kernel read diagnostic regression')

    monkeypatch.setattr(sampler, 'collect', broken)
    assert sampler.main(['--session', 'resource_failure_test', '--duration', '1', '--output', str(output)]) == 2
    report = json.loads(output.read_text())
    assert report['status'] == 'BLOCKED' and report['summary']['sample_count'] == 0
    assert 'TypeError: kernel read diagnostic regression' in report['failure_traceback']
    assert 'broken' in report['failure_traceback']
