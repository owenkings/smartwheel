"""Recorded source choice must affect commands and reject absent data."""
import hashlib
import json
import sqlite3
from pathlib import Path
from types import SimpleNamespace

import pytest

from wc_panel import catalog
from wc_panel.backend import PanelBackend
from wc_panel.storage import atomic_json


def bag_file(path, streams):
    path.parent.mkdir(parents=True, exist_ok=True)
    with sqlite3.connect(path) as db:
        db.execute('CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT)')
        db.execute('CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB)')
        for i, (side, cloud, count) in enumerate(streams, 1):
            topic = '/wc_mapping/lidar_' + side + '/source_frame' + ('_filtered' if cloud == 'filtered' else '')
            db.execute('INSERT INTO topics VALUES(?,?,?)', (i, topic, 'wc_interfaces/msg/SourceFrame'))
            for _ in range(count):
                db.execute('INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)', (i, i, b'fixture'))
    db.close()


@pytest.fixture
def case(tmp_path, monkeypatch):
    # Destination mount guards are tested separately; keep these SQLite and
    # planning tests portable to Windows without inventing Linux /proc files.
    monkeypatch.setattr('wc_panel.backend.resolve_user_destination',
                        lambda project, path, **kw: (Path(path), SimpleNamespace(check=lambda: None)))
    project = tmp_path/'project'; (project/'config').mkdir(parents=True)
    atomic_json(project/'config/storage.json', {'schema_version': 1, 'enabled': False})
    source = project/'data/experiments/dual-recording'; (source/'bag').mkdir(parents=True)
    output = project/'data/analysis'; output.mkdir()
    atomic_json(source/'runtime_config.json', {'mode': 'all'})
    atomic_json(source/'capture_manifest.json', {'status': 'COMPLETE', 'recording_complete': True,
                'runtime_config_path': 'runtime_config.json', 'bag_path': 'bag', 'mode': 'all'})
    return project, source, output


def fill(case, sides=('left', 'right'), clouds=('raw', 'filtered')):
    project, source, output = case
    bag_file(source/'bag/data.db3', [(side, cloud, 1) for side in sides for cloud in clouds])
    return PanelBackend(project), source, output


def file_hashes(source):
    return {str(path.relative_to(source)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in source.rglob('*') if path.is_file()}


def test_catalogue_uses_nonempty_topics_and_split_bags_instead_of_manifest_mode(case):
    _, source, _ = case
    bag_file(source/'bag/one.db3', [('left', 'raw', 2), ('right', 'raw', 0)])
    bag_file(source/'bag/two.db3', [('left', 'filtered', 1)])
    before = file_hashes(source)
    row = catalog.recording(source)
    assert row['sides'] == 'all'  # Intended capture mode stays visible separately.
    assert row['available_sides'] == ['left']
    assert row['available_clouds_by_side'] == {'left': ['raw', 'filtered'], 'right': []}
    assert row['source_inventory_error'] == ''
    assert file_hashes(source) == before


@pytest.mark.parametrize('side', ['left', 'right', 'all'])
def test_all_algorithm_paths_receive_the_real_selected_source_and_leave_input_unchanged(case, side):
    backend, source, output = fill(case)
    before = file_hashes(source)
    plan = backend.plan_offline(source, output, {'all': True, 'sides': side})
    assert plan['sides'] == plan['options']['sides'] == side
    assert plan['selected_sides'] == (['left', 'right'] if side == 'all' else [side])
    operations = set()
    for task in plan['tasks']:
        command = task['commands'][0]
        assert command[command.index('--sides')+1] == side
        operations.update(value for value in command if value in ('compare', 'refine', 'wc_panel.offline_worker'))
    assert operations == {'compare', 'refine', 'wc_panel.offline_worker'}
    assert file_hashes(source) == before and not Path(plan['output']).exists()


@pytest.mark.parametrize('side', ['left', 'right'])
def test_default_single_sensor_recording_never_pretends_to_be_dual(case, side):
    backend, source, output = fill(case, sides=(side,))
    plan = backend.plan_offline(source, output)
    assert plan['sides'] == side and plan['selected_sides'] == [side]
    for missing in ('all', 'right' if side == 'left' else 'left'):
        with pytest.raises(ValueError, match='没有所选雷达'):
            backend.plan_offline(source, output, {'sides': missing})


def test_selected_variant_cloud_must_exist_on_every_requested_sensor(case):
    backend, source, output = fill(case, clouds=('raw',))
    raw = next(row['id'] for row in catalog.variants() if row['enabled'] and row['cloud'] == 'raw')
    filtered = next(row['id'] for row in catalog.variants() if row['enabled'] and row['cloud'] == 'filtered')
    assert backend.plan_offline(source, output, {'variants': [raw], 'sides': 'left'})['sides'] == 'left'
    with pytest.raises(ValueError, match='没有录制 filtered'):
        backend.plan_offline(source, output, {'variants': [filtered], 'sides': 'left'})


def test_select_all_excludes_algorithms_for_cloud_types_absent_from_recording(case):
    backend, source, output = fill(case, clouds=('raw',))
    plan = backend.plan_offline(source, output, {'all': True, 'sides': 'right'})
    assert plan['comparison_count'] == 11
    assert {task['variant']['cloud'] for task in plan['tasks'] if task['variant']} == {'raw'}


def test_mixed_cloud_motion_candidates_are_separate_and_matched_to_each_algorithm(case):
    backend, source, output = fill(case)
    plan = backend.plan_offline(source, output, {'all': True, 'sides': 'left'})
    candidates = {}
    estimators = {'raw': set(), 'filtered': set()}
    for task in plan['tasks']:
        command = task['commands'][0]
        cloud = command[command.index('--cloud')+1]
        if task['variant'] is None:
            candidates[cloud] = str(Path(task['output'])/'motion_candidate.json')
            assert task['id'] == 'motion_candidate_'+cloud
    assert set(candidates) == {'raw', 'filtered'}
    assert len(set(candidates.values())) == 2 and plan['task_count'] == 24
    for task in plan['tasks']:
        variant = task['variant']
        if variant is None or not variant['motion_correction']: continue
        command = task['commands'][0]
        assert command[command.index('--motion-candidate')+1] == candidates[variant['cloud']]
        estimators[variant['cloud']].add(variant['estimator'])
    assert all(values == {'five_state', 'robot_localization'} for values in estimators.values())


def test_single_motion_cloud_uses_its_stream_even_when_first_variant_is_other_cloud(case):
    backend, source, output = fill(case)
    raw = next(v['id'] for v in catalog.variants() if v['enabled'] and v['cloud'] == 'raw' and not v['motion_correction'])
    filtered = next(v['id'] for v in catalog.variants() if v['enabled'] and v['cloud'] == 'filtered' and v['motion_correction'])
    plan = backend.plan_offline(source, output, {'variants': [raw, filtered]})
    task, = [task for task in plan['tasks'] if task['variant'] is None]
    command = task['commands'][0]
    assert command[command.index('--cloud')+1] == 'filtered'
    assert task['id'] == 'motion_candidate' and Path(task['output']).name == '_motion_candidate'
    assert plan['task_count'] == 3


def test_broken_source_inventory_is_reported_and_cannot_be_planned(case):
    project, source, output = case
    (source/'bag/broken.db3').write_bytes(b'not a database')
    row = catalog.recording(source)
    assert row['available_sides'] == [] and '无法读取' in row['source_inventory_error']
    with pytest.raises(ValueError, match='无法读取'):
        PanelBackend(project).plan_offline(source, output)


def test_invalid_source_choices_do_not_fall_back_to_dual(case):
    backend, source, output = fill(case)
    for sides in ('auto', 'both', '', True, ['left']):
        with pytest.raises(ValueError):
            backend.plan_offline(source, output, {'sides': sides})


def test_native_result_catalogue_retains_sensor_selection_label(case):
    _, _, output = case
    run = output/'one'; (run/'cell').mkdir(parents=True)
    selection = {'mode': 'right', 'selected_sides': ['right'], 'single_lidar_diagnostic': True}
    atomic_json(run/'result.json', {'status': 'COMPLETE', 'cells': {'cell': {}}, 'source_selection': selection})
    row, = catalog.results(output)
    assert row['source_selection'] == selection and '仅右雷达' in row['name']
