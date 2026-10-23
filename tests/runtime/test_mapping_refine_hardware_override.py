"""Exercise refine input selection without launching ROS or native mapping."""
import json
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

from wc_runtime import mapping_refine


class ReachedDatasetLoader(Exception):
    pass


@pytest.fixture
def boundary(monkeypatch, tmp_path):
    # Keep real storage/path selection. Replace only downstream ROS/native
    # modules, and stop at load_dataset before any output or replay is created.
    calls = []

    def load_dataset(source, **kwargs):
        calls.append((source, kwargs))
        raise ReachedDatasetLoader

    replacements = {
        'mapping_compare': dict(load_dataset=load_dataset, write_trajectory=None, run_native_maps=None),
        'compare_replay': dict(RecordedPackets=None, replay_cell=None),
        'compare_native': dict(digest=None),
        'offline_motion_adapter': dict(prior_factory=None),
        'offline_planar_process': dict(validate_process_noise=None),
        'offline_refinement_inputs': dict(verify_candidate=None, extract_and_calibrate=None),
        'offline_geometry_replay': dict(geometry_cell=None),
    }
    for name, attributes in replacements.items():
        module = ModuleType('wc_runtime.'+name)
        vars(module).update(attributes)
        monkeypatch.setitem(sys.modules, module.__name__, module)
    project = tmp_path/'project'
    project.mkdir()
    source = project/'data'/'source'
    source.mkdir(parents=True)
    args = SimpleNamespace(
        project_root=project, dataset=source, output=project/'data'/'new',
        motion_candidate=None, user_data_paths=False, resume=False,
        input_rate_hz=5., domain=89, native_wall_interval_s=.2,
        native_cells=['calibrated_motion'], native_map=False, geometry='off',
        process_noise='legacy', linear_acceleration_psd=None,
        angular_acceleration_psd=None, allow_partial=False,
        mechanical_initial=True, cloud='filtered',
    )
    return args, calls


def test_cli_accepts_explicit_override_and_retains_none_default(monkeypatch):
    parsed = []
    monkeypatch.setattr(mapping_refine, 'run', lambda args: parsed.append(args) or 0)
    common = ['--dataset', 'capture', '--output', 'new']
    assert mapping_refine.main(common+['--hardware-setup', 'offline.json']) == 0
    assert parsed[-1].hardware_setup == Path('offline.json')
    assert mapping_refine.main(common) == 0
    assert parsed[-1].hardware_setup is None


@pytest.mark.parametrize('include_argument', [False, True])
def test_default_loader_uses_recorded_setup_with_old_and_new_callers(boundary, include_argument):
    args, calls = boundary
    if include_argument:
        args.hardware_setup = None
    with pytest.raises(ReachedDatasetLoader):
        mapping_refine.run(args)
    assert len(calls) == 1
    assert calls[0][1]['hardware_setup'] is None
    assert not args.output.exists()


def test_project_relative_override_is_resolved_before_loading(boundary):
    args, calls = boundary
    args.hardware_setup = Path('config')/'offline.json'
    with pytest.raises(ReachedDatasetLoader):
        mapping_refine.run(args)
    assert calls[0][1]['hardware_setup'] == args.project_root/'config'/'offline.json'
    assert calls[0][1]['mechanical_initial'] is True


def test_storage_policy_rejection_does_not_fall_back_to_recorded_setup(boundary):
    args, calls = boundary
    args.hardware_setup = args.project_root.parent/'outside.json'
    with pytest.raises(ValueError, match='STORAGE_PATH_OUTSIDE_ALLOWED_ROOTS'):
        mapping_refine.run(args)
    assert calls == []
    assert not args.output.exists()


def test_enabled_storage_policy_resolves_historical_override_alias(boundary, monkeypatch):
    from wc_runtime.storage_policy import StoragePolicy
    args, calls = boundary
    archive = args.project_root.parent/'archive'
    (archive/'data'/'source').mkdir(parents=True)
    config = args.project_root/'config'
    config.mkdir()
    (config/'storage.json').write_text(json.dumps(dict(
        schema_version=2, backend='directory', fallback_allowed=False,
        archive_root=str(archive), original_project_root=str(args.project_root),
    )), encoding='utf-8')

    class Guard:
        mount_root = archive

        def check(self):
            return {'status': 'AVAILABLE'}

    monkeypatch.setattr(StoragePolicy, 'destination_guard', lambda self, path: Guard())
    args.hardware_setup = args.project_root/'data'/'offline.json'
    with pytest.raises(ReachedDatasetLoader):
        mapping_refine.run(args)
    assert calls[0][0] == archive/'data'/'source'
    assert calls[0][1]['hardware_setup'] == archive/'data'/'offline.json'
    assert not (archive/'data'/'new').exists()


@pytest.mark.parametrize('guard_fails', [False, True])
def test_explicit_user_override_retains_its_own_destination_guard(boundary, monkeypatch, guard_fails):
    from wc_panel import storage
    args, calls = boundary
    args.user_data_paths = True
    outside = args.project_root.parent/'selected_inputs'
    outside.mkdir()
    args.hardware_setup = outside/'offline.json'
    args.hardware_setup.write_text('{}', encoding='utf-8')
    guarded = []
    checks = []

    class Guard:
        def __init__(self, path):
            self.path = path

        def check(self):
            checks.append(self.path)
            if guard_fails and self.path == outside:
                raise ValueError('selected override mount changed')
            return {'status': 'AVAILABLE'}

    def destination(project, path, *, must_exist=True):
        guarded.append((path, must_exist))
        return path, Guard(path)

    # Real UserDataPaths must register the external file and check its parent
    # mount. If auxiliaries omit the override, base StoragePolicy rejects it.
    monkeypatch.setattr(storage, 'resolve_user_destination', destination)
    expected = ValueError if guard_fails else ReachedDatasetLoader
    with pytest.raises(expected):
        mapping_refine.run(args)
    assert (outside, True) in guarded
    assert outside in checks
    if guard_fails:
        assert calls == []
    else:
        assert calls[0][1]['hardware_setup'] == args.hardware_setup
        assert checks.count(outside) >= 2
    assert not args.output.exists()


def test_loader_rejection_of_explicit_override_is_not_retried_without_it(boundary, monkeypatch):
    args, calls = boundary
    args.hardware_setup = Path('config')/'bad.json'

    def reject(source, **kwargs):
        calls.append((source, kwargs))
        raise ValueError('geometry source evidence hash/catalogue check failed')

    monkeypatch.setattr(sys.modules['wc_runtime.mapping_compare'], 'load_dataset', reject)
    with pytest.raises(ValueError, match='geometry source evidence'):
        mapping_refine.run(args)
    assert len(calls) == 1
    assert calls[0][1]['hardware_setup'] == args.project_root/args.hardware_setup
    assert not args.output.exists()
