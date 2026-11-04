import hashlib
import os

import pytest

from wc_navigation.rebuild_cli import (arguments, validate_common_frames,
                                       validate_runtime_environment, validate_source_selection)


BASE = ['--project-root', '/tmp/project', '--source-cell', '/tmp/cell']


def test_cli_defaults_declare_all_variants_without_automatic_selection():
    args = arguments(BASE)
    assert args.variants == ['left_normals_ray', 'left_height_ray_05', 'left_height_ray_15']
    assert args.input_hz == 5 and args.wall_interval == .2 and args.frame_timeout == 30


def test_batch_requires_fresh_single_left_not_sparse_dual_subset():
    valid = {'mode': 'left', 'offline_source_selection': {'selected_sides': ['left']}}
    validate_source_selection(valid)
    for value in [None, {}, {'mode': 'left'},
                  {'mode': 'all', 'offline_source_selection': {'selected_sides': ['left']}},
                  {'mode': 'left', 'offline_source_selection': {'selected_sides': ['left']},
                   'offline_lidar_selection': {'selected_sides': ['right']}}]:
        with pytest.raises(ValueError):
            validate_source_selection(value)


@pytest.mark.parametrize('extra', [
    ['--variants', 'left_normals_ray', 'left_normals_ray'],
    ['--input-hz', 'nan'], ['--input-hz', '11'], ['--input-hz', '0'],
    ['--wall-interval', '.01'], ['--wall-interval', 'inf'],
    ['--frame-timeout', '.1'], ['--frame-timeout', '121'],
    ['--variants', 'erase_obstacles'],
])
def test_invalid_or_undeclared_experiments_rejected(extra):
    with pytest.raises(SystemExit):
        arguments(BASE + extra)


def manifest(stamps):
    return {'status': 'DERIVED_FRONTEND_COMPLETE', 'sources_unchanged': True,
            'source_cell': '/immutable/cell', 'source_runtime_sha256': 'a' * 64,
            'source_index_sha256': 'b' * 64, 'retained_frames': len(stamps),
            'eligible_stamp_sha256': hashlib.sha256(('\n'.join(str(x) for x in stamps) + '\n').encode()).hexdigest()}


def rows(stamps):
    return {stamp: {'stamp_ns': stamp} for stamp in stamps}


def test_variants_must_have_exact_observations_not_an_intersection():
    full, subset = [100, 200, 300], [100, 300]
    with pytest.raises(ValueError, match='different original observations'):
        validate_common_frames({'a': manifest(full), 'b': manifest(subset)},
                               {'a': rows(full), 'b': rows(subset)})
    assert validate_common_frames({'a': manifest(full), 'b': manifest(full)},
                                  {'a': rows(full), 'b': rows(full)}) == full


def test_manifest_hash_cannot_hide_changed_index_or_source():
    stamps = [100, 200]
    wrong = manifest(stamps)
    wrong['eligible_stamp_sha256'] = 'f' * 64
    with pytest.raises(ValueError):
        validate_common_frames({'a': wrong}, {'a': rows(stamps)})
    changed = manifest(stamps)
    changed['source_index_sha256'] = 'c' * 64
    with pytest.raises(ValueError, match='different original observations'):
        validate_common_frames({'a': manifest(stamps), 'b': changed},
                               {'a': rows(stamps), 'b': rows(stamps)})
    incomplete = manifest(stamps)
    incomplete['status'] = 'DERIVING'
    with pytest.raises(ValueError):
        validate_common_frames({'a': incomplete}, {'a': rows(stamps)})


@pytest.mark.skipif(os.name != 'posix', reason='supported target environment gate')
def test_domain_and_remote_discovery_are_rejected_before_native_launch(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '83')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.delenv('ROS_DISCOVERY_SERVER', raising=False)
    with pytest.raises(RuntimeError):
        validate_runtime_environment()
    monkeypatch.setenv('ROS_DOMAIN_ID', '94')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '0')
    with pytest.raises(RuntimeError):
        validate_runtime_environment()
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_DISCOVERY_SERVER', '192.0.2.1:11811')
    with pytest.raises(RuntimeError):
        validate_runtime_environment()
    monkeypatch.delenv('ROS_DISCOVERY_SERVER')
    validate_runtime_environment()
