"""Read-only pointer selection; isolate the existing Linux target import on Windows."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from types import ModuleType
from unittest.mock import patch

import pytest


@pytest.fixture
def entry(tmp_path):
    # Stub only target/ROOT at module import: no supervisor, fcntl, ROS or device
    # function is needed by this pure input resolver. Restore sys.modules at once.
    cli = ModuleType('wc_runtime.cli'); cli.ROOT = tmp_path; cli.target = lambda: None
    path = Path(__file__).resolve().parents[2]/'src/wc_runtime/calibration_picker.py'
    spec = importlib.util.spec_from_file_location('wc_runtime._picker_input_test', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {'wc_runtime.cli': cli}):
        spec.loader.exec_module(module)
    legacy = tmp_path/module.LEGACY_INPUT
    legacy.parent.mkdir(parents=True)
    legacy.write_text(json.dumps({'training': [{'id': 'SYN-LEGACY'}]}))
    return module


def write_pointer(entry, *, scene_id='SYN-FRESH'):
    source = entry.ROOT/'reports/calibration/new_scene/prepared.json'
    source.parent.mkdir(parents=True, exist_ok=True)
    source.write_text(json.dumps({'training': [{'id': 'SYN-FRESH'}, {'id': 'SYN-SECOND'}]}))
    data = {'schema_version': 1, 'status': 'READY_FOR_OFFLINE_PICKING',
        'prepared_input': 'reports/calibration/new_scene/prepared.json',
        'prepared_file_sha256': hashlib.sha256(source.read_bytes()).hexdigest()}
    if scene_id is not None:
        data['scene_id'] = scene_id
    pointer = entry.ROOT/entry.INPUT_POINTER
    pointer.parent.mkdir(parents=True, exist_ok=True)
    pointer.write_text(json.dumps(data))
    return source, pointer, data


def test_absent_pointer_retains_existing_legacy_default(entry):
    assert entry.resolve_input() == entry.ROOT/entry.LEGACY_INPUT
    assert not (entry.ROOT/entry.INPUT_POINTER).exists()


def test_valid_pointer_selects_current_scene_and_is_read_only(entry):
    source, pointer, _ = write_pointer(entry)
    before = {p: p.read_bytes() for p in (source, pointer, entry.ROOT/entry.LEGACY_INPUT)}
    assert entry.resolve_input() == source
    assert all(p.read_bytes() == value for p, value in before.items())


def test_legacy_pointer_migrates_without_rewriting_prepared_input(entry):
    source, pointer, _ = write_pointer(entry)
    original = pointer.read_bytes()
    source_bytes = source.read_bytes()
    legacy = entry.ROOT/'state/point_picker_input.json'
    legacy.parent.mkdir()
    pointer.rename(legacy)
    assert entry.resolve_input() == source
    assert source.read_bytes() == source_bytes and pointer.read_bytes() == original
    assert not legacy.exists()


def test_optional_scene_binding_selects_requested_scene(entry):
    source, _, _ = write_pointer(entry, scene_id='SYN-SECOND')
    assert entry.resolve_input(scene_index=1) == source
    with pytest.raises(RuntimeError, match='scene_id'): entry.resolve_input()
    source, _, _ = write_pointer(entry, scene_id=None)
    assert entry.resolve_input() == source


@pytest.mark.parametrize('fault', ['hash', 'malformed', 'status', 'version', 'version_bool', 'digest',
    'missing', 'directory', 'scene', 'negative_index', 'large_index', 'duplicate_key', 'oversized', 'nonfinite'])
def test_existing_bad_pointer_never_falls_back_to_legacy(entry, fault):
    source, pointer, data = write_pointer(entry)
    kwargs = {}
    if fault == 'hash': source.write_text(json.dumps({'training': [{'id': 'CHANGED'}]}))
    elif fault == 'malformed': pointer.write_text('{not-json')
    elif fault == 'status': data['status'] = 'PREPARING'
    elif fault == 'version': data['schema_version'] = 2
    elif fault == 'version_bool': data['schema_version'] = True
    elif fault == 'digest': data['prepared_file_sha256'] = 'not-a-digest'
    elif fault == 'missing': source.unlink()
    elif fault == 'directory':
        source.unlink(); source.mkdir()
    elif fault == 'scene': data['scene_id'] = 'WRONG'
    elif fault == 'negative_index': kwargs['scene_index'] = -1
    elif fault == 'large_index': kwargs['scene_index'] = 99
    elif fault == 'duplicate_key': pointer.write_text('{"schema_version":1,"schema_version":2}')
    elif fault == 'nonfinite': pointer.write_text(json.dumps(dict(data, unexpected=float('nan'))))
    else: pointer.write_text(' '*16385)
    if fault not in ('malformed', 'duplicate_key', 'oversized', 'nonfinite'):
        pointer.write_text(json.dumps(data))
    assert (entry.ROOT/entry.LEGACY_INPUT).is_file()
    with pytest.raises(RuntimeError, match='no legacy fallback'):
        entry.resolve_input(**kwargs)


@pytest.mark.parametrize('path', ['/tmp/prepared.json', 'C:/tmp/prepared.json', 'C:prepared.json',
    '../reports/prepared.json', 'reports/../prepared.json', 'reports/a/../../prepared.json',
    r'reports\calibration\prepared.json', 'data/calibration/prepared.json', 'reports/prepared.txt', '', None])
def test_pointer_paths_cannot_escape_reports_or_use_ambiguous_forms(entry, path):
    _, pointer, data = write_pointer(entry)
    data['prepared_input'] = path; pointer.write_text(json.dumps(data))
    with pytest.raises(RuntimeError, match='no legacy fallback'):
        entry.resolve_input()


def test_explicit_input_overrides_malformed_pointer_and_keeps_project_scope(entry):
    _, pointer, _ = write_pointer(entry)
    pointer.write_text('broken pointer')
    explicit = entry.ROOT/'data/explicit.json'
    explicit.parent.mkdir(); explicit.write_text('{}')
    assert entry.resolve_input(explicit) == explicit
    assert entry.resolve_input(Path('data/explicit.json')) == explicit
    with pytest.raises(RuntimeError, match='normal prepared file'):
        entry.resolve_input(entry.ROOT.parent/'outside.json')


def test_pointer_link_and_dangling_link_do_not_count_as_absent(entry, monkeypatch):
    pointer = entry.ROOT/entry.INPUT_POINTER
    original = Path.is_symlink
    monkeypatch.setattr(Path, 'is_symlink', lambda path: path == pointer or original(path))
    assert not pointer.exists()
    with pytest.raises(RuntimeError, match='symlinks'):
        entry.resolve_input()


def test_prepared_path_link_is_rejected_even_with_matching_hash(entry, monkeypatch):
    source, _, _ = write_pointer(entry)
    original = Path.is_symlink
    monkeypatch.setattr(Path, 'is_symlink', lambda path: path == source or original(path))
    with pytest.raises(RuntimeError, match='no legacy fallback'):
        entry.resolve_input()


def test_launcher_passes_resolved_input_to_existing_backend(entry, monkeypatch):
    from wc_calibration import picker
    source, _, _ = write_pointer(entry)
    calls = []
    monkeypatch.chdir(entry.ROOT)
    monkeypatch.setattr(picker, 'main', lambda options: calls.append(options) or 17)
    assert entry.main(['--no-browser', '--duration', '1']) == 17
    options, = calls
    assert options[options.index('--input')+1] == str(source)
    assert '--open-browser' not in options
    assert options[options.index('--output-root')+1] == str(entry.ROOT/'data/calibration/manual_points')
