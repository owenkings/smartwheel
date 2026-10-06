"""Relocated evidence retains original source identity and bytes."""
import copy
import hashlib
import json
from pathlib import Path
import shutil

import pytest

from wc_runtime.calibration_geometry import verify_geometry_sources
from wc_runtime.mapping_compare import archived_geometry_source_map


ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def archived(tmp_path):
    setup = json.loads((ROOT/'config/hardware_setup.json').read_text(encoding='utf-8'))
    mapping = archived_geometry_source_map(setup)
    for source in setup['source_documents']:
        destination = tmp_path/mapping[source['snapshot_path']]
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT/source['snapshot_path'], destination)
    return tmp_path, setup, mapping


def test_current_geometry_archive_mapping_passes_without_modifying_original_configuration_or_sources(archived):
    root, setup, mapping = archived
    original = copy.deepcopy(setup)
    before = {name: hashlib.sha256((root/name).read_bytes()).hexdigest() for name in mapping.values()}
    report = verify_geometry_sources(setup, root, source_path_map=mapping)
    assert report['status'] == 'PASS'
    assert setup == original
    assert report['source_path_map'] == mapping
    assert report['sources'][0]['layout_override']['reference_points_checked'] == 11
    for row in report['sources']:
        assert row['snapshot_path'] == mapping[row['declared_snapshot_path']]
    assert before == {name: hashlib.sha256((root/name).read_bytes()).hexdigest() for name in mapping.values()}


def test_relocation_still_checks_original_embedded_raw_document_hash(archived):
    root, setup, mapping = archived
    source = next(row for row in setup['source_documents'] if row['id'] == 'v7_measurement_record_20261005')
    path = root/mapping[source['snapshot_path']]
    path.write_bytes(path.read_bytes()+b'changed')
    source['sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
    assert verify_geometry_sources(setup, root, source_path_map=mapping)['status'] == 'FAIL'


@pytest.mark.parametrize('mapping', [
    [], {'undeclared.md': 'configuration/calibration/other.md'},
    {'config/calibration/sources/sensor_installation_20261002.md': '../outside.md'},
    {'config/calibration/sources/sensor_installation_20261002.md': 'C:/outside.md'},
])
def test_source_mapping_rejects_undeclared_or_escaped_locations(mapping):
    setup = json.loads((ROOT/'config/hardware_setup.json').read_text(encoding='utf-8'))
    with pytest.raises(ValueError):
        verify_geometry_sources(setup, ROOT, source_path_map=mapping)


def test_partial_mapping_does_not_silently_find_missing_other_sources(archived):
    root, setup, mapping = archived
    partial = {next(iter(mapping)): mapping[next(iter(mapping))]}
    assert verify_geometry_sources(setup, root, source_path_map=partial)['status'] == 'FAIL'


def test_capture_mapping_rejects_unfrozen_source_location():
    setup = json.loads((ROOT/'config/hardware_setup.json').read_text(encoding='utf-8'))
    setup['source_documents'][0]['snapshot_path'] = 'private/location.md'
    with pytest.raises(ValueError, match='not frozen'):
        archived_geometry_source_map(setup)
