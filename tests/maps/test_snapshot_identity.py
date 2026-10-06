"""Import/reload provenance regressions; all inputs here are synthetic fixtures."""

import copy
import hashlib
import json

import pytest

from wc_maps.builder import MapError
from wc_maps.store import MapStore, _snapshot_validate
from .test_store import snapshot_fixture, write_json


@pytest.fixture
def snapshot_inputs(tmp_path):
    source = tmp_path / "snapshot"
    snapshot_fixture(source)
    inputs = {path.name: json.loads(path.read_text()) for path in source.glob("*.json")}
    return source, inputs


def test_consistent_synthetic_import_is_valid(snapshot_inputs):
    _, inputs = snapshot_inputs
    _snapshot_validate(inputs)


@pytest.mark.parametrize("document,field,value", [
    ("snapshot.json", "source_mode", "real"),
    ("graph.json", "source_mode", "real"),
    ("calibration.json", "source_mode", "real"),
    ("config.json", "source_mode", "real"),
    ("observations.json", "source_mode", "real"),
    ("trajectory.json", "source_mode", "real"),
    ("graph.json", "session_id", "another-session"),
    ("calibration.json", "session_id", "another-session"),
    ("config.json", "session_id", "another-session"),
    ("observations.json", "session_id", "another-session"),
    ("trajectory.json", "session_id", "another-session"),
    ("calibration.json", "graph_epoch", "another-epoch"),
    ("config.json", "graph_epoch", "another-epoch"),
    ("observations.json", "graph_epoch", "another-epoch"),
    ("calibration.json", "calibration_id", "another-calibration"),
    ("graph.json", "frame_id", "another-map"),
    ("graph.json", "time_model_id", "another-time-model"),
    ("snapshot.json", "session_id", ""),
])
def test_foreign_provenance_is_rejected_before_commit(snapshot_inputs, tmp_path, document, field, value):
    source, inputs = snapshot_inputs
    inputs[document][field] = value
    write_json(source / document, inputs[document])
    with pytest.raises(MapError):
        MapStore(tmp_path / "maps").save_snapshot(source, "synthetic", "v1")
    assert not (tmp_path / "maps" / "synthetic" / "v1").exists()


@pytest.mark.parametrize("document", ["graph.json", "calibration.json"])
def test_missing_source_mode_cannot_borrow_snapshot_label(snapshot_inputs, document):
    _, inputs = snapshot_inputs
    del inputs[document]["source_mode"]
    with pytest.raises(MapError, match="source_mode"):
        _snapshot_validate(inputs)


@pytest.mark.parametrize("side_index", [0, 1])
@pytest.mark.parametrize("field,value", [
    ("session_id", "another-session"), ("graph_epoch", "another-epoch"), ("source_mode", "real")])
def test_each_original_sensor_frame_is_bound_to_snapshot(snapshot_inputs, side_index, field, value):
    _, inputs = snapshot_inputs
    inputs["observations.json"]["observations"][side_index][field] = value
    with pytest.raises(MapError, match="original observation"):
        _snapshot_validate(inputs)


@pytest.mark.parametrize("field", ["session_id", "graph_epoch"])
def test_original_frame_identity_cannot_be_missing(snapshot_inputs, field):
    _, inputs = snapshot_inputs
    del inputs["observations.json"]["observations"][0][field]
    with pytest.raises(MapError, match="original observation"):
        _snapshot_validate(inputs)


@pytest.mark.parametrize("artifact,field,value", [
    ("manifest.json", "source_mode", "real"),
    ("map_quality.json", "source_mode", "real"),
    ("raw_data_index.json", "session_id", "another-session"),
    ("raw_data_index.json", "graph_epoch", "another-epoch"),
])
def test_rehashed_package_with_contradictory_metadata_is_not_loadable(snapshot_inputs, tmp_path, artifact, field, value):
    source, _ = snapshot_inputs
    store = MapStore(tmp_path / "maps")
    saved = store.save_snapshot(source, "synthetic", "v1")
    directory = store.root / "synthetic" / "v1"
    manifest = copy.deepcopy(saved["manifest"])
    if artifact == "manifest.json":
        manifest[field] = value
    else:
        path = directory / artifact
        data = json.loads(path.read_text())
        data[field] = value
        write_json(path, data)
        content = path.read_bytes()
        manifest["artifacts"][artifact] = {"bytes": len(content), "sha256": hashlib.sha256(content).hexdigest()}
    write_json(directory / "manifest.json", manifest)
    (directory / "manifest.sha256").write_text(hashlib.sha256((directory / "manifest.json").read_bytes()).hexdigest() + "\n")
    # All byte hashes agree. Semantic checks must still reject mixed identities.
    with pytest.raises(MapError, match="disagrees"):
        store.verify("synthetic", "v1")
    with pytest.raises(MapError, match="disagrees"):
        store.load("synthetic", "v1")
