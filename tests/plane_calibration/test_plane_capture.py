"""SourceFrame/SQLite fixtures exercise the actual offline capture loader."""
import hashlib
import importlib
import json
from pathlib import Path
import pickle
import sqlite3
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

from wc_calibration import plane_capture as capture


@pytest.fixture(autouse=True)
def partial_reference_dependencies(monkeypatch):
    """A downloaded reference tree can omit unrelated ROS sensor modules.

    The test uses the actual pairing_capture and pointcloud implementations;
    only optional package exports/discovery are bypassed in that local tree.
    Installed project tests use their complete ordinary packages unchanged.
    """
    try:
        importlib.import_module('wc_runtime.project_paths')
    except ModuleNotFoundError as error:
        if error.name != 'wc_runtime.project_paths':
            raise
        module = ModuleType('wc_runtime.project_paths')
        module.project_root = lambda *args, **kwargs: Path.cwd()
        monkeypatch.setitem(sys.modules, 'wc_runtime.project_paths', module)
    try:
        importlib.import_module('wc_sensors.pointcloud')
    except ModuleNotFoundError as error:
        if error.name not in ('wc_sensors.h30', 'wc_sensors.metadata', 'wc_sensors.preflight'):
            raise
        reference = Path(__file__).resolve().parents[3] / 'reference/src/wc_sensors'
        if not (reference / 'pointcloud.py').is_file():
            raise
        package = ModuleType('wc_sensors')
        package.__path__ = [str(reference)]
        monkeypatch.setitem(sys.modules, 'wc_sensors', package)
        importlib.import_module('wc_sensors.pointcloud')


def _frame(side, sequence, monotonic, *, changed_ray=False, units='m', layout='row_major'):
    # Identical per-pixel directions in every frame; float64 storage makes the
    # ray-invariance test independent of float32 roundoff in this fixture.
    xyz = np.zeros((capture.POINTS, 3), dtype='<f8')
    xyz[:, 0] = 1 + sequence * .001
    if changed_ray:
        xyz[3, 1] = .01
    values = np.column_stack([xyz, np.full(capture.POINTS, 200 + sequence)])
    cloud = SimpleNamespace(width=capture.POINTS, height=1, point_step=32,
                            row_step=capture.POINTS * 32, is_bigendian=False,
                            fields=[SimpleNamespace(name=name, offset=i * 8, datatype=8, count=1)
                                    for i, name in enumerate(('x', 'y', 'z', 'intensity'))],
                            data=values.astype('<f8').tobytes())
    host_ns = monotonic + 1_000_000_000_000
    message = SimpleNamespace(session_id='fixture_pose', side=side, sensor_id='sensor_' + side,
                              stream_epoch='epoch_' + side, frame_sequence=sequence,
                              units=units, coordinate_convention='FLU', source_config_hash='a' * 64,
                              diagnostic_flags=['representation=before_host_filter', 'sdk_width=160',
                                                'sdk_height=60', 'sdk_point_order=' + layout],
                              host_receive_time=SimpleNamespace(sec=host_ns // 10**9,
                                                                nanosec=host_ns % 10**9),
                              host_monotonic_ns=monotonic, cloud=cloud)
    journal = dict(sensor_id=message.sensor_id, stream_epoch=message.stream_epoch, sequence=sequence,
                   host_monotonic_ns=monotonic, host_receive_ns=host_ns,
                   raw_sha256=hashlib.sha256(cloud.data).hexdigest(), raw_valid_count=capture.POINTS)
    return message, journal


@pytest.fixture
def recording(tmp_path):
    def build(*, count=45, duration=20, right_delta_ns=3_000_000,
              bad_units=None, bad_layout=None, payload_mismatch=False, missing_journal=False,
              malformed_point_fields=False):
        project = tmp_path / 'project'
        dataset = project / 'data/experiments/fixture_pose'
        bag = dataset / 'bag'
        bag.mkdir(parents=True)
        start = 10_000_000_000
        manifest = dict(status='COMPLETE', recording_complete=True, session_id='fixture_pose', bag_path='bag',
                        acquisition_window=dict(start_monotonic_ns=start,
                                                end_monotonic_ns=start + duration * 10**9,
                                                requested_duration_ns=duration * 10**9))
        (dataset / 'capture_manifest.json').write_text(json.dumps(manifest), encoding='utf-8')
        with sqlite3.connect(bag / 'bag_0.db3') as db:
            db.execute('CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT)')
            db.execute('CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,data BLOB)')
            for topic_id, side in enumerate(capture.SIDES, 1):
                db.execute('INSERT INTO topics VALUES(?,?,?)',
                           (topic_id, '/wc_mapping/lidar_' + side + '/source_frame', 'wc_interfaces/msg/SourceFrame'))
                journals = []
                # Include retained startup/tail frames around the formal window.
                for sequence, offset in enumerate(np.linspace(-1, duration + 1, count)):
                    mono = start + int(round(offset * 1e9)) + (right_delta_ns if side == 'right' else 0)
                    message, journal = _frame(side, sequence, mono,
                                              units=bad_units if bad_units and sequence == 0 else 'm',
                                              layout=bad_layout if bad_layout and sequence == 0 else 'row_major')
                    if payload_mismatch and side == 'left' and sequence == 0:
                        journal['raw_sha256'] = '0' * 64
                    if malformed_point_fields and side == 'left' and sequence == 0:
                        message.cloud.fields[0].offset = 99
                    db.execute('INSERT INTO messages(topic_id,data) VALUES(?,?)',
                               (topic_id, pickle.dumps(message)))
                    journals.append(journal)
                journal_dir = dataset / 'sources/lidar' / side / ('sensor_' + side)
                journal_dir.mkdir(parents=True)
                (journal_dir / 'source_frames.jsonl').write_text(
                    ''.join(json.dumps(row) + '\n' for row in (journals[:-1] if missing_journal else journals)),
                    encoding='utf-8')
        return project, dataset
    return build


def test_load_binds_sources_and_only_uses_latter_formal_window(recording):
    project, dataset = recording()
    loaded = capture.load_capture(project, dataset, decoder=pickle.loads)
    assert loaded['sensor_ids'] == dict(left='sensor_left', right='sensor_right')
    assert loaded['source_mode'] == 'real' and loaded['units'] == 'm'
    assert loaded['width'] == 160 and loaded['height'] == 60
    assert 5 <= len(loaded['frames']['left']) <= 20
    assert loaded['frames']['left'].shape[1:] == (9600, 3)
    for side in capture.SIDES:
        times = [row['monotonic_ns'] for row in loaded['frame_metadata'][side]]
        assert all(20_000_000_000 <= value <= 30_000_000_000 for value in times)
        report = loaded['provenance']['aggregation'][side]
        assert report['output_valid_pixel_count'] == 9600
        assert report['ray_changed_pixel_count'] == 0
        radial = np.linalg.norm(loaded['frames'][side], axis=2)
        assert np.allclose(np.asarray(loaded['clouds'][side])[:, 0], np.median(radial, axis=0))
        assert report['interpretation'] == 'TEMPORAL_REPEATABILITY_NOT_ACCURACY_OR_STATIC_PROOF'
    assert max(loaded['provenance']['pair_selection']['actual_pair_dt_ns']) == 3_000_000
    assert loaded['provenance']['hardware_synchronization_claimed'] is False
    capture.assert_sources(loaded)
    assert not list((dataset / 'bag').glob('*-wal'))
    assert not list((dataset / 'bag').glob('*-journal'))
    (dataset / 'capture_manifest.json').write_text('{}')
    with pytest.raises(ValueError, match='Bound original source changed'):
        capture.assert_sources(loaded)


@pytest.mark.parametrize('options,error', [
    ({'bad_units': 'mm'}, '米制 FLU'),
    ({'bad_layout': 'column_major'}, 'row-major'),
    ({'payload_mismatch': True}, 'payload SHA256'),
    ({'malformed_point_fields': True}, 'field exceeds point stride'),
    ({'missing_journal': True}, '会话、雷达或原始流'),
    ({'duration': 9}, '10 to 60 second'),
    ({'right_delta_ns': 150_000_000}, 'Fewer than 5'),
    ({'count': 9}, 'Fewer than 5'),
])
def test_invalid_sources_are_rejected_even_for_startup_frames(recording, options, error):
    project, dataset = recording(**options)
    with pytest.raises(ValueError, match=error):
        capture.load_capture(project, dataset, decoder=pickle.loads)


def test_partial_capture_and_live_sqlite_are_rejected(recording):
    project, dataset = recording()
    manifest_path = dataset / 'capture_manifest.json'
    manifest = json.loads(manifest_path.read_text())
    manifest['status'] = 'PARTIAL'
    manifest_path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='finalized COMPLETE'):
        capture.load_capture(project, dataset, decoder=pickle.loads)
    manifest['status'] = 'COMPLETE'
    manifest_path.write_text(json.dumps(manifest))
    (dataset / 'bag/bag_0.db3-wal').write_bytes(b'active')
    with pytest.raises(ValueError, match='not closed'):
        capture.load_capture(project, dataset, decoder=pickle.loads)


def test_source_additions_and_storage_switch_are_rejected(recording):
    project, dataset = recording()
    loaded = capture.load_capture(project, dataset, decoder=pickle.loads)
    added = dataset / 'bag/added.db3'
    added.write_bytes(b'new split bag')
    with pytest.raises(ValueError, match='file set changed'):
        capture.assert_sources(loaded)
    added.unlink()
    config = project / 'config'
    config.mkdir()
    (config / 'storage.json').write_text(json.dumps(dict(schema_version=1, enabled=False)))
    with pytest.raises(ValueError, match='Storage configuration changed'):
        capture.assert_sources(loaded)


def test_journal_is_bounded_before_shared_helper_materializes_rows(recording):
    project, dataset = recording()
    path = next((dataset / 'sources/lidar/left').glob('*/source_frames.jsonl'))
    line = path.read_bytes().splitlines(keepends=True)[0]
    path.write_bytes(line * 1001)
    with pytest.raises(ValueError, match='1000 raw frames'):
        capture.load_capture(project, dataset, decoder=pickle.loads)


def test_outside_current_experiments_is_rejected(recording):
    project, dataset = recording()
    with pytest.raises(ValueError, match='OUTSIDE_ALLOWED_ROOTS|data/experiments'):
        capture.load_capture(project, dataset.parent.parent, decoder=pickle.loads)
    linked = dataset.parent / 'linked'
    try:
        linked.symlink_to(dataset, target_is_directory=True)
    except OSError:
        pytest.skip('Creating symlinks is not permitted on this Windows host')
    with pytest.raises(ValueError, match='link|symlink'):
        capture.load_capture(project, linked, decoder=pickle.loads)


def test_pixel_validity_ray_changes_and_radial_median_do_not_blend_neighbors():
    xyz = np.zeros((20, capture.POINTS, 3))
    xyz[:, :, 0] = 1.0
    # One outlier affects a single pixel, not the adjacent pixel or its median.
    xyz[0, 10, 0] = 9.0
    xyz[:3, 11] = np.nan  # 85%, below the explicit 90% requirement.
    xyz[:2, 12] = np.nan  # 90%, accepted; middle reference remains valid.
    xyz[0, 13, 1] = .01  # Same pixel's ray changed: must be rejected.
    xyz[10, 14] = np.nan  # Exact chosen reference ray is absent.
    amp = np.full((20, capture.POINTS), 123.0)
    median, amplitudes, report = capture._aggregate(xyz, amp)
    assert np.array_equal(median[9], [1, 0, 0])
    assert np.array_equal(median[10], [1, 0, 0])
    assert np.isnan(median[11]).all()
    assert np.array_equal(median[12], [1, 0, 0])
    assert np.isnan(median[13]).all() and np.isnan(median[14]).all()
    assert np.isnan(amplitudes[[11, 13, 14]]).all()
    assert report['ray_changed_pixel_count'] == 1
    assert report['minimum_valid_samples'] == 18
    assert report['spatial_pixel_averaging'] is False
