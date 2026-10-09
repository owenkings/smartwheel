"""Saved-file and owned-process tests. No ROS, desktop or device is opened."""
from contextlib import closing, contextmanager
import copy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import sqlite3
import struct
import sys
from types import SimpleNamespace
import uuid

import numpy as np
import pytest
import yaml

from wc_runtime import map_viewer as viewer


@pytest.fixture(autouse=True)
def windows_import_only_lock_module(monkeypatch):
    if os.name == 'nt':
        monkeypatch.setitem(sys.modules, 'fcntl', SimpleNamespace(
            LOCK_EX=1, LOCK_NB=2, LOCK_UN=4, flock=lambda *a: None))


@pytest.fixture
def tmp_path():
    parent = Path(__file__).absolute().parent
    path = parent/('.map_viewer_test_'+uuid.uuid4().hex)
    path.mkdir()
    try:
        yield path
    finally:
        resolved = path.resolve()
        assert resolved.parent == parent and resolved.name.startswith('.map_viewer_test_')
        shutil.rmtree(resolved)


def geometry(path, *, binary=False):
    path.mkdir(parents=True, exist_ok=True)
    cloud = path/'map_3d_cloud.ply'
    if binary:
        cloud.write_bytes(b'ply\nformat binary_little_endian 1.0\nelement vertex 2\n'
                          b'property float x\nproperty float y\nproperty float z\nend_header\n'
                          +struct.pack('<ffffff', 0, 1, 2, 3, 4, 5))
    else:
        cloud.write_text('ply\nformat ascii 1.0\nelement vertex 2\n'
                         'property float x\nproperty float y\nproperty float z\nend_header\n'
                         '0 1 2\n3 4 5\n', encoding='ascii')
    (path/'map_3d.pgm').write_bytes(b'P5\n2 2\n255\n'+bytes([0, 255, 205, 0]))
    (path/'map_3d.yaml').write_text('image: map_3d.pgm\nresolution: 0.03\norigin: [1, 2, 0]\n'
                                  'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    return path


def database(path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with closing(sqlite3.connect(path)) as connection:
        connection.execute('CREATE TABLE Node(id INTEGER, stamp REAL)')
        connection.executemany('INSERT INTO Node VALUES (?, ?)', [(1, 1.0), (2, 2.0)])
        connection.commit()
    return path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.mark.parametrize('selection', ['session', 'export', 'database'])
def test_resolves_external_session_export_and_database(tmp_path, selection):
    session = tmp_path/'user maps'/'room 1'
    exported = geometry(session/'export')
    db = database(session/'slam/rtabmap.db')
    chosen = {'session': session, 'export': exported, 'database': db}[selection]
    source = viewer.map_source(chosen)
    report = viewer.inspect_source(source)
    assert report['source'] == str(chosen.resolve())
    assert report['requires_temporary_export'] == (selection == 'database')
    assert not report['hardware_started']
    assert not report['geometry_accuracy_validated']
    if selection != 'export':
        assert report['native_database']['nodes'] == 2
    if selection != 'database':
        assert report['clouds'][0]['vertices'] == 2
        assert report['maps'][0]['occupied_cells'] == 2


def test_check_database_is_read_only_and_needs_no_export_or_target(tmp_path, monkeypatch, capsys):
    db = database(tmp_path/'outside project'/'native.db')
    before = digest(db)
    monkeypatch.setattr(viewer, 'export_database', lambda *a, **k: pytest.fail('check exported data'))
    from wc_runtime import cli
    monkeypatch.setattr(cli, 'target', lambda: pytest.fail('check requires Orin target'))
    assert viewer.main([str(db), '--check'], project_root=tmp_path/'absent project') == 0
    report = json.loads(capsys.readouterr().out)
    assert report['native_database']['nodes'] == 2
    assert report['requires_temporary_export']
    assert digest(db) == before
    assert sorted(p.name for p in db.parent.iterdir()) == ['native.db']


def test_binary_native_export_parser_is_reused(tmp_path):
    directory = geometry(tmp_path/'export', binary=True)
    report = viewer.inspect_geometry(directory)
    assert report['clouds'][0]['format'] == 'binary_little_endian'
    np.testing.assert_array_equal(list(viewer.ply_blocks(directory/'map_3d_cloud.ply'))[0],
                                  [[0, 1, 2], [3, 4, 5]])


def test_saved_map_initial_view_fits_translated_geometry_without_changing_report():
    # These bounds exercise the large, off-origin map that the old 10 m view clipped.
    low=np.array([-16.51089859008789,-17.371883392333984,-4.936623573303223])
    high=np.array([13.952542304992676,6.924933433532715,3.8377819061279297])
    report={'clouds':[{'min_m':low.tolist(),'max_m':high.tolist()}]}
    unchanged=copy.deepcopy(report)
    view=viewer.offline_rviz(report)
    views=view['Visualization Manager']['Views']
    orbit=views['Current'];top=views['Saved'][0]
    center=np.array([orbit['Focal Point'][axis] for axis in ('X','Y','Z')])
    np.testing.assert_allclose(center,(low+high)/2)
    radius=np.linalg.norm(high-low)/2
    # Every AABB corner is inside this sphere; it fits a 45 degree vertical
    # perspective frustum with margin, independently of the oblique yaw/pitch.
    assert math.asin(radius/orbit['Distance']) < math.radians(22.5)
    assert orbit['Distance']>10
    assert (high[0]-low[0])*top['Scale'] < 640
    assert (high[1]-low[1])*top['Scale'] < 400
    assert [top['X'],top['Y']]==pytest.approx(center[:2])
    assert report==unchanged
    assert view['Visualization Manager']['Displays']==viewer.offline_rviz()['Visualization Manager']['Displays']
    assert orbit['Pitch']==.7 and orbit['Yaw']==.8


def test_view_fit_includes_grid_at_its_original_zero_height():
    report={'clouds':[{'min_m':[20,30,5],'max_m':[21,31,6]}],
            'maps':[{'world_xy_min_m':[-10,-20],'world_xy_max_m':[10,20]}]}
    views=viewer.offline_rviz(report)['Visualization Manager']['Views']
    assert views['Current']['Focal Point']=={'X':5.5,'Y':5.5,'Z':3.}
    top=views['Saved'][0]
    assert 31*top['Scale']<640 and 51*top['Scale']<400


def test_single_point_view_has_finite_nonzero_distance_and_scale():
    report={'clouds':[{'min_m':[10,20,30],'max_m':[10,20,30]}]}
    views=viewer.offline_rviz(report)['Visualization Manager']['Views']
    assert views['Current']['Focal Point']=={'X':10.,'Y':20.,'Z':30.}
    assert math.isfinite(views['Current']['Distance']) and views['Current']['Distance']>0
    assert math.isfinite(views['Saved'][0]['Scale']) and views['Saved'][0]['Scale']>0


@pytest.mark.parametrize('low,high',[([0,0],[1,1,1]),([0,0,0],[1,float('nan'),1]),([2,0,0],[1,1,1])])
def test_invalid_view_bounds_rejected(low,high):
    with pytest.raises(ValueError,match='bounds'):
        viewer.offline_rviz({'clouds':[{'min_m':low,'max_m':high}]})


def test_integration_audit_reuses_helpers_without_widening_session_audit():
    path = Path(__file__).parents[1]/'integration/check_mapping_app_export.py'
    spec = importlib.util.spec_from_file_location('saved_map_audit_test', path)
    old = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(old)
    assert old.inspect_ply is viewer.inspect_ply
    assert old.inspect_grid is viewer.inspect_grid
    assert old.publish_saved is viewer.publish_saved
    assert old.audit.__module__ == 'saved_map_audit_test'


def test_bad_ply_or_unclosed_database_rejected(tmp_path):
    directory = geometry(tmp_path/'export', binary=True)
    path = directory/'map_3d_cloud.ply'
    path.write_bytes(path.read_bytes()[:-1])
    with pytest.raises(ValueError, match='Truncated'):
        viewer.inspect_geometry(directory)
    db = database(tmp_path/'original.db')
    Path(str(db)+'-wal').write_bytes(b'pending')
    with pytest.raises(ValueError, match='not closed'):
        viewer.inspect_source(viewer.map_source(db))


def test_yaml_cannot_redirect_image_outside_selected_directory(tmp_path):
    directory = geometry(tmp_path/'export')
    yaml = directory/'map_3d.yaml'
    yaml.write_text(yaml.read_text().replace('image: map_3d.pgm', 'image: ../other.pgm'))
    with pytest.raises(ValueError, match='traversal'):
        viewer.inspect_geometry(directory)


def test_multiple_geometry_files_are_not_silently_mixed(tmp_path):
    directory = geometry(tmp_path/'export')
    shutil.copyfile(directory/'map_3d_cloud.ply', directory/'unrelated.ply')
    with pytest.raises(ValueError, match='exactly one'):
        viewer.inspect_geometry(directory)


def test_native_export_only_sees_own_backup(tmp_path):
    db = database(tmp_path/'source folder'/'native.db')
    source = viewer.map_source(db)
    report = viewer.inspect_source(source)
    before = digest(db)
    temporary = tmp_path/'owned temporary'
    temporary.mkdir()
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        copy = Path(command[-1])
        assert copy != db and copy.parent == temporary/'export'
        assert str(db) not in command
        assert kwargs['timeout'] == 900
        with closing(sqlite3.connect(copy)) as connection:
            connection.execute('CREATE TABLE ExportToolMetadata(value TEXT)')
        geometry(copy.parent, binary=True)
        return SimpleNamespace(returncode=0)

    viewer.export_database(source, report, temporary, project_root=tmp_path, runner=run)
    assert len(calls) == 1 and report['temporary_export']
    assert digest(db) == before
    assert report['clouds'][0]['vertices'] == 2
    viewer.verify_unchanged(report)


def test_native_export_failure_preserves_source(tmp_path):
    db = database(tmp_path/'source.db')
    source = viewer.map_source(db)
    report = viewer.inspect_source(source)
    temporary = tmp_path/'owned'
    temporary.mkdir()
    before = digest(db)

    def fail(command, **kwargs):
        kwargs['stdout'].write(b'native failure detail')
        kwargs['stdout'].flush()
        return SimpleNamespace(returncode=7)

    with pytest.raises(RuntimeError, match='native failure detail'):
        viewer.export_database(source, report, temporary, project_root=tmp_path, runner=fail)
    assert digest(db) == before


class Child:
    def __init__(self, code=None):
        self.code = code
        self.signals = []

    def poll(self):
        return self.code

    def send_signal(self, value):
        self.signals.append(value)
        self.code = 0

    def terminate(self):
        self.code = -15

    def kill(self):
        self.code = -9

    def wait(self, timeout):
        return self.code


@pytest.mark.parametrize('renderer',['software','system'])
def test_window_close_stops_only_owned_publisher_and_uses_offline_domain(tmp_path,renderer):
    report = viewer.inspect_source(viewer.map_source(geometry(tmp_path/'saved')))
    binary = tmp_path/'install/main/wc_bringup/lib/wc_bringup/mapping_rviz'
    binary.parent.mkdir(parents=True)
    binary.touch()
    work = tmp_path/'temporary'
    work.mkdir()
    calls = []
    publisher = Child()
    gui = Child(0)
    environment={'PATH':'/test','__GLX_VENDOR_LIBRARY_NAME':'nvidia'}
    original_environment=dict(environment)
    process_environment=dict(os.environ)

    def spawn(command, **kwargs):
        calls.append((command, kwargs))
        return publisher if len(calls) == 1 else gui

    result = viewer.run_view(report, work, project_root=tmp_path, spawn=spawn,
                             environment=environment, renderer=renderer, stop_requested=lambda: False)
    assert result['reason'] == 'RVIZ_WINDOW_CLOSED'
    assert publisher.signals and not gui.signals
    assert len(calls) == 2
    for command, kwargs in calls:
        assert kwargs['env']['ROS_DOMAIN_ID'] == '84'
        assert kwargs['env']['ROS_LOCALHOST_ONLY'] == '1'
        if renderer=='software':
            assert kwargs['env']['LIBGL_ALWAYS_SOFTWARE'] == '1'
            assert kwargs['env']['__GLX_VENDOR_LIBRARY_NAME'] == 'mesa'
            assert kwargs['env']['GALLIUM_DRIVER'] == 'llvmpipe'
        else:
            assert kwargs['env']['__GLX_VENDOR_LIBRARY_NAME'] == 'nvidia'
            assert 'LIBGL_ALWAYS_SOFTWARE' not in kwargs['env']
        assert 'wc_runtime.component' in command
        assert not any('driver' in part or part == 'bag' for part in command)
    assert environment==original_environment and dict(os.environ)==process_environment
    views=yaml.safe_load((work/'saved_map.rviz').read_text())['Visualization Manager']['Views']
    assert views==viewer.offline_rviz(report)['Visualization Manager']['Views']


@pytest.mark.parametrize('gui_spawn_fails', [False, True])
def test_publisher_failure_or_gui_spawn_failure_reaps_owned_children(tmp_path, gui_spawn_fails):
    report = viewer.inspect_source(viewer.map_source(geometry(tmp_path/'saved')))
    binary = tmp_path/'install/main/wc_bringup/lib/wc_bringup/mapping_rviz'
    binary.parent.mkdir(parents=True)
    binary.touch()
    work = tmp_path/'temporary'
    work.mkdir()
    publisher = Child(None if gui_spawn_fails else 1)
    gui = Child()
    calls = []

    def spawn(command, **kwargs):
        calls.append(command)
        if len(calls) == 1:
            return publisher
        if gui_spawn_fails:
            raise OSError('GUI unavailable')
        return gui

    with pytest.raises((OSError, ValueError), match='GUI unavailable|publisher exited'):
        viewer.run_view(report, work, project_root=tmp_path, spawn=spawn,
                        environment={}, stop_requested=lambda: False)
    assert publisher.poll() is not None
    if not gui_spawn_fails:
        assert gui.signals


def test_main_removes_only_own_temporary_files(tmp_path, monkeypatch, capsys):
    saved = geometry(tmp_path/'external saved')
    before = {p.name: digest(p) for p in saved.iterdir()}
    created = []

    @contextmanager
    def temporary_directory(**kwargs):
        work = tmp_path/'own temp'
        work.mkdir()
        created.append(work)
        try:
            yield str(work)
        finally:
            assert work.resolve().parent == tmp_path.resolve()
            shutil.rmtree(work)

    from wc_runtime import cli
    monkeypatch.setattr(cli, 'target', lambda: None)
    monkeypatch.setitem(sys.modules, 'fcntl', SimpleNamespace(LOCK_EX=1, LOCK_NB=2, flock=lambda *a: None))
    monkeypatch.setattr(viewer.tempfile, 'TemporaryDirectory', temporary_directory)
    monkeypatch.setattr(viewer, 'run_view', lambda *a, **k: {'status': 'VIEW_CLOSED', 'reason': 'TEST'})
    monkeypatch.setattr(viewer, 'export_database', lambda *a, **k: pytest.fail('exported an existing export'))
    assert viewer.main([str(saved)], project_root=tmp_path/'project') == 0
    assert created and not created[0].exists()
    assert {p.name: digest(p) for p in saved.iterdir()} == before
    assert json.loads(capsys.readouterr().out.splitlines()[-1])['source_unchanged']


def test_change_to_original_is_detected(tmp_path):
    report = viewer.inspect_source(viewer.map_source(geometry(tmp_path/'saved')))
    Path(report['maps'][0]['image']).write_bytes(b'changed')
    with pytest.raises(ValueError, match='changed'):
        viewer.verify_unchanged(report)


def test_main_database_export_failure_cleans_own_copy_and_keeps_original(tmp_path, monkeypatch, capsys):
    db = database(tmp_path/'user data'/'native.db')
    before = digest(db)
    created = []

    @contextmanager
    def temporary_directory(**kwargs):
        work = tmp_path/'own export temporary'
        work.mkdir()
        created.append(work)
        try:
            yield str(work)
        finally:
            assert work.resolve().parent == tmp_path.resolve()
            shutil.rmtree(work)

    def failed_export(source, report, temporary, **kwargs):
        copy = Path(temporary)/'private_copy.db'
        shutil.copyfile(source['database'], copy)
        raise RuntimeError('Synthetic native exporter failure')

    from wc_runtime import cli
    monkeypatch.setattr(cli, 'target', lambda: None)
    monkeypatch.setitem(sys.modules, 'fcntl', SimpleNamespace(LOCK_EX=1, LOCK_NB=2, flock=lambda *a: None))
    monkeypatch.setattr(viewer.tempfile, 'TemporaryDirectory', temporary_directory)
    monkeypatch.setattr(viewer, 'export_database', failed_export)
    monkeypatch.setattr(viewer, 'run_view', lambda *a, **k: pytest.fail('opened GUI after failed export'))
    assert viewer.main([str(db)], project_root=tmp_path/'project') == 1
    assert created and not created[0].exists()
    assert digest(db) == before
    assert json.loads(capsys.readouterr().out)['status'] == 'VIEW_FAILED'

def test_native_export_timeout_keeps_closed_source_and_diagnostic_copy(tmp_path):
    db = database(tmp_path/'source.db')
    before = digest(db)
    source = viewer.map_source(db)
    report = viewer.inspect_source(source)
    temporary = tmp_path/'owned timeout'
    temporary.mkdir()

    def timeout(command, **kwargs):
        backup = Path(command[-1])
        assert backup != db
        with closing(sqlite3.connect(backup.as_uri()+'?mode=ro', uri=True)) as connection:
            assert connection.execute('SELECT * FROM Node ORDER BY id').fetchall() == [(1, 1.0), (2, 2.0)]
        kwargs['stdout'].write(b'export still processing large map')
        kwargs['stdout'].flush()
        raise viewer.subprocess.TimeoutExpired(command, kwargs['timeout'])

    with pytest.raises(RuntimeError, match='900') as caught:
        viewer.export_database(source, report, temporary, project_root=tmp_path, runner=timeout)
    assert isinstance(caught.value.__cause__, viewer.subprocess.TimeoutExpired)
    assert digest(db) == before
    with closing(sqlite3.connect((temporary/'export/native_export_copy.db').as_uri()+'?mode=ro', uri=True)) as connection:
        assert connection.execute('PRAGMA integrity_check').fetchall() == [('ok',)]
        assert connection.execute('SELECT * FROM Node ORDER BY id').fetchall() == [(1, 1.0), (2, 2.0)]
    assert (temporary/'export/export.log').read_bytes() == b'export still processing large map'
    assert 'temporary_export' not in report
    assert 'clouds' not in report and 'maps' not in report


def test_main_export_timeout_reports_log_tail_after_temporary_cleanup(tmp_path, monkeypatch, capsys):
    db = database(tmp_path/'user data'/'native.db')
    before = digest(db)
    created = []
    original_export = viewer.export_database

    @contextmanager
    def temporary_directory(**kwargs):
        work = tmp_path/'own timed out export'
        work.mkdir()
        created.append(work)
        try:
            yield str(work)
        finally:
            assert work.resolve().parent == tmp_path.resolve()
            shutil.rmtree(work)

    def timeout(command, **kwargs):
        assert Path(command[-1]) != db
        kwargs['stdout'].write(b'processing large map: saved diagnostic tail')
        kwargs['stdout'].flush()
        raise viewer.subprocess.TimeoutExpired(command, kwargs['timeout'])

    def export_with_timeout(source, report, temporary, **kwargs):
        return original_export(source, report, temporary, runner=timeout, **kwargs)

    from wc_runtime import cli
    monkeypatch.setattr(cli, 'target', lambda: None)
    monkeypatch.setitem(sys.modules, 'fcntl', SimpleNamespace(LOCK_EX=1, LOCK_NB=2, flock=lambda *a: None))
    monkeypatch.setattr(viewer.tempfile, 'TemporaryDirectory', temporary_directory)
    monkeypatch.setattr(viewer, 'export_database', export_with_timeout)
    monkeypatch.setattr(viewer, 'run_view', lambda *a, **k: pytest.fail('opened GUI after export timeout'))
    assert viewer.main([str(db)], project_root=tmp_path/'project') == 1
    assert created and not created[0].exists()
    assert digest(db) == before
    assert sorted(p.name for p in db.parent.iterdir()) == ['native.db']
    failure = json.loads(capsys.readouterr().out)
    assert failure['status'] == 'VIEW_FAILED'
    assert '900' in failure['reason']
    assert 'processing large map: saved diagnostic tail' in failure['reason']
    assert str(created[0]) not in failure['reason']
