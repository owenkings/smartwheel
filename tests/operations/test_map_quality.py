"""Actual RTAB 0.23.7 encodings, with separate static/file/moving qualification."""
from contextlib import closing
import hashlib
import sqlite3
import struct
import zlib

import numpy as np
import pytest

from wc_runtime.map_quality import inspect_map_quality, validate_map_quality_policy


def pose_blob(x=0.):
    matrix=np.eye(4,dtype='<f4'); matrix[0,3]=x
    return matrix[:3].tobytes()


def compressed_scan(points):
    values=np.asarray(points,dtype='<f4')
    return zlib.compress(values.tobytes())+struct.pack('<iii',1,len(values),5+8*(values.shape[1]-1))


def native_database(tmp_path,positions=(0.,.6,1.2),*,version='0.23.7',links=True):
    path=tmp_path/'rtabmap.db'
    with closing(sqlite3.connect(path)) as db, db:
        db.execute('CREATE TABLE Admin(version TEXT)'); db.execute('INSERT INTO Admin VALUES(?)',(version,))
        db.execute('CREATE TABLE Node(id INTEGER PRIMARY KEY,stamp FLOAT,pose BLOB,weight INTEGER,map_id INTEGER)')
        db.execute('CREATE TABLE Data(id INTEGER PRIMARY KEY,scan BLOB,scan_info BLOB)')
        db.execute('CREATE TABLE Link(from_id INTEGER,to_id INTEGER,type INTEGER,transform BLOB)')
        scan=compressed_scan(np.tile([1.,2.,3.,4.],(125,1)))
        info=struct.pack('<7f',6.,0.,15.,0.,0.,0.,0.)+pose_blob()
        for identifier,x in enumerate(positions,1):
            db.execute('INSERT INTO Node VALUES(?,?,?,?,?)',(identifier,float(identifier),pose_blob(x),0,0))
            db.execute('INSERT INTO Data VALUES(?,?,?)',(identifier,scan,info))
            if links and identifier>1:
                db.execute('INSERT INTO Link VALUES(?,?,?,?)',(identifier-1,identifier,0,pose_blob(x-positions[identifier-2])))
    return path


def test_connected_native_moving_scan_map_is_only_candidate_and_unchanged(tmp_path):
    path=native_database(tmp_path); before=hashlib.sha256(path.read_bytes()).hexdigest()
    report=inspect_map_quality(path)
    assert report['file_integrity']['status']=='PASS'
    assert report['movement_map_qualification']['status']=='QUALIFIED_EXPERIMENTAL_CANDIDATE'
    assert report['graph']['component_count']==1 and report['graph']['unique_physical_edges']==2
    assert report['motion']['path_length_m']==pytest.approx(1.2)
    assert report['scan_summary']['pass']==3
    assert all(row['matrix']['cv_type']==29 and row['valid_nonzero_points']==125 for row in report['per_node_scans'])
    assert report['geometry_accuracy']['navigation_validated'] is False
    assert before==report['database_sha256_after']==hashlib.sha256(path.read_bytes()).hexdigest()
    assert report['database_modified'] is False


def test_static_map_valid_file_is_not_a_moving_map(tmp_path):
    report=inspect_map_quality(native_database(tmp_path,positions=(0.,)))
    assert report['file_integrity']['status']=='PASS'
    assert report['movement_map_qualification']['status']=='NOT_QUALIFIED'
    assert 'INSUFFICIENT_DATA_NODES' in report['movement_map_qualification']['reasons']
    assert report['movement_map_qualification']['static_map_save_allowed_if_file_and_export_integrity_pass'] is True


def test_pose_movement_without_connected_physical_links_fails(tmp_path):
    path=native_database(tmp_path,links=False)
    with closing(sqlite3.connect(path)) as db, db:
        # A virtual link must not turn disconnected scan nodes into a real map.
        db.execute('INSERT INTO Link VALUES(1,2,5,?)',(pose_blob(.6),))
    report=inspect_map_quality(path)
    assert report['graph']['component_count']==3
    assert report['graph']['ignored_virtual_prior_landmark_links']==1
    assert 'GRAPH_NOT_CONNECTED_OR_INVALID' in report['movement_map_qualification']['reasons']


@pytest.mark.parametrize('damage',['missing','nan','corrupt','budget'])
def test_each_node_scan_is_actually_decoded_and_checked(tmp_path,damage):
    path=native_database(tmp_path)
    with closing(sqlite3.connect(path)) as db, db:
        if damage=='missing': db.execute('DELETE FROM Data WHERE id=2')
        elif damage=='nan':
            values=np.tile([1.,2.,3.,4.],(125,1)); values[:25,:3]=np.nan
            db.execute('UPDATE Data SET scan=? WHERE id=2',(compressed_scan(values),))
        elif damage=='corrupt': db.execute("UPDATE Data SET scan=x'01020304050607080910111213' WHERE id=2")
        else:
            values=b'x'+struct.pack('<iii',1,100_000_000,29)
            db.execute('UPDATE Data SET scan=? WHERE id=2',(values,))
    report=inspect_map_quality(path)
    second=next(row for row in report['per_node_scans'] if row['node_id']==2)
    assert second['status']=='FAIL'
    assert report['file_integrity']['status']=='PASS'  # SQLite bytes can be intact while scan semantics fail.
    assert report['movement_map_qualification']['status']=='NOT_QUALIFIED'
    if damage=='nan': assert second['valid_point_fraction']==.8


def test_invalid_pose_does_not_hide_the_nodes_scan_audit(tmp_path):
    path=native_database(tmp_path)
    with closing(sqlite3.connect(path)) as db, db: db.execute("UPDATE Node SET pose=x'0000' WHERE id=2")
    report=inspect_map_quality(path)
    assert len(report['per_node_scans'])==3 and report['nodes']['valid_poses']==2
    assert report['nodes']['invalid'][0]['node_id']==2
    assert report['movement_map_qualification']['status']=='NOT_QUALIFIED'


@pytest.mark.parametrize('old',['version','scan_info_column'])
def test_old_or_missing_schema_never_claims_scan_gate_pass(tmp_path,old):
    path=native_database(tmp_path,version='0.20.0' if old=='version' else '0.23.7')
    if old=='scan_info_column':
        with closing(sqlite3.connect(path)) as db, db: db.execute('ALTER TABLE Data DROP COLUMN scan_info')
    report=inspect_map_quality(path)
    assert report['file_integrity']['status']=='PASS'
    assert report['movement_map_qualification']['status']=='UNKNOWN'
    assert report['per_node_scans']==[]


def test_explicit_policy_predeclares_limits_and_rejects_invalid(tmp_path):
    path=native_database(tmp_path)
    report=inspect_map_quality(path,policy={'min_path_length_m':2.})
    assert report['policy_source']=='EXPLICIT_CALLER_POLICY'
    assert 'INSUFFICIENT_STORED_PATH_LENGTH' in report['movement_map_qualification']['reasons']
    for value in ({'min_nodes':1},{'min_valid_scan_fraction':1.1},{'min_path_length_m':float('nan')},{'schema_version':True},{'bad':1}):
        with pytest.raises(ValueError): validate_map_quality_policy(value)


@pytest.mark.parametrize('outcome', ['complete', 'unsupported_schema', 'query_failure'])
def test_inspection_explicitly_closes_sqlite_on_return_and_failure(tmp_path, monkeypatch, outcome):
    path = native_database(tmp_path, version='0.20.0' if outcome == 'unsupported_schema' else '0.23.7')
    original_connect = sqlite3.connect
    opened, closed = [], []

    class TrackedConnection(sqlite3.Connection):
        def execute(self, sql, *args, **kwargs):
            if outcome == 'query_failure' and sql == 'PRAGMA query_only=ON':
                raise sqlite3.OperationalError('injected read-only inspection failure')
            return super().execute(sql, *args, **kwargs)

        def close(self):
            closed.append(self)
            return super().close()

    def connect(*args, **kwargs):
        connection = original_connect(*args, factory=TrackedConnection, **kwargs)
        opened.append(connection)
        return connection

    monkeypatch.setattr(sqlite3, 'connect', connect)
    report = inspect_map_quality(path)
    assert len(opened) == 1 and closed == opened
    with pytest.raises(sqlite3.ProgrammingError, match='closed'):
        opened[0].execute('SELECT 1')
    if outcome == 'query_failure':
        assert report['file_integrity']['status'] == 'FAIL'
        assert 'injected read-only inspection failure' in report['inspection_error']
    else:
        assert report['file_integrity']['status'] == 'PASS'
