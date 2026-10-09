"""SYNTHETIC real filesystem retention/atomic-save tests, no devices or SLAM."""
import copy
import hashlib
import json
import os
from pathlib import Path
import shutil
import uuid

import pytest

from wc_runtime import mapping_save as saving


@pytest.fixture
def tmp_path():
    parent = Path(__file__).absolute().parent
    directory = parent/('.mapping_save_files_'+uuid.uuid4().hex)
    directory.mkdir()
    try: yield directory
    finally:
        assert directory.resolve().parent == parent and directory.name.startswith('.mapping_save_files_')
        shutil.rmtree(directory)


@pytest.fixture
def session(tmp_path):
    root = tmp_path/'project'; directory = root/'reports/maps/SYNTHETIC'
    runtime = root/'.phase1_runtime/sessions/SYNTHETIC/mapping_app'
    directory.mkdir(parents=True); runtime.mkdir(parents=True)
    handle = {'session_id':'SYNTHETIC','directory':str(directory),'runtime':str(runtime),
              'mode':'right','mapping_enabled':True, 'retention_profile':'map_only'}
    (directory/'session.json').write_text(json.dumps({'session_id':'SYNTHETIC','output_dir':str(directory),
        'mapping_enabled':True,'data_retention':'TEMPORARY_UNTIL_USER_SAVE_CHOICE'}))
    for name in ('bag','input','slam','health','prior'): (directory/name).mkdir()
    (directory/'bag/recording.db3').write_bytes(b'SYNTHETIC raw bag')
    (directory/'input/frame.cdr').write_bytes(b'SYNTHETIC raw frame')
    (directory/'input/status.json').write_text('{"state":"STOPPED"}')
    (directory/'slam/rtabmap.db').write_bytes(b'SYNTHETIC already audited native database')
    (directory/'health/latest_cloud.npz').write_bytes(b'SYNTHETIC disposable snapshot')
    (directory/'health/status.json').write_text('{"failure":null}')
    (directory/'runtime_config.json').write_text('{"mapping_enabled":true}')
    (directory/'wheel_feedback.jsonl').write_text('SYNTHETIC temporary telemetry\n')
    (directory/'rviz.log').write_text('SYNTHETIC keep diagnostics\n')
    old = root/'reports/maps/OLD'; old.mkdir()
    (old/'keep.db').write_bytes(b'KEEP HISTORICAL MAP')
    return root, directory, runtime, handle


def closed(_): return {'state':'STOPPED','exit_code':0}


def test_default_experiment_retains_original_and_hash_reference(session, tmp_path):
    root, directory, runtime, handle = session
    handle.pop('retention_profile')
    destination = tmp_path/'retained_experiment'
    result = saving.save_session(root, handle, destination, inspect=closed, export=fake_export)
    assert result['retention_profile'] == 'experiment' and result['raw_retention'] == 'RETAINED_REFERENCED'
    reference = json.loads((destination/'raw_archive.reference.json').read_text(encoding='utf-8'))
    assert reference['source_directory'] == str(directory) and not reference['independent_copy']
    for relative, evidence in reference['files'].items():
        assert saving._file_digest(directory/relative) == evidence['sha256']
    assert (directory/'bag/recording.db3').is_file() and (directory/'input/frame.cdr').is_file()
    assert not (destination/'bag').exists()


def test_declining_map_save_does_not_delete_experiment_sources(session):
    root, directory, runtime, handle = session
    handle.pop('retention_profile')
    result = saving.discard_session(root, handle, inspect=closed)
    assert result['status'] == 'SESSION_DATA_RETAINED' and result['removed'] == []
    assert (directory/'bag/recording.db3').is_file() and (directory/'slam/rtabmap.db').is_file()


def test_explicit_user_no_overrides_experiment_retention_only_for_current_session(session):
    root, directory, runtime, handle = session
    handle.pop('retention_profile')
    result = saving.discard_session(root, handle, inspect=closed, user_confirmed=True)
    assert result['status']=='SESSION_DATA_DISCARDED'
    assert result['user_confirmed_discard'] and not result['replayable']
    assert result['original_retention_profile']=='experiment'
    assert not (directory/'bag').exists() and not (directory/'slam').exists()
    assert (directory/'session.json').is_file()
    assert (root/'reports/maps/OLD/keep.db').read_bytes()==b'KEEP HISTORICAL MAP'
    assert result['disposition_reason']=='USER_DECLINED_SAVE' and not result['automatic_preview_cleanup']
    assert json.loads(next(runtime.glob('discard-*.json')).read_text(encoding='utf-8'))==result


@pytest.mark.skipif(os.name=='nt',reason='real stopped-session flock requires POSIX')
def test_preview_auto_cleanup_receipts_do_not_claim_user_declined_save(session):
    root,directory,runtime,handle=session
    handle['mapping_enabled']=False
    handle.pop('retention_profile')
    metadata=json.loads((directory/'session.json').read_text())
    metadata['mapping_enabled']=False
    (directory/'session.json').write_text(json.dumps(metadata))
    (directory/'runtime_config.json').write_text('{"mapping_enabled":false}')
    independent=root/'data/experiments/independent';independent.mkdir(parents=True)
    (independent/'recording.db3').write_bytes(b'KEEP INDEPENDENT RECORDING')
    result=saving.discard_session(root,handle,inspect=closed,preview_complete=True)
    assert result['status']=='SESSION_DATA_DISCARDED'
    assert result['disposition_reason']=='LIVE_PREVIEW_COMPLETE'
    assert result['user_confirmed_discard'] is False and result['automatic_preview_cleanup'] is True
    assert result['original_retention_profile']=='experiment'
    assert json.loads((directory/'retention.json').read_text(encoding='utf-8'))==result
    assert json.loads(next(runtime.glob('discard-*.json')).read_text(encoding='utf-8'))==result
    assert not (directory/'bag').exists() and not (directory/'slam').exists()
    assert (independent/'recording.db3').read_bytes()==b'KEEP INDEPENDENT RECORDING'
    assert (root/'reports/maps/OLD/keep.db').read_bytes()==b'KEEP HISTORICAL MAP'


@pytest.mark.skipif(os.name=='nt',reason='real stopped-session flock requires POSIX')
@pytest.mark.parametrize('spoof',['flag_only','handle_only','session_and_handle_only'])
def test_preview_cleanup_cannot_disguise_mapping_session(session,spoof):
    root,directory,runtime,handle=session
    if spoof!='flag_only':handle['mapping_enabled']=False
    if spoof=='session_and_handle_only':
        metadata=json.loads((directory/'session.json').read_text())
        metadata['mapping_enabled']=False
        (directory/'session.json').write_text(json.dumps(metadata))
    with pytest.raises(ValueError):
        saving.discard_session(root,handle,inspect=closed,preview_complete=True)
    assert (directory/'bag/recording.db3').is_file() and (directory/'slam/rtabmap.db').is_file()
    assert not (directory/'retention.json').exists() and not list(runtime.glob('discard-*.json'))


@pytest.mark.skipif(os.name=='nt',reason='controller supervisor imports POSIX fcntl')
@pytest.mark.parametrize('enabled',[False,True])
def test_controller_discard_dispatch_distinguishes_preview_and_user_choice(monkeypatch,enabled):
    from wc_runtime import mapping_controller as controller
    received=[]
    def discard(root,handle,**options):received.append(options);return {'status':'fixture'}
    monkeypatch.setattr(saving,'discard_session',discard)
    assert controller.discard({'mapping_enabled':enabled})=={'status':'fixture'}
    assert received[0]['user_confirmed'] is enabled
    assert received[0]['preview_complete'] is (not enabled)


def test_preview_cleanup_rejects_contradictory_confirmation_before_touching_files(tmp_path):
    with pytest.raises(ValueError,match='明确区分'):
        saving.discard_session(tmp_path,{},inspect=lambda _:None,user_confirmed=True,preview_complete=True)


def test_explicit_discard_cannot_break_archive_referenced_by_saved_map(session,tmp_path):
    root,directory,runtime,handle=session
    handle.pop('retention_profile')
    saving.save_session(root,handle,tmp_path/'saved_map',inspect=closed,export=fake_export)
    with pytest.raises(ValueError,match='已保存地图引用'):
        saving.discard_session(root,handle,inspect=closed,user_confirmed=True)
    assert (directory/'bag/recording.db3').is_file()


def test_saved_reference_survives_source_retention_metadata_failure(session,tmp_path,monkeypatch):
    root,directory,runtime,handle=session
    handle.pop('retention_profile')
    replace=saving._replace_durable
    def fail_retention(path,value):
        if path==directory/'retention.json':raise OSError('SYNTHETIC source metadata sync failure')
        return replace(path,value)
    monkeypatch.setattr(saving,'_replace_durable',fail_retention)
    saved=saving.save_session(root,handle,tmp_path/'committed',inspect=closed,export=fake_export)
    assert saved['status']=='SAVED_EXPERIMENTAL_MAP' and saved['cleanup_errors']
    assert (tmp_path/'committed/raw_archive.reference.json').is_file()
    assert not (directory/'retention.json').exists()
    with pytest.raises(ValueError,match='保存提交意图'):
        saving.discard_session(root,handle,inspect=closed,user_confirmed=True)
    assert (directory/'bag/recording.db3').is_file()


@pytest.mark.parametrize('profile', ['experiment', 'map_only'])
def test_committed_map_remains_saved_when_runtime_journal_write_fails(session, tmp_path, monkeypatch, profile):
    root, directory, runtime, handle = session
    handle['retention_profile'] = profile
    destination = tmp_path/'committed_map'
    database_hash = saving._file_digest(directory/'slam/rtabmap.db')
    write = saving.write_new

    def fail_final_journal(path, data):
        if path.parent == runtime and path.name.startswith('save-'):
            raise OSError('SYNTHETIC runtime journal storage full')
        return write(path, data)

    monkeypatch.setattr(saving, 'write_new', fail_final_journal)
    saved = saving.save_session(root, handle, destination, inspect=closed, export=fake_export)
    assert saved['status'] == 'SAVED_EXPERIMENTAL_MAP'
    assert saved['cleanup_errors'] == ['SAVE_JOURNAL_WRITE_FAILED: SYNTHETIC runtime journal storage full']
    assert saving._file_digest(destination/'slam/rtabmap.db') == database_hash
    assert json.loads((destination/'saved_map.json').read_text())['status'] == 'SAVED_EXPERIMENTAL_MAP'
    assert json.loads((directory/'retention.json').read_text())['saved_map'] == str(destination)
    assert all(json.loads(p.read_text())['status'] == 'COMMITTED'
               for p in (directory/'save_commit_intents').iterdir())
    assert saved['raw_retention'] == ('RETAINED_REFERENCED' if profile == 'experiment' else 'DISCARDED')
    assert (directory/'bag/recording.db3').exists() is (profile == 'experiment')
    assert not list(runtime.glob('save-*.json'))


def test_discard_retains_existing_map_quality_report(session):
    root,directory,runtime,handle=session
    (directory/'export').mkdir()
    quality={'movement_map_qualification':{'status':'NOT_QUALIFIED'}}
    (directory/'export/map_quality.json').write_text(json.dumps(quality))
    result=saving.discard_session(root,handle,inspect=closed,user_confirmed=True)
    assert result['map_quality_evidence']=='retained_metadata/export/map_quality.json'
    assert json.loads((directory/result['map_quality_evidence']).read_text())==quality


def fake_export(handle):
    output = Path(handle['directory'])/'export'; output.mkdir()
    (output/'map_cloud.ply').write_bytes(b'SYNTHETIC already audited exported cloud')
    (output/'map.yaml').write_text('image: map.pgm\n')
    (output/'map.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
    return {'status':'EXPORTED_EXPERIMENTAL_MAP','output':str(output)}


def test_save_to_external_unicode_path_is_durable_and_discards_only_session_data(session, tmp_path):
    root, directory, runtime, handle = session
    destination = tmp_path/'用户地图/客厅 新图'
    result = saving.save_session(root,handle,destination,inspect=closed,export=fake_export)
    assert result['status'] == 'SAVED_EXPERIMENTAL_MAP' and result['cleanup_errors'] == []
    saved = json.loads((destination/'saved_map.json').read_text())
    assert saved['raw_recordings_saved'] is False
    for relative, evidence in saved['files'].items():
        assert saving._file_digest(destination/relative) == evidence['sha256']
    assert (destination/'slam/rtabmap.db').read_bytes() == b'SYNTHETIC already audited native database'
    assert not any((directory/name).exists() for name in saving.TEMPORARY_DATA)
    assert (directory/'rviz.log').is_file()
    assert (directory/'retained_metadata/input/status.json').is_file()
    assert (root/'reports/maps/OLD/keep.db').read_bytes() == b'KEEP HISTORICAL MAP'


def test_saved_map_contains_exact_confirmed_bias_and_bound_evidence_after_raw_cleanup(session, tmp_path):
    root, directory, runtime, handle = session
    evidence = {'schema_version': 1, 'source_session': str(directory),
                'physical_stationarity_confirmed_by_operator': True,
                'automatic_stationarity_proven': False,
                'candidate': {'start_stamp_ns': 1_000_000_000, 'end_stamp_ns': 6_000_000_000,
                              'sample_count': 1000, 'bias_native_rad_s': [.0001, -.0002, .0003]},
                'evidence_note': 'SYNTHETIC 静止区间；仅文件保留测试，不是真实标定。'}
    evidence_bytes = (json.dumps(evidence, ensure_ascii=False, indent=2)+'\n').encode('utf-8')
    bias = {'status': 'INDEPENDENTLY_CONFIRMED', 'sensor_id': 'H30-0000000015',
            'bias_native_rad_s': [.0001, -.0002, .0003],
            'source': 'operator_visual_confirmation', 'evidence_id': 'SYNTHETIC:1:6',
            'evidence_sha256': hashlib.sha256(evidence_bytes).hexdigest()}
    bias_bytes = (json.dumps(bias, indent=2)+'\n').encode('utf-8')
    expected = {'confirmed_gyro_bias.json': bias_bytes,
                'confirmed_gyro_bias.evidence.json': evidence_bytes}
    for name, data in expected.items():
        (directory/name).write_bytes(data)
    destination = tmp_path/'用户地图/已确认零偏'
    result = saving.save_session(root, handle, destination, inspect=closed, export=fake_export)
    saved = json.loads((destination/'saved_map.json').read_text(encoding='utf-8'))
    assert result['status'] == 'SAVED_EXPERIMENTAL_MAP' and result['cleanup_errors'] == []
    for name, data in expected.items():
        assert (destination/name).read_bytes() == data
        assert saved['files'][name] == {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
        assert (directory/name).read_bytes() == data  # Retained session metadata is not discarded.
    saved_bias = json.loads((destination/'confirmed_gyro_bias.json').read_text(encoding='utf-8'))
    assert saved_bias['evidence_sha256'] == saving._file_digest(destination/'confirmed_gyro_bias.evidence.json')
    assert not (directory/'bag').exists() and not (directory/'input').exists()
    assert (root/'reports/maps/OLD/keep.db').read_bytes() == b'KEEP HISTORICAL MAP'


def test_discard_never_calls_export_and_keeps_small_statuses(session):
    root, directory, runtime, handle = session
    result = saving.discard_session(root,handle,inspect=closed)
    assert result['status'] == 'SESSION_DATA_DISCARDED'
    assert not (directory/'bag').exists() and not (directory/'slam').exists()
    assert (directory/'health/status.json').is_file() and (directory/'session.json').is_file()


def test_final_user_discard_removes_failed_export_copies_but_retains_their_logs(session):
    root,directory,runtime,handle=session
    name='export_failed_'+'a'*32
    failure=directory/name;failure.mkdir()
    (failure/'native_export_copy.db').write_bytes(b'SYNTHETIC failed export database')
    (failure/'export.log').write_text('SYNTHETIC native export failure')
    unrelated=directory/'export_failed_notes';unrelated.mkdir();(unrelated/'keep').write_bytes(b'KEEP')
    saving.discard_session(root,handle,inspect=closed)
    assert not failure.exists()
    assert (directory/'retained_metadata'/name/'export.log').read_text()=='SYNTHETIC native export failure'
    assert (unrelated/'keep').read_bytes()==b'KEEP'


def test_parent_directory_sync_failure_preserves_raw_and_never_commits(session,tmp_path,monkeypatch):
    root,directory,runtime,handle=session
    destination=tmp_path/'new/parent/saved'
    actual=saving._sync_directory
    def fail(path):
        if path == tmp_path/'new': raise OSError('SYNTHETIC ancestor sync failed')
        actual(path)
    monkeypatch.setattr(saving,'_sync_directory',fail)
    with pytest.raises(OSError,match='ancestor sync failed'):
        saving.save_session(root,handle,destination,inspect=closed,export=fake_export)
    assert not destination.exists()
    assert (directory/'bag/recording.db3').is_file() and (directory/'slam/rtabmap.db').is_file()


def test_discard_rejects_raw_symlink_before_deleting_any_heavy_data(session,tmp_path):
    root,directory,runtime,handle=session
    outside=tmp_path/'outside';outside.mkdir();(outside/'keep').write_bytes(b'KEEP')
    (directory/'input/redirect').symlink_to(outside,target_is_directory=True)
    with pytest.raises(ValueError,match='符号链接'):
        saving.discard_session(root,handle,inspect=closed)
    assert (outside/'keep').read_bytes()==b'KEEP'
    assert (directory/'bag/recording.db3').is_file() and (directory/'slam/rtabmap.db').is_file()


@pytest.mark.parametrize('reason', ['running','failed','legacy','identity','runtime'])
def test_discard_refuses_unclosed_or_unowned_sessions(session, reason):
    root, directory, runtime, handle = session
    inspect = closed
    if reason in ('running','failed'): inspect = lambda _: {'state':reason.upper(),'exit_code':1}
    else:
        config = json.loads((directory/'session.json').read_text())
        if reason == 'legacy': config.pop('data_retention')
        elif reason == 'identity': config['session_id'] = 'OTHER'
        elif reason == 'runtime': handle['runtime'] = str(root/'.phase1_runtime/sessions/OTHER/mapping_app')
        (directory/'session.json').write_text(json.dumps(config))
    with pytest.raises((ValueError,RuntimeError,OSError)):
        saving.discard_session(root,handle,inspect=inspect)
    assert (directory/'bag/recording.db3').is_file()


def test_existing_destination_is_untouched_and_no_export_occurs(session,tmp_path):
    root,directory,runtime,handle=session
    destination=tmp_path/'existing'; destination.mkdir(); (destination/'keep').write_bytes(b'KEEP')
    with pytest.raises(ValueError,match='已存在'):
        saving.save_session(root,handle,destination,inspect=closed,export=lambda _:pytest.fail('must not export'))
    assert (destination/'keep').read_bytes()==b'KEEP' and (directory/'bag').is_dir()


def test_failed_copy_keeps_raw_data_and_removes_only_its_stage(session,tmp_path,monkeypatch):
    root,directory,runtime,handle=session; destination=tmp_path/'saved'
    def fail(*_): raise OSError('SYNTHETIC destination write failed')
    monkeypatch.setattr(saving,'_copy_durable',fail)
    with pytest.raises(OSError,match='destination write failed'):
        saving.save_session(root,handle,destination,inspect=closed,export=fake_export)
    assert not destination.exists() and not list(tmp_path.glob('.saved.saving-*'))
    assert (directory/'bag/recording.db3').is_file() and (directory/'slam/rtabmap.db').is_file()


def test_atomic_commit_refuses_destination_created_during_copy(session,tmp_path,monkeypatch):
    root,directory,runtime,handle=session; destination=tmp_path/'saved'
    actual=saving._rename_no_replace
    def race(source,target):
        destination.mkdir()
        actual(source,target)
    monkeypatch.setattr(saving,'_rename_no_replace',race)
    with pytest.raises(Exception,match='already exists'):
        saving.save_session(root,handle,destination,inspect=closed,export=fake_export)
    assert destination.is_dir() and not list(destination.iterdir())
    assert (directory/'bag').is_dir() and not list(tmp_path.glob('.saved.saving-*'))
