"""Synthetic diagnosis contracts; no ROS imports, sockets, or hardware."""
import json
from pathlib import Path
import shutil
import uuid
import pytest
from wc_runtime import runtime_health as health


@pytest.fixture
def tmp_path():
    parent = Path(__file__).absolute().parent
    directory = parent/('.runtime_health_'+uuid.uuid4().hex)
    directory.mkdir()
    try: yield directory
    finally:
        assert directory.resolve().parent == parent and directory.name.startswith('.runtime_health_')
        shutil.rmtree(directory)


def document(root, name, value):
    path = root/name; path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False), encoding='utf-8')


def session(root):
    document(root, 'session.json', {'session_id':'SYNTHETIC', 'status':'RUNNING'})
    document(root, 'runtime_config.json', {'session_id':'SYNTHETIC', 'mapping_enabled':True})


def codes(report): return {item['code'] for item in report['issues']}


def test_running_and_map_points_never_establish_quality_or_calibration(tmp_path):
    session(tmp_path)
    document(tmp_path, 'health/status.json', {'status':'RUNNING','map_points':500,'counts':{'cloud_map':3}})
    result = health.collect_session_health(tmp_path)
    assert result['lifecycle'] == 'RUNNING' and result['map_quality']['output_state'] == 'OBSERVED'
    assert result['map_quality']['status'] == 'UNVALIDATED' and not result['map_quality']['formal_acceptance']
    assert not result['map_quality']['navigation_validated'] and not result['hardware_actions_performed']


def test_geometry_blocks_only_dependent_capabilities(tmp_path):
    session(tmp_path)
    assessment = {'capabilities':{'native_capture':{'status':'AVAILABLE','validation_level':'CONFIG_ONLY'},
        'wheel_imu_motion_all':{'status':'BLOCKED','reasons':['外参未确认']}},
        'unresolved_components':['imu'], 'source_documents':['hardware_setup.json']}
    result = health.collect_session_health(tmp_path, assessment)
    assert result['capabilities']['native_capture']['status'] == 'AVAILABLE'
    issue = next(i for i in result['issues'] if i['code']=='CALIBRATION_REQUIRED')
    assert issue['capability_impact'] == ['wheel_imu_motion_all']
    assert result['parameter_provenance']['validation_level'] == 'CONFIG_ONLY'
    assert result['parameter_provenance']['unresolved_components'] == ['imu']


def test_missing_camera_is_not_wheel_estimator_failure(tmp_path):
    session(tmp_path)
    document(tmp_path,'wheel_status.json',{'state':'READY'})
    document(tmp_path,'cameras/status.json',{'slots':{'left_front':{'state':'UNAVAILABLE',
        'error':'FileNotFoundError: expected by-path disappeared'}}})
    result=health.collect_session_health(tmp_path)
    assert 'CAMERA_NOT_ENUMERATED' in codes(result)
    assert result['components']['wheel']['lifecycle']=='READY'
    item=next(i for i in result['issues'] if i['code']=='CAMERA_NOT_ENUMERATED')
    assert all(cause['status']=='HYPOTHESIS' for cause in item['possible_causes'])
    assert 'motion_fusion' not in item['capability_impact'] and item['retest']


def test_full_archive_is_distinct_from_preview_and_device_exposure_completeness(tmp_path):
    session(tmp_path)
    document(tmp_path,'dataset/capture_manifest.json',{'profile':'all_sensors','recording_complete':True,
        'source_accounting':{'camera_right_front':{'successfully_received':170,'persisted':170,'retained':170}}})
    result=health.collect_session_health(tmp_path)
    assert result['recording']['camera_archive_observed']
    assert result['recording']['sources']['camera_right_front']['observed']==170
    assert result['capabilities']['all_sensors_recording']['status']=='COMPLETE'
    assert not result['recording']['preview_rate_is_archive_rate']


def test_count_gap_overrules_declared_complete(tmp_path):
    session(tmp_path)
    document(tmp_path,'dataset/capture_manifest.json',{'profile':'all_sensors','recording_complete':True,
        'source_accounting':{'camera_left_front':{'successfully_received':170,'persisted':160}}})
    result=health.collect_session_health(tmp_path)
    assert not result['recording']['recording_complete']
    assert result['capabilities']['all_sensors_recording']['status']=='INCOMPLETE'
    assert {'RECORDING_GAP','RECORDING_COMPLETENESS_CONTRADICTION'} <= codes(result)


def test_ultrasonic_timeout_keeps_specific_observation(tmp_path):
    session(tmp_path)
    document(tmp_path,'dataset/capture_manifest.json',{'profile':'all_sensors','recording_complete':False,
        'issues':[{'code':'ULTRASONIC_RESPONSE_TIMEOUT','evidence':'sources/ultrasonic/events.jsonl','detail':'address=1; no reply'}]})
    result=health.collect_session_health(tmp_path)
    assert 'ULTRASONIC_RESPONSE_TIMEOUT' in codes(result)
    assert 'CAMERA_NOT_ENUMERATED' not in codes(result)


def test_old_session_reports_are_rejected(tmp_path):
    session(tmp_path)
    document(tmp_path,'health/status.json',{'session_id':'OTHER','status':'RUNNING','map_points':999})
    result=health.collect_session_health(tmp_path)
    assert 'SESSION_ID_MISMATCH' in codes(result)
    assert result['map_quality']['output_state']=='UNKNOWN'


def test_malformed_nested_schema_is_a_reported_error(tmp_path):
    session(tmp_path)
    document(tmp_path,'health/status.json',{'age_s':[]})
    document(tmp_path,'cameras/status.json',{'slots':[]})
    document(tmp_path,'dataset/capture_manifest.json',{'source_accounting':'wrong','issues':True})
    result=health.collect_session_health(tmp_path)
    assert 'REPORT_SCHEMA_INVALID' in codes(result)


def test_readonly_cli_does_not_create_files_or_import_ros(tmp_path,capsys):
    session(tmp_path)
    before={p.relative_to(tmp_path):p.read_bytes() for p in tmp_path.rglob('*') if p.is_file()}
    assert health.main(['--session-root',str(tmp_path),'--check'])==0
    result=json.loads(capsys.readouterr().out)
    assert not result['hardware_actions_performed']
    after={p.relative_to(tmp_path):p.read_bytes() for p in tmp_path.rglob('*') if p.is_file()}
    assert after==before


def test_report_has_chinese_evidence_retest_and_independent_retention(tmp_path):
    session(tmp_path)
    document(tmp_path,'retention.json',{'raw_retention':'DISCARDED'})
    report=health.write_health_report(tmp_path)
    assert report['recording']['retention']=='DISCARDED'
    assert '地图质量' in (tmp_path/'diagnosis_zh.md').read_text(encoding='utf-8')
    assert json.loads((tmp_path/'health.json').read_text(encoding='utf-8'))['schema']==health.SCHEMA


def test_capture_root_manifest_is_a_valid_identity_without_mapping_session_file(tmp_path):
    document(tmp_path,'capture_manifest.json',{'session_id':'SYNTHETIC_CAPTURE','status':'COMPLETE',
        'recording_complete':True,'profile':'all_sensors','raw_retention':'RETAINED',
        'source_accounting':{'camera_right_front':{'successfully_received':60,'persisted':60,'retained':60}}})
    document(tmp_path,'configuration/runtime_config.json',{'retention_profile':'experiment'})
    result=health.collect_session_health(tmp_path)
    assert result['session_id']=='SYNTHETIC_CAPTURE' and result['lifecycle']=='COMPLETE'
    assert 'SESSION_REPORT_MISSING' not in codes(result)
    assert result['recording']['capture_manifest_ref']=='capture_manifest.json'
    assert result['recording']['retention']=='RETAINED'


def test_prospective_final_manifest_updates_health_without_committing_source(tmp_path):
    pending={'session_id':'FINALIZING','status':'PARTIAL','recording_complete':False}
    document(tmp_path,'capture_manifest.json',pending)
    complete={**pending,'status':'COMPLETE','recording_complete':True}
    result=health.write_health_report(tmp_path,capture_manifest=complete)
    assert result['lifecycle']=='COMPLETE'
    assert result['recording']['recording_complete']
    assert json.loads((tmp_path/'capture_manifest.json').read_text())==pending


@pytest.mark.parametrize('existing', [None, {'session_id':'OTHER'}])
def test_prospective_manifest_requires_existing_same_session(tmp_path,existing):
    if existing is not None: document(tmp_path,'capture_manifest.json',existing)
    with pytest.raises(ValueError,match='existing root session'):
        health.collect_session_health(tmp_path,capture_manifest={'session_id':'NEW'})


def test_camera_preflight_missing_path_and_identity_mismatch_are_different(tmp_path):
    document(tmp_path,'capture_manifest.json',{'session_id':'SYNTHETIC_CAPTURE','preflight':{
        'camera_left_front':{'status':'BLOCKED','reason':'[Errno 2] No such file or directory: /dev/v4l/by-path/expected'},
        'camera_left_side':{'status':'BLOCKED','reason':'USB port identity or capture capability mismatch'}}})
    result=health.collect_session_health(tmp_path)
    left_front=next(i for i in result['issues'] if i['component']=='camera/left_front')
    left_side=next(i for i in result['issues'] if i['component']=='camera/left_side')
    assert left_front['code']=='CAMERA_NOT_ENUMERATED'
    assert left_side['code']=='CAPTURE_PREFLIGHT_BLOCKED'


def test_lifecycle_uses_monitor_not_static_experiment_profile(tmp_path):
    document(tmp_path,'session.json',{'session_id':'SYNTHETIC','status':'EXPERIMENT'})
    document(tmp_path,'health/status.json',{'status':'STOPPED'})
    result=health.collect_session_health(tmp_path)
    assert result['lifecycle']=='STOPPED' and result['session_profile_status']=='EXPERIMENT'
