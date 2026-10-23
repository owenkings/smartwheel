"""Real-time experiment contracts with synthetic sources; no hardware access."""
import copy
import hashlib
import json
from pathlib import Path
import shutil
import sys
import uuid

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'runtime'))
from test_offline_motion_adapter import candidate
from test_mapping_prior import config as prior_config, authority_weights, wheel, imu
from wc_runtime import offline_motion_calibration
from wc_runtime.hardware_setup import resolve_hardware_setup
from wc_runtime.live_experiment import (prepare_experiment, experiment_capability,
    canonical_sha256, validate_motion_parameters, STATUS)
from wc_runtime.live_motion_options import validate_live_options, calibration_report
from wc_runtime.mapping_input import validate_config as validate_input
from wc_runtime.mapping_prior import MotionPrior, PriorError
from wc_runtime.offline_motion_adapter import OfflineCorrectedPrior
from wc_runtime.single_mapping_input import InputFailure


ROOT = Path(__file__).resolve().parents[2]


def raw_json(value):
    return (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode('utf-8')


def feed(prior, *, bias=(0., 0., 0.), extra_queries=False):
    for index in range(101):
        stamp = 1_000_000_000+index*10_000_000
        row = wheel(index, stamp, -100, 110)
        row['device_id'] = prior.config['wheel_device_id']
        prior.add_wheel(row)
        imu(prior, index, stamp, gyro=np.asarray([0., 0., .025])+bias,
            sensor_id=prior.config['imu_sensor_id'], session_id=prior.session_id)
        if extra_queries and index > 2:
            prior.pose_at(stamp-5_000_000)
    return stamp


@pytest.fixture
def evidence():
    parent = Path(__file__).absolute().parent
    root = parent/('.live_experiment_'+uuid.uuid4().hex)
    root.mkdir()
    try:
        setup = json.loads((ROOT/'config/hardware_setup.json').read_text(encoding='utf-8'))
        for source in setup['source_documents']:
            destination = root/source['snapshot_path']
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT/source['snapshot_path'], destination)
        (root/'config/calibration').mkdir(parents=True, exist_ok=True)
        for name in ('mapping_live.json', 'live_unvalidated.json', 'wheel_feedback_current.json', 'device_bindings.json'):
            shutil.copyfile(ROOT/'config'/name, root/'config'/name)
        mapping_path = root/'config/mapping_live.json'
        mapping = json.loads(mapping_path.read_text(encoding='utf-8'))
        mapping['cameras_enabled'] = False
        mapping_path.write_bytes(raw_json(mapping))
        (root/'config/hardware_setup.json').write_bytes(raw_json(setup))
        bindings = json.loads((root/'config/device_bindings.json').read_text(encoding='utf-8'))
        wheel_hardware = json.loads((root/'config/wheel_feedback_current.json').read_text(encoding='utf-8'))
        identities = dict(imu_sensor_id=bindings['imu']['sensor_id'], wheel_device_id=wheel_hardware['device_id'])
        c = prior_config(); c.update(identities)
        original = candidate(c, bias=(.0003, -.0002, .0008), a=.906, b=-.0148)
        original['formal_calibration'] = False
        original['automatic_apply'] = False
        original['scope'] = 'same-session offline consistency; synthetic test artifact'
        original['provenance']['wheel_conversion'] = resolve_hardware_setup(setup)['wheel_candidate']
        # A future-dependent old anchor is deliberately present in the source.
        # None of it is allowed into the new session's coefficient snapshot.
        original['provenance']['time_mapping'] = {'imu_time_model': {
            'scope': 'OFFLINE_ONLY', 'anchor_ros_ns': 9000000000000000000,
            'future_data_used_for_offline_anchor': True}}
        original['candidate_id'] = offline_motion_calibration._candidate_digest(original)
        source = root/'data/analysis/synthetic/motion_candidate.json'
        source.parent.mkdir(parents=True)
        source.write_bytes(raw_json(original))
        profile = {'schema_version': 1, 'status': STATUS, 'scope': 'EXPLICIT_REALTIME_EXPERIMENT',
            'assembly_revision': setup['assembly']['revision'],
            'hardware_setup_content_sha256': canonical_sha256(setup), **identities,
            'motion_candidate': {'source_path': 'data/analysis/synthetic/motion_candidate.json',
                'sha256': hashlib.sha256(source.read_bytes()).hexdigest()},
            'limitations': ['Synthetic mechanical face-center/relative motion reuse; not calibrated.']}
        (root/'config/calibration/live_experiment_profile.json').write_bytes(raw_json(profile))
        yield root, setup, identities, original, profile
    finally:
        resolved = root.resolve()
        assert resolved.parent == parent and resolved.name.startswith('.live_experiment_')
        shutil.rmtree(resolved)


def prepare(evidence):
    root, setup, ids, _, _ = evidence
    return prepare_experiment(root, setup, resolve_hardware_setup(setup), **ids)


def test_preparation_is_readonly_preserves_unknown_and_source_bytes_and_has_no_old_anchor(evidence):
    root, setup, ids, original, profile = evidence
    before = copy.deepcopy(setup)
    source = root/profile['motion_candidate']['source_path']; source_bytes = source.read_bytes()
    result = prepare(evidence)
    assert setup == before and source.read_bytes() == source_bytes
    assert json.loads(result['candidate_raw_text']) == original
    assert result['motion_parameters']['source_time_policy'] == 'HOST_ARRIVAL_ONLY_NO_OFFLINE_ANCHOR'
    assert not {'offline_imu_time_model', 'time_mapping', 'windows', 'anchor_ros_ns'} & result['motion_parameters'].keys()
    geometry = result['resolved_geometry']
    assert geometry['offline_experiment'] is False and geometry['mount_model'] == 'live_experimental_mechanical_initial'
    assert geometry['geometry_report']['original_report']['unresolved_components']
    assert geometry['geometry_report']['original_report']['capabilities']['wheel_imu_motion_all']['status'] == 'BLOCKED'
    assert geometry['live_experiment_initialization']['remaining_datum_uncertainty']['face_to_actual_data_origin_m'] == {'left': None, 'right': None}
    for side, y in [('left', .22875), ('right', -.22875)]:
        np.testing.assert_allclose(geometry['mounts'][side]['t_axle_lidar_m'], [.323, y, .5915])
    assert result['provenance']['original_candidate_status'] == 'VALIDATED_FOR_OFFLINE_REPLAY'
    assert result['provenance']['gyro_bias_independently_confirmed'] is False
    assert result['provenance']['original_source_hashes'] == original['provenance']['sources']
    assert experiment_capability(root, setup, resolve_hardware_setup(setup), **ids)['available']


@pytest.mark.parametrize('defect', ['candidate_bytes', 'candidate_digest', 'imu', 'wheel', 'assembly', 'setup', 'geometry_source', 'rotation', 'wheel_conversion', 'missing_wheel_conversion'])
def test_stale_mismatched_or_tampered_evidence_fails_closed(evidence, defect):
    root, setup, ids, original, profile = evidence
    profile_path = root/'config/calibration/live_experiment_profile.json'
    source = root/profile['motion_candidate']['source_path']
    if defect == 'candidate_bytes':
        source.write_bytes(source.read_bytes()+b' ')
    elif defect in ('candidate_digest', 'rotation', 'wheel_conversion', 'missing_wheel_conversion'):
        if defect == 'candidate_digest': original['wheel_yaw_scale'] = 1.1
        elif defect == 'rotation': original['provenance']['R_reference_imu'] = [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]
        elif defect == 'wheel_conversion': original['provenance']['wheel_conversion']['track_width_m'] += .01
        else: del original['provenance']['wheel_conversion']
        if defect != 'candidate_digest': original['candidate_id'] = offline_motion_calibration._candidate_digest(original)
        source.write_bytes(raw_json(original))
        profile['motion_candidate']['sha256'] = hashlib.sha256(source.read_bytes()).hexdigest()
    elif defect == 'imu': profile['imu_sensor_id'] = 'H30-other'
    elif defect == 'wheel': profile['wheel_device_id'] = 'ZLAC8030D-other'
    elif defect == 'assembly': profile['assembly_revision'] = 'OLD'
    elif defect == 'setup': profile['hardware_setup_content_sha256'] = 'f'*64
    else:
        path = root/setup['source_documents'][0]['snapshot_path']; path.write_bytes(path.read_bytes()+b' ')
    profile_path.write_bytes(raw_json(profile))
    with pytest.raises(ValueError): prepare(evidence)
    capability = experiment_capability(root, setup, resolve_hardware_setup(setup), **ids)
    assert not capability['available'] and capability['reason'] and capability['provenance'] is None


def live_settings(evidence, **changes):
    c = prior_config()
    c.update(continuous_mapping=True, motion_model='planar_ekf', estimator='five_state',
        odometry_source='wheel_imu', wheel_imu_covariance=authority_weights(),
        live_scheme='experiment', live_experiment_motion=prepare(evidence)['motion_parameters'],
        wheel_candidate=resolve_hardware_setup(evidence[1])['wheel_candidate'],
        policy={'history_s': 5.}, **evidence[2])
    c.update(changes)
    return c


def test_experimental_math_matches_offline_with_fixed_bias_and_original_full_covariance(evidence):
    c = live_settings(evidence, motion_correction=True)
    live = MotionPrior(c, 'new_live_session', clock=lambda: 1000.)
    old = copy.deepcopy(evidence[3])
    offline_config = copy.deepcopy(c)
    for key in ('live_scheme', 'live_experiment_motion', 'motion_correction'): offline_config.pop(key)
    offline_config['offline_experiment'] = True
    offline = OfflineCorrectedPrior(offline_config, 'synthetic', candidate=old,
        candidate_sha256=evidence[4]['motion_candidate']['sha256'], clock=lambda: 1000.)
    bias = old['bias_native_rad_s']
    stamp = feed(live, bias=bias, extra_queries=True)
    feed(offline, bias=bias)
    a, b = live._planar_state_at(stamp), offline._planar_state_at(stamp)
    np.testing.assert_allclose(a.x, b.x, atol=1e-13, rtol=0)
    np.testing.assert_allclose(a.P, b.P, atol=1e-13, rtol=0)
    assert a.last_update['wheel']['measurement_covariance'][0][1] != 0
    np.testing.assert_allclose(a.last_update['gyro']['measurement'], [.025], atol=1e-14)
    assert live.config['confirmed_gyro_bias'] is None
    assert live.initialization['confirmed_gyro_bias'] is None
    assert live.initialization['gyro_bias_status'] == 'EXPERIMENTAL_REUSED_OFFLINE_CANDIDATE_APPLIED'
    np.testing.assert_array_equal(live.initialization['gyro_bias_candidate_rad_s'], bias)
    report = live.sample_report(stamp, live.pose_at(stamp))
    np.testing.assert_array_equal(report['gyro_bias_applied_native_rad_s'], bias)
    assert report['live_motion_correction']['gyro_bias_independently_confirmed'] is False
    assert live.bias_report()['fixed_for_session'] is True
    assert live.bias_estimator.status()['accepted_version']['version'] == 0
    with pytest.raises(PriorError, match='保持固定'):
        live.confirm_bias_stationarity(1, stamp, stamp, evidence_id='new', source='external', independent=True)


def test_disabled_correction_keeps_baseline_and_does_not_apply_candidate_bias(evidence):
    experimental = MotionPrior(live_settings(evidence, motion_correction=False), 'new_session')
    c = live_settings(evidence)
    c.pop('live_experiment_motion'); c['live_scheme'] = 'validated'
    baseline = MotionPrior(c, 'new_session')
    stamp = feed(experimental, bias=evidence[3]['bias_native_rad_s']); feed(baseline, bias=evidence[3]['bias_native_rad_s'])
    np.testing.assert_array_equal(experimental._planar_state_at(stamp).x, baseline._planar_state_at(stamp).x)
    assert calibration_report(experimental.config)['enabled'] is False
    assert experimental.sample_report(stamp, experimental.pose_at(stamp))['gyro_bias_applied_native_rad_s'] == [0., 0., 0.]


@pytest.mark.parametrize('change', [
    {'live_scheme': 'validated'}, {'offline_experiment': True}, {'confirmed_gyro_bias': {}},
    {'live_motion_calibration': {}}, {'offline_motion_correction': {}},
    {'offline_imu_time_mode': 'device_relative'}, {'offline_imu_time_model': {}},
    {'continuous_mapping': False}, {'motion_model': 'se3_gyro'},
])
def test_experiment_never_silently_enters_strict_offline_or_double_correction_paths(evidence, change):
    with pytest.raises(ValueError): validate_live_options(live_settings(evidence, motion_correction=True, **change))


def test_strict_missing_declaration_still_refused_and_parameter_snapshot_tampering_detected(evidence):
    c = live_settings(evidence, motion_correction=True)
    c.pop('live_experiment_motion'); c['live_scheme'] = 'validated'
    with pytest.raises(ValueError, match='设备校正'): validate_live_options(c)
    c = live_settings(evidence)
    c['live_experiment_motion']['bias_native_rad_s'][0] += .00001
    with pytest.raises(ValueError, match='内容哈希'): validate_motion_parameters(c['live_experiment_motion'], c)


@pytest.fixture
def controller_case(evidence, monkeypatch):
    from wc_runtime import mapping_controller as controller
    root = evidence[0]
    monkeypatch.setattr(controller, 'ROOT', root)
    monkeypatch.setattr(controller, 'RUN', root/'.phase1_runtime')
    monkeypatch.setattr(controller, 'require_device_bindings',
        lambda: json.loads((root/'config/device_bindings.json').read_text(encoding='utf-8')))
    def forbidden(*args, **kwargs): raise AssertionError('configuration must never launch devices/processes')
    monkeypatch.setattr(controller.subprocess, 'Popen', forbidden)
    monkeypatch.setattr(controller.subprocess, 'run', forbidden)
    request = dict(mode='all', session_id='new_experiment_session', config_path=str(root/'config/mapping_live.json'),
        project_root=str(root), output_dir=str(root/'reports/maps/new'), duration_s=10, owner_pid=123,
        mapping_enabled=True, live_scheme='experiment', estimator='five_state')
    return controller, request


@pytest.mark.parametrize('mode', ['left', 'right', 'all'])
@pytest.mark.parametrize('motion,geometry', [(False, False), (True, False), (False, True), (True, True)])
def test_controller_input_and_check_config_accept_experiment_options(controller_case, mode, motion, geometry):
    controller, request = controller_case
    request.update(mode=mode, motion_correction=motion, geometry=geometry)
    config = controller.configuration(request)
    assert validate_input(config)
    assert config['offline_experiment'] is False and config['input_rate_hz'] <= 5
    assert config['prior_template']['live_scheme'] == 'experiment'
    assert config['prior_template']['confirmed_gyro_bias'] is None
    assert config['prior_template']['geometry'] == geometry
    summary = controller.check_configuration(request)
    assert summary['hardware_started'] is False and summary['live_scheme'] == 'experiment'
    assert summary['geometry_capabilities']['wheel_imu_motion_'+mode]['experimental_only'] is True
    request['live_scheme'] = 'validated'
    with pytest.raises(ValueError, match='外参能力阻塞'): controller.configuration(request)


def test_controller_keeps_five_hz_limit_and_rejects_injected_mounts(controller_case):
    controller, request = controller_case
    path = Path(request['config_path']); config = json.loads(path.read_text(encoding='utf-8'))
    config['input_rate_hz'] = 10; path.write_bytes(raw_json(config))
    with pytest.raises(ValueError, match='0 至 5'): controller.configuration(request)
    config['input_rate_hz'] = 5; path.write_bytes(raw_json(config))
    composed = controller.configuration(request)
    composed['mounts']['left']['t_axle_lidar_m'][0] += .1
    with pytest.raises(InputFailure, match='FIXED_HARDWARE_SETUP'): validate_input(composed)


def test_profile_and_original_candidate_are_archived_byte_for_byte(controller_case, evidence):
    controller, request = controller_case
    request['motion_correction'] = True
    config = controller.configuration(request)
    destination = evidence[0]/'session'; destination.mkdir()
    frozen = controller.archive_configuration(destination, config)
    profile = evidence[0]/'config/calibration/live_experiment_profile.json'
    candidate_path = evidence[0]/evidence[4]['motion_candidate']['source_path']
    assert (destination/'live_experiment_profile.json').read_bytes() == profile.read_bytes()
    assert (destination/'live_experiment_motion_candidate.json').read_bytes() == candidate_path.read_bytes()
    assert not any(key.startswith('_live_experiment') for key in frozen)
    assert frozen['confirmed_gyro_bias'] is None
    assert not (destination/'confirmed_gyro_bias.json').exists()
    provenance = json.loads((destination/'live_experiment_provenance.json').read_text(encoding='utf-8'))
    assert provenance['original_source_hashes'] == evidence[3]['provenance']['sources']
    assert provenance['physical_accuracy_validated'] is False


@pytest.mark.parametrize('change', [
    {'offline_imu_time_mode': 'device_relative'}, {'offline_imu_time_model': {}},
    {'offline_motion_correction': {}}, {'gyro_bias_config': 'config/bias.json'},
    {'live_motion_calibration_config': 'config/motion.json'},
])
def test_top_level_config_rejects_conflicting_declarations_before_devices(controller_case, change):
    controller, request = controller_case
    path = Path(request['config_path']); config = json.loads(path.read_text(encoding='utf-8'))
    config.update(change); path.write_bytes(raw_json(config))
    with pytest.raises(ValueError): controller.configuration(request)
