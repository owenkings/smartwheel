"""Actual controller/bias-confirm configuration boundaries, without processes."""
import copy
import hashlib
import json
from pathlib import Path

import pytest

from wc_runtime import mapping_controller as controller
from wc_runtime.mapping_bias_confirm import save_confirmation
from test_mapping_controller import setup, tmp_path, change_config


def confirmed_fixture(request):
    root = Path(request['project_root'])
    session = root/'reports/maps/synthetic_evidence'
    (session/'prior').mkdir(parents=True)
    candidate = {'candidate_id': 1, 'start_stamp_ns': 1_000_000_000,
                 'end_stamp_ns': 6_100_000_000, 'sample_count': 205,
                 'screening_passed': True, 'stationary_truth_available': False,
                 'bias_native_rad_s': [.0001, -.0002, .0003]}
    (session/'prior/status.json').write_text(json.dumps({'gyro_bias': {'latest_candidate': candidate}}),
                                            encoding='utf-8')
    (session/'runtime_config.json').write_text(json.dumps({
        'source_mode': 'real', 'imu_sensor_id': 'H30-0000000015',
        'session_id': 'synthetic_evidence'}), encoding='utf-8')
    destination = root/'config/confirmed_fixture.json'
    review = save_confirmation(session, destination)
    assert review['status'] == 'CONFIRMATION_REQUIRED' and not destination.exists()
    assert review['hardware_started'] is False
    saved = save_confirmation(session, destination, confirmed=True,
                              candidate_token=review['candidate_token'])
    assert saved['applied_to_running_session'] is False
    evidence = destination.with_name(destination.stem+'.evidence.json')
    change_config(request, lambda c: c.update(gyro_bias_config='config/confirmed_fixture.json'))
    return destination, evidence


def test_default_planar_and_preview_checks_do_not_start_hardware_or_write_files(setup):
    root = Path(setup['project_root'])
    before = {str(p.relative_to(root)): p.read_bytes() for p in root.rglob('*') if p.is_file()}
    mapping = controller.configuration(setup)
    assert mapping['motion_model'] == mapping['prior_template']['motion_model'] == 'planar_ekf'
    assert mapping['confirmed_gyro_bias'] is None
    assert mapping['prior_template']['confirmed_gyro_bias'] is None
    assert mapping['continuous_mapping'] is True
    checked = controller.check_configuration(setup)
    preview = controller.check_configuration({**setup, 'mapping_enabled': False})
    assert checked['hardware_started'] is preview['hardware_started'] is False
    assert checked['motion_model'] == preview['motion_model'] == 'planar_ekf'
    assert preview['operation'] == 'live_preview' and preview['cloud_filter_effective'] is False
    assert preview['runtime_limits']['stationary_startup_required'] is False
    assert preview['runtime_limits']['data_freshness_shutdown'] is False
    after = {str(p.relative_to(root)): p.read_bytes() for p in root.rglob('*') if p.is_file()}
    assert before == after


def test_confirmed_bias_and_evidence_are_loaded_checked_and_archived_from_frozen_bytes(setup):
    path, evidence_path = confirmed_fixture(setup)
    original_bias, original_evidence = path.read_bytes(), evidence_path.read_bytes()
    resolved = controller.configuration(setup)
    checked = controller.check_configuration(setup)
    assert checked['hardware_started'] is False
    assert checked['gyro_bias_provenance'] == resolved['gyro_bias_provenance']
    assert resolved['confirmed_gyro_bias']['bias_native_rad_s'] == [.0001, -.0002, .0003]
    assert resolved['prior_template']['confirmed_gyro_bias'] == resolved['confirmed_gyro_bias']
    assert resolved['confirmed_gyro_bias']['source'] == 'operator_visual_confirmation'
    assert resolved['gyro_bias_provenance']['sha256'] == hashlib.sha256(original_bias).hexdigest()
    assert resolved['gyro_bias_provenance']['evidence_sha256'] == hashlib.sha256(original_evidence).hexdigest()
    # A later external edit must not silently change the resolved session.
    path.write_bytes(b'{"changed_after_configuration":true}\n')
    evidence_path.write_bytes(b'{"changed_after_configuration":true}\n')
    directory = Path(setup['output_dir']); directory.mkdir(parents=True)
    frozen = controller.archive_configuration(directory, resolved)
    assert (directory/'confirmed_gyro_bias.json').read_bytes() == original_bias
    assert (directory/'confirmed_gyro_bias.evidence.json').read_bytes() == original_evidence
    archived = json.loads((directory/'runtime_config.json').read_text(encoding='utf-8'))
    assert archived['confirmed_gyro_bias'] == resolved['confirmed_gyro_bias']
    assert archived['prior_template']['confirmed_gyro_bias'] == resolved['confirmed_gyro_bias']
    assert archived['gyro_bias_provenance'] == resolved['gyro_bias_provenance']
    assert '_gyro_bias_raw_text' not in archived and '_gyro_bias_evidence_raw_text' not in archived
    assert frozen['gyro_bias_provenance'] == resolved['gyro_bias_provenance']


def test_tampered_evidence_is_rejected_before_configuration_or_preview(setup):
    _, evidence_path = confirmed_fixture(setup)
    # Valid JSON with changed bytes still breaks the declared evidence hash.
    evidence_path.write_bytes(evidence_path.read_bytes()+b' ')
    with pytest.raises(ValueError, match='SHA-256'):
        controller.configuration(setup)
    with pytest.raises(ValueError, match='SHA-256'):
        controller.check_configuration({**setup, 'mapping_enabled': False})


@pytest.mark.parametrize('field', ['_gyro_bias_raw_text', '_gyro_bias_evidence_raw_text'])
def test_archive_rejects_tampered_in_memory_frozen_payload(setup, field):
    confirmed_fixture(setup)
    resolved = controller.configuration(setup)
    bad = copy.deepcopy(resolved); bad[field] += ' '
    directory = Path(setup['output_dir']); directory.mkdir(parents=True)
    with pytest.raises(ValueError, match='哈希'):
        controller.archive_configuration(directory, bad)
    assert not (directory/'runtime_config.json').exists()


def test_inline_bias_cannot_bypass_evidence_file_import(setup):
    change_config(setup, lambda c: c.update(gyro_bias_config=None,
                                          confirmed_gyro_bias={'status': 'INDEPENDENTLY_CONFIRMED'}))
    with pytest.raises(ValueError, match='gyro_bias_config'):
        controller.configuration(setup)
