"""Evidence file of the automatic velocity channel: the template is always rejected; synthetic records only."""
import copy
import hashlib
import json
from pathlib import Path

import pytest

from wc_motion import auto_evidence as e
from wc_motion.protocol import FeedbackError, read_request


ROOT = Path(__file__).resolve().parents[2]
LIMITS = {'max_linear_m_s': 0.10, 'max_angular_rad_s': 0.30, 'max_wheel_rpm': 27.0, 'max_linear_accel_m_s2': 0.30,
          'max_linear_decel_m_s2': 0.60, 'max_angular_accel_rad_s2': 0.80}


@pytest.fixture
def project(tmp_path):
    (tmp_path/'evidence').mkdir()
    record = tmp_path/'evidence'/'test.txt'
    record.write_text('synthetic test record\n')
    return tmp_path, {'kind': 'recording', 'path': 'evidence/test.txt',
                      'sha256': hashlib.sha256(record.read_bytes()).hexdigest()}


def evidence(source, stage='STAND_TEST'):
    test = {'verified': True, 'trials': 3, 'max_stop_time_s': 0.4, 'sources': [source]}
    value = {'schema': e.EVIDENCE_SCHEMA, 'state': 'REVIEWED', 'stage': stage, 'device_id': 'ZLAC8030D-0000000014',
             'firmware_id': 'SYNTHETIC', 'reviewer': 'test', 'reviewed_at': '2026-11-09',
             'approved_limits': dict(LIMITS),
             'driver_readback': [{'purpose': 'communication_timeout', 'request_hex': read_request(1, 0x2000, 1).hex(),
                                  'word_index': 0, 'mask': 65535, 'equals': 7}],
             'stop_tests': {name: copy.deepcopy(test) for name in e.STOP_TESTS}}
    if stage == 'SUPERVISED_CLOSED_LOOP':
        value['braking'] = [{'path': path, 'speed_m_s': 0.10, 'distances_m': [0.05, 0.05, 0.051, 0.052, 0.05],
                             'max_heading_change_deg': 1.0, 'sources': [source]} for path in e.BRAKING_LIMITS_M]
    return value


def test_template_is_always_rejected():
    template = json.loads((ROOT/'config/wheel_auto_drive_evidence.template.json').read_text())
    with pytest.raises(FeedbackError, match='not reviewed'):
        e.load_evidence(template, ROOT, 'ZLAC8030D-0000000014')
    reviewed = {**template, 'state': 'REVIEWED'}
    with pytest.raises(FeedbackError):
        e.load_evidence(reviewed, ROOT, 'ZLAC8030D-0000000014')


@pytest.mark.parametrize('stage', e.STAGES)
def test_complete_synthetic_evidence_gives_the_approved_limits(project, stage):
    root, source = project
    result = e.load_evidence(evidence(source, stage), root, 'ZLAC8030D-0000000014')
    assert result['stage'] == stage and result['limits'] == LIMITS
    assert result['readback'][0]['purpose'] == 'communication_timeout' and len(result['evidence_sha256']) == 64


@pytest.mark.parametrize('change, message', [
    (lambda v: v.update(stage='FIELD'), 'stage'),
    (lambda v: v.update(device_id='ZLAC8030D-0000000099'), 'another wheel controller'),
    (lambda v: v.update(firmware_id=''), 'firmware_id'),
    (lambda v: v['approved_limits'].update(max_linear_m_s=0.2), 'max_linear_m_s'),
    (lambda v: v['approved_limits'].update(max_angular_rad_s=0.4), 'max_angular_rad_s'),
    (lambda v: v['approved_limits'].pop('max_wheel_rpm'), 'approved_limits'),
    (lambda v: v.update(driver_readback=[]), 'readback'),
    (lambda v: v['driver_readback'][0].update(purpose='acceleration_time'), 'communication_timeout'),
    (lambda v: v['driver_readback'][0].update(request_hex='0106200e0007' + '0000'), 'RTU frame'),
    (lambda v: v['driver_readback'][0].update(equals=70000), 'equals'),
    (lambda v: v['stop_tests']['estop'].update(verified=False), 'estop'),
    (lambda v: v['stop_tests']['orin_power_cut'].update(trials=2), 'trials'),
    (lambda v: v['stop_tests']['dead_man'].update(sources=[]), 'sources'),
    (lambda v: v['stop_tests']['usb_unplugged']['sources'][0].update(sha256='0'*64), 'hash mismatch'),
    (lambda v: v['stop_tests']['owner_killed']['sources'][0].update(path='/etc/hostname'), 'checkout or the data root')])
def test_incomplete_or_inconsistent_evidence_is_rejected(project, change, message):
    root, source = project
    value = evidence(source)
    change(value)
    with pytest.raises(FeedbackError, match=message):
        e.load_evidence(value, root, 'ZLAC8030D-0000000014')


def test_stand_stage_does_not_need_the_automatic_stand_checks_but_ground_stage_does(project):
    root, source = project
    value = evidence(source, 'STAND_TEST')
    del value['stop_tests']['software_zero'], value['stop_tests']['auto_stand_checks']
    assert e.load_evidence(value, root, 'ZLAC8030D-0000000014')['stage'] == 'STAND_TEST'
    value['stage'] = 'GROUND_TEST'
    with pytest.raises(FeedbackError, match='software_zero'):
        e.load_evidence(value, root, 'ZLAC8030D-0000000014')


@pytest.mark.parametrize('change, message', [
    (lambda rows: rows.pop(), 'approved top speed'),
    (lambda rows: rows[0].update(distances_m=[0.2]*5), 'pass line'),
    (lambda rows: rows[1].update(distances_m=[0.05, 0.05, 0.05, 0.05, 0.08]), 'scatter'),
    (lambda rows: rows[2].update(max_heading_change_deg=6.0), 'heading'),
    (lambda rows: rows[3].update(distances_m=[0.05]*4), 'at least 5'),
    (lambda rows: [row.update(speed_m_s=0.05) for row in rows], 'approved top speed')])
def test_closed_loop_needs_braking_rows_within_the_pass_lines(project, change, message):
    root, source = project
    value = evidence(source, 'SUPERVISED_CLOSED_LOOP')
    change(value['braking'])
    with pytest.raises(FeedbackError, match=message):
        e.load_evidence(value, root, 'ZLAC8030D-0000000014')


def test_read_evidence_file_resolves_relative_to_the_checkout(project):
    root, source = project
    (root/'evidence'/'auto.json').write_text(json.dumps(evidence(source)))
    assert e.read_evidence_file('evidence/auto.json', root, 'ZLAC8030D-0000000014')['stage'] == 'STAND_TEST'
    (root/'evidence'/'bad.json').write_text('{"schema": NaN}')
    with pytest.raises(FeedbackError):
        e.read_evidence_file('evidence/bad.json', root, 'ZLAC8030D-0000000014')
