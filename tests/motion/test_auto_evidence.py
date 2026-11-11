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
NOT_INSTALLED = {'installed': False, 'decided_by': 'user', 'decided_at': '2026-11-09',
                 'decision': '不做硬件松手即停；第一次空车低速闭环用车上现有急停、人在车旁'}
DRIVER_KEPT_GOING = {'verified': True, 'trials': 3, 'driver_stopped': False, 'max_stop_time_s': None,
                     'decided_by': 'coordinator', 'decided_at': '2026-11-10 14:40',
                     'decision': '拔 USB 时驱动器不停：0.10 m/s 照样开，掉线靠人按急停，回头再改驱动器参数'}


@pytest.fixture
def project(tmp_path):
    (tmp_path/'evidence').mkdir()
    record = tmp_path/'evidence'/'test.txt'
    record.write_text('synthetic test record\n')
    return tmp_path, {'kind': 'recording', 'path': 'evidence/test.txt',
                      'sha256': hashlib.sha256(record.read_bytes()).hexdigest()}


def braking(source, speed, paths=tuple(e.BRAKING_LIMITS_M)):
    return [{'path': path, 'speed_m_s': speed, 'distances_m': [0.05, 0.05, 0.051, 0.052, 0.05],
             'max_heading_change_deg': 1.0, 'sources': [source],
             **({'moved_after_reset': False} if path == 'estop' else {})} for path in paths]


def evidence(source, stage='STAND_TEST', hold=None):
    test = {'verified': True, 'trials': 3, 'max_stop_time_s': 0.4, 'sources': [source]}
    value = {'schema': e.EVIDENCE_SCHEMA, 'state': 'REVIEWED', 'stage': stage, 'device_id': 'ZLAC8030D-0000000014',
             'firmware_id': 'SYNTHETIC', 'reviewer': 'test', 'reviewed_at': '2026-11-09',
             'hold_to_run_switch': copy.deepcopy(hold or {'installed': True}), 'communication_timeout_s': 0.3,
             'approved_limits': dict(LIMITS),
             'driver_readback': [{'purpose': 'communication_timeout', 'request_hex': read_request(1, 0x2000, 1).hex(),
                                  'word_index': 0, 'mask': 65535, 'equals': 300}],
             'stop_tests': {name: copy.deepcopy(test) for name in e.STOP_TESTS}}
    value['stop_tests']['estop']['moved_after_reset'] = False
    value['stop_tests']['auto_stand_checks'].update(speed_ratios=[0.98, 1.0, 1.03], estop_trials=1,
                                                    moved_after_reset=False)
    if stage == 'SUPERVISED_CLOSED_LOOP':
        value['braking'] = braking(source, 0.10)
    return value


def load(root, value):
    return e.load_evidence(value, root, 'ZLAC8030D-0000000014')


def test_template_is_always_rejected():
    template = json.loads((ROOT/'config/wheel_auto_drive_evidence.template.json').read_text())
    assert set(template['stop_tests']) == set(e.STOP_TESTS)
    with pytest.raises(FeedbackError, match='not reviewed'):
        load(ROOT, template)
    reviewed = {**template, 'state': 'REVIEWED'}
    with pytest.raises(FeedbackError):
        load(ROOT, reviewed)


@pytest.mark.parametrize('stage', e.STAGES)
def test_complete_synthetic_evidence_gives_the_approved_limits(project, stage):
    root, source = project
    result = load(root, evidence(source, stage))
    assert result['stage'] == stage and result['limits'] == LIMITS
    assert result['readback'][0]['purpose'] == 'communication_timeout' and len(result['evidence_sha256']) == 64
    assert result['hold_to_run_switch']['installed'] is True and result['communication_timeout_s'] == 0.3
    assert result['speed_caps'] == {'max_linear_m_s': 0.10, 'max_angular_rad_s': 0.30}


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
    # the owner reads these at session start, so only requests of the read-only whitelist
    (lambda v: v['driver_readback'][0].update(request_hex=read_request(1, 0x20AB, 2).hex()), 'whitelist'),
    (lambda v: v['stop_tests']['estop'].update(verified=False), 'estop'),
    (lambda v: v['stop_tests']['estop'].pop('moved_after_reset'), 'moved_after_reset'),
    (lambda v: v['stop_tests']['estop'].update(moved_after_reset=True), 'moved_after_reset'),
    (lambda v: v['stop_tests']['orin_power_cut'].update(trials=2), 'trials'),
    (lambda v: v['stop_tests']['dead_man'].update(sources=[]), 'sources'),
    (lambda v: v['stop_tests']['usb_unplugged']['sources'][0].update(sha256='0'*64), 'hash mismatch'),
    (lambda v: v['stop_tests']['owner_killed']['sources'][0].update(path='/etc/hostname'), 'checkout or the data root'),
    (lambda v: v.pop('hold_to_run_switch'), 'hold_to_run_switch'),
    (lambda v: v.update(hold_to_run_switch={'installed': 'no'}), 'hold_to_run_switch'),
    (lambda v: v.update(hold_to_run_switch={'installed': False}), 'decision'),
    (lambda v: v.update(hold_to_run_switch={**NOT_INSTALLED, 'decided_by': 'A'}), 'decided_by the user'),
    (lambda v: v.pop('communication_timeout_s'), 'communication_timeout_s'),
    # section 8.1 level 2: communication-loss stops within the driver's timeout plus 0.5 s, and they must be measured
    (lambda v: v['stop_tests']['owner_killed'].update(max_stop_time_s=None), 'owner_killed.max_stop_time_s'),
    (lambda v: v['stop_tests']['usb_unplugged'].update(max_stop_time_s=0.9), 'communication timeout plus'),
    (lambda v: v['stop_tests']['orin_power_cut'].update(max_stop_time_s=9.0), 'communication timeout plus'),
    (lambda v: v['stop_tests']['usb_unplugged'].update(driver_stopped='no'), 'driver_stopped'),
    (lambda v: v.update(closed_loop_runs=[{'max_linear_m_s': 0.1}]), 'SUPERVISED_CLOSED_LOOP')])
def test_incomplete_or_inconsistent_evidence_is_rejected(project, change, message):
    root, source = project
    value = evidence(source)
    change(value)
    with pytest.raises(FeedbackError, match=message):
        load(root, value)


def test_each_stage_needs_its_own_tests_and_the_others_are_optional(project):
    root, source = project
    value = evidence(source, 'STAND_TEST')
    for name in ('owner_killed', 'orin_power_cut', 'software_zero', 'source_killed', 'auto_stand_checks'):
        del value['stop_tests'][name]
    assert load(root, value)['stage'] == 'STAND_TEST'     # the manual channel on the stand: estop and USB pulled
    for name in ('estop', 'usb_unplugged', 'dead_man'):
        short = copy.deepcopy(value)
        del short['stop_tests'][name]
        with pytest.raises(FeedbackError, match=name):
            load(root, short)
    value['stage'] = 'GROUND_TEST'
    with pytest.raises(FeedbackError, match='auto_stand_checks'):
        load(root, value)
    value['stop_tests']['owner_killed'] = {'verified': False, 'trials': 0, 'max_stop_time_s': None, 'sources': []}
    value['stop_tests']['auto_stand_checks'] = {'verified': True, 'trials': 3, 'speed_ratios': [1.0, 0.97, 1.02],
                                                'estop_trials': 1, 'moved_after_reset': False,
                                                'sources': [source]}
    assert load(root, value)['stage'] == 'GROUND_TEST'   # an optional test not done yet is ignored
    value['stage'] = 'SUPERVISED_CLOSED_LOOP'
    value['braking'] = braking(source, 0.10)
    assert load(root, value)['stage'] == 'SUPERVISED_CLOSED_LOOP'


@pytest.mark.parametrize('ratios, message', [
    ([1.0, 1.0], 'one feedback/command ratio per trial'), (None, 'speed_ratios'),
    ([1.0, 1.11, 1.0], 'speed conversion: wheel feedback 111 %'), ([0.89, 1.0, 1.0], 'speed conversion'),
    ([1.0, 8.9, 1.0], 'speed conversion')])
def test_stand_checks_need_the_wheel_feedback_within_ten_percent_of_the_command(project, ratios, message):
    root, source = project
    value = evidence(source, 'GROUND_TEST')
    value['stop_tests']['auto_stand_checks']['speed_ratios'] = ratios
    with pytest.raises(FeedbackError, match=message):
        load(root, value)
    value['stop_tests']['auto_stand_checks']['speed_ratios'] = [0.9, 1.1, 1.0]
    assert load(root, value)['stage'] == 'GROUND_TEST'


@pytest.mark.parametrize('change, message', [
    (lambda v: v.pop('estop_trials'), 'auto_stand_checks.estop_trials'),
    (lambda v: v.update(estop_trials=0), 'auto_stand_checks.estop_trials'),
    (lambda v: v.pop('moved_after_reset'), 'auto_stand_checks.moved_after_reset must be false'),
    (lambda v: v.update(moved_after_reset=True), 'auto_stand_checks.moved_after_reset must be false')])
def test_stand_checks_need_an_emergency_stop_during_automatic_control_without_motion_on_the_reset(
        project, change, message):
    """Second review of 2026-11-10, item 2: field step 5.3, the stop the wheel feedback rule has to see."""
    root, source = project
    value = evidence(source, 'GROUND_TEST')
    change(value['stop_tests']['auto_stand_checks'])
    with pytest.raises(FeedbackError, match=message):
        load(root, value)


@pytest.mark.parametrize('moved', [None, True])
def test_closed_loop_estop_rows_need_the_chair_still_after_the_reset(project, moved):
    root, source = project
    value = evidence(source, 'SUPERVISED_CLOSED_LOOP')
    row = next(row for row in value['braking'] if row['path'] == 'estop')
    if moved is None:
        del row['moved_after_reset']
    else:
        row['moved_after_reset'] = moved
    with pytest.raises(FeedbackError, match='estop braking row needs moved_after_reset false'):
        load(root, value)


def test_without_a_known_timeout_a_line_loss_stop_after_1_5_s_counts_as_not_stopped(project):
    """Second review of 2026-11-10, item 6."""
    root, source = project
    value = evidence(source, 'STAND_TEST', hold=NOT_INSTALLED)
    del value['stop_tests']['dead_man'], value['stop_tests']['owner_killed'], value['stop_tests']['orin_power_cut']
    value['communication_timeout_s'] = None
    value['driver_readback'][0].update(purpose='input_functions', request_hex=read_request(1, 0x2016, 3).hex())
    value['stop_tests']['usb_unplugged'] = {'verified': True, 'trials': 3, 'driver_stopped': True,
                                            'max_stop_time_s': 1.5, 'sources': [source]}
    assert load(root, value)['communication_timeout_s'] is None
    for stop in (1.6, 9.9):
        value['stop_tests']['usb_unplugged']['max_stop_time_s'] = stop
        with pytest.raises(FeedbackError, match='counts as not stopped'):
            load(root, value)
    value['stop_tests']['usb_unplugged'] = dict(DRIVER_KEPT_GOING, sources=[source])   # recorded as not stopped
    assert load(root, value)['stage'] == 'STAND_TEST'


@pytest.mark.parametrize('stage', e.STAGES)
def test_a_driver_that_keeps_going_without_the_line_is_accepted_at_0_10_m_s_with_the_decision(project, stage):
    root, source = project
    value = evidence(source, stage, hold=NOT_INSTALLED)
    del value['stop_tests']['dead_man'], value['stop_tests']['owner_killed'], value['stop_tests']['orin_power_cut']
    value['stop_tests']['usb_unplugged'] = dict(DRIVER_KEPT_GOING, sources=[source])
    value['communication_timeout_s'] = None               # not known: the driver did not stop
    value['driver_readback'][0]['purpose'] = 'input_functions'
    value['driver_readback'][0]['request_hex'] = read_request(1, 0x2016, 3).hex()
    if stage == 'SUPERVISED_CLOSED_LOOP':                 # no communication-loss braking row needed
        value['braking'] = braking(source, 0.10, ('software_zero', 'source_killed', 'estop'))
    result = load(root, value)
    assert result['communication_timeout_s'] is None and result['limits'] == LIMITS
    stopped = copy.deepcopy(value)                        # stopped by itself, timeout still not known: accepted
    stopped['stop_tests']['usb_unplugged'] = {'verified': True, 'trials': 3, 'driver_stopped': True,
                                              'max_stop_time_s': 1.2, 'sources': [source]}
    if stage == 'SUPERVISED_CLOSED_LOOP':                 # then the communication-loss braking row is needed
        stopped['braking'] = braking(source, 0.10, ('software_zero', 'source_killed', 'estop'))
        with pytest.raises(FeedbackError, match='approved top speed for communication_loss'):
            load(root, stopped)
        stopped['braking'] = braking(source, 0.10, ('software_zero', 'source_killed', 'communication_loss', 'estop'))
    assert load(root, stopped)['communication_timeout_s'] is None
    timeout_check = {'purpose': 'communication_timeout', 'request_hex': read_request(1, 0x2000, 1).hex(),
                     'word_index': 0, 'mask': 65535, 'equals': 1000}
    for change, message in [
            (lambda v: v['approved_limits'].update(max_linear_m_s=0.2), 'communication_timeout_s is required above'),
            (lambda v: (v['approved_limits'].update(max_linear_m_s=0.2), v.update(communication_timeout_s=1.0),
                        v['driver_readback'].append(timeout_check)), 'only up to 0.10 m/s'),
            (lambda v: v['stop_tests']['usb_unplugged'].pop('decision'), 'usb_unplugged.decision'),
            (lambda v: v['stop_tests']['usb_unplugged'].update(decided_by='A'), 'decided_by must be one of'),
            (lambda v: v['stop_tests']['usb_unplugged'].update(decided_at=''), 'decided_at'),
            (lambda v: v['stop_tests']['usb_unplugged'].update(max_stop_time_s=1.2), 'must be null'),
            (lambda v: v['stop_tests']['usb_unplugged'].update(driver_stopped=True), 'usb_unplugged.max_stop_time_s'),
            (lambda v: v['stop_tests'].update(owner_killed={'verified': True, 'trials': 3, 'driver_stopped': 'yes',
                                                             'sources': [source]}), 'owner_killed.driver_stopped')]:
        changed = copy.deepcopy(value)
        change(changed)
        with pytest.raises(FeedbackError, match=message):
            load(root, changed)
    value['communication_timeout_s'] = 1.0   # known after all: the readback must then include it
    with pytest.raises(FeedbackError, match='driver readback must include communication_timeout'):
        load(root, value)


@pytest.mark.parametrize('stage', e.STAGES)
def test_switch_not_installed_is_recorded_and_drops_the_dead_man_test_and_braking_path(project, stage):
    root, source = project
    value = evidence(source, stage, hold=NOT_INSTALLED)
    del value['stop_tests']['dead_man']
    if stage == 'SUPERVISED_CLOSED_LOOP':
        value['braking'] = braking(source, 0.10, ('software_zero', 'source_killed', 'communication_loss', 'estop'))
    result = load(root, value)
    assert result['hold_to_run_switch'] == {'installed': False, 'decision': NOT_INSTALLED['decision'],
                                            'decided_at': '2026-11-09'}
    installed = evidence(source, stage)
    del installed['stop_tests']['dead_man']
    with pytest.raises(FeedbackError, match='dead_man'):
        load(root, installed)


@pytest.mark.parametrize('change, message', [
    (lambda rows: rows.pop(), 'approved top speed'),
    (lambda rows: rows.pop(1), 'approved top speed for source_killed'),
    (lambda rows: rows[0].update(distances_m=[0.2]*5), 'pass line'),
    (lambda rows: rows[1].update(distances_m=[0.19, 0.17, 0.17]), 'pass line for source_killed'),
    (lambda rows: rows[1].update(distances_m=[0.05, 0.05, 0.05, 0.05, 0.08]), 'scatter'),
    (lambda rows: rows[3].update(distances_m=[0.20, 0.24, 0.27]), 'scatter'),
    (lambda rows: rows[2].update(max_heading_change_deg=6.0), 'heading'),
    (lambda rows: rows[3].update(distances_m=[0.05]*2), 'at least 3'),
    (lambda rows: [row.update(speed_m_s=0.05) for row in rows], 'approved top speed')])
def test_closed_loop_needs_braking_rows_within_the_pass_lines(project, change, message):
    root, source = project
    value = evidence(source, 'SUPERVISED_CLOSED_LOOP')
    change(value['braking'])
    with pytest.raises(FeedbackError, match=message):
        load(root, value)


def test_braking_spread_allows_two_centimetres_at_low_speed(project):
    root, source = project
    value = evidence(source, 'SUPERVISED_CLOSED_LOOP')
    for row in value['braking']:
        row['distances_m'] = [0.02, 0.03, 0.04]          # 2 cm apart at a 3 cm mean: within the 2 cm floor
    assert load(root, value)['stage'] == 'SUPERVISED_CLOSED_LOOP'
    value['braking'][0]['distances_m'] = [0.02, 0.03, 0.045]
    with pytest.raises(FeedbackError, match='scatter'):
        load(root, value)
    value['braking'][0]['distances_m'] = [0.15, 0.17, 0.18]   # 20 % of 0.167 m is 3.3 cm: within
    assert load(root, value)['stage'] == 'SUPERVISED_CLOSED_LOOP'


def test_ground_test_goes_up_one_speed_step_after_braking_at_the_step_below(project):
    root, source = project
    value = evidence(source, 'GROUND_TEST')
    value['approved_limits']['max_linear_m_s'] = 0.20
    with pytest.raises(FeedbackError, match='next speed step 0.10'):
        load(root, value)                                   # nothing measured yet: 0.10 m/s first
    value['braking'] = braking(source, 0.10)
    assert load(root, value)['speed_caps']['max_linear_m_s'] == 0.20
    value['approved_limits']['max_linear_m_s'] = 0.30
    with pytest.raises(FeedbackError, match='next speed step 0.20'):
        load(root, value)                                   # 0.30 only after braking at 0.20
    value['braking'] += braking(source, 0.20)
    assert load(root, value)['speed_caps']['max_linear_m_s'] == 0.30
    value['approved_limits']['max_angular_rad_s'] = 0.40
    with pytest.raises(FeedbackError, match='max_angular_rad_s'):
        load(root, value)                                   # turning stays at 0.30 rad/s off the closed loop


def test_closed_loop_goes_up_one_step_after_a_passed_run_at_the_step_below(project):
    root, source = project
    value = evidence(source, 'SUPERVISED_CLOSED_LOOP')
    value['braking'] = braking(source, 0.20)
    value['approved_limits']['max_linear_m_s'] = 0.20
    with pytest.raises(FeedbackError, match='next speed step 0.10'):
        load(root, value)                                   # first closed loop: 0.10 m/s, 0.30 rad/s
    run = {'max_linear_m_s': 0.10, 'max_angular_rad_s': 0.30, 'passed': False, 'reviewed_at': '2026-11-20',
           'sources': [source]}
    value['closed_loop_runs'] = [run]
    with pytest.raises(FeedbackError, match='next speed step 0.10'):
        load(root, value)                                   # a failed run does not count
    run['passed'] = True
    assert load(root, value)['speed_caps'] == {'max_linear_m_s': 0.20, 'max_angular_rad_s': 0.40}
    value['approved_limits']['max_linear_m_s'] = 0.30
    with pytest.raises(FeedbackError, match='approved top speed|next speed step'):
        load(root, value)                                   # neither braked nor run at 0.20 yet
    run['sources'] = []
    value['approved_limits']['max_linear_m_s'] = 0.20
    with pytest.raises(FeedbackError, match='sources'):
        load(root, value)


def test_read_evidence_file_resolves_relative_to_the_checkout(project):
    root, source = project
    (root/'evidence'/'auto.json').write_text(json.dumps(evidence(source)))
    assert e.read_evidence_file('evidence/auto.json', root, 'ZLAC8030D-0000000014')['stage'] == 'STAND_TEST'
    (root/'evidence'/'bad.json').write_text('{"schema": NaN}')
    with pytest.raises(FeedbackError):
        e.read_evidence_file('evidence/bad.json', root, 'ZLAC8030D-0000000014')


def test_check_command_reads_the_file_as_the_wheel_owner_does(project, capsys):
    root, source = project
    (root/'evidence'/'auto.json').write_text(json.dumps(evidence(source, 'GROUND_TEST')))
    (root/'config').mkdir()
    (root/'config'/'wheel_feedback_current.json').write_text(json.dumps({'device_id': 'ZLAC8030D-0000000014'}))
    assert e.main(['--project-root', str(root), '--evidence', 'evidence/auto.json']) == 0
    out = capsys.readouterr().out
    assert out.startswith('PASS: stage GROUND_TEST, device ZLAC8030D-0000000014, limits 0.10 m/s 0.30 rad/s')
    assert 'readback communication_timeout' in out
    assert e.main(['--project-root', str(root), '--evidence', 'evidence/auto.json',
                   '--device-id', 'ZLAC8030D-5C66036979']) == 1
    assert 'REJECTED: evidence identifies another wheel controller' in capsys.readouterr().out
    assert e.main(['--project-root', str(root), '--evidence', str(ROOT/'config/wheel_auto_drive_evidence.template.json')]) == 1
    assert 'REJECTED: automatic drive evidence is not reviewed' in capsys.readouterr().out
    assert e.main(['--project-root', str(root), '--evidence', 'evidence/missing.json']) == 1
    assert 'REJECTED' in capsys.readouterr().out
