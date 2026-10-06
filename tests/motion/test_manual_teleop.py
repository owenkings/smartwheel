"""Manual operator intent, watchdog and failed-stop evidence using a simulation sink."""

import json
from pathlib import Path

import pytest

from wc_motion.manual_teleop import ManualController, DryRunBackend, REAL_BLOCK_REASON, main
from wc_motion.protocol import FeedbackError


CONFIG_PATH = Path(__file__).resolve().parents[2]/'config/wheel_manual_unvalidated.json'


@pytest.fixture
def config():
    return json.loads(CONFIG_PATH.read_text())


def armed(config):
    controller = ManualController(config)
    controller.heartbeat(10)
    controller.arm(10, operator_confirmed=True)
    return controller


def test_default_startup_and_disarmed_exit_send_nothing(config):
    controller = ManualController(config)
    controller.heartbeat(10)
    controller.key_event('w', True, 10)
    controller.close(10.1)
    assert controller.backend.commands == [] and controller.state == 'DISARMED'
    assert controller.status()['hardware_command_sent'] is False


def test_explicit_arm_required_with_fresh_link_and_no_implicit_enable(config):
    controller = ManualController(config)
    with pytest.raises(FeedbackError, match='operator arming'):
        controller.arm(10)
    with pytest.raises(FeedbackError, match='fresh'):
        controller.arm(10, operator_confirmed=True)
    controller.heartbeat(10)
    controller.arm(10, operator_confirmed=True)
    assert controller.backend.commands == []
    with pytest.raises(FeedbackError, match='already armed'):
        controller.arm(10.1, operator_confirmed=True)


def test_wasd_press_release_and_combined_input_stay_within_limits(config):
    controller = armed(config)
    controller.key_event('w', True, 10.01)
    controller.key_event('a', True, 10.02)
    controller.key_event('w', False, 10.03)
    controller.key_event('a', False, 10.04)
    controller.key_event('s', True, 10.05)
    controller.key_event('d', True, 10.06)
    assert controller.backend.commands == [(.1, 0), (.1, .2), (0, .2), (0, 0), (-.1, 0), (-.1, -.2)]
    assert controller.status()['hardware_stop_confirmed'] is False


def test_key_timeout_requests_zero_and_disarms_even_while_link_is_healthy(config):
    controller = armed(config)
    controller.key_event('w', True, 10.01)
    controller.heartbeat(10.2)
    controller.tick(10.3)
    assert controller.backend.commands[-1] == (0, 0) and controller.reason == 'KEY_TIMEOUT'
    assert controller.state == 'DISARMED'
    controller.key_event('w', True, 10.31)
    assert controller.backend.commands == [(.1, 0), (0, 0)]


def test_missing_link_heartbeat_stops_despite_fresh_key(config):
    controller = armed(config)
    controller.key_event('w', True, 10.01)
    controller.key_event('w', True, 10.2)
    controller.tick(10.26)
    assert controller.state == 'FAULT' and controller.reason == 'COMMAND_LINK_STALE'
    assert controller.backend.commands[-1] == (0, 0)
    with pytest.raises(FeedbackError, match='fault'):
        controller.arm(10.27, operator_confirmed=True)


def test_disconnect_attempts_stop_but_does_not_claim_physical_stop(config):
    events = []
    controller = armed(config)
    controller.observer = events.append
    controller.key_event('w', True, 10.01)
    controller.heartbeat(10.02, connected=False)
    assert controller.state == 'FAULT' and controller.target == (0, 0)
    assert controller.last_stop_simulated is False
    assert controller.status()['hardware_stop_confirmed'] is False
    assert any(event.get('reason') == 'COMMAND_LINK_DISCONNECTED' and event.get('linear_m_s') == 0 for event in events)
    controller.key_event(' ', True, 10.03)
    controller.heartbeat(10.04)
    with pytest.raises(FeedbackError, match='fault'):
        controller.arm(10.04, operator_confirmed=True)


def test_first_command_write_failure_still_attempts_zero_and_latches_fault(config):
    class Failing(DryRunBackend):
        def intent(self, linear, angular):
            self.commands.append((linear, angular))
            if (linear, angular) != (0, 0):
                raise OSError('simulated partial/non-acknowledged command')
            return {'simulation_acknowledged': True}
    controller = ManualController(config, backend=Failing())
    controller.heartbeat(10)
    controller.arm(10, operator_confirmed=True)
    with pytest.raises(OSError):
        controller.key_event('w', True, 10.01)
    assert controller.backend.commands == [(.1, 0), (0, 0)]
    assert controller.state == 'FAULT' and controller.last_stop_simulated is True


def test_logging_failure_cannot_prevent_stop_of_existing_simulated_motion(config):
    controller = armed(config)
    controller.key_event('w', True, 10.01)
    def broken_log(record):
        raise OSError('fixture disk failure')
    controller.observer = broken_log
    controller.key_event(' ', True, 10.02)
    assert controller.backend.commands[-1] == (0, 0)
    assert controller.state == 'FAULT' and controller.journal_error == 'fixture disk failure'


@pytest.mark.parametrize('cause', ['space', 'exit', 'invalid_key', 'time_reverse', 'time_nan'])
def test_manual_stop_and_input_faults_clear_moving_intent(config, cause):
    controller = armed(config)
    controller.key_event('w', True, 10.01)
    if cause == 'space':
        controller.key_event(' ', True, 10.02)
    elif cause == 'exit':
        controller.close(10.02)
    else:
        with pytest.raises(FeedbackError):
            if cause == 'invalid_key':
                controller.key_event('unexpected', True, 10.02)
            else:
                controller.tick(9 if cause == 'time_reverse' else float('nan'))
    assert controller.target == (0, 0) and controller.backend.commands[-1] == (0, 0)


@pytest.mark.parametrize('change', [{'hardware_enabled': True}, {'linear_speed_m_s': .16},
    {'angular_speed_rad_s': .31}, {'key_timeout_s': 10}, {'link_timeout_s': float('inf')}])
def test_invalid_manual_limits_or_hardware_flag_fail_before_any_intent(config, change):
    with pytest.raises(FeedbackError):
        ManualController({**config, **change})


def test_real_wasd_cannot_be_enabled_by_configuration_or_cli(config, tmp_path):
    with pytest.raises(FeedbackError, match=REAL_BLOCK_REASON):
        ManualController(config, mode='real')
    output = tmp_path/'must-not-exist.jsonl'
    with pytest.raises(FeedbackError, match=REAL_BLOCK_REASON):
        main(['--config', str(CONFIG_PATH), '--mode', 'real', '--output', str(output)])
    assert not output.exists()
