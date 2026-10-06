"""Experiment lifecycle never relaxes the formal dual-source gate."""
import pytest

from wc_runtime.single_mapping import make_plan


def test_single_plan_owns_only_requested_sensor_and_native_pipeline(tmp_path):
    plan = make_plan('single_contract', 'right', 80, 8770, tmp_path)
    assert plan['experiment']['source_mode'] == 'real'
    assert plan['experiment']['sensor_mode'] == 'single_right'
    assert plan['experiment']['formal_acceptance'] is False
    assert plan['experiment']['time_model_validated'] is False
    args = [part for command in plan['commands'] for part in command]
    assert 'source_mode:=single_right' in args
    assert 'device_config_policy:=preserve_current' in args
    assert 'max_runtime_seconds:=95' in args
    assert 'wc_runtime.single_mapping_input' in args
    assert 'wc_runtime.single_mapping_monitor' in args
    assert any(x.endswith('/single_mapping.launch.py') for x in args)
    assert not any(x in ('wc_imu.ros_node', 'wc_motion.feedback_transport', 'wc_motion.manual_control') for x in args)
    assert not any('lidar_left/' in topic for topic in plan['experiment']['bag_topics'])
    assert '/tf' not in plan['experiment']['bag_topics']
    assert '/wc_mapping/single_right/tf' in plan['experiment']['bag_topics']
    assert {'sensor_owner.lock', 'domain-83-source.lock', 'domain-83-processing.lock'} <= set(plan['locks'])
    assert plan['sigint_grace_s'] == 30


@pytest.mark.parametrize('side,duration,port', [('dual', 80, 8770), ('right', 0, 8770),
    ('right', 1801, 8770), ('right', 80, 80), ('right', True, 8770)])
def test_reject_unbounded_or_unsupported_session(side, duration, port, tmp_path):
    with pytest.raises(ValueError):
        make_plan('single_contract', side, duration, port, tmp_path)
