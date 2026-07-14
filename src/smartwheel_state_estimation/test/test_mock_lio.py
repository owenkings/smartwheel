import pytest

from smartwheel_state_estimation.mock_lio_node import CumulativeMotionDecoder, MockLioIntegrator


def model(seed=7, scale=1.0, bias=0.0):
    return MockLioIntegrator(seed, scale, 1.0, bias, 0.0, 0.0, 0.0)


def test_mock_lio_integrates_motion_without_absolute_pose_input():
    integrator = model()
    integrator.update(1.0, 0.0, 0.0)
    pose = integrator.update(2.0, 1.0, 0.0)
    assert pose.x == pytest.approx(1.0)
    assert pose.y == pytest.approx(0.0)


def test_mock_lio_scale_and_bias_create_measurable_drift():
    integrator = model(scale=1.02, bias=0.01)
    integrator.update(1.0, 0.0, 0.0)
    pose = integrator.update(3.0, 1.0, 0.0)
    assert pose.x == pytest.approx(2.06)


def test_mock_lio_rejects_regressing_timestamps():
    integrator = model()
    integrator.update(2.0, 0.0, 0.0)
    with pytest.raises(ValueError, match="strictly increasing"):
        integrator.update(1.0, 0.0, 0.0)


def test_cumulative_motion_recovers_distance_across_dropped_sequences():
    decoder = CumulativeMotionDecoder()
    assert decoder.update(10, 1.0, 2.0, 0.5) is None
    rates = decoder.update(13, 2.0, 5.0, 1.1)
    assert rates.linear_mps == pytest.approx(3.0)
    assert rates.angular_rps == pytest.approx(0.6)


def test_cumulative_motion_rejects_replayed_sequence():
    decoder = CumulativeMotionDecoder()
    decoder.update(3, 1.0, 0.0, 0.0)
    with pytest.raises(ValueError, match="sequence"):
        decoder.update(3, 2.0, 1.0, 0.0)
