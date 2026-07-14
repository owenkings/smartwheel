import math

import pytest

from wheel_odom_driver.kinematics import DifferentialOdometry, EncoderConfig, unwrap_delta


def config(**overrides):
    values = dict(
        wheel_radius_m=0.1,
        track_width_m=0.5,
        encoder_cpr=1000,
        gear_ratio=1.0,
        left_sign=1,
        right_sign=1,
        encoder_bits=16,
    )
    values.update(overrides)
    return EncoderConfig(**values)


def test_equal_counts_integrate_straight_motion():
    odom = DifferentialOdometry(config())
    assert odom.update(0, 0, 1.0) is None
    update = odom.update(1000, 1000, 2.0)
    assert update.x == pytest.approx(2.0 * math.pi * 0.1)
    assert update.y == pytest.approx(0.0)
    assert update.yaw == pytest.approx(0.0)
    assert update.linear_mps == pytest.approx(update.x)


def test_opposite_counts_rotate_in_place():
    odom = DifferentialOdometry(config())
    odom.update(0, 0, 1.0)
    update = odom.update(-100, 100, 2.0)
    assert update.x == pytest.approx(0.0)
    assert update.angular_rps > 0.0


def test_encoder_overflow_in_both_directions():
    assert unwrap_delta(2, 65534, 16) == 4
    assert unwrap_delta(65534, 2, 16) == -4


def test_left_and_right_signs_are_applied():
    odom = DifferentialOdometry(config(left_sign=-1, right_sign=1))
    odom.update(0, 0, 1.0)
    update = odom.update(-100, 100, 2.0)
    assert update.distance_left_m > 0.0
    assert update.distance_right_m > 0.0
    assert update.angular_rps == pytest.approx(0.0)


def test_non_monotonic_encoder_time_is_rejected():
    odom = DifferentialOdometry(config())
    odom.update(0, 0, 1.0)
    with pytest.raises(ValueError, match="strictly increasing"):
        odom.update(1, 1, 1.0)

