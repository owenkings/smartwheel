import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wheelchair_base.kinematics import DifferentialDriveModel, OdometryState  # noqa: E402
from wheelchair_base.modbus_rtu import crc16, from_i16, with_crc  # noqa: E402


def test_twist_to_wheel_rpm_roundtrip():
    model = DifferentialDriveModel(wheel_radius_m=0.1, wheel_separation_m=0.5)
    left, right = model.twist_to_wheel_rpm(0.2, 0.4)
    linear, angular = model.wheel_rpm_to_twist(left, right)

    assert linear == pytest.approx(0.2)
    assert angular == pytest.approx(0.4)


def test_odometry_integrates_forward_motion():
    odom = OdometryState()
    odom.integrate(0.5, 0.0, 2.0)

    assert odom.x == pytest.approx(1.0)
    assert odom.y == pytest.approx(0.0)


def test_modbus_crc_and_signed_register_decode():
    assert with_crc(bytes.fromhex("01 03 00 01 00 01")) == bytes.fromhex("01 03 00 01 00 01 d5 ca")
    assert crc16(bytes.fromhex("01 03 00 01 00 01")) == 0xCAD5
    assert from_i16(0xFFFE) == -2
