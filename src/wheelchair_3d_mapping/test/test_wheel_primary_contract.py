from __future__ import annotations

import ast
import struct
from pathlib import Path

import numpy as np
import pytest
import yaml

from wheelchair_3d_mapping.wheel_primary_cloud_math import transform_xyz_buffer
from wheelchair_3d_mapping.wheel_primary_health import HealthLimits, HealthSample, evaluate_health
from wheelchair_3d_mapping.wheel_primary_pipeline import _validate_inputs
from wheelchair_3d_mapping.wheel_primary_retry_queue import (
    BoundedRetryQueue,
    RetryDecision,
)


ROOT = Path(__file__).resolve().parents[3]
HERE = ROOT / 'src/wheelchair_3d_mapping/wheelchair_3d_mapping'


def _layout():
    return {
        "extrinsic_R": [1.0] * 9,
        "extrinsic_T": [0.0] * 3,
        "base_to_imu_R": [1.0] * 9,
        "base_to_imu_T": [0.0] * 3,
    }


def test_helper_rejects_unvalidated_layout_dict():
    layout = _layout()
    del layout["base_to_imu_R"]
    with pytest.raises(ValueError, match="load_diagnostic_layout"):
        _validate_inputs("bringup", "mapping", layout, "/tmp/session.db")


def test_ekf_is_planar_wheel_vx_plus_gyro_yaw_rate_only():
    config = yaml.safe_load((ROOT / 'src/wheelchair_bringup/config/right_diag_wheel_imu_ekf.yaml').read_text(encoding="utf-8"))
    params = config["wheel_imu_ekf"]["ros__parameters"]
    assert params["two_d_mode"] is True
    assert params["publish_tf"] is False
    assert [i for i, enabled in enumerate(params["odom0_config"]) if enabled] == [6]
    assert [i for i, enabled in enumerate(params["imu0_config"]) if enabled] == [11]


def test_helper_uses_external_gated_odom_and_native_cloud_not_lio_cloud():
    source = (HERE / "wheel_primary_pipeline.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    assert '"odom_mode": "external"' in source
    assert '"odom_topic": GATED_ODOM_TOPIC' in source
    assert '"points_topic": RAW_CLOUD_TOPIC' in source
    assert '"enable_loop_closure": "true"' in source
    assert "/cloud_registered" not in source.replace("/wheel_mapping/cloud_registered", "")
    assert "formal_3d_mapping.launch.py" not in source
    assert "GroupAction(" in source and "scoped=True" in source
    assert any(
        isinstance(node, ast.FunctionDef) and node.name == "build_wheel_primary_nodes"
        for node in ast.walk(tree)
    )


def test_transform_preserves_intensity_and_row_padding_exactly():
    point_step = 20
    row_step = 44  # two points plus four padding bytes
    raw = bytearray(row_step)
    struct.pack_into("<ffff", raw, 0, 1.0, 2.0, 3.0, 101.0)
    struct.pack_into("<ffff", raw, point_step, -1.0, 0.5, 4.0, 202.0)
    raw[40:44] = b"PAD!"
    transformed = transform_xyz_buffer(
        raw,
        width=2,
        height=1,
        point_step=point_step,
        row_step=row_step,
        x_offset=0,
        y_offset=4,
        z_offset=8,
        bigendian=False,
        rotation=np.eye(3),
        translation=(10.0, -2.0, 0.25),
    )
    assert struct.unpack_from("<fff", transformed, 0) == pytest.approx((11.0, 0.0, 3.25))
    assert struct.unpack_from("<fff", transformed, point_step) == pytest.approx((9.0, -1.5, 4.25))
    assert struct.unpack_from("<f", transformed, 12)[0] == pytest.approx(101.0)
    assert struct.unpack_from("<f", transformed, point_step + 12)[0] == pytest.approx(202.0)
    assert transformed[40:44] == b"PAD!"


def test_transform_rejects_overlapping_xyz_fields():
    with pytest.raises(ValueError, match="overlap"):
        transform_xyz_buffer(
            bytes(16), width=1, height=1, point_step=16, row_step=16,
            x_offset=0, y_offset=2, z_offset=8, bigendian=False,
            rotation=np.eye(3), translation=(0.0, 0.0, 0.0),
        )


def test_live_cloud_retries_until_exact_time_tf_becomes_available():
    queue = BoundedRetryQueue(max_items=10, max_age_sec=0.25)
    queue.push("same-stamp-cloud", 1.0)
    transform_available = False
    published = []

    def attempt(item):
        if not transform_available:
            return RetryDecision.RETRY, None
        return RetryDecision.READY, item

    first = queue.process(
        now=1.01, healthy=lambda: True, attempt=attempt, publish=published.append
    )
    assert first.waiting == 1
    assert published == []
    transform_available = True
    second = queue.process(
        now=1.03, healthy=lambda: True, attempt=attempt, publish=published.append
    )
    assert second.published == 1
    assert published == ["same-stamp-cloud"]
    assert len(queue) == 0


def test_live_cloud_retry_expires_and_queue_is_count_bounded():
    queue = BoundedRetryQueue(max_items=2, max_age_sec=0.25)
    assert queue.push("oldest", 1.0) == 0
    assert queue.push("middle", 1.01) == 0
    assert queue.push("newest", 1.02) == 1
    published = []
    ready = lambda item: (RetryDecision.READY, item)
    result = queue.process(
        now=1.27, healthy=lambda: True, attempt=ready, publish=published.append
    )
    assert result.expired == 1
    assert published == ["newest"]


def test_live_cloud_retry_clears_on_stale_health_and_checks_before_publish():
    queue = BoundedRetryQueue(max_items=10, max_age_sec=0.25)
    queue.push("cloud", 1.0)
    result = queue.process(
        now=1.01,
        healthy=lambda: False,
        attempt=lambda item: (RetryDecision.READY, item),
        publish=lambda _item: pytest.fail("stale cloud must not publish"),
    )
    assert result.health_cleared == 1
    assert len(queue) == 0

    queue.push("cloud", 2.0)
    checks = iter((True, True, False))
    result = queue.process(
        now=2.01,
        healthy=lambda: next(checks),
        attempt=lambda item: (RetryDecision.READY, item),
        publish=lambda _item: pytest.fail("health changed before publish"),
    )
    assert result.health_cleared == 1
    assert len(queue) == 0


def test_live_cloud_wrapper_forbids_latest_tf_fallback_and_uses_gated_default():
    source = (HERE / "wheel_registered_cloud_node.py").read_text(encoding="utf-8")
    assert '"gated_odom_topic": "/wheel_mapping/odometry_gated"' in source
    assert "stamp.sec == 0 and stamp.nanosec == 0" in source
    assert "Time.from_msg(message.header.stamp)" in source
    assert '"retry_queue_max_clouds": 10' in source
    assert '"retry_queue_max_age_sec": 0.25' in source
    assert '"retry_period_sec": 0.02' in source


def _healthy_sample(**overrides):
    values = dict(
        feedback_ok=True,
        feedback_receive_age=0.01,
        wheel_receive_age=0.01,
        imu_receive_age=0.01,
        wheel_header_age=0.01,
        imu_header_age=0.01,
        ekf_header_age=0.01,
        wheel_valid=True,
        imu_valid=True,
        ekf_valid=True,
    )
    values.update(overrides)
    return HealthSample(**values)


def test_health_policy_suppresses_stale_wheel_pose_and_thus_tf():
    limits = HealthLimits()
    assert evaluate_health(_healthy_sample(), limits) == "OK"
    assert evaluate_health(
        _healthy_sample(wheel_header_age=limits.wheel_header_age + 0.001), limits
    ) == "BLOCKED_WHEEL_HEADER_STALE"
    # The ROS wrapper calls sendTransform only after this result equals OK.
    source = (HERE / "wheel_pose_health_gate_node.py").read_text(encoding="utf-8")
    guard = source.index('if state != "OK"')
    early_return = source.index("return", guard)
    broadcast = source.index("self._tf.sendTransform", early_return)
    assert guard < early_return < broadcast


def test_health_policy_rejects_missing_or_future_stamps():
    limits = HealthLimits()
    assert evaluate_health(
        _healthy_sample(imu_header_age=float("inf")), limits
    ) == "BLOCKED_MISSING_OR_NONFINITE_TIME"
    assert evaluate_health(
        _healthy_sample(ekf_header_age=-(limits.future_tolerance + 0.001)), limits
    ) == "BLOCKED_EKF_FROM_FUTURE"
