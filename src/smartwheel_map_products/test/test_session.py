import numpy as np
import pytest

from smartwheel_map_products.session import (
    SessionCapture,
    SessionCaptureError,
    SessionStateError,
)


def _frame(stamp, x=0.0, intensity=None):
    points = np.array([[x, 0.0, 0.5], [x + 0.01, 0.0, 0.5]], dtype=np.float64)
    origins = np.zeros_like(points)
    return stamp, "camera_init", points, origins, intensity


def test_session_keeps_early_middle_and_late_frame_metadata():
    session = SessionCapture(0.05, 20, expected_frame_id="camera_init")
    session.start()
    for frame in (
        _frame(10.0, 0.0),
        _frame(15.0, 1.0),
        _frame(22.5, 2.0),
    ):
        session.add_frame(*frame)
    session.stop()

    summary = session.summary()
    assert summary.complete is True
    assert summary.state == SessionCapture.STOPPED
    assert summary.frame_count == 3
    assert summary.raw_point_count == 6
    assert summary.first_cloud_stamp == 10.0
    assert summary.last_cloud_stamp == 22.5
    assert summary.cloud_time_span_sec == 12.5
    assert summary.frame_ids == ("camera_init",)


def test_session_rejects_frame_change_and_timestamp_regression():
    session = SessionCapture(0.05, 20)
    session.start()
    session.add_frame(*_frame(2.0))
    with pytest.raises(SessionCaptureError, match="frame_id changed"):
        session.add_frame(3.0, "map", *_frame(3.0)[2:])
    assert session.state == SessionCapture.FAILED

    session = SessionCapture(0.05, 20)
    session.start()
    session.add_frame(*_frame(2.0))
    with pytest.raises(SessionCaptureError, match="not monotonic"):
        session.add_frame(*_frame(1.0))
    assert session.state == SessionCapture.FAILED


def test_session_overflow_fails_without_partial_mutation():
    session = SessionCapture(0.1, 1, expected_frame_id="camera_init")
    session.start()
    with pytest.raises(SessionCaptureError, match="configured limit 1"):
        session.add_frame(
            1.0,
            "camera_init",
            np.array([[0.0, 0.0, 0.0], [0.2, 0.0, 0.0]]),
            np.zeros((2, 3)),
        )
    assert len(session.accumulator) == 0
    assert session.summary().complete is False
    assert session.summary().failure_reason


def test_session_requires_start_and_does_not_accept_frames_after_stop():
    session = SessionCapture(0.1, 10)
    with pytest.raises(SessionStateError, match="cannot add frame"):
        session.add_frame(*_frame(1.0))
    session.start()
    session.stop()
    with pytest.raises(SessionStateError, match="cannot add frame"):
        session.add_frame(*_frame(2.0))
