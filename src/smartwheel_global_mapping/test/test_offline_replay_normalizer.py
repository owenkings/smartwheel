import math

from smartwheel_global_mapping.offline_replay_normalizer_node import (
    PoseSample,
    interpolate_pose,
)


def test_interpolate_pose_midpoint():
    samples = [
        PoseSample(10.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        PoseSample(10.2, (2.0, 4.0, 6.0), (0.0, 0.0, 1.0, 0.0)),
    ]
    result = interpolate_pose(samples, 10.1, 0.25)
    assert result is not None
    assert result.stamp == 10.1
    assert result.position == (1.0, 2.0, 3.0)
    assert math.isclose(sum(value * value for value in result.orientation), 1.0)


def test_interpolate_pose_accepts_nearby_endpoint():
    samples = [PoseSample(4.0, (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))]
    result = interpolate_pose(samples, 4.2, 0.25)
    assert result == samples[0]


def test_interpolate_pose_rejects_large_gap():
    samples = [
        PoseSample(1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        PoseSample(2.0, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    ]
    assert interpolate_pose(samples, 1.5, 0.25) is None
