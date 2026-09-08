import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wheelchair_perception.scan_merger_node import (  # noqa: E402
    MergeConfig,
    ScanSlice,
    merge_scan_slices,
    newest_nonzero_stamp,
)


def test_merge_scan_slices_keeps_nearest_range():
    config = MergeConfig(angle_min=-1.0, angle_max=1.0, angle_increment=1.0, range_min=0.1, range_max=5.0)
    left = ScanSlice(angle_min=-1.0, angle_increment=1.0, range_min=0.1, range_max=5.0, ranges=[2.0, 4.0, math.inf])
    right = ScanSlice(angle_min=-1.0, angle_increment=1.0, range_min=0.1, range_max=5.0, ranges=[3.0, 1.5, 2.5])

    assert merge_scan_slices([left, right], config) == pytest.approx([2.0, 1.5, 2.5])


def test_merge_scan_slices_filters_invalid_and_out_of_range_values():
    config = MergeConfig(angle_min=0.0, angle_max=2.0, angle_increment=1.0, range_min=0.2, range_max=3.0)
    scan = ScanSlice(angle_min=0.0, angle_increment=1.0, range_min=0.1, range_max=5.0, ranges=[0.1, 3.5, 2.0])

    merged = merge_scan_slices([scan], config)

    assert math.isinf(merged[0])
    assert math.isinf(merged[1])
    assert merged[2] == pytest.approx(2.0)


def test_nonintegral_span_never_creates_a_beam_past_angle_max():
    config = MergeConfig(angle_min=-1.5708, angle_max=1.5708, angle_increment=0.0087)

    assert config.beam_count == 362
    assert config.realized_angle_max <= config.angle_max
    assert config.realized_angle_max + config.angle_increment > config.angle_max

class _Stamp:
    def __init__(self, sec, nanosec):
        self.sec = sec
        self.nanosec = nanosec


class _Message:
    def __init__(self, sec, nanosec):
        self.header = type("Header", (), {})()
        self.header.stamp = _Stamp(sec, nanosec)


def test_newest_nonzero_stamp_uses_capture_stamp_not_timer_time():
    messages = [_Message(10, 900), _Message(11, 100)]

    stamp = newest_nonzero_stamp(messages)

    assert (stamp.sec, stamp.nanosec) == (11, 100)


def test_newest_nonzero_stamp_ignores_zero_and_malformed_stamps():
    messages = [_Message(0, 0), _Message(-1, 0), _Message(12, 1_000_000_000)]

    assert newest_nonzero_stamp(messages) is None
