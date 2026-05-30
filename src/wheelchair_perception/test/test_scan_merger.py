import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wheelchair_perception.scan_merger_node import MergeConfig, ScanSlice, merge_scan_slices  # noqa: E402


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
