import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wheelchair_sensors.xtm60_adapter_node import XTM60AdapterNode, extract_xyzi_points  # noqa: E402


class FakePoint:
    def __init__(self, x, y, z, i=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.i = i


class FakeFrame:
    hasPointcloud = True

    def __init__(self):
        self.points = [
            FakePoint(1.0, 0.0, 0.0),
            FakePoint(float("nan"), 0.0, 0.0),
            FakePoint(30.0, 0.0, 0.0),
            FakePoint(0.1, 0.2, 0.3, 8.0),
        ]
        self.amplData = [100, 200, 300, 400]


def test_extract_xyzi_points_filters_invalid_and_keeps_amplitude():
    points = extract_xyzi_points(FakeFrame(), unit_scale=1.0, range_min=0.05, range_max=20.0)

    assert len(points) == 2
    assert points[0] == (1.0, 0.0, 0.0, 100.0)
    assert points[1] == (0.1, 0.2, 0.3, 400.0)
    assert math.isfinite(points[1][2])


def test_xtm60_sdk_runtime_seconds_are_not_epoch_timestamps():
    assert not XTM60AdapterNode._is_plausible_epoch_stamp((1006, 792000000))
    assert XTM60AdapterNode._is_plausible_epoch_stamp((1779897171, 676858200))
