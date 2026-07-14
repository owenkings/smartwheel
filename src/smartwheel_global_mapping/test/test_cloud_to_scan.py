import math

import numpy as np

from smartwheel_global_mapping.cloud_to_scan_node import points_to_ranges


def test_cloud_to_scan_keeps_nearest_valid_return_per_angle():
    points = np.array(
        [
            [2.0, 0.0, 0.5],
            [1.0, 0.0, 0.5],
            [0.0, 1.0, 0.5],
            [-1.0, 0.0, 2.5],
            [float("nan"), 0.0, 0.5],
        ]
    )
    increment = math.pi / 2.0
    ranges = points_to_ranges(points, -math.pi, math.pi, increment, 0.1, 10.0, 0.1, 1.8)
    assert ranges.shape == (4,)
    assert ranges[2] == 1.0
    assert ranges[3] == 1.0
    assert math.isinf(float(ranges[0]))
