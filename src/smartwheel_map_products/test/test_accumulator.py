import numpy as np
import pytest

from smartwheel_map_products.accumulator import VoxelAccumulator


def test_accumulator_deduplicates_repeated_voxels():
    accumulator = VoxelAccumulator(0.1, 10)
    points = np.array([[0.01, 0.01, 0.01], [0.09, 0.09, 0.09], [0.11, 0.0, 0.0]])
    origins = np.zeros_like(points)

    assert accumulator.add(points, origins) == 2
    assert accumulator.add(points, origins) == 0
    accumulated_points, accumulated_origins = accumulator.arrays()
    assert accumulated_points.shape == (2, 3)
    assert accumulated_origins.shape == (2, 3)


def test_accumulator_fails_before_exceeding_configured_limit():
    accumulator = VoxelAccumulator(0.1, 1)
    with pytest.raises(OverflowError, match="configured limit 1"):
        accumulator.add(np.array([[0.0, 0.0, 0.0], [0.2, 0.0, 0.0]]), np.zeros((2, 3)))
    assert len(accumulator) == 0


def test_accumulator_rejects_invalid_shapes_and_values():
    accumulator = VoxelAccumulator(0.1, 10)
    with pytest.raises(ValueError, match="matching Nx3"):
        accumulator.add(np.zeros((2, 2)), np.zeros((2, 2)))
    with pytest.raises(ValueError, match="finite"):
        accumulator.add(np.array([[np.nan, 0.0, 0.0]]), np.zeros((1, 3)))
