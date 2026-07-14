import numpy as np
import pytest

from xtm60_ros2_driver.backends import MockXtm60Backend, create_backend


def test_mock_lidar_is_deterministic_for_seed():
    first = MockXtm60Backend(seed=20260714).sample(1.0)
    second = MockXtm60Backend(seed=20260714).sample(1.0)
    np.testing.assert_allclose(first, second)
    assert first.shape[1] == 3


def test_real_lidar_adapter_is_not_available_in_stage_a():
    with pytest.raises(RuntimeError, match="hardware_enabled=false"):
        create_backend("real", hardware_enabled=False)
    with pytest.raises(NotImplementedError, match="NOT_IMPLEMENTED"):
        create_backend("real", hardware_enabled=True)
