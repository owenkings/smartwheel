import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wheelchair_navigation.semantic_keepout import (  # noqa: E402
    GridGeometry,
    rasterize_keepout_zones,
    world_to_grid,
)
from wheelchair_navigation.semantic_keepout_node import validate_no_go_zones  # noqa: E402


def test_rasterize_square_keepout_zone():
    geometry = GridGeometry(5, 5, 1.0, 0.0, 0.0)
    zones = [{"name": "stairs", "polygon": [[1, 1], [3, 1], [3, 3], [1, 3]]}]

    data = rasterize_keepout_zones(zones, geometry)

    occupied = {
        (column, row)
        for row in range(5)
        for column in range(5)
        if data[row * 5 + column] == 100
    }
    assert occupied == {(1, 1), (2, 1), (1, 2), (2, 2)}


def test_rasterizer_clips_zone_to_grid():
    geometry = GridGeometry(2, 2, 1.0, 0.0, 0.0)
    zones = [{"polygon": [[-2, -2], [1, -2], [1, 1], [-2, 1]]}]

    data = rasterize_keepout_zones(zones, geometry)

    assert data == [100, 0, 0, 0]


def test_world_to_grid_supports_rotated_origin():
    geometry = GridGeometry(10, 10, 1.0, 2.0, 3.0, math.pi / 2.0)

    grid_x, grid_y = world_to_grid(2.0, 4.0, geometry)

    assert math.isclose(grid_x, 1.0, abs_tol=1e-9)
    assert math.isclose(grid_y, 0.0, abs_tol=1e-9)


def test_validate_no_go_zones_rejects_malformed_polygon():
    with pytest.raises(ValueError, match="at least 3 points"):
        validate_no_go_zones(
            {"no_go_zones": [{"name": "broken", "polygon": [[0, 0], [1, 0]]}]}
        )


def test_validate_no_go_zones_accepts_empty_configuration():
    assert validate_no_go_zones({"no_go_zones": []}) == []
