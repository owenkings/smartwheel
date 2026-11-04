import hashlib
import json
import math

import numpy as np
import pytest

from wc_navigation.geometry import Grid
from wc_navigation.map_guard import protect_observations, write_guarded_map


def test_walls_protect_full_observation_square_and_preserve_input():
    data = np.zeros((8, 8), dtype=np.int8)
    data[3, 3] = -1
    data[4, 4] = 100
    grid = Grid(data, .05)
    original = grid.data.tobytes()
    guarded, r = protect_observations(grid, [[.2, .2]], [], .1)
    assert np.all(guarded.data[3:5, 3:5] == 100)
    assert guarded.data[2, 3] == 0 and guarded.data[5, 3] == 0
    assert r['wall_observations']['unique_covered_cells'] == 4
    assert r['changes']['wall_unknown_to_occupied'] == 1
    assert r['changes']['wall_free_to_occupied'] == 2
    assert r['changes']['wall_existing_occupied'] == 1
    assert all(r['invariants'].values())
    assert grid.data.tobytes() == original
    assert not guarded.data.flags.writeable


def test_low_band_only_demotes_free_and_wall_support_wins():
    data = np.zeros((8, 8), dtype=np.int8)
    data[3, 3] = 100
    data[4, 4] = -1
    original = Grid(data, .1)
    low = [[.4, .4]]
    guarded, r = protect_observations(original, [], low, .2)
    assert guarded.data[3, 3] == 100
    assert guarded.data[4, 4] == -1
    assert guarded.data[3, 4] == guarded.data[4, 3] == -1
    assert r['changes']['low_free_to_unknown'] == 2
    both, report = protect_observations(original, low, low, .2)
    assert np.all(both.data[3:5, 3:5] == 100)
    assert report['changes']['low_free_to_unknown'] == 0
    assert report['policy']['overlap'] == 'WALL_WINS'


def test_unknown_and_occupied_never_become_free():
    data = np.full((10, 10), -1, dtype=np.int8)
    data[2, :] = 100
    data[3:8, 3:8] = 0
    grid = Grid(data, .1, (-.25, .13, 0.))
    walls = [grid.cell_to_world((4, 4)), grid.cell_to_world((5, 6))]
    low = [grid.cell_to_world((3, 3)), grid.cell_to_world((4, 4))]
    guarded, r = protect_observations(grid, walls, low)
    assert np.all(guarded.data[grid.data == 100] == 100)
    assert np.all(guarded.data[grid.data != 0] != 0)
    assert np.count_nonzero(guarded.data == 0) <= np.count_nonzero(grid.data == 0)
    json.dumps(r, allow_nan=False)


def test_world_voxels_respect_origin_and_partial_clipping():
    grid = Grid(np.zeros((5, 5), dtype=np.int8), .1, (10., -2., 0.))
    guarded, r = protect_observations(grid, [[10.01, -1.99], [20., 20.]], [], .1)
    assert guarded.data[0, 0] == 100
    assert np.count_nonzero(guarded.data == 100) == 1
    assert r['wall_observations']['partially_clipped_voxels'] == 1
    assert r['wall_observations']['outside_voxels'] == 1


def test_duplicate_observations_do_not_increase_unique_protection():
    grid = Grid(np.zeros((10, 10), dtype=np.int8), .1)
    one, a = protect_observations(grid, [[.5, .5]], [])
    many, b = protect_observations(grid, [[.5, .5]] * 3, [])
    assert np.array_equal(one.data, many.data)
    assert a['changes'] == b['changes']
    assert b['wall_observations']['input_voxels'] == 3
    assert b['support_verified_here'] is False


@pytest.mark.parametrize('points', [None, [[float('nan'), 0]], [[float('inf'), 0]],
                                  [[0, 1, 2]], [0, 1], [['1', '2']], np.empty((0, 3))])
def test_invalid_points_fail_closed(points):
    grid = Grid(np.zeros((10, 10), dtype=np.int8), .1)
    with pytest.raises(ValueError):
        protect_observations(grid, points, [])


@pytest.mark.parametrize('voxel', [0, -.1, float('nan'), float('inf'), .001, 2., True])
def test_invalid_voxel_size_rejected(voxel):
    with pytest.raises(ValueError):
        protect_observations(Grid(np.zeros((10, 10), dtype=np.int8), .1), [], [], voxel)


def test_rotated_map_is_explicitly_unsupported():
    grid = Grid(np.zeros((10, 10), dtype=np.int8), .1, (0., 0., .2))
    with pytest.raises(ValueError, match='axis-aligned'):
        protect_observations(grid, [], [])


def test_voxel_projection_overflow_rejected():
    grid = Grid(np.zeros((10, 10), dtype=np.int8), 1e-100)
    with pytest.raises(ValueError, match='overflow'):
        protect_observations(grid, [[1e308, 1e308]], [])


def test_resource_bound_rejected_before_allocating_guard_masks():
    grid = Grid(np.zeros((2001, 2500), dtype=np.int8), .1)
    with pytest.raises(ValueError, match='five-million-cell'):
        protect_observations(grid, [], [])


def test_export_roundtrips_ros_bottom_up_occupancy_and_exact_metadata(tmp_path):
    from wc_runtime.map_viewer import inspect_grid
    grid = Grid(np.asarray([[-1, 0, 100], [100, -1, 0]], dtype=np.int8),
                .05, (-7.82513, -5.40073, 0.))
    folder = tmp_path / 'guarded'
    metadata = {'source_map_sha256': 'a' * 64, 'support_bins': 3, 'guard': 'recorded multi-view evidence'}
    paths = write_guarded_map(grid, folder, metadata)
    stats, data, spec = inspect_grid(folder / 'map.yaml')
    assert np.array_equal(data, grid.data)
    assert spec['origin'] == list(grid.origin)
    assert spec['resolution'] == grid.resolution
    provenance = json.loads((folder / 'provenance.json').read_text(encoding='utf-8'))
    assert provenance['source_metadata'] == metadata
    assert provenance['navigation_validated'] is False
    for name in ['map.pgm', 'map.yaml', 'provenance.json']:
        assert paths['file_sha256'][name] == hashlib.sha256((folder / name).read_bytes()).hexdigest()
    with pytest.raises(FileExistsError):
        write_guarded_map(grid, folder, metadata)


def test_invalid_provenance_does_not_create_output(tmp_path):
    grid = Grid(np.zeros((10, 10), dtype=np.int8), .1)
    with pytest.raises(ValueError):
        write_guarded_map(grid, tmp_path / 'bad', {'invalid': float('nan')})
    assert not (tmp_path / 'bad').exists()
