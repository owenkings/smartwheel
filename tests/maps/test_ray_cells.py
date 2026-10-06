"""Synthetic oracle comparisons for scalar ray tracing and global voxel bounds.

The oracle below is the pre-optimization NumPy implementation, preserved here
verbatim in algorithm and operation order. It is not used by production code.
"""
import copy
import math
import unittest
from unittest.mock import patch

import numpy as np

import wc_maps.builder as builder
from wc_maps.builder import MapError


def numpy_ray_oracle(origin, endpoint, resolution, max_cells):
    cell = np.floor(origin / resolution).astype(np.int64)
    finish = np.floor(endpoint / resolution).astype(np.int64)
    count_bound = int(np.abs(finish - cell).sum()) + 1
    if count_bound > max_cells:
        raise MapError("ray exceeds max_ray_cells resource bound")
    direction = endpoint - origin
    step = np.sign(direction).astype(np.int64)
    delta = np.full(3, np.inf)
    crossing = np.full(3, np.inf)
    moving = direction != 0
    delta[moving] = resolution / np.abs(direction[moving])
    boundary = (cell + (step > 0)) * resolution
    crossing[moving] = (boundary[moving] - origin[moving]) / direction[moving]
    cells = []
    for _ in range(count_bound):
        if np.array_equal(cell, finish):
            return cells
        next_time = float(crossing.min())
        axes = np.isclose(crossing, next_time, atol=1e-12, rtol=0)
        cell[axes] += step[axes]
        crossing[axes] += delta[axes]
        if not np.array_equal(cell, finish):
            cells.append(tuple(int(v) for v in cell))
    raise MapError("ray traversal did not reach its endpoint")


def outcome(function, origin, endpoint, resolution=1.0, max_cells=10000):
    try:
        return "cells", function(np.asarray(origin, dtype=float), np.asarray(endpoint, dtype=float), resolution, max_cells)
    except MapError as error:
        return "error", str(error)


def random_rays(count, seed=40911):
    generator = np.random.default_rng(seed)
    rays = []
    for index in range(count):
        resolution = [0.1, 0.125, 0.5, 1.0, 2.0][index % 5]
        origin = generator.uniform(-4, 4, 3)
        endpoint = origin + generator.uniform(-3, 3, 3)
        if index % 4 == 0:
            origin[index % 3] = generator.integers(-5, 5) * resolution
        if index % 7 == 0:
            endpoint[(index + 1) % 3] = generator.integers(-5, 5) * resolution
        if index % 11 == 0:
            endpoint[index % 3] = origin[index % 3]
        rays.append((origin, endpoint, resolution, 10000))
    return rays


def map_fixture():
    config = dict(resolution=.25, ground_z=-1.0, ground_tolerance=.02,
                  collision_min_height=.1, collision_max_height=1.5,
                  ground_valid=True, sensor_mode="dual", max_voxels=100000)
    graph = dict(session_id="SCALAR_DDA_SYNTHETIC", graph_epoch="synthetic_graph", revision=1, nodes=[], edges=[])
    observations = []
    generator = np.random.default_rng(17319)
    for index in (1, 2):
        pose = np.eye(4)
        if index == 2:
            angle = .37
            pose[:2, :2] = [[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]]
            pose[:3, 3] = [1.25, -2.5, .25]
        graph["nodes"].append(dict(node_id=index, odom_epoch="fixture_odom", T_map_node=pose.tolist()))
        for side, y in (("left", .5), ("right", -.5)):
            sensor = np.eye(4)
            sensor[:3, 3] = [.25, y, .75]
            points = generator.uniform([.25, -2, -1.5], [3, 2, 1], (35, 3)).tolist()
            observations.append(dict(raw_key=side + str(index), sensor_id="fixture_" + side, side=side,
                node_id=index, odom_epoch="fixture_odom", points=points, T_node_sensor=sensor.tolist()))
    graph["edges"] = [dict(from_id=1, to_id=2, type="neighbor")]
    return graph, {"observations": observations}, config


class ScalarRayTests(unittest.TestCase):
    def assert_matches(self, origin, endpoint, resolution=1.0, max_cells=10000):
        expected = outcome(numpy_ray_oracle, origin, endpoint, resolution, max_cells)
        actual = outcome(builder._ray_cells, origin, endpoint, resolution, max_cells)
        self.assertEqual(actual, expected)
        if actual[0] == "cells":
            self.assertNotIn(tuple(np.floor(np.asarray(origin) / resolution).astype(int)), actual[1])
            self.assertNotIn(tuple(np.floor(np.asarray(endpoint) / resolution).astype(int)), actual[1])

    def test_seeded_random_rays_match_original_numpy_oracle(self):
        for index, ray in enumerate(random_rays(1200)):
            with self.subTest(index=index):
                self.assert_matches(*ray)

    def test_negative_axis_directions_exact_boundaries_and_signed_zero(self):
        cases = [([0, .5, .5], [-3, .5, .5]), ([-.5, -.5, -.5], [-3.5, -3.5, -3.5]),
                 ([-1, 0, 0], [1, 0, 0]), ([-0.0, 0, 1.25], [-0.0, 0, 3.25]),
                 ([1, 1, 1], [0, -1, -2]), ([0, 0, 0], [-1, -1, -1]),
                 ([.25, .25, .25], [.75, .75, .75]), ([.25, .25, .25], [.25, .25, .25])]
        for origin, endpoint in cases:
            for scale in (.1, 1.0, 4.0):
                with self.subTest(origin=origin, endpoint=endpoint, scale=scale):
                    self.assert_matches(np.asarray(origin) * scale, np.asarray(endpoint) * scale, scale)
        self.assertEqual(builder._ray_cells(np.array([0., .5, .5]), np.array([-3., .5, .5]), 1, 10),
                         [(-1, 0, 0), (-2, 0, 0)])

    def test_edge_and_corner_only_touch_never_clear_side_cells(self):
        origin = np.array([.5, .5, .5])
        for endpoint, expected in (([3.5, 3.5, .5], [(1, 1, 0), (2, 2, 0)]),
                                   ([3.5, 3.5, 3.5], [(1, 1, 1), (2, 2, 2)])):
            actual = builder._ray_cells(origin, np.asarray(endpoint), 1, 10)
            self.assertEqual(actual, expected)
            self.assert_matches(origin, endpoint)

    def test_absolute_tie_tolerance_both_sides_of_one_e_minus_twelve(self):
        first_x_crossing = .5 / 3.0
        for offset in (-2e-12, -5e-13, 0, 5e-13, 2e-12):
            for scale in (.001, 1, 1000):
                origin = np.array([.5, .5, .5]) * scale
                endpoint = np.array([3.5, .5 + .5 / (first_x_crossing + offset), .5]) * scale
                with self.subTest(offset=offset, scale=scale):
                    self.assert_matches(origin, endpoint, scale)
                    cells = builder._ray_cells(origin, endpoint, scale, 100)
                    if abs(offset) < 1e-12:
                        self.assertEqual(cells[0], (1, 1, 0))
                    else:
                        self.assertNotEqual(cells[0], (1, 1, 0))

    def test_max_ray_cells_counts_same_manhattan_upper_bound(self):
        for maximum in (0, 1, 3, 4, 9, 10):
            with self.subTest(maximum=maximum):
                self.assert_matches([.5, .5, .5], [3.5, 3.5, 3.5], max_cells=maximum)

    def test_multiorigin_full_revision_matches_original_ray_semantics(self):
        graph, frames, config = map_fixture()
        actual = builder.build_map(graph, frames, config)
        with patch.object(builder, "_ray_cells", numpy_ray_oracle):
            expected = builder.build_map(graph, frames, config)
        self.assertEqual(actual, expected)
        self.assertEqual(len(actual["origins"]), 4)
        self.assertNotEqual(actual["origins"][0]["origin_map"], actual["origins"][1]["origin_map"])
        graph["revision"] = 2
        graph["nodes"][0]["T_map_node"][0][3] += 2
        rebuilt = builder.build_map(graph, frames, config)
        with patch.object(builder, "_ray_cells", numpy_ray_oracle):
            corrected_oracle = builder.build_map(graph, frames, config)
        self.assertEqual(rebuilt, corrected_oracle)
        self.assertNotEqual(actual["occupancy"], rebuilt["occupancy"])

    def test_scalar_kernel_has_no_small_numpy_operations(self):
        origin, endpoint = np.array([.5, .5, .5]), np.array([3.5, 3.5, 3.5])
        with patch.object(builder, "np", None):
            self.assertEqual(builder._ray_cells(origin, endpoint, 1, 10), [(1, 1, 1), (2, 2, 2)])


class GlobalVoxelBoundTests(unittest.TestCase):
    def test_global_bound_counts_union_of_occupied_and_free_cells(self):
        graph, frames, config = map_fixture()
        original = builder.build_map(graph, frames, config)
        count = len(original["occupancy"]["voxels"])
        exact = builder.build_map(graph, frames, dict(config, max_voxels=count))
        self.assertEqual(original, exact)
        with self.assertRaisesRegex(MapError, "max_voxels resource bound"):
            builder.build_map(graph, frames, dict(config, max_voxels=count - 1))

    def test_bound_is_shared_between_sensors_not_reset_per_frame(self):
        graph, frames, config = map_fixture()
        totals = []
        for index in range(len(frames["observations"])):
            one_frame = copy.deepcopy(frames["observations"][index])
            one_graph = copy.deepcopy(graph)
            one_graph["nodes"] = [n for n in one_graph["nodes"] if n["node_id"] == one_frame["node_id"]]
            one_graph["edges"] = []
            single = builder.build_map(one_graph, {"observations": [one_frame]},
                                       dict(config, sensor_mode="single_" + one_frame["side"]))
            totals.append(len(single["occupancy"]["voxels"]))
        with self.assertRaisesRegex(MapError, "max_voxels resource bound"):
            builder.build_map(graph, frames, dict(config, max_voxels=max(totals)))

    def test_duplicate_rays_and_free_occupied_overlap_do_not_spend_extra_voxels(self):
        graph, frames, config = map_fixture()
        original = builder.build_map(graph, frames, config)
        limit = len(original["occupancy"]["voxels"])
        for frame in frames["observations"]:
            frame["points"] *= 3
        repeated = builder.build_map(graph, frames, dict(config, max_voxels=limit))
        self.assertEqual(original["occupancy"], repeated["occupancy"])
        self.assertEqual(original["grid"], repeated["grid"])

    def test_invalid_zero_and_too_small_global_limits_fail_closed(self):
        graph, frames, config = map_fixture()
        for value in (True, -1, .5, "500", None):
            with self.subTest(value=value), self.assertRaisesRegex(MapError, "max_voxels"):
                builder.build_map(graph, frames, dict(config, max_voxels=value))
        for value in (0, 1):
            with self.subTest(value=value), self.assertRaisesRegex(MapError, "max_voxels resource bound"):
                builder.build_map(graph, frames, dict(config, max_voxels=value))


if __name__ == "__main__":
    unittest.main()
