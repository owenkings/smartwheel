"""Synthetic geometry only; these tests provide no live mapping evidence."""

import copy
import unittest

import numpy as np

from wc_maps.builder import MapError, RevisionGuard, build_map


def transform(x=0, y=0, z=0):
    matrix = np.eye(4)
    matrix[:3, 3] = [x, y, z]
    return matrix.tolist()


def configuration(**changes):
    config = {"resolution": 1.0, "ground_z": -0.5, "ground_tolerance": 0.05,
              "collision_min_height": 0.6, "collision_max_height": 2.4,
              "ground_valid": True, "sensor_mode": "dual"}
    config.update(changes)
    return config


def graph_fixture():
    return {"session_id": "synthetic-session", "source_mode": "synthetic", "graph_epoch": 0, "revision": 1,
            "nodes": [{"node_id": 1, "odom_epoch": 0, "T_map_node": transform()}], "edges": []}


def observation(raw, side, origin, endpoints, node_id=1):
    points = (np.asarray(endpoints) - np.asarray(origin)).tolist()
    return {"raw_key": raw, "sensor_id": "synthetic-" + side, "side": side,
            "node_id": node_id, "odom_epoch": 0, "points": points,
            "T_node_sensor": transform(*origin)}


def observations_fixture():
    return {"observations": [
        observation("L0", "left", [.5, .5, .5], [[-2.5, .5, .5]]),
        observation("R0", "right", [.5, 3.5, .5], [[3.5, 3.5, .5]])]}


def cell(result, x, y):
    grid = result["grid"]
    resolution = grid["resolution"]
    ix = int(round(x - grid["origin"][0] / resolution))
    iy = int(round(y - grid["origin"][1] / resolution))
    return grid["data"][iy * grid["width"] + ix]


class BuilderTests(unittest.TestCase):
    def test_two_original_origins_no_rig_clearing(self):
        result = build_map(graph_fixture(), observations_fixture(), configuration())
        states = {tuple(v["index"]): v["state"] for v in result["occupancy"]["voxels"]}
        self.assertEqual(states[(1, 3, 0)], 0)
        self.assertNotIn((1, 1, 0), states)
        self.assertEqual(result["origins"][1]["origin_map"], [.5, 3.5, .5])
        self.assertEqual(result["source_point_counts"], {"left": 1, "right": 1})

    def test_only_high_rays_never_clear_collision_band(self):
        frames = {"observations": [
            observation("L0", "left", [.5, .5, 3.5], [[3.5, .5, 3.5]]),
            observation("R0", "right", [.5, 1.5, 3.5], [[3.5, 1.5, 3.5]])]}
        result = build_map(graph_fixture(), frames, configuration())
        self.assertTrue(all(v == -1 for v in result["grid"]["data"]))

    def test_low_obstacle_is_not_removed_as_ground(self):
        frames = observations_fixture()
        frames["observations"][0] = observation("L0", "left", [.5, .5, .5], [[2.5, .5, .2]])
        result = build_map(graph_fixture(), frames, configuration())
        self.assertEqual(cell(result, 2, 0), 100)
        # A doorway/space without complete vertical evidence remains unknown.
        self.assertEqual(cell(result, 1, 1), -1)

    def test_free_requires_every_vertical_cell_and_ground_return(self):
        frames = {"observations": [
            observation("L0", "left", [-.5, .5, .5], [[3.5, .5, .5]]),
            observation("R0", "right", [-.5, .5, 1.5], [[3.5, .5, 1.5]]),
            observation("G0", "left", [1.5, .5, -.4], [[1.5, .5, -.5]])]}
        result = build_map(graph_fixture(), frames, configuration())
        self.assertEqual(cell(result, 1, 0), 0)
        self.assertEqual(cell(build_map(graph_fixture(), frames, configuration(ground_valid=False)), 1, 0), -1)
        without_ground = {"observations": frames["observations"][:2]}
        self.assertEqual(cell(build_map(graph_fixture(), without_ground, configuration()), 1, 0), -1)
        without_low = copy.deepcopy(frames)
        without_low["observations"][0] = observation("L0", "left", [-.5, 1.5, .5], [[3.5, 1.5, .5]])
        self.assertEqual(cell(build_map(graph_fixture(), without_low, configuration()), 1, 0), -1)

    def test_high_ray_does_not_clear_existing_low_obstacle(self):
        frames = observations_fixture()
        frames["observations"][0] = observation("L0", "left", [.5, .5, .5], [[2.5, .5, .2]])
        frames["observations"][1] = observation("R0", "right", [.5, .5, 3.5], [[4.5, .5, 3.5]])
        self.assertEqual(cell(build_map(graph_fixture(), frames, configuration()), 2, 0), 100)

    def test_occupied_wins_over_free_without_duplicate_probability(self):
        frames = observations_fixture()
        frames["observations"][0] = observation("L0", "left", [.5, .5, .5], [[2.5, .5, .5], [2.5, .5, .5]])
        frames["observations"][1] = observation("R0", "right", [.5, .5, .5], [[4.5, .5, .5]])
        result = build_map(graph_fixture(), frames, configuration())
        matches = [v for v in result["occupancy"]["voxels"] if v["index"] == [2, 0, 0]]
        self.assertEqual(matches, [{"index": [2, 0, 0], "state": 100}])

    def test_revision_rebuild_removes_old_wall_and_moves_origins(self):
        graph = graph_fixture()
        before = build_map(graph, observations_fixture(), configuration())
        graph["revision"] = 2
        graph["nodes"][0]["T_map_node"] = transform(10)
        after = build_map(graph, observations_fixture(), configuration())
        occupied = {tuple(v["index"]) for v in after["occupancy"]["voxels"] if v["state"] == 100}
        self.assertNotIn((-3, 0, 0), occupied)
        self.assertIn((7, 0, 0), occupied)
        self.assertEqual(after["origins"][0]["origin_map"], [10.5, .5, .5])
        self.assertEqual(before["revision"], 1)
        self.assertEqual(after["revision"], 2)

    def test_negative_coordinates_use_floor_and_x_fastest_rows(self):
        result = build_map(graph_fixture(), observations_fixture(), configuration())
        self.assertEqual(result["grid"]["origin"][:2], [-3.0, 0.0])
        self.assertEqual(cell(result, -3, 0), 100)
        self.assertEqual(cell(result, 3, 3), 100)
        self.assertEqual(result["grid"]["data"][0], 100)

    def test_each_node_requires_both_raw_sources(self):
        graph = graph_fixture()
        graph["nodes"].append({"node_id": 2, "odom_epoch": 0, "T_map_node": transform(1)})
        with self.assertRaisesRegex(MapError, "lacks required raw"):
            build_map(graph, observations_fixture(), configuration())
        frames = observations_fixture()
        frames["observations"].pop()
        with self.assertRaisesRegex(MapError, "lacks required raw"):
            build_map(graph_fixture(), frames, configuration())
        self.assertEqual(build_map(graph_fixture(), frames, configuration(sensor_mode="single_left"))["sensor_mode"], "single_left")

    def test_duplicate_frame_and_id_reuse_across_sessions_rejected(self):
        frames = observations_fixture()
        frames["observations"].append(copy.deepcopy(frames["observations"][0]))
        with self.assertRaisesRegex(MapError, "raw frame reused"):
            build_map(graph_fixture(), frames, configuration())
        for field, value in (("session_id", "previous-session"), ("graph_epoch", 99), ("odom_epoch", 9)):
            frames = observations_fixture()
            frames["observations"][0][field] = value
            with self.assertRaisesRegex(MapError, "epoch|session"):
                build_map(graph_fixture(), frames, configuration())

    def test_opaque_epoch_identifiers_are_preserved(self):
        graph, frames = graph_fixture(), observations_fixture()
        graph["graph_epoch"] = "graph-uuid-synthetic"
        graph["nodes"][0]["odom_epoch"] = "odom-uuid-synthetic"
        for frame in frames["observations"]:
            frame["odom_epoch"] = "odom-uuid-synthetic"
            frame["graph_epoch"] = "graph-uuid-synthetic"
        result = build_map(graph, frames, configuration())
        self.assertEqual(result["graph_epoch"], "graph-uuid-synthetic")
        self.assertEqual(result["raw_data_index"][0]["odom_epoch"], "odom-uuid-synthetic")

    def test_missing_node_nonrigid_transform_and_invalid_depth_rejected(self):
        for alteration in ("missing", "nonrigid", "nan", "empty", "zero"):
            frames = observations_fixture()
            frame = frames["observations"][0]
            if alteration == "missing":
                frame["node_id"] = 99
            elif alteration == "nonrigid":
                frame["T_node_sensor"][0][0] = 2
            elif alteration == "nan":
                frame["points"][0][0] = float("nan")
            elif alteration == "empty":
                frame["points"] = []
            else:
                frame["points"] = [[0, 0, 0]]
            with self.subTest(alteration=alteration), self.assertRaises(MapError):
                build_map(graph_fixture(), frames, configuration())

    def test_unknown_ground_and_excessive_allocations_fail_closed(self):
        with self.assertRaises(MapError):
            build_map(graph_fixture(), observations_fixture(), configuration(ground_valid=None))
        for bound in ("max_points", "max_grid_cells", "max_ray_cells"):
            with self.subTest(bound=bound), self.assertRaisesRegex(MapError, "resource bound"):
                build_map(graph_fixture(), observations_fixture(), configuration(**{bound: 1}))

    def test_stale_worker_cannot_overwrite_new_revision(self):
        guard = RevisionGuard()
        graph = graph_fixture()
        first = build_map(graph, observations_fixture(), configuration())
        token1 = guard.request(graph["session_id"], 0, 1)
        self.assertTrue(guard.complete(token1, first))
        token2 = guard.request(graph["session_id"], 0, 2)
        self.assertTrue(guard.status()["stale"])
        graph["revision"] = 2
        second = build_map(graph, observations_fixture(), configuration())
        self.assertTrue(guard.complete(token2, second))
        self.assertFalse(guard.complete(token1, first))
        self.assertEqual(guard.status()["snapshot"]["revision"], 2)
        token3 = guard.request(graph["session_id"], 0, 3)
        guard.fail(token3, "source index incomplete")
        self.assertTrue(guard.status()["stale"])
        self.assertFalse(guard.status()["rebuilding"])
        self.assertEqual(guard.status()["snapshot"]["revision"], 2)


if __name__ == "__main__":
    unittest.main()
