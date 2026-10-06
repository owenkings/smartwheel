"""Synthetic sidecar/database tests; actual RTAB-Map core has its C++ test."""

import copy
import hashlib
import json
from pathlib import Path
import runpy
import sqlite3
import tempfile
import unittest

import numpy as np

from wc_maps.builder import MapError
from wc_maps.graph_snapshot import assemble_closed_snapshot, rebuild_revision, revision_path
from wc_maps.store import MapStore


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False), encoding="utf-8")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def scene(root):
    config = {"resolution": 1.0, "ground_z": -0.5, "ground_tolerance": 0.05,
              "collision_min_height": 0.6, "collision_max_height": 2.4, "ground_valid": True}
    calibration = {"calibration_id": "synthetic-calibration", "source_mode": "synthetic", "status": "UNVALIDATED"}
    left = np.eye(4); left[:3, 3] = [.5, .5, .5]
    right = np.eye(4); right[:3, 3] = [.5, 3.5, .5]
    raw = {"schema_version": 1, "session_id": "synthetic-session", "source_mode": "synthetic", "sensor_mode": "dual",
           "bundle_id": 1, "t_ref_ns": 1000000000, "calibration_id": calibration["calibration_id"], "time_model_id": "synthetic-time",
           "observations": [{"raw_key": "left-1", "sensor_id": "synthetic-left", "side": "left", "stamp_ns": 1000000000,
                             "points": [[-3, 0, 0]], "valid_count": 1, "T_ref_sensor": left.tolist(), "origin_in_ref": [.5, .5, .5]},
                            {"raw_key": "right-1", "sensor_id": "synthetic-right", "side": "right", "stamp_ns": 1000000000,
                             "points": [[3, 0, 0]], "valid_count": 1, "T_ref_sensor": right.tolist(), "origin_in_ref": [.5, 3.5, .5]}]}
    write_json(root / "raw_observations" / "1.json", raw)
    checksum = digest(root / "raw_observations" / "1.json")
    graph = {"session_id": "synthetic-session", "graph_epoch": "graph-uuid", "source_mode": "synthetic", "revision": 1,
             "frame_id": "map", "calibration_id": calibration["calibration_id"], "time_model_id": "synthetic-time",
             "left_clock_model_id": "left-clock", "right_clock_model_id": "right-clock", "raw_index_complete": True,
             "raw_index_hash": hashlib.sha256(f"1\t{checksum}\n".encode()).hexdigest(), "database_closed": False,
             "rtabmap_version": "synthetic-sidecar-fixture-not-core-execution",
             "nodes": [{"node_id": 1, "bundle_id": 1, "odom_epoch": "odom-uuid", "stamp_ns": 1000000000,
                        "T_map_node": np.eye(4).tolist(), "raw_file": "raw_observations/1.json", "raw_file_sha256": checksum,
                        "raw_keys": ["left-1", "right-1"], "t_node_sensor": [left.tolist(), right.tolist()]}], "edges": []}
    write_json(revision_path(root, 1), graph)
    (root / "slam").mkdir()
    with sqlite3.connect(root / "slam" / "rtabmap.db") as database:
        database.execute("CREATE TABLE synthetic_fixture_evidence (value TEXT)")
        database.execute("INSERT INTO synthetic_fixture_evidence VALUES ('not a real RTABMap schema')")
    marker = {"schema_version": 1, "state": "CLOSED_FOR_SNAPSHOT", "session_id": graph["session_id"],
              "graph_epoch": graph["graph_epoch"], "revision": 1, "raw_index_hash": graph["raw_index_hash"],
              "graph_file": "graph/revisions/r00000000000000000001.json", "graph_sha256": digest(revision_path(root, 1)),
              "database_file": "slam/rtabmap.db", "database_sha256": digest(root / "slam" / "rtabmap.db")}
    write_json(root / "graph" / "closed_snapshot.json", marker)
    return graph, raw, config, calibration


class SnapshotAssemblyTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="wc-assembly-synthetic-")
        self.root = Path(self.temporary.name)
        self.graph, self.raw, self.config, self.calibration = scene(self.root)

    def tearDown(self):
        self.temporary.cleanup()

    def rehash_raw(self):
        path = self.root / "raw_observations" / "1.json"
        write_json(path, self.raw)
        checksum = digest(path)
        self.graph["nodes"][0]["raw_file_sha256"] = checksum
        self.graph["raw_index_hash"] = hashlib.sha256(f"1\t{checksum}\n".encode()).hexdigest()

    def test_complete_dual_snapshot_then_mapstore_save_load(self):
        output = self.root / "snapshots" / "r001"
        result = assemble_closed_snapshot(self.root, output, self.config, self.calibration)
        self.assertEqual(result["snapshot"]["completed_map_revision"], 1)
        self.assertEqual(result["snapshot"]["graph_epoch"], "graph-uuid")
        self.assertEqual(result["source_point_counts"], {"left": 1, "right": 1})
        saved = MapStore(self.root / "maps")
        saved.save_snapshot(output, "synthetic-map", "v001")
        loaded = saved.load("synthetic-map", "v001")
        self.assertEqual(loaded["manifest"]["source_mode"], "synthetic")
        self.assertIsNone(loaded["current_pose"])
        self.assertEqual(loaded["trajectory"]["poses"][0]["T_map_rig"], np.eye(4).tolist())

    def test_missing_close_barrier_and_database_mutation_refuse_save(self):
        marker_path = self.root / "graph" / "closed_snapshot.json"
        original = marker_path.read_bytes()
        # A legacy latest pointer cannot substitute for the graph owner's
        # explicit DB-close barrier, even when its graph path/hash are valid.
        write_json(self.root / "graph" / "current.json", {
            "schema_version": 1, "revision": 1,
            "relative_file": "graph/revisions/r00000000000000000001.json",
            "sha256": digest(revision_path(self.root, 1))})
        marker_path.unlink()
        with self.assertRaises((MapError, OSError)):
            assemble_closed_snapshot(self.root, self.root / "snapshots" / "r001", self.config, self.calibration)
        marker_path.write_bytes(original)
        with sqlite3.connect(self.root / "slam" / "rtabmap.db") as database:
            database.execute("INSERT INTO synthetic_fixture_evidence VALUES ('changed after barrier')")
        with self.assertRaisesRegex(MapError, "database changed"):
            assemble_closed_snapshot(self.root, self.root / "snapshots" / "r001", self.config, self.calibration)

    def test_closed_snapshot_ignores_corrupt_or_misdirected_current_pointer(self):
        newer = copy.deepcopy(self.graph)
        newer["revision"] = 2
        newer["nodes"][0]["T_map_node"][0][3] = 10
        write_json(revision_path(self.root, 2), newer)
        pointer = self.root / "graph" / "current.json"
        store = MapStore(self.root / "maps")
        for mode in ("corrupt", "unsafe_path", "wrong_revision"):
            with self.subTest(current_pointer=mode):
                if mode == "corrupt":
                    pointer.write_bytes(b"not valid JSON and never an authority")
                else:
                    write_json(pointer, {"schema_version": 1, "revision": 2,
                        "relative_file": "../../unrelated.json" if mode == "unsafe_path" else
                            "graph/revisions/r00000000000000000002.json",
                        "sha256": digest(revision_path(self.root, 2))})
                prior_pointer = pointer.read_bytes()
                snapshot = self.root / "snapshots" / mode
                assembled = assemble_closed_snapshot(self.root, snapshot, self.config, self.calibration)
                self.assertEqual(assembled["snapshot"]["graph_revision"], 1)
                self.assertEqual(assembled["snapshot"]["completed_map_revision"], 1)
                self.assertEqual(assembled["source_point_counts"], {"left": 1, "right": 1})
                store.save_snapshot(snapshot, "synthetic-pointer-independent", mode)
                store.verify("synthetic-pointer-independent", mode)
                loaded = store.load("synthetic-pointer-independent", mode)
                self.assertEqual(loaded["manifest"]["graph_revision"], 1)
                self.assertEqual(loaded["grid"]["revision"], 1)
                self.assertEqual(loaded["occupancy"]["revision"], 1)
                self.assertEqual(loaded["trajectory"]["poses"][0]["T_map_rig"], np.eye(4).tolist())
                self.assertEqual(pointer.read_bytes(), prior_pointer,
                                 "consumer rewrote a legacy pointer during explicit-revision loading")

    def test_corrupted_raw_and_semantic_key_mismatch_are_rejected(self):
        path = self.root / "raw_observations" / "1.json"
        path.write_bytes(path.read_bytes() + b" ")
        with self.assertRaisesRegex(MapError, "archive hash"):
            rebuild_revision(self.root, self.graph, self.config, self.calibration)
        self.raw["observations"][0]["raw_key"] = "a-different-raw-frame"
        self.rehash_raw()
        with self.assertRaisesRegex(MapError, "raw key"):
            rebuild_revision(self.root, self.graph, self.config, self.calibration)

    def test_sensor_origin_session_time_and_missing_side_do_not_silently_pass(self):
        original = copy.deepcopy(self.raw)
        for kind in ("origin", "session", "time", "missing-side"):
            self.raw = copy.deepcopy(original)
            if kind == "origin":
                self.raw["observations"][1]["T_ref_sensor"][1][3] += 0.1
            elif kind == "session":
                self.raw["session_id"] = "other-session"
            elif kind == "time":
                self.raw["t_ref_ns"] += 1
            else:
                self.raw["observations"].pop()
            self.rehash_raw()
            with self.subTest(kind=kind), self.assertRaises(MapError):
                rebuild_revision(self.root, self.graph, self.config, self.calibration)

    def test_new_revision_changes_all_ray_origins_without_old_wall(self):
        before = rebuild_revision(self.root, self.graph, self.config, self.calibration)["mapped"]
        self.graph["revision"] = 2
        self.graph["nodes"][0]["T_map_node"][0][3] = 10
        after = rebuild_revision(self.root, self.graph, self.config, self.calibration)["mapped"]
        self.assertEqual(before["cloud"][0][0], -2.5)
        self.assertEqual(after["cloud"][0][0], 7.5)
        self.assertEqual(after["origins"][1]["origin_map"][0], 10.5)
        self.assertFalse(any(voxel["index"] == [-3, 0, 0] for voxel in after["occupancy"]["voxels"]))

    def test_output_collision_traversal_and_calibration_mismatch_refused(self):
        with self.assertRaisesRegex(MapError, "calibration"):
            rebuild_revision(self.root, self.graph, self.config, {"calibration_id": "wrong"})
        self.graph["nodes"][0]["raw_file"] = "../other.json"
        with self.assertRaisesRegex(MapError, "filename"):
            rebuild_revision(self.root, self.graph, self.config, self.calibration)
        output = self.root / "existing"
        output.mkdir()
        with self.assertRaisesRegex(MapError, "new directory"):
            assemble_closed_snapshot(self.root, output, self.config, self.calibration)

    def test_session_config_wrapper_passes_literal_arguments_and_explicit_noise(self):
        script = Path(__file__).resolve().parents[2] / "src" / "wc_slam" / "scripts" / "wc_graph_node"
        wrapper = runpy.run_path(str(script))
        config = {"session_root": str(self.root), "run_root": str(self.root / "run"), "session_id": "test $(no_shell)",
                  "graph_epoch": "graph-uuid", "source_mode": "synthetic", "time_model_id": "time-uuid",
                  "graph_noise_model": "diagonal_assumption", "graph_noise_diagonal": [0.01] * 6}
        path = self.root / "session config.json"
        write_json(path, config)
        arguments = wrapper["arguments"](["--ros-args", "-p", "session_config:=" + str(path)])
        self.assertIn('session_id:="test $(no_shell)"', arguments)
        self.assertIn('graph_edge_noise_model:="diagonal_assumption"', arguments)
        self.assertTrue(any(arg.startswith("graph_edge_noise_diagonal:=") for arg in arguments))


if __name__ == "__main__":
    unittest.main()
