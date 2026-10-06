"""Linux-target synthetic filesystem/database acceptance tests."""

import copy
import errno
import hashlib
import json
import multiprocessing
from pathlib import Path
import sqlite3
import sys
import tempfile
import unittest
from unittest import mock

from wc_maps.builder import MapError
from wc_maps.store import MapStore

try:
    from .test_builder import configuration, graph_fixture, observations_fixture
except ImportError:
    from test_builder import configuration, graph_fixture, observations_fixture


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, allow_nan=False), encoding="utf-8")


def snapshot_fixture(directory, wal=False):
    directory.mkdir(parents=True)
    snapshot = {"schema_version": 1, "session_id": "synthetic-session", "source_mode": "synthetic", "sensor_mode": "dual",
                "graph_epoch": 0, "graph_revision": 1, "completed_map_revision": 1, "frame_id": "map",
                "ground_valid": True, "calibration_id": "synthetic-calibration", "time_model_id": "synthetic-clock"}
    values = {"snapshot.json": snapshot, "graph.json": graph_fixture(), "observations.json": observations_fixture(),
              "config.json": configuration(), "calibration.json": {"source_mode": "synthetic", "status": "UNVALIDATED",
                                                                   "calibration_id": "synthetic-calibration"},
              "trajectory.json": {"session_id": "synthetic-session", "graph_epoch": 0, "graph_revision": 1,
                                  "body_frame_id": "rig_link", "poses": [{"node_id": 1, "T_map_rig": graph_fixture()["nodes"][0]["T_map_node"]}]}}
    for frame in values["observations.json"]["observations"]:
        frame.update(session_id=snapshot["session_id"], graph_epoch=snapshot["graph_epoch"])
    for filename, value in values.items():
        write_json(directory / filename, value)
    (directory / "slam").mkdir()
    connection = sqlite3.connect(directory / "slam" / "rtabmap.db")
    if wal:
        connection.execute("PRAGMA journal_mode=WAL")
        connection.execute("PRAGMA wal_autocheckpoint=0")
    connection.execute("CREATE TABLE synthetic_evidence (id INTEGER PRIMARY KEY, value TEXT)")
    connection.execute("INSERT INTO synthetic_evidence VALUES (1, 'committed-in-database')")
    connection.commit()
    if wal:
        connection.execute("INSERT INTO synthetic_evidence VALUES (2, 'committed-in-WAL')")
        connection.commit()
        return connection
    connection.close()
    return None


def save_worker(root, source, queue):
    try:
        MapStore(root).save_snapshot(source, "concurrent", "v001")
        queue.put("SAVED")
    except MapError as exc:
        queue.put("REFUSED:" + str(exc))
    except Exception as exc:
        queue.put("UNEXPECTED:" + repr(exc))


def fingerprints(directory):
    return {p.relative_to(directory).as_posix(): (hashlib.sha256(p.read_bytes()).hexdigest(), p.stat().st_mtime_ns)
            for p in directory.rglob("*") if p.is_file()}


@unittest.skipUnless(sys.platform.startswith("linux"), "MapStore runtime must be verified on the Linux target")
class StoreTests(unittest.TestCase):
    def setUp(self):
        self.workspace = tempfile.TemporaryDirectory(prefix="wc-maps-synthetic-")
        self.base = Path(self.workspace.name)
        self.source = self.base / "snapshot"
        snapshot_fixture(self.source)
        self.store = MapStore(self.base / "maps")

    def tearDown(self):
        self.workspace.cleanup()

    def test_save_verify_load_same_revision_and_pgm_row_order(self):
        saved = self.store.save_snapshot(self.source, "hospital", "v001")
        self.assertEqual(self.store.verify("hospital", "v001")["graph_revision"], 1)
        loaded = self.store.load("hospital", "v001")
        self.assertEqual(loaded["state"], "LOADED_VIEW_ONLY")
        self.assertIsNone(loaded["current_pose"])
        self.assertFalse(loaded["manifest"]["navigation_validated"])
        self.assertEqual(loaded["grid"]["revision"], loaded["occupancy"]["revision"])
        self.assertEqual(loaded["trajectory"]["graph_revision"], loaded["grid"]["revision"])
        directory = Path(saved["path"])
        grid = loaded["grid"]
        data = (directory / "occupancy" / "map.pgm").read_bytes().split(b"\n", 3)[3]
        expected = bytes({-1: 205, 0: 254, 100: 0}[grid["data"][y * grid["width"] + x]]
                         for y in reversed(range(grid["height"])) for x in range(grid["width"]))
        self.assertEqual(data, expected)
        self.assertIn("origin: [-3.0, 0.0, 0.0]", (directory / "occupancy" / "map.yaml").read_text())
        self.assertEqual(self.store.list_versions("hospital")[0]["version"], "v001")

    def test_live_wal_data_is_in_backup(self):
        wal_source = self.base / "wal-snapshot"
        writer = snapshot_fixture(wal_source, wal=True)
        try:
            self.assertTrue(Path(str(wal_source / "slam" / "rtabmap.db") + "-wal").exists())
            saved = self.store.save_snapshot(wal_source, "hospital", "wal001")
            with sqlite3.connect(Path(saved["path"]) / "slam" / "rtabmap.db") as read:
                self.assertEqual(read.execute("SELECT value FROM synthetic_evidence WHERE id=2").fetchone(), ("committed-in-WAL",))
        finally:
            writer.close()

    def test_load_and_verify_do_not_modify_immutable_version(self):
        saved = self.store.save_snapshot(self.source, "hospital", "v001")
        directory = Path(saved["path"])
        before = fingerprints(directory)
        self.store.load("hospital", "v001")
        self.store.verify("hospital", "v001")
        self.store.get_goals("hospital", "v001")
        self.assertEqual(fingerprints(directory), before)

    def test_duplicate_version_and_existing_empty_directory_refused(self):
        saved = self.store.save_snapshot(self.source, "hospital", "v001")
        before = fingerprints(Path(saved["path"]))
        with self.assertRaisesRegex(MapError, "immutable"):
            self.store.save_snapshot(self.source, "hospital", "v001")
        self.assertEqual(fingerprints(Path(saved["path"])), before)
        (self.store.root / "hospital" / "v002").mkdir()
        with self.assertRaisesRegex(MapError, "immutable"):
            self.store.save_snapshot(self.source, "hospital", "v002")

    def test_traversal_and_absolute_name_injection_refused(self):
        for value in ("../escape", "/tmp/escape", "a/b", "a\\b", ".", "..", "C:\\escape", "a\x00b"):
            with self.subTest(value=value), self.assertRaises(MapError):
                self.store.save_snapshot(self.source, value, "v001")
            with self.subTest(version=value), self.assertRaises(MapError):
                self.store.save_snapshot(self.source, "hospital", value)
        self.assertFalse((self.base / "escape").exists())

    def test_symlink_map_root_source_and_artifact_refused(self):
        root_link = self.base / "linked-root"
        root_link.symlink_to(self.store.root, target_is_directory=True)
        with self.assertRaisesRegex(MapError, "symbolic"):
            MapStore(root_link)
        source_link = self.base / "linked-source"
        source_link.symlink_to(self.source, target_is_directory=True)
        with self.assertRaisesRegex(MapError, "symbolic"):
            self.store.save_snapshot(source_link, "hospital", "v001")
        original = self.source / "calibration.json"
        actual = self.base / "actual-calibration.json"
        original.rename(actual)
        original.symlink_to(actual)
        with self.assertRaisesRegex(MapError, "symbolic"):
            self.store.save_snapshot(self.source, "hospital", "v001")

    def test_symlink_saved_file_cannot_pass_hash_verification(self):
        saved = self.store.save_snapshot(self.source, "hospital", "v001")
        artifact = Path(saved["path"]) / "calibration.json"
        other = self.base / "identical.json"
        other.write_bytes(artifact.read_bytes())
        artifact.unlink()
        artifact.symlink_to(other)
        with self.assertRaisesRegex(MapError, "symbolic"):
            self.store.verify("hospital", "v001")

    def test_corrupted_artifact_and_manifest_hash_refused(self):
        saved = self.store.save_snapshot(self.source, "hospital", "v001")
        directory = Path(saved["path"])
        with (directory / "occupancy" / "map.pgm").open("ab") as stream:
            stream.write(b"corruption")
        with self.assertRaisesRegex(MapError, "hash mismatch"):
            self.store.load("hospital", "v001")
        saved = self.store.save_snapshot(self.source, "hospital", "v002")
        manifest = Path(saved["path"]) / "manifest.json"
        manifest.write_bytes(manifest.read_bytes() + b" ")
        with self.assertRaisesRegex(MapError, "manifest hash mismatch"):
            self.store.verify("hospital", "v002")

    def test_revision_mismatch_and_incomplete_raw_index_prevent_commit(self):
        snapshot_path = self.source / "snapshot.json"
        snapshot = json.loads(snapshot_path.read_text())
        snapshot["completed_map_revision"] = 0
        write_json(snapshot_path, snapshot)
        with self.assertRaisesRegex(MapError, "incomplete map revision"):
            self.store.save_snapshot(self.source, "hospital", "v001")
        snapshot["completed_map_revision"] = 1
        write_json(snapshot_path, snapshot)
        frames = json.loads((self.source / "observations.json").read_text())
        frames["observations"].pop()
        write_json(self.source / "observations.json", frames)
        with self.assertRaisesRegex(MapError, "lacks required raw"):
            self.store.save_snapshot(self.source, "hospital", "v001")
        self.assertFalse((self.store.root / "hospital" / "v001").exists())

    def test_trajectory_revision_mismatch_is_not_silently_saved(self):
        trajectory = json.loads((self.source / "trajectory.json").read_text())
        trajectory["graph_revision"] = 42
        write_json(self.source / "trajectory.json", trajectory)
        with self.assertRaisesRegex(MapError, "trajectory graph_revision"):
            self.store.save_snapshot(self.source, "hospital", "v001")

    def test_trajectory_strong_contract_rejects_missing_duplicate_and_wrong_pose(self):
        path = self.source / "trajectory.json"
        original = json.loads(path.read_text())
        invalid = []
        for field in ("session_id", "graph_epoch", "graph_revision", "body_frame_id", "poses"):
            entry = copy.deepcopy(original)
            del entry[field]
            invalid.append(("missing-" + field, entry))
        empty = copy.deepcopy(original)
        empty["poses"] = []
        invalid.append(("empty", empty))
        duplicate = copy.deepcopy(original)
        duplicate["poses"].append(copy.deepcopy(duplicate["poses"][0]))
        invalid.append(("duplicate", duplicate))
        wrong = copy.deepcopy(original)
        wrong["poses"][0]["T_map_rig"][0][3] = 0.01
        invalid.append(("wrong-pose", wrong))
        foreign = copy.deepcopy(original)
        foreign["poses"][0]["node_id"] = 99
        invalid.append(("unknown-node", foreign))
        missing_transform = copy.deepcopy(original)
        del missing_transform["poses"][0]["T_map_rig"]
        invalid.append(("missing-transform", missing_transform))
        for label, entry in invalid:
            write_json(path, entry)
            with self.subTest(label=label), self.assertRaises(MapError):
                self.store.save_snapshot(self.source, "hospital", "v001")
            self.assertFalse((self.store.root / "hospital" / "v001").exists())

    def test_trajectory_must_cover_all_graph_nodes(self):
        graph = graph_fixture()
        second = copy.deepcopy(graph["nodes"][0])
        second["node_id"] = 2
        graph["nodes"].append(second)
        write_json(self.source / "graph.json", graph)
        with self.assertRaisesRegex(MapError, "trajectory must cover every graph node"):
            self.store.save_snapshot(self.source, "hospital", "v001")

    def test_disk_full_preserves_previous_version_and_removes_own_staging(self):
        saved = self.store.save_snapshot(self.source, "hospital", "v001")
        before = fingerprints(Path(saved["path"]))
        with mock.patch("wc_maps.store._write", side_effect=OSError(errno.ENOSPC, "synthetic disk-full injection")):
            with self.assertRaisesRegex(MapError, "disk full"):
                self.store.save_snapshot(self.source, "hospital", "v002")
        self.assertEqual(fingerprints(Path(saved["path"])), before)
        self.assertFalse((self.store.root / "hospital" / "v002").exists())
        self.assertFalse(list((self.store.root / "hospital").glob(".pending-*")))

    def test_concurrent_save_uses_cross_process_lock(self):
        context = multiprocessing.get_context("fork")
        queue = context.Queue()
        processes = [context.Process(target=save_worker, args=(str(self.store.root), str(self.source), queue)) for _ in range(2)]
        for process in processes:
            process.start()
        results = [queue.get(timeout=30) for _ in processes]
        for process in processes:
            process.join(timeout=30)
            self.assertEqual(process.exitcode, 0)
        self.assertEqual(results.count("SAVED"), 1)
        self.assertEqual(sum(result.startswith("REFUSED:") for result in results), 1, results)
        self.store.verify("concurrent", "v001")
        self.assertFalse(list((self.store.root / "concurrent").glob(".pending-*")))

    def test_utf8_goal_edit_review_and_version_invalidation(self):
        saved = self.store.save_snapshot(self.source, "hospital", "v001")
        before = fingerprints(Path(saved["path"]))
        goal = {"name": "一楼护士站", "position": [1, 2, 0], "orientation_xyzw": [0, 0, 0, 1]}
        self.assertEqual(self.store.set_goal("hospital", "v001", goal)["review_state"], "UNVERIFIED")
        self.assertEqual(self.store.review_goal("hospital", "v001", goal["name"], True)["review_state"], "VERIFIED")
        self.store.save_snapshot(self.source, "hospital", "v002")
        new = self.store.migrate_goals("hospital", "v001", "v002")
        self.assertEqual(new["goals"][0]["name"], goal["name"])
        self.assertEqual(new["goals"][0]["review_state"], "UNVERIFIED")
        self.assertEqual(new["goals"][0]["version"], "v002")
        self.assertEqual(self.store.get_goals("hospital", "v001")["goals"][0]["review_state"], "VERIFIED")
        self.assertEqual(fingerprints(Path(saved["path"])), before)
        edited = copy.deepcopy(goal)
        edited["position"][0] = 5
        self.assertEqual(self.store.set_goal("hospital", "v001", edited)["review_state"], "UNVERIFIED")

    def test_goal_copied_in_new_snapshot_cannot_keep_verified(self):
        goal = {"name": "充电位", "position": [0, 0, 0], "orientation_xyzw": [0, 0, 0, 1],
                "map_id": "old-map", "version": "old-version", "review_state": "VERIFIED"}
        write_json(self.source / "goals.json", {"goals": [goal]})
        self.store.save_snapshot(self.source, "hospital", "v001")
        stored = self.store.get_goals("hospital", "v001")["goals"][0]
        self.assertEqual(stored["map_id"], "hospital")
        self.assertEqual(stored["review_state"], "UNVERIFIED")

    def test_goal_binding_tamper_and_bad_quaternion_rejected(self):
        self.store.save_snapshot(self.source, "hospital", "v001")
        goal = {"name": "入口", "position": [0, 0, 0], "orientation_xyzw": [0, 0, 0, 0]}
        with self.assertRaisesRegex(MapError, "normalized"):
            self.store.set_goal("hospital", "v001", goal)
        goal["orientation_xyzw"] = [0, 0, 0, 1]
        self.store.set_goal("hospital", "v001", goal)
        path = self.store._annotation_path("hospital", "v001")
        data = json.loads(path.read_text())
        data["version"] = "wrong-version"
        write_json(path, data)
        with self.assertRaisesRegex(MapError, "different map/version"):
            self.store.load("hospital", "v001")


if __name__ == "__main__":
    unittest.main()
