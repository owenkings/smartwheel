"""Explicit synthetic JSON fixtures test evidence rejection, not real TF.

No ROS publisher, hardware or native runtime evidence is created by these tests.
The coordinator runs these pure-Python checks only on the verified target.
"""
import copy
import importlib.util
import json
import os
from pathlib import Path
import socket
import tempfile
import time
import unittest


OBSERVER = Path(__file__).resolve().parents[3] / "check_icp_pipeline.py"
SPEC = importlib.util.spec_from_file_location("wc_icp_observer_tf_test", OBSERVER)
observer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(observer)


class NativeTfEvidenceTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="wc_tf_validation_fixture_")
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "explicit_synthetic_fixture.json"
        now = time.time_ns()
        self.reference = now - 6_000_000_000
        self.config = {"session_id": "EVIDENCE_VALIDATION_FIXTURE", "graph_epoch": "fixture_graph", "source_mode": "synthetic",
                       "_observer_tf_evidence": str(self.path)}
        self.recorder = observer.Recorder(self.config, False)
        self.recorder.start_unix_ns = now - 8_000_000_000
        self.recorder.accepted[1] = dict(bundle=dict(bundle_id=1, stamp_ns=self.reference),
            tracking=dict(odometry=dict(pose_xyz_xyzw=[.2, 0, 0, 0, 0, 0, 1])))
        self.recorder.graphs[1] = dict(stamp_ns=self.reference, nodes=[dict(bundle_id=1, pose_xyz_xyzw=[.3, 0, 0, 0, 0, 0, 1])])
        self.document = dict(schema_version=1, test="native_tf_ownership_probe", status="PASS",
            session_id=self.config["session_id"], hostname=socket.gethostname(),
            ros_domain_id=os.environ.get("ROS_DOMAIN_ID", "0"), topic="/tf", rmw_gid_storage_size=24,
            navigation_validated=False, errors=[], interrupted=False, start_unix_ns=now - 10_000_000_000,
            end_unix_ns=now - 1_000_000_000, messages_received=2, endpoint_history=[], edges={})
        for edge, owner_topic, identity, node, x, topic_gid in (
                ("map->odom", "graph", "11" * 24, "/fixture_graph_node", .1, "33" * 24),
                ("odom->rig_link", "odom", "22" * 24, "/fixture_icp_node", .2, "44" * 24)):
            sample = dict(stamp_ns=self.reference, received_unix_ns=now - 5_000_000_000,
                          publisher_gid=identity, transform_xyz_xyzw=[x, 0, 0, 0, 0, 0, 1])
            python_sample = dict(sample, publisher_gid=None, static=False)
            self.recorder.tf[edge] = [python_sample]
            self.recorder.endpoints["tf"][identity] = dict(node=node)
            self.recorder.endpoints[owner_topic][topic_gid] = dict(node=node)
            self.document["endpoint_history"].append(dict(publisher_gid=identity, node=node))
            self.document["edges"][edge] = dict(transform_count=1, publisher_gids={identity: 1}, observations=[sample],
                                                min_stamp_ns=self.reference, max_stamp_ns=self.reference)
        self.write()

    def write(self):
        self.path.write_text(json.dumps(self.document), encoding="utf-8")

    def test_native_actual_gid_can_complete_missing_python_message_info(self):
        result = observer.tf_checks(self.recorder)
        self.assertEqual({value["status"] for value in result.values()}, {"PASS"})
        self.assertEqual(result["map->odom"]["matched_accepted_stamps"], 1)
        self.assertEqual(result["map->odom"]["publisher_gids"], ["11" * 24])

    def test_pending_discovery_name_is_not_a_second_node_identity(self):
        self.document['endpoint_history'].append(dict(publisher_gid='11'*24,
            node='_NODE_NAMESPACE_UNKNOWN_/_NODE_NAME_UNKNOWN_'))
        self.write()
        self.assertEqual(observer.tf_checks(self.recorder)['map->odom']['status'], 'PASS')

    def test_only_unresolved_discovery_name_cannot_prove_ownership(self):
        self.document['endpoint_history'][0]['node'] = '_NODE_NAMESPACE_UNKNOWN_/_NODE_NAME_UNKNOWN_'
        self.write()
        self.assertEqual(observer.tf_checks(self.recorder)['map->odom']['status'], 'FAIL')

    def test_conflicting_resolved_names_for_one_gid_remain_failure(self):
        self.document['endpoint_history'].append(dict(publisher_gid='11'*24,node='/different_node'))
        self.write()
        self.assertEqual(observer.tf_checks(self.recorder)['map->odom']['status'], 'FAIL')

    def test_same_label_with_different_actual_stamp_is_not_evidence(self):
        edge = self.document["edges"]["map->odom"]
        edge["observations"][0]["stamp_ns"] += 1
        edge["min_stamp_ns"] += 1
        edge["max_stamp_ns"] += 1
        self.write()
        self.assertEqual(observer.tf_checks(self.recorder)["map->odom"]["status"], "BLOCKED")

    def test_native_transform_must_match_independent_actual_graph_and_odom(self):
        self.recorder.graphs[1]["nodes"][0]["pose_xyz_xyzw"][0] = .31
        self.assertEqual(observer.tf_checks(self.recorder)["map->odom"]["status"], "FAIL")

    def test_second_native_edge_publisher_is_a_failure(self):
        edge = self.document["edges"]["map->odom"]
        second = copy.deepcopy(edge["observations"][0])
        second["publisher_gid"] = "55" * 24
        edge["observations"].append(second)
        edge["publisher_gids"][second["publisher_gid"]] = 1
        edge["transform_count"] += 1
        self.write()
        self.assertEqual(observer.tf_checks(self.recorder)["map->odom"]["status"], "FAIL")

    def test_same_session_label_from_other_host_is_refused(self):
        self.document["hostname"] = "DIFFERENT_FIXTURE_HOST"
        self.write()
        self.assertEqual(observer.tf_checks(self.recorder)["map->odom"]["status"], "FAIL")

    def test_same_gid_node_binding_must_match_actual_source_endpoint(self):
        self.recorder.endpoints["graph"]["33" * 24]["node"] = "/unexpected_fixture_node"
        self.assertEqual(observer.tf_checks(self.recorder)["map->odom"]["status"], "FAIL")

    def test_native_pass_cannot_hide_observed_static_motion_transform(self):
        self.recorder.tf["map->odom"][0]["static"] = True
        self.assertEqual(observer.tf_checks(self.recorder)["map->odom"]["status"], "FAIL")

    def test_pending_native_evidence_is_blocked(self):
        self.recorder.config["_observer_tf_evidence"] = str(Path(self.directory.name) / "not_created_yet.json")
        self.assertEqual(observer.tf_checks(self.recorder)["map->odom"]["status"], "BLOCKED")

    def test_incomplete_native_window_remains_blocked(self):
        self.document["status"] = "BLOCKED"
        self.document["interrupted"] = True
        self.write()
        self.assertEqual(observer.tf_checks(self.recorder)["map->odom"]["status"], "BLOCKED")


if __name__ == "__main__":
    unittest.main()
