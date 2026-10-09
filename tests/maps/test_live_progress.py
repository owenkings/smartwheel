"""Deterministic sustained-input progress and common-revision publication tests."""

from queue import Queue
import threading
import time

import pytest

from wc_maps.live_map import LatestRebuildWorker


def await_result(worker):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        result = worker.poll()
        if result is not None:
            return result
        time.sleep(0.001)
    raise AssertionError("worker produced no complete result")


def test_continuous_faster_submissions_publish_completed_revisions_then_catch_latest():
    entered = Queue()
    release = {revision: threading.Event() for revision in range(1, 32)}
    ran = []

    def build(revision):
        ran.append(revision)
        entered.put(revision)
        if not release[revision].wait(5):
            raise AssertionError("controlled build was not released")
        # This fixture tests worker scheduling only, not geometry correctness.
        return {"revision": revision}

    worker = LatestRebuildWorker(build)
    try:
        worker.submit(1)
        current = entered.get(timeout=5)
        completed = []
        for latest in range(6, 32, 5):
            # Five newer requests arrive before each running build completes.
            # Events, not scheduler timing, enforce sustained slower rebuilding.
            for revision in range(current + 1, latest + 1):
                worker.submit(revision)
            release[current].set()
            assert entered.get(timeout=5) == latest
            result = worker.poll()
            assert result == (current, {"revision": current}, None)
            completed.append(result[1]["revision"])
            assert worker.poll() is None
            current = latest
        assert completed == [1, 6, 11, 16, 21, 26]
        assert ran == [1, 6, 11, 16, 21, 26, 31]
        release[current].set()
        assert await_result(worker) == (31, {"revision": 31}, None)
    finally:
        for event in release.values():
            event.set()
        assert worker.close()


def test_new_submission_keeps_unpolled_completed_map_and_error_is_not_hidden():
    entered = Queue()
    release = {revision: threading.Event() for revision in (1, 2, 3)}

    def build(revision):
        entered.put(revision)
        if not release[revision].wait(5):
            raise AssertionError("controlled build was not released")
        if revision == 2:
            raise ValueError("original frame integrity failure")
        return {"revision": revision}

    worker = LatestRebuildWorker(build)
    try:
        worker.submit(1)
        assert entered.get(timeout=5) == 1
        worker.submit(2)
        release[1].set()
        assert entered.get(timeout=5) == 2
        worker.submit(3)
        assert worker.poll() == (1, {"revision": 1}, None)
        release[2].set()
        assert entered.get(timeout=5) == 3
        assert worker.poll() == (2, None, "original frame integrity failure")
        release[3].set()
        assert await_result(worker) == (3, {"revision": 3}, None)
    finally:
        for event in release.values():
            event.set()
        assert worker.close()


def test_older_complete_map_publishes_one_revision_with_explicit_lag(tmp_path):
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("wc_interfaces.msg")
    import copy
    from unittest.mock import patch
    from rclpy.context import Context
    from wc_interfaces.msg import GraphSnapshot, MapStatus
    from wc_maps.live_map import create_live_map_node
    from wc_maps.builder import build_map
    from .test_builder import configuration, graph_fixture, observations_fixture, transform

    context = Context()
    rclpy.init(context=context)
    node = None
    try:
        session = {"session_id": "synthetic-session", "source_mode": "synthetic", "graph_epoch": "epoch",
                   "session_root": str(tmp_path), "map_config": configuration(),
                   "calibration": {"source_mode": "synthetic", "calibration_id": "fixture"}}
        node = create_live_map_node(session, context=context)
        captured = {}

        class Publisher:
            def __init__(self, topic):
                self.topic = topic

            def publish(self, message):
                captured[self.topic] = message

        node.map_publishers = {topic: Publisher(topic) for topic in node.map_publishers}
        request = GraphSnapshot()
        request.session_id, request.source_mode, request.graph_epoch = session["session_id"], "synthetic", "epoch"
        request.revision = 3
        request.header.frame_id = "map"
        request.header.stamp.sec = 3
        node.requested = request
        frontend = MapStatus()
        frontend.session_id, frontend.source_mode = session["session_id"], "synthetic"
        frontend.latest_accepted_time_ns = 3_000_000_000
        frontend.left_fresh = frontend.right_fresh = frontend.calibration_valid = frontend.time_valid = frontend.tracking_valid = True
        node.frontend_status(frontend)

        def assembled(revision):
            graph = graph_fixture()
            graph.update(graph_epoch="epoch", revision=revision, frame_id="map", stamp_ns=revision * 1_000_000_000)
            graph["nodes"][0]["T_map_node"] = transform(x=revision)
            frames = observations_fixture()
            mapped = build_map(graph, frames, configuration())
            trajectory = {"poses": [{"node_id": 1, "T_map_rig": graph["nodes"][0]["T_map_node"]}]}
            return {"graph": graph, "mapped": mapped, "trajectory": trajectory}

        older = assembled(1)
        with patch.object(node.worker, "poll", return_value=(1, older, None)):
            node.frontend_status(frontend)
            node.tick()
        assert {captured[topic].header.stamp.sec for topic in ("map/cloud", "map/grid", "map/path")} == {1}
        status = captured["status"]
        assert status.graph_revision == 3 and status.completed_map_revision == 1
        assert not status.consistent_snapshot and status.rebuilding
        assert status.state == "REBUILDING" and "MAP_BEHIND_GRAPH" in status.pause_reasons
        assert captured["map/path"].poses[0].pose.position.x == 1.0

        latest = assembled(3)
        with patch.object(node.worker, "poll", return_value=(3, latest, None)):
            node.frontend_status(frontend)
            node.tick()
        assert {captured[topic].header.stamp.sec for topic in ("map/cloud", "map/grid", "map/path")} == {3}
        status = captured["status"]
        assert status.graph_revision == status.completed_map_revision == 3
        assert status.consistent_snapshot and not status.rebuilding and status.state == "MAPPING_DUAL"
        assert "MAP_BEHIND_GRAPH" not in status.pause_reasons
        assert captured["map/path"].poses[0].pose.position.x == 3.0
        # A duplicate/older completion cannot roll displayed geometry backwards.
        with patch.object(node.worker, "poll", return_value=(1, copy.deepcopy(older), None)):
            node.frontend_status(frontend)
            node.tick()
        assert captured["map/cloud"].header.stamp.sec == 3
    finally:
        if node is not None:
            node.destroy_node()
        if context.ok():
            rclpy.shutdown(context=context)
