"""Saved-map format checks and a real ROS 2 late-subscriber test when available."""

import copy
from pathlib import Path
import tempfile
import time
import unittest
import uuid

import numpy as np

from wc_maps.builder import MapError
from wc_maps.ros_view import quaternion_xyzw, read_ascii_ply, trajectory_segments, view_payload
from wc_maps.store import MapStore

try:
    from .test_builder import graph_fixture, transform
    from .test_store import snapshot_fixture
except ImportError:
    from test_builder import graph_fixture, transform
    from test_store import snapshot_fixture


class ViewFormatTests(unittest.TestCase):
    def test_quaternion_preserves_identity_and_half_turn(self):
        self.assertTrue(np.allclose(quaternion_xyzw(transform()), [0, 0, 0, 1]))
        for axis in range(3):
            matrix = np.eye(4)
            matrix[:3, :3] = -np.eye(3)
            matrix[axis, axis] = 1
            actual = np.abs(quaternion_xyzw(matrix))
            expected = np.zeros(4)
            expected[axis] = 1
            self.assertTrue(np.allclose(actual, expected))

    def test_trajectory_segments_never_cross_epoch_break(self):
        graph = graph_fixture()
        graph["nodes"] = [{"node_id": index, "odom_epoch": epoch, "T_map_node": transform(index)}
                          for index, epoch in ((1, 0), (2, 0), (3, 1), (4, 1), (5, 0))]
        trajectory = {"poses": [{"node_id": node["node_id"], "T_map_rig": node["T_map_node"]} for node in graph["nodes"]]}
        segments = trajectory_segments(graph, trajectory)
        self.assertEqual([[pose["node_id"] for pose in part] for part in segments], [[1, 2], [3, 4], [5]])

    def test_ply_reader_rejects_wrong_revision_and_extra_points(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "test.ply"
            text = "ply\nformat ascii 1.0\ncomment graph_revision 3\nelement vertex 1\nproperty double x\nproperty double y\nproperty double z\nend_header\n1 2 3\n"
            path.write_text(text)
            self.assertTrue(np.array_equal(read_ascii_ply(path, 3), [[1, 2, 3]]))
            with self.assertRaisesRegex(MapError, "revision"):
                read_ascii_ply(path, 4)
            path.write_text(text + "4 5 6\n")
            with self.assertRaisesRegex(MapError, "extra"):
                read_ascii_ply(path, 3)

    def test_saved_payload_keeps_geometry_and_no_current_pose(self):
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            snapshot_fixture(base / "snapshot")
            store = MapStore(base / "maps")
            store.save_snapshot(base / "snapshot", "synthetic", "v001")
            loaded = view_payload(store, "synthetic", "v001")
            self.assertEqual(loaded["state"], "LOADED_VIEW_ONLY")
            self.assertIsNone(loaded["current_pose"])
            self.assertTrue(np.allclose(loaded["points"], [[-2.5, .5, .5], [3.5, 3.5, .5]]))

    def test_rviz_map_display_and_private_goal_tool(self):
        try:
            import yaml
        except ImportError:
            self.skipTest("PyYAML unavailable; RViz config structure not validated")
        directory = Path(__file__).resolve().parents[2] / "config" / "rviz"
        for filename, view_class in (("mapping_3d.rviz", "rviz_default_plugins/Orbit"),
                                     ("mapping_2d.rviz", "rviz_default_plugins/TopDownOrtho")):
            with self.subTest(filename=filename):
                data = yaml.safe_load((directory / filename).read_text(encoding="utf-8"))
                manager = data["Visualization Manager"]
                self.assertEqual(manager["Global Options"]["Fixed Frame"], "map")
                self.assertEqual(manager["Views"]["Current"]["Class"], view_class)
                maps = [display for display in manager["Displays"] if display["Class"] == "rviz_default_plugins/Map"]
                self.assertEqual(len(maps), 1)
                self.assertEqual(maps[0]["Topic"]["Value"], "/wc_mapping/map/grid")
                self.assertEqual(maps[0]["Topic"]["Durability Policy"], "Transient Local")
                goals = [tool for tool in manager["Tools"] if tool["Class"] == "rviz_default_plugins/SetGoal"]
                self.assertEqual(goals[0]["Topic"]["Value"], "/wc_mapping/ui/goal_annotation")
                self.assertFalse(any("nav2" in tool["Class"] or "SetInitialPose" in tool["Class"] for tool in manager["Tools"]))


class RosLateSubscriberTests(unittest.TestCase):
    def test_transient_local_receives_map_after_initial_publication(self):
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
            from wc_interfaces.msg import MapStatus  # noqa: F401
            from wc_maps.ros_view import create_viewer_node
        except ImportError as exc:
            self.skipTest(f"ROS 2 and generated wc_interfaces required: {exc}")
        context = Context()
        rclpy.init(context=context)
        executor = SingleThreadedExecutor(context=context)
        viewer = subscriber = None
        try:
            with tempfile.TemporaryDirectory(prefix="wc-ros-view-synthetic-") as temporary:
                base = Path(temporary)
                snapshot_fixture(base / "snapshot")
                store = MapStore(base / "maps")
                store.save_snapshot(base / "snapshot", "synthetic", "v001")
                namespace = "/wc_maps_test_" + uuid.uuid4().hex[:12]
                viewer = create_viewer_node(store, "synthetic", "v001", namespace=namespace, context=context)
                executor.add_node(viewer)
                # All messages have already been published with zero subscribers;
                # the viewer has no periodic republish timer to mask QoS failures.
                deadline = time.monotonic() + 0.5
                while time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.05)
                subscriber = Node("late_map_subscriber", namespace=namespace, context=context)
                received = {}
                qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
                subscriptions = []
                for topic, message in viewer.snapshot_messages.items():
                    subscriptions.append(subscriber.create_subscription(type(message), topic,
                                         lambda message, key=topic: received.__setitem__(key, message), qos))
                executor.add_node(subscriber)
                deadline = time.monotonic() + 15
                while set(received) != set(viewer.snapshot_messages) and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.1)
                self.assertEqual(set(received), set(viewer.snapshot_messages))
                self.assertEqual(received["status"].state, "LOADED_VIEW_ONLY")
                self.assertFalse(received["status"].tracking_valid)
                self.assertFalse(received["status"].left_fresh)
                self.assertFalse(received["status"].navigation_validated)
                self.assertEqual(received["map/grid"].header.frame_id, "map")
                self.assertEqual(received["map/grid"].header.stamp.sec, 0)
                self.assertEqual(received["map/path"].poses[0].pose.orientation.w, 1.0)
                cloud = received["map/cloud"]
                points = np.frombuffer(bytes(cloud.data), dtype="<f4").reshape(-1, 3)
                self.assertTrue(np.allclose(points, [[-2.5, .5, .5], [3.5, 3.5, .5]]))
                self.assertEqual(cloud.point_step, 12)
                self.assertEqual(len(received["map/grid"].data), received["map/grid"].info.width * received["map/grid"].info.height)
                for publisher in viewer.snapshot_publishers.values():
                    self.assertEqual(publisher.qos_profile.durability, DurabilityPolicy.TRANSIENT_LOCAL)
                    self.assertEqual(publisher.qos_profile.reliability, ReliabilityPolicy.RELIABLE)
                topics = {topic for topic, _ in viewer.get_publisher_names_and_types_by_node("saved_map_viewer", namespace)}
                self.assertNotIn("/tf", topics)
                self.assertNotIn("/tf_static", topics)
                self.assertFalse(any("cmd_vel" in topic or "goal_pose" in topic for topic in topics))
        finally:
            executor.shutdown()
            if subscriber is not None:
                subscriber.destroy_node()
            if viewer is not None:
                viewer.destroy_node()
            rclpy.shutdown(context=context)


if __name__ == "__main__":
    unittest.main()
