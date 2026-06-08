import sys
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nav2_msgs.msg import CostmapFilterInfo  # noqa: E402
from nav_msgs.msg import OccupancyGrid  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from wheelchair_navigation.semantic_map_store import SemanticMapStore  # noqa: E402
from wheelchair_navigation.semantic_keepout_node import SemanticKeepoutNode  # noqa: E402


def spin_until(executor, predicate, timeout_sec=3.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return
    raise AssertionError("timed out waiting for ROS messages")


def test_node_publishes_mask_and_retains_it_on_invalid_config(tmp_path):
    semantic_path = tmp_path / "semantic_map.yaml"
    store = SemanticMapStore(str(semantic_path))
    store.upsert_no_go_zone(
        "test-zone",
        [[1.0, 1.0], [3.0, 1.0], [3.0, 3.0], [1.0, 3.0]],
    )
    map_topic = "/test_semantic_keepout/map"
    mask_topic = "/test_semantic_keepout/mask"
    info_topic = "/test_semantic_keepout/filter_info"
    status_topic = "/test_semantic_keepout/status"

    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            f"semantic_map_path:={semantic_path}",
            "-p",
            f"map_topic:={map_topic}",
            "-p",
            f"mask_topic:={mask_topic}",
            "-p",
            f"filter_info_topic:={info_topic}",
            "-p",
            f"status_topic:={status_topic}",
            "-p",
            "reload_period_sec:=0.2",
            "-p",
            "require_filter_transform:=false",
        ]
    )
    keepout_node = SemanticKeepoutNode()
    client_node = Node("semantic_keepout_test_client")
    executor = SingleThreadedExecutor()
    executor.add_node(keepout_node)
    executor.add_node(client_node)

    latched_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    masks = []
    infos = []
    statuses = []
    client_node.create_subscription(
        OccupancyGrid, mask_topic, masks.append, latched_qos
    )
    client_node.create_subscription(
        CostmapFilterInfo, info_topic, infos.append, latched_qos
    )
    client_node.create_subscription(
        String, status_topic, statuses.append, latched_qos
    )
    map_pub = client_node.create_publisher(OccupancyGrid, map_topic, latched_qos)

    source = OccupancyGrid()
    source.header.frame_id = "map"
    source.info.width = 5
    source.info.height = 5
    source.info.resolution = 1.0
    source.info.origin.orientation.w = 1.0

    try:
        map_pub.publish(source)
        spin_until(executor, lambda: masks and infos and statuses)

        occupied = {
            (column, row)
            for row in range(5)
            for column in range(5)
            if masks[-1].data[row * 5 + column] == 100
        }
        assert occupied == {(1, 1), (2, 1), (1, 2), (2, 2)}
        assert infos[-1].type == 0
        assert infos[-1].filter_mask_topic == mask_topic
        assert statuses[-1].data.startswith("READY:")

        semantic_path.write_text("no_go_zones: [\n", encoding="utf-8")
        spin_until(
            executor,
            lambda: statuses[-1].data.startswith("ERROR:"),
        )
        assert len(masks) == 1
        assert "retained_previous=true" in statuses[-1].data
    finally:
        executor.remove_node(client_node)
        executor.remove_node(keepout_node)
        client_node.destroy_node()
        keepout_node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


def test_filter_info_waits_for_required_transform(tmp_path):
    semantic_path = tmp_path / "semantic_map.yaml"
    SemanticMapStore(str(semantic_path))
    map_topic = "/test_semantic_keepout_wait/map"
    info_topic = "/test_semantic_keepout_wait/filter_info"
    status_topic = "/test_semantic_keepout_wait/status"

    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            f"semantic_map_path:={semantic_path}",
            "-p",
            f"map_topic:={map_topic}",
            "-p",
            f"filter_info_topic:={info_topic}",
            "-p",
            f"status_topic:={status_topic}",
            "-p",
            "filter_target_frame:=missing_odom",
            "-p",
            "reload_period_sec:=0.2",
        ]
    )
    keepout_node = SemanticKeepoutNode()
    client_node = Node("semantic_keepout_wait_test_client")
    executor = SingleThreadedExecutor()
    executor.add_node(keepout_node)
    executor.add_node(client_node)
    latched_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    infos = []
    statuses = []
    client_node.create_subscription(
        CostmapFilterInfo, info_topic, infos.append, latched_qos
    )
    client_node.create_subscription(
        String, status_topic, statuses.append, latched_qos
    )
    map_pub = client_node.create_publisher(OccupancyGrid, map_topic, latched_qos)
    source = OccupancyGrid()
    source.header.frame_id = "map"
    source.info.width = 2
    source.info.height = 2
    source.info.resolution = 1.0
    source.info.origin.orientation.w = 1.0

    try:
        map_pub.publish(source)
        spin_until(
            executor,
            lambda: statuses
            and statuses[-1].data.startswith("WAITING_FOR_TRANSFORM:"),
        )
        for _ in range(5):
            executor.spin_once(timeout_sec=0.05)
        assert infos == []
    finally:
        executor.remove_node(client_node)
        executor.remove_node(keepout_node)
        client_node.destroy_node()
        keepout_node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
