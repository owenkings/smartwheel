import json
import sys
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from wheelchair_navigation.startup_localization_node import (  # noqa: E402
    StartupLocalizationNode,
)


def spin_until(executor, predicate, timeout_sec=3.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return
    raise AssertionError("timed out waiting for ROS messages")


def latched_qos():
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def destroy_nodes(executor, *nodes):
    for node in nodes:
        executor.remove_node(node)
        node.destroy_node()
    executor.shutdown()
    if rclpy.ok():
        rclpy.shutdown()


def test_disabled_mode_publishes_status_without_initial_pose():
    initial_topic = "/test_startup_localization_disabled/initialpose"
    status_topic = "/test_startup_localization_disabled/status"
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "mode:=disabled",
            "-p",
            f"initialpose_topic:={initial_topic}",
            "-p",
            f"status_topic:={status_topic}",
        ]
    )
    localization_node = StartupLocalizationNode()
    client_node = Node("startup_localization_disabled_test_client")
    executor = SingleThreadedExecutor()
    executor.add_node(localization_node)
    executor.add_node(client_node)
    statuses = []
    initial_poses = []
    client_node.create_subscription(String, status_topic, statuses.append, latched_qos())
    client_node.create_subscription(
        PoseWithCovarianceStamped, initial_topic, initial_poses.append, 10
    )

    try:
        spin_until(executor, lambda: statuses)
        assert json.loads(statuses[-1].data)["state"] == "DISABLED"
        for _ in range(5):
            executor.spin_once(timeout_sec=0.05)
        assert initial_poses == []
    finally:
        destroy_nodes(executor, client_node, localization_node)


def test_fixed_mode_retries_until_amcl_acknowledges():
    prefix = "/test_startup_localization_fixed"
    initial_topic = f"{prefix}/initialpose"
    amcl_topic = f"{prefix}/amcl_pose"
    status_topic = f"{prefix}/status"
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "mode:=fixed",
            "-p",
            "fixed_x:=1.25",
            "-p",
            "fixed_y:=-0.5",
            "-p",
            "fixed_yaw:=0.0",
            "-p",
            "startup_delay_sec:=0.0",
            "-p",
            "retry_period_sec:=0.2",
            "-p",
            f"initialpose_topic:={initial_topic}",
            "-p",
            f"amcl_pose_topic:={amcl_topic}",
            "-p",
            f"status_topic:={status_topic}",
        ]
    )
    localization_node = StartupLocalizationNode()
    client_node = Node("startup_localization_fixed_test_client")
    executor = SingleThreadedExecutor()
    executor.add_node(localization_node)
    executor.add_node(client_node)
    statuses = []
    initial_poses = []
    client_node.create_subscription(String, status_topic, statuses.append, latched_qos())
    client_node.create_subscription(
        PoseWithCovarianceStamped, initial_topic, initial_poses.append, 10
    )
    amcl_pub = client_node.create_publisher(PoseWithCovarianceStamped, amcl_topic, 10)

    try:
        spin_until(executor, lambda: initial_poses)
        assert initial_poses[-1].header.frame_id == "map"
        assert initial_poses[-1].pose.pose.position.x == pytest.approx(1.25)
        assert initial_poses[-1].pose.pose.position.y == pytest.approx(-0.5)

        acknowledged = PoseWithCovarianceStamped()
        acknowledged.header.frame_id = "map"
        acknowledged.pose.pose.position.x = 1.25
        acknowledged.pose.pose.position.y = -0.5
        acknowledged.pose.pose.orientation.w = 1.0
        amcl_pub.publish(acknowledged)
        spin_until(
            executor,
            lambda: statuses and json.loads(statuses[-1].data)["state"] == "LOCALIZED",
        )
        status = json.loads(statuses[-1].data)
        assert status["acknowledged"] is True
        assert status["attempts"] >= 1
    finally:
        destroy_nodes(executor, client_node, localization_node)


def test_external_anchor_rejects_non_map_frame_then_starts_localization():
    prefix = "/test_startup_localization_anchor"
    initial_topic = f"{prefix}/initialpose"
    anchor_topic = f"{prefix}/anchor"
    status_topic = f"{prefix}/status"
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "mode:=external_anchor",
            "-p",
            f"initialpose_topic:={initial_topic}",
            "-p",
            f"anchor_topic:={anchor_topic}",
            "-p",
            f"status_topic:={status_topic}",
        ]
    )
    localization_node = StartupLocalizationNode()
    client_node = Node("startup_localization_anchor_test_client")
    executor = SingleThreadedExecutor()
    executor.add_node(localization_node)
    executor.add_node(client_node)
    statuses = []
    initial_poses = []
    client_node.create_subscription(String, status_topic, statuses.append, latched_qos())
    client_node.create_subscription(
        PoseWithCovarianceStamped, initial_topic, initial_poses.append, 10
    )
    anchor_pub = client_node.create_publisher(
        PoseWithCovarianceStamped, anchor_topic, 10
    )

    try:
        spin_until(
            executor,
            lambda: statuses
            and json.loads(statuses[-1].data)["state"] == "WAITING_FOR_ANCHOR",
        )
        invalid = PoseWithCovarianceStamped()
        invalid.header.frame_id = "odom"
        invalid.pose.pose.orientation.w = 1.0
        anchor_pub.publish(invalid)
        spin_until(
            executor,
            lambda: "rejected anchor" in json.loads(statuses[-1].data)["reason"],
        )
        assert initial_poses == []

        valid = PoseWithCovarianceStamped()
        valid.header.frame_id = "map"
        valid.pose.pose.position.x = -1.5
        valid.pose.pose.position.y = 0.25
        valid.pose.pose.orientation.w = 1.0
        anchor_pub.publish(valid)
        spin_until(executor, lambda: initial_poses)
        assert initial_poses[-1].pose.pose.position.x == pytest.approx(-1.5)
        assert initial_poses[-1].pose.pose.position.y == pytest.approx(0.25)
    finally:
        destroy_nodes(executor, client_node, localization_node)
