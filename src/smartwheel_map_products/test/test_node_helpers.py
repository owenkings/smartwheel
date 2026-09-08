import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header

from smartwheel_map_products.node import (
    _backend_snapshot_status,
    _load_formal_acceptance_evidence,
    _make_xyz_cloud,
    _tf_evidence_matches,
    _validated_backend_path,
    _validated_tf_edges,
)


def _path(stamps=(1.0, 2.0)):
    message = Path()
    message.header.frame_id = "map"
    message.header.stamp.sec = 10
    for stamp in stamps:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp.sec = int(stamp)
        pose.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1.0e9))
        pose.pose.orientation.w = 1.0
        message.poses.append(pose)
    return message


def test_xyz_product_does_not_fabricate_intensity():
    message = _make_xyz_cloud(
        Header(frame_id="map"), np.asarray([[1.0, 2.0, 3.0]], dtype=np.float32)
    )
    assert [field.name for field in message.fields] == ["x", "y", "z"]


def test_backend_path_retains_full_pose_and_rejects_bad_order():
    message = _path((1.25, 2.5))
    message.poses[1].pose.position.z = 0.7
    message.poses[1].pose.orientation.x = 1.0
    message.poses[1].pose.orientation.w = 1.0
    trajectory = _validated_backend_path(message)
    assert trajectory[1][0] == pytest.approx(2.5)
    assert trajectory[1][3] == pytest.approx(0.7)
    assert trajectory[1][4] == pytest.approx(np.sqrt(0.5))

    with pytest.raises(ValueError, match="strictly increasing"):
        _validated_backend_path(_path((2.0, 1.0)))


def test_backend_snapshot_must_be_matched_fresh_and_bounded():
    trajectory = [(1.0,) + (0.0,) * 7, (2.0,) + (0.0,) * 7]
    ok, _ = _backend_snapshot_status(
        cloud_snapshot_stamp=10.0,
        path_snapshot_stamp=10.0,
        trajectory=trajectory,
        session_last_stamp=2.1,
        stopped_at_monotonic=5.0,
        cloud_received_at=6.0,
        path_received_at=6.1,
        require_fresh_after_stop=True,
        maximum_trajectory_lag_sec=2.5,
        maximum_future_sec=0.15,
    )
    assert ok

    stale, reason = _backend_snapshot_status(
        cloud_snapshot_stamp=10.0,
        path_snapshot_stamp=10.0,
        trajectory=trajectory,
        session_last_stamp=2.1,
        stopped_at_monotonic=5.0,
        cloud_received_at=4.0,
        path_received_at=6.1,
        require_fresh_after_stop=True,
        maximum_trajectory_lag_sec=2.5,
        maximum_future_sec=0.15,
    )
    assert not stale
    assert "after session stop" in reason

    mismatch, reason = _backend_snapshot_status(
        cloud_snapshot_stamp=10.0,
        path_snapshot_stamp=11.0,
        trajectory=trajectory,
        session_last_stamp=2.1,
        stopped_at_monotonic=5.0,
        cloud_received_at=6.0,
        path_received_at=6.1,
        require_fresh_after_stop=True,
        maximum_trajectory_lag_sec=2.5,
        maximum_future_sec=0.15,
    )
    assert not mismatch
    assert "different snapshots" in reason

    invalid, reason = _backend_snapshot_status(
        cloud_snapshot_stamp=float("nan"),
        path_snapshot_stamp=float("nan"),
        trajectory=trajectory,
        session_last_stamp=2.1,
        stopped_at_monotonic=5.0,
        cloud_received_at=6.0,
        path_received_at=6.1,
        require_fresh_after_stop=True,
        maximum_trajectory_lag_sec=2.5,
        maximum_future_sec=0.15,
    )
    assert not invalid
    assert "finite and positive" in reason


def test_tf_chain_and_evidence_must_name_each_actual_edge():
    expected = _validated_tf_edges(
        {
            "global_correction": "map->camera_init",
            "local_odometry": "camera_init->body",
            "body_bridge": "body->base_link",
        }
    )
    evidence = {
        "status": "PASS",
        "edges": {
            role: {"edge": edge, "publishers": 1}
            for role, edge in expected.items()
        },
    }
    assert _tf_evidence_matches(evidence, expected)
    evidence["edges"]["local_odometry"]["edge"] = "odom->base_link"
    assert not _tf_evidence_matches(evidence, expected)

    with pytest.raises(ValueError, match="must end at base_link"):
        _validated_tf_edges(
            {
                "global_correction": "map->camera_init",
                "local_odometry": "camera_init->body",
                "body_bridge": "body->wrong_base",
            }
        )


def test_formal_evidence_loader_reads_replaced_snapshot(tmp_path):
    path = tmp_path / "formal.json"
    path.write_text('{"schema_version":1,"generation":1}', encoding="utf-8")
    assert _load_formal_acceptance_evidence(str(path))["generation"] == 1
    path.write_text('{"schema_version":1,"generation":2}', encoding="utf-8")
    assert _load_formal_acceptance_evidence(str(path))["generation"] == 2
