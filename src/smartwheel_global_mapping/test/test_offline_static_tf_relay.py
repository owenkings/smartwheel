from types import SimpleNamespace

from smartwheel_global_mapping.offline_static_tf_relay_node import (
    merge_static_transforms,
)


def transform(parent: str, child: str):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=parent), child_frame_id=child
    )


def test_merge_preserves_order_and_replaces_duplicate():
    old = transform("map", "odom")
    replacement = transform("/map", "/odom")
    other = transform("body", "base_link")
    merged = merge_static_transforms([old], [replacement, other])
    assert merged == [replacement, other]


def test_merge_accepts_empty_batches():
    item = transform("base_link", "lidar")
    assert merge_static_transforms([], [item]) == [item]
    assert merge_static_transforms([item], []) == [item]
