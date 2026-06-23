"""Tests for cloud_to_occupancy_grid_node accumulation (D038) + stable origin (D039)."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_accumulation_persists_obstacle_after_it_leaves_view():
    """D038: an obstacle seen in frame 1 must remain occupied in frame 2 even if
    frame 2 no longer contains it."""
    import rclpy
    from wheelchair_3d_mapping.cloud_to_occupancy_grid_node import CloudToOccupancyGridNode

    rclpy.init()
    try:
        node = CloudToOccupancyGridNode()
        node.accumulate = True
        node.inflation = 0.0
        node._offsets = [(0, 0)]
        node._publish = lambda grid, ox, oy, w, h: None

        node._tick_accumulate(np.array([[1.0, 0.0, 0.5]], dtype=np.float64))
        occ_cells_1 = int((node._acc == node.v_occ).sum())
        assert occ_cells_1 >= 1

        node._tick_accumulate(np.array([[2.0, 1.0, 0.0]], dtype=np.float64))
        # obstacle from frame 1 must still be occupied (sticky)
        assert int((node._acc == node.v_occ).sum()) >= occ_cells_1
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_origin_stable_across_frames():
    """D039: accumulate mode keeps a fixed origin/size across frames."""
    import rclpy
    from wheelchair_3d_mapping.cloud_to_occupancy_grid_node import CloudToOccupancyGridNode

    rclpy.init()
    try:
        node = CloudToOccupancyGridNode()
        node.accumulate = True
        node._offsets = [(0, 0)]
        node._publish = lambda *a, **k: None

        node._tick_accumulate(np.array([[1.0, 0.0, 0.5]], dtype=np.float64))
        origin1, wh1 = node._acc_origin, node._acc_wh
        node._tick_accumulate(np.array([[5.0, 5.0, 0.5]], dtype=np.float64))
        assert node._acc_origin == origin1
        assert node._acc_wh == wh1
        node.destroy_node()
    finally:
        rclpy.shutdown()
