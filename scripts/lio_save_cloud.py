#!/usr/bin/env python3
"""Accumulate FAST-LIO /cloud_registered frames into ONE merged PLY.

Decoupled from FAST-LIO's internal PCD save (which depends on ROOT_DIR/pcd_save_en
quirks): we just subscribe to the registered world-frame cloud and voxel-merge
all frames, then write a binary PLY (x,y,z,intensity) that CloudCompare opens.

Usage:
  python3 scripts/lio_save_cloud.py <out.ply> [--seconds N] [--voxel 0.05] [--topic /cloud_registered]
"""
import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def voxel_unique(xyzi, leaf):
    if leaf <= 0 or xyzi.shape[0] == 0:
        return xyzi
    keys = np.floor(xyzi[:, :3] / leaf).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return xyzi[np.sort(idx)]


def write_ply(path, xyzi):
    n = xyzi.shape[0]
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        "comment FAST-LIO accumulated cloud (xyz+intensity)\n"
        f"element vertex {n}\n"
        "property float x\nproperty float y\nproperty float z\nproperty float intensity\n"
        "end_header\n"
    ).encode()
    with open(path, "wb") as f:
        f.write(header)
        f.write(np.ascontiguousarray(xyzi, dtype="<f4").tobytes())


class Accum(Node):
    def __init__(self, topic, voxel):
        super().__init__("lio_save_cloud")
        self.voxel = voxel
        self.buf = []
        self.frames = 0
        self.create_subscription(PointCloud2, topic, self._cb, qos_profile_sensor_data)

    def _cb(self, msg):
        names = [f.name for f in msg.fields]
        want = ("x", "y", "z", "intensity") if "intensity" in names else ("x", "y", "z")
        arr = point_cloud2.read_points(msg, field_names=want, skip_nans=True)
        if arr is None or len(arr) == 0:
            return
        xyz = np.column_stack((arr["x"], arr["y"], arr["z"])).astype(np.float32)
        inten = arr["intensity"].astype(np.float32) if "intensity" in names else np.zeros(len(xyz), np.float32)
        self.buf.append(np.column_stack([xyz, inten]))
        self.frames += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--voxel", type=float, default=0.05)
    ap.add_argument("--topic", default="/cloud_registered")
    args = ap.parse_args()

    rclpy.init()
    node = Accum(args.topic, args.voxel)
    t_end = time.time() + args.seconds
    while rclpy.ok() and time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not node.buf:
        print(f"ERROR: no frames on {args.topic} in {args.seconds}s", file=sys.stderr)
        node.destroy_node(); rclpy.shutdown(); return 1

    allpts = np.vstack(node.buf)
    merged = voxel_unique(allpts, args.voxel)
    write_ply(args.out, merged)
    print(f"wrote {args.out}: {merged.shape[0]} points from {node.frames} frames (voxel {args.voxel})")
    node.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
