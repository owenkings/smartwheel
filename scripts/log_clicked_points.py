#!/usr/bin/env python3
"""Log RViz 'Publish Point' clicks (/clicked_point) as "x y z" for the PnP 3D side.

Usage: log_clicked_points.py --out pts3d_left.txt
Set RViz Fixed Frame = base_link first, so points are in base_link (matches
/points_merged and the colorizer lidar_frame). Click the SAME features, in the
SAME order, as in pick_image_points.
"""
import argparse

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


class Logger(Node):
    def __init__(self, out):
        super().__init__("log_clicked_points")
        self.out = out
        self.create_subscription(PointStamped, "/clicked_point", self._cb, 10)
        self.get_logger().info("Use RViz 'Publish Point' tool; each click prints x y z")

    def _cb(self, msg):
        p = msg.point
        line = f"{p.x:.4f} {p.y:.4f} {p.z:.4f}"
        self.get_logger().info(f"[{msg.header.frame_id}] {line}")
        if self.out:
            open(self.out, "a").write(line + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    rclpy.init()
    node = Logger(a.out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
