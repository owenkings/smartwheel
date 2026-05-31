#!/usr/bin/env python3
"""Click features in a camera image to record pixel (u,v) for PnP extrinsic.

Usage: pick_image_points.py --topic /camera/left/image_raw --out pts2d_left.txt
Left-click a feature -> prints/append "u v". Press q to quit. Pick the SAME
features (in the same order) that you click in RViz for the 3D side.
"""
import argparse

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def decode(msg):
    buf = np.frombuffer(msg.data, np.uint8)
    ch = 1 if msg.encoding == "mono8" else 3
    img = buf[: msg.height * msg.step].reshape(msg.height, msg.step)
    img = img[:, : msg.width * ch].reshape(msg.height, msg.width, ch)
    if msg.encoding == "rgb8":
        img = img[:, :, ::-1]
    if ch == 1:
        return cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_GRAY2BGR)
    return np.ascontiguousarray(img[:, :, :3])


class Picker(Node):
    def __init__(self, topic, out):
        super().__init__("pick_image_points")
        self.frame = None
        self.out = out
        self.create_subscription(Image, topic, self._cb, 10)
        cv2.namedWindow("pick")
        cv2.setMouseCallback("pick", self._click)

    def _cb(self, msg):
        self.frame = decode(msg)

    def _click(self, event, x, y, *_):
        if event == cv2.EVENT_LBUTTONDOWN:
            print(f"{x} {y}", flush=True)
            if self.out:
                open(self.out, "a").write(f"{x} {y}\n")

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.frame is not None:
                cv2.imshow("pick", self.frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", required=True)
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    rclpy.init()
    node = Picker(a.topic, a.out)
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
