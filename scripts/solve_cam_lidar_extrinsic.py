#!/usr/bin/env python3
"""Solve T_camera_optical_from_lidar via PnP for rgb_colorizer.yaml.

Usage: solve_cam_lidar_extrinsic.py <camera_calib.yaml> <correspondences.txt>

correspondences.txt: one per line "u v x y z" (pixel u,v <-> lidar/base_link
point x,y,z). Use >=6 well-spread points at various depths. solvePnP gives
camera-from-lidar directly, which is exactly what the colorizer needs.
"""
import math
import sys

import cv2
import numpy as np
import yaml


def load_k_d(path):
    d = yaml.safe_load(open(path))
    k = np.array(d["camera_matrix"]["data"], float).reshape(3, 3)
    dist = np.array(d["distortion_coefficients"]["data"], float)
    return k, dist


def rot_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    if i == 0:
        s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    if i == 1:
        s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s)


def main():
    if len(sys.argv) < 3:
        print("usage: solve_cam_lidar_extrinsic.py <camera_calib.yaml> <correspondences.txt>")
        return
    k, dist = load_k_d(sys.argv[1])
    rows = [r.split() for r in open(sys.argv[2]) if r.strip() and not r.strip().startswith("#")]
    pts2d = np.array([[float(r[0]), float(r[1])] for r in rows], float)
    pts3d = np.array([[float(r[2]), float(r[3]), float(r[4])] for r in rows], float)
    if len(pts2d) < 6:
        print(f"WARNING: only {len(pts2d)} correspondences (>=6 recommended)")
    ok, rvec, tvec = cv2.solvePnP(pts3d, pts2d, k, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    R, _ = cv2.Rodrigues(rvec)
    proj, _ = cv2.projectPoints(pts3d, rvec, tvec, k, dist)
    err = float(np.mean(np.linalg.norm(proj.reshape(-1, 2) - pts2d, axis=1)))
    q = rot_to_quat(R)
    t = tvec.flatten()
    print(f"# T_camera_optical_from_lidar  (mean reprojection error: {err:.2f} px, n={len(rows)})")
    print(f"cam_lidar_translation: [{t[0]:.5f}, {t[1]:.5f}, {t[2]:.5f}]")
    print(f"cam_lidar_quaternion: [{q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}, {q[3]:.6f}]")


if __name__ == "__main__":
    main()
