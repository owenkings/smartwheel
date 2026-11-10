"""Start check: do the standing scans fit the map at the operator's start pose, and better than turned 180 deg?

Before Nav2 and the safety gate start, the shadow's supervisor takes a few scans while the chair stands at the
given initial pose and scores them on the map: the share of scan endpoints within 0.15 m of an occupied cell
("support"), best within +-0.3 m and +-10 deg of the pose (the operator places the chair by eye), and the same
for the pose turned 180 deg about the axle.

- PASS: the given heading fits (support >= 0.6) and clearly better than the turned one (by >= 0.2).
- INITIAL_POSE_MISMATCH: the given heading does not fit, or the turned one fits clearly better.
- AMBIGUOUS: both fit about as well. In a straight corridor the scans look the same both ways, so the check
  cannot tell the heading there (2026-11-10 study: 79-86 % both ways at the ZL1107 corridor start, 85 % against
  0 % at the validation recording's start in a doorway facing across the corridor).

Modes: ``block`` (default) stops the run on a mismatch and warns on AMBIGUOUS; ``strict`` stops on both;
``warn`` only reports; ``off`` skips the check.
"""
from __future__ import annotations

import math

import numpy as np

SUPPORT_M = 0.15
WINDOW_XY_M = 0.3
WINDOW_YAW_DEG = 10.0
MIN_SUPPORT = 0.6
MIN_MARGIN = 0.2
MIN_SCANS = 10
MODES = ("block", "strict", "warn", "off")
STANDING_M = 0.05
STANDING_RAD = math.radians(2.0)


def local_endpoints(scans):
    """N x 2 endpoints in the laser frame from (ranges, angle_min, angle_increment) scans; inf/nan dropped."""
    points = []
    for ranges, angle_min, increment in scans:
        ranges = np.asarray(ranges, dtype=float)
        angles = angle_min + increment * np.arange(len(ranges))
        ok = np.isfinite(ranges) & (ranges > 0.0)
        points.append(np.column_stack([ranges[ok] * np.cos(angles[ok]), ranges[ok] * np.sin(angles[ok])]))
    return np.concatenate(points) if points else np.empty((0, 2))


def best_support(scorer, points, lever, pose, span_xy=WINDOW_XY_M, span_deg=WINDOW_YAW_DEG, step_xy=0.05,
                 step_deg=1.0):
    """(support at ``pose``, best support within the window, (dx, dy, dyaw_deg) of the best) for a base pose."""
    offsets = np.arange(-span_xy, span_xy + 1e-9, step_xy)
    dx, dy = (v.ravel() for v in np.meshgrid(offsets, offsets, indexing="ij"))
    centre = int(np.argmin(np.hypot(dx, dy)))
    at_pose, best, best_offset = 0.0, -1.0, (0.0, 0.0, 0.0)
    for dyaw_deg in np.arange(-span_deg, span_deg + 1e-9, step_deg):
        yaw = pose[2] + math.radians(dyaw_deg)
        c, s = math.cos(yaw), math.sin(yaw)
        rotated = points @ np.array([[c, s], [-s, c]])
        lx = pose[0] + dx + c * lever[0] - s * lever[1]
        ly = pose[1] + dy + s * lever[0] + c * lever[1]
        values = np.empty(len(dx))
        for lo in range(0, len(dx), 64):
            xy = np.stack([rotated[None, :, 0] + lx[lo:lo + 64, None], rotated[None, :, 1] + ly[lo:lo + 64, None]],
                          axis=-1)
            values[lo:lo + 64] = (scorer.lookup(xy) <= SUPPORT_M).mean(axis=1)
        i = int(np.argmax(values))
        if values[i] > best:
            best, best_offset = float(values[i]), (float(dx[i]), float(dy[i]), float(dyaw_deg))
        if abs(dyaw_deg) < 1e-9:
            at_pose = float(values[centre])
    return at_pose, best, best_offset


def evaluate(scorer, scans, lever, pose, min_support=MIN_SUPPORT, margin=MIN_MARGIN):
    """Result dict of the start check for base pose (x, y, yaw rad) and standing scans."""
    points = local_endpoints(scans)
    result = {"pose": [float(v) for v in pose], "scans": len(scans), "endpoints": int(len(points)),
              "support_m": SUPPORT_M, "window": {"xy_m": WINDOW_XY_M, "yaw_deg": WINDOW_YAW_DEG},
              "min_support": min_support, "min_margin": margin}
    if len(scans) < MIN_SCANS or len(points) < 50:
        result["status"] = "START_CHECK_NO_DATA"
        return result
    given = best_support(scorer, points, lever, pose)
    turned = best_support(scorer, points, lever, (pose[0], pose[1], pose[2] + math.pi))
    result["given"] = {"at_pose": round(given[0], 3), "best": round(given[1], 3),
                       "best_offset": [round(v, 3) for v in given[2]]}
    result["turned_180"] = {"at_pose": round(turned[0], 3), "best": round(turned[1], 3),
                            "best_offset": [round(v, 3) for v in turned[2]]}
    if given[1] < min_support or turned[1] - given[1] >= margin:
        result["status"] = "INITIAL_POSE_MISMATCH"
    elif given[1] - turned[1] >= margin:
        result["status"] = "PASS"
    else:
        result["status"] = "AMBIGUOUS"
    return result


def stops_run(status, mode):
    """Whether the run must stop before Nav2 and the gate start."""
    if mode in ("warn", "off") or status == "PASS":
        return False
    if status == "AMBIGUOUS":
        return mode == "strict"
    return True                     # mismatch, no data, the chair moved before the check finished


def operator_message(result, mode):
    """One operator line in the field's words (directions on the panel map, never compass points)."""
    status = result["status"]
    given, turned = result.get("given", {}), result.get("turned_180", {})
    numbers = ("给定朝向贴图 %.0f%%，掉头 180° 贴图 %.0f%%" % (100 * given.get("best", 0.0), 100 * turned.get("best", 0.0))
               if given else "")
    if status == "PASS":
        return "起点检查通过：" + numbers
    if status == "AMBIGUOUS":
        return ("起点检查：这个位置看不出车头朝向（%s）。请再确认车头朝向和 --initial-pose 一致；%s"
                % (numbers, "按 strict 不放行，运行停止" if mode == "strict" else "继续运行，闸门照常放行"))
    if status == "INITIAL_POSE_MISMATCH":
        hint = "车头可能和给定朝向反了。" if turned.get("best", 0.0) - given.get("best", 0.0) >= MIN_MARGIN else ""
        if not stops_run(status, mode):
            return "起点检查没通过（%s）：%s只记录（--start-check %s）" % (numbers, hint, mode)
        return ("起点检查没通过（%s）：%s不放行，运行停止。请按给定位姿摆好车，或改 --initial-pose，再重新启动"
                % (numbers, hint))
    if status == "START_CHECK_NOT_STANDING":
        return "起点检查没做成：发完初始位姿后车动了。请车停在起点、等闸门放行后再推，重新启动"
    return "起点检查没做成：静止扫描不够（%d 帧）" % result.get("scans", 0)


def moved(a, b):
    """Whether two odometry poses (x, y, yaw) differ by more than standing allows."""
    turn = math.atan2(math.sin(a[2] - b[2]), math.cos(a[2] - b[2]))
    return math.hypot(a[0] - b[0], a[1] - b[1]) > STANDING_M or abs(turn) > STANDING_RAD
