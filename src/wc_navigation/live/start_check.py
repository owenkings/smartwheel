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

Start-pose correction (``--start-pose-correction apply``, off by default; user 2026-11-12, D.md 11-12 00:15):
on the same standing scans, ``correction`` moves the given position, never the heading, when the given pose fits
poorly (support < 0.5) and some position within +-0.4 m along and across the given heading fits at that heading
(>= 0.6; in every case of the 11-11 study where the rule applied this held, 0.76-0.86, also with the heading 3 deg
off). Of the positions whose support is within 0.01 of the best, it takes the one nearest the given position. In a
corridor the support is flat along the corridor and sharp across it, so this rule fixes the offset across the
corridor and leaves the along-corridor position as given (taking the best instead added 0.30-0.47 m along the
corridor in the 11-11 study); the supervisor then sends the corrected pose to AMCL before Nav2 starts.
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
CORRECTION_MODES = ("off", "apply")
CORRECTION_SPAN_M = 0.4           # along and across the given heading
CORRECTION_STEP_M = 0.025
CORRECTION_POOR = 0.5             # the given pose fits worse than this ...
CORRECTION_GOOD = 0.6             # ... and the best position nearby, at the given heading, at least this well
CORRECTION_TOLERANCE = 0.01       # near-best at the given heading: within this of its best


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


def support_grid(scorer, points, lever, pose, span_m=CORRECTION_SPAN_M, step_m=CORRECTION_STEP_M):
    """(support[i_along, j_across], offsets m) of positions around base pose (x, y, yaw rad) at its heading; the
    offsets run along and across that heading."""
    offsets = np.round(np.arange(-span_m, span_m + 1e-9, step_m), 6)
    along, across = (v.ravel() for v in np.meshgrid(offsets, offsets, indexing="ij"))
    c, s = math.cos(pose[2]), math.sin(pose[2])
    rotated = points @ np.array([[c, s], [-s, c]])
    lx = pose[0] + c * along - s * across + c * lever[0] - s * lever[1]
    ly = pose[1] + s * along + c * across + s * lever[0] + c * lever[1]
    values = np.empty(len(lx))
    for lo in range(0, len(lx), 48):
        xy = np.stack([rotated[None, :, 0] + lx[lo:lo + 48, None], rotated[None, :, 1] + ly[lo:lo + 48, None]],
                      axis=-1)
        values[lo:lo + 48] = (scorer.lookup(xy) <= SUPPORT_M).mean(axis=1)
    return values.reshape(len(offsets), len(offsets)), offsets


def correction(scorer, scans, lever, pose):
    """The minimum-offset start-pose correction for base pose (x, y, yaw rad) and the start check's standing scans.

    Returns a dict for result.json: ``applied`` and, when applied, ``corrected_pose`` (same yaw), the offset along
    and across the given heading and in map axes, and the support before and after; else ``reason``:
    GIVEN_POSE_FITS, NOTHING_FITS_NEARBY, NO_DATA or GIVEN_POSE_IS_NEAREST.
    """
    points = local_endpoints(scans)
    result = {"rule": "minimum_offset", "span_m": CORRECTION_SPAN_M, "step_m": CORRECTION_STEP_M,
              "poor_below": CORRECTION_POOR, "good_from": CORRECTION_GOOD, "tolerance": CORRECTION_TOLERANCE,
              "given_pose": [float(v) for v in pose], "scans": len(scans), "applied": False}
    if len(scans) < MIN_SCANS or len(points) < 50:
        result["reason"] = "NO_DATA"
        return result
    grid, offsets = support_grid(scorer, points, lever, pose)
    centre = len(offsets) // 2
    best = float(grid.max())
    near = np.argwhere(grid >= best - CORRECTION_TOLERANCE)
    i, j = near[int(np.argmin(np.hypot(offsets[near[:, 0]], offsets[near[:, 1]])))]
    before = float(grid[centre, centre])
    result.update(support_before=round(before, 3), best_nearby=round(best, 3))
    if before >= CORRECTION_POOR:
        result["reason"] = "GIVEN_POSE_FITS"
        return result
    if best < CORRECTION_GOOD:
        result["reason"] = "NOTHING_FITS_NEARBY"
        return result
    along, across = float(offsets[i]), float(offsets[j])
    if i == centre and j == centre:
        result["reason"] = "GIVEN_POSE_IS_NEAREST"
        return result
    c, s = math.cos(pose[2]), math.sin(pose[2])
    dx, dy = c * along - s * across, s * along + c * across
    result.update(applied=True, reason=None, offset_along_m=round(along, 3), offset_across_m=round(across, 3),
                  offset_map_m=[round(dx, 3), round(dy, 3)],
                  corrected_pose=[round(pose[0] + dx, 4), round(pose[1] + dy, 4), float(pose[2])],
                  support_after=round(float(grid[i, j]), 3))
    return result


def correction_message(result):
    """One operator line about the start-pose correction (offsets in the chair's own directions)."""
    reason = result.get("reason")
    if result.get("applied"):
        along, across = result["offset_along_m"], result["offset_across_m"]
        moves = []
        if abs(along) >= 0.005:
            moves.append(("向前 %.2f m" if along > 0 else "向后 %.2f m") % abs(along))
        if abs(across) >= 0.005:
            moves.append(("向车左 %.2f m" if across > 0 else "向车右 %.2f m") % abs(across))
        x, y = result["corrected_pose"][:2]
        return ("起点修正：给定位姿贴图 %.0f%%，初始位姿改到 (%.2f, %.2f)，%s，贴图 %.0f%%；朝向不变，Nav2 启动前已重发"
                % (100 * result["support_before"], x, y, "、".join(moves), 100 * result["support_after"]))
    if reason == "GIVEN_POSE_FITS":
        return "起点修正：不用改，给定位姿贴图 %.0f%%" % (100 * result["support_before"])
    if reason == "NOTHING_FITS_NEARBY":
        return ("起点修正：不改，给定朝向下 ±%.1f m 内最好的贴图只有 %.0f%%（要 %.0f%% 以上）。请核对车的摆放和 --initial-pose"
                % (CORRECTION_SPAN_M, 100 * result["best_nearby"], 100 * CORRECTION_GOOD))
    if reason == "GIVEN_POSE_IS_NEAREST":
        return "起点修正：不改，给定位置已经是贴图最好的位置里最近的"
    return "起点修正：不改，没有足够的静止扫描"


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
