"""Metrics and preview for supervised autonomy runs (ground truth is simulated)."""
from __future__ import annotations

import csv
import math

import numpy as np
from scipy import ndimage

from .ros_util import compose, inverse

TRAJECTORY_COLUMNS = ["sim_time_s", "truth_x", "truth_y", "truth_yaw", "odom_x", "odom_y", "odom_yaw",
                      "estimate_x", "estimate_y", "estimate_yaw", "gated_linear", "gated_angular",
                      "nav_linear", "nav_angular"]


def write_trajectory(path, rows):
    with open(path, "x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(TRAJECTORY_COLUMNS)
        writer.writerows(rows)


def _stats(values):
    values = np.asarray([v for v in values if math.isfinite(v)], dtype=float)
    if not len(values):
        return None
    return {"count": int(len(values)), "mean": float(values.mean()), "p95": float(np.percentile(values, 95)),
            "max": float(values.max())}


def _pose_errors(estimate, truth):
    valid = np.isfinite(estimate).all(axis=1) & np.isfinite(truth).all(axis=1)
    position = np.hypot(estimate[valid, 0] - truth[valid, 0], estimate[valid, 1] - truth[valid, 1])
    yaw = np.abs(np.arctan2(np.sin(estimate[valid, 2] - truth[valid, 2]), np.cos(estimate[valid, 2] - truth[valid, 2])))
    return valid, position, np.degrees(yaw)


def trajectory_metrics(world, rows, radius_m, start_pose_world=None):
    """Errors of the map-frame estimate and, for comparison, of raw odometry.

    The simulated odometry frame starts at the start pose, so odometry in the
    world frame is ``start_pose_world * odom``.
    """
    if not rows:
        return {"samples": 0}
    data = np.asarray(rows, dtype=float)
    truth = data[:, 1:4]
    estimate = data[:, 7:10]
    valid, position_error, yaw_error = _pose_errors(estimate, truth)
    clearance = [world.clearance(x, y) for x, y in truth[:, :2]]
    gated = data[:, 10:12]
    odometry = {}
    if start_pose_world is not None:
        odom = np.array([compose(start_pose_world, tuple(row)) if np.isfinite(row).all() else (np.nan,) * 3
                         for row in data[:, 4:7]])
        _, odom_position, odom_yaw = _pose_errors(odom, truth)
        odometry = {"odometry_position_error_m": _stats(odom_position), "odometry_yaw_error_deg": _stats(odom_yaw)}
    return {
        "samples": int(len(data)), "sim_duration_s": float(data[-1, 0] - data[0, 0]),
        "estimate_available_fraction": float(valid.mean()),
        "localization_position_error_m": _stats(position_error),
        "localization_yaw_error_deg": _stats(yaw_error),
        **odometry,
        "minimum_true_clearance_m": float(min(clearance)),
        "minimum_true_gap_to_body_m": float(min(clearance) - radius_m),
        "max_gated_linear_mps": float(np.max(gated[:, 0])) if len(gated) else 0.0,
        "min_gated_linear_mps": float(np.min(gated[:, 0])) if len(gated) else 0.0,
        "max_abs_gated_angular_radps": float(np.max(np.abs(gated[:, 1]))) if len(gated) else 0.0,
        "estimate_source": "SLAM_OR_AMCL_MAP_TO_BASE_COMPOSED_WITH_WORLD_T_MAP",
    }


def map_in_world(world, data, resolution, origin, world_T_map):
    """Sample a map-frame trinary grid at every world cell centre (nearest cell)."""
    res = world.grid.resolution
    rows, cols = np.indices(world.solid.shape)
    wx = world.grid.origin[0] + (cols + 0.5) * res
    wy = world.grid.origin[1] + (rows + 0.5) * res
    ix, iy, iyaw = inverse(world_T_map)
    c, s = math.cos(iyaw), math.sin(iyaw)
    mx = ix + c * wx - s * wy
    my = iy + s * wx + c * wy
    mcol = np.floor((mx - origin[0]) / resolution).astype(np.int64)
    mrow = np.floor((my - origin[1]) / resolution).astype(np.int64)
    inside = (mcol >= 0) & (mcol < data.shape[1]) & (mrow >= 0) & (mrow < data.shape[0])
    sampled = np.full(world.solid.shape, -1, dtype=np.int8)
    sampled[inside] = data[mrow[inside], mcol[inside]]
    return sampled


def coverage_metrics(world, start_xy, data, resolution, origin, world_T_map, radius_m=None):
    """Compare a saved exploration map with the simulated ground truth.

    ``free_coverage_fraction`` covers all free space connected to the start.
    With ``radius_m``, ``reachable_center_*`` covers only where the assumed body
    centre can drive; a complete exploration should have mapped all of it.
    """
    sampled = map_in_world(world, data, resolution, origin, world_T_map)
    free_world = ~world.solid
    eight = np.ones((3, 3), dtype=bool)
    labels, _ = ndimage.label(free_world, structure=eight)
    col = int(math.floor((start_xy[0] - world.grid.origin[0]) / world.grid.resolution))
    row = int(math.floor((start_xy[1] - world.grid.origin[1]) / world.grid.resolution))
    region = labels == labels[row, col]
    # Distance inside solid to the nearest free cell: tolerate one cell of misalignment.
    depth_in_solid = ndimage.distance_transform_edt(world.solid, sampling=world.grid.resolution)
    mapped_free = sampled == 0
    area = world.grid.resolution ** 2
    result = {
        "connected_free_area_m2": float(region.sum() * area),
        "mapped_free_area_m2": float((mapped_free & region).sum() * area),
        "free_coverage_fraction": float((mapped_free & region).sum() / max(1, region.sum())),
        "false_free_area_m2": float((mapped_free & (depth_in_solid > 0.10)).sum() * area),
        "mapped_occupied_cells": int((sampled == 100).sum()),
        "alignment": "WORLD_T_MAP_IS_SIMULATED_START_POSE; SLAM drift appears as misalignment",
    }
    if radius_m is not None:
        centers = world.clearance_grid() >= radius_m
        center_labels, _ = ndimage.label(centers, structure=eight)
        reachable = (center_labels == center_labels[row, col]) if centers[row, col] else np.zeros_like(centers)
        result.update(reachable_center_area_m2=float(reachable.sum() * area),
                      reachable_center_mapped_fraction=float((mapped_free & reachable).sum() / max(1, reachable.sum())),
                      reachable_center_radius_m=float(radius_m))
    return result


def render_preview(path, world, rows, *, title, map_layer=None, goals=(), collisions=(), radius_m=0.55):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    res = world.grid.resolution
    ox, oy = world.grid.origin[:2]
    extent = [ox, ox + world.grid.width * res, oy, oy + world.grid.height * res]
    image = np.where(world.solid, 0.18, 1.0)
    fig, ax = plt.subplots(figsize=(12, 8) if world.grid.width / world.grid.height < 2.2 else (15, 6),
                           constrained_layout=True)
    ax.imshow(image, origin="lower", cmap="gray", vmin=0, vmax=1, extent=extent)
    if map_layer is not None:
        overlay = np.zeros((*map_layer.shape, 4))
        overlay[map_layer == 0] = [0.25, 0.55, 0.95, 0.30]
        overlay[map_layer == 100] = [0.85, 0.30, 0.10, 0.75]
        ax.imshow(overlay, origin="lower", extent=extent)
    for o in world.obstacles:
        ax.add_patch(plt.Circle((o.x, o.y), o.radius, color="#b33", alpha=0.6))
    if rows:
        data = np.asarray(rows, dtype=float)
        ax.plot(data[:, 1], data[:, 2], color="#1d8a4f", linewidth=1.8, label="Simulated true path")
        valid = np.isfinite(data[:, 7:9]).all(axis=1)
        ax.plot(data[valid, 7], data[valid, 8], color="#d98a1c", linewidth=1.0, linestyle="--",
                label="Localization estimate")
        ax.add_patch(plt.Circle(data[0, 1:3], radius_m, fill=False, color="#1d8a4f", linewidth=1.2))
        ax.add_patch(plt.Circle(data[-1, 1:3], radius_m, fill=False, color="#555", linewidth=1.2))
    for x, y, ok in goals:
        ax.scatter([x], [y], marker="*", s=90, color="#1d8a4f" if ok else "#c0392b", zorder=5)
    for x, y in collisions:
        ax.scatter([x], [y], marker="x", s=80, color="#c0392b", zorder=6)
    ax.set_title(title + "\nSIMULATION ONLY - assumed circular body, simulated sensor; not a vehicle test")
    ax.set_xlabel("world x (m)")
    ax.set_ylabel("world y (m)")
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=8)
    fig.savefig(path, dpi=140)
    plt.close(fig)


def world_goal(world_T_map, goal_map):
    return compose(world_T_map, goal_map)
