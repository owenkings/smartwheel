"""Offline replay of the exploration mapper on a frozen single-left frontend.

Reads the exact single-left frontend (prepared lidar XYZ plus the frozen
wheel/IMU prior pose of every frame) through the existing hashed loaders,
projects each frame into the same 161-bin, +/-60 degree scan the AMCL replay
uses (or, with ``--map-bin-deg``, the same projection with a finer azimuth
bin), and builds two maps from identical input: odometry only (``odom_grid``)
and odometry with bounded scan-to-map correction (``odom_icp``).

Missing bins are never turned into free space. Every fifth frame is held out:
it is neither mapped nor matched, and afterwards its endpoints, placed at the
pose each method predicted for it, are compared with that method's final map.
This is self-consistency on one recording, not ground truth. No ROS nodes,
devices or vehicle commands are involved; outputs go to reports storage.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import uuid

import numpy as np
from scipy.spatial import cKDTree

from .maps import save_trinary_map
from .occupancy import LogOddsGrid
from .ros_util import compose, inverse
from .scan_match import IcpConfig, correct_pose, map_points, scan_points

ANGLES = -math.pi / 3 + (math.pi / 240) * np.arange(161)   # amcl_replay.build_scan bins
RANGE_MIN, RANGE_MAX = 0.30, 8.0
BIN_CHOICES_DEG = (0.75, 0.5, 0.25)
# map_nav.yaml: cells mostly seen through (people walking, late stray returns) count as free.
NAVIGATION_CLASSIFICATION = {"contested_free": True, "min_observations": 5, "max_hit_ratio": 0.25}


def bin_scan(points, height_band, bin_deg=0.75):
    """Nearest observed return per azimuth bin over +/-60 degrees; returns (ranges, angles).

    Same acceptance and round-to-nearest bin index as ``amcl_replay.build_scan``,
    which is the 0.75 degree case. A finer bin keeps more of the measured
    directions of the dense cloud; a bin without any point stays +inf, so it
    never clears space.
    """
    if bin_deg not in BIN_CHOICES_DEG:
        raise ValueError("bin width must be one of %s degrees" % (BIN_CHOICES_DEG,))
    step = math.radians(bin_deg)
    count = int(round(120.0 / bin_deg)) + 1
    low, high = (float(v) for v in height_band)
    xyz = np.asarray(points, dtype=float)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("scan source must be an Nx3 array")
    good = xyz[np.isfinite(xyz).all(axis=1)]
    horizontal = np.hypot(good[:, 0], good[:, 1])
    azimuth = np.arctan2(good[:, 1], good[:, 0])
    accepted = ((horizontal >= RANGE_MIN) & (horizontal <= RANGE_MAX) & (good[:, 2] >= low) & (good[:, 2] <= high)
                & (azimuth >= -math.pi / 3) & (azimuth <= math.pi / 3))
    index = np.floor((azimuth[accepted] + math.pi / 3) / step + 0.5).astype(np.int64)
    ranges = np.full(count, math.inf)
    np.minimum.at(ranges, index, horizontal[accepted])
    return ranges, -math.pi / 3 + step * np.arange(count)


def planar(transform):
    t = np.asarray(transform, dtype=float)
    return float(t[0, 3]), float(t[1, 3]), math.atan2(t[1, 0], t[0, 0])


def _stats(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return None
    return {"count": int(len(values)), "median": float(np.median(values)),
            "p90": float(np.percentile(values, 90)), "max": float(values.max()),
            "within_0_10_m": float(np.mean(values <= 0.10))}


def replay(scans, *, scan_matching, holdout_every=5, icp=None, size_m=80.0, angles=ANGLES):
    """``scans``: list of (stamp_ns, odom_pose, ranges). Returns grid, rows, holdout evaluation."""
    icp = icp or IcpConfig()
    angles = np.asarray(angles, dtype=float)
    first = scans[0][1]
    grid = LogOddsGrid(resolution=0.05, size_m=size_m, center_xy=first[:2], clear_no_return=False)
    correction = (0.0, 0.0, 0.0)
    rows, held = [], []
    for k, (stamp, odom, ranges) in enumerate(scans):
        predicted = compose(correction, odom)
        if holdout_every and k % holdout_every == holdout_every - 1:
            held.append((predicted, ranges))
            rows.append([stamp, "HOLDOUT", *odom, *predicted, *correction])
            continue
        pose, status = predicted, "ODOMETRY_ONLY"
        if scan_matching:
            points = scan_points(angles, ranges, RANGE_MIN, RANGE_MAX, icp.max_points)
            result = correct_pose(map_points(grid, predicted[:2], RANGE_MAX + 0.5), predicted, points, icp)
            status = result["status"]
            if status == "CORRECTED":
                pose = tuple(result["pose"])
                correction = compose(pose, inverse(odom))
        grid.integrate(pose, angles, ranges, RANGE_MIN, RANGE_MAX)
        rows.append([stamp, status, *odom, *pose, *correction])
    occupied = _occupied(grid)
    distances = []
    if len(occupied):
        tree = cKDTree(occupied)
        for pose, ranges in held:
            points = scan_points(angles, ranges, RANGE_MIN, RANGE_MAX, 1000)
            c, s = math.cos(pose[2]), math.sin(pose[2])
            world = points @ np.array([[c, s], [-s, c]]) + np.array(pose[:2])
            distances.extend(tree.query(world)[0].tolist())
    return grid, rows, {"holdout_frames": len(held), "endpoint_to_map_m": _stats(distances)}


def unknown_pockets(grid, max_cells=20, **classification):
    """Unknown components enclosed by known cells of the saved crop; small ones are pinholes.

    A navigation consumer that treats unknown as an obstacle and inflates it
    turns every pinhole into a body-sized blocked disc.
    """
    from scipy import ndimage
    data, _ = grid.cropped(padding_m=1.0, **classification)
    labels, count = ndimage.label(data == -1, structure=np.ones((3, 3), dtype=bool))
    border = np.unique(np.concatenate([labels[0], labels[-1], labels[:, 0], labels[:, -1]]))
    sizes = np.bincount(labels.ravel(), minlength=count + 1)
    enclosed = np.setdiff1d(np.arange(1, count + 1), border)
    small = enclosed[sizes[enclosed] <= max_cells]
    return {"enclosed_components": int(len(enclosed)), "pinholes": int(len(small)),
            "pinhole_cells": int(sizes[small].sum()), "pinhole_max_cells": int(max_cells),
            "resolution_m": grid.resolution}


def _occupied(grid):
    rows, cols = np.nonzero(grid.logodds >= grid.occupied_at)
    return np.column_stack([grid.origin[0] + (cols + 0.5) * grid.resolution,
                            grid.origin[1] + (rows + 0.5) * grid.resolution])


def render(path, results, compare=None):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    panels = len(results) + (1 if compare is not None else 0)
    fig, axes = plt.subplots(panels, 1, figsize=(16, 4.2 * panels), constrained_layout=True)
    axes = np.atleast_1d(axes)
    for ax, (name, (grid, rows, evaluation)) in zip(axes, results.items()):
        cropped = grid.cropped(padding_m=0.5)
        data, origin = cropped
        image = np.where(data == 100, 0.0, np.where(data == 0, 1.0, 0.75))
        extent = [origin[0], origin[0] + data.shape[1] * grid.resolution,
                  origin[1], origin[1] + data.shape[0] * grid.resolution]
        ax.imshow(image, origin="lower", cmap="gray", vmin=0, vmax=1, extent=extent)
        track = np.array([[r[2], r[3]] for r in rows])
        used = np.array([[r[5], r[6]] for r in rows])
        ax.plot(track[:, 0], track[:, 1], color="#2b7bb9", linewidth=1.0, linestyle="--", label="Frozen wheel/IMU prior")
        if name == "odom_icp":
            ax.plot(used[:, 0], used[:, 1], color="#d98a1c", linewidth=1.2, label="Corrected sensor path")
        stats = evaluation["endpoint_to_map_m"] or {}
        ax.set_title("%s | held-out endpoints to map: median %.3f m, p90 %.3f m, within 0.10 m %.0f%%" % (
            name, stats.get("median", float("nan")), stats.get("p90", float("nan")),
            100 * stats.get("within_0_10_m", float("nan"))))
        ax.set_aspect("equal")
        ax.legend(loc="upper right", fontsize=8)
    if compare is not None:
        from wc_runtime.map_viewer import inspect_grid
        _, data, meta = inspect_grid(Path(compare))
        res, origin = meta["resolution"], meta["origin"]
        image = np.where(data == 100, 0.0, np.where(data == 0, 1.0, 0.75))
        extent = [origin[0], origin[0] + data.shape[1] * res, origin[1], origin[1] + data.shape[0] * res]
        axes[-1].imshow(image, origin="lower", cmap="gray", vmin=0, vmax=1, extent=extent)
        axes[-1].set_title("existing native rebuild for comparison: " + Path(compare).parent.name)
        axes[-1].set_aspect("equal")
    fig.suptitle("OFFLINE REPLAY of a frozen recording - prior frame of the recording, not ground truth")
    fig.savefig(path, dpi=110)
    plt.close(fig)


def _digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, required=True)
    parser.add_argument("--source-cell", type=Path, required=True, help="exact single-left frontend cell")
    parser.add_argument("--compare-map", type=Path, help="existing map YAML drawn beside the replay maps")
    parser.add_argument("--holdout-every", type=int, default=5)
    parser.add_argument("--scan-height", nargs=2, type=float, default=[-0.5, 0.5], metavar=("MIN", "MAX"))
    parser.add_argument("--map-bin-deg", type=float, choices=BIN_CHOICES_DEG, default=0.75,
                        help="azimuth bin of the mapping scan; 0.75 is the AMCL replay scan, finer bins keep "
                             "more measured directions and leave fewer unobserved pinholes")
    args = parser.parse_args(argv)
    if not 2 <= args.holdout_every <= 20:
        parser.error("--holdout-every must be within 2..20")
    from wc_runtime.storage_policy import StoragePolicy
    from ..amcl_replay import build_scan
    from ..rebuild import validate_frontend_row
    from ..replay_cli import checked_frames, select_rows
    policy = StoragePolicy(args.project_root)
    policy.check()
    source = args.source_cell.expanduser().resolve(strict=True)
    runtime_raw = (source / "runtime_config.json").read_bytes()
    index_raw = (source / "frontend/index.jsonl").read_bytes()
    runtime = json.loads(runtime_raw)
    if runtime.get("mode") != "left" or runtime.get("offline_source_selection", {}).get("selected_sides") != ["left"]:
        raise ValueError("exact frozen single-left source is required")
    rows = [json.loads(line) for line in index_raw.decode("utf-8").splitlines()]
    stamps = [validate_frontend_row(row) for row in rows]
    if not stamps or any(a >= b for a, b in zip(stamps, stamps[1:])):
        raise ValueError("unique increasing source timestamps required")
    hashes = {str(source / "runtime_config.json"): hashlib.sha256(runtime_raw).hexdigest(),
              str(source / "frontend/index.jsonl"): hashlib.sha256(index_raw).hexdigest()}
    frames, counts = checked_frames(source, select_rows(rows, stamps[0], (0.0, 1e7), 0.0), runtime, hashes)
    scans, observed = [], []
    for frame in frames:
        if args.map_bin_deg == 0.75:
            ranges, _ = build_scan(frame["points"], *args.scan_height)   # exactly the AMCL replay scan
            ranges, angles = ranges.astype(float), ANGLES
        else:
            ranges, angles = bin_scan(frame["points"], args.scan_height, args.map_bin_deg)
        scans.append((frame["stamp_ns"], planar(frame["odometry_T"]), ranges))
        observed.append(int(np.isfinite(ranges).sum()))
    output = policy.resolve("reports/navigation_replay_mapping/" + datetime.now(timezone.utc).strftime(
        "%Y%m%dT%H%M%SZ") + "_" + uuid.uuid4().hex[:8])
    output.mkdir(parents=True, exist_ok=False)
    results, summary = {}, {}
    for name, matching in (("odom_grid", False), ("odom_icp", True)):
        grid, table, evaluation = replay(scans, scan_matching=matching, holdout_every=args.holdout_every,
                                         angles=angles)
        results[name] = (grid, table, evaluation)
        directory = output / name
        directory.mkdir()
        data, origin = grid.cropped(padding_m=1.0)
        spec, _ = save_trinary_map(directory, "map", data, grid.resolution, origin)
        nav_data, nav_origin = grid.cropped(padding_m=1.0, **NAVIGATION_CLASSIFICATION)
        nav_spec, _ = save_trinary_map(directory, "map_nav", nav_data, grid.resolution, nav_origin)
        with (directory / "poses.csv").open("x", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["stamp_ns", "status", "odom_x", "odom_y", "odom_yaw", "used_x", "used_y", "used_yaw",
                             "map_odom_x", "map_odom_y", "map_odom_yaw"])
            writer.writerows(table)
        statuses = {}
        for row in table:
            statuses[row[1]] = statuses.get(row[1], 0) + 1
        final = table[-1][8:11]
        summary[name] = {"map": str(spec), "navigation_map": str(nav_spec),
                         "navigation_map_free_cells": int((nav_data == 0).sum()),
                         "frame_status_counts": statuses, **evaluation,
                         "final_map_to_odom": final,
                         "max_map_to_odom_translation_m": float(max(math.hypot(r[8], r[9]) for r in table)),
                         "max_map_to_odom_yaw_deg": float(max(abs(math.degrees(r[10])) for r in table)),
                         "occupied_cells": int((data == 100).sum()), "free_cells": int((data == 0).sum())}
    sources_unchanged = all(_digest(path) == digest for path, digest in hashes.items())
    report = {"schema_version": 1, "status": "COMPLETE" if sources_unchanged else "SOURCE_CHANGED",
              "source_cell": str(source), "frames": len(frames), "observed_bins_median": float(np.median(observed)),
              "scan": {"bins": len(angles), "bin_deg": args.map_bin_deg, "fov_deg": [-60, 60],
                       "range_m": [RANGE_MIN, RANGE_MAX], "height_band_m": args.scan_height,
                       "missing_bins_clear_space": False},
              "navigation_classification": dict(NAVIGATION_CLASSIFICATION),
              "unknown_pockets": {name: {"map": unknown_pockets(results[name][0]),
                                         "map_nav": unknown_pockets(results[name][0], **NAVIGATION_CLASSIFICATION)}
                                  for name in results},
              "holdout_every": args.holdout_every, "icp": IcpConfig().as_dict(), "results": summary,
              "sources_unchanged": sources_unchanged, "source_files_checked": len(hashes),
              "evaluation": "HELD_OUT_FRAMES_SAME_RECORDING_SELF_CONSISTENCY_NOT_GROUND_TRUTH",
              "hardware_started": False, "control_topics_published": False,
              "physical_navigation_validated": False}
    try:
        render(output / "preview.png", results, args.compare_map)
        report["preview"] = str(output / "preview.png")
    except Exception as exc:
        report["preview_error"] = type(exc).__name__ + ": " + str(exc)
    with (output / "result.json").open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
    policy.check()
    brief = {"status": report["status"], "output": str(output), "frames": len(frames), "bin_deg": args.map_bin_deg,
             **{name: {"endpoint_to_map_m": value["endpoint_to_map_m"], "frame_status_counts": value["frame_status_counts"],
                       "final_map_to_odom": value["final_map_to_odom"],
                       "pinholes": {kind: report["unknown_pockets"][name][kind]["pinholes"] for kind in ("map", "map_nav")}}
                for name, value in summary.items()}}
    print(json.dumps(brief, ensure_ascii=True, allow_nan=False), flush=True)
    return 0 if sources_unchanged else 1


if __name__ == "__main__":
    raise SystemExit(main())
