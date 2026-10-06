#!/usr/bin/env python3
"""Benchmark unchanged NumPy oracle versus scalar DDA on the verified target.

Run from project root, through the coordinator's resource scheduler:
  PYTHONPATH=src python3 tests/maps/benchmark_ray_cells.py --rays 2000 --repeats 3

Every ray is compared outside timed regions before reporting timings. No speed
threshold is silently assumed, and timing never establishes real-device/map
accuracy. No sensor, ROS, file deletion or system configuration is involved.
"""
import argparse
import hashlib
import json
import platform
import statistics
import time

import numpy as np

from wc_maps.builder import _ray_cells
from test_ray_cells import numpy_ray_oracle, outcome, random_rays


def benchmark(function, rays, repeats):
    elapsed = []
    counts = []
    for _ in range(repeats):
        before = time.perf_counter()
        cells = 0
        for ray in rays:
            cells += len(function(*ray))
        elapsed.append(time.perf_counter() - before)
        counts.append(cells)
    return {"elapsed_s": elapsed, "median_s": statistics.median(elapsed), "visited_cells": counts}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rays", type=int, default=2000)
    parser.add_argument("--repeats", type=int, default=3)
    args = parser.parse_args(argv)
    if not 1 <= args.rays <= 20000 or not 1 <= args.repeats <= 10:
        parser.error("require 1<=rays<=20000 and 1<=repeats<=10")
    rays = random_rays(args.rays)
    digest = hashlib.sha256()
    # Some near-boundary rays may be identically rejected by both algorithms;
    # they are verified, counted separately, and omitted from timing loops.
    valid = []
    rejected = 0
    for index, ray in enumerate(rays):
        expected = outcome(numpy_ray_oracle, *ray)
        actual = outcome(_ray_cells, *ray)
        if actual != expected:
            raise AssertionError("scalar/oracle semantic mismatch at ray " + str(index))
        digest.update(json.dumps(expected, separators=(",", ":")).encode())
        if expected[0] == "error":
            rejected += 1
        else:
            valid.append(ray)
    if not valid:
        raise AssertionError("no successful rays available for timing")
    oracle = benchmark(numpy_ray_oracle, valid, args.repeats)
    scalar = benchmark(_ray_cells, valid, args.repeats)
    if oracle["visited_cells"] != scalar["visited_cells"]:
        raise AssertionError("timed tracing count mismatch")
    print(json.dumps(dict(schema_version=1, test="scalar_dda_numpy_oracle_benchmark", status="PASS",
        verification_level="SYNTHETIC", seed=40911, checked_rays=len(rays), timed_rays=len(valid),
        identically_rejected_rays=rejected, result_sha256=digest.hexdigest(), repeats=args.repeats,
        python_version=platform.python_version(), numpy_version=np.__version__, machine=platform.machine(),
        timing_scope="Ray tracing and len accumulation only; semantic comparison was outside timed regions.",
        original_numpy=oracle, scalar=scalar, speedup=oracle["median_s"] / scalar["median_s"],
        performance_acceptance_threshold=None, navigation_validated=False), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
