"""Trinary occupancy map files written by exploration runs.

The PGM/YAML pair follows the map_server trinary convention used by the
existing recorded maps (free 254, occupied 0, unknown 205, free_thresh 0.196)
and is read back through the project's map inspector before it is reported.
Files are created exclusively; an existing map is never overwritten.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

FREE_PIXEL, OCCUPIED_PIXEL, UNKNOWN_PIXEL = 254, 0, 205


def save_trinary_map(directory, stem, data, resolution, origin):
    """Write ``stem.pgm`` and ``stem.yaml``; ``data`` uses ROS bottom-up rows."""
    import yaml
    data = np.asarray(data)
    if data.ndim != 2 or not data.size or not np.isin(data, (-1, 0, 100)).all():
        raise ValueError("map data must be a nonempty trinary grid")
    if not (math.isfinite(resolution) and resolution > 0) or len(origin) != 3 or not all(math.isfinite(v) for v in origin):
        raise ValueError("map resolution/origin must be finite")
    directory = Path(directory)
    pgm, spec = directory / (stem + ".pgm"), directory / (stem + ".yaml")
    pixels = np.full(data.shape, UNKNOWN_PIXEL, dtype=np.uint8)
    pixels[data == 0] = FREE_PIXEL
    pixels[data == 100] = OCCUPIED_PIXEL
    image = np.flipud(pixels)
    height, width = image.shape
    with pgm.open("xb") as stream:
        stream.write(b"P5\n# wc_navigation.autonomy exploration map\n%d %d\n255\n" % (width, height))
        stream.write(image.tobytes())
    meta = {"image": pgm.name, "mode": "trinary", "resolution": float(resolution),
            "origin": [float(v) for v in origin], "negate": 0,
            "occupied_thresh": 0.65, "free_thresh": 0.196}
    with spec.open("x", encoding="utf-8") as stream:
        stream.write(yaml.safe_dump(meta, sort_keys=False))
    return spec, pgm


def read_back(spec):
    """Load through the project inspector; returns (stats, data, meta)."""
    from wc_runtime.map_viewer import inspect_grid
    return inspect_grid(Path(spec).resolve(strict=True))
