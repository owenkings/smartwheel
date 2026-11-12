"""XT-M60 phase-wrap (12.49 m) ghost pixels of one organized frame: a pure host-side check, off unless asked for.

Why (window A, 2026-11-11, reports/lidar_frequency/20261111T1445Z_room/README.md): exposures 1-3 run at 12 MHz,
unambiguous to c / (2 f) = 12.491 m. The firmware resolves the ambiguity with its 6 MHz exposures, but only for
strong returns (amplitude above about 350). A weaker return from a surface 12.5-25 m away comes out 12.491 m short:
a stable point 0-12.5 m in front of the chair. Such a ghost usually borders pixels of the same far surface that the
firmware did resolve.

Rule (coordinator 2026-11-12: K = 3, no growing): a pixel p with a range below ``max_near_range_m`` and an
amplitude below ``max_amplitude`` is a ghost when some pixel q within +-``window_px`` rows and columns reports
more than ``min_anchor_range_m`` and |r(q) - (r(p) + unambiguous_range_m)| < ``tolerance_m``. Ranges are the
pixels' 3D distances from the lidar. The image is not wrapped at its edges.

Limits: only ghosts next to a resolved pixel of their surface are found. A real object can lose the pixels along
its outline when a far surface the firmware resolved lies about 12.49 m behind it; the evaluation report gives
the numbers. Nothing here uses a map, a pose or earlier frames.

Use: ``source_frame_mask(message, PhaseWrapFilter())`` on the SourceFrame as received (the SDK host-filtered frame
of the live chain), before the mapping host filter: that filter's height cut removes ceiling pixels, which are the
resolved neighbours of the ceiling's ghosts. Drop the flagged pixels after it, for example
``finite &= ~mask`` on its organized output.
"""
from collections.abc import Mapping
from dataclasses import asdict, dataclass, fields
import math

import numpy as np

from .pointcloud import validate_pointcloud2

SPEED_OF_LIGHT_M_S = 299792458.0
UNAMBIGUOUS_12MHZ_M = SPEED_OF_LIGHT_M_S / (2 * 12e6)      # 12.491 m
AMPLITUDE_FIELDS = ("intensity", "amplitude")


@dataclass(frozen=True)
class PhaseWrapFilter:
    """Settings of ``ghost_mask``; the defaults are the evaluated rule (window A, 2026-11-11/12)."""
    window_px: int = 3                 # neighbours within +-window_px rows and columns (K)
    tolerance_m: float = 0.6           # |r(q) - (r(p) + unambiguous_range_m)| below this
    max_amplitude: float = 450.0       # only pixels dimmer than this can be ghosts (the frame's amplitude scale)
    max_near_range_m: float = 6.5      # only pixels nearer than this can be ghosts
    min_anchor_range_m: float = 11.0   # a resolved neighbour reports more than this
    unambiguous_range_m: float = UNAMBIGUOUS_12MHZ_M

    def __post_init__(self):
        if isinstance(self.window_px, bool) or not isinstance(self.window_px, (int, np.integer)) \
                or not 1 <= int(self.window_px) <= 10:
            raise ValueError("window_px must be an integer within 1..10")
        object.__setattr__(self, "window_px", int(self.window_px))
        for name in ("tolerance_m", "max_amplitude", "max_near_range_m", "min_anchor_range_m",
                     "unambiguous_range_m"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float, np.integer, np.floating)) \
                    or not math.isfinite(float(value)) or float(value) <= 0:
                raise ValueError(name + " must be a positive finite number")
            object.__setattr__(self, name, float(value))
        if not 5.0 <= self.unambiguous_range_m <= 50.0:
            raise ValueError("unambiguous_range_m must be within 5..50 m")
        if self.tolerance_m > 2.0:
            raise ValueError("tolerance_m must be at most 2 m")
        if not self.max_near_range_m < self.unambiguous_range_m:
            raise ValueError("max_near_range_m must be below unambiguous_range_m")
        if not self.max_near_range_m < self.min_anchor_range_m <= 2 * self.unambiguous_range_m:
            raise ValueError("min_anchor_range_m must be above max_near_range_m and at most twice the "
                             "unambiguous range")

    def as_dict(self):
        """Plain values for a run configuration (JSON)."""
        return asdict(self)

    @classmethod
    def from_dict(cls, value):
        """Settings from ``as_dict`` output or a partial dict of the same keys; unknown keys are refused."""
        if not isinstance(value, Mapping):
            raise ValueError("phase-wrap filter settings must be an object")
        unknown = set(value) - {field.name for field in fields(cls)}
        if unknown:
            raise ValueError("unknown phase-wrap filter settings: " + ", ".join(sorted(unknown)))
        return cls(**dict(value))


def ghost_mask(ranges, amplitudes, cfg=PhaseWrapFilter()):
    """Boolean mask (rows, cols) of phase-wrap ghost pixels; the inputs are not modified.

    ``ranges``: (rows, cols) 3D distances in pixel order (row 0 at the top); NaN, infinite, zero or negative
    means no return. ``amplitudes``: the same shape; a non-finite amplitude never makes a ghost.
    """
    r = np.asarray(ranges, dtype=np.float64)
    a = np.asarray(amplitudes, dtype=np.float64)
    if r.ndim != 2 or r.shape != a.shape:
        raise ValueError("ranges and amplitudes must be 2D arrays of the same shape")
    out = np.zeros(r.shape, dtype=bool)
    with np.errstate(invalid="ignore"):
        valid = np.isfinite(r) & (r > 0.0)
        anchor = valid & (r > cfg.min_anchor_range_m)
        if not anchor.any():
            return out
        candidate = valid & (r < cfg.max_near_range_m) & np.isfinite(a) & (a < cfg.max_amplitude)
        if not candidate.any():
            return out
    rows, cols = r.shape
    # Visit the window around every anchor at once: anchors are a few hundred pixels of a frame, candidates most of
    # it. Neighbours outside the image are dropped, so the image does not wrap at its edges.
    offsets = np.arange(-cfg.window_px, cfg.window_px + 1)
    dy, dx = np.repeat(offsets, len(offsets)), np.tile(offsets, len(offsets))
    ar, ac = np.nonzero(anchor)
    pr, pc = ar[:, None] + dy[None, :], ac[:, None] + dx[None, :]
    inside = (pr >= 0) & (pr < rows) & (pc >= 0) & (pc < cols)
    neighbour = (pr * cols + pc)[inside]
    far = np.broadcast_to(r[ar, ac][:, None], inside.shape)[inside]
    flat_r, flat_candidate = r.reshape(-1), candidate.reshape(-1)
    with np.errstate(invalid="ignore"):
        match = flat_candidate[neighbour] & \
            (np.abs(far - (flat_r[neighbour] + cfg.unambiguous_range_m)) < cfg.tolerance_m)
    out.reshape(-1)[neighbour[match]] = True
    return out


def cloud_ghost_mask(xyz, amplitudes, shape, cfg=PhaseWrapFilter()):
    """Flat mask (rows*cols,) of an organized cloud: ``xyz`` (rows*cols, 3) in row-major pixel order."""
    rows, cols = (int(v) for v in shape)
    points = np.asarray(xyz, dtype=np.float64)
    a = np.asarray(amplitudes, dtype=np.float64).reshape(-1)
    if points.shape != (rows * cols, 3) or a.shape != (rows * cols,):
        raise ValueError("xyz must be (rows*cols, 3) and amplitudes (rows*cols,) in pixel order")
    with np.errstate(invalid="ignore", over="ignore"):
        ranges = np.linalg.norm(points, axis=1)
    return ghost_mask(ranges.reshape(rows, cols), a.reshape(rows, cols), cfg).reshape(-1)


def _get(obj, name, default=None):
    if isinstance(obj, Mapping):
        return obj.get(name, default)
    return getattr(obj, name, default)


def organized_shape(message):
    """(rows, cols) of a SourceFrame's cloud: an organized PointCloud2, or the SDK's row-major layout flags
    (``sdk_width``, ``sdk_height``, ``sdk_point_order=row_major``, as wc_runtime.mapping_filter reads them)."""
    layout = validate_pointcloud2(_get(message, "cloud"))
    if layout["height"] > 1:
        return layout["height"], layout["width"]
    flags = list(_get(message, "diagnostic_flags", ()) or ())
    values = {key: [str(flag).split("=", 1)[1] for flag in flags if str(flag).startswith(key + "=")]
              for key in ("sdk_width", "sdk_height", "sdk_point_order")}
    if any(len(v) != 1 for v in values.values()) or values["sdk_point_order"] != ["row_major"]:
        raise ValueError("no organized layout: need an organized cloud or one sdk_width, sdk_height and "
                         "sdk_point_order=row_major flag")
    width, height = values["sdk_width"][0], values["sdk_height"][0]
    if not (width.isascii() and width.isdecimal() and height.isascii() and height.isdecimal()) \
            or len(width) > 6 or len(height) > 6:
        raise ValueError("sdk_width and sdk_height must be small decimal integers")
    rows, cols = int(height), int(width)
    if rows < 2 or cols < 2 or rows * cols != layout["point_count"]:
        raise ValueError("sdk_width x sdk_height does not match the cloud's point count")
    return rows, cols


def source_frame_mask(message, cfg=PhaseWrapFilter()):
    """Flat ghost mask (one entry per point, in cloud order) of a SourceFrame (ROS message or dict alike).

    Reads x, y, z and the amplitude field (``intensity`` or ``amplitude``) of ``message.cloud``; the message is
    not modified.
    """
    cloud = _get(message, "cloud")
    rows, cols = organized_shape(message)
    layout = validate_pointcloud2(cloud)
    name = next((n for n in AMPLITUDE_FIELDS if n in layout["fields"]), None)
    if name is None or layout["fields"][name]["count"] != 1:
        raise ValueError("the cloud has no scalar intensity or amplitude field")
    storage = bytes(_get(cloud, "data"))
    order = ">" if layout["is_bigendian"] else "<"

    def column(field):
        spec = layout["fields"][field]
        return np.ndarray((layout["height"], layout["width"]), dtype=np.dtype(order + spec["dtype"]),
                          buffer=storage, offset=spec["offset"],
                          strides=(layout["row_step"], layout["point_step"])).reshape(-1).astype(np.float64)

    xyz = np.column_stack([column("x"), column("y"), column("z")])
    return cloud_ghost_mask(xyz, column(name), (rows, cols), cfg)
