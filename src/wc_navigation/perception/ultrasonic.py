"""Ultrasonic range modules as collision-monitor sources (window C; off by default, user 2026-11-12 22:05).

Four FD07-34R modules, "a further brake, never a release": left front (below the left lidar, facing ahead), left
side (facing left), right front, right side (mirrored). Window A publishes them in the panel session as
sensor_msgs/Range (radiation_type ULTRASOUND, field_of_view, min_range and max_range of the module, +inf without
an echo as REP 117), with the static transforms of their frames; window D relays them into the live chain and
checks in the gate that every configured module's data is fresh and valid (status/C.md 2026-11-13).

What the installed collision monitor (ros-humble-nav2-collision-monitor 1.1.20, src/range.cpp) does with a reading:
- it keeps the latest message per source and, at every command it filters, turns it into points on an arc at the
  measured range across the message's field_of_view, one point every ``obstacles_angle`` plus the last edge, in
  the sensor frame at z = 0, moved into base_link by the static transform (base_shift_correction off);
- no points (nothing stops) when the message is older than ``source_timeout`` (1.0 s here; it logs "Ignoring the
  source"), or when range < min_range or range > max_range: +inf (no echo) and -inf (too close, REP 117) alike,
  so the monitor never acts on a reading inside the module's blind zone;
- NaN passes that check and gives NaN points, which no zone contains;
- every source feeds every zone (1.1.20 has no per-zone source list), and a zone acts on more than
  ``MONITOR_MAX_POINTS`` points of all sources together.
``arc_points`` repeats that computation (float32 angles as in range.cpp) for checks and studies.
"""
from __future__ import annotations

import math

import numpy as np

PROBES = ("left_front", "right_front", "left_side", "right_side")
FRONT = ("left_front", "right_front")
MODES = {"off": (), "front": FRONT, "all": PROBES}
# Proposed to window A (status/C.md 2026-11-13): the Range topic and frame of each module, and one status topic.
TOPIC = "/wc_mapping/ultrasonic/{probe}/range"
FRAME = "ultrasonic_{probe}"
STATUS_TOPIC = "/wc_mapping/ultrasonic/status"
OBSTACLES_ANGLE_RAD = math.pi / 180        # the Humble default: one point per degree of the field of view
# The modules are polled one after the other, 0.11 s apart (wc_runtime.ultrasonic_capture): about 2.3 readings
# per module and second. A reading older than this is old, as for the collision monitor's source_timeout.
RANGE_MAX_AGE_S = 1.0


def source_name(probe):
    if probe not in PROBES:
        raise ValueError("unknown ultrasonic module: %s" % (probe,))
    return "ultrasonic_" + probe


def check_probes(probes, prefix=""):
    """(probes in PROBES order, prefix) for a profile; refuses unknown, repeated or badly prefixed values."""
    probes = tuple(probes or ())
    if len(set(probes)) != len(probes) or any(p not in PROBES for p in probes):
        raise ValueError("ultrasonic modules must be a subset of " + ", ".join(PROBES))
    if not isinstance(prefix, str) or (prefix and (not prefix.startswith("/") or prefix.endswith("/"))):
        raise ValueError("ultrasonic topic prefix must be empty or start with / and not end with /")
    if prefix and not probes:
        raise ValueError("an ultrasonic topic prefix needs ultrasonic modules")
    return tuple(p for p in PROBES if p in probes), prefix


def topic(probe, prefix=""):
    source_name(probe)
    return prefix + TOPIC.format(probe=probe)


def monitor_sources(probes, prefix=""):
    """Collision-monitor ``range`` sources, one per module (topics under the run's source prefix)."""
    return {source_name(p): {"type": "range", "topic": topic(p, prefix), "obstacles_angle": OBSTACLES_ANGLE_RAD,
                             "enabled": True} for p in probes}


def arc_points(range_m, field_of_view_rad, min_range_m, max_range_m, obstacles_angle_rad=OBSTACLES_ANGLE_RAD):
    """The points the 1.1.20 collision monitor makes of one reading, (N, 2) in the sensor frame (x along the axis).

    Empty when the reading is out of the module's span (as range.cpp: range < min_range or range > max_range); a NaN
    range gives NaN points. Message fields are float32, the loop angle is a float as in range.cpp.
    """
    r = np.float32(range_m)
    fov = np.float32(field_of_view_rad)
    if r < np.float32(min_range_m) or r > np.float32(max_range_m):
        return np.zeros((0, 2))
    half = fov / np.float32(2.0)
    angles = []
    angle = -half
    while angle < half:
        angles.append(angle)
        angle = np.float32(float(angle) + float(obstacles_angle_rad))
    angles.append(half)
    a = np.asarray(angles, dtype=np.float32)
    return np.column_stack([r * np.cos(a), r * np.sin(a)]).astype(np.float64)


def in_base(points_sensor, x_m, y_m, yaw_rad):
    """Sensor-frame arc points moved into base_link by a planar mount (x, y, yaw); the monitor drops z."""
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    p = np.asarray(points_sensor, dtype=float).reshape(-1, 2)
    return np.column_stack([x_m + c * p[:, 0] - s * p[:, 1], y_m + s * p[:, 0] + c * p[:, 1]])


def range_health(value, age_s, max_age_s=RANGE_MAX_AGE_S):
    """(ok, reason) of one module's latest reading for the gate (window D); a further brake, never a release.

    ``value``: {"range", "min_range", "max_range"} of the latest Range message (or None); ``age_s``: seconds since it
    was measured (its stamp) on the consumer's clock. ok: a fresh reading within the span, or no echo (+inf, or above
    max_range). Not ok:
    - ULTRASONIC_NO_DATA: nothing received;
    - ULTRASONIC_OLD: older than ``max_age_s`` (the collision monitor ignores it after its source_timeout);
    - ULTRASONIC_INVALID: NaN, or a message whose span is not 0 < min_range < max_range;
    - ULTRASONIC_TOO_CLOSE: -inf or below min_range, something inside the module's blind zone, which the collision
      monitor ignores.
    """
    if value is None or age_s is None:
        return False, "ULTRASONIC_NO_DATA"
    try:
        r, low, high, age = (float(value["range"]), float(value["min_range"]), float(value["max_range"]),
                             float(age_s))
    except (KeyError, TypeError, ValueError):
        return False, "ULTRASONIC_INVALID"
    if not (math.isfinite(low) and math.isfinite(high) and 0.0 < low < high) or math.isnan(r):
        return False, "ULTRASONIC_INVALID"
    if not 0.0 <= age <= float(max_age_s):
        return False, "ULTRASONIC_OLD"
    if r < low:
        return False, "ULTRASONIC_TOO_CLOSE"
    return True, ""
