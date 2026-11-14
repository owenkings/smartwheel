"""The live chain's side of window A's ultrasonic modules: frozen geometry, the gate's verdicts, the status line.

Off unless the shadow command gets window C's ``--monitor-ultrasonic front|all`` (总管 2026-11-13 D4 item 2; user
2026-11-12 22:05: a further brake, never a release). Then:

* ``settings`` reads the frozen ultrasonic configuration of the run's ``--cell`` (``ultrasonic_live.json``, which
  freeze-config takes from the panel's live session only when it runs the modules) and computes, with window A's
  ``wc_runtime.ultrasonic_live.probe_transforms`` and the frozen ``hardware_setup.json``, the static transforms
  base_link -> ultrasonic_<probe> the collision monitor needs (the recordings carry no /tf_static; the supervisor
  publishes them). A closed loop is refused until window A's field check has verified the configuration: before
  that nothing may brake on these ranges (wc_runtime.ultrasonic_live; C.md 2026-11-13).
* ``verdicts`` judges each configured module's latest Range with window C's ``perception.ultrasonic.range_health``
  on the age from its stamp (the reply's arrival on the panel's host clock) to the gate's node clock, as the
  collision monitor ages it: the gate holds the chair (ULTRASONIC_HOLD) unless every module is ok.
* ``operator_text`` is the closed-loop status line's part.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

NAMES = {"left_front": "左前", "right_front": "右前", "left_side": "左侧", "right_side": "右侧"}
CONFIG_FILE = "ultrasonic_live.json"          # in a frozen live configuration (live/current_config.freeze)
HARDWARE_FILE = "hardware_setup.json"


def _number_text(value):
    """A range for JSON documents: the finite value, or "+inf" / "-inf" / "nan" (result.json allows no NaN)."""
    value = float(value)
    if math.isfinite(value):
        return round(value, 4)
    if math.isnan(value):
        return "nan"
    return "+inf" if value > 0 else "-inf"


def settings(cell, probes, prefix="", closed_loop=False):
    """run_config's ``ultrasonic`` block for ``--monitor-ultrasonic``; ValueError when the run cannot have it.

    ``cell``: the frozen live configuration (freeze-config) with ``ultrasonic_live.json`` and ``hardware_setup.json``;
    ``probes``: the modules (perception.ultrasonic order); ``prefix``: their topics' prefix in this domain (the relay's
    output prefix on the vehicle or in a relay test, else empty).
    """
    from wc_runtime.ultrasonic_live import not_verified, probe_transforms, validate_live_config
    from ..perception.ultrasonic import RANGE_MAX_AGE_S, STATUS_TOPIC, check_probes, topic
    probes, prefix = check_probes(probes, prefix)
    if not probes:
        raise ValueError("no ultrasonic module configured")
    cell = Path(cell).expanduser()
    path = cell / CONFIG_FILE
    if not path.is_file():
        raise ValueError("the frozen configuration %s has no %s: the panel's live session runs without the "
                         "ultrasonic modules (mapping_live.json ultrasonic_enabled is false), so there is nothing to "
                         "read; freeze the configuration again once the panel runs them" % (cell, CONFIG_FILE))
    raw = path.read_bytes()
    config = validate_live_config(json.loads(raw.decode("utf-8")))
    hardware_path = cell / HARDWARE_FILE
    if not hardware_path.is_file():
        raise ValueError("the frozen configuration %s has no %s for the modules' geometry" % (cell, HARDWARE_FILE))
    transforms = probe_transforms(config, json.loads(hardware_path.read_text(encoding="utf-8")))
    sha256 = hashlib.sha256(raw).hexdigest()
    runtime_path = cell / "runtime_config.json"
    provenance = None
    if runtime_path.is_file():
        provenance = json.loads(runtime_path.read_text(encoding="utf-8")).get("ultrasonic_provenance")
    if provenance is not None:
        # the panel resolved and froze the same file and computed the same geometry
        frozen = ((provenance.get("files") or {}).get(CONFIG_FILE) or {}).get("sha256")
        if frozen != sha256:
            raise ValueError("%s does not match the panel's ultrasonic provenance (sha256 %s, frozen %s)"
                             % (path, sha256, frozen))
        for probe in probes:
            theirs = ((provenance.get("geometry") or {}).get(probe) or {}).get("T_base_probe") or {}
            ours = transforms[probe]["T_base_probe"]
            if any(abs(float(a) - float(b)) > 1e-6 for a, b in zip(
                    list(theirs.get("translation_m", [])) + list(theirs.get("rotation_xyzw", [])),
                    list(ours["translation_m"]) + list(ours["rotation_xyzw"]))) or not theirs:
                raise ValueError("the %s geometry differs from the panel's (ultrasonic_provenance)" % probe)
    open_checks = not_verified(config)
    if closed_loop and open_checks:
        raise ValueError("the closed loop needs the field-checked ultrasonic configuration (verified), still open in "
                         "%s: %s (docs/operations/ultrasonic_field_check.md)" % (path, ", ".join(open_checks)))
    return {"probes": list(probes), "prefix": prefix, "topics": {p: topic(p, prefix) for p in probes},
            "status_topic": prefix + STATUS_TOPIC, "max_age_s": RANGE_MAX_AGE_S,
            "verified": not open_checks, "not_verified": open_checks,
            "config": {"path": str(path.resolve()), "sha256": sha256},
            "limits": {k: config[k] for k in ("field_of_view_rad", "min_range_m", "max_range_m", "raw_unit_m")},
            "geometry_status": config["geometry_status"],
            "geometry": {p: {"frame_id": transforms[p]["frame_id"], **transforms[p]["T_base_probe"]} for p in probes},
            "geometry_source": "wc_runtime.ultrasonic_live.probe_transforms(frozen %s, frozen %s)"
                               % (CONFIG_FILE, HARDWARE_FILE)}


def static_transforms(block):
    """(parent, child, (x, y, z), (qx, qy, qz, qw)) of each configured module, from run_config's block."""
    return [(entry["parent"], entry["frame_id"], tuple(float(v) for v in entry["translation_m"]),
             tuple(float(v) for v in entry["rotation_xyzw"])) for entry in block["geometry"].values()]


def verdicts(latest, now_s, max_age_s):
    """Per module {"ok", "reason", "range_m", "age_s", "messages"} of the latest readings for the gate.

    ``latest``: module -> None or {"range", "min_range", "max_range", "stamp_s", "messages"} of its last Range;
    ``now_s``: the gate's node clock (the readings are stamped on the panel's host clock, as the monitor uses them).
    The verdict is window C's ``range_health``: ULTRASONIC_NO_DATA, ULTRASONIC_OLD, ULTRASONIC_INVALID or
    ULTRASONIC_TOO_CLOSE, ok for a fresh reading within the span or without an echo.
    """
    from ..perception.ultrasonic import range_health
    out = {}
    for probe, value in latest.items():
        if value is None:
            ok, reason = range_health(None, None, max_age_s)
            out[probe] = {"ok": ok, "reason": reason, "range_m": None, "age_s": None, "messages": 0}
            continue
        age = float(now_s) - float(value["stamp_s"])
        ok, reason = range_health(value, age, max_age_s)
        out[probe] = {"ok": ok, "reason": reason, "range_m": _number_text(value["range"]),
                      "age_s": round(age, 3) if math.isfinite(age) else None, "messages": int(value.get("messages", 0))}
    return out


def _probe_text(entry):
    if entry.get("ok"):
        value = entry.get("range_m")
        return "没回波" if value == "+inf" or value is None else "%.2f m" % float(value)
    reason = entry.get("reason")
    if reason == "ULTRASONIC_OLD":
        return "%.1f s 没更新" % float(entry.get("age_s") or 0.0) if entry.get("age_s") is not None else "没更新"
    return {"ULTRASONIC_NO_DATA": "没有读数", "ULTRASONIC_INVALID": "读数无效",
            "ULTRASONIC_TOO_CLOSE": "太近（盲区里有东西）"}.get(reason, str(reason))


def operator_text(block, status=None):
    """The status line's part: each module's reading or problem from the gate's ``ultrasonic`` block, "不要开始自动"
    while the gate holds for them, "未核对" while window A's status says the configuration is not verified."""
    flag = "（未核对）" if isinstance(status, dict) and status.get("verified") is False else ""
    if not isinstance(block, dict) or not isinstance(block.get("probes"), dict):
        return "超声波%s 闸门还没有判定" % flag
    parts = ["%s %s" % (NAMES.get(probe, probe), _probe_text(entry)) for probe, entry in block["probes"].items()]
    return "超声波%s %s%s" % (flag, "、".join(parts), "" if block.get("ok") else "：不要开始自动")
