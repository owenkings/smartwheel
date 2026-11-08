"""Freeze the panel's current live configuration for the live chain (reads files only: no device, no ROS).

On the vehicle the live chain must use the configuration the panel would use now, not a recording's frozen
frontend (window A, 2026-11-08). ``wc_runtime.mapping_controller.configuration()`` resolves it: local project
overrides (``config/local/project/``) or else the base files, the mounts, IMU mount and wheel parameters of the
selected ``hardware_setup.json``, the confirmed gyro bias named in ``mapping_live.json`` and the device bindings.
This module resolves the request of a left-lidar, filtered-cloud, robot_localization live session in a child
process whose project root is the given checkout (the panel's checkout on the vehicle) and writes the result with
the layout of a frontend cell, so ``navigation_live_shadow inputs|shadow --cell`` loads it unchanged:

* ``runtime_config.json``: the resolved configuration without the raw-text snapshots (as the panel archives it);
* ``prior_config.json``: the session's prior template plus the fixed mount transforms
  (``mapping_input.mount_transforms`` without startup gravity), as the live input writes it at bootstrap;
* ``hardware_setup.json`` and, when configured, ``confirmed_gyro_bias.json`` with its evidence (byte snapshots);
* ``frozen_configuration.json``: source root, request, sha256 of every file and the checks.

Checks: the hardware setup comes from the local override (otherwise the repository base values, e.g. wheel
radius 0.185 m, are silently in use); whether a confirmed gyro bias is configured (otherwise the live chain needs
an explicit ``--gyro-bias``); and, against a reference frontend cell, lever arm, mount rotation, host filter and
device identities. Nothing in wc_runtime is changed; the child only reads files below the project root.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np

REQUEST = {"mode": "left", "mapping_enabled": True, "cloud_source": "filtered", "estimator": "robot_localization",
           "live_scheme": "validated"}
LIVE_CONFIG = "config/mapping_live.json"
# What mapping_input writes for a continuous live session that has no static calibration window.
INITIAL_REFERENCE_MODE = "FIXED_MOUNT_REFERENCE_NO_STATIC_CALIBRATION"
SNAPSHOTS = (("_hardware_setup_raw_text", "hardware_setup.json"),
             ("_gyro_bias_raw_text", "confirmed_gyro_bias.json"),
             ("_gyro_bias_evidence_raw_text", "confirmed_gyro_bias.evidence.json"))

CHILD = r"""
import copy, json, sys
from wc_runtime import mapping_controller
from wc_runtime.mapping_input import mount_transforms
from wc_runtime.prepare_picker_input import json_bytes
request = json.loads(sys.argv[1])
config = mapping_controller.configuration(request)
snapshots = {name: config[key] for key, name in json.loads(sys.argv[2]) if config.get(key) is not None}
runtime = {k: v for k, v in config.items() if not (k.startswith("_") and k.endswith("_raw_text"))}
transforms = mount_transforms(config, None)
prior = copy.deepcopy(config["prior_template"])
prior.setdefault("estimator", config.get("wheel_imu_estimator", "five_state"))
prior.setdefault("offline_experiment", config.get("offline_experiment", False))
prior.update({key: transforms[key] for key in ("base_frame", "R_base_imu", "T_base_axle")})
prior.update(continuous_mapping=True, initial_reference_mode=sys.argv[3])
sys.stdout.write(json.dumps({"root": str(mapping_controller.ROOT),
                             "runtime": json.loads(json_bytes(runtime)), "prior": json.loads(json_bytes(prior)),
                             "snapshots": snapshots}))
"""


class FreezeError(RuntimeError):
    pass


def resolve(config_root, session_id, config_path=LIVE_CONFIG, timeout_s=120.0):
    """Run configuration() in a child process rooted at ``config_root``; returns its JSON result."""
    root = Path(config_root).expanduser().resolve(strict=True)
    code = Path(__file__).resolve().parents[2]            # this checkout's src (wc_runtime and wc_navigation)
    request = {**REQUEST, "config_path": str(root / config_path), "session_id": session_id}
    env = dict(os.environ)
    env.update(WHEELCHAIR_PROJECT_ROOT=str(root), PYTHONDONTWRITEBYTECODE="1", PYTHONNOUSERSITE="1",
               PYTHONPATH=os.pathsep.join([str(code)] + [p for p in env.get("PYTHONPATH", "").split(os.pathsep) if p]))
    completed = subprocess.run([sys.executable, "-B", "-s", "-c", CHILD, json.dumps(request), json.dumps(SNAPSHOTS),
                                INITIAL_REFERENCE_MODE], cwd=root, env=env, capture_output=True, text=True,
                               timeout=timeout_s)
    if completed.returncode != 0:
        raise FreezeError("configuration() failed: " + (completed.stderr.strip().splitlines() or ["?"])[-1])
    result = json.loads(completed.stdout)
    if Path(result["root"]).resolve() != root:
        raise FreezeError("the child resolved another project root: " + result["root"])
    return request, result


def _write(path, data):
    with Path(path).open("xb") as stream:
        stream.write(data)
    return hashlib.sha256(data).hexdigest()


def _json(value):
    return (json.dumps(value, ensure_ascii=False, indent=1) + "\n").encode("utf-8")


def compare_cells(frozen, reference):
    """Differences between two live configurations (load_live_config results) that the live chain uses."""
    def max_abs(a, b):
        return float(np.max(np.abs(np.asarray(a, dtype=float) - np.asarray(b, dtype=float))))
    wheel_keys = ("wheel_radius_m", "track_width_m", "left_sign", "right_sign", "register_to_wheel_rpm")
    return {"T_base_axle_max_abs_diff": max_abs(frozen["T_base_axle"], reference["T_base_axle"]),
            "R_reference_base_max_abs_diff": max_abs(frozen["R_reference_base"], reference["R_reference_base"]),
            "lever_xy_max_abs_diff_m": max_abs(frozen["lever_xy"], reference["lever_xy"]),
            "cloud_filter_equal": frozen["filter"]["cloud_filter"] == reference["filter"]["cloud_filter"],
            "lidar_left_equal": frozen["identities"]["lidar_left"] == reference["identities"]["lidar_left"],
            "imu_equal": frozen["identities"]["imu"] == reference["identities"]["imu"],
            "wheel": {key: {"frozen": frozen["prior"]["wheel_candidate"].get(key),
                            "reference": reference["prior"]["wheel_candidate"].get(key)} for key in wheel_keys}}


def freeze(output, config_root, session_id, config_path=LIVE_CONFIG, compare_cell=None, gyro_bias=None):
    """Resolve, write and check; returns the frozen_configuration.json content."""
    from .config import load_live_config
    request, result = resolve(config_root, session_id, config_path)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    files = {"runtime_config.json": _write(output / "runtime_config.json", _json(result["runtime"])),
             "prior_config.json": _write(output / "prior_config.json", _json(result["prior"]))}
    for name, text in result["snapshots"].items():
        files[name] = _write(output / name, text.encode("utf-8"))
    runtime = result["runtime"]
    provenance = runtime.get("hardware_setup_provenance") or {}
    if provenance.get("sha256") != files.get("hardware_setup.json"):
        raise FreezeError("hardware setup snapshot does not match its resolved provenance")
    source = str(provenance.get("source_path", ""))
    wheel = runtime.get("wheel_candidate") or {}
    checks = {"hardware_setup_source": source,
              "hardware_setup_from_local_override": "config/local/project/" in source,
              "wheel_radius_m": wheel.get("wheel_radius_m"), "track_width_m": wheel.get("track_width_m"),
              "confirmed_gyro_bias_configured": runtime.get("confirmed_gyro_bias") is not None,
              "gyro_bias_provenance": runtime.get("gyro_bias_provenance"),
              "imu_sensor_id": runtime.get("imu_sensor_id"), "lidar_left": (runtime.get("sensor_ids") or {}).get("left"),
              "imu_time_source": "arrival (live; no device-time model, window A 2026-11-08)"}
    bias = gyro_bias or (output / "confirmed_gyro_bias.json" if "confirmed_gyro_bias.json" in files else None)
    if bias is not None:
        live = load_live_config(output, output / "hardware_setup.json", bias)
        checks["live_chain_loads_it"] = True
        checks["gyro_bias_used"] = live["provenance"]["gyro_bias"]
        if compare_cell is not None:
            reference = load_live_config(compare_cell, output / "hardware_setup.json", bias)
            checks["against_reference_cell"] = {"cell": str(Path(compare_cell).resolve()), **compare_cells(live, reference)}
    else:
        checks["live_chain_loads_it"] = None
        checks["gyro_bias_note"] = ("no confirmed gyro bias in the live configuration: give --gyro-bias (a fresh "
                                    "10+10+10 s confirmation at the start, or the ZL1107 file of the same IMU)")
    record = {"schema_version": 1, "kind": "FROZEN_LIVE_CONFIGURATION", "config_root": result["root"],
              "request": request, "files_sha256": files, "checks": checks,
              "use": "navigation_live_shadow ... --cell %s --hardware-setup %s --gyro-bias <bias file>"
                     % (output, output / "hardware_setup.json"),
              "hardware_started": False, "ros_started": False}
    _write(output / "frozen_configuration.json", _json(record))
    return record
