"""Versioned offline JSON command line interface; every failed operation is nonzero."""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import sys
import tempfile

from .core import (CalibrationError, QualityProfile, calibrate, fit_clock_model,
                   fit_ground_plane, validate_calibration)


def _read(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        return json.load(stream)


def _version_name(value):
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,79}", value) or value in {".", ".."}:
        raise CalibrationError("version must be a simple 1..80 character name")
    return value


def save_version(result, output_root, version):
    """Write once under a unique version; never overwrite an installed calibration."""
    version = _version_name(version)
    root_input = Path(output_root).absolute()
    if root_input.is_symlink() or any(parent.is_symlink() for parent in root_input.parents):
        raise CalibrationError("calibration output root must not traverse symlinks")
    root_input.mkdir(parents=True, exist_ok=True)
    root = root_input.resolve(strict=True)
    target = root / version
    if target.exists() or target.is_symlink():
        raise CalibrationError("calibration version already exists")
    # mkdir is the exclusive reservation. Result is atomically renamed inside it.
    target.mkdir()
    temporary = None
    try:
        envelope = dict(result)
        envelope["version"] = version
        envelope["created_utc"] = datetime.now(timezone.utc).isoformat()
        fd, temporary = tempfile.mkstemp(prefix=".result-", suffix=".json", dir=target)
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(envelope, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target / "result.json")
        temporary = None
        return target / "result.json"
    except BaseException:
        if temporary is not None:
            Path(temporary).unlink(missing_ok=True)
        # Only this exclusively-created empty/result directory is eligible for cleanup.
        if not (target / "result.json").exists():
            target.rmdir()
        raise


def parser():
    result = argparse.ArgumentParser(description="Offline dual XT-M60 calibration; no device I/O")
    commands = result.add_subparsers(dest="command", required=True)
    for name, help_text in (("calibrate", "joint multiscale fixed SE(3) point-to-plane ICP"),
                            ("validate", "validate a frozen transform against independent scenes"),
                            ("ground", "trusted ROI RANSAC ground reference"),
                            ("clock", "fit centered integer-ns arrival clock model"),
                            ("prepare-files", "read explicit paired ASCII PCD/PLY into calibration JSON"),
                            ("prepare-bag", "read whitelisted SourceFrame bag windows and suggest static segments"),
                            ("manual-initial", "solve rigid initial right-to-left transform from matched XYZ pairs"),
                            ("imu-gravity", "native IMU gravity/roll/pitch candidate from explicit-unit JSON samples"),
                            ("imu-gravity-bag", "native IMU gravity candidate from a bounded H30Frame bag window"),
                            ("suggest-static", "shape-stability suggestions only; no automatic validation")):
        command = commands.add_parser(name, help=help_text)
        command.add_argument("--input", required=True)
        command.add_argument("--output-root", required=True)
        command.add_argument("--version", required=True)
        if name in {"calibrate", "validate"}:
            command.add_argument("--quality-profile", help="JSON QualityProfile; persisted exactly with result")
        if name == "calibrate":
            command.add_argument("--initial-result", help="saved manual-initial result; changes only the initial guess")
        if name == "validate":
            command.add_argument("--result", required=True, help="existing calibration result.json")
    report = commands.add_parser("report", help="print status, limits and provenance of a result")
    report.add_argument("--result", required=True)
    listing = commands.add_parser("list", help="list immutable versions and incomplete reservations")
    listing.add_argument("--output-root", required=True)
    return result


def main(argv=None):
    arguments = parser().parse_args(argv)
    try:
        if arguments.command == "list":
            root = Path(arguments.output_root)
            if not root.is_dir() or root.is_symlink():
                raise CalibrationError("version root must be an existing nonsymlink directory")
            versions = []
            for child in sorted(root.iterdir()):
                if child.is_symlink() or not child.is_dir():
                    continue
                result_file = child / "result.json"
                saved = _read(result_file) if result_file.is_file() else {}
                versions.append({"version": child.name, "status": saved.get("status", "INCOMPLETE"),
                                 "source_mode": saved.get("source_mode"), "input_hash": saved.get("input_hash")})
            print(json.dumps({"versions": versions}, ensure_ascii=False))
            return 0
        if arguments.command == "report":
            result = _read(arguments.result)
            print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
            return 0
        data = _read(arguments.input)
        if arguments.command in {"calibrate", "validate"}:
            profile = QualityProfile(**_read(arguments.quality_profile)) if arguments.quality_profile else None
            if arguments.command == "calibrate":
                if arguments.initial_result:
                    from .alignment import attach_manual_initial
                    data = attach_manual_initial(data, _read(arguments.initial_result))
                result = calibrate(data, profile)
                if arguments.initial_result:
                    result["manual_initial_provenance"] = data["manual_initial_provenance"]
            else:
                existing = _read(arguments.result)
                if existing.get("sensor_ids") != data.get("sensor_ids"):
                    raise CalibrationError("validation sensor identities do not match saved calibration")
                if existing.get("source_mode") == "synthetic" and data.get("source_mode") == "real":
                    raise CalibrationError("synthetic calibration cannot be promoted to live; solve from real data")
                if arguments.quality_profile is None and existing.get("quality_profile"):
                    profile = QualityProfile(**existing["quality_profile"])
                result = validate_calibration(data, existing["T_left_right"], profile)
                result["validated_parent_input_hash"] = existing.get("input_hash")
        elif arguments.command == "ground":
            result = fit_ground_plane(**data)
        elif arguments.command == "clock":
            result = fit_clock_model(**data)
        elif arguments.command == "prepare-files":
            from .importers import prepare_file_dataset
            result = prepare_file_dataset(data, Path(arguments.input).resolve().parent)
            result["status"] = "PREPARED_NOT_VALIDATED"
        elif arguments.command == "prepare-bag":
            from .importers import prepare_bag_dataset
            result = prepare_bag_dataset(data)
            result["status"] = "PREPARED_NOT_VALIDATED"
        elif arguments.command == "manual-initial":
            from .alignment import manual_initial
            result = manual_initial(data)
        elif arguments.command in {"imu-gravity", "imu-gravity-bag"}:
            from .alignment import h30_bag_samples, imu_gravity
            result = imu_gravity(h30_bag_samples(data) if arguments.command == "imu-gravity-bag" else data)
        else:
            from .importers import suggest_static_segments
            result = suggest_static_segments(data["pairs"], **data.get("policy", {}))
        location = save_version(result, arguments.output_root, arguments.version)
        print(json.dumps({"result_path": str(location), "status": result["status"],
                          "live_eligible": result.get("live_eligible", False),
                          "rejection_reasons": result.get("rejection_reasons", [])}, ensure_ascii=False))
        return 2 if result["status"] == "UNVALIDATED" else 0
    except (CalibrationError, OSError, ValueError, KeyError, TypeError) as error:
        print(json.dumps({"error": type(error).__name__, "detail": str(error)}, ensure_ascii=False), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
