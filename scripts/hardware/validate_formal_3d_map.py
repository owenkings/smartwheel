#!/usr/bin/env python3
"""Validate an exported SmartWheel formal 3D map bundle.

This command is intentionally fail-closed.  A successful file export or a
large point count is not enough to call a map formal; explicit evidence for
clock synchronisation, calibration, dynamics, real-time behaviour, TF
ownership and repeatability is required.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
from pathlib import Path

# Keep the command usable directly from a source checkout as well as from an
# installed ROS overlay.  The validator itself is ROS-free, so requiring a
# prior ``colcon build`` here would make offline CI and bundle review needlessly
# fragile.
_REPO_ROOT = Path(__file__).resolve().parents[2]
_SOURCE_PACKAGE = _REPO_ROOT / "src" / "smartwheel_map_products"
if _SOURCE_PACKAGE.is_dir() and str(_SOURCE_PACKAGE) not in sys.path:
    sys.path.insert(0, str(_SOURCE_PACKAGE))

from smartwheel_map_products.formal_acceptance import (
    SCHEMA_VERSION,
    template,
    validate_formal_3d_bundle,
)


def _write_json(path: Path, value: dict) -> None:
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        raise FileExistsError(f"refusing to overwrite existing file: {path}")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            descriptor = -1
            json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        temporary.unlink(missing_ok=True)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", nargs="?", help="completed map bundle directory")
    parser.add_argument("--evidence", help="external formal_acceptance.json evidence file")
    parser.add_argument("--calibration-contract", help="approved calibration contract JSON")
    parser.add_argument("--single-lidar", action="store_true", help="diagnostic single-lidar mode; not a formal dual-lidar product")
    parser.add_argument("--no-loop-closure", action="store_true")
    parser.add_argument("--no-repeatability", action="store_true")
    parser.add_argument("--no-intensity", action="store_true")
    parser.add_argument("--require-2d", action="store_true")
    parser.add_argument("--max-time-offset-sec", type=float, default=0.020)
    parser.add_argument("--max-output-gap-sec", type=float, default=0.500)
    parser.add_argument("--max-drop-rate", type=float, default=0.010)
    parser.add_argument("--min-realtime-duration-sec", type=float, default=60.0)
    parser.add_argument("--max-rejected-frame-rate", type=float, default=0.0)
    parser.add_argument("--json", action="store_true", help="print the complete JSON report")
    parser.add_argument("--report-out", help="write the JSON report to a new file")
    parser.add_argument("--template", metavar="PATH", help="write a conservative evidence template and exit")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.template:
        _write_json(Path(args.template), template())
        print(f"wrote schema-{SCHEMA_VERSION} evidence template: {Path(args.template).expanduser().resolve()}")
        return 0
    if not args.bundle:
        _parser().error("bundle is required unless --template is used")
    if any(
        value < 0.0
        for value in (
            args.max_time_offset_sec,
            args.max_output_gap_sec,
            args.max_drop_rate,
            args.min_realtime_duration_sec,
            args.max_rejected_frame_rate,
        )
    ):
        _parser().error("thresholds must be non-negative")
    report = validate_formal_3d_bundle(
        args.bundle,
        evidence_path=args.evidence,
        calibration_contract_path=args.calibration_contract,
        require_dual_lidar=not args.single_lidar,
        require_loop_closure=not args.no_loop_closure,
        require_repeatability=not args.no_repeatability,
        require_intensity=not args.no_intensity,
        require_occupancy=args.require_2d,
        max_time_offset_sec=args.max_time_offset_sec,
        max_output_gap_sec=args.max_output_gap_sec,
        max_drop_rate=args.max_drop_rate,
        min_realtime_duration_sec=args.min_realtime_duration_sec,
        max_rejected_frame_rate=args.max_rejected_frame_rate,
    )
    if args.report_out:
        _write_json(Path(args.report_out), report)
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
    else:
        state = "PASS (formal-ready)" if report["formal_ready"] else "BLOCKED (not formal-ready)"
        print(f"{state}: {report['bundle']}")
        for blocker in report.get("blockers", []):
            print(f"- {blocker}")
    return 0 if report["formal_ready"] else 1


if __name__ == "__main__":
    sys.exit(main())
