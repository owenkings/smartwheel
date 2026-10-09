"""Local/offline CLI. No command starts a sensor, ROS node, or actuator."""

import argparse
import json
import sqlite3
import sys

from .builder import MapError
from .store import MapStore


def main(argv=None):
    parser = argparse.ArgumentParser(description="Offline versioned map packages; loading is view-only")
    parser.add_argument("--root", required=True, help="Explicit project-owned map storage directory")
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ("save", "verify", "inspect", "load"):
        command = commands.add_parser(name)
        command.add_argument("--map-id", required=True)
        command.add_argument("--version", required=True)
        if name == "save":
            command.add_argument("--snapshot", required=True)
    for name in ("list", "version"):
        command = commands.add_parser(name)
        command.add_argument("--map-id", required=True)
    goals = commands.add_parser("goals")
    goals.add_argument("--map-id", required=True)
    goals.add_argument("--version", required=True)
    actions = goals.add_subparsers(dest="action", required=True)
    actions.add_parser("list")
    edit = actions.add_parser("set")
    edit.add_argument("--name", required=True)
    edit.add_argument("--position", type=float, nargs=3, required=True)
    edit.add_argument("--orientation-xyzw", type=float, nargs=4, required=True)
    review = actions.add_parser("review")
    review.add_argument("--name", required=True)
    review.add_argument("--state", choices=("UNVERIFIED", "VERIFIED"), required=True)
    migrate = actions.add_parser("migrate")
    migrate.add_argument("--from-version", required=True)
    args = parser.parse_args(argv)
    try:
        store = MapStore(args.root)
        if args.command == "save":
            result = store.save_snapshot(args.snapshot, args.map_id, args.version)
        elif args.command in ("verify", "inspect"):
            result = store.verify(args.map_id, args.version)
        elif args.command == "load":
            result = store.load(args.map_id, args.version)
        elif args.command in ("list", "version"):
            result = store.list_versions(args.map_id)
        elif args.action == "list":
            result = store.get_goals(args.map_id, args.version)
        elif args.action == "set":
            result = store.set_goal(args.map_id, args.version, {"name": args.name, "position": args.position, "orientation_xyzw": args.orientation_xyzw})
        elif args.action == "review":
            result = store.review_goal(args.map_id, args.version, args.name, args.state == "VERIFIED")
        else:
            result = store.migrate_goals(args.map_id, args.from_version, args.version)
        print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False))
        return 0
    except (MapError, OSError, ValueError, KeyError, TypeError, sqlite3.Error) as exc:
        print(json.dumps({"status": "ERROR", "error": str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
