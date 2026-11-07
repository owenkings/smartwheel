"""Add closed-loop simulation runs and recorded-data shadow runs to a copy of an offline progress snapshot.

The base snapshot is read through the dashboard validator and never modified.
Each run contributes its result.json hash, headline metrics and its preview
image (embedded as a PNG data URL; shadow previews as a narrower copy). The
new snapshot keeps the offline, not-live and physically-unvalidated flags, and
is rendered with the existing dashboard template. Outputs are created only
inside configured reports storage.
"""
from __future__ import annotations

import argparse
import base64
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

MAX_RESULT_BYTES = 2_000_000
MAX_PREVIEW_BYTES = 1_000_000
STATUS_LABELS = {"PASS": "完成，无仿真碰撞", "PARTIAL": "未全部完成，无仿真碰撞", "FAIL": "失败或有仿真碰撞"}
SHADOW_STATUS_LABELS = {"COMPLETE": "完整回放", "INCOMPLETE": "未完整回放"}
SHADOW_PREVIEW_WIDTH_PX = 1100     # embedded copy only; keeps the snapshot inside the dashboard size bound
FAILURE_LABELS = {
    "START_WITHIN_LETHAL_RADIUS_OF_OCCUPIED": "起点离地图障碍不足 0.65 m",
    "START_WITHIN_LETHAL_RADIUS_OF_UNKNOWN": "起点贴近未知格",
    "INFLATED_UNKNOWN_CUTS_THE_ROUTE": "未知格膨胀后截断路线",
    "OCCUPIED_CELLS_CUT_THE_ROUTE_OR_GOAL": "地图障碍截断路线或终点",
    "LIVE_COSTMAP_BLOCKS_THE_START": "实时扫描标记压住起点",
    "LIVE_COSTMAP_CUTS_THE_ROUTE": "实时扫描标记截断路线",
    "NOT_EXPLAINED_BY_RECORDED_COSTMAP": "代价地图未能解释",
    "NO_RECORDED_COSTMAP": "尚无代价地图记录",
    "OUTSIDE_MAP": "超出地图",
}
SHADOW_NOTES = [
    "开环：录包里的轮椅照原样移动，不受 Nav2 指令影响；结果说明真实传感器数据上的规划、停车与一致性，不代表 Nav2 接管后的行为。",
    "地图坐标即录包的轮速+IMU 里程计坐标，定位按恒等关系给出，没有检验定位；车体仍是 0.55 m 圆形假设加 0.10 m 余量。",
    "速度上限 0.30 m/s，司机录包时常在 0.5–0.9 m/s，所以只比较方向和是否前进，不比较速度大小。",
]
MAPPER_LABELS = {"odom_icp": "里程计+扫描匹配", "odom_grid": "纯里程计", "slam_toolbox": "slam_toolbox"}
NOTES = [
    "仿真轮椅：圆形假设包络 0.55 m 加 0.10 m 余量；平面激光按左雷达约 ±57.5° 视场、0.3–8 m 建模。",
    "定位误差用仿真真值计算，实车没有对应真值；仿真通过不代表实车可以运行。",
    "速度链路只经平滑器、碰撞监控和安全闸门到仿真底盘，不发布 /cmd_vel，也不连接轮控。",
]


def _sha256(data):
    return hashlib.sha256(data).hexdigest()


def _bounded(path, limit):
    path = Path(path)
    if path.is_symlink() or not path.is_file() or path.stat().st_size > limit:
        raise ValueError("ordinary bounded file required: " + str(path))
    return path.read_bytes()


def run_row(directory, *, preview=True):
    directory = Path(directory).resolve(strict=True)
    raw = _bounded(directory / "result.json", MAX_RESULT_BYTES)
    result = json.loads(raw.decode("utf-8"))
    if result.get("simulation_only") is not True or result.get("hardware_control") is not False:
        raise ValueError("only simulation results without hardware control can be shown: " + str(directory))
    config = result["config"]
    world = config["world"].get("name") or Path(config["world"].get("yaml", "map")).parent.name
    if config["world"].get("kind") == "synthetic":
        world += " r%d" % int(config["world"].get("revision", 1))  # layout revision
    mode = result["mode"]
    trajectory = result.get("trajectory") or {}
    error = trajectory.get("localization_position_error_m") or {}
    yaw = trajectory.get("localization_yaw_error_deg") or {}
    odometry = trajectory.get("odometry_position_error_m") or {}
    localization = ("均值 %.2f m · 最大 %.2f m" % (error["mean"], error["max"])) if error else "未提供"
    if error and yaw:
        localization += " · 航向最大 %.1f°" % yaw["max"]
    if error and odometry:
        localization += "（纯里程计最大 %.2f m）" % odometry["max"]
    if mode == "explore":
        coverage = result.get("coverage") or {}
        explored = result.get("exploration") or {}
        goals = [g for g in explored.get("goals", []) if g.get("kind") == "frontier"]
        outcome = "覆盖 %.0f%% · 误判空地 %.2f m² · 前沿目标 %d/%d 到达 · %s" % (
            100 * coverage.get("free_coverage_fraction", 0.0), coverage.get("false_free_area_m2", 0.0),
            sum(1 for g in goals if g.get("succeeded")), len(goals), explored.get("status", "未完成"))
        mapper = MAPPER_LABELS.get(config.get("mapper", "slam_toolbox"), config.get("mapper"))
        label = "自主探索建图 · " + world + " · " + mapper
    else:
        outcomes = result.get("goal_outcomes") or []
        reached = [o for o in outcomes if o.get("succeeded")]
        worst = max((o.get("true_position_error_m", 0.0) for o in reached), default=None)
        outcome = "目标 %d/%d 到达" % (len(reached), len(outcomes)) + (
            " · 终点真值误差 ≤ %.2f m" % worst if worst is not None else "")
        if config["world"].get("obstacles"):
            outcome += " · 地图外障碍 %d 个" % len(config["world"]["obstacles"])
        label = "已保存地图目标导航 · " + world
    row = {"run_id": directory.name, "label": label, "world": world, "mode": mode,
           "status": result["status"], "status_label": STATUS_LABELS.get(result["status"], result["status"]),
           "distance_m": result.get("distance_m"), "collisions": result["collisions"]["count"],
           "localization_label": localization, "outcome_label": outcome,
           "source": {"path": str(directory / "result.json"), "bytes": len(raw), "sha256": _sha256(raw)}}
    image = directory / "preview.png"
    if preview and image.exists():
        data = _bounded(image, MAX_PREVIEW_BYTES)
        if not data.startswith(b"\x89PNG\r\n\x1a\n"):
            raise ValueError("preview is not a PNG: " + str(image))
        row["preview_data_url"] = "data:image/png;base64," + base64.b64encode(data).decode("ascii")
        row["preview_sha256"] = _sha256(data)
    return row


def _percent(value):
    return "未提供" if value is None else "%.0f%%" % (100 * value)


def _scaled_png(data, max_width):
    """The dashboard copy of a preview, at most ``max_width`` pixels wide; the run keeps its original file."""
    from io import BytesIO
    from PIL import Image
    with Image.open(BytesIO(data)) as image:
        image.load()
        if image.width <= max_width:
            return data
        size = (int(max_width), max(1, round(image.height * max_width / image.width)))
        scaled = image.convert("RGB").resize(size, Image.LANCZOS)
    out = BytesIO()
    scaled.save(out, format="PNG", optimize=True)
    return out.getvalue()


def shadow_row(directory, *, preview=True):
    """One open-loop Nav2 shadow run on a recording: headline agreement, planning and stop numbers."""
    directory = Path(directory).resolve(strict=True)
    raw = _bounded(directory / "result.json", MAX_RESULT_BYTES)
    result = json.loads(raw.decode("utf-8"))
    if (result.get("mode") != "shadow" or result.get("open_loop") is not True
            or result.get("commands_sent") is not False or result.get("hardware_control") is not False):
        raise ValueError("only open-loop shadow results that sent no command can be shown: " + str(directory))
    config = result["config"]
    commands = result.get("commands") or {}
    planner = result.get("planner") or {}
    fractions = commands.get("outcome_fractions_while_recording_moves") or {}
    reasons = sorted((planner.get("failure_reasons") or {}).items(), key=lambda item: -item[1])
    turns = commands.get("turn_direction_agreement") or {}
    requests = planner.get("plan_requests") or 0
    succeeded = round((planner.get("plan_success_fraction") or 0.0) * requests)
    window = config.get("window_s") or [0.0, 0.0]
    recording = Path(config["source_cell"]).parent.name
    try:  # the frontend analysis records which capture it came from
        frontend = json.loads(_bounded(Path(config["source_cell"]).parent / "result.json", MAX_RESULT_BYTES))
        recording = Path(frontend["dataset"]).name
    except (OSError, ValueError, KeyError, TypeError):
        pass
    mode = result.get("shadow_mode", config.get("shadow_mode", "navigate"))
    profile = config.get("profile") or {}
    label = "录包影子 · " + ("Nav2 规划并跟踪" if mode == "navigate" else "控制器沿司机路线")
    if profile.get("radius_m") is not None:
        label += " · 假设包络 %.2f m" % profile["radius_m"]
    row = {"run_id": directory.name, "label": label, "shadow_mode": mode, "recording": recording,
           "window_label": "%s · %.0f–%.0f s" % (recording, window[0], window[1]),
           "map_label": Path(config["map_yaml"]).parent.parent.name + "/" + Path(config["map_yaml"]).parent.name
                        + "/" + Path(config["map_yaml"]).name,
           "status": result["status"], "status_label": SHADOW_STATUS_LABELS.get(result["status"], result["status"]),
           "drive_label": _percent(commands.get("drives_forward_fraction_while_recording_drives_forward")),
           "plan_label": ("%d/%d 成功" % (succeeded, requests) if requests
                          else "不经规划（沿司机路线）" if mode == "follow" else "未提供"),
           "failure_label": "；".join("%s %d" % (FAILURE_LABELS.get(k, k), v) for k, v in reasons) or "无",
           "turn_label": ("%s（%d 个样本）" % (_percent(turns.get("fraction")), turns.get("samples", 0))
                          if turns.get("fraction") is not None else "未提供"),
           "monitor_label": "停车 %s · 减速 %s · 无指令 %s" % (
               _percent(fractions.get("STOPPED_BY_COLLISION_MONITOR")),
               _percent(commands.get("slowed_by_collision_monitor_fraction_while_moving")),
               _percent(fractions.get("NAV2_NOT_COMMANDING"))),
           "source": {"path": str(directory / "result.json"), "bytes": len(raw), "sha256": _sha256(raw)}}
    image = directory / "preview.png"
    if preview and image.exists():
        data = _bounded(image, MAX_PREVIEW_BYTES)
        if not data.startswith(b"\x89PNG\r\n\x1a\n"):
            raise ValueError("preview is not a PNG: " + str(image))
        embedded = _scaled_png(data, SHADOW_PREVIEW_WIDTH_PX)
        row["preview_data_url"] = "data:image/png;base64," + base64.b64encode(embedded).decode("ascii")
        row["preview_sha256"] = _sha256(data)
        row["embedded_preview_sha256"] = _sha256(embedded)
    return row


def _add_completed(summary, item):
    completed = list(summary.get("completed", []))
    if all(entry.get("work") != item["work"] for entry in completed):
        completed.append(item)
    summary["completed"] = completed


def updated_snapshot(base, rows, *, shadow_rows=(), tests=None, next_remote=None, notes=(), shadow_notes=()):
    """New snapshot dict; ``rows`` replace the simulation table when given, ``shadow_rows`` add the shadow table."""
    snapshot = json.loads(json.dumps(base))
    meta = snapshot["snapshot"]
    meta.update(label="已核验结果的离线快照（含仿真闭环与录包影子运行）" if shadow_rows else "已核验结果的离线快照（含仿真闭环）",
                live_sensor_feed=False, physical_navigation_validated=False, hardware_started=False,
                control_topics_published=False)
    snapshot["generated_at"] = datetime.now(timezone.utc).replace(microsecond=0).isoformat()
    summary = snapshot["summary"]
    if rows:
        passed = sum(1 for r in rows if r["status"] == "PASS")
        snapshot["autonomy"] = {
            "status_label": "仿真闭环 %d/%d 完成" % (passed, len(rows)),
            "summary": ("仿真轮椅在独立 ROS 域中完成自主探索建图、保存地图后的 AMCL 目标导航；"
                        "规划、控制、碰撞监控和安全闸门组成同一条速度链路。所有结果仅为仿真。"),
            "runs": list(rows), "notes": list(NOTES) + [str(n) for n in notes],
            "simulation_only": True, "physical_navigation_validated": False}
        _add_completed(summary, {
            "work": "自主框架仿真闭环：探索建图、目标导航与安全闸门",
            "use": "在不依赖外参和实车的情况下，把建图、规划、控制和停车约束串成可运行、可回归的整体框架。",
            "limit": "仿真世界与传感器模型是假设；SLAM 参数、速度与闸门阈值接实车前需影子模式和现场测量复核。"})
    elif notes and snapshot.get("autonomy"):
        snapshot["autonomy"]["notes"] = list(snapshot["autonomy"].get("notes", [])) + [str(n) for n in notes]
    if shadow_rows:
        complete = sum(1 for r in shadow_rows if r["status"] == "COMPLETE")
        snapshot["shadow"] = {
            "status_label": "录包影子 %d/%d 完整" % (complete, len(shadow_rows)),
            "summary": ("冻结录包里的轮椅运动带动 base_link，录到的扫描进入代价地图和碰撞监控；Nav2 朝录包中稍后的位姿"
                        "规划并照常下发速度，各级指令只记录、不发出，再与司机的实际运动对比。"),
            "runs": list(shadow_rows), "notes": list(SHADOW_NOTES) + [str(n) for n in shadow_notes],
            "open_loop": True, "commands_sent": False, "physical_navigation_validated": False}
        _add_completed(summary, {
            "work": "真实录包影子运行：Nav2 全链路",
            "use": "第一次把规划、控制、碰撞监控和闸门放到真实扫描上跑，并把每次规划失败归到地图、起点或实时障碍。",
            "limit": "开环且定位按里程计恒等给出；车体包络、制动和实车定位仍待现场测量与验收。"})
    if tests:
        summary["tests"] = dict(summary.get("tests", {}), **tests)
        summary["tests_passed"] = tests.get("passed", summary.get("tests_passed"))
    if next_remote:
        summary["next_remote"] = list(next_remote)
    provenance = snapshot.setdefault("provenance", {})
    files = provenance.setdefault("files", {})
    for row in rows:
        files["autonomy:" + row["run_id"] + "/result.json"] = dict(row["source"])
    for row in shadow_rows:
        files["shadow:" + row["run_id"] + "/result.json"] = dict(row["source"])
    provenance["previous_snapshot_sha256"] = _sha256(json.dumps(base, ensure_ascii=False, sort_keys=True).encode())
    return snapshot


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, required=True)
    parser.add_argument("--base-snapshot", type=Path, required=True)
    parser.add_argument("--run", type=Path, action="append", default=[],
                        help="simulation result directory; when given, the simulation table is replaced")
    parser.add_argument("--compare-run", type=Path, action="append", default=[],
                        help="result directory listed in the table without its preview image")
    parser.add_argument("--shadow-run", type=Path, action="append", default=[],
                        help="result directory of a Nav2 shadow run on a recording (navigation_nav2_shadow)")
    parser.add_argument("--shadow-compare-run", type=Path, action="append", default=[],
                        help="shadow run listed in the table without its preview image")
    parser.add_argument("--shadow-note", action="append", default=[], help="extra line under the shadow table")
    parser.add_argument("--output-dir", type=Path, required=True, help="new directory under reports/")
    parser.add_argument("--tests-passed", type=int)
    parser.add_argument("--tests-source")
    parser.add_argument("--next-remote", action="append", default=[])
    parser.add_argument("--note", action="append", default=[], help="extra line under the autonomy table")
    args = parser.parse_args(argv)
    if not (args.run or args.shadow_run):
        parser.error("give at least one --run or --shadow-run")
    if args.compare_run and not args.run:
        parser.error("--compare-run needs --run")
    if args.shadow_compare_run and not args.shadow_run:
        parser.error("--shadow-compare-run needs --shadow-run")
    from wc_runtime.storage_policy import StoragePolicy
    from ..dashboard import read_snapshot, render_html
    policy = StoragePolicy(args.project_root)
    policy.check()
    output = policy.resolve(args.output_dir)
    logical = Path(policy.logical(output))
    if not logical.parts or logical.parts[0] != "reports" or output.exists():
        raise ValueError("output must be a new directory inside configured reports storage")
    base, base_raw = read_snapshot(args.base_snapshot)
    rows = [run_row(d) for d in args.run] + [run_row(d, preview=False) for d in args.compare_run]
    shadow_rows = ([shadow_row(d) for d in args.shadow_run]
                   + [shadow_row(d, preview=False) for d in args.shadow_compare_run])
    tests = None
    if args.tests_passed is not None:
        tests = {"passed": args.tests_passed, "total": args.tests_passed, "failed": 0, "skipped": 0,
                 "source": args.tests_source, "meaning": "导航与自主框架软件测试；不是实车验收"}
    snapshot = updated_snapshot(base, rows, shadow_rows=shadow_rows, tests=tests,
                                next_remote=args.next_remote or None, notes=args.note, shadow_notes=args.shadow_note)
    template = (Path(__file__).resolve().parent.parent / "progress_dashboard.html").read_text(encoding="utf-8")
    encoded = json.dumps(snapshot, ensure_ascii=False, allow_nan=False, indent=1)
    output.mkdir(parents=True)
    snapshot_path = output / "snapshot.json"
    with snapshot_path.open("x", encoding="utf-8") as stream:
        stream.write(encoded)
    checked, raw = read_snapshot(snapshot_path)  # same validator as the renderer
    rendered = render_html(template, checked)
    with (output / "index.html").open("x", encoding="utf-8") as stream:
        stream.write(rendered)
    if args.base_snapshot.read_bytes() != base_raw:
        raise ValueError("base snapshot changed while updating")
    policy.check()
    print(json.dumps({"status": "READ_ONLY_OFFLINE_DASHBOARD_UPDATED", "output": str(output / "index.html"),
                      "snapshot_sha256": _sha256(raw), "runs": [r["run_id"] for r in rows],
                      "shadow_runs": [r["run_id"] for r in shadow_rows],
                      "physical_navigation_validated": False, "hardware_started": False}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
