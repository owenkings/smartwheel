"""Live shadow chain: rosbag test of the live inputs, and the (prepared) vehicle shadow run.

``inputs``   ros2 bag play of a recording's raw sensor topics into the live inputs node; the scans and the
             odometry are compared with the frozen frontend of the same recording.
``shadow``   live inputs + map_server/AMCL on a saved map + Nav2 + safety gate + operator goals + recorder.
             ``--source bag`` (recorded time) runs now; ``--source vehicle`` is prepared and refuses to start
             without the user's dated approval (see docs/operations/navigation_live_shadow.md).
``checklist`` prints the pre-flight checklist for a vehicle shadow run.
Outputs go to reports/navigation_live_shadow/; nothing commands the wheelchair.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import uuid

import numpy as np

from .scan_adapter import LIDAR_TIME_OFFSET_CONFIG, LIDAR_TIME_OFFSET_SNAPSHOT, load_lidar_time_offset

OUTPUT_PREFIX = "/wc_live/relayed"   # relay_node.OUTPUT_PREFIX (importing it here would pull in rclpy)
VEHICLE_SHADOW_DOMAIN = "100"     # user decision 2026-11-07; localhost only
CHECKLIST = """上车前检查单（实车影子模式，闸门输出不接底盘）
 0. 第一次上车前，总管和用户已评审单向转发（docs/architecture/live_shadow.md“实车影子模式的单向转发”一节）。
 1. 用户在场，并明确同意本次上车运行；把同意写进状态文件，命令前设置 WC_LIVE_SHADOW_VEHICLE_APPROVAL=APPROVED_BY_USER_<日期>。
 2. 影子框架用域 100（ROS_LOCALHOST_ONLY=1，用户 2026-11-07 同意）；面板域（现为 83）用 --sensor-domain 给出。影子框架只经单向转发读三个传感器话题（best effort、volatile、keep last，不参与驱动结束录制时的确认等待），从不在面板域发布。
 3. 先在面板里开始录制或实时建图，等面板显示所有设备就绪、数据在走，再启动影子命令。转发会等到三个话题都已有面板自己的订阅者才订阅（状态 WAITING_FOR_PANEL_SOURCES），自检通过才继续：驱动见到第一个订阅者就开始发，转发不能先于面板的录制连上。
 4. 物理急停可用、轮控所有者照常由面板管理；确认没有任何进程订阅 /wc_nav/cmd_vel_gated，也没有进程发布 /cmd_vel（ros2 topic info 检查）。
 5. 传感器在线：左雷达 host_filtered 帧约 10 Hz、H30 约 200 Hz、轮速反馈约 10 Hz（面板里看，或在影子域看转发后的 /wc_live/relayed/... 话题）。
 6. 配置：上车前运行 navigation_live_shadow freeze-config --config-root /home/nvidia/wheelchair --compare-cell <同一台车最近的单左前端> --gyro-bias <零偏文件>，冻结面板当前的实时配置并记 sha256。确认 hardware_setup 来自 config/local/project/hardware_setup.json（否则会退回仓库基础值）、轮参数是设备当前值（2026-11-07 20:19 起 0.1646/0.599）、外参和主机滤波与最近的前端一致。影子命令用冻结目录作 --cell、其中的 hardware_setup.json 作 --hardware-setup。
 7. 陀螺零偏二选一：现场开头静止做一次 10+10+10 确认，或先用 ZL1107 的零偏文件（同一个 IMU H30-5B8E666933，温度和时间不同）；用 --gyro-bias 给出，IMU 编号必须一致。实时 IMU 用到达时间，暂不需要设备时间模型。雷达时间偏移取冻结目录里的 lidar_time_offset.json（来自 config/calibration/，2026-11-08 用户批准 75 ms，按到达时间估计），freeze-config 的核对结果里应能看到这个值。
 8. 地图：--map 用要导航的那张地图的 map_nav.yaml；起点在地图覆盖范围内，雷达能看到已建图的墙（门内或地图空白处起步时，先推到走廊里再给初始位姿）。
 9. 初始位姿：在已知停靠点启动，或由操作员给 --initial-pose X Y YAW_DEG 和不小于 0.25 m、8.6 度的标准差；不能用零协方差。
10. 目标：--goal 按路线给若干个地图坐标目标；推车的人按路线走，框架只计算不执行。
11. 记录空间：数据根剩余空间足够（每分钟约 20 MB）；结束时 Ctrl-C 停止本命令，只会停掉本命令启动的进程。
12. 运行后核对 result.json：输入状态全程 RUNNING、转发计数和自检、AMCL 收敛（与车位一致）、闸门不可信时长与原因（含原地更新次数）、各级指令与推车运动的对比；面板的录包没有被标成 PARTIAL。
"""


def _finite(value):
    number = float(value)
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError("finite number required")
    return number


def _common(parser):
    parser.add_argument("--project-root", type=Path, required=True)
    parser.add_argument("--cell", type=Path, required=True,
                        help="frozen single-left frontend cell of this hardware (mounts, cloud filter, motion prior)")
    parser.add_argument("--hardware-setup", type=Path, required=True,
                        help="hardware setup whose wheel_odometry block replaces the frozen wheel values")
    parser.add_argument("--gyro-bias", type=Path, required=True, help="confirmed gyro bias file of this IMU")
    parser.add_argument("--bag", type=Path, help="rosbag2 directory (bag/) of a recording, played with --clock")
    parser.add_argument("--bag-rate", type=_finite, default=1.0)
    parser.add_argument("--scan-rate-hz", type=_finite, default=10.0, help="0 keeps every lidar frame")
    parser.add_argument("--relay-test", action="store_true",
                        help="play the bag under a prefix and pass it through the one-way relay (same domain)")
    parser.add_argument("--lidar-time-offset-ms", type=_finite, metavar="MS",
                        help="stamp each scan MS earlier than its lidar frame, because the lidar stamps are late "
                             "against the wheel/IMU odometry; 0 = recorded stamps. Default: the approved value, from "
                             "the frozen configuration given as --cell, else " + LIDAR_TIME_OFFSET_CONFIG)


def resolve_lidar_time_offset(args):
    """(ms, provenance): explicit value, else the frozen configuration's snapshot, else the approved config file."""
    if args.lidar_time_offset_ms is not None:
        return float(args.lidar_time_offset_ms), {"source": "COMMAND_LINE"}
    for candidate in (Path(args.cell).expanduser() / LIDAR_TIME_OFFSET_SNAPSHOT,
                      Path(args.project_root) / LIDAR_TIME_OFFSET_CONFIG):
        if candidate.is_file():
            value, provenance = load_lidar_time_offset(candidate)
            return value, {"source": "CONFIGURATION", **provenance}
    raise SystemExit("no approved lidar time offset (%s, or %s in the frozen configuration): give "
                     "--lidar-time-offset-ms explicitly (0 = recorded stamps)"
                     % (LIDAR_TIME_OFFSET_CONFIG, LIDAR_TIME_OFFSET_SNAPSHOT))


def arguments(argv=None):
    from ..autonomy.shadow_cli import _finite as finite
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    inputs = sub.add_parser("inputs", help="rosbag test of the live inputs against the frozen frontend")
    _common(inputs)
    inputs.add_argument("--compare-cell", type=Path,
                        help="frozen frontend of the same recording to compare with (default --cell; give it when "
                             "--cell is a frozen live configuration)")
    shadow = sub.add_parser("shadow", help="live shadow run (bag now; vehicle only with approval)")
    _common(shadow)
    shadow.add_argument("--source", choices=["bag", "vehicle"], required=True)
    shadow.add_argument("--sensor-domain", type=int, help="vehicle: domain of the panel's sensor topics")
    shadow.add_argument("--map", dest="map_yaml", type=Path, required=True)
    shadow.add_argument("--initial-pose", nargs=3, type=finite, required=True, metavar=("X_M", "Y_M", "YAW_DEG"))
    shadow.add_argument("--initial-pose-std", nargs=2, type=finite, default=[0.25, math.degrees(0.15)],
                        metavar=("XY_M", "YAW_DEG"))
    shadow.add_argument("--goal", action="append", nargs=3, type=finite, metavar=("X_M", "Y_M", "YAW_DEG"))
    shadow.add_argument("--goals-along-cell", type=Path,
                        help="bag test: goals every --goal-spacing m along this frontend's path, placed by --map-alignment-json")
    shadow.add_argument("--map-alignment-json", type=Path)
    shadow.add_argument("--goal-spacing", type=finite, default=8.0)
    shadow.add_argument("--advance-radius", type=finite, default=1.0)
    shadow.add_argument("--amcl-params", default="framework")
    # Defaults are the safety gate's own (2026-11-07): 0.25 m or 10 deg at the robot within 2 s, std <= 0.50 m.
    shadow.add_argument("--gate-correction-limits", nargs=2, type=finite, default=[0.25, 10.0],
                        metavar=("TRANSLATION_M", "YAW_DEG"))
    shadow.add_argument("--gate-covariance-limits", nargs=2, type=finite, default=[0.50, math.degrees(0.25)],
                        metavar=("XY_STD_M", "YAW_STD_DEG"))
    shadow.add_argument("--gate-correction-reference", choices=["odom_origin", "robot"], default="robot")
    shadow.add_argument("--footprint", nargs="+", type=finite)
    shadow.add_argument("--footprint-source", default="EXAMPLE_NOT_MEASURED")
    shadow.add_argument("--max-duration", type=finite, default=900.0, help="wall seconds")
    sub.add_parser("checklist", help="print the pre-flight checklist")
    freeze = sub.add_parser("freeze-config", help="freeze the panel's current live configuration (files only)")
    freeze.add_argument("--project-root", type=Path, required=True)
    freeze.add_argument("--config-root", type=Path, required=True,
                        help="checkout whose configuration the panel uses (on the vehicle: the main checkout)")
    freeze.add_argument("--config", default="config/mapping_live.json", help="live mapping configuration below it")
    freeze.add_argument("--compare-cell", type=Path, help="recent single-left frontend cell to compare the mounts with")
    freeze.add_argument("--gyro-bias", type=Path, help="bias file to use when the live configuration names none")
    args = parser.parse_args(argv)
    if args.command in ("checklist", "freeze-config"):
        return args
    if args.command == "inputs" and args.bag is None:
        parser.error("inputs needs --bag")
    if args.command == "shadow":
        if args.source == "bag" and args.bag is None:
            parser.error("--source bag needs --bag")
        if args.source == "vehicle" and (args.bag is not None or args.sensor_domain is None):
            parser.error("--source vehicle needs --sensor-domain and no --bag")
        if not args.goal and not args.goals_along_cell:
            parser.error("give --goal (repeatable) or --goals-along-cell")
        if args.goals_along_cell and not args.map_alignment_json:
            parser.error("--goals-along-cell needs --map-alignment-json")
        from .amcl_cross import CANDIDATES
        if args.amcl_params != "framework" and args.amcl_params not in CANDIDATES:
            parser.error("--amcl-params must be framework or one of " + ", ".join(CANDIDATES))
        if args.footprint is not None:
            from ..autonomy import nav2_config
            if len(args.footprint) < 6 or len(args.footprint) % 2:
                parser.error("--footprint needs at least three x y pairs")
            args.footprint = [args.footprint[k:k + 2] for k in range(0, len(args.footprint), 2)]
            nav2_config.NavProfile(footprint=args.footprint)
    if not 0.1 <= args.bag_rate <= 2.0:
        parser.error("--bag-rate must be within 0.1..2")
    from .scan_adapter import MAX_LIDAR_TIME_OFFSET_MS
    if args.lidar_time_offset_ms is not None and abs(args.lidar_time_offset_ms) > MAX_LIDAR_TIME_OFFSET_MS:
        parser.error("--lidar-time-offset-ms must be within +-%g" % MAX_LIDAR_TIME_OFFSET_MS)
    return args


def goals_along(cell, alignment_json, spacing):
    """Waypoints every ``spacing`` metres along a frontend's axle path (mapped part), placed in the map."""
    from ..autonomy.shadow_localization import compose_poses
    from .bag_check import frozen_frontend
    stamps, _, axle, _ = frozen_frontend(cell)
    best = json.loads(Path(alignment_json).read_text(encoding="utf-8"))["best_alignment"]
    placed = compose_poses((best["dx_m"], best["dy_m"], math.radians(best["yaw_deg"])), axle)
    distance = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(placed[:, :2], axis=0).T))])
    goals, next_at = [], spacing
    for k in range(1, len(placed)):
        if distance[k] >= next_at:
            goals.append([float(v) for v in placed[k]])
            next_at += spacing
    goals.append([float(v) for v in placed[-1]])
    return goals


def main(argv=None):
    args = arguments(argv)
    if args.command == "checklist":
        print(CHECKLIST)
        return 0
    from wc_runtime.storage_policy import StoragePolicy
    if args.command == "freeze-config":
        from .current_config import freeze
        storage = StoragePolicy(args.project_root)
        storage.check()
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        output = storage.resolve("reports/navigation_live_shadow/" + stamp + "_config_" + uuid.uuid4().hex[:8])
        record = freeze(output, args.config_root, "wc_live_freeze_" + stamp, args.config,
                        compare_cell=args.compare_cell, gyro_bias=args.gyro_bias)
        storage.check()
        print(json.dumps({"status": "FROZEN", "output": str(output), "checks": record["checks"], "use": record["use"]},
                         ensure_ascii=False, indent=1, default=str), flush=True)
        return 0
    from ..autonomy import nav2_config
    from ..autonomy.bringup import domain_lock
    from .bag_check import check_run
    from .config import load_live_config
    from .supervisor import LiveSupervisor, VEHICLE_APPROVAL_ENV, vehicle_approved
    if args.command == "shadow" and args.source == "vehicle":
        import os
        if not vehicle_approved():
            print(CHECKLIST)
            raise SystemExit("vehicle shadow runs need the user present and %s=APPROVED_BY_USER_<date>"
                             % VEHICLE_APPROVAL_ENV)
        if os.environ.get("ROS_DOMAIN_ID") != VEHICLE_SHADOW_DOMAIN:
            raise SystemExit("the vehicle shadow runs in domain %s (user decision 2026-11-07); "
                             "set WC_AUTONOMY_ROS_DOMAIN=%s" % (VEHICLE_SHADOW_DOMAIN, VEHICLE_SHADOW_DOMAIN))
    storage = StoragePolicy(args.project_root)
    storage.check()
    live = load_live_config(args.cell, args.hardware_setup, args.gyro_bias)
    lidar_offset_ms, lidar_offset_source = resolve_lidar_time_offset(args)
    run_id = (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + args.command + "_" + uuid.uuid4().hex[:8])
    output = storage.resolve("reports/navigation_live_shadow/" + run_id)
    bag = None if args.bag is None else args.bag.expanduser().resolve(strict=True)
    config = {"schema_version": 1, "kind": args.command, "source": "bag" if bag else "vehicle",
              "bag": None if bag is None else str(bag), "bag_rate": args.bag_rate,
              "use_sim_time": bag is not None, "scan_rate_hz": args.scan_rate_hz,
              "lidar_time_offset_ms": lidar_offset_ms, "lidar_time_offset_source": lidar_offset_source,
              "live_config_file": str(output / "live_config.json"), "record_scan_ranges": args.command == "inputs",
              "hardware_control": False, "commands_sent": False, "open_loop": True}
    if args.relay_test:
        config["bag_remap_prefix"] = "/relay_source"
        config["relay"] = {"source_domain": None, "source_prefix": "/relay_source", "output_prefix": OUTPUT_PREFIX,
                           "wait_for_panel_consumers": False}
        config["source_prefix"] = OUTPUT_PREFIX
    if args.command == "shadow":
        from .amcl_cross import CANDIDATES
        map_yaml = args.map_yaml.expanduser().resolve(strict=True)
        profile = nav2_config.NavProfile(footprint=args.footprint or ())
        goals = [[g[0], g[1], math.radians(g[2])] for g in (args.goal or [])]
        if args.goals_along_cell:
            goals += goals_along(args.goals_along_cell, args.map_alignment_json, args.goal_spacing)
            config["reference"] = {"cell": str(args.goals_along_cell.expanduser().resolve(strict=True)),
                                   "alignment_json": str(args.map_alignment_json.expanduser().resolve(strict=True))}
        config.update(profile=profile.as_dict(), map_yaml=str(map_yaml),
                      body_model=("POLYGON_" + args.footprint_source) if args.footprint else "CIRCLE_ASSUMED_NOT_MEASURED",
                      amcl={"params": args.amcl_params, "changes_from_framework": dict(CANDIDATES.get(args.amcl_params, {}))},
                      gate_limits={"correction_translation_m": args.gate_correction_limits[0],
                                   "correction_yaw_deg": args.gate_correction_limits[1],
                                   "correction_reference": args.gate_correction_reference,
                                   "covariance_xy_std_m": args.gate_covariance_limits[0],
                                   "covariance_yaw_std_deg": args.gate_covariance_limits[1]},
                      initial_pose=[args.initial_pose[0], args.initial_pose[1], math.radians(args.initial_pose[2])],
                      initial_pose_std=list(args.initial_pose_std), goals=goals, advance_radius_m=args.advance_radius,
                      max_duration_s=args.max_duration)
        if args.source == "vehicle":
            config["relay"] = {"source_domain": args.sensor_domain, "source_prefix": "", "output_prefix": OUTPUT_PREFIX,
                               "wait_for_panel_consumers": True}
            config["source_prefix"] = OUTPUT_PREFIX
    with domain_lock(args.project_root):
        output.mkdir(parents=True, exist_ok=False)
        if config.get("relay") is not None and config["relay"]["source_domain"] is None:
            import os
            config["relay"]["source_domain"] = int(os.environ["ROS_DOMAIN_ID"])
        (output / "live_config.json").write_text(json.dumps(live, ensure_ascii=False, indent=1), encoding="utf-8")
        (output / "run_config.json").write_text(json.dumps(config, ensure_ascii=False, indent=2), encoding="utf-8")
        report = LiveSupervisor(args.project_root, output, config).run()
    storage.check()
    result = {"schema_version": 1, "kind": args.command, "status": "INCOMPLETE", "config": config,
              "live_provenance": live["provenance"], "supervisor": report,
              "hardware_control": False, "commands_sent": False, "physical_navigation_validated": False}
    if args.command == "inputs":
        result["comparison"] = check_run(output, args.compare_cell or args.cell, config["lidar_time_offset_ms"])
        scans = result["comparison"]["scans"]
        odometry = result["comparison"]["odometry"]
        result["status"] = ("COMPLETE" if "error" not in report and scans["matched_frames"] > 0
                            and odometry.get("status") == "COMPARED" else "INCOMPLETE")
    else:
        from .shadow_summary import summarize_shadow
        result.update(summarize_shadow(config, output))
        result["status"] = "COMPLETE" if "error" not in report and result.get("commands") else "INCOMPLETE"
    with (output / "result.json").open("x", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False, default=float)
    brief = {"status": result["status"], "output": str(output), "error": report.get("error")}
    if args.command == "inputs":
        brief["scans"] = {k: result["comparison"]["scans"][k] for k in ("matched_frames", "identical_frames",
                                                                         "differing_bins_total", "unmatched_frames")}
        brief["odometry"] = result["comparison"]["odometry"]
    else:
        brief.update({k: result.get(k) for k in ("outcomes_while_moving", "gate_untrusted_s", "localization")})
    print(json.dumps(brief, ensure_ascii=True, default=float), flush=True)
    return 0 if result["status"] == "COMPLETE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
