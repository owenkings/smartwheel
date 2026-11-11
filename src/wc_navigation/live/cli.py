"""Live shadow chain: rosbag test of the live inputs, and the (prepared) vehicle shadow run.

``inputs``   ros2 bag play of a recording's raw sensor topics into the live inputs node; the scans and the
             odometry are compared with the frozen frontend of the same recording.
``shadow``   live inputs + map_server/AMCL on a saved map + Nav2 + safety gate + operator goals + recorder.
             ``--source bag`` (recorded time) runs now; ``--source vehicle`` is prepared and refuses to start
             without the user's dated approval (see docs/operations/navigation_live_shadow.md).
``shadow --closed-loop`` drives: window A's base bridge hands the gate's decisions to the wheel serial owner of
             the running panel session (0.10 m/s, 0.30 rad/s; today's WC_AUTO_DRIVE_APPROVAL required).
``checklist`` prints the pre-flight checklist for a vehicle shadow run.
``relay-graph`` read-only query of the relay's entry node in the sensor domain (creates no endpoint there).
Outputs go to reports/navigation_live_shadow/. Without --closed-loop nothing commands the wheelchair. The first
Ctrl-C stops a run; later ones cannot interrupt the shutdown, and result.json is written either way.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import uuid

import numpy as np

from .scan_adapter import LIDAR_TIME_OFFSET_CONFIG, LIDAR_TIME_OFFSET_SNAPSHOT, load_lidar_time_offset

OUTPUT_PREFIX = "/wc_live/relayed"   # relay_node.OUTPUT_PREFIX (importing it here would pull in rclpy)
VEHICLE_SHADOW_DOMAIN = "100"     # user decision 2026-11-07; localhost only
VEHICLE_SHADOW_NICENESS = 10      # 2026-11-09 field result: the panel's recording comes first, the relay ends at 19
WORK_ROOT = Path("/dev/shm")      # where a run's processes write while it runs (see run_command)
# Vehicle closed loop (BOARD 2026-11-10 14:40, D.md 16:45). The first closed loop runs at 0.10 m/s and 0.30 rad/s;
# the gate slows to 0.05 m/s while the deviation along the heading is above 0.50 m (velocity_channel.md 9.2
# item 8) and takes the enable as stale after 0.5 s (item 2). Higher speeds need new evidence and the user.
CLOSED_LOOP_MAX_LINEAR, CLOSED_LOOP_MAX_ANGULAR = 0.10, 0.30
CLOSED_LOOP_SLOW_LIMITS = (0.50, 0.05)
CLOSED_LOOP_ENABLE_TIMEOUT_S = 0.5
CLOSED_LOOP_START_CHECKS = ("block", "strict")
CLOSED_LOOP_SMOOTHER, CLOSED_LOOP_SMOOTHER_TIMEOUT_S = "open_loop", 0.3   # window B's PlanningChoice; gated later
AUTO_DRIVE_APPROVAL_ENV = "WC_AUTO_DRIVE_APPROVAL"   # the same variable as window A's base bridge and wheel owner
BRIDGE_NODE = "/wc_auto_bridge"                      # wc_motion.auto_bridge: the only enable and estop publisher
CHECKLIST = """上车前检查单（实车影子模式，闸门输出不接底盘）
 0. 第一次上车前，总管和用户已评审单向转发（docs/architecture/live_shadow.md“实车影子模式的单向转发”一节）。
 1. 用户在场，并明确同意本次上车运行；把同意写进状态文件，命令前设置 WC_LIVE_SHADOW_VEHICLE_APPROVAL=APPROVED_BY_USER_<日期>。
 2. 影子框架用域 100（ROS_LOCALHOST_ONLY=1，用户 2026-11-07 同意）；面板域（现为 83）用 --sensor-domain 给出。影子框架只经单向转发读三个传感器话题（best effort、volatile、keep last，不参与驱动结束录制时的确认等待），从不在面板域发布。开三维障碍输入时多读右雷达 host_filtered，共四个话题（用户 2026-11-10 同意）。
 3. 先在面板里开始录制或实时建图，等面板显示所有设备就绪、数据在走，再启动影子命令。转发会等到要转发的话题都已有面板自己的订阅者才订阅（状态 WAITING_FOR_PANEL_SOURCES），自检通过才继续：驱动见到第一个订阅者就开始发，转发不能先于面板的录制连上。开障碍输入时面板要用两台雷达录制或建图，否则右雷达没有发布者，转发会一直等。
    优先级：面板的录制优先。车上模式整个影子命令以 nice 10 运行，命令自己会把 nice 调到不低于 10，转发再降到 19（2026-11-09 第二次这样跑：录包 COMPLETE，里程计落后最大 0.43 s）。result.json 的 supervisor.niceness 记实际值。
    转发 RUNNING 后，另开终端，用同样的环境变量运行 navigation_live_shadow relay-graph --sensor-domain <面板域>（开障碍输入时加 --obstacles）。status 应为 PASS：入口节点只有三个订阅（开障碍时四个），都是 best effort、volatile；没有发布者、服务、客户端；查询节点自己也没有端点。不要在面板域用 ros2 命令（ros2 node info、ros2 topic list 等），它们会在面板域起 ros2 daemon，那是一个有发布者和服务的节点。
 4. 物理急停可用、轮控所有者照常由面板管理；在影子域（100）里用 ros2 topic info 确认：订阅 /wc_nav/cmd_vel_gated 的只有平滑器反馈节点 wc_velocity_feedback（--smoother-feedback gated，默认；它只把闸门输出转成平滑器的反馈），没有进程发布 /cmd_vel。
 5. 传感器在线：左雷达 host_filtered 帧约 10 Hz、H30 约 200 Hz、轮速反馈约 10 Hz；开障碍输入时右雷达 host_filtered 也要约 10 Hz（面板里看，或在影子域看转发后的 /wc_live/relayed/... 话题）。
 6. 配置：上车前运行 navigation_live_shadow freeze-config --config-root /home/nvidia/wheelchair --compare-cell <同一台车最近的单左前端> --gyro-bias <零偏文件>，冻结面板当前的实时配置并记 sha256。确认 hardware_setup 来自 config/local/project/hardware_setup.json（否则会退回仓库基础值）、轮参数是设备当前值（2026-11-07 20:19 起 0.1646/0.599）、外参和主机滤波与最近的前端一致。影子命令用冻结目录作 --cell、其中的 hardware_setup.json 作 --hardware-setup。
 7. 陀螺零偏二选一：现场开头静止做一次 10+10+10 确认，或先用 ZL1107 的零偏文件（同一个 IMU H30-5B8E666933，温度和时间不同）；用 --gyro-bias 给出，IMU 编号必须一致。实时 IMU 用到达时间，暂不需要设备时间模型。雷达时间偏移取冻结目录里的 lidar_time_offset.json（来自 config/calibration/，2026-11-08 用户批准 75 ms，按到达时间估计），freeze-config 的核对结果里应能看到这个值。
 8. 地图：--map 用要导航的那张地图的 map_nav.yaml；起点在地图覆盖范围内，雷达能看到已建图的墙（门内或地图空白处起步时，先推到走廊里再给初始位姿）。
 9. 初始位姿：在已知停靠点启动，或由操作员给 --initial-pose X Y YAW_DEG 和不小于 0.25 m、8.6 度的标准差；不能用零协方差。
    车头朝向必须和 YAW_DEG 一致。方向一律按看板地图说：右 = 地图 +x，左 = 地图 −x。不要用东西南北，现实里地图 −x 是东。
    ZL1107 地图的起点 (−0.323, −0.229, 0°) 是车头朝看板地图的右边：往前约 5 m 直走廊到头，接开阔区和一条拐出去的走廊；车后是约 12 m 长的直走廊。
    2026-11-09 两次车头都朝左，和给定航向差 180°。走廊两边对称，AMCL 和闸门在前 4–5 m 都看不出来。
    起点检查：Nav2 和闸门启动之前，命令用静止扫描核对给定位姿，打印"起点检查…"一行，看到之前不要推车。
    - 通过：照常运行。
    - 没通过（INITIAL_POSE_MISMATCH）：运行自己停止。按给定位姿摆好车或改 --initial-pose，再重新启动。
    - 看不出朝向（AMBIGUOUS，走廊里沿走廊起步时就是这样）：照常运行，朝向只能靠上面的说明核对。
10. 目标：--goal 按路线给若干个地图坐标目标；推车的人按路线走，框架只计算不执行。
11. 记录空间：数据根剩余空间足够（每分钟约 20 MB）。结束时按一次 Ctrl-C 停止本命令：只停本命令启动的进程；收尾期间再按 Ctrl-C 不会打断，收尾完写出 result.json（supervisor.stopped = OPERATOR_INTERRUPT；车上模式这样结束是正常结束）。
12. 运行后核对 result.json：输入状态全程 RUNNING、转发计数和自检、AMCL 收敛（与车位一致）、闸门不可信时长与原因（含原地更新次数）、闸门限速时长（gate_slow_s，第 5 级第一次闭环前要逐段复核）、各级指令与推车运动的对比；面板的录包没有被标成 PARTIAL。
闭环（shadow --closed-loop，闸门输出经 A 的底盘桥交给面板会话的轮控所有者；现场步骤以 A 的步骤单为准）：
13. 用户当天同意闭环，命令前另设 WC_AUTO_DRIVE_APPROVAL=APPROVED_BY_USER_<YYYYMMDD>（和桥、轮控所有者同一个变量，日期必须是今天）。面板会话已开自动控制源（会话目录对应的 auto.sock 存在），命令给 --panel-root、--session-root、--session-id。
    固定值：0.10 m/s、0.30 rad/s；闸门限速 0.50 m / 0.05 m/s；使能超时 0.5 s；平滑器反馈开环、velocity_timeout 0.3 s；--monitor-base-shift off；起点检查 block（或更严的 strict）。
14. 命令起来后先让车站着跑约 30 s，看每秒一行的"闭环状态"（这一段代替单独的实车影子）：闸门 DISABLED（OPERATOR_ENABLE_OFF）、"影子判断"是它放行时会发的速度（例如 会放行 0.10 m/s）、定位可信、底盘不在 AUTO、急停否、目标 NAVIGATING。起点检查没通过、定位不可信，或影子判断的方向不对，不要点"开始自动"。
    使能和急停只能由桥发：/wc_nav/enable、/wc_nav/estop 的发布者不止桥一个时，闸门停在 DISABLED（ENABLE_SOURCE_AMBIGUOUS）。
    车前空地：状态行的"车前空地 N 点"是车前 5 m 内、车体以外、落在地图空地里的扫描点数（最近 10 帧的中位数）。超过门限（--front-check-max-points，默认 10）时写"不要开始自动"：先看车前有没有人或东西，有就挪开；车前其实是空的（雷达假点，2026-11-11 的 12.49 m 折叠），就停下来按 A 的雷达设置处理。--front-check block（闭环默认）下，这时点"开始自动"闸门也拦住（START_BLOCKED），车不动；挡着的人走开、判定变空，闸门放行，车起步。--front-check warn 只提示。
15. 点"开始自动"：底盘进 AUTO 后闸门放行当时的指令，车从 0 按轮控所有者的加速度上限起步。急停、软件急停、"停止自动"、空格或 WASD 都会停；急停以后要松开、再点一次"开始自动"。车动过以后，目标失败不自动重发。
16. 结束：按一次 Ctrl-C。收尾先停桥，轮控所有者看到断开就退出自动、写零；再停其他进程。result.json 的 closed_loop 一节记桥的计数、最后的底盘状态和车前空地的点数。
    桥自己退出时（例如轮控所有者 FAULT 结束了会话），运行停止，supervisor.stopped 是 BASE_BRIDGE_EXITED，supervisor.stop_reason 写一句原因（桥的退出原因、轮控所有者最后报的状态、会话轮控日志里的故障），终端也打印这一句。
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
    from ..perception import shadow_cli_options        # window C: the 3D obstacle input, off by default
    shadow_cli_options.add_live_arguments(inputs, inputs=True)
    shadow = sub.add_parser("shadow", help="live shadow run (bag now; vehicle only with approval)")
    _common(shadow)
    shadow.add_argument("--source", choices=["bag", "vehicle"], required=True)
    shadow.add_argument("--sensor-domain", type=int, help="vehicle: domain of the panel's sensor topics")
    shadow.add_argument("--map", dest="map_yaml", type=Path, required=True)
    shadow.add_argument("--initial-pose", nargs=3, type=finite, required=True, metavar=("X_M", "Y_M", "YAW_DEG"))
    shadow.add_argument("--initial-pose-std", nargs=2, type=finite, default=[0.25, math.degrees(0.15)],
                        metavar=("XY_M", "YAW_DEG"))
    from .start_check import MODES as START_CHECK_MODES
    shadow.add_argument("--start-check", choices=START_CHECK_MODES, default="block",
                        help="standing scans against the map at the start pose before Nav2 and the gate start: block "
                             "stops on INITIAL_POSE_MISMATCH and warns on AMBIGUOUS (default), strict stops on both, "
                             "warn only reports, off skips it")
    # Window B's smoother feedback (user 2026-11-10: gated by default; velocity timeout 0.3 s on the vehicle chain)
    shadow.add_argument("--smoother-feedback", choices=["open_loop", "gated"], default="gated",
                        help="gated: planning/velocity_feedback.py feeds the gate's output back to the smoother")
    shadow.add_argument("--smoother-timeout", type=finite, default=0.3, metavar="S",
                        help="the smoother's velocity_timeout (0.2-5 s)")
    shadow.add_argument("--goal", action="append", nargs=3, type=finite, metavar=("X_M", "Y_M", "YAW_DEG"))
    shadow.add_argument("--goals-along-cell", type=Path,
                        help="bag test: goals every --goal-spacing m along this frontend's path, placed by --map-alignment-json")
    shadow.add_argument("--map-alignment-json", type=Path)
    shadow.add_argument("--goal-spacing", type=finite, default=8.0)
    shadow.add_argument("--advance-radius", type=finite, default=1.0)
    shadow.add_argument("--amcl-params", default="framework")
    # Defaults are the safety gate's own: 0.25 m or 10 deg at the robot within 2 s, std <= 0.50 m (2026-11-07),
    # the position limit across the heading with a slow-down along it (lateral covariance mode, 2026-11-09).
    shadow.add_argument("--gate-correction-limits", nargs=2, type=finite, default=[0.25, 10.0],
                        metavar=("TRANSLATION_M", "YAW_DEG"))
    shadow.add_argument("--gate-covariance-limits", nargs=2, type=finite, default=[0.50, math.degrees(0.25)],
                        metavar=("XY_STD_M", "YAW_STD_DEG"))
    shadow.add_argument("--gate-correction-reference", choices=["odom_origin", "robot"], default="robot")
    from ..autonomy.safety_gate import add_covariance_mode_arguments
    add_covariance_mode_arguments(shadow)
    shadow.add_argument("--footprint", nargs="+", type=finite)
    shadow.add_argument("--footprint-source", default="EXAMPLE_NOT_MEASURED")
    shadow.add_argument("--max-duration", type=finite, default=900.0, help="wall seconds")
    # Vehicle closed loop (BOARD 2026-11-10 14:40; D.md 16:45)
    shadow.add_argument("--closed-loop", action="store_true",
                        help="drive: start window A's base bridge, which hands the gate's decisions to the wheel "
                             "serial owner of the running panel session; 0.10 m/s and 0.30 rad/s; needs "
                             + AUTO_DRIVE_APPROVAL_ENV + "=APPROVED_BY_USER_<today, YYYYMMDD>")
    # 总管 2026-11-11 item 2: scan points in the map's free space ahead before "start automatic" (live/front_check.py)
    from .front_check import MAX_POINTS as FRONT_MAX_POINTS, MODES as FRONT_MODES
    shadow.add_argument("--front-check", choices=FRONT_MODES,
                        help="scan points within 5 m ahead in the map's free space: block holds the gate at each start "
                             "while there are more than --front-check-max-points (default with --closed-loop), warn "
                             "only reports, off skips it (default without --closed-loop)")
    shadow.add_argument("--front-check-max-points", type=int, default=FRONT_MAX_POINTS, metavar="N",
                        help="most points still counted as clear (default %d, median of the last 10 scans)"
                             % FRONT_MAX_POINTS)
    shadow.add_argument("--panel-root", type=Path, help="closed loop: the panel's checkout (its session sockets)")
    shadow.add_argument("--session-root", type=Path, help="closed loop: directory of the running panel session")
    shadow.add_argument("--session-id", help="closed loop: identity of that session")
    shadow_cli_options.add_live_arguments(shadow)
    sub.add_parser("checklist", help="print the pre-flight checklist")
    graph = sub.add_parser("relay-graph", help="read-only query of the relay's entry node in the sensor domain")
    graph.add_argument("--sensor-domain", type=int, required=True, help="domain the relay reads from (panel: 83)")
    graph.add_argument("--source-prefix", default="", help="prefix of the relayed topics there (bag test: /relay_source)")
    graph.add_argument("--wait", type=_finite, default=3.0, help="seconds to wait for discovery (1..30)")
    graph.add_argument("--obstacles", action="store_true",
                       help="the run has the 3D obstacle input: the relay must read four topics (with the right lidar)")
    graph.add_argument("--project-root", type=Path, help=argparse.SUPPRESS)   # added by the wrapper; not used
    freeze = sub.add_parser("freeze-config", help="freeze the panel's current live configuration (files only)")
    freeze.add_argument("--project-root", type=Path, required=True)
    freeze.add_argument("--config-root", type=Path, required=True,
                        help="checkout whose configuration the panel uses (on the vehicle: the main checkout)")
    freeze.add_argument("--config", default="config/mapping_live.json", help="live mapping configuration below it")
    freeze.add_argument("--compare-cell", type=Path, help="recent single-left frontend cell to compare the mounts with")
    freeze.add_argument("--gyro-bias", type=Path, help="bias file to use when the live configuration names none")
    args = parser.parse_args(argv)
    if args.command == "relay-graph":
        if not 0 <= args.sensor_domain <= 232 or not 1.0 <= args.wait <= 30.0:
            parser.error("--sensor-domain must be a ROS domain id and --wait within 1..30 s")
        if args.source_prefix and not args.source_prefix.startswith("/"):
            parser.error("--source-prefix must start with /")
    if args.command in ("checklist", "freeze-config", "relay-graph"):
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
        from ..autonomy.safety_gate import GateConfig, covariance_mode_settings
        slow_limits, max_linear = args.gate_slow_limits, GateConfig.max_linear
        if args.closed_loop:
            if not (args.panel_root and args.session_root and args.session_id):
                parser.error("--closed-loop needs --panel-root, --session-root and --session-id of the running "
                             "panel session")
            if args.start_check not in CLOSED_LOOP_START_CHECKS:
                parser.error("--closed-loop needs --start-check " + " or ".join(CLOSED_LOOP_START_CHECKS))
            max_linear = CLOSED_LOOP_MAX_LINEAR
            if slow_limits is None and (args.gate_covariance_mode or GateConfig.covariance_mode) == "lateral":
                slow_limits = CLOSED_LOOP_SLOW_LIMITS
            # Fixed for the first closed loop (BOARD 2026-11-10 14:40): the smoother's feedback stays open loop with
            # velocity_timeout 0.3 s, and the collision monitor runs with base_shift off (user 2026-11-09 23:50).
            given = [str(a).split("=", 1)[0] for a in (sys.argv[1:] if argv is None else argv)]
            if any(a.startswith("--smoother-f") for a in given) and args.smoother_feedback != CLOSED_LOOP_SMOOTHER:
                parser.error("--closed-loop keeps the smoother feedback %s for the first closed loop"
                             % CLOSED_LOOP_SMOOTHER)
            if args.smoother_timeout != CLOSED_LOOP_SMOOTHER_TIMEOUT_S:
                parser.error("--closed-loop uses --smoother-timeout %g" % CLOSED_LOOP_SMOOTHER_TIMEOUT_S)
            # only --monitor-base-shift (or an abbreviation of it, --monitor-b...) given as on is refused; other
            # --monitor-* options such as window C's --monitor-slow-margin go with the closed loop
            if any(a.startswith("--monitor-b") for a in given) and args.monitor_base_shift != "off":
                parser.error("--closed-loop runs the collision monitor with --monitor-base-shift off")
            args.smoother_feedback, args.monitor_base_shift = CLOSED_LOOP_SMOOTHER, "off"
        elif args.panel_root or args.session_root or args.session_id:
            parser.error("--panel-root, --session-root and --session-id belong to --closed-loop")
        if args.front_check is None:
            args.front_check = "block" if args.closed_loop else "off"
        if not 0 <= args.front_check_max_points <= 161:
            parser.error("--front-check-max-points must be within 0..161 (the scan has 161 bins)")
        try:
            args.gate_covariance = covariance_mode_settings(args.gate_covariance_mode, slow_limits, max_linear)
        except ValueError as exc:
            parser.error(str(exc))
        from ..planning.stack import PlanningChoice
        try:
            PlanningChoice(smoother_feedback=args.smoother_feedback, smoother_timeout_s=args.smoother_timeout)
        except ValueError as exc:
            parser.error("smoother: " + str(exc))
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
    shadow_cli_options.check_live_arguments(parser, args)
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
    if args.command == "relay-graph":
        return relay_graph_command(args)
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
    from .supervisor import InterruptGuard
    # From here on the first Ctrl-C (SIGTERM, SIGHUP) stops the run and later ones cannot cut the shutdown.
    guard = InterruptGuard().install()
    try:
        return run_command(args, guard)
    except KeyboardInterrupt:      # before the run directory existed: nothing was started
        _report_line("stopped before the run started; no process was started")
        return 130
    finally:
        guard.restore()


def closed_loop_settings(args):
    """Checks before a closed-loop run, and its configuration block; nothing has been started yet.

    Today's user approval in the same variable as window A's bridge and wheel owner, the bridge's domain, and the
    panel session's automatic control socket (the owner creates it only with auto_controls, today's approval and
    valid evidence). The panel root and the session directory are passed to the bridge as given here, so both
    derive the same socket path.
    """
    import os
    import stat
    from wc_runtime.mapping_auto import auto_socket_path
    today = datetime.now().strftime("%Y%m%d")
    approval = os.environ.get(AUTO_DRIVE_APPROVAL_ENV, "")
    if approval != "APPROVED_BY_USER_" + today:
        print(CHECKLIST)
        raise SystemExit("the closed loop needs the user's approval of today: %s=APPROVED_BY_USER_%s"
                         % (AUTO_DRIVE_APPROVAL_ENV, today))
    if os.environ.get("ROS_DOMAIN_ID") != VEHICLE_SHADOW_DOMAIN:
        raise SystemExit("the closed loop runs in domain %s, like window A's base bridge; set WC_AUTONOMY_ROS_DOMAIN=%s"
                         % (VEHICLE_SHADOW_DOMAIN, VEHICLE_SHADOW_DOMAIN))
    panel = args.panel_root.expanduser().absolute()
    session = args.session_root.expanduser().absolute()
    if not session.is_dir():
        raise SystemExit("no panel session directory " + str(session))
    path = auto_socket_path(panel, session, args.session_id)
    try:
        is_socket = stat.S_ISSOCK(path.stat().st_mode)
    except OSError:
        is_socket = False
    if not is_socket:
        raise SystemExit("no automatic control socket %s: panel session %s does not offer automatic control (window "
                         "A's wheel owner creates it only with auto_controls, today's approval and valid evidence)"
                         % (path, args.session_id))
    return {"panel_root": str(panel), "session_root": str(session), "session_id": args.session_id,
            "auto_socket": str(path), "bridge_node": BRIDGE_NODE, "approval": approval,
            "max_linear": CLOSED_LOOP_MAX_LINEAR, "max_angular": CLOSED_LOOP_MAX_ANGULAR,
            "enable_timeout_s": CLOSED_LOOP_ENABLE_TIMEOUT_S}


def relay_graph_command(args):
    """Print the relay entry node's graph in the sensor domain; creates no endpoint there (see entry_graph)."""
    import os
    from ..autonomy.ros_util import SIMULATION_DOMAINS
    from .supervisor import VEHICLE_APPROVAL_ENV, vehicle_approved
    if str(args.sensor_domain) not in SIMULATION_DOMAINS and not vehicle_approved():
        raise SystemExit("joining domain %d needs the user present and %s=APPROVED_BY_USER_<date>, as for the relay"
                         % (args.sensor_domain, VEHICLE_APPROVAL_ENV))
    if os.environ.get("ROS_LOCALHOST_ONLY", "1") != "1":
        raise SystemExit("ROS_LOCALHOST_ONLY must be 1: the panel's domain and the relay are localhost only")
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    from .relay_node import entry_graph
    report = entry_graph(args.sensor_domain, args.source_prefix, args.wait, obstacles=args.obstacles)
    print(json.dumps(report, ensure_ascii=False, indent=1), flush=True)
    return 0 if report["status"] == "PASS" else 1


def move_run_files(work, output):
    """Copy everything a run wrote under ``work`` into its run directory, then remove ``work``.

    A file that cannot be copied is listed and ``work`` is kept, so nothing is lost.
    """
    import shutil
    copied, errors = 0, []
    for path in sorted(Path(work).rglob("*")):
        target = Path(output) / path.relative_to(work)
        try:
            if path.is_dir():
                target.mkdir(parents=True, exist_ok=True)
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(path, target)
                copied += 1
        except OSError as exc:
            errors.append("%s: %s" % (path, exc))
    if not errors:
        shutil.rmtree(work, ignore_errors=True)
    return {"work_dir": str(work), "copied_files": copied, "errors": errors[:20], "work_dir_removed": not errors}


def _report_line(text):
    try:
        print(text, flush=True)
    except (OSError, ValueError):   # the terminal may be gone (SIGHUP); result.json is already written
        pass


def run_command(args, guard):
    """inputs / shadow: one supervised run and its result.json, also after an operator stop."""
    from wc_runtime.storage_policy import StoragePolicy
    from ..autonomy import nav2_config
    from ..autonomy.bringup import domain_lock
    from ..perception import shadow_cli_options
    from .bag_check import check_run
    from .config import load_live_config
    from .supervisor import INTERRUPTED, LiveSupervisor, VEHICLE_APPROVAL_ENV, vehicle_approved
    if args.command == "shadow" and args.source == "vehicle":
        import os
        if not vehicle_approved():
            print(CHECKLIST)
            raise SystemExit("vehicle shadow runs need the user present and %s=APPROVED_BY_USER_<date>"
                             % VEHICLE_APPROVAL_ENV)
        if os.environ.get("ROS_DOMAIN_ID") != VEHICLE_SHADOW_DOMAIN:
            raise SystemExit("the vehicle shadow runs in domain %s (user decision 2026-11-07); "
                             "set WC_AUTONOMY_ROS_DOMAIN=%s" % (VEHICLE_SHADOW_DOMAIN, VEHICLE_SHADOW_DOMAIN))
    closed_loop = closed_loop_settings(args) if args.command == "shadow" and args.closed_loop else None
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
    # The 3D obstacle input (window C) uses the offset resolved above for /scan, never its own.
    obstacles = shadow_cli_options.live_entries(args, args.cell, live, config["lidar_time_offset_ms"],
                                                config["lidar_time_offset_source"])
    if obstacles is not None:
        config["obstacles"] = obstacles
    if args.relay_test:
        config["bag_remap_prefix"] = "/relay_source"
        config["relay"] = {"source_domain": None, "source_prefix": "/relay_source", "output_prefix": OUTPUT_PREFIX,
                           "wait_for_panel_consumers": False, "obstacles": bool(config.get("obstacles"))}
        config["source_prefix"] = OUTPUT_PREFIX
    if args.command == "shadow":
        from .amcl_cross import CANDIDATES
        map_yaml = args.map_yaml.expanduser().resolve(strict=True)
        speeds = ({"max_linear": CLOSED_LOOP_MAX_LINEAR, "max_angular": CLOSED_LOOP_MAX_ANGULAR}
                  if closed_loop else {})      # Nav2's controller, smoother and behaviours and the gate follow these
        profile = nav2_config.NavProfile(footprint=args.footprint or (), **speeds,
                                         **shadow_cli_options.live_profile_fields(args))
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
                                   "covariance_yaw_std_deg": args.gate_covariance_limits[1],
                                   **args.gate_covariance},
                      initial_pose=[args.initial_pose[0], args.initial_pose[1], math.radians(args.initial_pose[2])],
                      initial_pose_std=list(args.initial_pose_std), goals=goals, advance_radius_m=args.advance_radius,
                      max_duration_s=args.max_duration)
        config["start_check"] = args.start_check   # live/start_check.py, before Nav2 and the gate start
        from . import front_check
        config["front_check"] = {"mode": args.front_check, "max_points": args.front_check_max_points,
                                 "range_m": front_check.RANGE_M, "wall_tolerance_m": front_check.WALL_TOLERANCE_M,
                                 "frames": front_check.FRAMES}
        config["smoother_feedback"], config["smoother_timeout_s"] = args.smoother_feedback, args.smoother_timeout
        if closed_loop:
            config["closed_loop"] = closed_loop
            config.update(open_loop=False, hardware_control=True, commands_sent=True)
        if args.source == "vehicle":
            # D5 (user 2026-11-10): with the 3D obstacle input the relay also reads the right lidar, four topics
            config["relay"] = {"source_domain": args.sensor_domain, "source_prefix": "", "output_prefix": OUTPUT_PREFIX,
                               "wait_for_panel_consumers": True, "obstacles": bool(config.get("obstacles"))}
            config["source_prefix"] = OUTPUT_PREFIX
            config["niceness"] = VEHICLE_SHADOW_NICENESS
    # While the run goes, its processes write logs and records under /dev/shm: a large file copied to the data disk
    # (NTFS through FUSE) blocks every other write there for seconds (2026-11-10). They are copied in afterwards.
    work = WORK_ROOT / ("wc_live_" + run_id)
    with domain_lock(args.project_root):
        output.mkdir(parents=True, exist_ok=False)
        work.mkdir(parents=True, exist_ok=False)
        try:
            try:
                if config.get("relay") is not None and config["relay"]["source_domain"] is None:
                    import os
                    config["relay"]["source_domain"] = int(os.environ["ROS_DOMAIN_ID"])
                (output / "live_config.json").write_text(json.dumps(live, ensure_ascii=False, indent=1), encoding="utf-8")
                (output / "run_config.json").write_text(json.dumps(config, ensure_ascii=False, indent=2), encoding="utf-8")
                report = LiveSupervisor(args.project_root, work, config, guard=guard).run()
            except KeyboardInterrupt:  # stopped before the supervisor ran: nothing was started
                guard.begin_shutdown()
                report = {"stages": [], "process_exit_codes": {}, "stopped": INTERRUPTED,
                          "signals": list(guard.received)}
        finally:
            guard.begin_shutdown()  # the copy, the summary and result.json go on even if Ctrl-C is pressed again
            run_files = move_run_files(work, output)
    report = json.loads(json.dumps(report, default=float).replace(str(work), str(output)))
    report["run_files"] = run_files
    storage.check()
    interrupted = report.get("stopped") == INTERRUPTED
    result = {"schema_version": 1, "kind": args.command, "status": "INCOMPLETE", "config": config,
              "live_provenance": live["provenance"], "supervisor": report,
              "hardware_control": closed_loop is not None, "commands_sent": closed_loop is not None,
              "physical_navigation_validated": False}
    if args.command == "inputs":
        try:
            result["comparison"] = check_run(output, args.compare_cell or args.cell, config["lidar_time_offset_ms"])
            scans = result["comparison"]["scans"]
            odometry = result["comparison"]["odometry"]
            # a bag check stopped by the operator covers only part of the recording
            complete = ("error" not in report and not interrupted and scans["matched_frames"] > 0
                        and odometry.get("status") == "COMPARED")
            if config.get("obstacles"):             # only then the obstacle check must pass as well
                from ..perception.live_check import check_run as check_obstacles
                result["obstacles"] = check_obstacles(output, config, args.compare_cell or args.cell,
                                                      args.compare_right_cell)
                complete = complete and result["obstacles"]["status"] == "PASS"
            result["status"] = "COMPLETE" if complete else "INCOMPLETE"
        except Exception as exc:
            result["summary_error"] = type(exc).__name__ + ": " + str(exc)
    else:
        from .shadow_summary import summarize_shadow
        try:
            result.update(summarize_shadow(config, output))
            # the vehicle run ends with Ctrl-C by design; a bag run stopped early did not see the whole recording;
            # a run the start check stopped never started Nav2 and the gate
            complete = ("error" not in report and result.get("commands")
                        and report.get("stopped") in (None, INTERRUPTED, "DURATION_LIMIT")
                        and not (interrupted and config["source"] == "bag"))
            result["status"] = "COMPLETE" if complete else "INCOMPLETE"
        except Exception as exc:
            result["summary_error"] = type(exc).__name__ + ": " + str(exc)
    with (output / "result.json").open("x", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False, default=float)
    brief = {"status": result["status"], "output": str(output), "error": report.get("error"),
             "stopped": report.get("stopped"), "summary_error": result.get("summary_error")}
    if "comparison" in result:
        brief["scans"] = {k: result["comparison"]["scans"][k] for k in ("matched_frames", "identical_frames",
                                                                         "differing_bins_total", "unmatched_frames")}
        brief["odometry"] = result["comparison"]["odometry"]
    elif args.command == "shadow":
        brief.update({k: result.get(k) for k in ("outcomes_while_moving", "gate_untrusted_s", "localization")})
    if "obstacles" in result:
        brief["obstacles"] = {"status": result["obstacles"].get("status"), "reasons": result["obstacles"].get("reasons")}
    _report_line(json.dumps(brief, ensure_ascii=True, default=float))
    return 0 if result["status"] == "COMPLETE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
