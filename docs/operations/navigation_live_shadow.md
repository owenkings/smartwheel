# 实车影子链路：已存地图定位、录包检验与上车准备

本页是验证阶梯第 4 级的操作说明：在另一段录包的地图上检验 AMCL，在影子运行里换成真实定位，以及实时链路和实车影子启动器（后两项另见下文）。设计和证据见[已存地图定位与实车影子链路](../architecture/live_shadow.md)。这里的命令都不打开设备、不连接轮控、不发布 `/cmd_vel`。

## 跨录包 AMCL 检查

```bash
R=/media/nvidia/tiany-USB/wheelchair
bash scripts/navigation_amcl_replay cross \
  --source-cell $R/data/analysis/20261107_190332_ZL1107_nav_single_left_gyrobias_wheel/robot_localization_hz5_filter_on \
  --map $R/reports/navigation_replay_mapping/20261108T051511Z_fe74c4f4/odom_grid/map.yaml \
  --alignment-json $R/dev_archive/20261104_104621_navigation_autonomy_framework_20261104/validation/commands_20261107_field/checks/align_wheel.json \
  --lidar-time-offset-ms 75
```

正式走廊地图 fe74c4f4 是按 75 ms 建的，在它上面定位要给 `--lidar-time-offset-ms 75`（BOARD 接口约定 5，结论注明）；旧图 3f0ae14a 按 0 ms 建，只作对比。

- 在隔离域 95 里逐帧回放被定位录包的冻结扫描和里程计；map_server、AMCL 和本进程以外不能有节点。每次 AMCL 运行约 10 s。
- 对齐用 `--alignment X Y YAW_DEG` 或 `--alignment-json`（读 `best_alignment`），表示被定位录包的里程计坐标系在地图里的位姿。
- `--profile` 选参数集，可重复：`framework`（`nav2_config.localization_parameters`，按 AMCL 自身运动门限更新）、`framework_alpha_0_05`、`framework_sharp_laser`、`candidate`（前两项合并）、`replay_tool`（原回放工具参数，每帧强制更新）。默认 `framework` 和 `replay_tool`。
- `--perturbation FORWARD_M LEFT_M YAW_DEG` 在起点车轴坐标系里加扰动，可重复，限 2 m、45°；不给时用 21 个默认扰动。只有 `--perturb-profile`（默认 `framework`）的参数集跑扰动，其余只跑对齐起点。`--initial-std XY_M YAW_DEG` 是起点标准差，默认 0.25 m、8.6°（与仿真导航相同）。
- `--lidar-time-offset-ms MS`（默认 0，录包原样）：雷达时间戳比轮速/IMU 晚 MS 毫秒时，每帧扫描配 stamp − MS 处的冻结里程计（相邻帧位姿间线性插值），扫描时间戳不变；限 ±250。用于比较时间偏移，结论见设计说明“雷达时间偏移”一节。
- 回放是确定的：同样的输入跑两次，结果逐字节相同。要看 AMCL 的离散，用几个很小的起点扰动（例如 ±5 cm、±1°）当作不同的实现，而不是重复运行。
- 中段帧按录包原样的里程计分成转弯（|角速度| ≥ 0.3 rad/s）、直行（< 0.05 rad/s 且 ≥ 0.2 m/s）、静止和其他；每个运行的 `by_motion_middle` 和固定对齐的 `fit_by_motion_middle` 按这四类给出端点拟合、2 s 内修正变化、标准差和闸门不可信帧数。分类不随时间偏移变，不同偏移比的是同一批帧。

结果在数据根 `reports/navigation_amcl_cross/<时间>_<编号>/`：

| 文件 | 内容 |
|---|---|
| `summary.json` | 分段（起点门内、中段、终点门内）、固定对齐的端点拟合、每个参数集和扰动的拟合、协方差（含沿车头/横向）、相对固定对齐的偏离、2 s 内修正变化（`map→odom` 本身和在车体处）、闸门协方差和修正条件的判定、沿车头歧义、收敛判定。 |
| `cases/<参数集>/<扰动>.csv` | 逐帧：AMCL 车轴位姿、端点拟合与支撑、协方差、修正变化、闸门判定、歧义标记，以及固定对齐的位姿和拟合。 |
| `cases/<参数集>/<扰动>/` | 那次 AMCL 运行的参数文件、日志和 `result.json`（含每帧 `map→odom`）。 |
| `preview.png` | 地图上的固定对齐和 AMCL 轨迹、扰动起点（按收敛着色）、拟合、标准差和与未扰动运行的距离。 |

判读：这是两段录包之间的自洽性，不是真值；两段的里程计来自同一个前端，对齐只用了验证包前 40 s。端点拟合在地图空白处没有意义，门内帧要单独看。A 重建地图后，用新地图和新的对齐重跑这条命令，对比 `summary.json` 里的 `fixed_alignment` 和各参数集的拟合与收敛计数。

## 影子运行换成真实定位

```bash
WC_AUTONOMY_ROS_DOMAIN=100 bash scripts/navigation_nav2_shadow \
  --source-cell <验证包前端> --map <建图包的 map_nav.yaml> \
  --footprint 0.81 0.2425 0.2 0.35 -0.255 0.35 -0.255 -0.35 0.2 -0.35 0.81 -0.2425 --footprint-source MEASURED_20261107 \
  --mode navigate --localization amcl --map-alignment-json <对齐结果>
```

- `--localization identity`（默认）：地图在录包自己的里程计坐标系里，`map→odom` 恒等，行为与以前相同。
- `--localization fixed`：地图来自另一段录包，静态 `map→odom` 取对齐值，不做修正；是跨录包时与 AMCL 对比的基线。
- `--localization amcl`：map_server + AMCL。起点是对齐后的录包起点，`--initial-pose-offset FORWARD_M LEFT_M YAW_DEG` 可加操作员误差，`--initial-pose-std` 默认 0.25 m、8.6°；`--amcl-params` 选 `framework`（默认）或一组候选改动。闸门同时要求 AMCL 协方差；`--gate-correction-limits TRANSLATION_M YAW_DEG`、`--gate-covariance-limits XY_STD_M YAW_STD_DEG` 和 `--gate-correction-reference robot|odom_origin` 改闸门的定位条件，默认与闸门本身相同（2026-11-07 起：车体位置 2 s 内 0.25 m 或航向 10°，协方差 0.50 m、14.3°，按车体位置量；2026-11-09 起协方差只看横向，`--gate-covariance-mode`、`--gate-slow-limits` 见下面“闸门协方差判据”一节）。
- fixed 和 amcl 需要 `--map-alignment X Y YAW_DEG` 或 `--map-alignment-json`；记录路径放进地图后至少一半落在空闲格上才运行。

输出与原影子运行相同，另有 `localization_samples.csv`（10 Hz 的 `map→odom` 和每个 AMCL 位姿与协方差）、`recording_map.npz`（放进地图的录包，给 carrot 和统计用）和 `result.json` 的 `localization` 一节：AMCL 与对齐的差、端点拟合（AMCL 与对齐）、闸门实际因定位不可信停车的秒数（按原因）、离线换阈值重算的不可信比例（两种修正量法、几档平移和航向、几档协方差、端点支撑），以及 2 s 内修正变化的分布。

判读：follow 模式的路线按对齐放进地图，AMCL 修正越大，车体估计离路线越远，控制器的原地转向和前方碰撞会因此增加；这是定位修正对跟线的影响，不全是控制器的问题。端点支撑条件目前只在离线重算里，闸门还没有这一项。

## 实时链路的录包检验

```bash
R=/media/nvidia/tiany-USB/wheelchair
CELL=$R/data/analysis/20261107_190332_ZL1107_nav_single_left_gyrobias_wheel/robot_localization_hz5_filter_on
WC_AUTONOMY_ROS_DOMAIN=100 bash scripts/navigation_live_shadow inputs \
  --cell $CELL \
  --hardware-setup $R/data/calibration/wheel_odometry/ZL1107_ab_hardware_setup_override.json \
  --gyro-bias $R/data/calibration/gyro_bias/ZL1107capture_2026_1107_154018_gyro_bias.json \
  --bag $R/data/experiments/ZL21107corridor_nav_2026_1107_155149/bag
```

- `ros2 bag play --clock` 只回放左雷达 host_filtered 帧、H30 帧和轮速记录三个话题，按录包时间送进扫描和里程计两个进程；不启动驱动、面板或任何会打开设备的节点。整个运行在 `WC_AUTONOMY_ROS_DOMAIN` 的隔离域里，占用本检出该域的锁。
- `--cell` 给外参、主机滤波、运动先验和设备编号；`--hardware-setup` 的 `wheel_odometry` 和 `--gyro-bias` 替换轮参数和零偏。ZL1107 系列录包冻结的是旧轮参数，必须给覆盖文件，否则和冻结前端对不上。
- `--scan-rate-hz`（默认 10，0 为每帧都发）、`--bag-rate`（0.1–2）、`--relay-test`（录包放在 `/relay_source` 前缀下，经单向转发进程进来，检验转发）。
- `--lidar-time-offset-ms MS`（`inputs` 和 `shadow` 都有；0 为录包原样）：发布的 `/scan` 时间戳比雷达帧早 MS 毫秒，AMCL、代价地图和碰撞监控就在扫描实际采集的时刻取里程计。不给时取批准的值：冻结配置目录里的 `lidar_time_offset.json`，否则本检出的 `config/calibration/lidar_time_offset.json`（2026-11-08 用户批准 75 ms）；都没有就报错，要求显式给。录包检验按运行记录的值在 stamp − MS 处找对应的冻结帧；`run_config.json` 记下值和来源（文件路径和 sha256），输入状态里也有。
- `--compare-cell`（只 `inputs`）：比对用的冻结前端，默认就是 `--cell`；`--cell` 给的是 `freeze-config` 冻结的当前配置时，用它指定同一录包的前端。
- 经转发时（`--relay-test` 或车上），扫描和里程计进程订阅改名后的 `/wc_live/relayed/...`。
- 输出在 `reports/navigation_live_shadow/<时间>_inputs_<编号>/`：`scans.csv`（含 161 格距离）、`odom.csv`、`events.jsonl`、各进程日志和 `result.json`。`result.json` 的 `comparison` 给出与冻结前端逐帧的扫描比较（匹配帧、完全相同的帧、不同的格数）和里程计比较（第一帧对齐后的位置与航向差）。
- 三维障碍输入（窗口 C，默认不开）：`inputs` 和 `shadow` 都可以加 `--obstacle-input`、`--obstacle-scan filtered`、`--obstacle-monitor` 等选项。
  - 这时录包多放右雷达的 host_filtered 帧，多起一个 obstacles 进程。
  - `inputs` 的 `result.json` 多一节 `obstacles`，这一节通过才算 COMPLETE；`--compare-right-cell` 给出右雷达的冻结前端。
  - `shadow` 另有 `--monitor-base-shift on|off`（碰撞监控是否按里程计移动点，默认 on）和 `--monitor-slow-margin M`（碰撞监控减速框伸出车头多远，默认 0.65 m）。
  - 用法见[障碍感知操作说明](navigation_obstacles.md#实时链路的三维障碍输入)。

## 实车影子运行

```bash
WC_AUTONOMY_ROS_DOMAIN=100 bash scripts/navigation_live_shadow shadow --source bag \
  --cell $CELL --hardware-setup <同上> --gyro-bias <同上> \
  --bag $R/data/experiments/ZL21107corridor_nav_2026_1107_155149/bag \
  --map $R/reports/navigation_replay_mapping/20261108T051511Z_fe74c4f4/odom_grid/map_nav.yaml \
  --initial-pose -0.331 1.902 -85.0 \
  --goals-along-cell $CELL --map-alignment-json <对齐结果> \
  --footprint 0.81 0.2425 0.2 0.35 -0.255 0.35 -0.255 -0.35 0.2 -0.35 0.81 -0.2425 --footprint-source MEASURED_20261107
```

一条命令按序启动：记录器、扫描与里程计进程、map_server + AMCL（框架参数或 `--amcl-params`），由监督进程按操作员方式发初始位姿（`--initial-pose`，标准差 `--initial-pose-std` 默认 0.25 m、8.6°）并做起点检查（见下），再起 Nav2 各服务器（参数读回核对）、平滑器反馈节点和安全闸门，最后启动目标序列（`--goal X Y YAW_DEG` 可重复；录包检验可用 `--goals-along-cell` 沿冻结前端的路线每 `--goal-spacing` 米取一个目标）。推车的人走近当前目标 `--advance-radius`（默认 1 m）内就换下一个。闸门的定位条件用 `--gate-correction-limits`、`--gate-covariance-limits`、`--gate-correction-reference`、`--gate-covariance-mode`、`--gate-slow-limits`，默认与闸门本身相同。闸门输出 `/wc_nav/cmd_vel_gated` 只有平滑器反馈节点订阅；监督进程发送使能心跳，只是为了记录“闸门会放行什么”。

- 平滑器反馈（2026-11-10 起默认，用户决定）：`--smoother-feedback gated` 时，Nav2 读回之后起窗口 B 的反馈节点 `wc_velocity_feedback`，等 `/wc_nav/smoother_feedback` 有发布者再起闸门。它把闸门输出转成平滑器的反馈：闸门停车后平滑器从静止重新起步。
  - `--smoother-feedback open_loop` 退回开环；`--smoother-timeout S` 是平滑器的速度超时（默认 0.3 s，0.2–5 s）。
  - 读回核对包括平滑器的 `feedback`、`odom_topic`、`velocity_timeout`；逐周期记录在 `velocity_feedback.csv`，实际用的值在 `run_config.json`。
- `--source bag`：上面的录包检验，录包时间，域 100。
- `--source vehicle --sensor-domain <面板域>`：在车上，墙钟时间，影子框架在域 100（`WC_AUTONOMY_ROS_DOMAIN=100`，用户 2026-11-07 同意）；三个传感器话题（开三维障碍输入时加右雷达，共四个）经单向转发从面板域进影子域，转发的订阅范围和只读保证见[设计说明](../architecture/live_shadow.md#实车影子模式的单向转发第一次上车前由总管和用户评审)。第一次上车前总管和用户先评审这一部分；之后只有在用户在场并同意时运行，命令前设置 `WC_LIVE_SHADOW_VEHICLE_APPROVAL=APPROVED_BY_USER_<日期>`，否则拒绝启动。

输出在 `reports/navigation_live_shadow/<时间>_shadow_<编号>/`：`commands.csv`（四级指令）、`odom.csv`（人推的实际运动）、`localization.csv`（AMCL 位姿与协方差、`map→odom`）、`plans.jsonl`、`events.jsonl`（闸门、输入、目标的状态变化）和 `result.json`（推车行驶时各级指令的结局占比、闸门各状态和定位不可信的秒数、AMCL 统计；录包检验还给出 AMCL 车位与对齐的冻结路线的差）。

运行中的文件（2026-11-10 起）：
- `inputs` 和 `shadow` 运行时，各进程的日志、ROS 日志和记录器的 CSV 先写在 `/dev/shm/wc_live_<运行编号>/`。所有进程停了以后拷进运行目录、删掉临时目录，再汇总、写 `result.json`。运行目录里的文件和以前一样，`supervisor.run_files` 记拷了多少个、有没有出错。
- 原因：数据盘是 NTFS，经 FUSE 挂载。往它拷大文件并 fsync 的几秒里，其他进程往这块盘写一行也要等。
  - 实测拷 700 MB 时，一次写等了 4.15 s。
  - 2026-11-09 第一次实车影子，面板失败后把约 660 MB 的录包拷到数据盘。正在写日志的 collision_monitor 因此 4 s 没心跳，Nav2 被整体重置。
  - 改了以后，同样拷 700 MB，记录器的行间隔最长 0.13 s（改之前 4.2 s）。
- 监督进程被 SIGKILL 或断电时，文件只留在 `/dev/shm`，重启前要手动拷出来。

停止、优先级和面板域查询（2026-11-09 实车影子之后）：
- 停止：`inputs` 和 `shadow` 收到第一次 Ctrl-C（或 SIGTERM、SIGHUP）就开始收尾。
  - 逐个停本命令启动的进程，一个停失败记进 `supervisor.stop_errors`，接着停下一个。
  - 收尾期间再收到信号只打印"收尾中"，不再打断。要强停只能用 SIGKILL，那样会留下进程。
  - 收尾后照常汇总并写 `result.json`：`supervisor.stopped` 为 `OPERATOR_INTERRUPT`，`supervisor.signals` 是收到的信号。汇总出错时记 `summary_error`，文件照样写。
  - 车上模式就是这样结束的，没有错误就是 `COMPLETE`；录包运行被打断记 `INCOMPLETE`。
- 优先级：车上模式整个命令以 nice 10 运行，命令自己会把 nice 调到不低于 10，转发进程再降 10（到 19），面板的录制优先。实际值记在 `supervisor.niceness`。
  - 依据：2026-11-09 第二次这样跑，面板录包 COMPLETE。实时里程计落后中位 0.13 s、最大 0.43 s，超过 0.3 s 的 9.4%，主要在开头 Nav2 启动时。
  - 第 5 级闭环前再评估，那时里程计落后对闸门 TF 时效更要紧。
- 面板域里转发入口节点的查询：转发 RUNNING 后，另开终端，用同样的环境变量运行下面的命令。

  ```bash
  WC_AUTONOMY_ROS_DOMAIN=100 WC_LIVE_SHADOW_VEHICLE_APPROVAL=APPROVED_BY_USER_<日期> \
    bash scripts/navigation_live_shadow relay-graph --sensor-domain 83
  ```

  - 它只在面板域建一个没有任何端点的只读节点（和转发入口同一个类），读发现缓存，打印后退出。
  - `status` 为 `PASS` 的条件：入口节点没有发布者、服务、客户端，正好三个订阅，都是 best effort、volatile；查询节点自己也没有端点。
  - 开三维障碍输入的运行要加 `--obstacles`：这时入口节点应正好四个订阅，多的是右雷达 host_filtered。
  - 不要在面板域用 `ros2 node info`、`ros2 topic list` 这类 ros2 命令：它们会在面板域起 ros2 daemon，那是一个有发布者和服务、命令结束后还留着的节点。
- 初始位姿的朝向：车头朝向必须和 `--initial-pose` 的 YAW_DEG 一致。
  - 给用户的检查单和提示一律按看板地图说方向：右 = 地图 +x，左 = 地图 −x。不用东西南北：用户 2026-11-10 说明，现实里地图 −x 是东。
  - ZL1107 地图的起点 (−0.323, −0.229, 0°) 是车头朝看板地图的右边。
  - 2026-11-09 两次实车影子车头都朝左（地图 −x），给的却是 0°（地图 +x）。走廊两边对称，AMCL 和闸门在前 4–5 m 都没发现。分析见 `reports/navigation_live_shadow/20261109T152258Z_vehicle_shadow_start_heading/`。
- 起点检查（`--start-check block|strict|warn|off`，默认 block，用户 2026-11-09 同意）：
  - 发完初始位姿、AMCL 有位姿之后，Nav2 和闸门启动之前做。监督进程取 10 帧静止扫描，算 0.15 m 以内贴图的端点比例。比两个值：给定位姿附近（±0.3 m、±10°）最好的，和掉头 180° 后附近最好的。
  - 三种结果：

    | 结果 | 条件 | 怎么处理 |
    |---|---|---|
    | 通过 PASS | 给定朝向的 ≥ 0.6，且比掉头的高 0.2 以上 | 照常运行 |
    | 没通过 INITIAL_POSE_MISMATCH | 给定朝向的 < 0.6，或掉头的高 0.2 以上 | Nav2 和闸门不启动，运行停止，`supervisor.stopped` 记这个原因 |
    | 看不出朝向 AMBIGUOUS | 两者差不到 0.2 | block 下只警告，照常运行 |

  - 收扫描时车动了（位移超过 0.05 m 或转过 2°），记 START_CHECK_NOT_STANDING，按没通过处理。
  - 其他模式：`strict` 时 AMBIGUOUS 也停；`warn` 只记录；`off` 不做。
  - 结果在 `result.json` 的 `supervisor.start_check`，也打印一行"起点检查…"给操作员。
  - 门限的依据（2026-11-10，静止扫描）：
    - 验证包起点在门口、车头朝走廊对面：给定朝向 85%，掉头 0%，分得很清楚。
    - ZL1107 建图起点车头沿走廊：对的 85–86%，反了 180° 的也有 79%，分不开，这里只能报 AMBIGUOUS。
    - 只看位姿本身也不行：对的起点放偏 0.2 m、6° 就掉到 48%。
    - 数据见设计说明。

## 闸门协方差判据：默认只看横向（2026-11-09）

用户 2026-11-08 决定先做成显式选项试，2026-11-09 决定改成默认。仿真导航（`navigation_autonomy_sim navigate`，默认的 `--localization amcl`）、录包影子运行（`navigation_nav2_shadow --localization amcl`）和实时影子（`navigation_live_shadow shadow`）不给选项时都用 lateral。这三处都有这两个选项：

```bash
... --gate-covariance-mode xy                # 回到 2026-11-09 前的判据
... --gate-slow-limits 0.50 0.15             # lateral 的限速条件（这是默认值）
```

- `lateral`（默认）：只有车头横向的标准差超过 0.50 m 或航向标准差超过 0.25 rad 才停车，修正突变条件不变。放行时，误差椭圆长轴的标准差超过 STD_M 就把线速度限到 SPEED_M_S，降到 STD_M − 0.05 m 以下才恢复。线速度和角速度按同一比例缩小（转弯半径不变），原地转向不限，限速值按 0.6 / 0.3 m/s² 过渡。`--gate-covariance-limits` 的位置标准差在 lateral 下是车头横向的上限。
- `xy`（显式选择）：地图坐标轴上较大的位置标准差超过 0.50 m，或航向标准差超过 0.25 rad，闸门停车；不能和 `--gate-slow-limits` 一起给。
- `--gate-slow-limits STD_M SPEED_M_S`：默认 0.50 m、0.15 m/s；STD_M 在 0.05–2 m，SPEED_M_S 不超过最大线速度。
- 仿真的 `--localization truth` 不看协方差，这两个选项不能和它一起给。
- 依据和验证结果见[设计说明](../architecture/live_shadow.md#闸门协方差只看横向默认2026-11-09)。
- 记录：
  - AMCL 运行的配置里写明实际用的模式和限速值：仿真的 `run_config.json`（`gate_options`）和 `gate_config.json`，影子运行的 `gate_limits`。
  - `/wc_nav/gate_status` 有一节 `covariance`：模式；lateral 时还有横向、沿车头、长轴和航向标准差，是否限速，当前限速值。
  - 限速时状态仍是 `ACTIVE`，`reasons` 里有 `LOCALIZATION_SLOW`；横向超限的停车原因是 `COVARIANCE_ACROSS_HEADING_TOO_LARGE`。
  - 实时影子的 `result.json` 有 `gate_slow_s`：闸门限速放行的秒数，和 `gate_state_s` 同一时段，xy 下是 0。
- `navigation_amcl_replay cross` 不需要选项：离线评估逐帧算三个闸门变体，`gate_until_20261107`、`gate_xy`、`gate_lateral`。2026-11-09 前的运行里，后两个叫 `gate_default`、`gate_lateral_trial`。`summary.json` 的 `default_gate_variant` 指明哪个是闸门现在的默认；逐帧 CSV 有 `gate_lateral_slow` 列，闸门判定里有 `slow_frames` 和 `full_speed_frames`。

## 上车前冻结当前配置

```bash
WC_AUTONOMY_ROS_DOMAIN=100 bash scripts/navigation_live_shadow freeze-config --config-root /home/nvidia/wheelchair \
  --compare-cell <同一台车最近的单左前端> --gyro-bias <零偏文件>
```

- 在子进程里用 `wc_runtime.mapping_controller.configuration()` 解析面板当前的实时配置（`--config` 默认 `config/mapping_live.json`），只读文件，不连设备、不起 ROS。
- 输出 `reports/navigation_live_shadow/<时间>_config_<编号>/`：`runtime_config.json`、`prior_config.json`、`hardware_setup.json`（有零偏配置时还有零偏和依据文件）、`lidar_time_offset.json`（那个检出的批准时间偏移）、`frozen_configuration.json`（每个文件的 sha256 和核对结果）。
- 看核对结果：`hardware_setup_from_local_override` 必须是 true；轮参数是设备当前值；`confirmed_gyro_bias_configured` 为 false 时，影子命令必须用 `--gyro-bias` 显式给零偏；`against_reference_cell` 里外参、杆臂、主机滤波和设备编号应与最近的前端一致。
- 影子命令用冻结目录作 `--cell`，其中的 `hardware_setup.json` 作 `--hardware-setup`（命令行打印的 `use` 一行）。

## 实车闭环（--closed-loop，第一次闭环）

闸门的决定经窗口 A 的底盘桥交给面板会话的轮控所有者，车自己走。现场步骤以 A 的步骤单为准；设计见[设计说明](../architecture/live_shadow.md#实车闭环模式第一次闭环2026-11-10)。

```bash
WC_AUTONOMY_ROS_DOMAIN=100 WC_LIVE_SHADOW_VEHICLE_APPROVAL=APPROVED_BY_USER_<日期> \
  WC_AUTO_DRIVE_APPROVAL=APPROVED_BY_USER_<今天，YYYYMMDD> \
  bash scripts/navigation_live_shadow shadow --source vehicle --sensor-domain 83 --closed-loop \
  --panel-root /home/nvidia/wheelchair --session-root <面板会话目录> --session-id <会话编号> \
  --cell <冻结目录> --hardware-setup <冻结目录>/hardware_setup.json --gyro-bias <零偏文件> \
  --map <map_nav.yaml> --initial-pose X Y YAW_DEG --goal X Y YAW_DEG \
  --footprint 0.81 0.2425 0.2 0.35 -0.255 0.35 -0.255 -0.35 0.2 -0.35 0.81 -0.2425 --footprint-source MEASURED_20261107 \
  --monitor-base-shift off
```

- 面板会话的 `--session-root`、`--session-id`：用窗口 A 的 `python3 -s -m wc_runtime.auto_operator --project-root /home/nvidia/wheelchair session` 查，它打印正在录制、开了自动控制源的会话，格式就是 `--session-root … --session-id …`。
- 底盘桥由这个命令自己起，不要另起 `wc_motion.auto_bridge`：两个桥会让闸门停在 `DISABLED(ENABLE_SOURCE_AMBIGUOUS)`。
- 其他选项照影子的写法，例如窗口 C 的 `--monitor-slow-margin M`（碰撞监控减速框伸出车头多远）可以一起给。`--monitor-base-shift` 只能是 off，不给时也按 off。
- 启动前命令自己查：`WC_AUTO_DRIVE_APPROVAL` 的日期是今天；域 100；面板会话的 `auto.sock` 存在。会话没开自动控制源时没有这个 socket，命令直接拒绝。
- 固定值（总管 2026-11-10 14:40）：0.10 m/s、0.30 rad/s；闸门限速 0.50 m / 0.05 m/s；使能超时 0.5 s；平滑器反馈开环、`velocity_timeout` 0.3 s；`--monitor-base-shift off`；`--start-check` 默认 block（也可以给更严的 strict）。给了别的值，命令拒绝启动。
- 起来以后先让车站着跑约 30 s，看每秒一行的“闭环状态”。这一段代替单独的实车影子：目标已经发出，Nav2 照常规划、出指令，闸门因为没有使能输出零。
  - 没点“开始自动”时应是：闸门 `DISABLED(OPERATOR_ENABLE_OFF)`，底盘不在 AUTO，急停否，目标 `NAVIGATING`。
  - “影子判断”是使能、底盘都就绪时闸门会放行什么，例如“会放行 0.10 m/s 0.00 rad/s”，或者“LOCALIZATION_UNTRUSTED(…)”。
  - 起点检查没通过、定位不可信，或影子判断的方向不对，不要点“开始自动”。
- 点“开始自动”后底盘进 AUTO，闸门放行当时的指令，车从 0 按轮控所有者的加速度上限起步。
  - 急停、软件急停、“停止自动”、空格、WASD 都会停；急停以后要松开、再点一次“开始自动”。
  - 站着时 Nav2 的目标失败会照影子的做法重发；车动过以后（闸门 ACTIVE 过），目标失败不再重发。
- 使能和急停只能由桥发：这两个话题有别的发布者时，闸门停在 `DISABLED(ENABLE_SOURCE_AMBIGUOUS)`。
- 车前空地检查（总管 2026-11-11 第 2 条；`--front-check block|warn|off`，闭环默认 block；`--front-check-max-points N`，默认 10）：
  - 状态行里的“车前空地 N 点”：车前 5 m 内、车体以外、落在地图空地里（离最近的占据格超过 0.30 m）的扫描点数，取最近 10 帧的中位数。
  - 超过门限时写“车前空地 N 点 > 门限 10，最近 x m：不要开始自动”。先看车前有没有人或东西，有就挪开。车前其实是空的，就是雷达假点：停下来，按 A 的雷达设置处理，或者等 C 的过滤。
  - block 下这时点“开始自动”，轮控所有者照常进 AUTO，闸门在 `START_BLOCKED(FRONT_NOT_CLEAR)` 输出零，车不动。挡着的人走开、判定变空，闸门放行，车起步。只在每次开始（使能上升）到第一次放行之间查；车动起来以后，前方的东西归 Nav2 和碰撞监控。
  - 站着时“影子判断”也会显示 `START_BLOCKED(FRONT_NOT_CLEAR)`。warn 只提示，不拦。
  - 2026-11-11 20:27 那次的录制上量是 34–40 点；11-07 验证录制的门口起点（朝走廊对面墙）是 8 点。
- 按一次 Ctrl-C 结束：先停桥，轮控所有者退出自动、写零，再停其他进程。`result.json` 的 `closed_loop` 一节记最后一条底盘状态（含桥的计数）、闸门各状态的计数和 `front_check`（开始前站着时的点数）。
- 桥自己退出（例如轮控所有者 FAULT 结束了会话）时，运行停止：`supervisor.stopped` 是 `BASE_BRIDGE_EXITED`，`supervisor.stop_reason` 写一句原因，终端也打印这一句，例如“底盘桥退出（退出码 1：wheel owner closed the automatic control socket）；轮控所有者最后报 STOPPING（FEEDBACK_OVERSPEED）；会话轮控日志：NOT_STATIONARY_AFTER_STOP”。细节在 `closed_loop.bridge_exit`。

上车前检查单：`bash scripts/navigation_live_shadow checklist`。内容与命令里的相同，要点：
- 用户同意并记录。
- 先在面板里开始录制并看到设备就绪，再启动影子命令；转发会等面板自己的订阅者到齐才订阅。开障碍输入时面板要用两台雷达。
- 整个命令以 nice 10 运行；转发 RUNNING 后用 `relay-graph` 查入口节点（开障碍输入时加 `--obstacles`）。
- 物理急停可用；在影子域确认订阅闸门输出的只有平滑器反馈节点，没有进程发布 `/cmd_vel`。
- 传感器频率；开障碍输入时两台雷达的 host_filtered 都约 10 Hz。
- 用 `freeze-config` 冻结并核对当前配置，零偏二选一。
- 起点在已建图区域；初始位姿带不小于 0.25 m、8.6° 的标准差，车头朝向和给定航向一致。
- 目标；记录空间。
- 按一次 Ctrl-C 结束；运行后核对结果，面板录包不是 PARTIAL。
- 闭环（检查单第 13–16 条）：另要今天的 `WC_AUTO_DRIVE_APPROVAL` 和开了自动控制源的面板会话；先站着看状态行，再点“开始自动”。

返回[操作导航](README.md)。
