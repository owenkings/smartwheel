# 实车影子链路：已存地图定位、录包检验与上车准备

本页是验证阶梯第 4 级的操作说明：在另一段录包的地图上检验 AMCL，在影子运行里换成真实定位，以及实时链路和实车影子启动器（后两项另见下文）。设计和证据见[已存地图定位与实车影子链路](../architecture/live_shadow.md)。这里的命令都不打开设备、不连接轮控、不发布 `/cmd_vel`。

## 跨录包 AMCL 检查

```bash
R=/media/nvidia/tiany-USB/wheelchair
bash scripts/navigation_amcl_replay cross \
  --source-cell $R/data/analysis/20261107_190332_ZL1107_nav_single_left_gyrobias_wheel/robot_localization_hz5_filter_on \
  --map $R/reports/navigation_replay_mapping/20261107T111004Z_3f0ae14a/odom_grid/map.yaml \
  --alignment-json $R/dev_archive/20261104_104621_navigation_autonomy_framework_20261104/validation/commands_20261107_field/checks/align_wheel.json
```

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
- `--localization amcl`：map_server + AMCL。起点是对齐后的录包起点，`--initial-pose-offset FORWARD_M LEFT_M YAW_DEG` 可加操作员误差，`--initial-pose-std` 默认 0.25 m、8.6°；`--amcl-params` 选 `framework`（默认）或一组候选改动。闸门同时要求 AMCL 协方差；`--gate-correction-limits TRANSLATION_M YAW_DEG`、`--gate-covariance-limits XY_STD_M YAW_STD_DEG` 和 `--gate-correction-reference robot|odom_origin` 改闸门的定位条件，默认与闸门本身相同（2026-11-07 起：车体位置 2 s 内 0.25 m 或航向 10°，协方差 0.50 m、14.3°，按车体位置量）。
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
- `--lidar-time-offset-ms MS`（`inputs` 和 `shadow` 都有，默认 75，用户 2026-11-08 决定；0 为录包原样）：发布的 `/scan` 时间戳比雷达帧早 MS 毫秒，AMCL、代价地图和碰撞监控就在扫描实际采集的时刻取里程计。录包检验按运行记录的值在 stamp − MS 处找对应的冻结帧；`run_config.json` 和输入状态里记录所用的值。
- `--compare-cell`（只 `inputs`）：比对用的冻结前端，默认就是 `--cell`；`--cell` 给的是 `freeze-config` 冻结的当前配置时，用它指定同一录包的前端。
- 经转发时（`--relay-test` 或车上），扫描和里程计进程订阅改名后的 `/wc_live/relayed/...`。
- 输出在 `reports/navigation_live_shadow/<时间>_inputs_<编号>/`：`scans.csv`（含 161 格距离）、`odom.csv`、`events.jsonl`、各进程日志和 `result.json`。`result.json` 的 `comparison` 给出与冻结前端逐帧的扫描比较（匹配帧、完全相同的帧、不同的格数）和里程计比较（第一帧对齐后的位置与航向差）。

## 实车影子运行

```bash
WC_AUTONOMY_ROS_DOMAIN=100 bash scripts/navigation_live_shadow shadow --source bag \
  --cell $CELL --hardware-setup <同上> --gyro-bias <同上> \
  --bag $R/data/experiments/ZL21107corridor_nav_2026_1107_155149/bag \
  --map $R/reports/navigation_replay_mapping/20261107T111004Z_3f0ae14a/odom_grid/map_nav.yaml \
  --initial-pose -0.331 1.902 -85.0 \
  --goals-along-cell $CELL --map-alignment-json <对齐结果> \
  --footprint 0.81 0.2425 0.2 0.35 -0.255 0.35 -0.255 -0.35 0.2 -0.35 0.81 -0.2425 --footprint-source MEASURED_20261107
```

一条命令按序启动：记录器、扫描与里程计进程、map_server + AMCL（框架参数或 `--amcl-params`）、Nav2 各服务器（参数读回核对）、安全闸门，然后由监督进程按操作员方式发初始位姿（`--initial-pose`，标准差 `--initial-pose-std` 默认 0.25 m、8.6°），再启动目标序列（`--goal X Y YAW_DEG` 可重复；录包检验可用 `--goals-along-cell` 沿冻结前端的路线每 `--goal-spacing` 米取一个目标）。推车的人走近当前目标 `--advance-radius`（默认 1 m）内就换下一个。闸门的定位条件用 `--gate-correction-limits`、`--gate-covariance-limits`、`--gate-correction-reference`，默认与闸门本身相同。闸门输出 `/wc_nav/cmd_vel_gated` 没有任何订阅者；监督进程发送使能心跳，只是为了记录“闸门会放行什么”。

- `--source bag`：上面的录包检验，录包时间，域 100。
- `--source vehicle --sensor-domain <面板域>`：在车上，墙钟时间，影子框架在域 100（`WC_AUTONOMY_ROS_DOMAIN=100`，用户 2026-11-07 同意）；三个传感器话题经单向转发从面板域进影子域，转发的订阅范围和只读保证见[设计说明](../architecture/live_shadow.md#实车影子模式的单向转发第一次上车前由总管和用户评审)。第一次上车前总管和用户先评审这一部分；之后只有在用户在场并同意时运行，命令前设置 `WC_LIVE_SHADOW_VEHICLE_APPROVAL=APPROVED_BY_USER_<日期>`，否则拒绝启动。

输出在 `reports/navigation_live_shadow/<时间>_shadow_<编号>/`：`commands.csv`（四级指令）、`odom.csv`（人推的实际运动）、`localization.csv`（AMCL 位姿与协方差、`map→odom`）、`plans.jsonl`、`events.jsonl`（闸门、输入、目标的状态变化）和 `result.json`（推车行驶时各级指令的结局占比、闸门各状态和定位不可信的秒数、AMCL 统计；录包检验还给出 AMCL 车位与对齐的冻结路线的差）。

## 上车前冻结当前配置

```bash
WC_AUTONOMY_ROS_DOMAIN=100 bash scripts/navigation_live_shadow freeze-config --config-root /home/nvidia/wheelchair \
  --compare-cell <同一台车最近的单左前端> --gyro-bias <零偏文件>
```

- 在子进程里用 `wc_runtime.mapping_controller.configuration()` 解析面板当前的实时配置（`--config` 默认 `config/mapping_live.json`），只读文件，不连设备、不起 ROS。
- 输出 `reports/navigation_live_shadow/<时间>_config_<编号>/`：`runtime_config.json`、`prior_config.json`、`hardware_setup.json`（有零偏配置时还有零偏和依据文件）、`frozen_configuration.json`（每个文件的 sha256 和核对结果）。
- 看核对结果：`hardware_setup_from_local_override` 必须是 true；轮参数是设备当前值；`confirmed_gyro_bias_configured` 为 false 时，影子命令必须用 `--gyro-bias` 显式给零偏；`against_reference_cell` 里外参、杆臂、主机滤波和设备编号应与最近的前端一致。
- 影子命令用冻结目录作 `--cell`，其中的 `hardware_setup.json` 作 `--hardware-setup`（命令行打印的 `use` 一行）。

上车前检查单：`bash scripts/navigation_live_shadow checklist`（内容与命令里的相同，要点：用户同意并记录；先在面板里开始录制并看到设备就绪，再启动影子命令，转发会等面板自己的订阅者到齐才订阅；物理急停、没有任何进程订阅闸门输出或发布 `/cmd_vel`；传感器频率；用 `freeze-config` 冻结并核对当前配置，零偏二选一；起点在已建图区域；初始位姿带不小于 0.25 m、8.6° 的标准差；目标；记录空间；运行后核对结果，面板录包不是 PARTIAL）。

返回[操作导航](README.md)。
