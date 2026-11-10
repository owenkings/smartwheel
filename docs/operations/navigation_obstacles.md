# 障碍感知：三维障碍输入、离线检查与仿真行人

用左右雷达的三维点云代替 /scan 做局部代价地图和碰撞监控的障碍输入，在冻结录包上开环检验；在闭环仿真里加移动行人。只读冻结前端和地图，不打开设备、不发布速度指令；影子运行和仿真在独立的 ROS 域里，本窗口用 99。设计与证据见[障碍感知与避障](../architecture/obstacle_avoidance.md)。数据只用 ZL1107 走廊系列。

## 影子运行

在[影子运行](navigation_autonomy.md#在真实录包上做-nav2-影子运行)的命令上加障碍输入选项：

```bash
R=/media/nvidia/tiany-USB/wheelchair
WC_AUTONOMY_ROS_DOMAIN=99 nice -n 10 bash scripts/navigation_nav2_shadow --mode follow \
  --source-cell $R/data/analysis/20261107_190329_ZL1107_map_single_left_gyrobias_wheel/robot_localization_hz5_filter_on \
  --map $R/reports/navigation_replay_mapping/20261107T111004Z_3f0ae14a/odom_grid/map_nav.yaml \
  --footprint 0.81 0.2425 0.2 0.35 -0.255 0.35 -0.255 -0.35 0.2 -0.35 0.81 -0.2425 \
  --footprint-source MEASURED_20261107 \
  --obstacle-input cloud \
  --obstacle-right-cell $R/data/analysis/20261107_203741_ZL1107_map_single_right_gyrobias_wheel/robot_localization_hz5_filter_on
```

| 选项 | 含义 |
|---|---|
| `--obstacle-input scan\|cloud\|scan_and_cloud` | 局部代价地图和碰撞监控用什么。`scan`（默认）与以前完全相同；`cloud` 用三维障碍点代替 /scan；`scan_and_cloud` 两者都用。 |
| `--obstacle-scan raw\|filtered` | 代价地图里 ObstacleLayer 读的扫描（全局代价地图总有一个，`scan` 和 `scan_and_cloud` 时局部也有）：`raw`（默认）是 /scan；`filtered` 是 `/obstacles/left/scan`，左雷达过滤后的障碍点按 /scan 同样的 161 个方位格取最近距离（frame laser，8 m 内）。和 `--obstacle-input scan` 一起用时两个代价地图都还是二维 ObstacleLayer，只是换成过滤后的扫描，碰撞监控仍用 /scan，只准备左雷达点云（右前端只作见证）。和 B 的 `--global-costmap static` 一起用时，`cloud` 输入下没有代价地图读它，直接报错。 |
| `--obstacle-monitor scan\|cloud\|scan_and_cloud` | 碰撞监控单独用什么（默认和 `--obstacle-input` 相同）。例如 `--obstacle-input scan --obstacle-scan filtered --obstacle-monitor cloud`：两个代价地图是二维过滤扫描，监控用两台雷达的当前帧点云，没有体素记忆。 |
| `--monitor-base-shift on\|off` | 碰撞监控的 base_shift_correction（默认 on，和以前一样）。off 只用静态外参，不按里程计把点移到当前时刻。录包影子的 TF 总跟着时钟，这里两种差不多；实时链路里 on 会让碰撞监控每个周期等里程计（见[设计说明](../architecture/obstacle_avoidance.md#第一次实车影子碰撞监控11-09)）。 |
| `--monitor-slow-margin M` | 碰撞监控减速框伸出车头多远（默认 0.65 m，不随速度变）。0.10 m/s 时用 0.15，见[设计说明](../architecture/obstacle_avoidance.md#010-ms-第一次闭环的碰撞监控11-10)。 |
| `--lidar-time-offset-ms MS` | 雷达时间戳比轮速/IMU 晚 MS：每帧（/scan、两台雷达的障碍点、清除射线、过滤扫描）配 stamp − MS 处的录包位姿，回放时把这些消息的时间戳提前 MS 发出；语义和实时链路、AMCL 回放的同名选项一致。默认 0（按录包原样）；用 75 ms 重建的地图上要给 75。 |
| `--obstacle-right-cell` | 同一录包的单右冻结前端。给了才有右雷达障碍点，也才有两台雷达互相见证。 |
| `--obstacle-sides left right` | 发布哪几台的障碍点（默认：左，给了右前端时左右都发）。 |
| `--obstacle-no-consensus` | 不用另一台雷达否决悬空小团，只按尺寸判别。 |
| `--obstacle-right-lever frozen\|symmetric\|X,Y,Z` | 右雷达杆臂：冻结的现场外参（默认）、左雷达的镜像位置，或显式值。 |
| `--obstacle-filter '{"max_range_m": 3.5}'` | 覆盖过滤参数（`ObstacleFilter` 的字段，见源码）。例如 `{"persist_min_age_s": 0.6}` 打开前后帧持续性检查：通过见证和尺寸规则的悬空小团，0.6–2 s 前同一台雷达在同一世界位置没有候选点的就去掉（默认关）。 |

验证包和建图包是两段录包，要在建图包地图上跑验证包时，加定位模式 `--localization fixed|amcl --map-alignment-json <对齐结果>`（见[实车影子链路](navigation_live_shadow.md)）。障碍点坐标系由回放进程里一个不受重映射影响的节点发布，三种定位模式下都可用。

过滤在 prepare_recording 里一次做完（按 `--window` 只处理窗口内的帧），障碍点和清除射线写进运行目录的 `recording.npz`（`obstacle_<side>_*` 数组），回放时按各自采集时间发布到 `/obstacles/<side>/points` 和 `/obstacles/<side>/clearing`。

结果在 `reports/navigation_nav2_shadow/<时间>_shadow_<模式>_<编号>/`，比默认运行多出：

| 位置 | 内容 |
|---|---|
| `run_config.json` 的 `obstacles` | 过滤参数、两台雷达杆臂及来源、体素与清除射线统计、各阶段去掉的点数。 |
| `result.json` 的 `obstacle_cloud_audit` | 每台雷达按采集位姿统计落进停车区和减速框的帧，停车帧的点高度和距离。 |
| `result.json` 的 `supervisor.parameter_readback` | 读回的局部代价地图插件和碰撞监控观测源，确认确实接上了点云。 |
| `replay` 的 `obstacle_clouds_published` | 每台雷达发出的帧数。 |

## 离线检查

```bash
nice -n 10 bash scripts/navigation_obstacle_eval \
  --left-cell <单左前端> --right-cell <单右前端> \
  --footprint 0.81 0.2425 0.2 0.35 -0.255 0.35 -0.255 -0.35 0.2 -0.35 0.81 -0.2425 \
  --footprint-source MEASURED_20261107 --label ZL1107_map_cloud
```

不启动 ROS 节点。对每帧过滤后的障碍点，检查它们是否落在车接下来 `--horizon`（默认 4 s）秒的录包路径扫过的外形里（外形内缩 3 cm）；静止障碍不可能在那里，所以这类点是伪点或走开的物体。同时对当前 /scan 做同样统计。车静止的帧没有未来路径可查，汇总里单独列在 `static` 下（建图包开头 0–33 s 和结尾 262.7–377.7 s 都是静止段）。`--lidar-time-offset-ms` 同上。输出在 `reports/navigation_obstacles/<时间>_<标签>_<编号>/`：`result.json`（汇总、参数、来源）、`frames.jsonl`（逐帧计数）、`swept_points.npz`（落进扫掠外形的点）。

## 障碍航迹（离线）

```bash
nice -n 10 bash scripts/navigation_obstacle_tracks \
  --left-cell <单左前端> --right-cell <单右前端> \
  --map $R/reports/navigation_replay_mapping/20261108T051511Z_fe74c4f4/odom_grid/map_nav.yaml \
  --footprint 0.81 0.2425 0.2 0.35 -0.255 0.35 -0.255 -0.35 0.2 -0.35 0.81 -0.2425 \
  --footprint-source MEASURED_20261107 --filter '{"persist_min_age_s": 0.6}' \
  --lidar-time-offset-ms 75 --static-before 33.0 --static-after 262.65 --label ZL1107_map_tracks
```

不启动 ROS 节点。两台雷达按影子运行的方式过滤，每个左雷达帧配右雷达最近的一帧（0.15 s 内），各自按采集位姿投到 odom，做一次 `tracks.Tracker` 更新；地图去掉墙和地图里的家具。
- 验证包用 `--map-alignment-json align_wheel.json` 把录包放进地图。
- `--tracker '{...}'` 可以改 TrackerConfig 的字段。
- 输出在 `reports/navigation_obstacles/<时间>_<标签>_<编号>/`：
  - `result.json`：各类航迹的条数和秒数，以及所有 moving 航迹的时间、位置、速度和高度，按开头静止 / 行驶 / 结尾静止分段。
  - `tracks.jsonl`：每次更新一条 `/obstacles/tracks` 消息，stamp 是测量时刻（录包时间减偏移）。
- 没人走动的录包上，moving 航迹都是误报或当时在场的人，要对照录包看。

## 实时链路的三维障碍输入

`navigation_live_shadow inputs|shadow`（D 的实时链路，见[实车影子操作说明](navigation_live_shadow.md)）加上障碍选项，就多起一个 obstacles 进程（`perception/obstacles_node.py`，和转发一样在命令的 nice 上再加 10，车上模式下是 19）。默认不开，不给这些选项时命令、回放的话题、Nav2 参数和 `result.json` 都和以前一样。

```bash
WC_AUTONOMY_ROS_DOMAIN=99 bash scripts/navigation_live_shadow inputs --project-root "$PWD" \
  --cell <验证包单左前端> --hardware-setup <轮参数覆盖文件> --gyro-bias <零偏文件> \
  --bag $R/data/experiments/ZL21107corridor_nav_2026_1107_155149/bag --compare-right-cell <验证包单右前端> \
  --footprint 0.81 0.2425 0.2 0.35 -0.255 0.35 -0.255 -0.35 0.2 -0.35 0.81 -0.2425 --footprint-source MEASURED_20261107 \
  --obstacle-input scan --obstacle-scan filtered --obstacle-monitor cloud --obstacle-filter '{"persist_min_age_s": 0.6}'
```

| 选项 | 作用 |
|---|---|
| `--obstacle-input`、`--obstacle-monitor`、`--obstacle-scan` | 和影子运行同名选项一样，决定发哪些话题、Nav2 怎么接。上面是“组合”：代价地图用过滤扫描，碰撞监控用两台雷达的三维点云。 |
| `--obstacle-sides` | 发布哪台雷达的障碍点（默认两台；只用过滤扫描时只发左雷达，右雷达只作见证）。 |
| `--obstacle-filter JSON` | ObstacleFilter 的参数覆盖，例如持续性检查 `persist_min_age_s`。 |
| `--obstacle-rate-hz HZ` | 每秒处理几对帧（默认 5，最多 10）；两台雷达都是 10 Hz。 |
| `--obstacle-no-witness` | 不读另一台雷达判悬空团。 |
| `--compare-right-cell`（只 `inputs`） | 右雷达的冻结前端，也核对右雷达的点云。 |
| `--footprint`、`--footprint-source`（只 `inputs`；`shadow` 用它自己的外形） | 过滤用的车体外形，不给时按 0.55 m 假设圆。 |
| `--monitor-base-shift on\|off`（只 `shadow`，和障碍输入无关） | 碰撞监控的 base_shift_correction。on（默认）时每个周期都要等里程计发出“现在”的 odom→base_link，最多 0.3 s，等不到就这一周期不看传感器；off 只用静态外参。用户 2026-11-10 同意实车影子用 off，默认值不改。 |
| `--monitor-slow-margin M`（只 `shadow`，和障碍输入无关） | 碰撞监控减速框伸出车头多远，默认 0.65 m，不随速度变。0.10 m/s 的第一次闭环用 0.15：11-07、11-09 的左雷达悬空薄带就进不了框。读回会核对 PolygonSlow.points。 |

- 处理：每对帧（两台雷达同一周期，相差几毫秒）先过 D 的 `scan_adapter.reference_points`（生产主机滤波、转到 mapping_reference，和冻结前端逐点相同），每台雷达用自己的外参和设备编号。再等 /odom 覆盖 stamp − 偏移的时刻（最多 0.3 s），然后按影子运行的规则过滤，另一台雷达作见证（`consensus.BoxWitness`，结论和 `Witness` 相同）。等不到位姿的帧照样发，只是没有见证和持续性检查，状态里计数。
- 输出：`/obstacles/<left|right>/points`（坐标系 `obstacles_lidar_<side>`，静态变换在雷达原点、轴和 base_link 一致），VoxelLayer 时还有 `/clearing`，过滤扫描时有 `/obstacles/left/scan`。时间戳和 /scan 一样是雷达时间戳减偏移，偏移就是命令为 /scan 解析出的那一个。状态在 `/wc_live/inputs_status/obstacles`（RUNNING/STALE/FAILED、计数、耗时）。每处理一帧，运行目录的 `obstacles.jsonl` 写一行：位姿、配对、输入和输出的 sha256、各级计数、耗时。
- `inputs` 的 `result.json` 多一节 `obstacles`（`perception/live_check.py`），这一节 PASS 才算 COMPLETE：
  - 输入：处理过的帧里，冻结前端有的那些（约三分之一），点云 sha256 要相同。
  - 输出：按 `obstacles.jsonl` 记下的配对和位姿，用录包里的同一批帧离线重算，点、射线和过滤扫描都要相同。
  - 频率：每台雷达不低于 0.8 × `--obstacle-rate-hz`；等不到位姿的周期不超过 5%；进程没有 FAILED。
- `shadow` 的 `result.json` 多一节 `obstacles`：频率、配对、见证、等待、耗时和各级计数，不重算。
- 车上（`--source vehicle`）：用户 2026-11-10 同意开，转发要多转右雷达（D5，窗口 D）。D5 进来之前命令仍拒绝开障碍输入；运行时照常要用户当次同意。

## 仿真里的移动行人

闭环仿真（[自主导航操作说明](navigation_autonomy.md)）加 `--actors 场景.json`：

```bash
WC_AUTONOMY_ROS_DOMAIN=99 nice -n 10 bash scripts/navigation_autonomy_sim navigate \
  --world-map $R/reports/navigation_replay_mapping/20261107T111004Z_3f0ae14a/odom_grid/map_nav.yaml \
  --start -1.5 -0.2 3.14159 --goal -10.0 -0.2 3.14159 --max-duration 240 \
  --footprint 0.81 0.2425 0.2 0.35 -0.255 0.35 -0.255 -0.35 0.2 -0.35 0.81 -0.2425 \
  --footprint-source MEASURED_20261107 --actors oncoming.json
```

场景文件（世界坐标，`--world-map` 时就是地图坐标）：

```json
{"schema_version": 1, "name": "oncoming",
 "actors": [{"id": "walker", "waypoints": [[-10.5, -0.35], [0.5, -0.35]], "speed_mps": 1.0,
             "radius_m": 0.25, "start_after_s": 2.0, "pause_s": 0.0, "repeat": "once", "yield_gap_m": 0.0}]}
```

- 行人是圆盘，按路点匀速走；时钟从车第一次收到非零指令开始，再加 `start_after_s`，之前站在第一个路点。`repeat` 为 once（停在终点）、loop、pingpong；`pause_s` 是每到一个路点停的时间。行人不看墙，路点要放在空地上。
- `yield_gap_m` 为 0（默认，最坏情况）时行人不管车，会走进车里；大于 0 时，不走会让它离车体更近、且近于这个距离的那一步。
- 行人每个物理步之前并入仿真世界，所以仿真扫描、扫掠碰撞否决和接触间隙都包括行人。车开进行人照常算这次运行的碰撞（FAIL）；行人走进停着的车只记在 `actors` 一节。
- 运行目录多出 `actors.jsonl`（每 0.1 s 仿真时间一行位置、间隙和状态，另有第一次指令、接触开始和结束、车被行人挡住的事件），`result.json` 多出 `actors`：每个行人到车体的最小间隙和时刻、0.3/0.5/1.0/1.5 m 内的时间、接触次数（车在动时的另计）、行人 1 m 内的车速、让行时间，以及行人 1.5 m 内和以外的停车时间。不给 `--actors` 时配置和结果与以前相同。

## 边界

结论只针对冻结录包和空车外形。过滤参数（高度窗、远处低点门限 2 m、悬空小团规则）是在建图包上定的，验证包和以后的走廊录包要再看。坐人时乘坐者伸出外形的部分会被当成障碍，需要坐人录包确认。仿真行人只是圆盘，没有腿、身高和真实的避让行为。返回[操作导航](README.md)。
