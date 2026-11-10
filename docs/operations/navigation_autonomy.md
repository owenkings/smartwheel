# 自主导航仿真：探索建图与目标导航

在 Orin 上用仿真轮椅闭环运行整套自主导航：未知环境中自主探索并建图，保存地图后用 AMCL 定位并依次到达目标。同一套 Nav2 配置还能在冻结录包上开环影子运行，用真实扫描检验规划、控制和停车链路。全部进程运行在本机回环的 ROS 域 96（或 97），并有独占锁；不打开设备，不连接轮控，也不需要外参。框架结构见[自主导航总体框架](../architecture/autonomy_framework.md)。

## 运行前

Orin 已安装 Humble 版 Nav2 1.1.20（含 collision_monitor 与 RPP）和 slam_toolbox 2.6，无需额外安装。确认没有其他人在用域 96；有其他节点时程序会拒绝启动。需要同时跑第二个仿真时，在命令前加 `WC_AUTONOMY_ROS_DOMAIN=97`（可选 97–101），各次运行各自持有独占锁、互不干扰。独占锁放在各自检出目录的 `.phase1_runtime/locks` 下，多个检出（git worktree）并行开发时要给每个检出分配不同的域，启动前的外来节点检查只是兜底。

## 自主探索建图

```bash
cd /home/nvidia/wheelchair
bash scripts/navigation_autonomy_sim explore --world two_rooms
bash scripts/navigation_autonomy_sim explore --world office_floor --max-duration 1500
bash scripts/navigation_autonomy_sim explore \
  --world-map /已保存的走廊地图/map.yaml --real-time-factor 1.0
```

- `--world` 选合成楼层：`two_rooms`（10 m × 7 m，两个房间加走廊）或 `office_floor`（20 m × 12 m，走廊两侧五个房间）。门宽 1.8 m、走廊 2.4 m，按假设包络 0.55 m 加 0.10 m 余量设计。布局带版本号和哈希，记录在运行配置里；布局改动后旧运行不能再按新布局重算。`office_floor` 第 2 版把走廊中间的柜子移到墙边——第 1 版中它两侧各只剩 1.0 m，地图 +x 那半层对假设包络不可达。
- `--world-map` 把已保存的栅格地图当作真值环境，未知格按实体处理；起点自动选在最大连通区域。
- `--footprint`、`--footprint-source` 与影子运行相同：给了外形，仿真碰撞（含原地转向扫过）、代价地图和碰撞监控都用这个凸多边形，不给时都用圆。`--planner`、`--controller` 等规划控制选项见[规划与控制](navigation_planning.md)。
- 流程：先原地环视一周，再反复选最佳可达前沿、导航、失败或到达后仍未消除的前沿列入黑名单；没有前沿后回到起点并保存 `map.pgm/map.yaml`。
- 常用参数：`--max-duration`（仿真秒）、`--real-time-factor`（0.1–4）、`--obstacle X Y R`（地图中没有的圆形障碍，可重复）、`--no-initial-spin`、`--no-return-home`、`--slam-loop-closure`（默认关闭，见下）。
- `--odometry-error` 选择里程计误差模型：`wheel_imu`（默认，轮速距离加陀螺航向，偏置取实测静止陀螺 z 均值约 0.0004 rad/s，2% 距离比例误差）、`wheel_only`（只用轮速航向的压力测试，偏置 5 倍、转向比例误差 1.5%）、`ideal`（无误差，用于单独检查规划控制）。
- `--mapper` 选择探索建图方式：`odom_grid`（默认，只用轮速+IMU 里程计，对应 ADR 0001 的建图位姿依据）、`odom_icp`（实验：里程计预测加受限的扫描到地图修正，仿真中更好、已录走廊上更差）、`slam_toolbox`（相关性扫描匹配，可加 `--slam-loop-closure`）。仿真与真实录包对比见[框架说明](../architecture/autonomy_framework.md#建图方式对比仿真)。
- 三种方式默认都不做回环：仿真中窄视场和相似房间曾产生约 2 m 的错误回环，比缓慢漂移更危险。与现有离线 RTAB-Map 重建关闭额外回环的做法一致。
- 探索结束状态：`COMPLETE` 表示没有可达前沿；连续 5 个目标失败时停止选点、回起点并标为 `INCOMPLETE_REPEATED_GOAL_FAILURES`；回起点失败时状态后缀 `_RETURN_HOME_FAILED`。三种情况都会保存地图，剩余前沿列在结果中。回起点只要求回到起点位置，车头朝向取到达方向，不再原地转回出发时的朝向。

## 已保存地图上的目标导航

```bash
bash scripts/navigation_autonomy_sim navigate \
  --from-exploration /数据根/reports/navigation_autonomy/<探索任务目录>
bash scripts/navigation_autonomy_sim navigate \
  --from-exploration <探索任务目录> --goal 3.0 2.5 0 --goal 0.5 0.5 3.14 \
  --obstacle 4.0 1.4 0.3 --initial-pose-error 0.2 -0.1 0.08
```

- 使用探索任务保存的地图、同一仿真世界以及两者之间的坐标关系。监督进程像操作员一样发送近似初始位姿：默认在真实起点上加 0.15/−0.10 m、0.06 rad 误差（`--initial-pose-error`），并附 0.25 m、0.15 rad 标准差（`--initial-pose-std`）。不使用 AMCL 自带的零协方差初始化，避免滤波器对猜测位姿“过分确定”。
- 未给 `--goal` 时自动在可达已知空闲区选取相距最远的 3 个目标（`--auto-goals` 可改）。目标坐标为地图坐标系。
- `--obstacle` 放入保存地图中没有的障碍，用来检验局部代价地图、重规划和碰撞监控。

## 输出与判读

结果写入数据根 `reports/navigation_autonomy/<时间>_<模式>_<编号>/`：

| 文件 | 内容 |
|---|---|
| `result.json` | 状态、配置、参数读回、各阶段耗时、碰撞、定位与纯里程计误差、覆盖率（含按规划包络可达区的覆盖）或目标结果、进程退出码。 |
| `trajectory.csv` | 5 Hz 仿真真值、里程计、定位估计（换算到世界坐标）、闸门输出与导航原始指令。 |
| `events.jsonl` | 闸门状态变化、探索/任务状态、碰撞事件。 |
| `preview.png` | 真值世界、探索地图叠加、真实路径与定位估计、目标点。 |
| `map.yaml/map.pgm`、`explorer_result.json` 或 `mission_result.json` | 探索地图与任务明细。 |
| `nav2_params.yaml` 等参数文件、`logs/` | 本次实际使用的参数和每个进程日志。 |

`PASS` 表示任务完成且没有仿真碰撞；`PARTIAL` 表示没有碰撞但任务未完全完成（例如到时限、目标失败）；`FAIL` 表示出现碰撞或运行错误。定位误差以仿真真值计算，只在仿真中可得，实车没有对应真值。

## 在已录数据上检验建图

```bash
bash scripts/navigation_replay_mapping \
  --source-cell /数据根/data/analysis/<精确单左前端结果>/robot_localization_hz5_filter_on \
  --compare-map /数据根/data/analysis/<原生重建>/native/left_height_ray_05/export/map_3d.yaml
```

只读冻结前端（逐帧核对 CDR 哈希，结束再核对来源未变），不启动 ROS 节点或设备。用同一输入分别建纯里程计和扫描匹配两张图，每 `--holdout-every`（默认 5）帧留出 1 帧做自洽性评估；`--scan-height` 默认 −0.5..0.5 m（相对雷达）。输出在 `reports/navigation_replay_mapping/<时间>/`：每种方式一个目录，含 `map.yaml`、导航用的 `map_nav.yaml`、逐帧位姿与状态 `poses.csv`，以及 `result.json` 和对比图。缺测扇区不清空空间；结论是同一录包内的自洽性，不是真值。`--window 起 止`（秒，从前端第一帧算）只用这段时间里的帧建图，例如去掉开头或结尾的长时间静止；地图坐标系不变，仍是前端的里程计坐标系，`result.json` 记下 `frame_window_s`。`--lidar-time-offset-ms 75` 让每帧扫描配 stamp − 75 ms 处的冻结里程计（雷达时间戳比轮速/IMU 晚，和 AMCL 回放、实时链路的同名选项算法相同），默认 0。

`map.yaml` 按对数几率阈值分三类。`map_nav.yaml` 另外把“看过至少 5 次、其中回波不超过 25%”的格记为空闲：这类格多数时候被射线穿过，是有人走过或零星回波留下的痕迹，不是没观测到。没观测到或观测不足 5 次的格仍是未知，占据阈值仍然优先。规划器把未知格当障碍并按包络膨胀（与离线规划的保守约定一致），所以一个 5 cm 的未知格就能封住 2.6 m 走廊；`result.json` 的 `unknown_pockets` 分别给出两张图里被已知格包围的小未知块（≤20 格）数量。`--map-bin-deg 0.5|0.25` 用更细的方位扇区建图，留出帧略好，但小未知块并没有减少，因为它们来自有冲突证据的格，而不是射线间隙。

### 为没有冻结外参的录包生成单左前端

（历史记录：2026-11-07 起 RoomA1/RoomAB 录包停用，不再需要这一步。下面用到的 `config/local/project/hardware_setup.json` 也已在 2026-11-07 改成新的轮参数，不再是 `12ea737c…` 那份文件。）

2026-10-09 的 RoomA1/RoomAB 录包没有冻结左雷达外参，`compare` 会按外参能力检查拒绝。经用户同意，可以用 2026-10-26 现场外参作显式实验覆盖，参数与走廊前端完全相同（不加原生地图、运动候选或机械初值），输出目录名注明实验：

```bash
R=/media/nvidia/tiany-USB/wheelchair
python3 -s scripts/wc_phase1 compare \
  --dataset $R/data/experiments/RoomA1capture_2026_1009_154830 \
  --output  $R/data/analysis/<时间>_RoomA1_single_left_fieldext20261026_EXPERIMENTAL \
  --lidar left --cloud filtered --estimators robot_localization --input-rate-hz 5 --filter on \
  --hardware-setup config/local/project/hardware_setup.json
```

覆盖文件同时带入现场轮半径 0.172 m。运行前核对它的 SHA256 仍是 `12ea737c…`（与走廊前端相同）；录包与标定之间是否重装雷达没有记录，结论都要按实验外参解读。每份录包约 8–10 分钟，派生结果约 1–1.5 GB。

## 在真实录包上做 Nav2 影子运行

```bash
bash scripts/navigation_nav2_shadow \
  --source-cell /数据根/data/analysis/<精确单左前端结果>/robot_localization_hz5_filter_on \
  --map /数据根/reports/navigation_replay_mapping/<建图重放>/odom_grid/map_nav.yaml
bash scripts/navigation_nav2_shadow --mode follow --source-cell <同上> --map <同上>
```

录包里的轮椅运动带动 `base_link`（冻结轮速+IMU 先验，经冻结杆臂换到车轴），录到的 161 扇区扫描进入代价地图和碰撞监控；地图坐标就是录包的里程计坐标，`map→odom` 为恒等。Nav2 节点、参数读回和安全闸门与仿真导航完全相同，各级指令（控制器、平滑器、碰撞监控、闸门）都被记录，但没有任何进程接收闸门输出，录包运动也不会因指令改变。默认窗口从第一次运动前 5 s 到录包结束。

- `--mode navigate`（默认）：每 4 s 把 NavigateToPose 目标更新为录包中 15 s 之后的位姿（`--lead`、`--update-period`），由规划器选路线。检验在自建地图和真实扫描上能否规划、规划失败的原因、路线与司机是否一致。
- `--mode follow`：把司机接下来 15 s 的实际路线直接交给控制器（FollowPath），不经规划器和地图。检验控制器、碰撞监控和闸门沿司机路线会怎么做。
- 地图必须在录包里程计坐标中（例如同一录包的建图重放结果）；记录路径落在空闲格上的比例低于一半时拒绝运行。`--assumed-radius`、`--margin`、`--max-linear`、`--max-angular` 与仿真相同；`WC_AUTONOMY_ROS_DOMAIN=97` 可与另一次运行并行。
- `--footprint X1 Y1 X2 Y2 ...` 用凸多边形外形代替圆（车轴为原点，x 向前、y 向左，至少 3 个点，车轴须在外形内）。`--footprint-source` 写明外形来源，默认 `EXAMPLE_NOT_MEASURED`；实测后改为例如 `MEASURED_20261120`，结果和看板会按来源标注“示例”或“实测”。
- `--planner`、`--controller`（默认 NavFn 与 RPP，不变）选其他规划器和控制器，见[规划与控制](navigation_planning.md)。
- 地图来自另一段录包时用 `--localization fixed|amcl` 加 `--map-alignment`（固定对齐或已存地图上的 AMCL），见[实车影子链路](navigation_live_shadow.md)。

输出在 `reports/navigation_nav2_shadow/<时间>_shadow_<模式>_<编号>/`：

| 文件 | 内容 |
|---|---|
| `result.json` | 司机行驶时各级指令的结局占比、转向方向一致率、规划请求与每次失败的原因、静态地图沿路检查、扫描落入停车圈的帧、日志计数、来源哈希核对。 |
| `samples.csv` | 0.1 s 网格上的司机速度与四级指令、结局标签、与最新规划路径的横向距离。 |
| `plans.json`、`global_costmap_blocked.npz` | 每次全局规划路径；1 Hz 全局代价地图中不可通行的格（代价 ≥253），用于归因。 |
| `preview.png` | 地图上的司机路径、规划路径、规划失败位置（按原因着色），以及速度、角速度和最近回波随时间的曲线。 |

结局标签按速度链路依次判断：`DRIVES_FORWARD`（闸门输出前进）、`ROTATES_IN_PLACE`（控制器要求原地转向且已通过各级）、`STOPPED_BY_COLLISION_MONITOR`、`STOPPED_BY_GATE`、`SMOOTHER_ZERO`、`CONTROLLER_ZERO`（控制器输出全零，如前方碰撞、到达或取消）、`NAV2_NOT_COMMANDING`（0.5 s 内没有控制器输出，通常是没有可用规划）。规划失败先在静态地图上按 NavFn 的方式复核（起点格清空、4 连通、终点 0.25 m 容差），再用失败前最近一帧全局代价地图复核实时扫描标记，原因分为起点离地图障碍不足致命半径、未知格膨胀后截断路线、地图障碍截断、实时标记压住起点、实时标记截断路线和仍未能解释。

判读时注意这是开环：司机偏离 Nav2 路线后，Nav2 无法把轮椅拉回来，控制器会一直要求修正；15 s 前方目标会跳过司机的折返和进房间，规划路线在这些地方直接回头。因此 `navigate` 模式里“前进”占比主要看规划可用性和路线差异，控制器与安全链路的表现看 `follow` 模式。速度上限 0.30 m/s，司机常在 0.5–0.9 m/s，只比较方向与是否前进。结果不验证定位、包络或制动。

## 更新进度看板

把若干次运行加入看板，生成新的快照目录（不覆盖已有快照，原快照不改）：

```bash
bash scripts/navigation_dashboard_update \
  --base-snapshot /数据根/reports/navigation_dashboard/<上一版>/snapshot.json \
  --run /数据根/reports/navigation_autonomy/<探索任务> \
  --run /数据根/reports/navigation_autonomy/<导航任务> \
  --output-dir reports/navigation_dashboard/<新快照名>
```

只接受 `simulation_only=true`、`hardware_control=false` 的运行结果，逐个记录 `result.json` 的 SHA256，并把 `preview.png` 校验为 PNG 后内嵌进页面。`--shadow-run <影子运行目录>`（可重复）加入“真实录包影子运行”一节，只接受 `open_loop=true`、`commands_sent=false` 的结果；只给 `--shadow-run` 时保留上一版快照里的仿真表格，`--shadow-note` 在影子表格下追加说明。`--compare-run` 加入的运行只出现在表格里、不嵌入预览图，用于对比不同建图方式而不让页面过大。看板“进度总览”里新增“自主框架：仿真闭环”一节；页面仍标注离线快照、非实时、实车未验收。可选 `--tests-passed N --tests-source 路径` 更新测试数，`--next-remote 文本`（可重复）替换“下一步远程推进”，`--note 文本`（可重复）在仿真表格下追加说明。

## 边界

仿真结论只针对仿真：圆形假设包络、理想化平面激光、简化底盘与里程计误差。影子运行用了真实扫描，但仍是开环、恒等定位和同一假设包络。SLAM 调参、速度限制和安全闸门阈值都是候选值，接实车前要用现场测量重新确认。运行不会发布 `/cmd_vel`，域 96/97 中也没有真实设备节点。实现见[源码](../../src/wc_navigation/autonomy/README.md)，测试见[导航测试](../../tests/navigation/README.md)。返回[操作导航](README.md)。
