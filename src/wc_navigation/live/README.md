# 定位与实车影子链路

验证阶梯第 4 级（实车影子模式）需要的定位和实时链路：已存地图上的定位、把实时传感器转成 Nav2 输入、只记录不下发的影子启动器。本目录的程序都不打开设备、不连接轮控、不发布 `/cmd_vel`。

| 文件 | 职责 |
|---|---|
| [amcl_cross.py](amcl_cross.py) | 跨录包定位检查：验证录包的冻结扫描和里程计，在另一段建图录包的地图上跑 map_server + AMCL（隔离域 95，经 `amcl_replay`）。初值来自两段里程计坐标系的刚体对齐和刻意扰动；与固定对齐（纯里程计）比较扫描端点到地图的距离，统计协方差、收敛与失败、沿车头方向的歧义，以及闸门条件在真实 AMCL 输出上的表现。起点和终点在地图空白区（门内）的帧单独统计，中段帧再按转弯、直行、静止分开；`--lidar-time-offset-ms`（默认 0）把每帧扫描配到 stamp − 偏移处的里程计，用来比较雷达时间偏移。 |
| [config.py](config.py) | 实时链路配置：外参、主机滤波、运动先验和设备编号取自冻结前端，轮参数和陀螺零偏由显式标定文件替换，IMU 用到达时间。 |
| [scan_adapter.py](scan_adapter.py)、[odometry_adapter.py](odometry_adapter.py) | 左雷达 host_filtered 帧经生产主机滤波、旋到 `mapping_reference` 后按离线 `build_scan` 投影成 161 格扫描；轮速记录和 H30 帧经原样使用的 `MotionPrior` 得到车轴里程计。 |
| [inputs_node.py](inputs_node.py) | 扫描进程（`/scan`）和里程计进程（`/odom`、`odom→base_link`、静态 `base_link→laser`），状态在 `/wc_live/inputs_status/<role>`。雷达时间偏移（默认取 `config/calibration/lidar_time_offset.json` 的批准值，现为 75 ms）让 `/scan` 比雷达帧早打时间戳。 |
| [relay_node.py](relay_node.py) | 三个传感器话题从传感器域到影子隔离域的单向原样转发（best effort、volatile、keep last；车上等面板自己的订阅者到齐才订阅；影子域里改名为 `/wc_live/relayed/...`）；开三维障碍输入时加右雷达，开超声波时加所配探头的 Range 和 A 的超声波状态；转发非仿真域需要用户当次同意。 |
| [ultrasonic_check.py](ultrasonic_check.py) | `--monitor-ultrasonic`：从冻结配置读 A 的超声波配置，按 A 的 `probe_transforms` 算碰撞监控要的静态变换；闭环要 A 的现场核对；闸门按 C 的 `range_health` 判每个探头的读数；状态行的超声波一段。 |
| [current_config.py](current_config.py) | `freeze-config`：用 `wc_runtime.mapping_controller.configuration()` 解析面板当前的实时配置（只读文件），按前端目录格式冻结并记 sha256，和最近的前端核对；上车时作 `--cell`。 |
| [recorder_node.py](recorder_node.py)、[goals_node.py](goals_node.py) | 记录器（四级指令、实际运动、定位、规划、状态，逐行落盘）和操作员目标序列。 |
| [supervisor.py](supervisor.py)、[cli.py](cli.py)、[bag_check.py](bag_check.py)、[shadow_summary.py](shadow_summary.py) | 一条命令启动录包检验或实车影子运行（域锁、按序启动、参数读回、初始位姿、只停自己的进程）、与冻结前端的比较和影子运行的汇总；上车前检查单。 |

命令为 `bash scripts/navigation_amcl_replay cross ...` 和 `bash scripts/navigation_live_shadow inputs|shadow|checklist ...`，用法见[实车影子操作说明](../../../docs/operations/navigation_live_shadow.md)，设计见[实车影子链路](../../../docs/architecture/live_shadow.md)。测试为 `tests/navigation/test_amcl_cross.py`、`test_live_chain.py`、`test_live_checks.py`。返回[离线导航开发](../README.md)。
