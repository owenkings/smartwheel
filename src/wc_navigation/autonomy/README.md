# 自主导航框架（仿真闭环）

把自主建图、路径规划和自动驾驶串成一条完整链路，在独立的本机 ROS 域 96 中对一辆仿真轮椅闭环运行；同一套 Nav2 配置也能在冻结录包上开环影子运行（录到的扫描与运动，指令不发出）。实时传感器、外参标定和车辆接口都不参与；本目录也不连接轮控所有者。

## 组成

| 文件 | 职责 |
|---|---|
| [world.py](world.py) | 仿真真值世界：合成楼层或已保存地图（未知按实体处理）、额外圆形障碍、平面激光光线投射、车体碰撞判断（假设圆，或与 `NavProfile.footprint` 相同的凸多边形，含原地转向扫过，见 [planning/body.py](../planning/body.py)）。 |
| [actors.py](actors.py) | 仿真里的移动行人（`--actors 场景.json`）：按路点和速度走的圆盘，可选让行距离；每个物理步之前并入世界，结果里给最小间隙、接触和行人附近的停车时间，见[障碍感知操作说明](../../../docs/operations/navigation_obstacles.md)。 |
| [kinematics.py](kinematics.py) | 差速底盘：加速度上限、指令超时刹停、带比例误差/偏置/噪声的里程计（`wheel_imu`、`wheel_only`、`ideal`）。 |
| [safety_gate.py](safety_gate.py) | 最后一道软件闸门：急停锁存、操作员使能心跳、定位可信、地图修正突变、指令新鲜；限速并禁止倒车。 |
| [frontier.py](frontier.py) | 前沿探索：前沿聚类、带额外间隙的已知空闲目标、同一连通区可达性与黑名单。 |
| [occupancy.py](occupancy.py)、[scan_match.py](scan_match.py)、[mapper_node.py](mapper_node.py) | 探索建图：对数几率栅格。默认只用轮速+IMU 里程计（`odom_grid`，`map` 与 `odom` 重合）；实验选项 `odom_icp` 以里程计预测为先验，做法向一致、只沿受约束方向、有上限的点到线修正并发布 `map→odom`。都不做回环。 |
| [nav2_config.py](nav2_config.py) | Nav2、slam_toolbox（可选建图）、AMCL 参数，无倒车/无自转恢复的行为树，速度链路与读回清单。 |
| [maps.py](maps.py) | 探索地图的 PGM/YAML 写出与读回核对。 |
| [sim_node.py](sim_node.py)、[gate_node.py](gate_node.py)、[explorer_node.py](explorer_node.py)、[mission_node.py](mission_node.py) | 仿真、闸门、探索与目标任务四个独立进程。 |
| [bringup.py](bringup.py) | 监督一次运行：域锁、按序启动、生命周期激活、参数读回、初始位姿、记录真值与指令、只清理自己启动的进程。 |
| [report.py](report.py)、[cli.py](cli.py) | 指标、覆盖率、预览图与命令入口。 |
| [dashboard_update.py](dashboard_update.py) | 把归档的仿真运行加入离线进度看板的新副本，原快照不改。 |
| [replay_mapping.py](replay_mapping.py) | 在冻结的单左前端上离线重放两种建图方式，用留出帧比较自洽性，另存按观测证据分类的导航用地图 `map_nav.yaml`；不启动节点或设备。 |
| [shadow.py](shadow.py)、[replay_node.py](replay_node.py)、[carrot_node.py](carrot_node.py)、[shadow_cli.py](shadow_cli.py) | 录包影子运行：录包运动带动 `base_link`、录到的扫描进入代价地图与碰撞监控；Nav2 朝录包前方位姿规划（`navigate`）或直接跟随司机路线（`follow`）。各级指令只记录不发出；规划失败按静态地图和录下的全局代价地图归因。 |
| [shadow_localization.py](shadow_localization.py)、[shadow_localization_node.py](shadow_localization_node.py) | 影子运行的定位模式：恒等（默认）、地图来自另一段录包时的固定对齐、已存地图上的 AMCL；闸门定位条件在真实 AMCL 输出上的统计与离线换阈值重算，见[实车影子链路](../../../docs/operations/navigation_live_shadow.md)。 |

速度链路：`controller_server/behavior_server → /cmd_vel_nav → velocity_smoother → /cmd_vel_smoothed → collision_monitor → /cmd_vel_monitored → 安全闸门 → /wc_nav/cmd_vel_gated → 底盘`。整个链路没有进程发布 `/cmd_vel`。

## 入口与验证

命令为 `bash scripts/navigation_autonomy_sim explore|navigate`、`bash scripts/navigation_replay_mapping`、`bash scripts/navigation_nav2_shadow` 与 `bash scripts/navigation_dashboard_update`，用法见[操作说明](../../../docs/operations/navigation_autonomy.md)，设计见[框架说明](../../../docs/architecture/autonomy_framework.md)。纯逻辑测试为 `tests/navigation/test_autonomy_*.py`，运行 `bash tests/run_target_tests.sh tests/navigation`；完整闭环运行结果按任务归档，不能代替实车验收。

依赖 ROS 2 Humble、Nav2 1.1.20（含 collision_monitor、RPP）、slam_toolbox 2.6、numpy、scipy、matplotlib；使用系统 Python 并禁用用户站点包。新增仿真或自主功能放本目录；接实车的适配器另建目录并单独评审。返回[离线导航开发](../README.md)。
