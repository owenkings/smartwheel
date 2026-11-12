# 规划与控制：车体外形、规划器与控制器选项

仿真（`navigation_autonomy_sim`）和录包影子运行（`navigation_nav2_shadow`）共用下面的选项。不加任何选项时行为与以前完全相同：0.55 m 圆、NavFn、Regulated Pure Pursuit。设计与对比证据见[规划与控制设计](../architecture/planning_control.md)。

## 车体外形

```bash
WC_AUTONOMY_ROS_DOMAIN=98 bash scripts/navigation_autonomy_sim explore --world two_rooms \
  --footprint 0.81 0.2425 0.2 0.35 -0.255 0.35 -0.255 -0.35 0.2 -0.35 0.81 -0.2425 \
  --footprint-source MEASURED_20261107
```

- `--footprint X1 Y1 X2 Y2 ...`：凸多边形，车轴中点为原点，x 向前、y 向左，至少 3 个点，车轴须在外形内。给了外形，代价地图（全局加余量、局部不加）、碰撞监控和仿真碰撞都用它。仿真按多边形对栅格逐格精确判碰撞，每步沿运动（含原地转向）细分，任何车体点在相邻检查之间移动不超过半格。
- `--footprint-source`：外形来源标签，默认 `EXAMPLE_NOT_MEASURED`。当前统一用实测外形 `MEASURED_20261107`（上面的 6 个点：车轴到最前点 0.81 m、到最后点 0.255 m，全宽 0.70 m，脚踏段宽 0.485 m；内切半径 0.255 m，原地转向扫过 0.846 m，见看板共享产物）；0.55 m 圆只作对照。仿真只接受大写字母、数字和下划线。
- 仿真结果的 `result.json` 里 `body_model` 记录来源；`trajectory.minimum_true_gap_to_body_m` 对多边形是外形到障碍的精确间隙（`gap_basis` 为 `BODY_OUTLINE_AT_TRUE_POSE`，1 m 封顶），对圆仍是车轴间隙减半径。碰撞事件带 `body_gap_m`。

## 规划器与控制器

| 选项 | 取值 |
|---|---|
| `--planner` | `navfn`（默认）、`smac_2d`、`theta_star`、`smac_hybrid`（Hybrid-A*，只前进的 Dubins 运动，最小转弯半径 0.40 m）、`smac_lattice`（状态栅格，5 cm/0.5 m 差速原语，关倒车扩展） |
| `--planner-param NAME=VALUE` | 覆盖 `GridBased` 规划器参数，值按 JSON 解析，可重复，例如 `minimum_turning_radius=0.6` |
| `--planner-fallback 规划器` | 第二个规划器（插件名 `Fallback`），行为树只在 `--planner` 找不到路径时才问它，例如只前进的 `smac_hybrid` 后面接能原地转向的 `smac_lattice`；默认 `none` |
| `--planner-rescue navfn\|smac_2d\|theta_star` | 最后一个规划器（插件名 `Rescue`），前面的都找不到路径时才问它；默认 `none`。用来对付“起点压到余量”：AMCL 估计偏向墙、外形加余量已经碰到墙格时，按外形检查的规划器拒绝从这里起步，只查中心格的规划器照样能规划，车就能开出来（设计文档“起点压到余量”一节） |
| `--controller` | `rpp`（默认）、`shim_rpp`（新路线起点偏差超过 45° 时先原地转向并检查碰撞，之后 RPP 关闭行进中原地转向、沿曲率边走边转；只配 `smac_hybrid` 或 `smac_lattice`，因为它不在终点原地对正朝向，而 NavFn 的路径不管朝向，组合会卡在目标附近）、`mppi`（MPPI，差速模型，只前进 `vx_min=0`，代价评价按外形） |
| `--controller-param NAME=VALUE` | 覆盖 `FollowPath` 控制器参数，例如 `use_rotate_to_heading=false` |
| `--checker-param NAME=VALUE` | 覆盖目标检查和进度检查参数，只限 `general_goal_checker.*`、`progress_checker.*`，例如 `general_goal_checker.xy_goal_tolerance=0.35` |
| `--recovery-retries N`、`--recovery-wait S` | 行为树恢复轮数（默认 4）和每次等待秒数（默认 3）；恢复动作仍然只有清代价地图和等待，没有原地自转和倒车 |
| `--global-costmap live\|static` | `live`（默认）：导航时全局代价地图也标记实时扫描；`static`：只按保存的地图规划，实时扫描仍进局部代价地图、控制器碰撞检查和碰撞监控 |
| `--local-padding M` | 局部代价地图的外形余量，也就是 RPP 和 RotationShim 自己碰撞检查用的余量（默认 0，不变；最大 0.3 m）。设成和规划余量一样（0.10），控制器就不会把车开进规划器不接受的位姿；碰撞监控不受影响 |
| `--people-reaction` | 对航迹里的人做反应（减速、让行，靠速度上限），默认不开；只能和仿真的 `--tracks truth` 一起用，见下面“仿真”一节 |
| `--people-keepout` | 和 `--people-reaction` 一起用：站着不动满 3 s 的人在两个代价地图里另加禁行区，默认不开 |
| `--smoother-feedback gated\|open_loop\|odom` | 速度平滑器以为的当前速度。`gated`（仿真和录包影子的默认，11-10 用户决定）：平滑器自己上一次的输出，但闸门输出为零时取零，停车以后从静止起步；多启动一个 planning/velocity_feedback.py 进程（话题 `/wc_nav/smoother_feedback`，运行目录里记 `velocity_feedback.csv`）。`open_loop`：Nav2 的开环，只用自己上一次的输出，实时链路现在还是这样。`odom`：Nav2 的 /odom 闭环，只作对比，减速区里会几乎停住（见设计文档） |
| `--smoother-timeout S` | 平滑器的 velocity_timeout：控制器停发后，平滑器还继续发上一条速度多久（默认 1.0 s，不变；实车用 0.3 s，用户 11-10 同意；允许 0.2–5 s） |
| `--replanning periodic\|on_invalid` | `periodic`（默认，不变）：行为树每秒重新规划一次。`on_invalid`：目标不变、而且 Humble 的 IsPathValid 认为路径没被挡（沿路径每个位姿，加余量的外形压不到致命格）时，一直沿用当前路径；目标变了或路径被挡才重新规划（同 Humble 的 navigate_w_replanning_only_if_path_becomes_invalid.xml）。本来是想避免每秒重规划时路线在障碍两侧来回翻（11-11 闭环每翻一次就原地转一次），但 11-11 仿真里没有减少翻转（来回横移的行人：两次各翻 1 次和 5 次，periodic 各 1 次），不建议闭环用。原因：NavFn 路径中间的位姿朝向都是 0°，而 NavFn 只要求车轴离障碍 0.355 m，加余量的外形半宽是 0.45 m；路线离障碍不到约 0.45 m 时 IsPathValid 会一直判“被挡”，那一段照样每秒重规划 |

- `smac_hybrid`、`smac_lattice` 在每个朝向上检查外形（加余量）；`navfn`、`smac_2d`、`theta_star` 只按外形加余量后的内切半径通过，外形更长的车头和转向时扫出的范围不检查。Nav2 的外形检查只看外形的边和车轴处的膨胀代价，外形内部的孤立一两格看不到，所以地图里的孤立伪点要在规划用地图里先清掉（见设计文档）。
- 不倒车不能被覆盖：插件类型、`allow_reversing`、`allow_unknown`、`allow_reverse_expansion`、`motion_model_for_search` 等由选项和模式决定，覆盖它们会被拒绝。
- 运行前照常读回参数：规划器插件类型、未知区设置、不倒车相关设置和每一个覆盖项都要与请求一致，否则不开始。
- 影子运行的 `result.json` 在 `planner.plan_duration_s` 里给出每次规划从受理到结束的时长（仿真时钟，1 倍速时等于墙钟）；`log_counts` 另计 Smac 的起终点无效和迭代超限、MPPI 的优化失败、控制循环超时。
- 有 `--planner-fallback` 或 `--planner-rescue` 时，比规划成功率要看 `planner.plan_cycle_success_fraction`（按行为树的规划周期：任一规划器给出路径算成功，最后一个也失败才算失败；各规划器的失败数和接住的周期数在 `planner.plan_cycles`，从 planner_server 日志数）。`planner.plan_success_fraction` 按每次 ComputePathToPose 计数，GridBased 失败、Fallback 接住会记成一次失败加一次成功；只有一个规划器时两者相同。

## 仿真：用真值世界当地图

```bash
WC_AUTONOMY_ROS_DOMAIN=98 bash scripts/navigation_autonomy_sim navigate \
  --world-map /数据根/reports/navigation_replay_mapping/20261107T111004Z_3f0ae14a/odom_grid/map_nav.yaml \
  --start -0.323 -0.229 0 --goal 7.945 1.098 -2.336 --goal -3.04 -0.053 -3.111 \
  --planner smac_hybrid --footprint ...（同上） --footprint-source MEASURED_20261107
```

合成世界 `narrow_doors`（13 m × 8.4 m，第 1 版）按这栋楼和实测车体设计：1.8 m 走廊（y 3.2–5.0 m），上方三个房间的门宽 1.2、1.0、0.9 m，下方房间门宽 1.0 m、课桌间过道 1.0 m，起点 (1.0, 4.1, 0)。

`navigate --world 名称` 或 `--world-map 地图` 不需要先探索：真值世界本身就是导航地图（合成世界写到运行目录的 `world_map.yaml`，已保存地图直接使用；`--obstacle` 圆盘只在真值里、不进地图），世界坐标与地图坐标重合。这样比较规划和控制时不混入建图误差；AMCL 和闸门照常运行。起点要让车体（按外形）不与世界重叠。

`--localization truth`（只对 navigate）：仿真按真值发布 `map→odom`，只启动 map_server、不启动 AMCL，闸门不要求 AMCL 协方差（其余闸门条件照常）。用来在走廊地图上只比较规划和控制；默认 `amcl` 不变。走廊里 AMCL 的修正和协方差会单独决定结果（见设计文档），系统级结论仍要用 `amcl` 跑。

`--tracks truth --people-reaction`（只对 navigate，默认都不开）：仿真把行人（`--actors`）按 C 的航迹格式发到 `/obstacles/tracks`（来源和 seen_by 都是 truth），planning/people_node.py 按 people.py 决定速度上限，发到 `/speed_limit`（控制器服务器自己会用）。相隔不到 5 s 的几次让行算一次等待，等满 20 s 后这些人按站着处理（间隙 0.3 m、不预测）。再加 `--people-keepout` 时，站着不动满 3 s 的人在两个代价地图的 KeepoutFilter 里另画禁行区，半径是人的半径加 0.3 m 再加距离滞后余量；走动的人（包括走一下停一下的人）不画。默认不开禁行区：11-09 仿真里，人站在走廊车道上时禁行区把路堵死，Nav2 规划失败，约 10 s 后放弃目标；把走动的人预测的位置画进去，人来回横穿时走廊一直是堵的。运行目录里记 `people.jsonl`（每次决定）和 `people_costmaps.npz`（每秒一张局部代价地图和全局代价地图车附近 10 m 的截图，用来查旧位置残留）。实时链路还没有接。

`--drive-response ideal|zlac8030d_20261111`（仿真的 explore 和 navigate，默认 `ideal` 不变）：`zlac8030d_20261111` 让两只轮子按 11-11 实车数据（录制 ZLauto_loop_2026_1111_202618 和 ZLauto_stand_wasd_2026_1111_153154）的样子跟指令，模型在 autonomy/kinematics.py 的 `WheelResponse`：
- 先按轮控所有者的斜坡（0.3 m/s²、减速 0.6 m/s²、0.8 rad/s²）走，再拆成左右轮；
- 每只轮子按 0.12 s 跟上，低速起步更慢（目标低于 13 字时 0.8 s；1 字 = 0.112 rpm = 0.00193 m/s）；
- 原地转（两轮反向）时左轮到 93%、右轮只到 37%；
- 原地转结束后，右轮从第一个非零指令起再卡 0.7 s，然后按原地转时欠下的量冲过头（实车冲到 162–442 字，持续 0.4–0.6 s）。模型里，之后的指令和原地转时右轮同向才冲，反向只卡不冲。实车上反向时右轮也可能卡得更久再冲一下（20:27 右转原地转以后卡了约 2 s，冲到 281 字，只有一帧），从静止起步也会冲到 108–136 字（窗口 A 11-12 00:25 的报告），这两种模型都没有。
- 有轮子超过 14.4 rpm（0.248 m/s）持续 0.2 s，记一次“超速”。实车上轮控所有者在这里停车（FEEDBACK_OVERSPEED），仿真只计数。
- `result.json` 的 `drive_response` 段：原地转秒数、冲过头次数、超速次数和时刻、任意时长超限的次数、最快轮速。
- 拿实车的指令回放这个模型（~/.kiro_work_planning/fit_wheel_model.py）：20:27 闭环的超速时刻对上了（模型 134.15 s 判超速，实车 134.18 s 停车，壁钟秒的末三位），峰值 259 字（实车 304 字）；手动驾驶那段 8 次超速对上 4 次。没对上的是倒车弯道和很短的原地转。所以模型偏乐观，仿真里的超速次数只能当下限。

仿真也可以用窗口 C 的 `--monitor-base-shift off` 和 `--monitor-slow-margin M`（同影子运行），让碰撞监控和实车闭环一样（闭环模式是 base shift off、减速框伸出车头 0.15 m）。

`--ghosts FILE`（仿真的 explore 和 navigate，默认没有）：在仿真扫描里加雷达假点，比如左雷达的 12.49 m 相位折叠点。假点只进 /scan，不算实物：不进地图、不算碰撞，也不算最小间隙。文件格式（autonomy/world.py 的 `ghost_scenario`）：

```json
{"schema_version": 1, "name": "说明",
 "ghosts": [{"id": "fold", "frame": "start", "radius_m": 0.03, "points": [[1.45, 0.10], [1.50, 0.15]]}]}
```

- `points` 是 base_link 里的点（车轴原点，x 朝前，y 朝左，单位 m），每个点是半径 `radius_m`（默认 0.03 m）的小圆，最多 2000 个点。
- `frame` 是 `start` 时，点按起点位姿放进世界，之后固定不动（远处墙面的折叠，车往前开时它在世界里不动）；是 `chair` 时，点跟着车走（天花板折叠，一直在车前同样的位置）。
- 离雷达不到 0.3 m（扫描的最近距离）的假点不出现在扫描里。
- `result.json` 的 `ghosts` 段记场景文件的路径、sha256，以及扫描帧数、有假点的帧数、被假点占的光束数。
- 11-12 −x 闭环用的两个场景在 `R/reports/navigation_planning_analysis/20261112_minus_x_ghosts/`，取法见设计文档。

`--enable-pause AFTER_S DURATION_S`（只对 navigate，验证用，默认不开）：任务开始后 AFTER_S 仿真秒，监督进程的使能心跳改发 false，持续 DURATION_S（最多 30 s），闸门停车一次再放行，用来看放行后怎么起步（配合 `--smoother-feedback`）。时间窗写在 `result.json` 的 `supervisor.enable_pause_sim_s`。

`--goal-heading planned|turn_at_goal`（只对 navigate 任务的目标，默认 `planned` 不变）。`turn_at_goal` 时，任务节点不把目标朝向直接交给规划器，分三步：

1. 选到达朝向：
   - 按朝向规划的规划器（`smac_hybrid`、`smac_lattice`）：先按 12 个朝向（每 30°，从要求的朝向开始）各问一次 ComputePathToPose。要求朝向的路径不比最短的长 3% + 0.10 m 时，就用要求的朝向；否则在这个范围内选离要求朝向最近的一个。
   - 不管朝向的规划器：取路径最后 0.5 m 的方向。
2. 按到达朝向发 NavigateToPose。
3. 到点后，朝向差超过目标检查的 yaw 容差（默认 0.35 rad）时，检查原地转向扫过的范围：实测外形从当前朝向转到要求朝向扫过的区域，最大是半径 0.846 m 的圆。扫过的范围要离保存地图的占据和未知格、局部代价地图的障碍格都至少有规划余量那么远。先试较短的方向，不行再试另一个方向。有一个方向空着，就用 Nav2 的 Spin 原地转过去（Spin 自己的碰撞检查、碰撞监控和闸门照常）；两个方向都不空，车停在原地，记录里报告朝向差。

`mission_result.json` 每个目标多一个 `heading`：要求和到达的朝向、选择理由、每个候选朝向的路径长度、转向判断（每个方向在两张图里的最小间隙）、Spin 结果和最后的朝向差。这时 `succeeded` 要求到点并且朝向在容差内，`status` 仍是 NavigateToPose 的结果。车后没有传感器：从没看到过、地图里也没有的东西不会挡住转向，这和任何原地转向一样。

## 规划台架：同一批请求比较各规划器

```bash
WC_AUTONOMY_ROS_DOMAIN=98 bash scripts/navigation_planner_bench \
  --recording /数据根/reports/navigation_nav2_shadow/<影子运行>/recording.npz \
  --map <同一录包的 map_nav.yaml> --label ZL1107 --window 27.3 267 \
  --footprint ...（实测外形） --footprint-source MEASURED_20261107
```

一个 planner_server 同时加载所选规划器（默认 5 个），代价地图与 navigate 配置的全局代价地图相同但只有静态层和膨胀（没有实时扫描），所以只比较规划器本身。请求取自录包：t 时刻的车轴位姿为起点，`--lead`（默认 15 s）之后的位姿为终点，每 `--every`（默认 4 s）一个，每个请求依次交给每个规划器（ComputePathToPose，显式起点），记录服务器自己报告的 `planning_time`。`--recording` 也可以给冻结前端目录。`--planner-param 规划器.参数=值` 覆盖单个规划器；`--plan-radius R` 用半径 R 的圆（加余量）规划、仍按外形评分，用来比较“按半宽规划”的网格规划器；`--margin` 改规划余量。

评分用真实车体（不加余量）对地图占据格的精确间隙，路径朝向对 Hybrid-A* 和 State Lattice 用规划器给的朝向，其余用行进方向。每个请求先按司机自己的路线分类：`clear`（车体加余量处处放得下）、`tight`（车体放得下、余量放不下）、`conflict`（车体本身在司机路线上就与占据或未知格重叠，多半是地图里的临时物体或漂移，任何规划都不可能既成功又安全）。“安全成功”指路径上没有任何位姿让车体与占据或未知格重叠；“不安全成功”是规划器返回了、但真实车体跟不了的路径。

输出在 `reports/navigation_planner_bench/<时间>_bench_<标签>_<编号>/`：`result.json`（按规划器和类别的成功率、安全成功、耗时、间隙、急弯、原地转向、倒车、与司机路线的偏差）、`answers.jsonl`（每个请求每个规划器的结果和路径）、`requests.json`、`preview.png`。没有录包时用 `--requests-file 文件.json`（`[{"start": [x, y, yaw], "goal": [x, y, yaw]}, ...]`）给出请求，例如合成世界导出的地图。

## 推荐方案的命令行

[设计文档](../architecture/planning_control.md#推荐方案待复核)里的推荐方案（用户有条件同意，在新地图上复核通过后才改默认值）在仿真和影子运行里这样打开：

```bash
--planner smac_hybrid --planner-fallback smac_lattice --controller shim_rpp \
  --footprint 0.81 0.2425 0.2 0.35 -0.255 0.35 -0.255 -0.35 0.2 -0.35 0.81 -0.2425 \
  --footprint-source MEASURED_20261107
```

仿真 navigate 的任务再加 `--goal-heading turn_at_goal`：复核里只前进的 Hybrid-A* 为终点朝向在小房间里绕弯是失败原因之一，见设计文档。影子运行的目标由录包给出，不用这个选项。

11-10 的复核又加了 `--planner-rescue navfn`：AMCL 估计压进规划余量时，由只查中心格的 NavFn 接住（设计文档“复核（11-10）”一节）。rescue 算不算进推荐方案，等用户定。

前提是规划用地图里，司机路线上没有伪占据格（例如 v2 那样把车开过的格清空）；路线旁还剩孤立伪点时，这个组合会把车带到伪点前停住。仿真里比较规划和控制时可以加 `--localization truth`；要得出系统级结论，就用默认的 AMCL 跑，设计文档里的 AMCL 结果都是在 D 的闸门新默认值下跑的。

## 跟随模式审查：原地转向、停车原因、窄处速度

```bash
PYTHONPATH=src python3 -s -m wc_navigation.planning.follow_audit <影子运行目录>... --output 审查.json
```

不启动 ROS，只读影子运行目录（`recording.npz`、`samples.csv`、`run_config.json`、地图）。逐项给出：司机边走边转时控制器要求原地转向的时长；每次控制器停车时，最近 2 s 的扫描（各按采集位姿放进地图）有没有落进车体沿司机实际下一段 0.6 m 路线的范围——有，说明扫描本身与司机路线矛盾（近处或移动物体、近场或地面回波、外形比车大）；没有，才是控制器自己的预测弧线或原地转向造成的误停；以及司机前进时控制器和闸门输出速度按车体到静态地图间隙分段的中位数。

返回[操作导航](README.md)。
