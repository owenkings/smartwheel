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
| `--controller` | `rpp`（默认）、`shim_rpp`（新路线起点偏差超过 45° 时先原地转向并检查碰撞，之后 RPP 关闭行进中原地转向、沿曲率边走边转）、`mppi`（MPPI，差速模型，只前进 `vx_min=0`，代价评价按外形） |
| `--controller-param NAME=VALUE` | 覆盖 `FollowPath` 控制器参数，例如 `use_rotate_to_heading=false` |
| `--checker-param NAME=VALUE` | 覆盖目标检查和进度检查参数，只限 `general_goal_checker.*`、`progress_checker.*`，例如 `general_goal_checker.xy_goal_tolerance=0.35` |
| `--recovery-retries N`、`--recovery-wait S` | 行为树恢复轮数（默认 4）和每次等待秒数（默认 3）；恢复动作仍然只有清代价地图和等待，没有原地自转和倒车 |
| `--global-costmap live\|static` | `live`（默认）：导航时全局代价地图也标记实时扫描；`static`：只按保存的地图规划，实时扫描仍进局部代价地图、控制器碰撞检查和碰撞监控 |

- `smac_hybrid`、`smac_lattice` 在每个朝向上检查完整外形（加余量）；`navfn`、`smac_2d`、`theta_star` 只按外形加余量后的内切半径通过，外形更长的车头和转向时扫出的范围不检查。
- 不倒车不能被覆盖：插件类型、`allow_reversing`、`allow_unknown`、`allow_reverse_expansion`、`motion_model_for_search` 等由选项和模式决定，覆盖它们会被拒绝。
- 运行前照常读回参数：规划器插件类型、未知区设置、不倒车相关设置和每一个覆盖项都要与请求一致，否则不开始。
- 影子运行的 `result.json` 在 `planner.plan_duration_s` 里给出每次规划从受理到结束的时长（仿真时钟，1 倍速时等于墙钟）；`log_counts` 另计 Smac 的起终点无效和迭代超限、MPPI 的优化失败、控制循环超时。

## 仿真：用真值世界当地图

```bash
WC_AUTONOMY_ROS_DOMAIN=98 bash scripts/navigation_autonomy_sim navigate \
  --world-map /数据根/reports/navigation_replay_mapping/20261107T111004Z_3f0ae14a/odom_grid/map_nav.yaml \
  --start -0.323 -0.229 0 --goal 7.945 1.098 -2.336 --goal -3.04 -0.053 -3.111 \
  --planner smac_hybrid --footprint ...（同上） --footprint-source MEASURED_20261107
```

合成世界 `narrow_doors`（13 m × 8.4 m，第 1 版）按这栋楼和实测车体设计：1.8 m 走廊（y 3.2–5.0 m），上方三个房间的门宽 1.2、1.0、0.9 m，下方房间门宽 1.0 m、课桌间过道 1.0 m，起点 (1.0, 4.1, 0)。

`navigate --world 名称` 或 `--world-map 地图` 不需要先探索：真值世界本身就是导航地图（合成世界写到运行目录的 `world_map.yaml`，已保存地图直接使用；`--obstacle` 圆盘只在真值里、不进地图），世界坐标与地图坐标重合。这样比较规划和控制时不混入建图误差；AMCL 和闸门照常运行。起点要让车体（按外形）不与世界重叠。

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

## 跟随模式审查：原地转向、停车原因、窄处速度

```bash
PYTHONPATH=src python3 -s -m wc_navigation.planning.follow_audit <影子运行目录>... --output 审查.json
```

不启动 ROS，只读影子运行目录（`recording.npz`、`samples.csv`、`run_config.json`、地图）。逐项给出：司机边走边转时控制器要求原地转向的时长；每次控制器停车时，最近 2 s 的扫描（各按采集位姿放进地图）有没有落进车体沿司机实际下一段 0.6 m 路线的范围——有，说明扫描本身与司机路线矛盾（近处或移动物体、近场或地面回波、外形比车大）；没有，才是控制器自己的预测弧线或原地转向造成的误停；以及司机前进时控制器和闸门输出速度按车体到静态地图间隙分段的中位数。

返回[操作导航](README.md)。
