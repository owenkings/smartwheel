# 规划与控制：车体外形、规划器与控制器选项

仿真（`navigation_autonomy_sim`）和录包影子运行（`navigation_nav2_shadow`）共用下面的选项。不加任何选项时行为与以前完全相同：0.55 m 圆、NavFn、Regulated Pure Pursuit。设计与对比证据见[规划与控制设计](../architecture/planning_control.md)。

## 车体外形

```bash
WC_AUTONOMY_ROS_DOMAIN=98 bash scripts/navigation_autonomy_sim explore --world two_rooms \
  --footprint 0.79 0.35 -0.35 0.35 -0.35 -0.35 0.79 -0.35 \
  --footprint-source FRONT_FROM_BLIND_RECORDING_REST_EXAMPLE
```

- `--footprint X1 Y1 X2 Y2 ...`：凸多边形，车轴中点为原点，x 向前、y 向左，至少 3 个点，车轴须在外形内。给了外形，代价地图（全局加余量、局部不加）、碰撞监控和仿真碰撞都用它。仿真按多边形对栅格逐格精确判碰撞，每步沿运动（含原地转向）细分，任何车体点在相邻检查之间移动不超过半格。
- `--footprint-source`：外形来源标签，默认 `EXAMPLE_NOT_MEASURED`。当前可用的部分实测外形标 `FRONT_FROM_BLIND_RECORDING_REST_EXAMPLE`（车头 0.79 m 来自盲区录包，半宽 0.35 m 和车尾 0.35 m 是示例）；完整实测后用 `MEASURED_<日期>`。仿真只接受大写字母、数字和下划线。
- 仿真结果的 `result.json` 里 `body_model` 记录来源；`trajectory.minimum_true_gap_to_body_m` 对多边形是外形到障碍的精确间隙（`gap_basis` 为 `BODY_OUTLINE_AT_TRUE_POSE`，1 m 封顶），对圆仍是车轴间隙减半径。碰撞事件带 `body_gap_m`。

## 规划器与控制器

| 选项 | 取值 |
|---|---|
| `--planner` | `navfn`（默认）、`smac_2d`、`theta_star`、`smac_hybrid`（Hybrid-A*，只前进的 Dubins 运动，最小转弯半径 0.40 m）、`smac_lattice`（状态栅格，5 cm/0.5 m 差速原语，关倒车扩展） |
| `--planner-param NAME=VALUE` | 覆盖 `GridBased` 规划器参数，值按 JSON 解析，可重复，例如 `minimum_turning_radius=0.6` |
| `--controller` | `rpp`（默认） |
| `--controller-param NAME=VALUE` | 覆盖 `FollowPath` 控制器参数，例如 `use_rotate_to_heading=false` |

- `smac_hybrid`、`smac_lattice` 在每个朝向上检查完整外形（加余量）；`navfn`、`smac_2d`、`theta_star` 只按外形加余量后的内切半径通过，外形更长的车头和转向时扫出的范围不检查。
- 不倒车不能被覆盖：插件类型、`allow_reversing`、`allow_unknown`、`allow_reverse_expansion`、`motion_model_for_search` 等由选项和模式决定，覆盖它们会被拒绝。
- 运行前照常读回参数：规划器插件类型、未知区设置、不倒车相关设置和每一个覆盖项都要与请求一致，否则不开始。
- 影子运行的 `result.json` 在 `planner.plan_duration_s` 里给出每次规划从受理到结束的时长（仿真时钟，1 倍速时等于墙钟）。

返回[操作导航](README.md)。
