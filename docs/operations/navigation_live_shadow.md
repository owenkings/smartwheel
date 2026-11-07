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
- `--localization amcl`：map_server + AMCL。起点是对齐后的录包起点，`--initial-pose-offset FORWARD_M LEFT_M YAW_DEG` 可加操作员误差，`--initial-pose-std` 默认 0.25 m、8.6°；`--amcl-params` 选 `framework`（默认）或一组候选改动。闸门同时要求 AMCL 协方差；`--gate-correction-limits TRANSLATION_M YAW_DEG`、`--gate-covariance-limits XY_STD_M YAW_STD_DEG` 和 `--gate-correction-reference odom_origin|robot` 改闸门的定位条件，默认与闸门本身相同（0.25 m、5°，0.30 m、14.3°，按 `map→odom` 量）。
- fixed 和 amcl 需要 `--map-alignment X Y YAW_DEG` 或 `--map-alignment-json`；记录路径放进地图后至少一半落在空闲格上才运行。

输出与原影子运行相同，另有 `localization_samples.csv`（10 Hz 的 `map→odom` 和每个 AMCL 位姿与协方差）、`recording_map.npz`（放进地图的录包，给 carrot 和统计用）和 `result.json` 的 `localization` 一节：AMCL 与对齐的差、端点拟合（AMCL 与对齐）、闸门实际因定位不可信停车的秒数（按原因）、离线换阈值重算的不可信比例（两种修正量法、几档平移和航向、几档协方差、端点支撑），以及 2 s 内修正变化的分布。

判读：follow 模式的路线按对齐放进地图，AMCL 修正越大，车体估计离路线越远，控制器的原地转向和前方碰撞会因此增加；这是定位修正对跟线的影响，不全是控制器的问题。端点支撑条件目前只在离线重算里，闸门还没有这一项。

返回[操作导航](README.md)。
