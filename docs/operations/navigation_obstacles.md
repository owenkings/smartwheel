# 障碍感知：三维障碍输入的影子运行与离线检查

用左右雷达的三维点云代替 /scan 做局部代价地图和碰撞监控的障碍输入，在冻结录包上开环检验。只读冻结前端和地图，不打开设备、不发布速度指令；影子运行在独立的 ROS 域里，本窗口用 99。设计与证据见[障碍感知与避障](../architecture/obstacle_avoidance.md)。数据只用 ZL1107 走廊系列。

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
| `--obstacle-input scan\|cloud\|scan_and_cloud` | 局部代价地图和碰撞监控用什么。`scan`（默认）与以前完全相同；`cloud` 用三维障碍点代替 /scan；`scan_and_cloud` 两者都用。全局代价地图始终用 /scan。 |
| `--obstacle-right-cell` | 同一录包的单右冻结前端。给了才有右雷达障碍点，也才有两台雷达互相见证。 |
| `--obstacle-sides left right` | 发布哪几台的障碍点（默认：左，给了右前端时左右都发）。 |
| `--obstacle-no-consensus` | 不用另一台雷达否决悬空小团，只按尺寸判别。 |
| `--obstacle-right-lever frozen\|symmetric\|X,Y,Z` | 右雷达杆臂：冻结的现场外参（默认）、左雷达的镜像位置，或显式值。 |
| `--obstacle-filter '{"max_range_m": 3.5}'` | 覆盖过滤参数（`ObstacleFilter` 的字段，见源码）。 |

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

不启动 ROS 节点。对每帧过滤后的障碍点，检查它们是否落在车接下来 `--horizon`（默认 4 s）秒的录包路径扫过的外形里（外形内缩 3 cm）；静止障碍不可能在那里，所以这类点是伪点或走开的物体。同时对当前 /scan 做同样统计。输出在 `reports/navigation_obstacles/<时间>_<标签>_<编号>/`：`result.json`（汇总、参数、来源）、`frames.jsonl`（逐帧计数）、`swept_points.npz`（落进扫掠外形的点）。

## 边界

结论只针对冻结录包和空车外形。过滤参数（高度窗、远处低点门限 2 m、悬空小团规则）是在建图包上定的，验证包和以后的走廊录包要再看。坐人时乘坐者伸出外形的部分会被当成障碍，需要坐人录包确认。返回[操作导航](README.md)。
