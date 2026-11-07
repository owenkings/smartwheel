# 离线导航开发



在已保存的二维地图上运行 Nav2 路径规划，再用纯数值差速模型跟踪。输入为冻结地图、显式起终点或自动演示点、显式假设的圆形包络。此模块不读取实车设备、不运行控制器或车辆驱动。



- [cli.py](cli.py)：地图检查、存储归档、结果图与命令入口。

- [geometry.py](geometry.py)：坐标变换、未知区及边界膨胀、独立碰撞检查、数值跟踪。

- [nav2_planner.py](nav2_planner.py)：隔离 ROS 域中的 map_server 与 planner_server；仅计算路径。

- [__init__.py](__init__.py)：包声明。



运行方法和后续工作见[操作说明](../../docs/operations/navigation_offline.md)，测试见[导航测试](../../tests/navigation/README.md)。返回[源码导航](../README.md)。



运行依赖 ROS 2 Humble/Nav2、numpy、scipy、PyYAML、matplotlib；使用系统 Python 并禁用用户站点包。新增导航代码放本目录，运行产物经 StoragePolicy 保存到数据根 reports/navigation，不写入源码目录。



- [rebuild.py](rebuild.py)、[rebuild_cli.py](rebuild_cli.py)：单雷达派生前端与原生栅格重建。

- [grid_quality.py](grid_quality.py)：共同世界区域、包络连通性和高度/墙体支持评估。

- [replay_localization.py](replay_localization.py)、[replay_cli.py](replay_cli.py)：训练/留出分开的静态墙面回放定位。



操作见[录包重建与定位](../../docs/operations/navigation_rebuild_replay.md)。



- [autonomy/](autonomy/README.md)：自主探索建图、Nav2 规划与控制、安全闸门在仿真轮椅上的闭环框架，以及在真实录包上的开环影子运行，见[说明](../../docs/operations/navigation_autonomy.md)。
- [planning/](planning/README.md)：车体模型（圆或凸多边形）与规划、控制的可选配置和对比工具。

- [amcl_replay.py](amcl_replay.py)、[amcl_cli.py](amcl_cli.py)：隔离域中的 AMCL 单雷达功能回放。



- [localization_coverage.py](localization_coverage.py)：扫描终点地图覆盖与沿车头方向歧义复核；未知和地图外不作为空地，也不放行导航。



- [dynamic_simulation.py](dynamic_simulation.py)、[dynamic_cli.py](dynamic_cli.py)：冻结路径上的动态圆盘停车、等待和恢复数值场景；含有限制动和独立连续碰撞审计，不接车辆接口。说明见[动态仿真](../../docs/operations/navigation_dynamic.md)。



- [replanning.py](replanning.py)、[replan_cli.py](replan_cli.py)：当前观测障碍的临时占据覆盖、保守连通诊断、停稳后隔离 Nav2 重规划仿真。见[说明](../../docs/operations/navigation_replanning.md)。



- [replan_worker.py](replan_worker.py)：每个隔离Nav2请求的独立进程入口，正常结束及终止异常均走自有子进程清理。



- [recorded_observations.py](recorded_observations.py)、[shadow_safety.py](shadow_safety.py)、[recorded_cli.py](recorded_cli.py)：真实冻结录包的因果观测交付、车轴速度转换和影子停车触发；不执行车辆控制，见[说明](../../docs/operations/navigation_recorded_shadow.md)。



- [near_returns.py](near_returns.py)：完整保留点与体素包络见证、三维车体格复现诊断，见[近回波核查](../../docs/operations/navigation_near_returns.md)。

- [localization_trust.py](localization_trust.py)、[localization_trust_cli.py](localization_trust_cli.py)：按原始时间戳核查地图支持、匹配歧义与共享先验分歧，见[定位证据台账](../../docs/operations/navigation_localization_trust.md)。



- [dashboard.py](dashboard.py)、[progress_dashboard.html](progress_dashboard.html)：真实归档结果的只读进度看板，含地图回看、近点与定位证据。见[说明](../../docs/operations/navigation_dashboard.md)。


- [native_pair_registration.py](native_pair_registration.py)：未知车体安装变换时的原生左雷达点云一致性诊断，不生成车轴位姿。见[说明](../../docs/operations/navigation_native_pair.md)。
- [autonomy/](autonomy/README.md)：自主探索建图、Nav2 规划与控制、安全闸门在仿真轮椅上的闭环框架，以及在真实录包上的开环影子运行，见[说明](../../docs/operations/navigation_autonomy.md)。
