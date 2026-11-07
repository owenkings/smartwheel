# 命令入口

用户命令、构建和维护入口放这里；复杂计算与可复用逻辑放入 src 对应功能包。

## 内容与入口

- [check_layout.py](check_layout.py)：检查维护目录 README 覆盖和相对文档链接。
- [align_lidar_clouds.sh](align_lidar_clouds.sh)
- [build_tools_and_driver.sh](build_tools_and_driver.sh)
- [build_verify.sh](build_verify.sh)
- [capture_usb](capture_usb)
- [collect_delivery_evidence.py](collect_delivery_evidence.py)
- [configure_storage](configure_storage)
- [confirm_gyro_bias](confirm_gyro_bias)
- [dev_archive](dev_archive)
- [lidar_field_check](lidar_field_check)：现场外参检查，录一项后当场判定合格与否（只读采集）。
- [lidar_field_solve](lidar_field_solve)：离线联合求解现场外参候选，并用独立录包验证。
- [discover_remote.sh](discover_remote.sh)
- [map](map)
- [map_single_lidar.sh](map_single_lidar.sh)
- [panel](panel)
- 其余同类文件遵守本目录的职责和命名规则。

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

修改后运行[对应功能测试](../tests/README.md)；涉及入口、配置、资源或路径时同时检查安装后的调用与旧数据读取。

依赖与构建方法见[项目入口](../README.md)；返回[上级目录](../README.md)。

- [navigation_offline](navigation_offline)：隔离的 Nav2 路径规划和数值跟踪。

- [navigation_rebuild](navigation_rebuild)：冻结单左前端的原生地图重建。
- [navigation_replay](navigation_replay)：训练/留出时段分开的单雷达回放定位。

- [navigation_amcl_replay](navigation_amcl_replay)：重建地图上的 AMCL 单雷达功能回放。

- [navigation_dynamic_sim](navigation_dynamic_sim)：读取已归档成功的离线 Nav2 结果，在原地图和原包络上运行纯数值动态停车场景，见[使用说明](../docs/operations/navigation_dynamic.md)。

- [navigation_replan_sim](navigation_replan_sim)：在冻结保护地图上向当前观测障碍加占据，停稳后调用隔离 Nav2，数值验证绕行或无路等待；见[说明](../docs/operations/navigation_replanning.md)。

- [navigation_recorded_shadow](navigation_recorded_shadow)：对已校验单左前端做不发命令的停车影子评估及延迟/断流注入，见[说明](../docs/operations/navigation_recorded_shadow.md)。

- [navigation_localization_trust](navigation_localization_trust)：原始时间戳精确对齐的离线定位证据核查，见[说明](../docs/operations/navigation_localization_trust.md)。

- [navigation_dashboard](navigation_dashboard)：生成可直接浏览的离线导航进度快照，见[说明](../docs/operations/navigation_dashboard.md)。

- [navigation_autonomy_sim](navigation_autonomy_sim)：在 ROS 域 96 中对仿真轮椅运行自主探索建图或 AMCL 目标导航，见[说明](../docs/operations/navigation_autonomy.md)。
- [navigation_dashboard_update](navigation_dashboard_update)：把归档的仿真运行加入离线进度看板的新副本，见[说明](../docs/operations/navigation_autonomy.md#更新进度看板)。
- [navigation_progress_overview](navigation_progress_overview)：按人工整理的内容生成给用户看的一页进展总览（验证阶梯、关键数字、各工作线、在用的走廊地图），见[说明](../docs/operations/navigation_dashboard.md#一页进展总览)。
- [navigation_replay_mapping](navigation_replay_mapping)：在冻结的单左前端上离线重放探索建图（纯里程计与扫描匹配对比、留出帧自洽性），见[说明](../docs/operations/navigation_autonomy.md#在已录数据上检验建图)。
- [navigation_nav2_shadow](navigation_nav2_shadow)：在冻结录包上开环运行 Nav2 全链路（规划或沿司机路线），记录各级速度指令但不发出，见[说明](../docs/operations/navigation_autonomy.md#在真实录包上做-nav2-影子运行)。
- [navigation_planner_bench](navigation_planner_bench)：同一批录包起终点、同一张静态代价地图上并排比较各 Nav2 规划器（成功率、规划耗时、车体到障碍间隙、窄过道），见[说明](../docs/operations/navigation_planning.md)。
- [navigation_obstacle_eval](navigation_obstacle_eval)：在冻结左右前端上离线检查三维障碍过滤，统计录包未来扫掠外形里的障碍点，并与 /scan 对比，见[说明](../docs/operations/navigation_obstacles.md)。
