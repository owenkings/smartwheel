# 离线导航测试



验证坐标、包络、未知区、边界、通道、数值跟踪与命令输入。测试不启动实车驱动。Nav2 真实地图联调结果独立归档，不把合成输入测试视为实车验收。



运行：`bash tests/run_target_tests.sh tests/navigation`。



测试源文件放本目录，小型合成输入由测试生成；真实地图留在数据根。实现见[导航源码](../../src/wc_navigation/README.md)，返回[测试导航](../README.md)。



新增测试覆盖单侧字段与来源、原生栅格覆盖边界、不同地图原点的共同区域、低障碍支持、训练/留出隔离、退化拒绝与参考轨迹不参与估计。



覆盖与歧义诊断见 [test_localization_coverage.py](test_localization_coverage.py)，回放坐标连接见 [test_coverage_replay.py](test_coverage_replay.py)。验证未知区不贡献贴图支持、长走廊平移可混淆、末端墙区分，以及雷达与车轴参考一致性。



动态停车数值模型见 [test_dynamic_simulation.py](test_dynamic_simulation.py)，覆盖延迟观测、制动、恢复、断流及不可避免碰撞。负例通过表示检出了失败，不能解释为所有场景都无碰撞。看板更新的来源哈希、PNG 校验与离线标志见 [test_autonomy_dashboard.py](test_autonomy_dashboard.py)。这些测试不启动 ROS 闭环；完整仿真运行结果另行归档。

多边形车体的精确栅格检查（与密集采样对照）、原地转向扫过、薄墙不漏检、车体间隙也在 [test_autonomy_world.py](test_autonomy_world.py)；仿真车体随外形选项变化、默认仍是圆见 [test_autonomy_config.py](test_autonomy_config.py)。可选规划器与控制器（默认参数不变、不倒车、读回一致、覆盖项校验）、用真值世界当地图的导航配置见 [test_planning_stack.py](test_planning_stack.py)；规划台架的录包请求、司机路线分类、真实车体路径评分（间隙、重叠、原地转向、倒车）与汇总见 [test_planning_bench.py](test_planning_bench.py)；跟随模式审查（扫描是否与司机路线矛盾、原地转向计时）见 [test_planning_follow_audit.py](test_planning_follow_audit.py)。



重规划覆盖与连续路径见 [test_replanning.py](test_replanning.py)，停稳触发、延迟交付、观测失信拒绝和无路去重见 [test_dynamic_replanning.py](test_dynamic_replanning.py)。



真实录包影子评估见 [test_recorded_observations.py](test_recorded_observations.py)、[test_shadow_safety.py](test_shadow_safety.py)、[test_recorded_cli.py](test_recorded_cli.py)，覆盖捕获/交付时序、近点保留、杆臂速度转换、有限制动、陈旧观测和未验证覆盖。



近回波保留与体素成员核查见 [test_near_returns.py](test_near_returns.py)；定位证据因果性、坐标参考和完整帧台账见 [test_localization_trust.py](test_localization_trust.py) 与 [test_saved_localization_trust.py](test_saved_localization_trust.py)。



离线看板的快照状态、注入转义与报告路径保护见 [test_dashboard.py](test_dashboard.py)。


原生点云刚体配准、逆向一致性、平面弱约束与未知变换保留见 [test_native_pair_registration.py](test_native_pair_registration.py)。

自主框架的纯逻辑部分：仿真世界光线投射、碰撞与底盘模型见 [test_autonomy_world.py](test_autonomy_world.py)；安全闸门与定位修正监控见 [test_autonomy_gate.py](test_autonomy_gate.py)；前沿选择见 [test_autonomy_frontier.py](test_autonomy_frontier.py)；里程计栅格的射线标记、无回波/过近语义与裁剪见 [test_autonomy_occupancy.py](test_autonomy_occupancy.py)；扫描到地图修正的位姿恢复、走廊退化方向保持预测、跨墙对应点拒绝与大偏差拒绝见 [test_autonomy_scan_match.py](test_autonomy_scan_match.py)；离线重放建图的留出评估、缺测不清空、细扇区投影与小未知块计数见 [test_autonomy_replay_mapping.py](test_autonomy_replay_mapping.py)；录包影子运行的位姿与速度换算、停车圈计数、速度链路结局归属、规划失败的静态与代价地图归因、规划路径对比与无 ROS 的汇总见 [test_autonomy_shadow.py](test_autonomy_shadow.py)；Nav2/SLAM/AMCL 参数约束、行为树、碰撞监控停车圈、地图文件、命令配置与监督进程前置检查见 [test_autonomy_config.py](test_autonomy_config.py)；看板更新的来源哈希、PNG 校验与离线标志见 [test_autonomy_dashboard.py](test_autonomy_dashboard.py)。这些测试不启动 ROS 闭环；完整仿真运行结果另行归档。

三维障碍输入：合成点云上的高度窗、远处低点门限、车体剔除、悬空小团与双雷达见证、清除射线见 [test_perception_obstacle_cloud.py](test_perception_obstacle_cloud.py)；默认参数不变、局部代价地图 VoxelLayer 与碰撞监控点云源、读回、影子运行的打包与命令行选项、PointCloud2 消息与停车区核查见 [test_perception_nav2.py](test_perception_nav2.py)；离线检查的录包路径、未来扫掠外形与 /scan 基线见 [test_perception_evaluate.py](test_perception_evaluate.py)。
