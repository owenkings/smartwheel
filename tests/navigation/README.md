# 离线导航测试

验证坐标、包络、未知区、边界、通道、数值跟踪与命令输入。测试不启动实车驱动。Nav2 真实地图联调结果独立归档，不把合成输入测试视为实车验收。

运行：`bash tests/run_target_tests.sh tests/navigation`。

测试源文件放本目录，小型合成输入由测试生成；真实地图留在数据根。实现见[导航源码](../../src/wc_navigation/README.md)，返回[测试导航](../README.md)。

新增测试覆盖单侧字段与来源、原生栅格覆盖边界、不同地图原点的共同区域、低障碍支持、训练/留出隔离、退化拒绝与参考轨迹不参与估计。

覆盖与歧义诊断见 [test_localization_coverage.py](test_localization_coverage.py)，回放坐标连接见 [test_coverage_replay.py](test_coverage_replay.py)。验证未知区不贡献贴图支持、长走廊平移可混淆、末端墙区分，以及雷达与车轴参考一致性。

动态停车数值模型见 [test_dynamic_simulation.py](test_dynamic_simulation.py)，覆盖延迟观测、制动、恢复、断流及不可避免碰撞。负例通过表示检出了失败，不能解释为所有场景都无碰撞。

重规划覆盖与连续路径见 [test_replanning.py](test_replanning.py)，停稳触发、延迟交付、观测失信拒绝和无路去重见 [test_dynamic_replanning.py](test_dynamic_replanning.py)。

真实录包影子评估见 [test_recorded_observations.py](test_recorded_observations.py)、[test_shadow_safety.py](test_shadow_safety.py)、[test_recorded_cli.py](test_recorded_cli.py)，覆盖捕获/交付时序、近点保留、杆臂速度转换、有限制动、陈旧观测和未验证覆盖。

近回波保留与体素成员核查见 [test_near_returns.py](test_near_returns.py)；定位证据因果性、坐标参考和完整帧台账见 [test_localization_trust.py](test_localization_trust.py) 与 [test_saved_localization_trust.py](test_saved_localization_trust.py)。

离线看板的快照状态、注入转义与报告路径保护见 [test_dashboard.py](test_dashboard.py)。

原生点云刚体配准、逆向一致性、平面弱约束与未知变换保留见 [test_native_pair_registration.py](test_native_pair_registration.py)。
