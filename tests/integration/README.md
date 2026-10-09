# 跨模块流程与隔离集成测试

此目录按功能维护回归测试；共享测试数据使用 tests/fixtures，正式计算实现放对应 src 功能包。

## 内容与入口

- [ros_tools](ros_tools/README.md)：下一级职责说明。
- [analyze_mapping_geometry_20260915.py](analyze_mapping_geometry_20260915.py)
- [check_alignment_browser.cjs](check_alignment_browser.cjs)
- [check_amplitude_browser.cjs](check_amplitude_browser.cjs)
- [check_calculation_browser.cjs](check_calculation_browser.cjs)
- [check_continuous_mapping_gap.py](check_continuous_mapping_gap.py)
- [check_dashboard_candidate_grid.py](check_dashboard_candidate_grid.py)
- [check_dashboard_delayed_map.py](check_dashboard_delayed_map.py)
- [check_dashboard_recorded_scene.py](check_dashboard_recorded_scene.py)
- [check_dashboard_recorded_visuals.py](check_dashboard_recorded_visuals.py)
- [check_dashboard_saved_render.py](check_dashboard_saved_render.py)
- [check_fusion_ros_faults.py](check_fusion_ros_faults.py)
- [check_recorder.py](check_recorder.py)
- [check_icp_ablation.py](check_icp_ablation.py)
- 其余同类文件遵守本目录的职责和命名规则。

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

在受支持的 Linux 环境通过 `scripts/run_software_tests.py` 选择本目录的软件测试；需要原生组件的测试先完成对应构建。测试不得隐式启动真实设备。

依赖与构建方法见[项目入口](../../README.md)；返回[上级目录](../README.md)。
