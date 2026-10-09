# 诊断计算测试

此目录按功能维护回归测试；共享测试数据使用 tests/fixtures，正式计算实现放对应 src 功能包。

## 内容与入口

- [check_level_ground.py](check_level_ground.py)
- [check_single_mapping_session.py](check_single_mapping_session.py)
- [inspect_scene_geometry.py](inspect_scene_geometry.py)
- [test_cartesian_transform_recovery.cpp](test_cartesian_transform_recovery.cpp)
- [test_mapping_time_ab.py](test_mapping_time_ab.py)

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

在受支持的 Linux 环境通过 `scripts/run_software_tests.py` 选择本目录的软件测试；需要原生组件的测试先完成对应构建。测试不得隐式启动真实设备。

依赖与构建方法见[项目入口](../../README.md)；返回[上级目录](../README.md)。
