# 命令入口和运行流程测试

此目录按功能维护回归测试；共享测试数据使用 tests/fixtures，正式计算实现放对应 src 功能包。

## 内容与入口

- [cli_probe.py](cli_probe.py)
- [process_helper.py](process_helper.py)
- [test_camera_encoder_cli.py](test_camera_encoder_cli.py)
- [test_cli.py](test_cli.py)
- [test_compare_truth.py](test_compare_truth.py)
- [test_current_geometry.py](test_current_geometry.py)
- [test_delivery_evidence.py](test_delivery_evidence.py)
- [test_development_archive.py](test_development_archive.py)
- [test_device_config_policy.py](test_device_config_policy.py)
- [test_estimation_upgrade.py](test_estimation_upgrade.py)
- [test_geometry.py](test_geometry.py)
- [test_hardware_setup.py](test_hardware_setup.py)
- 其余同类文件遵守本目录的职责和命名规则。

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

在受支持的 Linux 环境通过 `scripts/run_software_tests.py` 选择本目录的软件测试；需要原生组件的测试先完成对应构建。测试不得隐式启动真实设备。

依赖与构建方法见[项目入口](../../README.md)；返回[上级目录](../README.md)。
