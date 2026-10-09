# 面板后台与任务调度测试

此目录按功能维护回归测试；共享测试数据使用 tests/fixtures，正式计算实现放对应 src 功能包。

## 内容与入口

- [test_backend.py](test_backend.py)
- [test_calibration_tools.py](test_calibration_tools.py)
- [test_capture_naming.py](test_capture_naming.py)
- [test_config_profiles.py](test_config_profiles.py)
- [test_extrinsic_evidence.py](test_extrinsic_evidence.py)
- [test_installation_parameters.py](test_installation_parameters.py)
- [test_job_lifecycle.py](test_job_lifecycle.py)
- [test_offline_sensor_selection.py](test_offline_sensor_selection.py)
- [test_pairing_capture.py](test_pairing_capture.py)
- [test_parameter_panel_flow.py](test_parameter_panel_flow.py)
- [test_result_catalog.py](test_result_catalog.py)
- [test_stage_logs.py](test_stage_logs.py)
- 其余同类文件遵守本目录的职责和命名规则。

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

在受支持的 Linux 环境通过 `scripts/run_software_tests.py` 选择本目录的软件测试；需要原生组件的测试先完成对应构建。测试不得隐式启动真实设备。

依赖与构建方法见[项目入口](../../README.md)；返回[上级目录](../README.md)。

## 本机配置隔离验收

`test_private_config_overrides.py` 验证面板只写实际选中的本机副本，公共模板和冻结录包字节保持；覆盖原字节备份、多文件失败回滚、同字节来源路径变化、无效本机文件、schema 2 配置集与事务恢复，以及旧 schema 1 的受限兼容。存在来源切换或不支持的恢复状态时，必须拒绝恢复并保留记录。

这些测试使用临时工程与明确的模拟中断，不读取真实设备或启动采集。合成工程夹具只复制完整的公共配置与公共绑定，不依赖或混入运行测试机器的 `device_bindings.local.json`；需要本机覆盖的专测在临时工程内构造一致副本。真实设备的本机配置应由部署步骤在 `config/local/project/` 中完整准备。
