# 采集、回放与进程管理测试

此目录按功能维护回归测试；共享测试数据使用 tests/fixtures，正式计算实现放对应 src 功能包。

## 内容与入口

- [test_archive_source_paths.py](test_archive_source_paths.py)
- [test_calibration_picker_input.py](test_calibration_picker_input.py)
- [test_camera_archive_compression.py](test_camera_archive_compression.py)
- [test_camera_archive_flush.py](test_camera_archive_flush.py)
- [test_capture.py](test_capture.py)
- [test_capture_audit_contract.py](test_capture_audit_contract.py)
- [test_capture_contract.py](test_capture_contract.py)
- [test_capture_destination.py](test_capture_destination.py)
- [test_capture_manual_drive.py](test_capture_manual_drive.py)
- [test_capture_memory_staging.py](test_capture_memory_staging.py)
- [test_capture_preview.py](test_capture_preview.py)
- [test_capture_receipt_readiness.py](test_capture_receipt_readiness.py)
- [test_project_config.py](test_project_config.py)：验证完整本机覆盖、无效覆盖拒绝、雷达目录完整性和录包原始字节冻结；不打开设备。
- 其余同类文件遵守本目录的职责和命名规则。

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

在受支持的 Linux 环境通过 `scripts/run_software_tests.py` 选择本目录的软件测试；需要原生组件的测试先完成对应构建。测试不得隐式启动真实设备。

配置选择与冻结可单独运行 `bash tests/run_target_tests.sh tests/runtime/test_project_config.py`；采集、建图和命令构造集成检查需要 Linux，符号链接检查需要文件系统允许创建链接。

依赖与构建方法见[项目入口](../../README.md)；返回[上级目录](../README.md)。
