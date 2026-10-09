# 换机部署与路径独立性测试

此目录按功能维护回归测试；共享测试数据使用 tests/fixtures，正式计算实现放对应 src 功能包。

## 内容与入口

- [test_cli_binding_errors.py](test_cli_binding_errors.py)
- [test_device_bindings.py](test_device_bindings.py)
- [test_entrypoints.py](test_entrypoints.py)
- [test_host_locks.py](test_host_locks.py)
- [test_project_paths.py](test_project_paths.py)

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

在受支持的 Linux 环境通过 `scripts/run_software_tests.py` 选择本目录的软件测试；需要原生组件的测试先完成对应构建。测试不得隐式启动真实设备。

依赖与构建方法见[项目入口](../../README.md)；返回[上级目录](../README.md)。
