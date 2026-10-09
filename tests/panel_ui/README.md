# 窗口交互与结果显示测试

此目录按功能维护回归测试；共享测试数据使用 tests/fixtures，正式计算实现放对应 src 功能包。

## 内容与入口

- [render_screenshots.py](render_screenshots.py)
- [test_capture_naming_ui.py](test_capture_naming_ui.py)
- [test_device_flow.py](test_device_flow.py)
- [test_device_overview.py](test_device_overview.py)
- [test_installation_ui.py](test_installation_ui.py)
- [test_offline_sources_ui.py](test_offline_sources_ui.py)
- [test_pairing_capture_ui.py](test_pairing_capture_ui.py)
- [test_pairing_name_options.py](test_pairing_name_options.py)
- [test_panel_ui.py](test_panel_ui.py)
- [test_pcd_lzf.py](test_pcd_lzf.py)
- [test_popup_controls.py](test_popup_controls.py)
- [test_profiles_ui.py](test_profiles_ui.py)
- 其余同类文件遵守本目录的职责和命名规则。

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

在受支持的 Linux 环境通过 `scripts/run_software_tests.py` 选择本目录的软件测试；需要原生组件的测试先完成对应构建。测试不得隐式启动真实设备。

依赖与构建方法见[项目入口](../../README.md)；返回[上级目录](../README.md)。
