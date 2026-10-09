# 任务生命周期与数据管理

采集、回放、进程、存储、配置冻结及状态路径放这里；独立算法优先进入专门功能包。

## 内容与入口

- [single_mapping_assets](single_mapping_assets/README.md)：下一级职责说明。
- [__init__.py](__init__.py)
- [calibration_geometry.py](calibration_geometry.py)
- [calibration_picker.py](calibration_picker.py)
- [capture.py](capture.py)
- [capture_audit.py](capture_audit.py)
- [capture_contract.py](capture_contract.py)
- [capture_destination.py](capture_destination.py)
- [capture_names.py](capture_names.py)
- [capture_preview.py](capture_preview.py)
- [capture_storage.py](capture_storage.py)
- [capture_support.py](capture_support.py)
- [cli.py](cli.py)
- [imu_timing_config.py](imu_timing_config.py)：选择本机时序配置；录制冻结后不再读取机器覆盖。
- [mapping_app.py](mapping_app.py)：建图命令入口与单次会话的停止保存流程。
- [mapping_interactive.py](mapping_interactive.py)：同一窗口的预览、建图切换和独立保存进程编排。
- [mapping_interactive_control.py](mapping_interactive_control.py)：本机窗口所有者、命令确认和状态代次，元数据存入内部运行目录。
- [mapping_interactive_worker.py](mapping_interactive_worker.py)：处理已正常停止的旧地图，不启动设备。
- [mapping_controller.py](mapping_controller.py)、[mapping_save.py](mapping_save.py)：真实会话的进程计划、配置冻结、停止核验与地图持久化。
- [supervisor.py](supervisor.py)：管理本会话所属组件与生命周期，不接管其他任务。
- [project_config.py](project_config.py)：统一选择当前项目配置及雷达配置目录；本机覆盖采用完整文件或完整目录，不合并字段，损坏或链接覆盖直接报错。
- [legacy_contracts.json](legacy_contracts.json)：旧档案字段的精确兼容映射，保留原始序列化标识。
- 其余同类文件遵守本目录的职责和命名规则。

## 添加与验证

当前设备配置优先读取 `config/local/project/` 下受支持的同名文件，雷达优先读取完整的 `config/local/lidar/`；没有覆盖时使用项目中的基础配置。采集冻结实际选中配置的原始字节，显式外部输入和历史录包仍使用各自配置，不隐式套用本机覆盖。

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

修改后运行[对应功能测试](../../tests/README.md)；涉及入口、配置、资源或路径时同时检查安装后的调用与旧数据读取。

建图编排和异常收尾的纯软件回归放在 [operations](../../tests/operations/README.md)，原生按钮与视口切换测试放在 [panel_rviz](../../tests/panel_rviz/README.md)。Qt/RViz 窗口实现在 [wc_bringup](../wc_bringup/README.md)；窗口控件、任务报告和运行状态不得写入本包。操作流程见[面板手册](../../docs/operations/panel_user_guide.md)。

依赖与构建方法见[项目入口](../../README.md)；返回[上级目录](../README.md)。
