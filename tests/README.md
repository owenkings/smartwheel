# 功能验证

测试按被验证功能归属子目录；小型可复现输入放 fixtures，跨模块链路放 integration。正式实现不得放入测试目录。

## 内容与入口

- [cad](cad/README.md)：下一级职责说明。
- [calibration](calibration/README.md)：下一级职责说明。
- [camera_panel](camera_panel/README.md)：下一级职责说明。
- [cameras](cameras/README.md)：下一级职责说明。
- [diagnostics](diagnostics/README.md)：下一级职责说明。
- [fixtures](fixtures/README.md)：下一级职责说明。
- [fusion](fusion/README.md)：下一级职责说明。
- [imu](imu/README.md)：下一级职责说明。
- [imu_timing_capture](imu_timing_capture/README.md)：下一级职责说明。
- [imu_timing_replay](imu_timing_replay/README.md)：下一级职责说明。
- [integration](integration/README.md)：下一级职责说明。
- [maps](maps/README.md)：下一级职责说明。
- [motion](motion/README.md)：下一级职责说明。
- [official_motion_adapter](official_motion_adapter/README.md)：下一级职责说明。
- [operations](operations/README.md)：下一级职责说明。
- [panel_backend](panel_backend/README.md)：下一级职责说明。
- [panel_capture](panel_capture/README.md)：下一级职责说明。
- [panel_live_algorithms](panel_live_algorithms/README.md)：下一级职责说明。
- [panel_runtime](panel_runtime/README.md)：下一级职责说明。
- [panel_rviz](panel_rviz/README.md)：下一级职责说明。
- [panel_ui](panel_ui/README.md)：下一级职责说明。
- [portability](portability/README.md)：下一级职责说明。
- [runtime](runtime/README.md)：下一级职责说明。
- [sensors](sensors/README.md)：下一级职责说明。
- [single_lidar_offline](single_lidar_offline/README.md)：下一级职责说明。
- [slam](slam/README.md)：下一级职责说明。
- [teleop_panel](teleop_panel/README.md)：下一级职责说明。
- [timing](timing/README.md)：下一级职责说明。
- [unit](unit/README.md)：下一级职责说明。
- [usb_runtime](usb_runtime/README.md)：下一级职责说明。
- [usb_storage](usb_storage/README.md)：下一级职责说明。
- [run_target_tests.sh](run_target_tests.sh)

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

在受支持的 Linux 环境通过 `scripts/run_software_tests.py` 选择本目录的软件测试；需要原生组件的测试先完成对应构建。测试不得隐式启动真实设备。

依赖与构建方法见[项目入口](../README.md)；返回[上级目录](../README.md)。
