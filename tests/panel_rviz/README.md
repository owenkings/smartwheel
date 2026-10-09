# 统一原生可视化测试

此目录按功能维护回归测试；共享测试数据使用 tests/fixtures，正式计算实现放对应 src 功能包。

## 内容与入口

- [run_synthetic.py](run_synthetic.py)
- [test_rviz_lifecycle.cpp](test_rviz_lifecycle.cpp)
- [test_save_flow.cpp](test_save_flow.cpp)
- [test_mapping_interactive_control.cpp](test_mapping_interactive_control.cpp)：建图按钮、所属窗口身份、状态代次、命令确认及非法状态拒绝。

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

本目录的 C++ 测试由 [wc_bringup/CMakeLists.txt](../../src/wc_bringup/CMakeLists.txt) 在 `BUILD_TESTING=ON` 时构建；加载 ROS 及本项目安装环境后使用 `colcon test --build-base build/main --install-base install/main --packages-select wc_bringup --return-code-on-test-failure`。图形生命周期验证使用合成数据和独立显示环境。测试使用独立 ROS domain，禁止隐式启动真实设备。

Python 建图会话生命周期与保存进程回归放在 [operations](../operations/README.md)；WASD 输入和轮控制状态回归放在 [teleop_panel](../teleop_panel/README.md)。新的 RViz 会话切换、视口生命周期和建图按钮测试放入本目录；正式实现放在 [wc_bringup](../../src/wc_bringup/README.md) 和 [wc_runtime](../../src/wc_runtime/README.md)。

依赖与构建方法见[项目入口](../../README.md)；返回[上级目录](../README.md)。
