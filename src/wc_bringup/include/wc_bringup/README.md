# 程序启动与原生交互

ROS 启动描述、Qt/RViz 原生窗口及安装接线属于本包；算法实现放相应功能包。

## 内容与入口

- [grid_view_fit.hpp](grid_view_fit.hpp)
- [mapping_health.hpp](mapping_health.hpp)
- [mapping_interactive_control.hpp](mapping_interactive_control.hpp)：本窗口所有者、状态代次和原子命令校验，以及开始／停止建图按钮。
- [mapping_session_view.hpp](mapping_session_view.hpp)：切换会话时重建三维显示、二维视口、TF、健康状态和手动交互绑定。
- [mapping_render_diagnostics.hpp](mapping_render_diagnostics.hpp)
- [mapping_teleop.hpp](mapping_teleop.hpp)
- [unified_rviz.hpp](unified_rviz.hpp)

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

修改后运行[对应功能测试](../../../../tests/README.md)；涉及入口、配置、资源或路径时同时检查安装后的调用与旧数据读取。

建图窗口协议对应 [panel_rviz](../../../../tests/panel_rviz/README.md) 的原生测试；轮控制输入对应 [teleop_panel](../../../../tests/teleop_panel/README.md)。窗口类仅负责显示和向已核验的所属进程提出请求，设备采集与建图生命周期实现放在 [wc_runtime](../../../wc_runtime/README.md)，算法放所属功能包。

依赖与构建方法见[项目入口](../../../../README.md)；返回[上级目录](../README.md)。

## 目录职责与新增内容

ROS 启动描述、Qt/RViz 原生窗口及安装接线属于本包；算法实现放相应功能包。

新增文件须同步说明用途，并补充所属功能测试。返回[上级目录](../README.md)。
