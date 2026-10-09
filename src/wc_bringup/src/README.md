# 程序启动与原生交互

ROS 启动描述、Qt/RViz 原生窗口及安装接线属于本包；算法实现放相应功能包。

## 内容与入口

- [manual_capture_ui.cpp](manual_capture_ui.cpp)
- [map_save_choice.cpp](map_save_choice.cpp)
- [mapping_health.cpp](mapping_health.cpp)
- [mapping_rviz.cpp](mapping_rviz.cpp)
- [mapping_teleop.cpp](mapping_teleop.cpp)
- [unified_capture_rviz.cpp](unified_capture_rviz.cpp)

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

修改后运行[对应功能测试](../../../tests/README.md)；涉及入口、配置、资源或路径时同时检查安装后的调用与旧数据读取。

依赖与构建方法见[项目入口](../../../README.md)；返回[上级目录](../README.md)。

## 目录职责与新增内容

ROS 启动描述、Qt/RViz 原生窗口及安装接线属于本包；算法实现放相应功能包。

新增文件须同步说明用途，并补充所属功能测试。返回[上级目录](../README.md)。
