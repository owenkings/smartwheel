# 可视化配置

ROS/RViz 显示配置放这里；新增显示项必须核对话题、坐标系与启动入口。

## 内容与入口

- [cameras.rviz](cameras.rviz)
- [mapping_2d.rviz](mapping_2d.rviz)
- [mapping_3d.rviz](mapping_3d.rviz)
- [mapping_live.rviz](mapping_live.rviz)
- [sensor_left.rviz](sensor_left.rviz)
- [sensor_right.rviz](sensor_right.rviz)

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

修改后运行[对应功能测试](../../tests/README.md)；涉及入口、配置、资源或路径时同时检查安装后的调用与旧数据读取。

依赖与构建方法见[项目入口](../../README.md)；返回[上级目录](../README.md)。
