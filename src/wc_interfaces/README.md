# ROS 消息与服务

公共消息放 msg，服务放 srv；变更字段必须同步生产者、消费者和契约测试。

## 内容与入口

- [msg](msg/README.md)：下一级职责说明。
- [srv](srv/README.md)：下一级职责说明。
- [CMakeLists.txt](CMakeLists.txt)
- [package.xml](package.xml)

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

修改后运行[对应功能测试](../../tests/README.md)；涉及入口、配置、资源或路径时同时检查安装后的调用与旧数据读取。

依赖与构建方法见[项目入口](../../README.md)；返回[上级目录](../README.md)。
