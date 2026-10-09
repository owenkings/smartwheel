# 轮反馈与手动控制边界

反馈解析、状态及明确授权的手动控制链路放这里；不增加隐式设备写入或自动运动。

## 内容与入口

- [__init__.py](__init__.py)
- [config_template.json](config_template.json)
- [feedback_transport.py](feedback_transport.py)
- [history_preview.py](history_preview.py)
- [manual_hardware.py](manual_hardware.py)
- [manual_teleop.py](manual_teleop.py)
- [model.py](model.py)
- [passive_serial.py](passive_serial.py)
- [protocol.py](protocol.py)
- [replay.py](replay.py)
- [ros_node.py](ros_node.py)

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

修改后运行[对应功能测试](../../tests/README.md)；涉及入口、配置、资源或路径时同时检查安装后的调用与旧数据读取。

依赖与构建方法见[项目入口](../../README.md)；返回[上级目录](../README.md)。
