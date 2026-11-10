# 轮反馈与手动控制边界

反馈解析、状态及明确授权的手动控制链路放这里；不增加隐式设备写入或自动运动。例外是默认关闭的自动速度通道（[速度通道设计](../../docs/architecture/velocity_channel.md)）的消息、证据格式和底盘桥：没有用户当天的同意、审过的证据、使能开关读入和驱动器参数回读就不运行，写串口的仍只有 wc_runtime 里的轮控所有者。

## 内容与入口

- [__init__.py](__init__.py)
- [auto_bridge.py](auto_bridge.py)：底盘桥，把导航域的闸门决定经 `auto.sock` 交给轮控所有者；不开串口。
- [auto_evidence.py](auto_evidence.py)：自动速度通道的证据文件格式和检查。
- [auto_protocol.py](auto_protocol.py)：闸门决定、底盘桥和轮控所有者之间的消息格式。
- [config_template.json](config_template.json)
- [driver_readback.py](driver_readback.py)：驱动器寄存器只读回读（[速度通道设计](../../docs/architecture/velocity_channel.md) 7.3 第 7 步）：只发 FC03 读请求，寄存器白名单，经轮控用的同一个串口租约独占；没有用户当天的同意不开串口。
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
