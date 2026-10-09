# 雷达驱动

已审查 SDK 的驱动适配、队列及设备配置校验放这里；不直接修改厂商原件。

## 内容与入口

- [acquisition_budget.hpp](acquisition_budget.hpp)
- [bounded_work_queue.hpp](bounded_work_queue.hpp)
- [device_config_policy.hpp](device_config_policy.hpp)
- [host_lock.hpp](host_lock.hpp)
- [imaging_allowlist.hpp](imaging_allowlist.hpp)
- [owned_stop.hpp](owned_stop.hpp)
- [shutdown_deadline.hpp](shutdown_deadline.hpp)
- [udp_assembler.hpp](udp_assembler.hpp)
- [xtcfg.hpp](xtcfg.hpp)

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

修改后运行[对应功能测试](../../../../tests/README.md)；涉及入口、配置、资源或路径时同时检查安装后的调用与旧数据读取。

依赖与构建方法见[项目入口](../../../../README.md)；返回[上级目录](../README.md)。

## 目录职责与新增内容

已审查 SDK 的驱动适配、队列及设备配置校验放这里；不直接修改厂商原件。

新增文件须同步说明用途，并补充所属功能测试。返回[上级目录](../README.md)。
