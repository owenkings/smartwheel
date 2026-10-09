# 接口与数据契约

配置字段、坐标、时间、数据格式和接口约束放这里。

## 内容与入口

- [calibration_configuration.md](calibration_configuration.md)
- [geometry.md](geometry.md)
- [imu_timing.md](imu_timing.md)
- [interface_contract.md](interface_contract.md)
- [offline_contract.md](offline_contract.md)
- [panel_backend_api.md](panel_backend_api.md)
- [recording_contract.md](recording_contract.md)
- [storage.md](storage.md)

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

修改后运行[对应功能测试](../../tests/README.md)；涉及入口、配置、资源或路径时同时检查安装后的调用与旧数据读取。

依赖与构建方法见[项目入口](../../README.md)；返回[上级目录](../README.md)。
