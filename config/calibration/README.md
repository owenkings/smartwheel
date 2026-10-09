# 标定配置与来源

维护标定参数、来源索引和必要原始证据；原始文件内容和 SHA256 不因目录美化而改写。

## 内容与入口

- [imu_timing_20261008](imu_timing_20261008/README.md)：下一级职责说明。
- [sources](sources/README.md)：下一级职责说明。
- [legacy_hardware_setup_before_v7.json](legacy_hardware_setup_before_v7.json)

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

修改后运行[对应功能测试](../../tests/README.md)；涉及入口、配置、资源或路径时同时检查安装后的调用与旧数据读取。

依赖与构建方法见[项目入口](../../README.md)；返回[上级目录](../README.md)。
