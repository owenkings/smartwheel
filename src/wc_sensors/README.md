# 传感器解析与诊断

协议解析、来源校验及可复用采集诊断放这里；真实设备启动仍遵守正式入口与锁。

## 内容与入口

- [__init__.py](__init__.py)
- [capture_stability.py](capture_stability.py)
- [clock_adapter.py](clock_adapter.py)
- [h30.py](h30.py)
- [metadata.py](metadata.py)
- [pointcloud.py](pointcloud.py)
- [preflight.py](preflight.py)
- [stability.py](stability.py)

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

修改后运行[对应功能测试](../../tests/README.md)；涉及入口、配置、资源或路径时同时检查安装后的调用与旧数据读取。

依赖与构建方法见[项目入口](../../README.md)；返回[上级目录](../README.md)。
