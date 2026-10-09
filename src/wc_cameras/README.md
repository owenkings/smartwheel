# 相机采集

设备绑定、采集节点与帧封装放这里；面板交互分别放 wc_panel 和 wc_camera_panel。

## 内容与入口

- [__init__.py](__init__.py)
- [capture.py](capture.py)
- [config.py](config.py)
- [node.py](node.py)

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

修改后运行[对应功能测试](../../tests/README.md)；涉及入口、配置、资源或路径时同时检查安装后的调用与旧数据读取。

依赖与构建方法见[项目入口](../../README.md)；返回[上级目录](../README.md)。
