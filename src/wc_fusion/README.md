# 时空融合

多源时间、坐标与数据融合逻辑放这里；界面与录制管理不放这里。

## 内容与入口

- [__init__.py](__init__.py)
- [archive.py](archive.py)
- [cache.py](cache.py)
- [core.py](core.py)
- [graph_delivery.py](graph_delivery.py)
- [ros_node.py](ros_node.py)
- [source_join.py](source_join.py)
- [wheel_prior.py](wheel_prior.py)
- [wheel_tf.py](wheel_tf.py)

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

修改后运行[对应功能测试](../../tests/README.md)；涉及入口、配置、资源或路径时同时检查安装后的调用与旧数据读取。

依赖与构建方法见[项目入口](../../README.md)；返回[上级目录](../README.md)。
