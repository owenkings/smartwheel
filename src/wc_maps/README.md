# 地图数据处理

地图生成、导出及格式处理放这里；任务启停与界面保存在对应模块。

## 内容与入口

- [__init__.py](__init__.py)
- [__main__.py](__main__.py)
- [builder.py](builder.py)
- [graph_snapshot.py](graph_snapshot.py)
- [live_map.py](live_map.py)
- [ros_view.py](ros_view.py)
- [store.py](store.py)

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

修改后运行[对应功能测试](../../tests/README.md)；涉及入口、配置、资源或路径时同时检查安装后的调用与旧数据读取。

依赖与构建方法见[项目入口](../../README.md)；返回[上级目录](../README.md)。
