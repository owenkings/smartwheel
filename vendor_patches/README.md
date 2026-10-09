# 依赖补丁与来源

仅保存审查过的补丁、来源清单和适用版本；完整厂商目录及构建产物不放在这里。

## 内容与入口

- [xtcfg_deploy_files.json](xtcfg_deploy_files.json)
- [xtcfg_handoff.md](xtcfg_handoff.md)
- [xtsdk_filter_3d3db067.manifest.json](xtsdk_filter_3d3db067.manifest.json)
- [xtsdk_ros_965d31a.manifest.json](xtsdk_ros_965d31a.manifest.json)
- [xtsdk_ros_965d31a.patch](xtsdk_ros_965d31a.patch)

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

修改后运行[对应功能测试](../tests/README.md)；涉及入口、配置、资源或路径时同时检查安装后的调用与旧数据读取。

依赖与构建方法见[项目入口](../README.md)；返回[上级目录](../README.md)。
