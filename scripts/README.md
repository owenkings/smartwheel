# 命令入口

用户命令、构建和维护入口放这里；复杂计算与可复用逻辑放入 src 对应功能包。

## 内容与入口

- [check_layout.py](check_layout.py)：检查维护目录 README 覆盖和相对文档链接。
- [align_lidar_clouds.sh](align_lidar_clouds.sh)
- [build_tools_and_driver.sh](build_tools_and_driver.sh)
- [build_verify.sh](build_verify.sh)
- [capture_usb](capture_usb)
- [collect_delivery_evidence.py](collect_delivery_evidence.py)
- [configure_storage](configure_storage)
- [confirm_gyro_bias](confirm_gyro_bias)
- [dev_archive](dev_archive)
- [discover_remote.sh](discover_remote.sh)
- [map](map)
- [map_single_lidar.sh](map_single_lidar.sh)
- [panel](panel)
- 其余同类文件遵守本目录的职责和命名规则。

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

修改后运行[对应功能测试](../tests/README.md)；涉及入口、配置、资源或路径时同时检查安装后的调用与旧数据读取。

依赖与构建方法见[项目入口](../README.md)；返回[上级目录](../README.md)。
