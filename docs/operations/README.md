# 功能操作手册

按用户实际功能维护入口、输入、步骤、保存位置与停止方式；历史实验过程进入开发归档。

## 内容与入口

- [amplitude_alignment.md](amplitude_alignment.md)
- [capture.md](capture.md)
- [command_reference.md](command_reference.md)
- [field_extrinsics.md](field_extrinsics.md)：现场双雷达外参与轮速标定（当前方法）
- [lidar_plane_workbench.md](lidar_plane_workbench.md)
- [hardware_setup_zh.md](hardware_setup_zh.md)
- [mapping_app_zh.md](mapping_app_zh.md)
- [offline_cloud_alignment.md](offline_cloud_alignment.md)
- [offline_motion_compare.md](offline_motion_compare.md)
- [orin_commands_zh.md](orin_commands_zh.md)
- [panel_user_guide.md](panel_user_guide.md)
- [runtime_limits_zh.md](runtime_limits_zh.md)
- [single_lidar_mapping_zh.md](single_lidar_mapping_zh.md)
- [storage.md](storage.md)

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

修改后运行[对应功能测试](../../tests/README.md)；涉及入口、配置、资源或路径时同时检查安装后的调用与旧数据读取。

依赖与构建方法见[项目入口](../../README.md)；返回[上级目录](../README.md)。

- [离线导航开发](navigation_offline.md)：保存地图规划、数值跟踪及远程工作顺序。
- [自主导航仿真](navigation_autonomy.md)：仿真轮椅上的自主探索建图、AMCL 目标导航与安全闸门闭环，以及真实录包上的 Nav2 影子运行。
- [规划与控制](navigation_planning.md)：车体外形（仿真碰撞、代价地图、碰撞监控同一外形）、可选规划器与控制器。
- [实车影子链路](navigation_live_shadow.md)：跨录包 AMCL 检查、影子运行的定位模式、实时链路与上车前检查单。
