# 任务生命周期与数据管理

采集、回放、进程、存储、配置冻结及状态路径放这里；独立算法优先进入专门功能包。

## 内容与入口

- [offline_compaction.py](offline_compaction.py)：完成离线地图的逐图无损点云压缩、派生缓存精简、空间预检和中断恢复；保留生成证据及独立保留清单。

- [single_mapping_assets](single_mapping_assets/README.md)：下一级职责说明。
- [__init__.py](__init__.py)
- [calibration_geometry.py](calibration_geometry.py)
- [calibration_picker.py](calibration_picker.py)
- [capture.py](capture.py)
- [capture_audit.py](capture_audit.py)
- [capture_contract.py](capture_contract.py)
- [capture_destination.py](capture_destination.py)
- [capture_names.py](capture_names.py)
- [capture_preview.py](capture_preview.py)
- [capture_storage.py](capture_storage.py)
- [capture_support.py](capture_support.py)
- [cli.py](cli.py)
- [imu_timing_config.py](imu_timing_config.py)：选择本机时序配置；录制冻结后不再读取机器覆盖。
- [mapping_app.py](mapping_app.py)：建图命令入口与单次会话的停止保存流程。
- [mapping_interactive.py](mapping_interactive.py)：同一窗口的预览、建图切换和独立保存进程编排。
- [mapping_interactive_control.py](mapping_interactive_control.py)：本机窗口所有者、命令确认和状态代次，元数据存入内部运行目录。
- [mapping_interactive_worker.py](mapping_interactive_worker.py)：处理已正常停止的旧地图，不启动设备。
- [mapping_controller.py](mapping_controller.py)、[mapping_save.py](mapping_save.py)：真实会话的进程计划、配置冻结、停止核验与地图持久化。
- [live_experiment.py](live_experiment.py)：显式实时实验方案的只读能力检查、机械初值和固定运动候选复用；保留原始未标定状态与来源。
- [live_motion_options.py](live_motion_options.py)：严格设备校正与实验候选分别校验；因果几何修正只访问当前已收到的扫描。
- [supervisor.py](supervisor.py)：管理本会话所属组件与生命周期，不接管其他任务。
- [project_config.py](project_config.py)：统一选择当前项目配置及雷达配置目录；本机覆盖采用完整文件或完整目录，不合并字段，损坏或链接覆盖直接报错。
- [legacy_contracts.json](legacy_contracts.json)：旧档案字段的精确兼容映射，保留原始序列化标识。
- 其余同类文件遵守本目录的职责和命名规则。

## 添加与验证

当前设备配置优先读取 `config/local/project/` 下受支持的同名文件，雷达优先读取完整的 `config/local/lidar/`；没有覆盖时使用项目中的基础配置。采集冻结实际选中配置的原始字节，显式外部输入和历史录包仍使用各自配置，不隐式套用本机覆盖。

实时入口默认 `--scheme validated`，沿用数据外参与独立设备校正声明的严格检查。显式选择 `--scheme experiment` 后，读取 `live_experiment_profile_config`（默认 `config/calibration/live_experiment_profile.json`），核对当前装配、硬件安装内容哈希、IMU/轮身份、原始离线候选 SHA-256、候选内容摘要、IMU 旋转及轮转换参数，再以机械面中心作为雷达数据原点初值。原始硬件配置的 `UNKNOWN`、原候选的离线范围与未标定状态保留；此方案不能叠加独立零偏或设备校正声明，不能用于导航或精度验收。

实验运动修正只复用固定原生轴零偏、轮偏航比例和速度系数，零偏减一次后执行安装旋转；后台候选统计不更新这些固定值。新会话保持实时到达时间与 5 Hz 上限，不使用原录包的时间窗、设备相对时间模型或未来时间锚点。会话冻结 profile、原始候选原字节及来源 SHA；原录包的消息组合摘要按原定义保留，不在每次实时能力查询中重读整个录包。几何反馈适用于五状态 EKF，校正当前发布位姿并保留未修正先验，不把扫描配准重复当作 EKF 测量；官方 EKF 的几何反馈仍不可选。

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

修改后运行[对应功能测试](../../tests/README.md)；涉及入口、配置、资源或路径时同时检查安装后的调用与旧数据读取。

建图编排和异常收尾的纯软件回归放在 [operations](../../tests/operations/README.md)，原生按钮与视口切换测试放在 [panel_rviz](../../tests/panel_rviz/README.md)。Qt/RViz 窗口实现在 [wc_bringup](../wc_bringup/README.md)；窗口控件、任务报告和运行状态不得写入本包。操作流程见[面板手册](../../docs/operations/panel_user_guide.md)。

依赖与构建方法见[项目入口](../../README.md)；返回[上级目录](../README.md)。
