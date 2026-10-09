# 配置与标定依据

通用默认值和模板按功能归类；真实机器覆盖使用被忽略的 *.local.json 或 local 子目录。

## 内容与入口

- [calibration](calibration/README.md)：下一级职责说明。
- [comparison_20260917_handpush](comparison_20260917_handpush/README.md)：下一级职责说明。
- [rviz](rviz/README.md)：下一级职责说明。
- [cameras.json](cameras.json)
- [device_bindings.json](device_bindings.json)
- `device_bindings.local.json`：本机设备绑定覆盖，不随仓库提供。
- [hardware_setup.json](hardware_setup.json)
- [imu_timing.json](imu_timing.json)
- `imu_timing.local.json`：本机 IMU 时序配置及证据路径，不随仓库提供。
- [live_unvalidated.json](live_unvalidated.json)
- [mapping_3d_diagnostic.json](mapping_3d_diagnostic.json)
- [mapping_live.json](mapping_live.json)
- [quality_experimental.json](quality_experimental.json)
- [storage.json](storage.json)
- `storage.local.json`：本机数据根目录与可移除介质身份，不随仓库提供。
- 其余同类文件遵守本目录的职责和命名规则。

## 本机覆盖和任务冻结

- `device_bindings.local.json`、`storage.local.json`、`imu_timing.local.json` 分别完整覆盖设备绑定、存储及时序策略。
- `local/project/` 按基础配置的相对路径保存完整机器副本，支持相机、轮反馈、在线设备身份、安装几何及建图配置。固定支持列表由 [project_config.py](../src/wc_runtime/project_config.py) 定义。
- `local/lidar/` 保存完整雷达 YAML 和 xtcfg 输入，不能只复制单侧或混合多个来源。
- `local/imu_timing/` 保存实际时序证据的原字节和校验。公共示例证据不能改写为现场证据。
- `panel_profiles/`、`calibration/panel_extrinsics/`、`calibration/panel_live/` 是现场生成的参数档案与证据，忽略且不安装；需私下备份。

这些位置都不上传 Git。文件存在但损坏、被替换为链接或内容不完整时拒绝加载。没有本机副本的项目文件仍使用基础配置；因此先按[部署步骤](../README.md#5-配置本机设备保留公开模板)创建副本，再通过面板修改设备参数。

正常采集会冻结所选配置原字节及证据；离线估计和历史录包不读取当前机器覆盖。显式输入外部配置的工具继续使用指定文件。

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

修改后运行[对应功能测试](../tests/README.md)；涉及入口、配置、资源或路径时同时检查安装后的调用与旧数据读取。

依赖与构建方法见[项目入口](../README.md)；返回[上级目录](../README.md)。
