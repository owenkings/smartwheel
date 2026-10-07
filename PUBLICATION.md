# 公共源码与本机配置

本分支是 Orin V7 当前工程的独立源码快照。它不携带 Orin 的原始 `.git` 历史、录包、室内地图、相机画面、测试输出或历史报告。原工作树、现场配置和 Git 历史由项目所有者在 USB 上保留；GitHub 的 `main` 分支不因此改变。

## 配置模板

公开文件中的雷达、IMU、轮控制适配器、摄像头序列号已替换为结构兼容的示例值。公共存储默认是独立目录；可拔插盘配置示例使用显式占位符，实际 UUID 只写入被忽略的本机覆盖。`WHEELCHAIR_DATA` 仅为历史文档中的挂载标签示例。不要直接用这些值操作真实设备。

接入本机设备前，应核实并统一替换 `config/` 中的身份，以及源码/脚本中相同的示例绑定。当前工程仍有部分身份校验常量：主要检查 `src/wc_runtime/cli.py`、`src/wc_cameras/config.py`、录制存储配置和入口脚本；不要只改 JSON 后关闭校验。如果本机明确选择可拔插盘后端，拔出或身份不符时应停止写入，不回退到同名本机目录。使用固定目录后端不要求 U 盘。详见 docs/storage_portability.md。

现场设备绑定及用户测量原件在 Orin/USB 上保留原字节。公开机械几何数值保留；若公开副本中的账户路径或设备身份发生脱敏，导出程序会明确更新该副本的 `source_documents[].sha256` 引用，且在 `publication_manifest.json` 同时记录源文件 SHA-256、发布文件 SHA-256 和修改类别。没有把脱敏副本声称为原件，也没有随意更新未变化的 SDK/补丁哈希。

`publication_manifest.json` 是导出时的源文件关联清单。`PREPARED_NOT_PUSHED` 表示导出阶段的事实；后续 Git 提交与分支本身记录是否完成上传，不能把该字段理解为运行验收。

## 依赖与许可

目标环境为 Linux aarch64、ROS 2 Humble、项目兼容版本的 RTAB-Map、robot_localization、Qt/RViz，以及 NumPy/SciPy/OpenCV/Python 串口组件。构建使用项目内 CMake；厂商程序和 SDK 安装脚本不自动运行。

厂商 SDK 主体通过固定版本 clone 恢复，不重复上传。`SDKs/xtsdk_filter_3d3db067/` 仅包含本次明确要求保留的两份原字节过滤库及来源说明；项目滤波参数、机械配置和 SDK 补丁随代码保留。其他 SDK、OpenCV 提取目录和安装产物不上传。版本、哈希与恢复步骤见 [vendor_patches/README.md](vendor_patches/README.md)、过滤库 [NOTICE](SDKs/xtsdk_filter_3d3db067/NOTICE.md) 和 [部署指南](docs/deployment.md)。

保留源码和 `package.xml` 中已有许可声明。本次发布不为未知来源代码或厂商二进制添加新许可，不宣称整个仓库或所有依赖统一采用某一许可证。公开可读不等于任意第三方依赖均获再分发许可。

## 本地验证与公开验证

硬件运行、真实录包和 GUI 验证在 Orin 上进行。公开分支经过脱敏，不能继承本机配置的硬件验证结论。测试源码保留；依赖本机现场资料的测试需要项目所有者的私有资料，普通合成回归不应读取真实设备。
