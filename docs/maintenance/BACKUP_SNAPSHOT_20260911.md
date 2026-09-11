# SmartWheel 2026-09-11 清理后软件备份快照

用户授权将 `/home/nvidia/smartwheel` 当前状态备份并上传 GitHub。本快照不是稳定发行版或建图精度验收。

## 分支与范围

- 备份源分支：`local/formal-3d-hardening-20260908`。
- 快照前主树 HEAD：`103b569c0156c90bfcb6647e5da15fc12035b993`。
- 远端：`https://github.com/owenkings/smartwheel.git`。
- 上传当前分支及带日期备份标签，不覆盖 `main` 或其他开发分支，不操作另一人的 `smartwheel-rtab-old`。
- 纳入主树源码、配置、测试、启动/导出脚本、已有诊断报告和本次清理说明。
- 不纳入已删除录包、地图、操作系统、build/install、设备运行日志、缓存、用户凭据或整个 Windows 工作目录。因此这是可重建的软件快照，不是 Orin 磁盘镜像。

## 当前运行路线与未完成项

`scripts/run_wheel_imu_mapping.sh` 是目前使用的右雷达入口：轮速 vx + H30 yaw rate 经平面 EKF 提供连续位姿，右 XT-M60 原始 XYZI + RTAB-Map 提供累积地图和回环候选。FAST-LIO 代码保留，隔离旁路默认关闭。MOTION 默认 false，用户显式选择 true 时才允许进入驾驶控制流程。

已接入手推/WASD 显式模式切换及保存/重载软件流程；之前报告中的 225 项 Python 测试与 Qt 测试是历史软件验证，不代表此次进行了完整实车验收。机械手推时编码器是否保持有效、轮速尺度、最终外参、同步、回环精度和动态地图质量仍待确认。

右雷达倾角问题尚未完成本轮有效标定及新角度写入。不要将当前 provisional 配置当作已验证的角度补偿结果。FAST-LIO 严重漂移仍不能宣称全部修复。

## 原始数据已删除

用户授权清理后，主树 maps/bags 及 auto_test 中大量原始/导出数据、默认旧 RTAB 数据库已经删除，没有另行备份。历史报告中的路径可能已不存在，保留下来的报告/manifest 不代表数据包完整。后续角度标定和历史故障复查需要重新采集。

磁盘可用空间约从 1.10 GiB 增至 21.06 GiB，详见同目录 `ORIN_DISK_CLEANUP_20260911.md`。

## 被忽略的依赖源码改动

主仓库有意忽略第三方源码和 SDK，不能仅凭主树 clean 就认定依赖没有改动。本快照额外保存：

| 依赖 | 当前基准提交 | 快照补丁 |
|---|---|---|
| Ericsii/FAST_LIO_ROS2 | `916fefe8f60f9c91995923dd2213a013dcbdb21c` | `patches/snapshots/fastlio_working_tree_20260911.patch` |
| XT-Toffuture/xtsdk_py | `1770ebe479dc9bdfcdff090f9b02fd61d76ebd6a` | `patches/snapshots/xtsdk_working_tree_20260911.patch` |

`patches/snapshots/dependency_manifest_20260911.json` 保存仓库 URL、基准提交、补丁和变化文件的 SHA-256。补丁不含运行日志，也不自动写硬件。恢复时使用相应基准的干净副本，先 `git apply --check`，再应用对应快照补丁，并核对变化文件校验值。

注意：原有 `patches/apply_fastlio_patches.sh` 是另一条针对固定上游 `2fffc570...` 的完整重建路线，当前校验值与已部署关键文件相符。不能将其完整补丁与本快照增量补丁混用或叠加；应选定一条恢复路线。依赖的离线 Git bundle 另保存在 Windows，以便不依赖上游网络取得基准提交。

## 备份与恢复原则

先获取此分支/标签的主仓库，再恢复指定依赖和补丁，安装文档所需 ROS/系统依赖并构建。不要把版本恢复误当成外参已标定或实车安全已通过。不要在现有脏工作区中使用强制 reset/覆盖来恢复，优先在新目录核对。

本次主仓库 Git bundle、依赖 Git bundle 和源码检查清单保存在 Windows 的 `.work/disk_audit_20260911/`。原始设备数据已按前述授权删除，不在这些备份内。
