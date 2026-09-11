# Orin 用户授权磁盘清理结果

时间：2026-09-11。用户明确授权删除缓存、旧日志、失败建图记录及指定的旧地图数据库和 Conda 包缓存。

## 空间变化

| 指标 | 清理前 | 清理后 |
|---|---:|---:|
| 根分区普通用户可用空间 | 1,182,535,680 字节（1.10 GiB） | 22,613,688,320 字节（21.06 GiB） |
| 根分区使用率 | 98% | 61% |

净释放约 **19.96 GiB**。这是文件系统测量值，可能随后台日志写入略有变化。

## 已清理

- 主树 `maps` 中既有会话地图、产品、录包：约 7.45 GiB。
- 主树 `bags` 中历史录包，包括之前漂移诊断数据：约 0.71 GiB。
- 主树 `auto_test` 中数据库、原始点云、二进制采样及大型导出数据：约 5.72 GiB。保留脚本、文字报告、模板和 `repair_delivery_before_20260910_JrAbTl` 代码备份等，剩余约 195 MiB。
- `/home/nvidia/.ros/rtabmap.db`：约 1.89 GiB，已删除。
- APT 下载缓存：通过 `apt-get clean` 清理，未卸载软件。
- 系统 journal：通过 rotate/vacuum 清理历史日志，目标保留约 300 MiB；保留活动日志。
- 超过 24 小时的 ROS、Trae 管理器、Kiro 服务及项目构建日志；活动文件跳过机制已启用，当前清单无跳过项。
- `/home/nvidia/.cache/pip` 中缓存文件。
- `/var/crash` 中 7 份超过 24 小时的 `.crash` 文件。
- `/home/nvidia/miniforge3/pkgs`：先执行 Conda 常规清理，再扫描 `/home/nvidia` 范围，未发现缓存目录以外的符号链接依赖该缓存，随后通过 `conda clean --force-pkgs-dirs --yes --json` 删除剩余缓存目录。没有删除 Conda 环境；硬链接共享数据仍由环境持有，不把缓存标注大小全部当成释放量。Conda 版本查询及基础 Python 的 SSL、SQLite、Conda 导入检查通过。没有逐个测试所有环境或应用。

第一阶段逐文件清单实际删除 11,687 个文件，删除前核对了绝对路径、真实路径、设备号、inode、大小、修改时间、Git 跟踪状态及本用户可见的打开文件。没有删除跟踪文件，主树 Git 工作区状态在该阶段前后完全一致。

## 保留与限制

源码、`.git`、`install`、`build`、雷达 SDK、硬件配置、测试脚本、分析报告、修复前代码备份、CUDA/ROS/NVIDIA/NoMachine、编辑器用户会话均保留。另一人的 `/home/nvidia/smartwheel-rtab-old` 未改动。

没有提交、暂存、切分支、启动建图或驱动电机。本次清理不代表角度补偿、漂移或建图质量已修复。旧文件清单和历史报告仅是历史证据索引，不再表示其原始数据仍然存在。

**原始录包和旧地图按授权直接删除，没有另做备份，不能通过本次生成的文件清单恢复。** `auto_test` 保留下来的报告/manifest 可能引用已删除数据，不再是可重放或完整可加载的产品包。后续标定和复查需重新采集。

详细删除元数据及管理命令结果仅保存在 Windows：

- `.work/disk_audit_20260911/cleanup_plan.json`
- `.work/disk_audit_20260911/cleanup_result.json`
- `.work/disk_audit_20260911/cleanup_managed_result.json`
- `.work/disk_audit_20260911/cleanup_remaining_result.json`

上述记录没有保存密码，也不是已删除数据的内容备份。
