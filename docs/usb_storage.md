# U 盘数据存储与迁移

代码仍在 `/home/nvidia/wheelchair`；长期数据统一保存在
`/media/nvidia/WHEELCHAIR_DATA/wheelchair`。`config/storage.json` 固定挂载点和文件系统 UUID。
原 `data/...`、`reports/...`、`maps/...` 参数由程序映射到 U 盘同名目录。
U 盘未挂载、换成其他盘或会话中改变挂载身份时，程序报错，不回退写内部盘。

| 目录 | 内容 |
|---|---|
| `data/experiments/<session>` | 原始录包、逐源身份、时间、配置和完整性记录 |
| `data/analysis` | 同数据融合、建图、算法比较结果 |
| `data/calibration` | 点云对应点、外参候选及标定记录 |
| `reports` | 历史测试、诊断、运行与构建报告 |
| `maps` | 地图版本 |
| `research_cache/wc_compare_20261006.tar.gz` | 本次内存研究缓存的无损归档，可解包恢复 |
| `migration_20261006` | 迁移清单、SHA-256、原位置删除记录及回退源码/Git |
| `docs/history`、`handoffs`、`log` | 过往验收说明、交接记录和构建日志 |

锁、进程登记、Unix 控制 socket 留在工程的 `.phase1_runtime`。
采集仍可先暂存内存，雷达临时源日志也由运行目录管理，收尾进入 U 盘档案。
这些临时运行状态不表示数据已经持久保存。不要直接删除正在运行的会话或拔盘。

## 录制

在 Orin 图形桌面终端执行，无需先运行 `--dry-run`：

```bash
cd /home/nvidia/wheelchair
SESSION="v7_core_$(date +%Y%m%d_%H%M%S)"
python3 scripts/wc_phase1 capture \
  --profile mapping_core --session "$SESSION" --duration 300 \
  --manual-drive --preview
```

此配置记录左右雷达各自的 raw/filtered 点云、IMU、轮反馈及关联证据，不启动相机和超声波。
300 秒为上限。提前结束按一次 Ctrl+C，等待停源、转存和核验结束再关闭终端。
短于上限不会自动补成 300 秒；档案保留实际时长和原始完整性状态。

```bash
python3 scripts/wc_phase1 doctor \
  --session-root "data/experiments/$SESSION" --verify-archive
```

也可给 `--session-root` U 盘中的绝对目录。

## 已有数据与左右点云网页

原实验 `v7_core_20261006_130530` 位于
`/media/nvidia/WHEELCHAIR_DATA/wheelchair/data/experiments/v7_core_20261006_130530`。
既有融合地图位于 `data/analysis/v7_loop_20261006`，原档案内部内容及哈希保持不变。

```bash
cd /home/nvidia/wheelchair
bash scripts/align_lidar_clouds.sh \
  --input reports/v7_recording_20261006/overlap_web/prepared.json \
  --scene-index 0 --duration 7200 --no-browser
```

在 Orin 浏览器打开 `http://127.0.0.1:8767/alignment`。旧逻辑路径自动映射到 U 盘。
网页生成的对应点和候选也保存到 U 盘 `data/calibration`。

查看既有 3D 地图：

```bash
python3 scripts/view_map \
  /media/nvidia/WHEELCHAIR_DATA/wheelchair/data/analysis/v7_loop_20261006/v7_loop_factors_02/native/five_state_hz5_filter_on/export
```

## exFAT 与恢复边界

本 U 盘为 exFAT，不支持 Unix socket、符号链接、硬链接和内核原子无覆盖重命名。
旧日志的 85 个符号链接保存在 `migration_20261006/original_symlinks.tar`，
每个原路径和链接目标另见 `manifest.json`；普通数据文件保留原目录结构。
需要完整恢复这些链接时，将资料复制到支持符号链接的 Linux 文件系统后，
检查 tar 成员，再在该恢复根目录解包。不要把符号链接恢复到 exFAT。

地图/快照提交在该盘上使用本程序共享的内部文件锁协调，提交前重新检查目的盘和目标不存在。
这防止本程序并发写同名结果；不具有 Linux `RENAME_NOREPLACE` 对绕过锁的外部写入者的原子保证。
保存过程中不要用其他程序创建或替换同名目标。普通文件及 SQLite、目录同步另有现场探针记录。

回归测试源码 `tests/`、当前使用说明和配置来源继续保留在工程；迁移的“测试文件夹”指历史运行产物。
GitHub 新分支是代码发布快照，设备绑定做了模板化；它不能直接覆盖这台 Orin 的实际配置。
