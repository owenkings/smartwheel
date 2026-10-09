# U 盘数据存储与迁移

代码仍在 `/home/nvidia/wheelchair`；长期数据统一保存在
`/media/USER/WHEELCHAIR_DATA/wheelchair`。`config/storage.local.json` 保存这台 Orin 的挂载点和文件系统 UUID，且不上传 Git。
仓库的 `config/storage.json` 是通用目录默认，其他机器不需要这只 U 盘；配置方法见 [代码与数据分离说明](storage_portability.md)。
原 `data/...`、`reports/...`、`maps/...` 参数由程序映射到 U 盘同名目录。
U 盘未挂载、换成其他盘或会话中改变挂载身份时，程序报错，不回退写内部盘。

| 目录 | 内容 |
|---|---|
| `data/experiments/<session>` | 原始录包、逐源身份、时间、配置和完整性记录 |
| `data/analysis` | 同数据融合、建图、算法比较结果 |
| `data/calibration` | 点云对应点、外参候选及标定记录 |
| `reports` | 历史测试、诊断、运行与构建报告 |
| `maps` | 地图版本 |
| `research_cache/wc_compare_20261006_metadata.tar.gz` | 旧计算缓存的参数、轨迹、日志和结论，288个文件；不包含完整点云缓存 |
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
`/media/USER/WHEELCHAIR_DATA/wheelchair/data/experiments/v7_core_20261006_130530`。
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
  /media/USER/WHEELCHAIR_DATA/wheelchair/data/analysis/v7_loop_20261006/v7_loop_factors_02/native/five_state_hz5_filter_on/export
```

## 开机自动挂载（本机配置，2026-10-07）

本机 `/etc/fstab` 已按 UUID `REPLACE-WITH-ACTUAL-UUID` 配置 `exfat-fuse`，挂载到
`/media/USER/WHEELCHAIR_DATA`。U 盘保持连接时，正常开机由 systemd 挂载，
无需每次手动输入 sudo 密码；未修改 sudoers 或 SSH 权限。

挂载使用 `nofail`，缺盘不阻塞系统启动；等待设备 10 秒，挂载命令超时 30 秒。
这不是热插拔自动重试服务。开机后才插盘或设备在等待超时后出现时，执行：

```bash
sudo mount /media/USER/WHEELCHAIR_DATA
findmnt --mountpoint /media/USER/WHEELCHAIR_DATA
```

`findmnt` 显示 `fuseblk` 是 FUSE 挂载的正常结果。录制仍要求 UUID、挂载点和读写状态正确，
不会因开机配置存在而跳过检查。没有使用 `x-systemd.automount`，以免叠加挂载影响身份检查。

本次已验证 systemd 生成单元、正常卸载后重新挂载、缺失挂载时拒绝数据访问、
普通 nvidia 用户写入/同步/读回；没有重启整机或拔盘模拟启动。
“未干净卸载”不由此配置自动修复。录制和归档结束后正常关机；拔盘前先 `sync`、成功卸载。

原始 fstab 及带哈希检查的回滚脚本位于
`/var/backups/wheelchair-usb-boot/20261007T081451484107Z/`。
现场报告位于数据根下 `reports/usb_automount_20261007/20261007T081451484107Z/`。
如需恢复原配置：

```bash
sudo python3 /var/backups/wheelchair-usb-boot/20261007T081451484107Z/rollback.py
```

该命令恢复原 fstab 并重载 systemd，保留当前已挂载的 U 盘。
systemd 对 `nofail` 和超时的定义见 [Ubuntu systemd.mount 手册](https://manpages.ubuntu.com/manpages/jammy/man5/systemd.mount.5.html)。

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
