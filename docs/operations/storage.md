# 数据存储与迁移

源码和配置留在项目目录，录包、地图、派生分析与开发归档进入配置的数据根。本机位置由忽略的 `config/storage.local.json` 保存，完整覆盖通用 `config/storage.json`。默认可以使用普通目录；外置盘模式另外核验挂载点和文件系统 UUID。

## 配置与入口

在项目根目录设置普通数据目录：

```bash
python3 scripts/configure_storage --backend directory --archive-root "$HOME/wheelchair-data"
```

需要外置盘时，先自行挂载并用 `lsblk -f` 核对真实设备，再填写：

```bash
python3 scripts/configure_storage --backend removable \
  --archive-root /实际挂载目录/wheelchair \
  --mount-point /实际挂载目录 \
  --required-uuid 实际文件系统UUID
```

该入口不格式化、挂载或自动挑选磁盘。掉盘、挂载身份改变或配置不完整时拒绝写入，不回退到内部同名目录。面板“数据与存储”中的默认录制/融合目录按同样的目标检查规则使用；修改默认值不搬迁已有数据。

## 目录职责

| 数据根内位置 | 内容 |
|---|---|
| `data/experiments/<session>` | 原始录包、逐源身份、冻结配置与完整性记录。 |
| `data/analysis/` | 基于同一输入的融合、建图、算法比较结果。 |
| `data/calibration/` | 点云对应点、候选变换及标定记录。 |
| `reports/` | 运行诊断、生产分析和地图工作会话。 |
| `maps/` | 带版本和完整性清单的地图包。 |
| `dev_archive/` | 开发任务、测试/构建结果、历史报告、快照及迁移清单。 |

锁、进程登记、用户输入索引和 Unix socket 保留在本机 `.phase1_runtime/`，不放到 exFAT 等外置归档文件系统。内存暂存和运行目录中的日志尚未等于完成持久保存。

查看当前数据位置：

```bash
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
WC_DATA="$(dirname -- "$(python3 -m wc_runtime.storage_policy --project-root "$PWD" data)")"
xdg-open "$WC_DATA"
du -h --max-depth=1 "$WC_DATA"
```

旧逻辑参数 `data/...`、`reports/...`、`maps/...` 由支持该约定的入口解析到配置数据根。外部文件或手工 Python 读文件不会自动解析这些逻辑路径；应使用对应命令的路径约定。

## 录制、停止与核验

在设备配置正确的图形桌面终端执行下列命令会连接传感器；`--manual-drive` 另外明确启用本次人工驾驶：

```bash
SESSION="capture_$(date +%Y%m%d_%H%M%S)"
python3 scripts/wc_phase1 capture \
  --profile mapping_core --session "$SESSION" --duration 300 --preview
```

`mapping_core` 不启动相机和超声波。`--duration 0` 等待正常停止，正数为录制时长上限。提前结束使用 RViz“结束录制”、正常关闭窗口或一次 Ctrl+C，再等待停源、转存和核验。默认先存内存；外置 exFAT 上的人工驾驶录制保留 `--staging memory`，容量同时受内存和磁盘限制。

```bash
python3 scripts/wc_phase1 doctor \
  --session-root "$WC_DATA/data/experiments/$SESSION" --verify-archive
```

以上路径用于默认命名；有自定义目录名或同名避让时使用程序最终返回的实际路径。短于上限不会补成足时录包，`PARTIAL` 保持真实缺失信息。

## 已有结果与移动数据

用面板“结果对比”或 `python3 scripts/view_map /实际地图目录` 查看地图；用“设备参数 → 点云配对”准备和选择点云。已有 prepared 描述时：

```bash
bash scripts/align_lidar_clouds.sh --input /实际路径/prepared.json
```

移动前正常结束使用这些文件的采集、融合、地图查看和网页服务。原始录包及证据保持原字节，先复制并核对清单/哈希，再处理原位置。部分结果仍引用原始录包，只保存 PLY 不能恢复全部重算输入。开发资料通过 `python3 scripts/dev_archive 任务名称` 建立归档，不散放源码目录。

## 本机自动挂载与故障处理

Git clone 不会配置系统 `/etc/fstab`、udev 或网络。需要开机自动挂载时由本机管理员按真实 UUID 和目标文件系统设置；不要复制其他机器的挂载身份。系统显示挂载成功后，仍需项目的 UUID、读写和空间检查通过。

任务结束且数据保存完成后才可以卸载外置盘；正常同步和卸载完成后再拔盘。空间不足或保存失败时保留会话与日志，先查具体错误，不批量删除 `PARTIAL` 录包。

具体字段、暂存布局和边界见[存储配置参考](../reference/storage.md)，完整录制/融合操作见[命令手册](command_reference.md)，首次部署见[根 README](../../README.md)。
