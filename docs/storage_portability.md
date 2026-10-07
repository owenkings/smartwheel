# 代码、数据目录与换机

## 1. 默认使用普通本机目录

代码目录保存维护中的 `src/`、`scripts/`、`config/`、`tests/`、`docs/` 和补丁。`tests/` 是回归测试源码，不是应该随测试清理的产物。依赖与构建位置 `SDKs/`、`build/`、`install/` 留在普通本机 Linux 文件系统；仅已声明的固定过滤库随 Git 发布，其余依赖与构建产物换机重新准备。

录包、地图、融合结果、诊断报告、测试日志及缓存写入独立数据根，默认 `~/wheelchair-data`。工程可 clone 到其他目录或账号下，主机名无需保持相同。已有档案不因修改存储配置自动迁移或重写。

```bash
# 在已 clone 的工程根运行；无需任何 U 盘。
python3 scripts/configure_storage \
  --backend directory --archive-root "$HOME/wheelchair-data"
```

这只选择保存位置，不安装 ROS、SDK 或设备驱动。完整部署见 [部署指南](deployment.md)。内部 SSD 上的其他普通目录同样使用 directory 后端。

## 2. 配置优先级

| 文件 | 用途 | Git |
|---|---|---|
| `config/storage.json` | schema 2 通用 directory 默认 | 保留 |
| `config/storage.local.json` | 本机完整覆盖 | 忽略，私下备份 |
| `config/storage.removable.example.json` | 可选外置盘格式示例 | 保留 |
| `config/device_bindings.local.json` | 本机 IMU 和可选网卡覆盖，与存储独立 | 忽略，私下备份 |

local 文件存在时完整替换通用配置，不逐字段混合。无效覆盖报错，不偷偷换一个存储目的地。schema 1 历史配置仍可读取。新会话冻结实际配置及来源哈希；配置修改只影响之后启动的会话。

## 3. 路径写法

使用逻辑路径，避免将磁盘位置重复写进每条命令：

```bash
python3 scripts/wc_phase1 capture --profile mapping_core \
  --session session_001 --duration 300 --manual-drive --preview

python3 scripts/wc_phase1 doctor \
  --session-root data/experiments/session_001 --verify-archive

python3 scripts/wc_phase1 refine \
  --dataset data/experiments/session_001 \
  --output data/analysis/session_001_refined \
  --mechanical-initial --native-map
```

采集命令由现场操作者执行。原录包若为 PARTIAL，先检查原因，接受限制用于研究后才显式追加 `--allow-partial`。`--mechanical-initial` 使用已保存的 V7 机械初值，不更改正式外参。

映射关系：

```text
data/experiments/... → <数据根>/data/experiments/...
data/analysis/...    → <数据根>/data/analysis/...
reports/...         → <数据根>/reports/...
maps/...            → <数据根>/maps/...
config/...          → <代码根>/config/...
```

查询实际位置：

```bash
PYTHONPATH=src python3 -B -m wc_runtime.storage_policy \
  --project-root "$PWD" data/experiments
```

## 4. 运行状态、暂存和测试产物

进程登记、设备所有权及 Unix socket 使用本机 Linux 运行位置，不能整体搬到 exFAT。`.phase1_runtime` 保留本工程运行状态；同一用户的跨 checkout 锁和短路径 socket 使用受限的本机临时目录。它们不是原始录包的长期保存目的地，不应在运行中手工删锁。

`capture` 命令行默认 `--staging memory`。原始数据先进入本机 `/dev/shm/wc_capture_<用户编号>/<会话名>`，停止数据源后转存至配置的数据根，再进行持久化及完整性核验。最终档案仍保存到配置的位置；运行中内存里的数据还不能算已保存，断电或重启可能丢失。

| 模式 | 行为与限制 |
|---|---|
| `--staging memory`（命令行默认） | 需要本机 tmpfs 和足够可用内存，同时监控目标盘；停止后等待 `TRANSFERRING` 及核验完成 |
| `--staging disk` | 直接向档案目录录制，受目标盘写入性能影响；不等于通过断电恢复验收 |
| 手动驾驶 + exFAT | 当前只支持 `--staging memory`；轮控制所有者需要经过认证的 RAM 采集会话，`--manual-drive --staging disk` 会在启动前拒绝 |

上述 exFAT 限制也适用于驱动将该文件系统报告为 `fuseblk` / `fuse.exfat` 的情形。将 Unix socket 放到本机临时目录，并不自动解除手动驾驶会话的存储限制。普通本机 Linux 文件系统可选择 disk；切换时仍需按实际写入速度验收。

`--duration` 接受 1–3600 秒，表示请求的最长录制时间。内存和目标盘两者都可能先达到运行下限，程序会停止来源并保存可留存部分；增加 U 盘容量不会同时增加暂存内存。提前结束按一次 Ctrl+C，保留终端并等待完整收尾，不用关闭窗口或拔盘代替停止。

使用维护的测试入口：

```bash
bash tests/run_target_tests.sh \
  tests/usb_storage tests/operations/test_software_test_runner.py
```

日志、JUnit、pytest / Matplotlib 缓存和临时夹具归档在数据根 `reports/test_runs/<运行编号>`。需要 Unix socket、权限语义或符号链接的夹具在本机临时目录运行，归档核验后只清理本次拥有的临时目录。不要把 POSIX 夹具直接放到不支持这些特性的盘上。

测试源码始终保留；软件通过不等于设备采集或动态精度通过。历史真实会话的收尾证据不是普通缓存，不能随意清理。

## 5. 换电脑或重置

1. clone 所需 Git 分支 / 提交，准备系统依赖、固定 SDK、OpenCV ABI 并重新构建。
2. 配置新数据根；把要继续研究的录包另行复制进去，核对档案哈希。
3. 仅离线使用时，传感器身份来自档案。真实采集则恢复本机设备绑定、udev 和雷达接收网络。
4. 当前装配几何保存在配置与来源资料中，换计算机无需重测；改变装配时只更新相关项。
5. 软件、离线重放、静态采集、操作者驾驶分别验收。

不直接把旧 `install/` 当成新平台已构建产物。GitHub 保存维护源码及模板，不是现场工程与数据盘的自动镜像；新修改需要另行提交推送。

## 6. 可选 U 盘 / 外置 SSD

需要可移动数据时，先挂载并用 `lsblk -f` 核对实际 UUID，再指定：

```bash
python3 scripts/configure_storage --backend removable \
  --archive-root /实际挂载目录/wheelchair \
  --mount-point /实际挂载目录 \
  --required-uuid 实际文件系统UUID
```

配置脚本不格式化、不挂载磁盘，不选择“第一只 U 盘”。已存在现场覆盖的机器不必重复配置。选择 removable 后，盘未挂载、只读、掉线或身份变化会停止写入，不回退同名内部目录；directory 后端不需要任何盘符、卷标或 UUID。

前次本机配置的归档保存在数据根 `reports/storage_configuration/`。切换目的地不会自动搬动旧实验；迁移应单独核验完整文件和哈希，然后再决定清理原副本。
