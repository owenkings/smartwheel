# 代码备份、数据目录与换机部署

## 工程和数据的边界

工程目录保存 src、scripts、config、tests、docs 和 vendor_patches。tests 是可维护的回归测试源码，不是测试结果，应该随代码备份。build、install、SDKs 是这台机器的依赖／构建安装目录，不纳入 Git，换机需要重新安装依赖和构建。

录包、地图、离线融合、诊断报告、测试日志及测试缓存保存到配置的数据根目录。公共默认目录为 ~/wheelchair-data；部署者可通过本机覆盖选择固定目录或外置盘。已保存的历史录包不会因本次配置分离而移动或改写。

.phase1_runtime 中的锁、进程登记和 Unix socket 保留在本机 Linux 文件系统，不能整体搬到 exFAT U 盘；它不是长期数据存储目录。过去会话的收尾证据暂保留，不能把真实采集记录当缓存删除。

## 通用配置与本机配置

| 文件 | 用途 | 是否进入 Git |
|---|---|---|
| config/storage.json | 通用默认：directory 后端，~/wheelchair-data | 是 |
| config/storage.local.json | 当前机器完整覆盖配置；可指向本机固定目录或已核实的外置盘 | 否 |
| config/storage.removable.example.json | 其他 U 盘／SSD 的配置格式示例 | 是 |

优先级是 storage.local.json 存在时完整覆盖，否则读取 storage.json，不合并字段。覆盖文件损坏会报错，不会改用另一个目的地。已启动会话冻结自己的配置；修改仅对新会话生效。配置选了 U 盘时，未挂载或身份变化仍然停止写入，不自动改写内置盘。

schema 1 历史配置继续可读；schema 2 支持 directory 和 removable。directory 不需要 U 盘、特定卷标或 UUID。已有档案保留当时原配置字节，新档案同时记录默认、本机覆盖、生效配置及其哈希。

## 换到没有 U 盘的新 Orin

现有硬件入口按 Linux／Jetson、nvidia 用户和 `/home/nvidia/wheelchair` 安装约定使用；普通 Windows 电脑可保存和开发代码，但不能直接运行这套 Orin 驱动。先按工程构建说明安装 ROS、厂商 SDK 和依赖；核对新设备的串口身份、网络与安装参数。Git 备份不包含厂商二进制，不保证未经部署即可驱动另一套硬件。

然后在新机器的工程根目录运行：

```bash
python3 scripts/configure_storage --backend directory --archive-root "$HOME/wheelchair-data"
```

这会创建工程以外的数据目录及子目录，写入被 Git 忽略的本机存储配置。不需要连接 U 盘。更换内部 SSD 路径同样用 directory 并指定明确的目录。

若采用可拔插 U 盘／外置 SSD，先挂载目标盘并通过 lsblk -f 确认实际 UUID，然后使用：

```bash
python3 scripts/configure_storage --backend removable \
  --archive-root /实际挂载目录/wheelchair \
  --mount-point /实际挂载目录 \
  --required-uuid 实际文件系统UUID
```

该命令不会格式化、挂载磁盘或修改系统设置，不自动选择“第一只 U 盘”。重配前的本机配置保存到选定数据根 reports/storage_configuration。两种模式均不静默回退到其他存储位置。

已有本机存储配置的机器不必重复配置；勿用公共示例覆盖已核实的现场配置。

## 日常命令与路径写法

优先使用逻辑路径，避免把某个 U 盘绝对路径写进日常命令：

```bash
python3 scripts/wc_phase1 capture --profile mapping_core \
  --session v7_example_001 --duration 300 --manual-drive --preview

python3 scripts/wc_phase1 doctor \
  --session-root data/experiments/v7_example_001 --verify-archive

python3 scripts/wc_phase1 refine \
  --dataset data/experiments/v7_example_001 \
  --output data/analysis/v7_example_001_refined \
  --allow-partial --mechanical-initial --native-map
```

data、reports、maps 会解析到当前配置的数据根；源码和配置不会搬走。上面的录制由操作者主动执行。不同硬件条件、零偏验证及录制完整性仍按各命令实际报告处理。

查询真实位置：

```bash
PYTHONPATH=src python3 -B -m wc_runtime.storage_policy --project-root "$PWD" data/experiments
```

## 测试产物

使用 tests/run_target_tests.sh 运行指定软件测试，日志、JUnit、pytest 缓存、Matplotlib 缓存和最终临时文件归档保存在数据根 reports/test_runs/运行编号。源码测试保留在 tests 中。测试过程需要 Unix socket／符号链接的临时夹具在系统 /tmp 执行，结束后校验归档到数据根，再清理该次创建的临时目录；不把 POSIX 夹具直接放在不支持这些特性的 exFAT 上。

```bash
bash tests/run_target_tests.sh tests/usb_storage tests/operations/test_software_test_runner.py
```

测试不会自动证明硬件采集或动态精度已通过。不要把没有审核的全套硬件实验脚本作为普通单元测试运行。

## GitHub 的含义

此前 v7/orin-usb-20261006 分支包含源码和配置快照，标题提及 USB 是因为那次增加了存储重定向功能，并非上传 U 盘数据。该分支不是 Orin 目录的自动镜像，后续现场修改必须另外审查、提交和推送。

公共备份保留程序、配置模板和测试源码；不纳入录包、地图、诊断大文件、运行日志、本机 storage.local.json、构建安装产物和厂商 SDK 二进制。原始实测配置的私有备份仍在现场归档中，公开仓库的设备模板不能当成另一套硬件的已验证配置。
