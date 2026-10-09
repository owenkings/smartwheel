# XT-M60 配置来源与版本

当前启动配置（文件日期表示用户提供/导出版本，不是部署日期）：

| 侧别 | 主启动文件 | SHA-256 |
|---|---|---|
| 左 | `left-2026-09-11.xtcfg` | `e694fd7859c82d862500396dd92d79ee6e2c93ab35d52bebee0721d2f69fee62` |
| 右 | `right-2026-09-11.xtcfg` | `2f2595661d42fdfefa93592b4edb88916c1be340ff8c467f6fe357552a1d5c6e` |

原件是用户提供的 `G:\tiany\wheelchair\left-2026-09-11.xtcfg` 与 `G:\tiany\wheelchair\right-2026-09-11.xtcfg`。项目早期记录注明从 XT-Toffuture-V2.10.6 上位机导出。2026-09-15 已核对原件、此前本地工程配置、Orin 源码配置和安装配置逐字节一致，包含换行和全部 46 个配置项。

`dual_sources.launch.py` 读取已安装包 `share/wc_xt_driver/config/` 内带日期的同名文件。仅修改源码目录不代表安装副本已更新。当前文件是固定配置，没有按当前房间自动选参或完成场景精度标定的证据。

旧名 `left.xtcfg`、`right.xtcfg` 保留为历史录包核对工具的兼容副本，内容未变，主启动入口不再读取旧名。未来参数调整应保留本版本并使用实际新版本日期，避免覆盖原始导出内容。修改日期名时同步更新 launch、活动配置测试和本文。

当前 `preserve_current` 策略仍应用主机滤波，但不按这些文件重设设备 HDR、曝光、调制等成像参数。文件数值不等于设备此刻的读回值。

来源证据：项目 `reports/config_origin_maps_20260915/config_origin_zh.md` 与 `config_origin.json`。

## 目录职责与新增内容

已审查 SDK 的驱动适配、队列及设备配置校验放这里；不直接修改厂商原件。

新增文件须同步说明用途，并补充所属功能测试。返回[上级目录](../README.md)。
