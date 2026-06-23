# Orin 主机运行须知（所有会话通用）

## sudo 密码
- 本机 `nvidia` 用户的 sudo 密码是 `nvidia`。
- 非交互用法：`echo nvidia | sudo -S <command>`。
- 用于 `nvpmodel`、`jetson_clocks`、启用持久化 journald 等需要 root 的操作。

## 重启根因：电源 brownout（不是软件/温度/内存）
症状：执行重命令时主机无预兆重启。已排查证据：
- 温度低（tj ~46°C）→ 非热保护。
- 内存 61GB、空闲 56GB、swap 未用 → 非 OOM。
- `/sys/fs/pstore` 为空、`/var/crash` 无内核 dump → 内核没来得及记录 panic
  → 典型的 PMIC 因输入电源轨瞬时跌落而硬复位。
- 重启时间点与重负载命令高度相关；板子在 `MODE_50W`，VDD_IN 空闲 ~19W，
  并行编译或 fastlio+rviz+bag 全栈会瞬时拉高 CPU/GPU 电流 → 供电跌落 → 复位。

## 运行重命令的防护规则（务必遵守）
1. 用 `auto_test/20260623_dual_radar_calib/cpu_safe.sh` 包装重活：
   `bash cpu_safe.sh -c 0-5 -- <command>`（taskset 限核 + nice 15 + ionice idle，
   压住电流尖峰）。
2. 不要 `colcon build` 满核并行；用 `--parallel-workers 2` 或 `taskset -c 0-3`。
3. 不要同时跑采集与离线处理；采集 teardown 后再处理。
4. 雷达上电硬上限 ~20s，带 watchdog 强制 teardown。
5. 大点云（>4万点）做 RANSAC 前先子采样到 ~4万，避免 SIGKILL/内存尖峰。

## 建议（需 sudo，尚未执行，可按需开启）
- 启用持久化日志以便抓到下次复位前日志：
  `echo nvidia | sudo -S mkdir -p /var/log/journal && echo nvidia | sudo -S systemctl restart systemd-journald`
- 如仍复位，考虑降功耗模式（更稳）：`echo nvidia | sudo -S nvpmodel -m <更低功耗档>`。

## 传感器现状
- 左雷达被放入盒子中、视场被遮挡，标定不可用。**当前只使用右雷达。**
  - 右雷达 IP 192.168.1.100；话题 `/xtm60/right/points`；网卡 eno1。
  - 右雷达标定（20260623）：高度 51.0cm、pitch −1.04°、roll −9.1°、平面残差 4.3mm。
