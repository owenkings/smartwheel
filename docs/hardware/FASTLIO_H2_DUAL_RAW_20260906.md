# H2 双雷达原始采集加速尝试（2026-09-06）

用户要求加快进度；在此前安全条件仍成立的同一轮中，启动了只读双路入口：左右 XT-M60 + H30，独立 ROS domain，不启动 FAST-LIO、底盘或电机，录包前加入 `/h2/operator_marker`。

证据目录：`/home/nvidia/smartwheel/auto_test/h2_real_dual_raw_20260906_132843`。

- bag 时长 `165.283 s`，约 `256.6 MiB`，`44043` 条消息；marker 1 条，IMU 33054 条。
- 右路 `/points`、`/timing`、`/quality` 各 1640 条，右 status 为 connected/measuring。
- 左路只有 2772 条 status，内容为 `waiting: XT-M60 SDK not running; waiting for ping 192.168.0.101`；左 `/points`、`/timing`、`/quality` 全为 0。
- 没有做现场转动；本次仅确认左路未 ping。日志和 bag 均保留，未写设备配置。
- 录包、左右适配器与 IMU 已停止；结束时无相关进程、无 UDP 7687。

结论：双雷达加速采集被左路当前未响应阻塞。右路先前单路稳定并有 Stage 1 证据，因此此前只用右路是风险控制；这不代表左路可被跳过。下一步先做左路电源、网线/交换机端口和 IP 的单变量只读排查，左路恢复有效 points/timing 后再做带 marker 的双路短采集。
