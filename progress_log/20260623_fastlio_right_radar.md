# 进展记录 — 2026-06-23 — FAST-LIO 右雷达路径 + 收尾

## 主题
完成 `fastlio-narrow-fov-mapping` spec 的剩余补充:右雷达路径、LASER_POINT_COV 加固、外参标定、文档与进展记录系统。

## 背景(硬件现状变更)
- LEFT 雷达被放入盒子、视场遮挡,标定不可用(地面点 <~150)。**部署改为 RIGHT 雷达单雷达。**
- RIGHT 雷达标定(20260623):高度 **51.0cm**、pitch **−1.04°**、roll **−9.1°**、平面残差 4.3mm(地面点充足,可靠)。

## 本次做了什么
1. **右雷达配置** `src/wheelchair_3d_mapping/config/xtm60_right_lio.yaml`:右雷达外参
   `extrinsic_T=[0.606,-0.24,0.06]`、`extrinsic_R`(R_CONV ⊕ roll −9.1° ⊕ pitch −1.04°);其余同左(lidar_type=2、deskew off、Task-7 协方差)。
2. **launch 参数化** `fast_lio_mapping.launch.py`:新增 `radar:=left|right`(默认 right),按雷达选 config +
   输入话题 + `base_link→xtm60_<radar>_link` 静态 TF。
3. **顶层右雷达 launch** `manual_mapping_lio_right.launch.py`(EKF off,FAST-LIO 拥有位姿)。
4. **run 脚本** `scripts/run_rviz_manual_mapping_left.sh`:`RADAR` 选 launch(默认 right);左雷达分支保留。
5. **LASER_POINT_COV 加固**:存 patch `patches/fastlio_laser_point_cov.patch` + 幂等重放脚本
   `patches/apply_fastlio_patches.sh`(re-clone/submodule update 后恢复 100.0,否则偏航不跟踪/漩涡复发)。
6. **文档**:新建顶层 `README.md`;`docs/fastlio_mapping.md` 增 §7(双雷达外参/右雷达启动)、§8(LASER_POINT_COV 脆弱点)、§9(Orin 供电 brownout)。
7. **进展记录系统**:`progress_log/`(本目录)+ hook `save-progress-log`。

## 结果 / 验证
- `colcon build`(限核)两包通过;三个 launch 解析正常(right 9、left 9、fast_lio_mapping 6 实体)。
- 右雷达**离线冒烟**(dual bag 回放 `radar:=right`):`fast_lio` 产出 `/Odometry`×79、`/cloud_registered`×79、`/path`×7 → 链路端到端可用。
- 详见 `auto_test/20260623_dual_radar_calib/`。

## 未完成 / 待办(交接)
- **右雷达"地面可见 + 有运动"的现场建图验收**(Task 15 运动部分):需空旷场地 + 充电充足 + 离地/急停在手;
  确认偏航跟踪(对照 ~98° 基准)、地面 z≈0、墙平行、无漩涡。
- LEFT 雷达出盒后用 `RADAR=left` 恢复其路径(配置/外参已就绪)。
- 可选:把 `LASER_POINT_COV` 提升为 ROS 参数(免改第三方源码)。
- RTAB-Map 回环后端(`enable_loop_backend:=true`)仍需 `odom↔camera_init` 对齐 TF 才能用(默认 off 不影响)。

## 关键坑 / 注意事项
- **LASER_POINT_COV 在 vendored 源码**:re-clone 会还原 → 先跑 `patches/apply_fastlio_patches.sh`。
- **Orin 供电 brownout**:重命令限核(`taskset -c 0-3`、`--parallel-workers 2`),采集与处理不并行,大点云 RANSAC 前子采样 ≤4 万。
- **早期外参数字互相矛盾已废弃**:以本记录/`docs/fastlio_mapping.md §7.1` 表为准。
- 雷达过热:上电硬上限 ~20s + watchdog。
