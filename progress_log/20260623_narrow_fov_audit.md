# 进展记录 — 2026-06-23 — narrow-fov-mapping-audit spec 执行

## 主题
执行新 spec `narrow-fov-mapping-audit`:对 120°×45° 窄视场 FAST-LIO 建图系统做全项目审查 + gap 修复,
对标示例截图(3D 上色云 + 2D 栅格 + LaserScan + 轨迹 + 电池面板)。

## 背景
- 在分支 `feature/fastlio-narrow-fov-mapping`(从 RTAB-Map 时代 `feature/rviz-first-mapping-mvp` 分出,
  FAST-LIO 工作首次提交 `26927ea`)。
- FOV 确认:水平 120°、纵向 **45°**(此前文档误标 90°/60°,已全改)。
- 旧 RTAB-Map 时代文档已归档至 `docs/archive/`。

## 本次做了什么（Task 1–11,除 Task 5 需硬件）
1. **Task 1 基线审查**:确认 D024/D025/D038/D039 仍存在 → `auto_test/20260623_baseline_audit/report.md`。
2. **Task 2 修复 D024/D025**(`pointcloud_to_laserscan_node`):`restamp_output` 默认 False(保留采集 stamp);
   TF lookup 用 `msg.header.stamp` 而非 `Time()`(最新)。+2 测试。
3. **Task 3 修复 D038/D039**(`cloud_to_occupancy_grid_node`):新增 `accumulate=True` 持久累积栅格
   (障碍 sticky、free 填 unknown)+ 固定锚点 origin(不再逐帧漂移)。+2 测试。
4. **Task 4 右雷达 RViz 版式** `manual_mapping_lio_right.rviz`(轨迹改青色对标示例);右 launch 改引用它。
5. **Task 6 RTAB-Map 回环后端 TF 对齐**:`enable_loop_backend:=true` 时发 identity `odom→camera_init`
   静态 TF,补全 `odom→camera_init→body→base_link` 链;文档化。
6. **Task 7 D034**:确认先前会话已修(融合输出用源帧 stamp),5/5 测试通过。
7. **Task 8 D001/D005 评估**:FAST-LIO 主线 EKF off、轮速里程不发 TF/不参与位姿 → 无影响,降级后续。
8. **Task 11 回归**:核心 4 包 **65/65 测试通过**;顺带修复 `wheelchair_base` 测试桩 `FakeLogger`
   缺 `info/debug/error`(production 调 `get_logger().info(...)` 致 6 项预存失败)。

## 结果 / 验证
- 全部离线;未上电雷达。
- `auto_test/20260623_baseline_audit/` 下各 `.pytest_*.txt`。

## 未完成 / 待办（交接）
- **Task 5 右雷达现场建图验收**(需硬件):真实右雷达 + 空旷 + 充电 + 离地/急停;
  确认偏航跟踪、地面 z≈0、墙平行、无漩涡、2D 栅格有历史积累、RViz 各面板齐全(截图对标示例)。
- **Task 9** spec 状态同步:`fastlio-narrow-fov-mapping` 的 Task 15 待 Task 5 完成后标 [x];Task 16 文档收尾。
- **电池面板**:项目无 `/battery_state` 发布者与电池 rviz 面板;示例截图的 Battery Status 需后续补
  发布者 + 自定义面板(req 8.9 可选,数据缺失不破版,当前未加)。
- LEFT 雷达出盒后 `RADAR=left` 恢复。

## 关键坑 / 注意事项
- Orin brownout:所有命令经 `cpu_safe.sh -c 0-3`;采集与处理不并行;大点云 RANSAC 前子采样。
- RViz 多面板停靠几何(2D 图/激光在独立底部 dock)难以手写 .rviz 可靠复现,建议现场交互排布后保存。
