# docs/archive — 已归档文档（RTAB-Map 时代）

本目录保存被 **FAST-LIO 主导建图方案** 取代的旧文档,仅作历史参考,**不反映当前实现**。

当前主线文档见上级目录:
- `docs/fastlio_mapping.md` — FAST-LIO 建图(获取/编译/外参/启动/保存/双雷达)。
- `docs/project_code_audit.md` — 全项目代码审查(活文档)。
- 顶层 `README.md` — 项目总览。

## 归档清单与原因
| 文件 | 原内容 | 归档原因 |
|---|---|---|
| `ekf_pose_mapping.md` | 纯 EKF 位姿 + RTAB-Map 零配准建图 | 被 FAST-LIO LiDAR-惯性方案取代;且 FOV 标注(120°×60°)已过时(实为 120°×45°) |
| `mapping_code_review_2026-06-17.md` | 旧建图链路代码审查 50 条 | 已迁移并重编号进 `docs/project_code_audit.md` §1/§4 |
| `ROADMAP.md` | RViz-first 分阶段路线图 | 硬件基线已过时(称右雷达损坏/左雷达为默认;实际左雷达入盒遮挡、右雷达为当前部署) |
| `HOWTO_RVIZ_MAPPING.md` | 单左雷达手动建图操作指南 | 基于左雷达 + RTAB-Map 链路,已被 FAST-LIO 右雷达链路取代 |
| `specs/` | stage0–3 旧分阶段 spec | 被 `.kiro/specs/` 下的正式 spec 取代 |

> 历史代码与文档同时也在 GitHub 分支 `feature/rviz-first-mapping-mvp`(RTAB-Map 时代)中保留。
> FAST-LIO 时代代码在分支 `feature/fastlio-narrow-fov-mapping`。
