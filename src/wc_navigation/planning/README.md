# 路径规划与运动控制

让规划和控制适配轮椅的外形和这栋楼的窄通道：车体模型（圆或凸多边形）、规划器与控制器的可选配置，以及对比它们用的工具。只用于仿真和录包影子运行，不连接车辆。

| 文件 | 职责 |
|---|---|
| [body.py](body.py) | 车体模型：假设圆或凸多边形外形（车轴为原点，x 前 y 左）。多边形对栅格的精确分离轴检查、扫过检查（含原地转向）、车体到障碍的间隙。仿真碰撞、仿真报告都用它。 |
| [options.py](options.py) | 仿真和影子运行共用的命令行选项：`--footprint`、`--footprint-source`、`--planner`、`--controller` 及参数覆盖、`--smoother-feedback`。 |
| [stack.py](stack.py) | 可选规划器（NavFn 默认、Smac 2D、Theta*、Smac Hybrid-A*、State Lattice）和控制器（RPP 默认、RotationShim+RPP、MPPI）的参数生成与读回；默认选择与原参数逐项相同，不倒车不可覆盖。 |
| [bench.py](bench.py)、[bench_cli.py](bench_cli.py) | 规划台架：录包起终点请求、各规划器并排规划、按真实车体评分（间隙、重叠、急弯、原地转向、倒车）与按司机路线分类。入口 `scripts/navigation_planner_bench`。 |
| [follow_audit.py](follow_audit.py) | 跟随模式影子运行的离线审查：边走边转时的原地转向、每次控制器停车是否由扫描与司机路线矛盾引起、窄处速度。 |
| [goal_heading.py](goal_heading.py) | 任务选项 `--goal-heading turn_at_goal` 的几何：选到达朝向（候选朝向的路径长度、路径末段方向），到点后判断原地转向扫过的范围是否空着（两个方向，按保存地图和局部代价地图）。由 `autonomy/mission_node.py` 调用。 |
| [people.py](people.py) | 车对人的反应（纯计算，还没有接 ROS 节点）：从 C 的航迹消息（`wc_obstacle_tracks/1`）取出人并预测，按车体外形算间隙，给出速度上限（提前减速、让行、最长等待），画两个代价地图的 KeepoutFilter 掩膜。 |
| [velocity_feedback.py](velocity_feedback.py) | 选项 `--smoother-feedback gated` 的 ROS 节点：给速度平滑器的闭环反馈，闸门放行时是平滑器自己上一次的输出，闸门输出为零时是零，停车以后从静止起步。 |

外形与 `NavProfile.footprint` 是同一个：给了外形，代价地图、碰撞监控和仿真碰撞都用它；不给时都用 0.55 m 圆。测试在 `tests/navigation/test_autonomy_world.py`、`test_autonomy_config.py`、`test_planning_stack.py`、`test_planning_bench.py`、`test_planning_goal_heading.py`、`test_planning_people.py`、`test_planning_velocity_feedback.py`。用法见[规划与控制操作](../../../docs/operations/navigation_planning.md)。上层框架见[自主导航框架](../autonomy/README.md)，返回[离线导航开发](../README.md)。
