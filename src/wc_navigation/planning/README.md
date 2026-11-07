# 路径规划与运动控制

让规划和控制适配轮椅的外形和这栋楼的窄通道：车体模型（圆或凸多边形）、规划器与控制器的可选配置，以及对比它们用的工具。只用于仿真和录包影子运行，不连接车辆。

| 文件 | 职责 |
|---|---|
| [body.py](body.py) | 车体模型：假设圆或凸多边形外形（车轴为原点，x 前 y 左）。多边形对栅格的精确分离轴检查、扫过检查（含原地转向）、车体到障碍的间隙。仿真碰撞、仿真报告都用它。 |
| [options.py](options.py) | 仿真和影子运行共用的命令行选项：`--footprint`、`--footprint-source`。 |

外形与 `NavProfile.footprint` 是同一个：给了外形，代价地图、碰撞监控和仿真碰撞都用它；不给时都用 0.55 m 圆。测试在 `tests/navigation/test_autonomy_world.py`、`tests/navigation/test_autonomy_config.py`。上层框架见[自主导航框架](../autonomy/README.md)，返回[离线导航开发](../README.md)。
