# 障碍感知（窗口 C）

把冻结前端准备好的三维点云变成局部避障用的障碍点：去地面、按高度窗留点、去掉车体自身、拒绝两台雷达各自车头方向的悬空伪点，并给出代价地图清除用的自由空间射线。只读冻结前端和录包，不打开设备，不发布速度指令。

| 文件 | 职责 |
|---|---|
| [obstacle_cloud.py](obstacle_cloud.py) | 单帧过滤（纯 numpy）：高度窗、远处低点门限、车体外形剔除、悬空小团判别、体素降采样、清除射线。 |
| [consensus.py](consensus.py) | 另一台雷达的“见证”：它的射线穿过、却没有回波的体素，用来否决这台雷达的悬空小团；同一台雷达 0.6–2 s 前在同一世界位置的候选点（持续性检查，默认关）。 |
| [frontend_frames.py](frontend_frames.py) | 按索引哈希逐帧读取单左或单右冻结前端（mapping_reference 点云和帧位姿）。 |
| [nav2_obstacles.py](nav2_obstacles.py) | 局部代价地图 VoxelLayer 和碰撞监控 pointcloud 源的参数片段、代价地图用的过滤扫描 `/obstacles/left/scan`、话题与坐标系约定。 |
| [shadow_obstacles.py](shadow_obstacles.py)、[shadow_cli_options.py](shadow_cli_options.py) | 影子运行的障碍输入：在 prepare_recording 里过滤左右前端并写进 recording.npz，命令行选项与结果里的停车区核查。 |
| [evaluate.py](evaluate.py) | 离线评估：录包未来几秒的扫掠外形里不该有静止障碍，按帧统计新旧输入落在里面的点（不启动 ROS）。 |
| [tracks.py](tracks.py)、[track_eval.py](track_eval.py) | 地图外障碍的航迹：去掉地图墙格附近的点、聚类、常速度卡尔曼跟踪，分 moving / person_candidate / static_unknown，输出 `/obstacles/tracks` 的 JSON 格式；离线在录包上跑并列出所有“在动”的航迹。还没接进影子运行或仿真。 |

话题：`/obstacles/<left|right>/points`（障碍点，标记和清除）与 `/obstacles/<left|right>/clearing`（自由空间射线终点，只清除），PointCloud2 x y z float32，坐标系 `lidar_<left|right>`（base_link 下的静态变换，平移为冻结 3D 杆臂）。只有影子运行加 `--obstacle-input cloud|scan_and_cloud` 或 `--obstacle-scan filtered` 时才发布；默认 `scan`、`raw` 时 /scan、Nav2 参数和结果都与以前相同。

设计与证据见[障碍感知与避障](../../../docs/architecture/obstacle_avoidance.md)，用法见[操作说明](../../../docs/operations/navigation_obstacles.md)，测试为 `tests/navigation/test_perception_*.py`。返回[离线导航开发](../README.md)。
