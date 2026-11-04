# 先远程推进导航

相机外参尚不稳定时，可以先完成保存地图上的规划、数值跟踪、录包自动建图和单雷达回放定位。软件候选与实车验收分别记录。

## 保存地图规划与数值跟踪

在 Orin 的工程目录执行：

```bash
bash scripts/navigation_offline \
  --map /实际地图目录/map_3d.yaml \
  --assumed-radius 0.55 --margin 0.10 --demo
```

`--assumed-radius` 必须显式给出；这里的 0.55 m 只是数值实验的圆形包络假设，轮椅含脚踏总长尚未量定，不能当作实车外形。`--demo` 在最大连通可通行区域选择虚拟起终点，不代表当前轮椅位置。也可给 `--start X Y --goal X Y`，坐标单位为米，使用输入地图的坐标系。人工合成地图必须加 `--map-kind synthetic`。

程序复用现有 YAML/PGM 检查器，记录输入哈希。未知区域和地图外均按障碍处理；根据包络与余量膨胀后，只启动 Nav2 的地图服务器和全局规划器，不启动控制器、传感器或车辆驱动。规划器使用专用 localhost ROS 域 93 和独立命名空间；若域已被其他节点占用，则拒绝启动。生产设备与离线建图进程不加入此域。

路径经独立的连续圆形包络碰撞检查后，交给纯数值差速模型跟踪。该模型只计算数组，不发布任何车辆速度。使用有限速度、加速度、总时长和扫掠碰撞检查；无法通行则报告失败，不缩小包络、放宽未知格或自动修补墙体。当前 Nav2 接口明确拒绝 origin yaw 非零的地图。

输出经 StoragePolicy 保存至数据根 `reports/navigation/<独立任务>/`：

- `result.json`：路径、假设、地图身份、Nav2 版本、可通行区域统计、规划和模拟结论。
- `preflight.json`：规划前的输入与虚拟起终点。
- `simulation.json`、`trajectory.csv`：数值轨迹和终态。
- `preview.png`：原地图、包络膨胀区、规划路线和数值轨迹。
- 参数及进程日志：用于远程排查。

规划成功仅说明所选地图与假设下的软件路径成立。当前地图可能有墙根/地面假障碍，报告会保留膨胀后的连通区域大小；很短的可通行区域不能证明整段走廊可导航。定位、动态障碍、地面高差、实车制动均不在此模拟结论中。

## 继续推进建图与定位

现有离线融合队列可后台处理冻结录包并自动导出地图，支持单雷达，退出面板或断开 SSH 后仍由后台监督任务管理。单雷达可暂时避开跨雷达外参，但仍需录包中的雷达/IMU/车体坐标契约；缺少时只能显式使用机械初值候选或带证据的几何覆盖，不能静默绕过检查。

下一阶段先对已有录包和地图做单雷达回放定位，验证地图中的雷达轨迹，再接入车体定位。之后接 Nav2 局部控制、动态障碍和探索目标选择，先在隔离仿真中验证。

现场测试安排在远程软件、日志和回放稳定后。将含脚踏外形补测、现场定位初始化、停止行为和受控低速路线验证准备好后集中一次完成。日常调整、结果查看和后台运行通过远程连接完成。

实现见[导航源码](../../src/wc_navigation/README.md)，验证入口为 `bash tests/run_target_tests.sh tests/navigation`。Nav2 接口依据[官方坐标变换说明](https://github.com/ros-navigation/docs.nav2.org/blob/rolling/docs/configuration_and_development/first_time_robot_setup_guide/transformation/setup_transforms.md)与[代价地图说明](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/)，本机使用已安装的 Humble 1.1.20 并记录实际版本。返回[操作导航](README.md)。

录包重建和定位的实际命令、时段划分及输出见[重建与回放说明](navigation_rebuild_replay.md)。

动态圆盘的减速、停车、等待与恢复数值验证见[动态停车仿真](navigation_dynamic.md)。它使用冻结路径和虚拟位姿，不接实车局部控制器。
