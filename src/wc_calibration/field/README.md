# 现场外参与轮速标定

不用标定板和卷尺，靠普通室内环境和手推行驶，求两台 XT-M60 的安装姿态、左右相对外参和轮速比例。方法和结果的说明见[操作文档](../../../docs/operations/field_extrinsics.md)。返回[上级目录](../README.md)。

## 内容与入口

- [geometry.py](geometry.py)：平面拟合、由墙面求竖直方向、找平旋转，以及固定倾角的左右 4 自由度联合点到面配准。
- [capture.py](capture.py)：只读加载录包中的 filtered 点云和轮速反馈，按轮速找静止段。
- [motion.py](motion.py)：单雷达逐帧 ICP 里程计（复用 `wc_runtime.icp_shadow`），以及雷达与轮速轨迹的平面手眼（求偏航）。
- [checks.py](checks.py)：现场三类录制的合格判断，包括墙面、推向墙、房间多处停顿。
- [check_cli.py](check_cli.py)：现场工具，录一项后当场分析并显示进度。入口是 `scripts/lidar_field_check`。
- [solve.py](solve.py)：离线联合求解，用独立录包验证，输出候选文件。入口是 `scripts/lidar_field_solve`。

## 边界

- 只读录包，不打开设备。现场工具通过 `wc_phase1 capture` 录制，只读，不驱动电机。
- 结果写入存储策略下的 `data/calibration/field_check/` 和 `field_solve/`，不修改正式配置。候选要写进设备参数时，走面板"设备参数"或 `wc_panel.parameters.ParameterStore` 的校验和备份流程。
- 判据数值是经验门限，不是精度证书。

## 验证

`tests/field_calibration/` 是合成场景回归：竖直方向、4 自由度配准、平面手眼、轮速静止段和合格判定。修改后运行：

```bash
bash tests/run_target_tests.sh tests/field_calibration
```
