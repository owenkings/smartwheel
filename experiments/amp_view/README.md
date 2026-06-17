# 实验：实时 PointCloud+Amp 显示（复刻上位机单帧效果）

**目标**：在 Orin / RViz 上看到和上位机一样的 PointCloud+Amp 实时图像。
**不建图、不动电机**，纯显示当前帧。

## 原理
XT-M60 本质是 160×60 的 Flash ToF **相机**。上位机那张漂亮的 "PointCloud+Amp"
其实就是**把幅度(amp)图按颜色映射显示**。RViz 不能把点云连成面,但能显示稠密的 2D 图像。
本实验把有序点云**还原成 2D 图像**,所以你在 RViz 的 Image 面板能看到那张"扫描出前方
物体"的稠密彩图——这才是最接近上位机观感的方式。

`amp_image_node.py` 订阅有序点云 `/xtm60/left/points`,reshape 回 160×60,发布:
- `/xtm60/left/amp_image` —— 幅度,JET 彩色映射(= 上位机 PointCloud+Amp 的观感)
- `/xtm60/left/amp_mono`  —— 幅度,灰度(红外照片感)
- `/xtm60/left/depth_image` —— 深度/距离,JET 彩色
图像放大 4 倍(160×60 → 640×240)便于观看。

## 用法
```bash
bash experiments/amp_view/run_amp_view.sh
```
RViz 起来后看三个 Image 面板:
- **Amp Image (PointCloud+Amp)** —— 主角,和上位机一样的彩色幅度图。
- **Amp Mono** —— 灰度版(更像红外照片)。
- **Depth Image** —— 按距离上色。
另外 3D 视图里有 `Left Cloud (amp)` 按 amp 上色的点云。

## 可调(amp_image_node 参数)
- `amp_min/amp_max`(默认 0~2039)：幅度上色范围,调对比度。
- `depth_min/depth_max`(默认 0~8m)：深度上色范围。
- `upscale`(默认 4)：图像放大倍数。

## 说明
- 这只是**显示**,和建图(伪SLAM / RTAB-Map / slam_toolbox)完全独立、互不影响。
- 只启动左雷达,不启动 IMU/超声/相机/电机,最轻量。
