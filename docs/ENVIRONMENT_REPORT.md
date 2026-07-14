# Stage A Environment Report

Audit date: 2026-07-14 (Asia/Shanghai)

Scope: read-only software and filesystem inspection. No serial, CAN, RS485, USB camera,
LiDAR network, or motor interface was opened.

## Host

| Item | Observed value |
| --- | --- |
| Kernel | `Linux ubuntu 5.15.148-tegra #1 SMP PREEMPT Mon Jun 16 08:24:48 PDT 2025 aarch64` |
| OS | Ubuntu 22.04.5 LTS (Jammy) |
| ROS distribution | Humble (`ROS_DISTRO=humble`) |
| Python | 3.10.12 |
| GCC / G++ | 11.4.0 |
| CMake | 3.22.1 |
| colcon | `colcon-core 0.21.0`, `colcon-ros 0.5.0` |
| PCL | Runtime libraries 1.12 found; `pkg-config pcl_common` metadata was not found |
| Eigen | 3.4.0 |
| OpenCV | System `opencv4` 4.8.0; Python wheel `cv2` 4.13.0 |
| CUDA | 12.6, nvcc 12.6.68 |
| Jetson Linux | R36.4.4 from `/etc/nv_tegra_release` |
| GPU | Orin (nvgpu), driver 540.4.0 |
| Disk | Root filesystem 57 GiB total, 12 GiB available, 80% used |

The `nvidia-jetpack` Debian metapackage was not listed. The exact JetPack marketing
version is therefore intentionally not inferred from Jetson Linux R36.4.4.

## ROS Mapping Dependencies

| Component | Status | Version |
| --- | --- | --- |
| `rtabmap_ros` | Installed in `/opt/ros/humble` | 0.23.7 |
| `rtabmap_slam` | Installed in `/opt/ros/humble` | 0.23.7 |
| `rtabmap_odom` | Installed in `/opt/ros/humble` | 0.23.7 |
| standalone ROS package `rtabmap` | Not found by `ros2 pkg prefix` | n/a |
| `slam_toolbox` | Installed in `/opt/ros/humble` | 2.6.10 |
| `robot_localization` | Installed in `/opt/ros/humble` | 3.5.4 |
| `xacro`, `robot_state_publisher`, `rosbag2` | Required by mapping v2; verified during build/runtime gates | See implementation status |

The failed `colcon version-check` command attempted a network version check; local
package metadata above is authoritative for the installed colcon version.

## Repository

- Repository: `/home/nvidia/smartwheel`
- Audit branch: `feature/mapping-v2-clean-architecture`
- Branch point: `45b1163`
- Preserved user file: untracked `docs/goal.md`

Existing ROS packages with overlapping responsibilities are:

- `wheelchair_sensors`
- `wheelchair_description`
- `wheelchair_base`
- `wheelchair_bringup`
- `wheelchair_3d_mapping`
- `wheelchair_mapping`
- `wheelchair_diagnostics`
- `wheelchair_safety`
- `wheelchair_perception`
- `wheelchair_navigation`

They remain in the repository for reference but are not included by the mapping-v2
launch files. Their launch, TF, sensor, and mapping configuration must not be mixed
with the new stack.

## Vendor SDK and FAST-LIO2 Evidence

- `/home/nvidia/smartwheel/xtsdk_py` exists and appears SDK-related, but its status as
  the complete official XT-M60 SDK is not established. It was not imported or loaded.
- An existing checkout of `Ericsii/FAST_LIO_ROS2` exists at
  `src/third_party/FAST_LIO_ROS2`, commit
  `2fffc570a25d0df172720bac034fbdb6a13d2162`.
- Generated `build/fast_lio` and `install/fast_lio` directories exist from the legacy
  workspace. They are not evidence that the mapping-v2 integration has been tested.

## Missing or Risky Items

- Only 12 GiB was free during the audit. Rosbags and map versions need free-space gates
  and retention discipline.
- PCL has runtime libraries but no `pkg-config` metadata; source builds that require
  PCL development metadata may need `libpcl-dev`.
- System OpenCV and the Python wheel differ in version. Mapping-v2 Python code must not
  assume ABI compatibility between them.
- Exact XT-M60 SDK capability, H30 protocol, encoder register definitions, and all real
  extrinsics remain Phase B TODOs.

Use `scripts/install_dependencies.sh` to review missing package commands. The script is
dry-run by default and was not executed with `sudo` during Stage A.

