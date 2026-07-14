# Third-Party Dependencies and Licenses

| Dependency | Pin used by Stage A | License | Local modifications | Selection and risk |
| --- | --- | --- | --- | --- |
| Ericsii/FAST_LIO_ROS2 | `2fffc570a25d0df172720bac034fbdb6a13d2162` | GPL-2.0 | None in mapping v2 | ROS 2 port already present locally; XT-M60 field/time compatibility remains unverified |
| hku-mars/ikd-Tree | `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4` | GPL-2.0 | None | FAST-LIO2 submodule; build/architecture risk on aarch64 |
| hku-mars/FAST_LIO | `7cc4175de6f8ba2edf34bab02a42195b141027e9` | GPL-2.0 | Not vendored | Upstream mathematical reference; the ROS 2 fork is not assumed byte-equivalent |
| rtabmap_ros | ROS Humble Debian `0.23.7` | BSD | Configuration only | Installed and Humble-compatible; database/output behavior is tested only with synthetic input in Stage A |
| RTAB-Map | Transitively supplied by ROS packages | BSD | None | Standalone ROS package name `rtabmap` was not found; executable availability is checked by launch/runtime gates |
| slam_toolbox | ROS Humble Debian `2.6.10` | LGPL | Configuration only | Optional 2D comparison backend, mutually exclusive with RTAB-Map TF ownership |
| robot_localization | ROS Humble Debian `3.5.4` | Apache-2.0 | Configuration only | Optional fallback/selector; duplicate H30 fusion is forbidden |

Pins were recorded on 2026-07-14. `third_party/dependencies.repos` is the source-fetch
manifest; ROS distribution packages remain managed through apt/rosdep and are pinned by
the versions above. No third-party mathematical core is copied into a `smartwheel_*`
package.

