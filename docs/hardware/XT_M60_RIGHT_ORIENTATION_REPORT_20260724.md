# XT-M60 Right Orientation Correction

Date: 2026-07-24 (Asia/Shanghai)
Branch: `feature/mapping-v2-rviz-workbench`
Scope: right radar axes/ground orientation and runtime TF only; motors disabled

## Result

The right XT-M60 point cloud direction is corrected provisionally in the ROS TF tree.
The correction is evidence-based for axis convention, height, roll and pitch. Horizontal
yaw and the longitudinal position are not calibrated and remain blocked from final
dual-LiDAR fusion acceptance.

No persistent command was sent to either radar. Normal startup retained
`apply_device_config=false`; the SDK only performed its required runtime UDP destination
request and measurement start.

## Root cause

The XT-M60 SDK publishes camera coordinates: `+z` forward, `+x` left and `+y` up. The
current URDF had removed both radar fixed joints because comments assigned the TF edges
to `ground_plane_calibrator_node`. That node has no source and no installed executable
in the current branch. The fallback `static_transforms.yaml` still described both raw
SDK clouds as if they used body/laser axes (`rpy=[0,-0.0873,0]`). Thus there was no
valid single runtime owner for the SDK-axis to base-axis transform.

## Measurements

The wheelchair was user-confirmed level and stationary. Both radars ran concurrently
through independent SDK processes while the diagnostic read point clouds only.

| Fit | Right floor normal in sensor xyz | Distance | Inlier ratio | Mean residual |
| --- | --- | ---: | ---: | ---: |
| 10-frame initial | `[-0.029855,0.988593,-0.147623]` | `0.7766 m` | `19.997%` | `12.86 mm` |
| 30-frame repeat | `[-0.026524,0.990374,-0.135851]` | `0.7541 m` | `23.029%` | `12.17 mm` |

The repeat implies approximately `7.81 deg` nose-down and `-1.53 deg` roll about the
optical-forward axis. If correcting the mount mechanically instead of in software, tilt
the optical face upward by about `7.8 deg` and level its roll, then rerun calibration;
do not keep both the physical correction and this software tilt unchanged.

The simultaneous left fits selected a surface above the sensor, not the floor, and were
rejected. Left orientation therefore retains the last reliable floor-visible calibration
from `feature/fastlio-narrow-fov-mapping` commit `45b1163` rather than using the invalid
current fit.

## Applied provisional TF

User measurements set radar optical-centre height to approximately `0.735 m` and left-
right spacing to approximately `0.60 m`. The TF therefore uses `y=+/-0.30 m` and
`z=0.735 m`. The earlier `x=0.45 m` estimate is retained and explicitly remains
unverified.

| Frame | Translation xyz m | RPY rad | Status |
| --- | --- | --- | --- |
| `xtm60_left_link` | `[0.45,0.30,0.735]` | `[1.551515,-0.013615,1.570931]` | historical reliable roll/pitch; yaw/xyz not final |
| `xtm60_right_link` | `[0.45,-0.30,0.735]` | `[1.707117,0.026527,1.574435]` | current ground roll/pitch; yaw constrained |

For the right radar, the corresponding sensor-to-base quaternion is
`[0.525700,0.539967,0.458534,0.470979]` (`xyzw`). The matrix is orthonormal and right-
handed, maps the measured floor normal to `[0,0,1]`, and places the fitted floor at
`base_link z=-0.0191 m` when using the measured `0.735 m` height.

Horizontal yaw is set to `0 deg` by constraining the projection of optical `+z` to
base `+x`. A flat floor contains no yaw information. This must not be interpreted as a
measured complementary-FOV angle.

## Verification and remaining gate

- `wheelchair_description` and `wheelchair_bringup` built and tested successfully.
- Runtime `tf2_echo` returned the intended left and right transforms.
- `/tf_static` had one publisher, `robot_state_publisher`; no duplicate calibrator exists.
- `static_transforms.yaml`, `sensor_layout.yaml`, and the URDF now agree.
- No fusion node, FAST-LIO2, mapping, navigation, controller, encoder or motor test ran.

Before dual-cloud fusion or FAST-LIO2, measure horizontal yaw and longitudinal/lateral
translation using an overlapping 3D target or point-cloud extrinsic calibration, then
update the currently stale FAST-LIO2 extrinsics. User visual confirmation in RViz is
also required. Right intensity values above the manual's `2039` maximum remain a
separate vendor question.

## Evidence

- `docs/hardware/evidence/XT_M60_LEFT_GROUND_CURRENT_INITIAL_20260724.json`
- `docs/hardware/evidence/XT_M60_RIGHT_GROUND_CURRENT_INITIAL_20260724.json`
- `docs/hardware/evidence/XT_M60_LEFT_GROUND_CURRENT_REPEAT_20260724.json`
- `docs/hardware/evidence/XT_M60_RIGHT_GROUND_CURRENT_REPEAT_20260724.json`
- `docs/hardware/evidence/XT_M60_RIGHT_ORIENTATION_PROVISIONAL_20260724.json`
- `docs/hardware/evidence/XT_M60_TF_RUNTIME_20260724.txt`
- `scripts/hardware/xtm60_ground_plane_diagnostic.py`
- `scripts/hardware/xtm60_orientation_diagnostic.py`
