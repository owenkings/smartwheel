# Wheelchair estimator comparison

`wc_rl_worker` links the installed official `robot_localization` `rl_lib`.
It creates no ROS node, TF publisher, device connection or motor command.
The Python provider exposes the same copy/predict/wheel/gyro/output contract as
the five-state filter. `MotionPrior.pose_at` remains the single authority for
both map odometry and dual-lidar compensation; every historical query clones
state, so a partial prediction cannot change subsequent source-event updates.

Build on the verified target with the project's existing colcon build/install
locations. Package dependencies are `robot_localization`, `rclcpp`, `angles`
and Eigen. The Python provider resolves the installed executable through the
ament index, or an explicit `WC_RL_WORKER` executable path. Missing dependencies
fail explicitly; a Python approximation is never substituted.

The official 15-state model and the five-state constant-turn model differ.
Both use wheel forward/yaw velocity and native-bias-subtracted, installation-
rotated gyro Z. The RL adapter adds the ROS wrapper's seven 2D zero constraints
with variance 1e-6. Its first measurement uses official initialization semantics.
The process covariance diagonal and first-measurement policy are reported.
The candidate 5 sigma gate is not an empirically calibrated rejection limit.
Missing source intervals freeze pose by an explicit application policy, which
differs from the official ROS node's sensor-timeout extrapolation.

Run the hardware-free official node regression after a native build:

```bash
PYTHONPATH=src:$PYTHONPATH python3 -B tests/integration/check_robot_localization_equivalence.py \
  --output reports/rl_parity_unique_name --domain 90 --steps 40
PYTHONPATH=src:$PYTHONPATH python3 -B -m pytest tests/operations/test_robot_localization_native.py -q
```

The first command needs an unused domain and output directory. It compares
state and covariance after each original-stamp correction, including both
wheel and gyro outliers; it records loaded official gate parameters. A pass
establishes adapter consistency for that tested continuous 2D protocol, not
vehicle accuracy or slope suitability.

Compare an immutable complete capture, retaining all original inputs:

```bash
python3 -B -m wc_runtime.mapping_compare --dataset reports/capture_session \
  --output reports/compare_unique_name --estimators five_state robot_localization \
  --input-rate-hz 5 10 --filter off on --cloud raw
```

The 5/10 Hz factor gates cloud consumption only. It never resamples wheel or
IMU measurements. Every stream uses one host-monotonic-to-ROS epoch mapping;
original bag bytes, wall/device stamps, source sequences and SHA-256 evidence
are retained. This does not establish physical measurement time synchronization.
Current captures require complete confirmed schema2 motion geometry; native
gyro calibration is available while geometry is blocked. Historical schema1
bags may be replayed only as their historical configuration, never adopted for
the current V7 assembly. `--allow-partial` and `--source-limit` produce explicitly
incomplete diagnostics. No independent truth means no claimed absolute error.

By default only frontend trajectories/CDR/counts/coverage/hash comparisons run.
`--native-map --domain 89 --native-rate-hz 1` additionally runs isolated native
RTAB-Map with the same common source-time subset, closes each database normally
and exports maps. The selected map rate and native acknowledgments are separate
from frontend 5/10 Hz consumption; a subset map does not prove 10 Hz throughput.
`--icp-shadow point_to_plane` writes correspondence, residual, rank, conditioning
and rejection diagnostics only. It never replaces the authority trajectory.
Both stages preserve the current production loop-correction-disabled policy.

Strict source-time gyro calibration is an offline operation:

```bash
python3 -B -m wc_runtime.mapping_bias_confirm --capture-dataset reports/capture_session \
  --stationary-evidence config/calibration/independent_stationary_interval.json \
  --output config/calibration/new_native_bias.json
```

The separate evidence JSON must declare `physically_stationary: true`, source
`operator_visual_confirmation` or `external_stationarity_calibration`, a
nonempty `evidence_id`, and `start_stamp_ns`/`end_stamp_ns` covering the displayed
mapped 30-second interval. Its actual file hash is computed by the tool.
Calibration uses 10 seconds warmup, 10 seconds estimation and a separate 10
seconds residual holdout. Exactly zero CRC-checked wheel registers are a
conservative screen requiring no guessed wheel scale; they do not independently
prove stillness. The tool never changes a running session or installs the result.
