# Native graph delivery regressions

These tests require the target's installed ROS 2 Humble, generated
`wc_interfaces`, and the project's matching RTAB-Map/OpenCV build. They do not
start drivers or access sensors. The coordinator owns target execution and the
build lock; source preparation on Windows is not test evidence.

After rebuilding `wc_slam` with `BUILD_TESTING=ON` and sourcing its install:

```sh
ctest --test-dir build/wc_slam --output-on-failure -j1 -R '^wc_slam_(fingerprint|retry)$'
```

- `wc_slam_fingerprint`: 256 allocator poison patterns must produce identical
  full-message SHA256 values. Mutations cover all top-level bundle/assessment
  fields and nested cloud/odometry content, including single float-bit changes,
  a cloud byte, source mask, covariance, and signed zero. Only 32 bytes are
  retained per accepted node; message payloads are not cached by the graph.
- `wc_slam_retry`: actual native graph subscription callbacks receive synthetic
  dual-source XYZ scans. An identical retry and historical ID retry must ACK the
  latest complete GraphSnapshot without increasing revision, TF count, or graph
  file bytes/mtimes. Tests continue at `max_nodes=2`, close the actual RTAB-Map DB,
  and prove a further retry leaves every session file hash and mtime unchanged.
  One cloud-bit conflict for a committed ID must latch `PAUSED_INVALID` and stop
  all later ACKs/processing. Private topic/service/TF remaps and `mkdtemp`
  session/run roots isolate this software fixture. Failed roots remain in `/tmp`
  for inspection; successful tests remove only their own temporary root.

The retry digest uses two calls to the installed ROS serializer. The first
determines buffer capacity/length. The second starts with all allocated bytes
zeroed, and its address/capacity/length must remain unchanged. No YAML or rounded
float representation is used. The digest is scoped to this process's installed
ROS type support and is not an interchange hash across middleware versions.

Passing these tests proves software delivery idempotency. It does not establish
60-frame end-to-end throughput, real dual-lidar timing/calibration, physical
motion accuracy, or real-device loop closure. The separate full-pipeline run
must still show every accepted ID present in the final graph.
