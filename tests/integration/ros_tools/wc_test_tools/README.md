# Native TF provenance evidence

This small test package only subscribes to `/tf` with best-effort depth 100.
It records the actual `rclcpp::MessageInfo` publisher GID for `map->odom` and
`odom->rig_link`. It publishes no messages, broadcasts no TF, calls no services
and accesses no device. It must run on the already verified target in the same
ROS domain as the session being observed.

The actual target Humble headers define `TopicEndpointInfo` inside
`rclcpp/node_interfaces/node_graph_interface.hpp`, with `endpoint_gid()`.
`rmw_gid_t.data` and the endpoint array both contain 24 bytes. Evidence uses
lowercase hex of all 24 bytes, matching Python `bytes(endpoint_gid).hex()`.

Build only through the coordinator's target build lock, from the project root:

```sh
colcon --log-base log/test_tools build \
  --base-paths tests/integration/ros_tools \
  --build-base build/test_tools --install-base install/test_tools \
  --executor sequential --packages-select wc_test_tools
python3 -m unittest discover \
  -s tests/integration/ros_tools/wc_test_tools/test -v
```

After sourcing that install alongside the main project's install, start this
probe and the Python observer **before** starting the fixture. Use a new output
path for each run. For the values below, the native probe completes first;
the Python observer only consumes its final immutable JSON when its own
observation duration ends.

```sh
ros2 run wc_test_tools tf_ownership_probe --ros-args \
  -p output:="$SESSION/evidence/tf_native.json" \
  -p duration_s:=100 -p session_id:="$SESSION_ID"

python3 tests/integration/check_icp_pipeline.py \
  --session-config "$CONFIG" --duration 110 --close-snapshot \
  --tf-evidence "$SESSION/evidence/tf_native.json" \
  --output "$SESSION/evidence/pipeline_with_native_tf.json"
```

Run those two commands in separately supervised processes (the displayed
commands are separate foreground examples). `TF_PROBE_READY` and
`OBSERVER_READY` confirm subscriptions have been created; they do not establish
pipeline correctness. Then start the fixture through the coordinator's normal
session launcher. The probe's duration must cover every fixture TF sample;
the observer must outlast the probe. Neither program modifies ROS domain or
network configuration.

The native JSON stores its host/domain, observation wall-time window, actual
TF header stamps/transforms, per-edge GID counts, and discovered endpoint
history/current snapshot. Its `session_id` is clearly an operator-supplied
label. Python accepts it only after checking the same session/host/domain,
overlapping windows, every Python TF sample, unique independent endpoint/node
bindings, and TF transforms against the actual accepted ICP odometry and
optimized graph at every accepted session timestamp. Missing Python GIDs are
not fabricated. Existing Python contradictions cannot be overridden.

Outputs use atomic same-filesystem link publication with no overwrite; path
traversal and symlinks are refused. Native exit 0 means both requested edges
were observed with one resolvable publisher during the requested window.
Missing evidence/interruption or contradiction exits 1; runtime/configuration
failure exits 2. Receiving TF does not establish real sensor calibration,
physical motion, RViz rendering, navigation safety or map accuracy.

Source preparation only: no build or runtime test was performed by this agent.
The coordinator owns actual target compilation and test evidence.
The nine pure-Python tests use explicitly synthetic JSON to check rejection of
other hosts, conflicting GIDs/nodes, wrong poses/stamps, static motion TF and
missing/incomplete native evidence. Those fixtures do not count as real TF.
