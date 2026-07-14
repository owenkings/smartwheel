# Third-Party Source Policy

Third-party algorithm cores are fetched as pinned repositories and are not copied into
the mapping-v2 packages.

```bash
sudo apt-get install python3-vcstool
vcs import src/third_party_v2 < third_party/dependencies.repos
```

The dependency manifest places `ikd-Tree` at FAST-LIO's required nested path and
includes the ARM64 Livox ROS 2 dependency at an exact commit. The mathematical
upstream reference is intentionally outside the build manifest. Fetch it only for
source comparison:

```bash
vcs import /tmp/smartwheel-reference-sources < third_party/references.repos
```

The command is optional for Stage A mock tests. FAST-LIO2 and RTAB-Map adapters expose
standard ROS topics, parameters, and launch boundaries. Any future XT-M60 preprocessing
adapter must be isolated, minimal, and covered by message/field/timestamp tests.

The pre-existing `src/third_party/FAST_LIO_ROS2` and `livox_ros_driver2` checkouts
belong to the legacy workspace and were inspected read-only. Mapping v2 records their
exact commits but does not silently patch either source tree.
