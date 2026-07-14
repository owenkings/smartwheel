# Third-Party Source Policy

Third-party algorithm cores are fetched as pinned repositories and are not copied into
the mapping-v2 packages.

```bash
vcs import src/third_party_v2 < third_party/dependencies.repos
```

The command is optional for Stage A mock tests. FAST-LIO2 and RTAB-Map adapters expose
standard ROS topics, parameters, and launch boundaries. Any future XT-M60 preprocessing
adapter must be isolated, minimal, and covered by message/field/timestamp tests.

The pre-existing `src/third_party/FAST_LIO_ROS2` checkout belongs to the legacy
workspace and was inspected read-only. Mapping v2 records the same known commit but does
not silently patch its mathematical core.

