# XT SDK integration

Upstream: https://github.com/XT-Toffuture/xtsdk_ros.git
Pinned commit: `965d31ae726c44b47ad646666c3c5fa2a4d91bf4`.
Source location: `SDKs/xtsdk_ros`. The Windows preparation copy has its upstream Git history; the target contains its reviewed exported snapshot without a fabricated `.git`. No vendor install, selection, firmware or sample program was executed. The exact project changes are in `xtsdk_ros_965d31a.patch`.

Every target configure checks the 32 source/header/library entries in `xtsdk_ros_965d31a.manifest.json`, the pinned patch bytes, and a reverse patch check on an isolated copy that normalizes CRLF to LF only. Original SDK binaries remain unchanged; reviewed source changes are recorded by the patch. A real SDK checkout must also match its own recorded HEAD; an export is explicitly `VERIFIED_EXPORTED_SNAPSHOT`, `git_checkout_verified=false`, `sdk_head=null`. Generated build/compiler/input provenance is installed at `install/main/wc_xt_driver/share/wc_xt_driver/provenance/vendor_provenance.json`. A failed check blocks the build; do not bypass it or replace an existing SDK to hide a mismatch.

The project compiles the reviewed C++ source in its own CMake target/build directory. Upstream `selros.sh` and source-writing CMake entry points are unused. Proprietary `libxtsdk_shared.so` is required by the SDK; aarch64 SHA256 is `3b34f29269858b07bc7b1fb22a7a5b4e709a7e87e7b8a814a57c03eb9f5293e5`. That original binary is preserved and is no longer selected for the filtered driver build. The user-authorized host filtering now uses the ABI-matched official filter bundle at `SDKs/xtsdk_filter_3d3db067`, verified by its separate manifest. Closed filtering internals cannot be source-audited. SDK depth decoding and geometry conversion remain vendor behavior; the raw branch is before host filtering, not a raw-photon interface. See `xtcfg_handoff.md` for exact scope and the remaining Windows-only switch.

Changes:

- Bind UDP to the configured concrete receive IP without SO_REUSEADDR; source IP must match the configured device. TCP also binds the requested local IP. The unchanged public pre-start `setUdpDestIp` API supplies host-side configuration; its documented false result before connection is expected. The connection-time device UDP write is suppressed, and command 19 is blocked at transport.
- Bounded UDP reassembly checks actual datagram size, frame size, offset, overlapping/conflicting duplicate fragments and envelope length. At most three incomplete frames are retained. TCP response parsing now handles split/coalesced stream messages instead of assuming one read_some equals one response. Only the SDK's explicit endianness is used; no ambiguous endianness guessing is attempted.
- Both command transport paths use the shared length-checked `imaging_allowlist.hpp`: reads 0/4/5/18, acquisition 1/2, and user-authorized transient imaging 8 (six exposure times), 9 (minimum amplitude), 10 (HDR), 12 (vertical binning), 27 (five modulation frequencies), 56 (maximum FPS). Payload lengths/enums are checked. Network, clock, firmware, reset and persistent save remain denied. No generic custom-command service is exposed.
- SDK raw/image queues each use a 64-frame / 64MiB owned-payload budget. Strict recorder mode uses FIFO; preview latest-frame discards remain explicit counters. Producers stop before raw and image queues drain; queue drop/highwater/pending snapshots do not wait for callback or I/O locks. The driver callback reserves its own 64-frame / 64MiB budget and deep-copies the raw/filtered pair, while a separate worker hashes, journals and publishes. Strict capture cannot be COMPLETE after rejection, SDK drop, pending data, malformed frame, or an unconfirmed reliable publication ACK.
- Worker threads are joined before internal state is destroyed. Nonblocking receive calls permit shutdown without indefinite blocking reads. Cross-thread lifecycle flags are atomic. The source process has a default 10s shutdown watchdog covering SDK join, worker drain, the shared 3s raw/filtered ACK budget and artifact sync; expiry exits nonzero rather than detaching live workers or writing a successful summary. These new software paths require the root-owned target build/tests and a recorded-data completeness audit; source preparation is not runtime proof.
- Failed/invalid lens reads are not replaced with default intrinsics. Device strings are length-bounded. Frame metadata/trailer bounds are checked.
- After invalid lens parameters suspend projection, receiving the original valid parameters rebuilds the projection table. A cached parameter match may return early only while the table is valid; it must not leave projection permanently disabled. This does not change lens values or constitute a calibration fix.
- HDR level nibbles must be below the five-element integration/frequency array count; malformed levels are rejected, never clamped. Odd packed-level pixel counts, truncation and selected unknown frequency codes reject the frame. `Frame::sort_data_valid` is appended without modifying IFrame's virtual method signatures, and the worker reports only successfully decoded images. Rejections emit local `wc_frame_rejected` events and appear in driver diagnostics as `sdk_malformed_frames_rejected`. This fix was specifically approved after PDF/source semantic review uncovered the upstream indexing defect.

Original same-port finding: upstream `communicationNet.cpp` binds `0.0.0.0:port`, sets `reuse_address(true)`, and passes UDP payloads to reassembly without checking `remote_ep`. Two identical copies cannot establish separate ownership of both streams. This patch permits the numeric port 7687 on two distinct local IPs, with source filtering. Root executed all four native target tests successfully: `xt_packet_test`, `xt_loopback_test`, `xt_hdr_bounds_test`, and `xt_shutdown_test`; the shutdown test also passed ten consecutive repetitions (`reports/builds/shutdown_repeat10.log`). These software tests do not establish optical isolation or physical scale.

Root also completed real concurrent bags `static_20260911T141849Z` and `static_20260911T144022Z`, each containing 296 left and 296 right authoritative frames. Both recorded-data audits are `DATA_REVIEW_OK`. The private recordings verified distinct identities. This public documentation substitutes example serials: left `XTM60B00000000000012`, device `192.168.0.101` to receiver `192.168.0.100:7687`; right `XTM60B00000000000013`, device `192.168.1.101` to receiver `192.168.1.100:7687`. The source identities and epochs are distinct, with no raw-key collision in the audited recordings. The private recordings provide same-port concurrent reception evidence for their original endpoints; the public example serials are not actual readback, not evidence of absent optical interference, metric accuracy, physical mounting, or validated sample synchronization. See `reports/sensor_handoff.md` and `reports/live_static/`; the second session's component logs are archived in `reports/live_static/static_20260911T144022Z/` and show both drivers completed SDK cleanup and exited cleanly.

Build/test and live hardware execution are root-owned on Orin. `wc_xt_driver` defaults to disabled hardware and a read-only probe. Identity/endpoint mismatch refuses acquisition. A read-only probe never applies imaging setters. A managed live acquisition strictly parses all 46 xtcfg fields. The default `preserve_current` policy retains device imaging settings and applies the selected supported host filters; device setters require the explicit reviewed `apply_xtcfg` policy. The xtcfg input file remains unchanged; before/after readbacks, command ACKs and every field disposition are stored. `pclFilterOn` remains explicitly unsupported and causes a PARTIAL_XTCFG label, not a quality-test gate or an acquisition refusal.

Current exact patch SHA256: `633832092c07a47e2e557d0023ddf1f882295a97e2b9bcf640e8f0a7ab9e8ecd`. This supersedes the prior revision and includes the compatible filter backport, raw/filtered pairing and bounded strict SDK queue/drain diagnostics. Native results quoted below are historical until root reruns the updated build. `tests/sensors/test_xt_hdr_bounds.cpp` directly tests the real vendor decoder with synthetic data, all 5 valid stages and all 22 invalid nibble placements, plus truncation, stale validity, odd dimensions, selected unknown frequency and recovery. Its target result is PASS, as reported above; source preparation alone is not a passed test.

## Reproduce the pinned source without touching an existing checkout

These commands are for the verified Orin user/project only. They are documentation, not automatically executed here. If `SDKs/xtsdk_ros` already exists (including a symlink), the subshell stops and preserves it. Do not reset, delete, replace or reapply over an existing project SDK. The final SDK is built only through this project's reviewed CMake, never a vendor sample or installation/selection script.

```bash
(
  set -euo pipefail
  test "$(hostname)" = ubuntu
  test "$(id -un)" = nvidia
  cd /home/nvidia/wheelchair
  test "$(pwd -P)" = /home/nvidia/wheelchair
  if test -e SDKs/xtsdk_ros || test -L SDKs/xtsdk_ros; then
    printf '%s\n' 'Existing SDK preserved; stop and review it separately.' >&2
    exit 1
  fi
  mkdir -p SDKs
  test ! -L SDKs
  git clone --no-checkout https://github.com/XT-Toffuture/xtsdk_ros.git SDKs/xtsdk_ros
  git -C SDKs/xtsdk_ros checkout --detach 965d31ae726c44b47ad646666c3c5fa2a4d91bf4
  test "$(git -C SDKs/xtsdk_ros rev-parse HEAD)" = 965d31ae726c44b47ad646666c3c5fa2a4d91bf4
  printf '%s\n' '633832092c07a47e2e557d0023ddf1f882295a97e2b9bcf640e8f0a7ab9e8ecd  vendor_patches/xtsdk_ros_965d31a.patch' | sha256sum --check -
  git -C SDKs/xtsdk_ros apply --check /home/nvidia/wheelchair/vendor_patches/xtsdk_ros_965d31a.patch
  git -C SDKs/xtsdk_ros apply /home/nvidia/wheelchair/vendor_patches/xtsdk_ros_965d31a.patch
  git -C SDKs/xtsdk_ros diff --stat
)
```

The reproduction recipe above restores reviewed SDK source only. This branch includes the two unchanged ABI-matched filter runtime binaries; verify the path and hashes declared in `xtsdk_filter_3d3db067.manifest.json`; the original `.so` is deliberately retained. CMake refuses a missing/mismatched new bundle.

## Included filter runtime

Only the fixed aarch64 and x86_64 filter libraries are vendored under `SDKs/xtsdk_filter_3d3db067/`. The rest of the SDK uses the clone recipe above. See the [bundle NOTICE](../SDKs/xtsdk_filter_3d3db067/NOTICE.md) for exact upstream ownership and hashes, and the [deployment guide](../docs/deployment.md) for ABI dependencies and a complete build.
