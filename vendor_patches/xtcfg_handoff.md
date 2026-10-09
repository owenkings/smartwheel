# Applied per-device XT configuration handoff

Source branch: `feature/xtcfg-applied`; all target/hardware work remains root-owned.

## Actual implementation

The two unchanged `.xtcfg` files are strictly parsed (46 keys each). The software applies supported values without a cloud-quality or measured-delay prerequisite. Distinct fifth integration/frequency settings survive per-side parsing: left int5=0/freq5=24MHz; right int5=200us/freq5=3MHz.

Device API calls are conditional on before-readback difference and logged before dispatch and after ACK: `setHdrMode` (cmd10), `setIntTimesus` (8), `setMultiModFreq` (27), `setMinAmplitude` (9), `setMaxFps` (56), `setBinningV` (12). These are ordinary imaging setters, not persistent save commands. Network/clock/reset/firmware and generic command exposure remain forbidden. Horizontal binning has no public setter; supplied zero is recorded as READBACK_MATCH if the observed value agrees. A different observed value is explicitly unsupported. Device type/render type are GUI metadata. No original device configuration file or map is replaced.

Host filters: median3, Kalman factor300/threshold300/timedf2000, edge150, dust9000/framecount2/timedf300/validpercent100, post threshold5/dynamics1/window9/motion3, reflective0.5..2, spatial0.7/70/2; average remains disabled as requested. Factor scaling and post/dynamics argument mapping are supported by local `XT_M60_SDK.zip` Python examples. The 2000/300 ms values are algorithm time-difference tolerances, NOT measured sample or processing latency. `dynamicsMotionsize` maps to the fourth parameter of the actual current SDK API. All enabled flag bits are checked, including postprocess's void public return. Library rejection now propagates false instead of the upstream misleading true.

`Setting.pclFilterOn=1` is retained as UNSUPPORTED_WINDOWS_POINTCLOUD_PROCESSOR_SWITCH: the local Windows GUI includes `pointclouds_process_shared.dll`, while upstream `xtpcl_filter.cpp` is WIN32-only and no equivalent Linux API was found. This switch's exact GUI routing is not source-available. It is NOT silently treated as an SDK filter enable switch. Every acquisition is visibly PARTIAL_XTCFG, and GUI output parity is UNPROVEN. All requested `Filters` keys have supported or disabled-as-requested handling. The remaining switch does not prevent managed acquisition.

## Native SDK compatibility and patch

Original reviewed SDK origin remains xtsdk_ros@965d31ae726c44b47ad646666c3c5fa2a4d91bf4. Backported filtering comes from official xtsdk_cpp@3d3db067ae9bdc0528202c3087bc10fd3b706638, fetched to independent SDKs/xtsdk_cpp_reference solely for review. `IFrame` virtual functions are unchanged (two sentinel constants added). `IBaseFilter_lib.h`, BaseFilter and LibHandler are copied together because the newer library inserts virtual methods; mixing only the .so with old vtables is prohibited. Public filter paths receive scalar configuration and IFrame arrays/log callbacks, and expose no device transport handles. The proprietary binary's internals are not source-auditable.

The old per-architecture libraries remain untouched. Actual new runtime is under SDKs/xtsdk_filter_3d3db067/lib/linux/{aarch64,x86_64}/libxtsdk_shared.so. Both hashes and source origin are locked in xtsdk_filter_3d3db067.manifest.json and checked at CMake configure. aarch64 SHA256: 3336a76590b5447efd7c037929e61c287523fcd79e8125589a3adb35eee83411. x86_64 SHA256: 69d80f0d64f1b7a65c1cf76aa57d7ba84fb0ff34a03014e6d04d52092e683821.

Necessary additional fixes: `setMultiModFreq` used a four-element stack array then wrote/read index4; now a five-element std::array. Loader pointer initialization avoids a null factory/double-close path, and filter setter failure propagation prevents false application claims. The patch and canonical 31-file manifest include these changes.

## Frozen ROS interface

- `source_frame` and `points_raw`: decoded cloud BEFORE all host filtering/corner/min-amp/mirror cuts (device imaging still applies).
- `source_frame_filtered` and `points_filtered`: same sensor frame AFTER requested supported host processing.
- Exact same session/side/sensor/stream_epoch/frame_sequence, device timestamps, header, host_receive_time and monotonic capture time in both envelopes. Their source_config_hash values intentionally differ by representation.
- Filtered diagnostic flags include `raw_source_config_hash=<raw value>` and `before_host_filter_cloud_sha256=<SHA256 of raw PointCloud2.data bytes>`. These allow an exact identity/content join; never treat the two different clouds as identical bytes.
- Raw copy is made in SDK reportImage BEFORE doBaseFilter; raw XYZ uses a per-call geometry bypass, so lazy lens-map initialization cannot re-enable corner cuts for the raw copy. Same callback owns both ROS buffers.
- Both envelopes record host_filter_and_dispatch_elapsed_ns. It covers raw geometry, filters, filtered geometry and callback queue dispatch, not physical measurement latency. Header stamp remains arrival_only and common time remains invalid.

Root integration must record both source_frame topics and join them by exact raw key; feed filtered points to registration but retain original raw observations/origins for map reconstruction. Independent processed raw-archive naming would misrepresent data.

## Before/after evidence and root execution

Each identity-locked, endpoint-checked session writes device_before.txt, configuration_commands.txt, device_after.txt and device_readback.txt with all requested dispositions, effective flags, separate representation hashes and native provenance. Read-only probes do not invoke imaging/host apply. The run remains arrival_only and does not bypass missing real extrinsics/time validity.

Local preparation validated source/patch reverse applicability and both pinned binary hashes via the production checker. Four Python checks ran successfully through direct fixture invocation; pytest's Windows mode-0700 temp creation is blocked by local ACL and produced two setup errors, so this is not recorded as a clean pytest run. No local C++ compiler/ROS environment was present. No hardware, sensor socket or remote compilation was performed during this local preparation.

Root should run normal isolated build with new runtime present, then CTest including `xtcfg_test` (pure 46-key/parameter/256-command/five-frequency policy) and `xt_filter_runtime_test` (actual matching filter library configuration plus synthetic raw preservation, no startup/device). Run tests/sensors/test_xtcfg_binding.py through target pytest plus the existing sensor suite. Check new .so dependencies on Orin before starting hardware. Existing SDK test results apply to the previous version until rerun. Any failure is to be repaired, not skipped.

Deployment must preserve existing SDK original binaries and upload only the changed reviewed text files plus the NEW runtime directory. `vendor_patches/xtcfg_deploy_files.json` lists all changed relative SDK paths and raw hashes. Do not copy SDK .git or the reference checkout to masquerade as target source history. Patch bytes must remain LF as hashed; normal source hashes allow CRLF-to-LF only.

## Hidden native dependencies found by target execution

The first actual filter runtime test failed loading `_ZTIN3tbb4taskE`. DT_NEEDED alone was insufficient: ELF dynamic symbols show 17 legacy TBB imports and two OpenCV affine-estimation imports without corresponding declared dependencies. The checked official SDK CMake at commit 3d3db067 links OpenCV and dl, but has no explicit TBB dependency. Read-only Orin inspection found existing `/usr/lib/aarch64-linux-gnu/libtbb.so.2` exports the required old task RTTI/arena/destroy symbols; the unversioned `.so` instead resolves to oneTBB `.so.12.5`. No TBB library existed in the project OpenCV sysroot.

The project CMake now resolves exact `libtbb.so.2` and links it with a scoped `--no-as-needed`, preserving its dynamic dependency and global symbol visibility for the filter's dlopen. No library is installed, replaced or preloaded through environment variables. The actual selected path and SHA256 are part of generated build provenance and reconfiguration inputs. Existing OpenCV linking supplies the calibration imports. Native filter tests explicitly check all 19 relevant symbols via RTLD_DEFAULT before constructing the filter, then still execute real filtering and raw-preservation tests. A missing dependency fails, never disables the test or silently disables filtering. Target rebuild and execution are still required after this fix.
