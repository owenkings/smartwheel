# Panel backend contract

`wc_panel.backend.PanelBackend(project_root)` is the sole GUI service entry.
All results are JSON-shaped dictionaries/lists. Exceptions are user-displayable
`ValueError`, `RuntimeError`, or `OSError`. Discovery, plans and storage scans run
on UI workers. No method starts hardware before explicit `start_capture/live`.

- `settings()` returns `recording_root`, `results_root`; `update_settings(recording_root, results_root)` validates both pre-existing, independent absolute directories and atomically persists `config/panel.local.json`.
- `capabilities()` returns `estimators[{value,label}]`, `clouds`, `sides`, `process_noise`, `conditions`, `offline_variants` and disabled `official_geometry`. Estimator values: `robot_localization|five_state`; noise: `legacy|white_acceleration`.
- `list_recordings(root=None)` returns `{id,name,path,status,complete,bytes,sides,reason}`. A valid listing requires runtime configuration and SQLite bag parts. Incomplete data are labeled and never treated as complete.
- `list_results(root=None)` returns `{id,name,path,status,bytes,images,clouds,trajectories,dataset,group_id,run,cell,image_roles}`. One row represents one experimental cell, including nested comparisons and the numbered 01–12 summary/gallery layout. Indexes of the same archived run/cell merge without duplicating results. `group_id` is the displayed summary ID when available; `id` remains its unique relative cell path. `image_roles` maps indexed `top`, `grid`, `3d`, and `trajectory` figures to their absolute paths. Other images remain in `images`. Clouds include PLY/PCD and `.ply.xz`/`.pcd.xz`, which viewers must stream without silently extracting to storage. Archived `retained_trajectory` identifies relocated supplement cells; stale `/dev/shm` paths are never followed. A missing cloud remains an empty list rather than borrowing another cell's same-named file.
- `plan_offline(dataset,output_root=None,options=None)` returns `{id,dataset,output_root,output,tasks,task_count,comparison_count,options,created_at}`. `options.variants` selects capability IDs; `all:true` chooses 22 supported variants. Otherwise accept `estimator,cloud,filtering,motion_correction,process_noise,geometry`. Optional `input_rate_hz`, explicit `allow_partial`, and explicit `mechanical_initial`. Plans do not create results or start processes. One verified same-session candidate stage precedes corrected comparisons; both EKFs use the same candidate. Unsupported official geometry stays visible and disabled. PSD/geometry combinations use the actual refine capabilities, with no fabricated Cartesian product.
- `start_offline(plan)` only accepts the unmodified plan issued by this backend. Generates a unique session output, records plan/configuration, and queues actual compare/refine processes.
- `start_capture(output_root=None,sides='all',manual_drive=True)` starts a unique date/time child folder, mapping-core capture, user-stopped duration, memory staging and unified preview. The panel preserves the established user-operated WASD/hand-push workflow with `--manual-drive`; this does not authorize an assistant to drive. A hardware read-only acceptance run must explicitly call `manual_drive=False`, or use the capture CLI without `--manual-drive`. Its preview has no control widget. RAM/tmpfs reserve limits still apply; duration 0 is not an unlimited-capacity guarantee. `start_live(options)` accepts `mode:preview|mapping`, sides, cloud, estimator, noise and boolean correction/geometry. Uses the project's normal mapping supervisor and close/save dialog.
- `jobs()` returns `{id,kind,label,status,progress,step,total_steps,log_path,error,output,started_at,ended_at,task_states}`. `progress` is 0..1 completed-stage fraction, not guessed frame progress. Each task state has id/label/status/error/output and an `exit_code` once available. `cancel(job_id)` requests SIGINT of the owned supervisor and returns its current state; STOPPING remains active until finalization. Offline cancellation is CANCELLED; orderly live/capture stop is COMPLETE. `read_log(job_id,offset=0)` returns `{text,offset,status}` with byte offsets.
- `device_parameters()` returns `{revision,identity,groups}`. Groups have `id,label,value,warning_required,effective,path,conditions`. IDs: `extrinsics,wheels,display,algorithms`; device identities are read-only. UI shows the warning **before enabling parameter edits**. After user confirmation call `request_parameter_edit(group)` to get `{token,message,revision}`. `save_parameters(group,value,warning_token=None,expected_revision=None)` requires the read revision and warning token for extrinsics/wheels. Returns `{revision,backup,effective}`. Existing runtime validators remain authoritative; unknown transforms remain unknown. Original bytes and before/after revisions are kept in configured reports/panel/parameter_revisions. Existing sessions keep their frozen configuration; changes apply to next task.
- `scan_storage(root)` returns `{root,bytes,free_bytes,disk_total_bytes,items}`; items include name/path/bytes/kind/active. `prepare_delete(root,paths)` accepts only explicitly selected direct child directories of the currently configured data roots, returning `{token,root,paths,bytes,file_count,expires_at}`. Show this manifest and ask confirmation, then `execute_delete(token)`. It rechecks destination identity, activity and exact inventory. It does not follow symlinks, cross nested mounts, add new files to the selection, or delete a root. One-time token expires after 5 min.

`wc_panel.storage.resolve_user_destination(project_root,path,must_exist=True)`
returns `(Path, guard)` for runtime integration. The guard must be retained and
checked throughout work. USB identity remains mandatory under its configured
mount even after loss. Explicit other mounted block devices bind their UUID;
ordinary internal directories use a session-bound directory guard. No fallback.

## Evidence-backed live capabilities and calibration

`capabilities()` also returns `live_mapping`, `live_motion_correction`, and
`live_geometry`. Each has `enabled`, `reason`, and
`by_sides: {all:{enabled,reason},left:{enabled,reason},right:{enabled,reason}}`.
These are software/configuration capability checks, not a successful hardware
startup claim. Mapping remains unavailable for a side without its measured
`T_axle_lidar`; native preview remains available. Correction/PSD/geometry cannot
be selected in preview. Official geometry stays visible and disabled.

The `live_motion` parameter group requires the warning token and revision. Its
value is `{calibration, gyro_bias_config, calibration_evidence_path}`. Unknown
values are null; no numerical device correction is invented.
`load_live_calibration(calibration_path,gyro_bias_config,calibration_evidence_path=None)`
reads and validates an explicit device declaration and the independent gyro
confirmation plus their respective same-name `.evidence.json` files. Missing
or altered hashes, wrong devices, a mismatching gyro bias, and session-only
offline candidates are rejected. The optional evidence path defaults to the
declaration's same-name file. It returns the group value without writing.

Saving copies the four files under `config/calibration/panel_live/<revision>/`
and writes **only two references** in `mapping_live.json`:
`live_motion_calibration_config` and `gyro_bias_config`. The runtime independently
validates and snapshots the declarations and evidence again. Device scope is
operator-declared; SHA matches do not independently prove stationarity or
physical correction accuracy. A changed referenced file also changes the
parameter revision and invalidates pending edit tokens.

## Queue lifecycle and recorded configuration

`list_logs(job_id)` returns `{id,label,path,bytes,available,error}` entries for
the task main log and `.log` files under registered offline stage outputs.
`read_log(job_id,offset=0,log_id=None)` retains the old runtime-log default and
adds `log_id,available,error` to its response. Pass an ID obtained from the list;
arbitrary paths are rejected. A completed wrapper can have an empty main log
while `native/<cell>/native.log` holds RTAB-Map output, so UI should initially
select an available nonempty stage log when the main log is empty. Switching
logs resets the byte offset. Unsafe links, nested mounts and unreadable stages
are reported individually without hiding readable logs; original destination
guards are checked when present. Discovery is bounded to depth 8 / 8192 entries
per stage. A missing source is unavailable, never an empty successful read.

Every planned fusion task records `native_rate_hz` and
`native_wall_interval_s`. Compare explicitly receives the chosen input rate as
its native consumption cap; refine uses that same input rate internally. Both
use a 0.2 s native wall interval, so a mixed queue does not inherit compare's
different CLI default and silently consume fewer frames for baseline cells.

On the target Linux system, offline jobs run in detached `wc_panel.job_worker`
processes. A kernel lock serializes them in submission order. Closing and
reopening the panel does not stop the offline queue; status/log reads attach
to the same worker after checking PID, process start ticks and boot identity.
The parent writes that spawn identity into a separate `worker_launch.json`
receipt before returning submission; the worker later records its own identity
in `job.json`. Reopening during bootstrap accepts the launch receipt without
overwriting the worker's status. The small pre-receipt window remains QUEUED
for at most 10 seconds; an unclaimed expired launch becomes INTERRUPTED.
Cancellation uses a durable per-job request consumed by that worker and then
signals only its owned active command for normal cleanup. GUI close is not
system shutdown recovery: an exited or reboot-lost worker is shown INTERRUPTED,
and its work is never silently restarted. Live acquisition, recording, and
map recovery continue to require normal managed closure from their windows.

Submission failures retain a FAILED job and any created result manifest, with
the cause in `error` and the GUI-visible `runtime.log`; they do not remain QUEUED
or keep data marked active. The same log now includes detached-supervisor output.
If the log destination disappears, the operation fails without recreating its
directory elsewhere. `submit` raises a user-displayable error after recording
the failure when the destination remains writable.

Stopping live/capture/recovery still checks the command exit code: requesting
stop never turns a failed close into COMPLETE. Accepted code 3 means PENDING
for both the job and its task (save choice unfinished); code 0 means a normal
close, and capture additionally requires its final COMPLETE durability manifest.
Capture jobs expose `phase` and `stage_message`: normal RViz closure is observed
through the owned `progress.json`, and STOPPING_SOURCES, TRANSFERRING and VERIFYING
all show job status STOPPING until finalization finishes. The output path remains
the selected final destination throughout memory staging and transfer.

Offline **algorithm choices** come from the submitted plan. Extrinsics, wheel
parameters and base algorithm values come from the **recording's frozen
configuration**, not later edits to the project's current parameters.
`plan.recorded_configuration` documents this distinction and contains the
original configuration/manifest SHA-256 values. Submission and every queued
task verify those hashes before starting. Choosing the mechanical-initial
experiment acts on the archived setup; it does not promote the current live
mounts or silently replace them.

## Recovery and disk accounting

`list_recoverable_maps()` lists this project's temporary mapping sessions under
the configured `reports/maps`, excluding active sessions and saved/discarded
data. Rows have `id,path,status,recoverable,reason`. Normal stopped sessions with
an unfinished save decision are SAVE_PENDING and recoverable. Incomplete stop
or close evidence produces RECOVERY_BLOCKED with the exact reason; detection
does not imply the database can safely be exported.
`recover_map(path)` revalidates and launches the existing `scripts/save_map`
desktop save/discard dialog as a managed recovery job, without starting devices.
Cancelling a save dialog yields PENDING instead of claiming save success.

Storage inventories and deletion previews include `allocated_bytes` from
`st_blocks * 512`, or null where the platform cannot report allocation.
`bytes` remains logical file size. A successful delete returns
`free_before_bytes`, `free_after_bytes`, and `free_delta_bytes`; the latter is
an observed filesystem change that may include concurrent writers. Do not
label logical file size as measured space released.

`scan_storage` includes root-level ordinary files and hidden entries. Each item
also has `entry_type`, `status`, `error`, `problems[{path,error}]`, `deletable`,
`total_complete`, `known_bytes`, `known_allocated_bytes`, and
`allocation_available`. Links, junctions, nested mounts (including bind mounts),
and unreadable entries are reported without following them or losing other rows.
Incomplete totals are null; `known_*` values describe only the measured portion.
Top-level `total_complete` and `unknown_items` describe completeness of size
accounting. `active:null` separately means process occupancy could not be checked
and must block deletion even if byte accounting completed. Allocated totals
include the root directory's own `root_allocated_bytes` when available.
Scattered files and hidden entries are informational: deletion remains restricted
to non-hidden, ordinary, explicitly selected direct child directories. UI must
honor `deletable`; backend revalidates every destructive request independently.

## Explicit external extrinsics evidence

The `extrinsics` parameter group additionally supplies `assembly_revision`,
`source_documents` and `allowed_statuses`. User choices remain explicit:
`UNKNOWN`, `CAD_NOMINAL`, `LEGACY_CANDIDATE`, `USER_MEASURED_EXPERIMENT`, or
`CALIBRATED`. Importing evidence never fills numeric values or changes status.

`prepare_extrinsic_evidence(path,evidence_type,assembly_revision,note,expected_revision)`
checks an explicit ordinary local file (nonempty, at most 16 MiB), the current
assembly revision, and the user's description of its measurement scope. Types
are `GEOMETRY_MEASUREMENT` and `EXTRINSIC_CALIBRATION`. It returns
`{token,source,revision,physical_accuracy_validated:false,status:'PENDING_PARAMETER_SAVE'}`.
The source includes a generated ID, destination snapshot path, SHA-256, type,
assembly, note and original path. This is a read-only preview with an opaque
15-minute token bound to the current parameter revision; no configuration is
changed yet.

UI explicitly selects which transform's translation/rotation evidence refers
to `source.id`, preserving all existing numeric values and statuses. New files
must not inherit unrelated old document line numbers. Then
`save_parameters('extrinsics',value,warning_token,expected_revision,evidence_tokens=[...])`
validates each token is referenced, rechecks original bytes, copies snapshots
under `config/calibration/panel_extrinsics/`, registers the sources, validates
all source hashes/geometry, and backs up/atomically writes hardware_setup.json.
Unreferenced, stale, changed or expired imports fail. A failed final validation
does not leave the proposed configuration installed. Existing snapshot changes
invalidate open edits even when hardware_setup.json itself is unchanged.

UNKNOWN still requires null; experimental use requires explicit measured
status and a current-assembly measured source. CALIBRATED additionally requires
an EXTRINSIC_CALIBRATION source. Hash/file checks verify provenance and declared
applicability, not physical accuracy or the truth of operator assertions.

## 2026-10-08 安装参数、配置集与命名录制

- `device_parameters()` 增加只读 `installation` 快照，供主要安装量与离线初值释义使用。保存仍经过原参数组校验、依据绑定和备份。
- `list_configuration_profiles()`、`save_configuration_profile(name, expected_revision)`、`preview_configuration_profile(id)`、`apply_configuration_profile(token)`：同设备命名参数集；预览版本绑定、活动实时/录制阻止切换，多文件切换备份/回滚及重开恢复。
- `start_capture(..., manual_drive=True, name_options=None)`：命名选项为 `{prefix, year, date, time}`。None 保持旧命名行为。用户目录与内部 session_id 分离；CLI `capture --folder-name` 可显式指定最终目录名，手动驾驶自定义名称使用 memory staging。
- `prepare_pairing_capture(dataset)`：只读取 COMPLETE 双雷达原始录包，准备精确绑定该会话的独立 prepared.json；准备期间保护输入/输出免于存储清理。新录制使用 `manual_drive=False`，不启动驾驶链路。

配置与点云准备使用现有外部存储身份检查；失联不改写内部磁盘。上述界面和软件测试不能代替现场静态录制与人工移动验证。
