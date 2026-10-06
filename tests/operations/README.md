# Operations test evidence

Run only on the already confirmed Linux target, from the deployed project:

```bash
cd /home/nvidia/wheelchair
PYTHONNOUSERSITE=1 PYTHONPATH=/home/nvidia/wheelchair/src /usr/bin/python3 -m pytest -v tests/operations
```

The tests start actual Linux Python processes and exercise the production
`wc_runtime.supervisor` and `wc_runtime.component`, real flock contention,
signals, pidfds, subreaping, stop, and finite duration. They do not start ROS,
RViz, native sensor drivers, serial devices, motor commands, or builds.
Operations tests must run serially; stubborn descendants deliberately ignore
SIGINT/SIGTERM so several cases need about eight seconds to reach SIGKILL.

Each process test creates a distinct UUID directory below
`.phase1_runtime/sessions/ops-test-*/operations/`. It preserves `plan.json`,
`manifest.json` where one was created, `supervisor.log`, component logs, and a
`helper-pids.jsonl` with PID/start ticks/token/ancestry. Invalid plans may be
rejected before a manifest exists. These directories contain synthetic process
test evidence, never sensor data or mapping acceptance.

Covered cases include:

- Stop targets the registered supervisor and leaves an unrelated process alive.
- Stale PID start ticks and command-line substring impersonation are rejected.
- Duration ends normally; negative, NaN, and infinite durations launch nothing.
- A nonzero component exit remains a failure when a peer exits zero.
- Normal stop, component failure, and supervisor SIGKILL leave no live owned
  wrapper or helper descendant, including a stubborn grandchild using setsid.
- Device/resource ownership locks remain held throughout orphan cleanup and
  become available after the surviving wrapper has finished cleanup.
- Lock contention fails before any component helper starts.
- Cleanup refuses to signal a live process outside its own ancestor tree.
- Graceful stop sends SIGINT only to the wrapper and then its direct command;
  an actual test launcher forwarding that signal cannot cause its descendant
  to receive a duplicate SIGINT. TERM/KILL still clean orphaned survivors.
- Duplicate session/role creation leaves all original plan/log bytes intact.
- CLI errors return nonzero; forwarded help executes the actual downstream
  parser; unknown CLI options cannot dispatch acquisition/ROS.
- User arguments remain separate shell arguments; replay topics exclude
  control, historical TF, and historical pose channels.

`cli_probe.py` explicitly replaces the target check and runtime dispatch only
for parser tests, recording any attempted dispatch. This does not validate
target identity, device preflight, ROS startup, or graphics. No mocked ROS or
hardware success is reported. Failure cleanup signals only this test's recorded
PID identities using pidfd and start ticks, and runs after assertions; it is
never counted as successful product cleanup.

The component wrapper uses Linux PR_SET_CHILD_SUBREAPER so orphaned descendants
remain its children even across setsid/double fork, and PR_SET_PDEATHSIG so a
supervisor death enters the same bounded cleanup path. Signals go through
pidfd_send_signal after identity and current ancestry checks. The wrapper stays
alive to reap during SIGKILL cleanup rather than killing its own process group.
Kernel tasks stuck in uninterruptible sleep, an externally SIGKILLed wrapper,
or elevated processes that revoke signal permission cannot be guaranteed by a
userspace wrapper; incomplete cleanup is an error, never a PASS.

Primary API references:

- https://man7.org/linux/man-pages/man2/PR_SET_CHILD_SUBREAPER.2const.html
- https://man7.org/linux/man-pages/man2/PR_SET_PDEATHSIG.2const.html
- https://docs.python.org/3.10/library/signal.html#signal.pidfd_send_signal

Status at source handoff: statically reviewed, not executed on Windows. Root
owns deployment and actual Orin pytest evidence and reports the resulting level.
