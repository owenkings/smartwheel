import ctypes
import importlib
import json
import math
import os
import platform
import statistics
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np


def _candidate_libstdcxx_paths() -> List[Path]:
    candidates: List[Path] = []
    explicit = os.environ.get("XTSDK_LIBSTDCXX_PATH", "")
    if explicit:
        candidates.append(Path(explicit).expanduser())

    for prefix in (
        os.environ.get("CONDA_PREFIX", ""),
        str(Path.home() / "miniforge3"),
        str(Path.home() / "miniconda3"),
        str(Path.home() / "anaconda3"),
    ):
        if prefix:
            candidates.append(Path(prefix).expanduser() / "lib" / "libstdc++.so.6")
    return candidates


def _preload_libstdcxx() -> None:
    for candidate in _candidate_libstdcxx_paths():
        if not candidate.exists():
            continue
        try:
            ctypes.CDLL(str(candidate), mode=ctypes.RTLD_GLOBAL)
            return
        except OSError:
            continue


if platform.system() == "Linux":
    _preload_libstdcxx()

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2, PointField
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Float64, Header, String
except ImportError:
    rclpy = None
    ExternalShutdownException = RuntimeError
    Node = object
    PointCloud2 = None
    PointField = None
    point_cloud2 = None
    Header = None
    Float64 = None
    String = None


XYZI = Tuple[float, float, float, float]


def _stamp_pair_to_dict(stamp: Optional[Tuple[int, int]]) -> Optional[dict]:
    if stamp is None:
        return None
    sec, nanosec = stamp
    return {"sec": int(sec), "nanosec": int(nanosec)}


def build_timing_diagnostic(
    *,
    sdk_stamp: Optional[Tuple[int, int]],
    host_receive_wall_time_ns: Optional[int],
    cloud_header_stamp: Optional[Tuple[int, int]],
    publish_wall_time_ns: Optional[int],
    timestamp_source: str,
) -> str:
    """Serialize per-frame timestamp provenance for H1 offline correlation."""
    return json.dumps(
        {
            "sdk_timestamp": _stamp_pair_to_dict(sdk_stamp),
            "host_receive_wall_time_ns": (
                None
                if host_receive_wall_time_ns is None
                else int(host_receive_wall_time_ns)
            ),
            "cloud_header_stamp": _stamp_pair_to_dict(cloud_header_stamp),
            "publish_wall_time_ns": (
                None if publish_wall_time_ns is None else int(publish_wall_time_ns)
            ),
            "timestamp_source": str(timestamp_source),
        },
        separators=(",", ":"),
        sort_keys=True,
    )


@dataclass(frozen=True)
class CloudQualityResult:
    accepted: bool
    reason: str
    valid_fraction: float
    baseline_valid_fraction: Optional[float]
    overlap_fraction: float
    median_range_delta: Optional[float]
    p95_range_delta: Optional[float]
    temporal_jump_detected: bool
    relative_validity_drop_detected: bool
    accepted_frames: int
    dropped_frames: int


class CloudFrameQualityGate:
    """Reject transient organized-cloud frames that are unsafe for mapping.

    The raw XT-M60 topic remains untouched. This gate protects a separate
    validated topic using scene-independent absolute validity plus temporal
    pixel-range stability. The temporal checks are only meaningful for the
    organized 160x60 grid, where a pixel keeps the same ray between frames.
    """

    def __init__(
        self,
        *,
        enabled: bool = True,
        min_valid_fraction: float = 0.50,
        min_relative_valid_fraction: float = 0.75,
        baseline_window_frames: int = 30,
        max_median_range_delta: float = 0.10,
        max_p95_range_delta: float = 0.75,
        min_overlap_fraction: float = 0.20,
        reset_after_gap_sec: float = 0.50,
        temporal_hard_reject: bool = True,
    ):
        self.enabled = bool(enabled)
        self.min_valid_fraction = float(min_valid_fraction)
        self.min_relative_valid_fraction = float(min_relative_valid_fraction)
        self.max_median_range_delta = float(max_median_range_delta)
        self.max_p95_range_delta = float(max_p95_range_delta)
        self.min_overlap_fraction = float(min_overlap_fraction)
        self.reset_after_gap_sec = float(reset_after_gap_sec)
        self.temporal_hard_reject = bool(temporal_hard_reject)
        if not 0.0 <= self.min_valid_fraction <= 1.0:
            raise ValueError("min_valid_fraction must be in [0, 1]")
        if not 0.0 <= self.min_relative_valid_fraction <= 1.0:
            raise ValueError("min_relative_valid_fraction must be in [0, 1]")
        if not 0.0 <= self.min_overlap_fraction <= 1.0:
            raise ValueError("min_overlap_fraction must be in [0, 1]")
        if self.max_median_range_delta <= 0.0:
            raise ValueError("max_median_range_delta must be > 0")
        if self.max_p95_range_delta <= 0.0:
            raise ValueError("max_p95_range_delta must be > 0")
        if self.reset_after_gap_sec <= 0.0:
            raise ValueError("reset_after_gap_sec must be > 0")

        self._valid_history = deque(maxlen=max(1, int(baseline_window_frames)))
        self._previous_ranges: Optional[np.ndarray] = None
        self._previous_receive_monotonic: Optional[float] = None
        self.accepted_frames = 0
        self.dropped_frames = 0

    @staticmethod
    def _ranges(points: Sequence[XYZI]) -> np.ndarray:
        try:
            array = np.asarray(points, dtype=np.float64)
        except (TypeError, ValueError):
            return np.full(len(points), np.nan, dtype=np.float64)
        if array.ndim != 2 or array.shape[0] != len(points) or array.shape[1] < 3:
            return np.full(len(points), np.nan, dtype=np.float64)
        xyz = array[:, :3]
        valid = np.all(np.isfinite(xyz), axis=1)
        ranges = np.linalg.norm(xyz, axis=1)
        valid &= np.isfinite(ranges) & (ranges > 0.0)
        ranges[~valid] = np.nan
        return ranges

    def evaluate(
        self,
        points: Sequence[XYZI],
        receive_monotonic: Optional[float],
        *,
        organized: bool,
        ranges_override: Optional[np.ndarray] = None,
    ) -> CloudQualityResult:
        candidate = (
            np.asarray(ranges_override, dtype=np.float64)
            if ranges_override is not None
            else None
        )
        ranges = (
            candidate
            if candidate is not None
            and candidate.ndim == 1
            and candidate.size == len(points)
            else self._ranges(points)
        )
        total = int(ranges.size)
        valid_count = int(np.count_nonzero(np.isfinite(ranges)))
        valid_fraction = (valid_count / total) if total else 0.0
        baseline = (
            float(statistics.median(self._valid_history))
            if self._valid_history
            else None
        )

        receive_time = (
            float(receive_monotonic)
            if receive_monotonic is not None
            else math.nan
        )
        time_nonmonotonic = False
        compare_temporally = False
        if (
            organized
            and self._previous_ranges is not None
            and self._previous_ranges.size == total
            and math.isfinite(receive_time)
            and self._previous_receive_monotonic is not None
        ):
            delta_time = receive_time - self._previous_receive_monotonic
            time_nonmonotonic = delta_time <= 0.0
            compare_temporally = (
                not time_nonmonotonic
                and delta_time <= self.reset_after_gap_sec
            )

        overlap_fraction = 0.0
        median_delta: Optional[float] = None
        p95_delta: Optional[float] = None
        if compare_temporally:
            overlap = np.isfinite(ranges) & np.isfinite(self._previous_ranges)
            overlap_count = int(np.count_nonzero(overlap))
            overlap_fraction = (overlap_count / total) if total else 0.0
            if overlap_count and overlap_fraction >= self.min_overlap_fraction:
                deltas = np.abs(ranges[overlap] - self._previous_ranges[overlap])
                median_delta = float(np.median(deltas))
                p95_delta = float(np.percentile(deltas, 95))

        reasons: List[str] = []
        temporal_bad = False
        relative_bad = False
        if self.enabled:
            if time_nonmonotonic:
                reasons.append("nonmonotonic_receive_time")
            if valid_fraction < self.min_valid_fraction:
                reasons.append("absolute_valid_fraction")
            temporal_bad = (
                median_delta is not None
                and median_delta > self.max_median_range_delta
            ) or (
                p95_delta is not None
                and p95_delta > self.max_p95_range_delta
            )
            relative_bad = (
                baseline is not None
                and valid_fraction
                < baseline * self.min_relative_valid_fraction
            )
            if temporal_bad and self.temporal_hard_reject:
                reasons.append("temporal_range_jump")
            # Cross-frame changes are legitimate while the chair turns or
            # translates. In report-only mode keep the metrics/flags but reject
            # only per-frame corruption (absolute validity) or broken time.
            if self.temporal_hard_reject and relative_bad and (
                temporal_bad or overlap_fraction < self.min_overlap_fraction
            ):
                reasons.append("relative_valid_fraction")

        accepted = not reasons
        if accepted:
            self.accepted_frames += 1
            self._valid_history.append(valid_fraction)
        else:
            self.dropped_frames += 1

        # Compare the next frame with what the sensor actually emitted, not
        # with an older accepted frame. This lets the gate recover immediately
        # after a brief transient instead of amplifying it indefinitely.
        self._previous_ranges = ranges.copy() if organized else None
        self._previous_receive_monotonic = (
            receive_time if math.isfinite(receive_time) else None
        )
        return CloudQualityResult(
            accepted=accepted,
            reason="ok" if accepted else ",".join(reasons),
            valid_fraction=valid_fraction,
            baseline_valid_fraction=baseline,
            overlap_fraction=overlap_fraction,
            median_range_delta=median_delta,
            p95_range_delta=p95_delta,
            temporal_jump_detected=temporal_bad,
            relative_validity_drop_detected=relative_bad,
            accepted_frames=self.accepted_frames,
            dropped_frames=self.dropped_frames,
        )


@dataclass
class XTM60SdkConfig:
    sdk_root: str = ""
    connection_mode: str = "ethernet"
    ip_address: str = "192.168.0.101"
    serial_port: str = ""
    image_type: int = 4
    frame_id: str = "laser_link"
    point_unit_scale: float = 1.0
    range_min: float = 0.05
    range_max: float = 20.0
    publish_intensity: bool = True
    organized_cloud: bool = True
    enable_sdk_filters: bool = True
    kalman_factor: int = 300
    kalman_threshold: int = 200
    kalman_range: int = 2000
    median_size: int = 3
    edge_threshold: int = 150
    # Extra SDK filters matching the XT-Toffuture upper-computer M60 defaults
    # (dev_scene.ini [M60.Scene1.Filters]). 0/false disables each one.
    dust_enable: bool = True
    dust_threshold: int = 9000
    dust_frames: int = 2
    postprocess_enable: bool = True
    postprocess_threshold: float = 5.0
    postprocess_dynamic_enable: int = 1
    postprocess_dynamic_winsize: int = 9
    reflective_enable: bool = True
    reflective_th_min: float = 0.5
    reflective_th_max: float = 2.0
    # --- Device-side IMAGING params (pushed to the radar on connect, matching
    #     the upper-computer "Config Device" flow / xintan.xtcfg). These set how
    #     the sensor actually images: exposure (integration times), HDR, minimum
    #     amplitude threshold, max fps, modulation frequencies. The ROS adapter
    #     previously did NOT set these, so the radar used its power-on defaults
    #     -> ROS clouds looked worse than the upper-computer's tuned version.
    #     apply_device_config=false keeps the device's stored settings.
    #     verify_device_config can still fail closed on stored-setting drift
    #     without writing anything to the radar. ---
    apply_device_config: bool = False
    verify_device_config: bool = False
    require_device_config_match: bool = False
    expected_serial: str = ""
    int_time_gs: int = 2000      # grayscale integration time (us)
    int_time_1: int = 1600       # HDR exposure 1 (us)
    int_time_2: int = 200        # HDR exposure 2 (us)
    int_time_3: int = 30         # HDR exposure 3 (us), verified dual baseline
    int_time_4: int = 1600       # HDR exposure 4 (us) -- xtcfg int4
    hdr_mode: int = 1            # 0=off, 1=temporal HDR
    min_amplitude: int = 70      # minLSB amplitude threshold
    max_fps: int = 10
    # Modulation frequencies (SDK enum indices, matching xintan.xtcfg freqN).
    # The official sdk_example_3d.py passes freq1..freq4 from the cfg plus a
    # hardcoded ModulationFreq(2) as the 5th arg -> mod_freq5 defaults to 2.
    mod_freq1: int = 0
    mod_freq2: int = 0
    mod_freq3: int = 0
    mod_freq4: int = 3
    mod_freq5: int = 2
    reconnect_interval_sec: float = 3.0
    require_ping_before_start: bool = True
    frame_timeout_sec: float = 8.0
    wait_for_phase_reference: bool = False
    start_on_phase_grid: bool = False
    phase_period_sec: float = 0.1
    phase_offset_sec: float = 0.05
    phase_realign_interval_sec: float = 0.0
    udp_dest_ip: str = ""
    udp_dest_port: int = 0


def _candidate_sdk_roots(configured_root: str) -> List[Path]:
    candidates: List[Path] = []
    for raw in (configured_root, os.environ.get("XTSDK_PY_ROOT", "")):
        if raw:
            candidates.append(Path(raw).expanduser())

    here = Path(__file__).resolve()
    for parent in here.parents:
        candidates.append(parent / "xtsdk_py-main")
        candidates.append(parent / "xtsdk_py")
        candidates.append(parent / "third_party" / "xtsdk_py-main")
        candidates.append(parent / "third_party" / "xtsdk_py")
        candidates.append(parent / "reference" / "xtsdk_py_main" / "xtsdk_py-main")
    return candidates


def find_xtsdk_root(configured_root: str = "") -> Path:
    for candidate in _candidate_sdk_roots(configured_root):
        if (candidate / "cfg").is_dir() and (candidate / "lib").is_dir():
            return candidate
    searched = "\n  ".join(str(p) for p in _candidate_sdk_roots(configured_root))
    raise FileNotFoundError(
        "xtsdk_py root not found. Set XTSDK_PY_ROOT or sdk_root. Searched:\n  "
        + searched
    )


def configure_xtsdk_import_path(sdk_root: Path) -> Path:
    system_name = platform.system()
    if system_name == "Windows":
        lib_dir = sdk_root / "lib" / "win32"
        if hasattr(os, "add_dll_directory"):
            os.add_dll_directory(str(lib_dir))
        os.environ["PATH"] = str(lib_dir) + os.pathsep + os.environ.get("PATH", "")
    elif system_name == "Linux":
        arch = platform.machine()
        if arch not in ("x86_64", "aarch64"):
            raise RuntimeError(f"unsupported Linux architecture for xtsdk_py: {arch}")
        lib_dir = sdk_root / "lib" / "linux" / arch
        shared = lib_dir / "libxtsdk_shared.so"
        if shared.exists():
            _preload_libstdcxx()
            ctypes.CDLL(str(shared), mode=ctypes.RTLD_GLOBAL)
    else:
        raise RuntimeError(f"unsupported OS for xtsdk_py: {system_name}")

    if not lib_dir.is_dir():
        raise FileNotFoundError(f"xtsdk library directory not found: {lib_dir}")

    for path in (lib_dir, sdk_root / "cfg"):
        text = str(path)
        if text not in sys.path:
            sys.path.insert(0, text)
    return lib_dir


def _import_xintan_sdk(sdk_root: Path):
    if "xintan_sdk" in sys.modules:
        return sys.modules["xintan_sdk"]
    import_cwd = sdk_root / "sdk_example"
    if not import_cwd.is_dir():
        return importlib.import_module("xintan_sdk")

    cwd = Path.cwd()
    try:
        os.chdir(import_cwd)
        return importlib.import_module("xintan_sdk")
    finally:
        os.chdir(cwd)


def _with_sdk_example_cwd(sdk_root: Path, func):
    run_cwd = sdk_root / "sdk_example"
    if not run_cwd.is_dir():
        return func()

    cwd = Path.cwd()
    try:
        os.chdir(run_cwd)
        return func()
    finally:
        os.chdir(cwd)


XT_AMPLITUDE_INVALID_MIN = 64000.0


def _extract_intensity(point, amplitudes: Optional[Sequence], index: int):
    """Return finite SDK amplitude, or None for the vendor invalid sentinel.

    XTSDK documents amplitude values 0..2039 as valid and values >=64000 as
    invalid-pixel markers. Preserve any intermediate firmware-defined values,
    but never publish the explicit invalid sentinel as a real intensity/point.
    """
    if amplitudes is not None and index < len(amplitudes):
        try:
            intensity = float(amplitudes[index])
        except (TypeError, ValueError):
            return None
    else:
        try:
            intensity = float(getattr(point, "i", 0.0))
        except (TypeError, ValueError):
            return None
    if not math.isfinite(intensity) or intensity >= XT_AMPLITUDE_INVALID_MIN:
        return None
    return intensity


def extract_xyzi_points(
    frame,
    unit_scale: float = 1.0,
    range_min: float = 0.05,
    range_max: float = 20.0,
) -> List[XYZI]:
    """Extract finite XYZI points from an XT SDK frame.

    The SDK frame points are treated as the lidar-frame coordinates. Unit scale
    is configurable because deployments should verify whether a given SDK build
    returns meters or millimeters.
    """
    if not getattr(frame, "hasPointcloud", False):
        return []
    raw_points = getattr(frame, "points", None)
    if not raw_points:
        return []

    amplitudes: Optional[Sequence] = getattr(frame, "amplData", None)
    scaled_min = max(0.0, float(range_min))
    scaled_max = max(scaled_min, float(range_max))
    scale = float(unit_scale)
    result: List[XYZI] = []

    for index, point in enumerate(raw_points):
        x = float(getattr(point, "x", math.nan)) * scale
        y = float(getattr(point, "y", math.nan)) * scale
        z = float(getattr(point, "z", math.nan)) * scale
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        distance = math.sqrt(x * x + y * y + z * z)
        if distance < scaled_min or distance > scaled_max:
            continue
        intensity = _extract_intensity(point, amplitudes, index)
        if intensity is None:
            continue
        result.append((x, y, z, intensity))
    return result


def extract_xyzi_grid(
    frame,
    unit_scale: float = 1.0,
    range_min: float = 0.05,
    range_max: float = 20.0,
    return_ranges: bool = False,
):
    """Extract an ORGANIZED XYZI cloud preserving the sensor's row/col grid.

    Returns (points, width, height). Every one of width*height pixels keeps its
    slot in row-major order; invalid / out-of-range pixels become (nan,nan,nan,0)
    instead of being dropped. This keeps the cloud "organized" so RViz/Open3D can
    treat it as a depth image (neighbours stay adjacent), which looks far denser
    and more surface-like than a shuffled, gap-filled flat list.

    Falls back to (None, 0, 0) if the frame has no usable grid dimensions, so the
    caller can use the unordered extractor instead.
    """
    if not getattr(frame, "hasPointcloud", False):
        return (None, 0, 0, None) if return_ranges else (None, 0, 0)
    raw_points = getattr(frame, "points", None)
    if not raw_points:
        return (None, 0, 0, None) if return_ranges else (None, 0, 0)
    width = int(getattr(frame, "width", 0) or 0)
    height = int(getattr(frame, "height", 0) or 0)
    if width <= 0 or height <= 0 or width * height != len(raw_points):
        # Grid dimensions missing or inconsistent with the point count.
        return (None, 0, 0, None) if return_ranges else (None, 0, 0)

    amplitudes: Optional[Sequence] = getattr(frame, "amplData", None)
    scaled_min = max(0.0, float(range_min))
    scaled_max = max(scaled_min, float(range_max))
    scale = float(unit_scale)
    nan = float("nan")
    grid: List[XYZI] = [None] * len(raw_points)
    ranges = (
        np.full(len(raw_points), np.nan, dtype=np.float64)
        if return_ranges
        else None
    )

    for index, point in enumerate(raw_points):
        x = float(getattr(point, "x", nan)) * scale
        y = float(getattr(point, "y", nan)) * scale
        z = float(getattr(point, "z", nan)) * scale
        valid = math.isfinite(x) and math.isfinite(y) and math.isfinite(z)
        if valid:
            distance = math.sqrt(x * x + y * y + z * z)
            if distance < scaled_min or distance > scaled_max:
                valid = False
        if not valid:
            grid[index] = (nan, nan, nan, 0.0)
            continue
        intensity = _extract_intensity(point, amplitudes, index)
        if intensity is None:
            grid[index] = (nan, nan, nan, 0.0)
            continue
        grid[index] = (x, y, z, intensity)
        if ranges is not None:
            ranges[index] = distance
    if return_ranges:
        return grid, width, height, ranges
    return grid, width, height


class XTM60SdkAdapter:
    """Small XT-M60 SDK wrapper for ROS2 publishing.

    The SDK owns the network/USB receiving thread and invokes callbacks. This
    wrapper keeps callbacks lightweight: copy the latest point frame under a
    lock, then let the ROS node publish it from a timer.
    """

    def __init__(self, config: XTM60SdkConfig, logger):
        self.config = config
        self.logger = logger
        self._sdk = None
        self._xintan_sdk = None
        self._lock = threading.Lock()
        self._latest_points: Optional[List[XYZI]] = None
        self._latest_frame_id: Optional[int] = None
        self._latest_sdk_stamp: Optional[Tuple[int, int]] = None
        self._latest_receive_monotonic: Optional[float] = None
        self._latest_receive_wall_time_ns: Optional[int] = None
        self._latest_quality_ranges: Optional[np.ndarray] = None
        self._latest_temperature_c: Optional[float] = None
        self._latest_vcsel_temperature_c: Optional[float] = None
        self._latest_grid: Tuple[int, int] = (0, 0)
        self._connected = False
        self._measurement_started = False
        self._last_connect_attempt = 0.0
        self._last_error = ""
        self._last_frame_time = 0.0
        self._last_restart_time = 0.0
        self._last_measurement_start_time = 0.0
        self._scheduled_measurement_start_time: Optional[float] = None
        self._last_phase_reference_time = 0.0
        self._last_udp_attempt = 0.0
        self._device_config_verified = not self.config.verify_device_config
        self._last_device_config_check = 0.0
        self._udp_dest_applied = not (
            self.config.connection_mode.lower().strip() == "ethernet"
            and self.config.udp_dest_port > 0
        )

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def measurement_started(self) -> bool:
        return self._measurement_started

    @property
    def last_error(self) -> str:
        return self._last_error

    def start(self) -> None:
        sdk_root = find_xtsdk_root(self.config.sdk_root)
        lib_dir = configure_xtsdk_import_path(sdk_root)
        self.logger.info(f"XT-M60 SDK root: {sdk_root}")
        self.logger.info(f"XT-M60 SDK lib: {lib_dir}")

        try:
            xintan_sdk = _import_xintan_sdk(sdk_root)
        except ImportError as exc:
            self._last_error = str(exc)
            raise RuntimeError(
                "failed to import xintan_sdk; check Python version and SDK binary"
            ) from exc

        self._xintan_sdk = xintan_sdk
        self._sdk = _with_sdk_example_cwd(sdk_root, xintan_sdk.XtSdk)
        self._sdk.setCallback(self._on_event, self._on_frame)
        self._apply_optional_filters()
        self._configure_connection()
        self._sdk.startup()
        self._last_connect_attempt = self._now()

    def stop(self) -> None:
        self._measurement_started = False
        self._scheduled_measurement_start_time = None
        self._connected = False
        self._device_config_verified = not self.config.verify_device_config
        sdk = self._sdk
        self._sdk = None
        if sdk is None:
            return
        try:
            sdk.stop()
        except BaseException as exc:
            self.logger.warning(f"XT-M60 SDK stop failed: {exc}")
        try:
            sdk.shutdown()
        except BaseException as exc:
            self.logger.warning(f"XT-M60 SDK shutdown failed: {exc}")

    def stop_and_exit_process(self, code: int = 0) -> None:
        """Stop the vendor SDK and bypass pybind destructors that can segfault on SIGINT."""
        self._measurement_started = False
        self._scheduled_measurement_start_time = None
        self._connected = False
        self._device_config_verified = not self.config.verify_device_config
        sdk = self._sdk
        self._sdk = None
        if sdk is not None:
            try:
                sdk.stop()
            except BaseException as exc:
                self.logger.warning(f"XT-M60 SDK stop failed: {exc}")
            try:
                sdk.shutdown()
            except BaseException as exc:
                self.logger.warning(f"XT-M60 SDK shutdown failed: {exc}")
        os._exit(code)

    def poll(self) -> None:
        if self._sdk is None:
            return
        try:
            connected = bool(self._sdk.isconnect())
        except Exception as exc:
            self._last_error = str(exc)
            self.logger.warning(f"XT-M60 SDK poll failed: {exc}")
            self._connected = False
            self._measurement_started = False
            self._device_config_verified = not self.config.verify_device_config
            self._udp_dest_applied = False
            return
        self._connected = connected
        if not connected:
            # Let the SDK self-reconnect (TCP re-handshake + UDP reopen, ~5-15s).
            # Do NOT destroy the SDK here; just re-issue start() once it is back.
            self._measurement_started = False
            self._scheduled_measurement_start_time = None
            self._device_config_verified = not self.config.verify_device_config
            if self.config.udp_dest_port > 0:
                self._udp_dest_applied = False
            return
        if not self._measurement_started:
            if not self._apply_udp_destination():
                return
            if (
                self.config.wait_for_phase_reference
                and self._scheduled_measurement_start_time is None
            ):
                return
            self._start_measurement()
            return
        realign_interval = float(self.config.phase_realign_interval_sec)
        if (
            (
                self.config.wait_for_phase_reference
                or self.config.start_on_phase_grid
            )
            and realign_interval > 0.0
            and self._last_measurement_start_time > 0.0
            and self._now() - self._last_measurement_start_time >= realign_interval
        ):
            self.logger.info(
                "XT-M60 phase-controlled stream pausing for scheduled realignment"
            )
            try:
                self._sdk.stop()
            except Exception as exc:
                self.logger.warning(f"XT-M60 phase realign stop failed: {exc}")
            self._measurement_started = False
            self._scheduled_measurement_start_time = None
            return
        # Connected and measuring but the UDP stream stalled: gently re-issue
        # start() (NOT a destroy) to kick it. Rate-limited and generous so we
        # never interfere with the SDK's own recovery.
        timeout = float(self.config.frame_timeout_sec)
        if timeout > 0:
            now = self._now()
            frame_reference = self._last_frame_time or self._last_measurement_start_time
            if (frame_reference > 0.0
                    and now - frame_reference > timeout
                    and now - self._last_restart_time > timeout):
                self.logger.warning(
                    f"XT-M60 connected but no frames for {now - frame_reference:.1f}s; re-issuing start()")
                self._last_restart_time = now
                if self.config.wait_for_phase_reference:
                    try:
                        self._sdk.stop()
                    except Exception as exc:
                        self.logger.warning(
                            f"XT-M60 phase follower stalled-stop failed: {exc}"
                        )
                    self._measurement_started = False
                    self._scheduled_measurement_start_time = None
                else:
                    self._start_measurement()

    def take_latest_points(self):
        with self._lock:
            points = self._latest_points
            stamp = self._latest_sdk_stamp
            grid = self._latest_grid
            receive_monotonic = self._latest_receive_monotonic
            receive_wall_time_ns = self._latest_receive_wall_time_ns
            quality_ranges = self._latest_quality_ranges
            temperature_c = self._latest_temperature_c
            vcsel_temperature_c = self._latest_vcsel_temperature_c
            self._latest_points = None
            self._latest_sdk_stamp = None
            self._latest_receive_monotonic = None
            self._latest_receive_wall_time_ns = None
            self._latest_quality_ranges = None
            self._latest_temperature_c = None
            self._latest_vcsel_temperature_c = None
            self._latest_grid = (0, 0)
        return (
            points,
            stamp,
            grid,
            receive_monotonic,
            receive_wall_time_ns,
            quality_ranges,
            temperature_c,
            vcsel_temperature_c,
        )

    def schedule_measurement_start_from_reference(
        self, reference_monotonic: float
    ) -> bool:
        if not self.config.wait_for_phase_reference or self._measurement_started:
            return False
        reference = float(reference_monotonic)
        if not math.isfinite(reference) or reference <= self._last_phase_reference_time:
            return False
        now = self._now()
        target = self._phase_target_after_reference(
            reference,
            now,
            float(self.config.phase_period_sec),
            float(self.config.phase_offset_sec),
        )
        self._last_phase_reference_time = reference
        self._scheduled_measurement_start_time = target
        return True

    @staticmethod
    def _phase_target_after_reference(
        reference: float, now: float, period: float, offset: float
    ) -> float:
        if not math.isfinite(period) or period <= 0.0:
            raise ValueError("phase_period_sec must be finite and > 0")
        if not math.isfinite(offset) or offset < 0.0 or offset >= period:
            raise ValueError("phase_offset_sec must satisfy 0 <= offset < period")
        target = float(reference) + offset
        if target <= now:
            target += (math.floor((now - target) / period) + 1) * period
        return target

    @staticmethod
    def _phase_target_on_host_grid(
        now: float, period: float, offset: float
    ) -> float:
        if not math.isfinite(period) or period <= 0.0:
            raise ValueError("phase_period_sec must be finite and > 0")
        if not math.isfinite(offset) or offset < 0.0 or offset >= period:
            raise ValueError("phase_offset_sec must satisfy 0 <= offset < period")
        return (math.floor((now - offset) / period) + 1) * period + offset

    def _configure_connection(self) -> None:
        mode = self.config.connection_mode.lower().strip()
        if mode == "ethernet":
            ok = self._sdk.setConnectIpaddress(self.config.ip_address)
            if not ok:
                self.logger.warning(f"XT-M60 setConnectIpaddress returned false: {self.config.ip_address}")
        elif mode == "usb":
            if not self.config.serial_port:
                raise ValueError("serial_port is required when connection_mode is usb")
            ok = self._sdk.setConnectSerialportName(self.config.serial_port)
            if not ok:
                self.logger.warning(f"XT-M60 setConnectSerialportName returned false: {self.config.serial_port}")
        else:
            raise ValueError("connection_mode must be ethernet or usb")

    def _apply_udp_destination(self) -> bool:
        if self._udp_dest_applied or self.config.udp_dest_port <= 0:
            return True
        now = self._now()
        if now - self._last_udp_attempt < max(0.1, self.config.reconnect_interval_sec):
            return False
        self._last_udp_attempt = now
        setter = getattr(self._sdk, "setUdpDestIp", None)
        if setter is None:
            self._last_error = "XT-M60 SDK does not provide setUdpDestIp"
            self.logger.error(self._last_error)
            return False
        try:
            ok = setter(self.config.udp_dest_ip, int(self.config.udp_dest_port))
        except Exception as exc:
            self._last_error = str(exc)
            self.logger.warning(f"XT-M60 setUdpDestIp failed: {exc}")
            return False
        if ok is False:
            self._last_error = (
                f"setUdpDestIp returned false for "
                f"{self.config.udp_dest_ip}:{self.config.udp_dest_port}"
            )
            self.logger.warning(f"XT-M60 {self._last_error}")
            return False
        self._udp_dest_applied = True
        self.logger.info(
            f"XT-M60 UDP dest set to {self.config.udp_dest_ip}:{self.config.udp_dest_port}"
        )
        return True

    def _apply_optional_filters(self) -> None:
        if not self.config.enable_sdk_filters:
            return
        calls = [
            ("setSdkMedianFilter", (int(self.config.median_size),)),
            ("setSdkEdgeFilter", (int(self.config.edge_threshold),)),
        ]
        for name, args in calls:
            func = getattr(self._sdk, name, None)
            if func is None:
                continue
            try:
                func(*args)
            except Exception as exc:
                self.logger.warning(f"XT-M60 optional filter {name} failed: {exc}")

        kalman = getattr(self._sdk, "setSdkKalmanFilter", None)
        if kalman is not None:
            try:
                kalman(
                    int(self.config.kalman_factor),
                    int(self.config.kalman_threshold),
                    int(self.config.kalman_range),
                )
            except TypeError:
                try:
                    kalman(int(self.config.kalman_factor), int(self.config.kalman_threshold))
                except Exception as exc:
                    self.logger.warning(f"XT-M60 optional filter setSdkKalmanFilter failed: {exc}")
            except Exception as exc:
                self.logger.warning(f"XT-M60 optional filter setSdkKalmanFilter failed: {exc}")

        # Extra filters matching the upper-computer (dust / postprocess /
        # reflective). Each is guarded by its own enable flag and skipped if the
        # SDK build does not expose the API.
        if self.config.dust_enable:
            dust = getattr(self._sdk, "setSdkDustFilter", None)
            if dust is not None:
                try:
                    dust(int(self.config.dust_threshold), int(self.config.dust_frames))
                except Exception as exc:
                    self.logger.warning(f"XT-M60 optional filter setSdkDustFilter failed: {exc}")

        if self.config.postprocess_enable:
            postprocess = getattr(self._sdk, "setPostProcess", None)
            if postprocess is not None:
                try:
                    postprocess(
                        float(self.config.postprocess_threshold),
                        int(self.config.postprocess_dynamic_enable),
                        int(self.config.postprocess_dynamic_winsize),
                    )
                except Exception as exc:
                    self.logger.warning(f"XT-M60 optional filter setPostProcess failed: {exc}")

        if self.config.reflective_enable:
            reflective = getattr(self._sdk, "setSdkReflectiveFilter", None)
            if reflective is not None:
                try:
                    reflective(
                        float(self.config.reflective_th_min),
                        float(self.config.reflective_th_max),
                    )
                except Exception as exc:
                    self.logger.warning(f"XT-M60 optional filter setSdkReflectiveFilter failed: {exc}")

    def _apply_device_config(self) -> None:
        """Push imaging params to the radar (exposure/HDR/minAmp/fps/modfreq).

        Mirrors the upper-computer "Config Device" flow and the official SDK
        example (sdk_example_3d.py): without this the radar uses its power-on
        defaults and the ROS cloud is dimmer/sparser than the tuned upper-
        computer view. Each call is guarded; a missing API or failure only warns.
        """
        if not self.config.apply_device_config:
            return
        sdk = self._sdk
        xs = self._xintan_sdk

        def _try(name, fn):
            try:
                fn()
            except Exception as exc:
                self.logger.warning(f"XT-M60 device-config {name} failed: {exc}")

        # Stop measurement before reconfiguring (official flow stops first).
        stop = getattr(sdk, "stop", None)
        if stop is not None:
            _try("stop", lambda: stop())

        set_int = getattr(sdk, "setIntTimesus", None)
        if set_int is not None:
            # Official sdk_example_3d.py: setIntTimesus(intgs, int1, int2, int3, int4, 0).
            # Fall back to the 4-arg form if the SDK build expects fewer args.
            def _do_set_int():
                try:
                    set_int(
                        int(self.config.int_time_gs), int(self.config.int_time_1),
                        int(self.config.int_time_2), int(self.config.int_time_3),
                        int(self.config.int_time_4), 0)
                except TypeError:
                    set_int(
                        int(self.config.int_time_gs), int(self.config.int_time_1),
                        int(self.config.int_time_2), int(self.config.int_time_3))
            _try("setIntTimesus", _do_set_int)

        set_hdr = getattr(sdk, "setHdrMode", None)
        if set_hdr is not None and xs is not None and hasattr(xs, "HDRMode"):
            _try("setHdrMode", lambda: set_hdr(xs.HDRMode(int(self.config.hdr_mode))))

        set_freq = getattr(sdk, "setMultiModFreq", None)
        if set_freq is not None and xs is not None and hasattr(xs, "ModulationFreq"):
            _try("setMultiModFreq", lambda: set_freq(
                xs.ModulationFreq(int(self.config.mod_freq1)),
                xs.ModulationFreq(int(self.config.mod_freq2)),
                xs.ModulationFreq(int(self.config.mod_freq3)),
                xs.ModulationFreq(int(self.config.mod_freq4)),
                xs.ModulationFreq(int(self.config.mod_freq5))))

        set_amp = getattr(sdk, "setMinAmplitude", None)
        if set_amp is not None:
            _try("setMinAmplitude", lambda: set_amp(int(self.config.min_amplitude)))

        set_fps = getattr(sdk, "setMaxFps", None)
        if set_fps is not None:
            _try("setMaxFps", lambda: set_fps(int(self.config.max_fps)))

        self.logger.info(
            f"XT-M60 device-config applied: int=({self.config.int_time_gs},"
            f"{self.config.int_time_1},{self.config.int_time_2},{self.config.int_time_3}) "
            f"HDR={self.config.hdr_mode} minAmp={self.config.min_amplitude} "
            f"fps={self.config.max_fps}")

    @staticmethod
    def _sdk_value(obj, name):
        try:
            return getattr(obj, name)
        except Exception:
            return None

    def _verify_device_config(self) -> bool:
        """Read back identity/imaging settings and optionally fail closed."""
        if not self.config.verify_device_config or self._device_config_verified:
            return True
        now = self._now()
        retry_interval = max(0.1, float(self.config.reconnect_interval_sec))
        if now - self._last_device_config_check < retry_interval:
            return False
        self._last_device_config_check = now

        get_info = getattr(self._sdk, "getDevInfo", None)
        get_config = getattr(self._sdk, "getDevConfig", None)
        if get_info is None or get_config is None:
            mismatch = "SDK does not provide getDevInfo/getDevConfig"
        else:
            try:
                info_ok, info = get_info()
                config_ok, actual = get_config()
            except Exception as exc:
                mismatch = f"readback failed: {exc}"
            else:
                if not info_ok or not config_ok:
                    mismatch = (
                        "getDevInfo/getDevConfig returned false "
                        f"({bool(info_ok)}/{bool(config_ok)})"
                    )
                else:
                    differences = []
                    serial = str(self._sdk_value(info, "sn") or "")
                    if (
                        self.config.expected_serial
                        and serial != self.config.expected_serial
                    ):
                        differences.append(
                            f"serial={serial!r} expected="
                            f"{self.config.expected_serial!r}"
                        )
                    expected_fields = {
                        "integrationTimeGs": int(self.config.int_time_gs),
                        "integrationTimes": [
                            int(self.config.int_time_1),
                            int(self.config.int_time_2),
                            int(self.config.int_time_3),
                            int(self.config.int_time_4),
                        ],
                        "hdrMode": int(self.config.hdr_mode),
                        "miniAmp": int(self.config.min_amplitude),
                        "maxfps": int(self.config.max_fps),
                    }
                    for name, expected in expected_fields.items():
                        value = self._sdk_value(actual, name)
                        if name == "integrationTimes" and value is not None:
                            value = [int(item) for item in value]
                        elif value is not None:
                            value = int(value)
                        if value != expected:
                            differences.append(
                                f"{name}={value!r} expected={expected!r}"
                            )
                    mismatch = "; ".join(differences)

        if mismatch:
            self._last_error = f"device config verification mismatch: {mismatch}"
            if self.config.require_device_config_match:
                self.logger.error(
                    f"XT-M60 {self._last_error}; measurement remains stopped"
                )
                return False
            self.logger.warning(f"XT-M60 {self._last_error}; continuing by policy")
        else:
            self.logger.info(
                "XT-M60 device identity/imaging config readback verified"
            )
        self._device_config_verified = True
        return True

    def _start_measurement(self) -> bool:
        image_type = self._xintan_sdk.ImageType(int(self.config.image_type))
        try:
            self._apply_device_config()
            if self.config.apply_device_config:
                self._device_config_verified = not self.config.verify_device_config
            if not self._verify_device_config():
                self._measurement_started = False
                return False
            if self.config.wait_for_phase_reference:
                if self._scheduled_measurement_start_time is None:
                    return False
                delay = self._scheduled_measurement_start_time - self._now()
                if delay > 0.0:
                    time.sleep(delay)
            elif self.config.start_on_phase_grid:
                now = self._now()
                target = self._phase_target_on_host_grid(
                    now,
                    float(self.config.phase_period_sec),
                    float(self.config.phase_offset_sec),
                )
                time.sleep(max(0.0, target - now))
            try:
                ok = self._sdk.start(image_type, False)
            except TypeError:
                ok = self._sdk.start(image_type)
            if ok is False:
                self._last_error = "XT-M60 start() returned false"
                self._measurement_started = False
                self.logger.error(self._last_error)
                return False
            self._measurement_started = True
            self._last_measurement_start_time = self._now()
            self._scheduled_measurement_start_time = None
            self.logger.info(f"XT-M60 measurement started with ImageType({self.config.image_type})")
            return True
        except Exception as exc:
            self._last_error = str(exc)
            self._measurement_started = False
            self.logger.error(f"XT-M60 measurement start failed: {exc}")
            return False

    def _on_event(self, event) -> None:
        event_name = str(getattr(event, "eventstr", ""))
        cmd_id = getattr(event, "cmdid", None)
        if event_name == "sdkState" and cmd_id == 0xFF:
            # TCP disconnected; SDK will self-reconnect. Other event classes
            # also use 0xFF for normal device-state notifications.
            self._connected = False
            self._measurement_started = False
            self._scheduled_measurement_start_time = None
            self._device_config_verified = not self.config.verify_device_config
            if self.config.udp_dest_port > 0:
                self._udp_dest_applied = False
            self.logger.warning("XT-M60 SDK disconnected (cmdid=0xFF); waiting for self-reconnect")
        elif event_name == "sdkState" and cmd_id == 0xFE:
            # (Re)connected / handshake done -> restart measurement.
            self._connected = True
            if self._measurement_started:
                self.logger.info(
                    "XT-M60 SDK connected event arrived after measurement start; "
                    "keeping the active stream"
                )
                return
            self._measurement_started = False
            self._scheduled_measurement_start_time = None
            self._device_config_verified = not self.config.verify_device_config
            if self.config.udp_dest_port > 0:
                self._udp_dest_applied = False
            self.logger.info("XT-M60 SDK connected (cmdid=0xFE); will (re)start measurement")
        elif event_name == "sdkState":
            try:
                self._connected = bool(self._sdk is not None and self._sdk.isconnect())
            except Exception:
                self._connected = False

    def _on_frame(self, frame) -> None:
        receive_monotonic = self._now()
        receive_wall_time_ns = time.time_ns()
        self._last_frame_time = receive_monotonic
        width = 0
        height = 0
        quality_ranges = None
        if self.config.organized_cloud:
            points, width, height, quality_ranges = extract_xyzi_grid(
                frame,
                unit_scale=self.config.point_unit_scale,
                range_min=self.config.range_min,
                range_max=self.config.range_max,
                return_ranges=True,
            )
            if points is None:
                # Grid unavailable this frame: fall back to the unordered cloud.
                points = extract_xyzi_points(
                    frame,
                    unit_scale=self.config.point_unit_scale,
                    range_min=self.config.range_min,
                    range_max=self.config.range_max,
                )
                width = 0
                height = 0
        else:
            points = extract_xyzi_points(
                frame,
                unit_scale=self.config.point_unit_scale,
                range_min=self.config.range_min,
                range_max=self.config.range_max,
            )
        if not points:
            return
        sdk_stamp = None
        sec = getattr(frame, "timeStampS", None)
        nsec = getattr(frame, "timeStampNS", None)
        if isinstance(sec, int) and isinstance(nsec, int):
            sdk_stamp = (sec, nsec)
        frame_id = getattr(frame, "frame_id", None)
        temperature_raw = getattr(frame, "temperature", None)
        vcsel_temperature_raw = getattr(frame, "vcseltemperature", None)
        temperature_c = (
            float(temperature_raw) / 100.0
            if isinstance(temperature_raw, (int, float))
            and float(temperature_raw) > 0.0
            else None
        )
        vcsel_temperature_c = (
            float(vcsel_temperature_raw) / 100.0
            if isinstance(vcsel_temperature_raw, (int, float))
            and float(vcsel_temperature_raw) > 0.0
            else None
        )
        with self._lock:
            self._latest_points = points
            self._latest_sdk_stamp = sdk_stamp
            self._latest_receive_monotonic = receive_monotonic
            self._latest_receive_wall_time_ns = receive_wall_time_ns
            self._latest_quality_ranges = quality_ranges
            self._latest_temperature_c = temperature_c
            self._latest_vcsel_temperature_c = vcsel_temperature_c
            self._latest_frame_id = frame_id
            self._latest_grid = (int(width), int(height))

    @staticmethod
    def _now() -> float:
        try:
            import time

            return time.monotonic()
        except Exception:
            return 0.0


class XTM60AdapterNode(Node):
    def __init__(self):
        super().__init__("xtm60_adapter_node")
        self.declare_parameter("mode", "real")
        self.declare_parameter("frame_id", "laser_link")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("use_sdk_timestamps", False)
        self.declare_parameter("timestamp_source", "host_receive")
        self.declare_parameter("sdk_root", "")
        self.declare_parameter("connection_mode", "ethernet")
        self.declare_parameter("ip_address", "192.168.0.101")
        self.declare_parameter("serial_port", "")
        self.declare_parameter("image_type", 4)
        self.declare_parameter("point_unit_scale", 1.0)
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 20.0)
        self.declare_parameter("publish_intensity", True)
        self.declare_parameter("organized_cloud", True)
        self.declare_parameter("enable_sdk_filters", True)
        self.declare_parameter("kalman_factor", 300)
        self.declare_parameter("kalman_threshold", 200)
        self.declare_parameter("kalman_range", 2000)
        self.declare_parameter("median_size", 3)
        self.declare_parameter("edge_threshold", 150)
        self.declare_parameter("dust_enable", True)
        self.declare_parameter("dust_threshold", 9000)
        self.declare_parameter("dust_frames", 2)
        self.declare_parameter("postprocess_enable", True)
        self.declare_parameter("postprocess_threshold", 5.0)
        self.declare_parameter("postprocess_dynamic_enable", 1)
        self.declare_parameter("postprocess_dynamic_winsize", 9)
        self.declare_parameter("reflective_enable", True)
        self.declare_parameter("reflective_th_min", 0.5)
        self.declare_parameter("reflective_th_max", 2.0)
        # Device-side imaging params (xintan.xtcfg / upper-computer "Config Device").
        self.declare_parameter("apply_device_config", False)
        self.declare_parameter("verify_device_config", False)
        self.declare_parameter("require_device_config_match", False)
        self.declare_parameter("expected_serial", "")
        self.declare_parameter("int_time_gs", 2000)
        self.declare_parameter("int_time_1", 1600)
        self.declare_parameter("int_time_2", 200)
        self.declare_parameter("int_time_3", 30)
        self.declare_parameter("int_time_4", 1600)
        self.declare_parameter("hdr_mode", 1)
        self.declare_parameter("min_amplitude", 70)
        self.declare_parameter("max_fps", 10)
        self.declare_parameter("mod_freq1", 0)
        self.declare_parameter("mod_freq2", 0)
        self.declare_parameter("mod_freq3", 0)
        self.declare_parameter("mod_freq4", 3)
        self.declare_parameter("mod_freq5", 2)
        self.declare_parameter("reconnect_interval_sec", 3.0)
        self.declare_parameter("require_ping_before_start", True)
        self.declare_parameter("frame_timeout_sec", 8.0)
        self.declare_parameter("phase_mode", "independent")
        self.declare_parameter("phase_reference_topic", "/xtm60/right/phase")
        self.declare_parameter("phase_period_sec", 0.1)
        self.declare_parameter("phase_offset_sec", 0.05)
        self.declare_parameter("phase_realign_interval_sec", 0.0)
        self.declare_parameter("quality_gate_enabled", False)
        self.declare_parameter("quality_min_valid_fraction", 0.50)
        self.declare_parameter("quality_min_relative_valid_fraction", 0.75)
        self.declare_parameter("quality_baseline_window_frames", 30)
        self.declare_parameter("quality_max_median_range_delta", 0.10)
        self.declare_parameter("quality_max_p95_range_delta", 0.75)
        self.declare_parameter("quality_min_overlap_fraction", 0.20)
        self.declare_parameter("quality_reset_after_gap_sec", 0.50)
        self.declare_parameter("quality_temporal_hard_reject", True)
        self.declare_parameter("udp_dest_ip", "")
        self.declare_parameter("udp_dest_port", 0)

        self.mode = self.get_parameter("mode").value
        self.frame_id = self.get_parameter("frame_id").value
        self.phase_mode = str(self.get_parameter("phase_mode").value).strip().lower()
        if self.phase_mode not in ("independent", "grid", "leader", "follower"):
            raise ValueError(
                "phase_mode must be independent, grid, leader, or follower"
            )
        self.config = self._read_config()
        self.pub = self.create_publisher(PointCloud2, "/xtm60/points", 10)
        self.rejected_pub = self.create_publisher(
            PointCloud2, "/xtm60/points_rejected", 10
        )
        self.quality_pub = self.create_publisher(String, "/xtm60/quality", 10)
        self.status_pub = self.create_publisher(String, "/xtm60/status", 10)
        self.timing_pub = self.create_publisher(String, "/xtm60/timing", 10)
        self.phase_pub = (
            self.create_publisher(Float64, "/xtm60/phase", 10)
            if self.phase_mode == "leader"
            else None
        )
        self.phase_sub = (
            self.create_subscription(
                Float64,
                str(self.get_parameter("phase_reference_topic").value),
                self._on_phase_reference,
                10,
            )
            if self.phase_mode == "follower"
            else None
        )
        self.adapter: Optional[XTM60SdkAdapter] = None
        self.sdk_start_failed = False
        self.last_start_attempt = 0.0
        self.quality_gate = CloudFrameQualityGate(
            enabled=bool(self.get_parameter("quality_gate_enabled").value),
            min_valid_fraction=float(
                self.get_parameter("quality_min_valid_fraction").value
            ),
            min_relative_valid_fraction=float(
                self.get_parameter(
                    "quality_min_relative_valid_fraction"
                ).value
            ),
            baseline_window_frames=int(
                self.get_parameter("quality_baseline_window_frames").value
            ),
            max_median_range_delta=float(
                self.get_parameter("quality_max_median_range_delta").value
            ),
            max_p95_range_delta=float(
                self.get_parameter("quality_max_p95_range_delta").value
            ),
            min_overlap_fraction=float(
                self.get_parameter("quality_min_overlap_fraction").value
            ),
            reset_after_gap_sec=float(
                self.get_parameter("quality_reset_after_gap_sec").value
            ),
            temporal_hard_reject=bool(
                self.get_parameter("quality_temporal_hard_reject").value
            ),
        )
        self.last_quality_result: Optional[CloudQualityResult] = None

        if self.mode == "real":
            self._maybe_start_adapter(force=True)

        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def destroy_node(self):
        if self.adapter is not None:
            try:
                self.adapter.stop()
            except BaseException as exc:
                self.get_logger().warning(f"XT-M60 adapter cleanup interrupted: {exc}")
            finally:
                self.adapter = None
        try:
            super().destroy_node()
        except KeyboardInterrupt:
            pass

    def exit_process_cleanly(self, code: int = 0) -> None:
        if self.adapter is not None:
            self.adapter.stop_and_exit_process(code)
        os._exit(code)

    def tick(self):
        if self.mode == "mock":
            receive_wall_time_ns = time.time_ns()
            cloud = self.make_mock_cloud()
            self.pub.publish(cloud)
            publish_wall_time_ns = time.time_ns()
            self._publish_timing(
                sdk_stamp=None,
                cloud=cloud,
                receive_wall_time_ns=receive_wall_time_ns,
                publish_wall_time_ns=publish_wall_time_ns,
            )
            self._publish_status("mock publishing /xtm60/points")
            return

        if self.adapter is None or self.sdk_start_failed:
            self._maybe_start_adapter()
        if self.adapter is None or self.sdk_start_failed:
            self._publish_status(f"waiting: XT-M60 SDK not running; {self._startup_block_reason()}")
            return

        self.adapter.poll()
        (
            points,
            sdk_stamp,
            grid,
            receive_monotonic,
            receive_wall_time_ns,
            quality_ranges,
            temperature_c,
            vcsel_temperature_c,
        ) = self.adapter.take_latest_points()
        if points:
            width, height = int(grid[0]), int(grid[1])
            quality = self.quality_gate.evaluate(
                points,
                receive_monotonic,
                organized=(
                    width > 0
                    and height > 0
                    and width * height == len(points)
                ),
                ranges_override=quality_ranges,
            )
            self.last_quality_result = quality
            cloud = self._make_cloud(
                points, sdk_stamp, grid, receive_wall_time_ns=receive_wall_time_ns
            )
            if cloud is None:
                # A formal device-clock route must not silently degrade to
                # callback, publish, or ROS-now time when the selected source
                # cannot produce a valid epoch stamp. Keep raw provenance on
                # /xtm60/timing, but publish no geometrically plausible-looking
                # cloud that could be mistaken for a synchronized acquisition.
                self._publish_timing(
                    sdk_stamp=sdk_stamp,
                    cloud=None,
                    receive_wall_time_ns=receive_wall_time_ns,
                    publish_wall_time_ns=time.time_ns(),
                )
                self._publish_status(
                    "blocked: selected XT-M60 timestamp source has no valid epoch stamp"
                )
                return
            if quality.accepted:
                self.pub.publish(cloud)
            else:
                # Rejected frames remain available for bounded diagnosis and
                # rosbag evidence without duplicating every healthy cloud.
                self.rejected_pub.publish(cloud)
            self._publish_timing(
                sdk_stamp=sdk_stamp,
                cloud=cloud,
                receive_wall_time_ns=receive_wall_time_ns,
                publish_wall_time_ns=time.time_ns(),
            )
            self._publish_quality(
                quality,
                temperature_c=temperature_c,
                vcsel_temperature_c=vcsel_temperature_c,
            )
            if self.phase_pub is not None and receive_monotonic is not None:
                phase = Float64()
                phase.data = float(receive_monotonic)
                self.phase_pub.publish(phase)
        state = "connected" if self.adapter.connected else "disconnected"
        started = "measuring" if self.adapter.measurement_started else "waiting_measurement"
        quality_text = "quality=waiting"
        if self.last_quality_result is not None:
            quality = self.last_quality_result
            median_mm = (
                "na"
                if quality.median_range_delta is None
                else f"{quality.median_range_delta * 1000.0:.1f}"
            )
            p95_mm = (
                "na"
                if quality.p95_range_delta is None
                else f"{quality.p95_range_delta * 1000.0:.1f}"
            )
            quality_text = (
                f"quality={quality.reason}; valid={quality.valid_fraction:.3f}; "
                f"delta_mm={median_mm}/{p95_mm}; "
                f"accepted={quality.accepted_frames}; "
                f"dropped={quality.dropped_frames}"
            )
        self._publish_status(
            f"{state}; {started}; phase={self.phase_mode}; {quality_text}; "
            f"{self.adapter.last_error}"
        )

    def _on_phase_reference(self, msg) -> None:
        if self.adapter is None:
            return
        try:
            self.adapter.schedule_measurement_start_from_reference(float(msg.data))
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"XT-M60 invalid phase reference: {exc}")

    def _read_config(self) -> XTM60SdkConfig:
        return XTM60SdkConfig(
            sdk_root=str(self.get_parameter("sdk_root").value),
            connection_mode=str(self.get_parameter("connection_mode").value),
            ip_address=str(self.get_parameter("ip_address").value),
            serial_port=str(self.get_parameter("serial_port").value),
            image_type=int(self.get_parameter("image_type").value),
            frame_id=str(self.get_parameter("frame_id").value),
            point_unit_scale=float(self.get_parameter("point_unit_scale").value),
            range_min=float(self.get_parameter("range_min").value),
            range_max=float(self.get_parameter("range_max").value),
            publish_intensity=bool(self.get_parameter("publish_intensity").value),
            organized_cloud=bool(self.get_parameter("organized_cloud").value),
            enable_sdk_filters=bool(self.get_parameter("enable_sdk_filters").value),
            kalman_factor=int(self.get_parameter("kalman_factor").value),
            kalman_threshold=int(self.get_parameter("kalman_threshold").value),
            kalman_range=int(self.get_parameter("kalman_range").value),
            median_size=int(self.get_parameter("median_size").value),
            edge_threshold=int(self.get_parameter("edge_threshold").value),
            dust_enable=bool(self.get_parameter("dust_enable").value),
            dust_threshold=int(self.get_parameter("dust_threshold").value),
            dust_frames=int(self.get_parameter("dust_frames").value),
            postprocess_enable=bool(self.get_parameter("postprocess_enable").value),
            postprocess_threshold=float(self.get_parameter("postprocess_threshold").value),
            postprocess_dynamic_enable=int(self.get_parameter("postprocess_dynamic_enable").value),
            postprocess_dynamic_winsize=int(self.get_parameter("postprocess_dynamic_winsize").value),
            reflective_enable=bool(self.get_parameter("reflective_enable").value),
            reflective_th_min=float(self.get_parameter("reflective_th_min").value),
            reflective_th_max=float(self.get_parameter("reflective_th_max").value),
            apply_device_config=bool(self.get_parameter("apply_device_config").value),
            verify_device_config=bool(
                self.get_parameter("verify_device_config").value
            ),
            require_device_config_match=bool(
                self.get_parameter("require_device_config_match").value
            ),
            expected_serial=str(self.get_parameter("expected_serial").value),
            int_time_gs=int(self.get_parameter("int_time_gs").value),
            int_time_1=int(self.get_parameter("int_time_1").value),
            int_time_2=int(self.get_parameter("int_time_2").value),
            int_time_3=int(self.get_parameter("int_time_3").value),
            int_time_4=int(self.get_parameter("int_time_4").value),
            hdr_mode=int(self.get_parameter("hdr_mode").value),
            min_amplitude=int(self.get_parameter("min_amplitude").value),
            max_fps=int(self.get_parameter("max_fps").value),
            mod_freq1=int(self.get_parameter("mod_freq1").value),
            mod_freq2=int(self.get_parameter("mod_freq2").value),
            mod_freq3=int(self.get_parameter("mod_freq3").value),
            mod_freq4=int(self.get_parameter("mod_freq4").value),
            mod_freq5=int(self.get_parameter("mod_freq5").value),
            reconnect_interval_sec=float(self.get_parameter("reconnect_interval_sec").value),
            require_ping_before_start=bool(self.get_parameter("require_ping_before_start").value),
            frame_timeout_sec=float(self.get_parameter("frame_timeout_sec").value),
            wait_for_phase_reference=(
                str(self.get_parameter("phase_mode").value).strip().lower()
                == "follower"
            ),
            start_on_phase_grid=(
                str(self.get_parameter("phase_mode").value).strip().lower()
                == "grid"
            ),
            phase_period_sec=float(self.get_parameter("phase_period_sec").value),
            phase_offset_sec=float(self.get_parameter("phase_offset_sec").value),
            phase_realign_interval_sec=float(
                self.get_parameter("phase_realign_interval_sec").value
            ),
            udp_dest_ip=str(self.get_parameter("udp_dest_ip").value),
            udp_dest_port=int(self.get_parameter("udp_dest_port").value),
        )

    def _maybe_start_adapter(self, force: bool = False):
        if self.adapter is not None:
            return
        now = time.monotonic()
        if not force and now - self.last_start_attempt < self.config.reconnect_interval_sec:
            return
        self.last_start_attempt = now
        if self.config.require_ping_before_start and not self._configured_ip_reachable():
            self.sdk_start_failed = True
            return
        adapter = XTM60SdkAdapter(self.config, self.get_logger())
        try:
            adapter.start()
        except Exception as exc:
            self.sdk_start_failed = True
            self.get_logger().error(f"XT-M60 SDK startup failed: {exc}")
            try:
                adapter.stop()
            except BaseException:
                pass
            return
        self.adapter = adapter
        self.sdk_start_failed = False

    def _configured_ip_reachable(self) -> bool:
        if self.config.connection_mode.lower().strip() != "ethernet":
            return True
        try:
            result = subprocess.run(
                ["ping", "-c", "1", "-W", "1", self.config.ip_address],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=2.0,
                check=False,
            )
            return result.returncode == 0
        except Exception:
            return False

    def _startup_block_reason(self) -> str:
        if (
            self.config.connection_mode.lower().strip() == "ethernet"
            and self.config.require_ping_before_start
            and self.sdk_start_failed
        ):
            return f"waiting for ping {self.config.ip_address}"
        return "startup pending"

    def _make_cloud(
        self,
        points: Sequence[XYZI],
        sdk_stamp: Optional[Tuple[int, int]],
        grid: Tuple[int, int] = (0, 0),
        *,
        receive_wall_time_ns: Optional[int] = None,
    ):
        header = Header()
        header.frame_id = self.frame_id
        use_sdk_timestamps = bool(self.get_parameter("use_sdk_timestamps").value)
        timestamp_source = str(self.get_parameter("timestamp_source").value).strip().lower()
        if use_sdk_timestamps:
            # Backward compatibility for older profiles. Only epoch-shaped SDK
            # timestamps are accepted; XT-M60 runtime/uptime stamps must not be
            # guessed into the ROS time domain.
            timestamp_source = "sdk_epoch"
        stamp = self._select_cloud_stamp(
            timestamp_source, sdk_stamp=sdk_stamp, receive_wall_time_ns=receive_wall_time_ns
        )
        if stamp is None:
            return None
        header.stamp.sec, header.stamp.nanosec = stamp

        width, height = int(grid[0]), int(grid[1])
        organized = width > 0 and height > 0 and width * height == len(points)

        if bool(self.get_parameter("publish_intensity").value):
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            xyzi = list(points)
        else:
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            xyzi = [(x, y, z) for x, y, z, _i in points]

        cloud = point_cloud2.create_cloud(header, fields, xyzi)
        if organized:
            # Mark the cloud as organized (grid). create_cloud() defaults to
            # height=1; set the real sensor grid so RViz/consumers treat it as a
            # depth image (neighbours adjacent -> denser, surface-like view).
            cloud.height = height
            cloud.width = width
            cloud.row_step = cloud.point_step * width
            cloud.is_dense = False
        return cloud

    @staticmethod
    def _is_plausible_epoch_stamp(stamp: Optional[Tuple[int, int]]) -> bool:
        if stamp is None:
            return False
        sec, nsec = stamp
        return int(sec) >= 1_000_000_000 and 0 <= int(nsec) < 1_000_000_000

    @classmethod
    def _select_cloud_stamp(
        cls,
        timestamp_source: str,
        *,
        sdk_stamp: Optional[Tuple[int, int]],
        receive_wall_time_ns: Optional[int],
    ) -> Optional[Tuple[int, int]]:
        """Choose one declared timestamp domain without inventing a fallback.

        ``timeStampS/timeStampNS`` is copied only when it is already an epoch
        timestamp. The observed XT-M60 uptime-shaped values are deliberately
        rejected: there is no verified reset/epoch conversion, so mapping them
        using host arrival time would be a fabricated device timestamp.
        """

        source = str(timestamp_source).strip().lower()
        if source == "sdk_epoch":
            if cls._is_plausible_epoch_stamp(sdk_stamp):
                return int(sdk_stamp[0]), int(sdk_stamp[1])
            return None
        if source == "host_receive":
            if isinstance(receive_wall_time_ns, int) and receive_wall_time_ns > 0:
                return (
                    receive_wall_time_ns // 1_000_000_000,
                    receive_wall_time_ns % 1_000_000_000,
                )
            return None
        return None

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _publish_quality(
        self,
        quality: CloudQualityResult,
        *,
        temperature_c: Optional[float],
        vcsel_temperature_c: Optional[float],
    ):
        msg = String()
        msg.data = json.dumps(
            {
                "accepted": quality.accepted,
                "reason": quality.reason,
                "valid_fraction": quality.valid_fraction,
                "baseline_valid_fraction": quality.baseline_valid_fraction,
                "overlap_fraction": quality.overlap_fraction,
                "median_range_delta_mm": (
                    None
                    if quality.median_range_delta is None
                    else quality.median_range_delta * 1000.0
                ),
                "p95_range_delta_mm": (
                    None
                    if quality.p95_range_delta is None
                    else quality.p95_range_delta * 1000.0
                ),
                "temporal_jump_detected": quality.temporal_jump_detected,
                "relative_validity_drop_detected": (
                    quality.relative_validity_drop_detected
                ),
                "temporal_hard_reject": self.quality_gate.temporal_hard_reject,
                "temperature_c": temperature_c,
                "vcsel_temperature_c": vcsel_temperature_c,
                "accepted_frames": quality.accepted_frames,
                "dropped_frames": quality.dropped_frames,
            },
            separators=(",", ":"),
            sort_keys=True,
        )
        self.quality_pub.publish(msg)

    def _publish_timing(
        self,
        *,
        sdk_stamp: Optional[Tuple[int, int]],
        cloud,
        receive_wall_time_ns: Optional[int],
        publish_wall_time_ns: Optional[int],
    ) -> None:
        header_stamp = getattr(getattr(cloud, "header", None), "stamp", None)
        cloud_stamp = None
        if header_stamp is not None:
            cloud_stamp = (
                int(getattr(header_stamp, "sec", 0)),
                int(getattr(header_stamp, "nanosec", 0)),
            )
        timestamp_source = str(self.get_parameter("timestamp_source").value)
        if bool(self.get_parameter("use_sdk_timestamps").value):
            timestamp_source = "sdk_epoch"
        msg = String()
        msg.data = build_timing_diagnostic(
            sdk_stamp=sdk_stamp,
            host_receive_wall_time_ns=receive_wall_time_ns,
            cloud_header_stamp=cloud_stamp,
            publish_wall_time_ns=publish_wall_time_ns,
            timestamp_source=timestamp_source,
        )
        self.timing_pub.publish(msg)

    def make_mock_cloud(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        points = []
        for deg in range(-60, 61, 3):
            angle = math.radians(deg)
            points.append([2.0 * math.cos(angle), 2.0 * math.sin(angle), 0.0])
        return point_cloud2.create_cloud_xyz32(header, points)


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS2 Python packages are required to run this node")
    rclpy.init(args=args)
    node = XTM60AdapterNode()
    exit_code = 0
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except BaseException:
        exit_code = 1
        raise
    finally:
        # Humble may consume SIGINT inside rclpy and return from spin() without
        # raising KeyboardInterrupt. For a real XT-M60, always use the guarded
        # SDK stop/shutdown path followed by os._exit so the vendor pybind
        # destructor cannot run a second time and segfault during interpreter
        # teardown. Mock mode has no vendor object and keeps normal ROS cleanup.
        if node.mode == "real":
            node.exit_process_cleanly(exit_code)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
