import json
import os
import re
import signal
import shutil
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

import yaml


CONFLICTING_NAV_NODES = {
    "/amcl",
    "/map_server",
    "/planner_server",
    "/controller_server",
    "/bt_navigator",
    "/waypoint_follower",
    "/behavior_server",
}


@dataclass(frozen=True)
class MapSaveResult:
    map_name: str
    version_id: str
    version_yaml: Path
    current_yaml: Path


def find_workspace_root() -> Path:
    configured = os.environ.get("SMARTWHEEL_WORKSPACE")
    if configured:
        return Path(configured).expanduser().resolve()
    candidates = [Path.cwd(), *Path(__file__).resolve().parents]
    for candidate in candidates:
        if (candidate / "install" / "setup.bash").exists() and (
            candidate / "src" / "wheelchair_bringup"
        ).exists():
            return candidate
    return Path("/home/nvidia/smartwheel")


def safe_map_name(name: Optional[str]) -> str:
    if not name:
        return datetime.now().strftime("map_%Y%m%d_%H%M%S")
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", name.strip())
    cleaned = cleaned.strip("._-")
    return cleaned or datetime.now().strftime("map_%Y%m%d_%H%M%S")


class MappingManager:
    def __init__(self, workspace_root: Optional[Path] = None):
        self.workspace_root = workspace_root or find_workspace_root()
        self.map_dir = self.workspace_root / "maps"
        self.version_dir = self.map_dir / "versions"
        self.version_manifest_path = self.map_dir / "map_versions.json"
        self.log_dir = self.workspace_root / ".ros" / "log"
        self.lock = threading.RLock()
        self.process: Optional[subprocess.Popen] = None
        self.log_handle = None
        self.state = "IDLE"
        self.reason = "等待建图"
        self.started_at: Optional[float] = None
        self.map_name: Optional[str] = None
        self.last_map_yaml: Optional[Path] = None
        self.log_path: Optional[Path] = None
        self.quality_report_path: Optional[Path] = None
        self.quality_status: Optional[Dict[str, Any]] = None
        self.last_map_version: Optional[Dict[str, Any]] = None
        self.external_slam = False
        self.smartwheel_service_was_running = False
        # Mapping backend: rtabmap (3D, default main line) | slam_toolbox (2D fallback).
        self.backend = (os.environ.get("SMARTWHEEL_MAP_BACKEND", "rtabmap").strip().lower() or "rtabmap")
        self.grid_topic = "/rtabmap/grid_map" if self.backend == "rtabmap" else "/map"
        self.last_map_3d: Optional[Path] = None
        self._load_last_saved_map()

    def status(self, ros_status: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        with self.lock:
            self._refresh_process_locked()
            preflight = self.preflight(ros_status or {})
            elapsed = None if self.started_at is None else max(0.0, time.monotonic() - self.started_at)
            progress = self._progress_locked(elapsed)
            return {
                "state": self.state,
                "reason": self.reason,
                "map_name": self.map_name,
                "elapsed_sec": elapsed,
                "progress": progress,
                "running": self.state in ("MAPPING", "SAVING"),
                "external_slam": self.external_slam,
                "backend": self.backend,
                "saved_map": str(self.last_map_yaml) if self.last_map_yaml else None,
                "saved_map_3d": str(self.last_map_3d) if self.last_map_3d else None,
                "log_path": str(self.log_path) if self.log_path else None,
                "quality_report": str(self.quality_report_path) if self.quality_report_path else None,
                "quality_status": self.quality_status,
                "version_manifest": str(self.version_manifest_path),
                "map_version": self.last_map_version,
                "recent_map_versions": self._recent_map_versions(5),
                "preflight": preflight,
            }

    def preflight(self, ros_status: Dict[str, Any]) -> Dict[str, Any]:
        sensors = ros_status.get("sensors") or {}
        hardware = ros_status.get("hardware_status") or {}
        localization = ros_status.get("localization_health") or {}
        ultrasonic = sensors.get("ultrasonic") or []
        scan_online = bool((sensors.get("scan") or {}).get("online"))
        laser_online = bool((sensors.get("laser") or {}).get("online"))
        checks = [
            self._check("scan", "雷达/scan", scan_online or laser_online, True, "建图需要 XT-M60 点云或 /scan"),
            self._check("odom", "轮速里程计", bool((sensors.get("odom") or {}).get("online")), True, "建图需要连续 /wheel/odom"),
            self._check("imu", "H30 IMU", bool((sensors.get("imu") or {}).get("online")), False, "建议开启 IMU 记录姿态"),
            self._check(
                "ultrasonic",
                "超声波",
                any(bool(item.get("online")) for item in ultrasonic),
                False,
                "用于近距离安全冗余",
            ),
            self._check(
                "camera_front",
                "前摄像头",
                bool((sensors.get("camera_front") or {}).get("online")),
                False,
                "用于远程观察现场",
            ),
            self._check(
                "hardware",
                "硬件看门狗",
                (hardware.get("state") in (None, "UNKNOWN", "OK", "DEGRADED")),
                False,
                hardware.get("reason", "等待硬件诊断"),
            ),
            self._check(
                "localization",
                "定位状态",
                localization.get("state") not in ("LOST", "ERROR"),
                False,
                localization.get("reason", "建图阶段定位可为空"),
            ),
        ]
        required_failed = [item for item in checks if item["required"] and not item["ok"]]
        return {
            "can_start": not required_failed,
            "checks": checks,
            "required_failed": [item["key"] for item in required_failed],
        }

    @staticmethod
    def _check(key: str, label: str, ok: bool, required: bool, detail: str) -> Dict[str, Any]:
        return {"key": key, "label": label, "ok": bool(ok), "required": required, "detail": detail}

    def start(self, ros_status: Dict[str, Any], map_name: Optional[str], force: bool = False,
              backend: Optional[str] = None) -> Dict[str, Any]:
        with self.lock:
            self._refresh_process_locked()
            if self.state in ("MAPPING", "SAVING"):
                return self.status(ros_status)

            be = (backend or self.backend or "rtabmap").strip().lower()
            self.backend = be
            self.grid_topic = "/rtabmap/grid_map" if be == "rtabmap" else "/map"
            engine = "RTAB-Map 3D" if be == "rtabmap" else "slam_toolbox 2D"
            ext_node = "/rtabmap" if be == "rtabmap" else "/slam_toolbox"

            node_names = self._ros_node_names()
            if ext_node in node_names:
                self.state = "MAPPING"
                preflight = self.preflight(ros_status)
                suffix = "" if preflight["can_start"] else "，等待必要话题上线：" + ", ".join(preflight["required_failed"])
                self.reason = f"已接入正在运行的 {engine} 建图，请推动轮椅完成闭环" + suffix
                self.started_at = time.monotonic()
                self.map_name = safe_map_name(map_name)
                self.quality_report_path = None
                self.quality_status = None
                self.external_slam = True
                return self.status(ros_status)

            conflicts = sorted(CONFLICTING_NAV_NODES.intersection(node_names))
            if conflicts:
                self.state = "ERROR"
                self.reason = "当前处于定位/导航模式，不能直接建图。冲突节点：" + ", ".join(conflicts)
                return self.status(ros_status)

            # smartwheel.service runs the same hardware nodes (serial ports, DDS
            # topics) as the mapping launch; stop it first to avoid Modbus/TF
            # collisions and unreliable maps, restart on cancel/finish.
            self.smartwheel_service_was_running = self._smartwheel_service_active()
            if self.smartwheel_service_was_running:
                self._stop_smartwheel_service()

            self.map_dir.mkdir(parents=True, exist_ok=True)
            self.log_dir.mkdir(parents=True, exist_ok=True)
            log_path = self.log_dir / f"mapping_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
            self.log_path = log_path
            self.log_handle = log_path.open("a", encoding="utf-8")
            if be == "rtabmap":
                # RTAB-Map 3D main line: 3D cloud_map is the primary product;
                # /rtabmap/grid_map is the projected 2D grid saved for Nav2.
                command = [
                    "ros2", "launch", "wheelchair_3d_mapping",
                    "rtabmap_3d_mapping.launch.py", "bringup_sensors:=true",
                ]
                self.reason = "RTAB-Map 3D 建图已启动（3D 点云为主成果，同时投影 2D 导航底图），等待传感器上线"
            else:
                command = [
                    "ros2", "launch", "wheelchair_bringup", "mapping.launch.py",
                    "use_mock:=false", "use_rviz:=false", "enable_dual_xtm60:=false",
                    # EKF fuses /wheel/odom + /imu so slam_toolbox gets a good
                    # odom->base_link yaw prior for the 120-deg-FOV single radar.
                    "use_ekf:=true",
                ]
                self.reason = "slam_toolbox 2D 保底建图已启动，等待传感器和里程计上线"
            self.process = subprocess.Popen(
                command,
                cwd=str(self.workspace_root),
                env=self._env(),
                stdout=self.log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                text=True,
            )
            self.state = "MAPPING"
            self.started_at = time.monotonic()
            self.map_name = safe_map_name(map_name)
            self.quality_report_path = None
            self.quality_status = None
            self.external_slam = False
            return self.status(ros_status)

    def finish(self, map_name: Optional[str] = None) -> Dict[str, Any]:
        with self.lock:
            self._refresh_process_locked()
            if self.state != "MAPPING":
                self.reason = "当前不在建图状态，不能保存地图"
                return self.status({})
            self.state = "SAVING"
            self.reason = "正在保存地图"
            target_name = safe_map_name(map_name or self.map_name)
            self.map_name = target_name

        try:
            save_result = self._save_map(target_name)
        except Exception as exc:
            with self.lock:
                self.state = "ERROR"
                self.reason = str(exc)
                return self.status({})

        map_3d = self._save_map_3d(save_result.version_id) if self.backend == "rtabmap" else None
        quality_status, quality_report_path = self._run_quality_check(save_result.version_yaml)
        map_version = self._record_map_version(save_result, quality_status, quality_report_path)

        with self.lock:
            self.last_map_yaml = save_result.current_yaml
            self.quality_status = quality_status
            self.quality_report_path = quality_report_path
            self.last_map_version = map_version
            self.last_map_3d = map_3d
            self.state = "MAP_READY"
            quality_text = self._quality_reason_text(quality_status)
            if self.backend == "rtabmap":
                p3d = f"3D 主地图 {map_3d}" if map_3d else "3D 主地图导出跳过（需移动建图≥2关键帧，或用 --live-pcd）"
                self.reason = f"{p3d}；2D 导航投影 {save_result.current_yaml}；版本 {save_result.version_id}{quality_text}"
            else:
                self.reason = f"2D 地图已保存：{save_result.current_yaml}；版本：{save_result.version_id}{quality_text}"
            self.started_at = None
            if not self.external_slam:
                self._stop_process_locked()
                self._restart_smartwheel_if_was_running()
            return self.status({})

    def cancel(self) -> Dict[str, Any]:
        with self.lock:
            self._stop_process_locked()
            self._restart_smartwheel_if_was_running()
            self.state = "IDLE"
            self.reason = "建图已取消"
            self.started_at = None
            self.map_name = None
            self.external_slam = False
            return self.status({})

    def _run_quality_check(self, yaml_path: Path) -> tuple[Optional[Dict[str, Any]], Optional[Path]]:
        script = self._quality_script_path()
        if script is None:
            return self._quality_warning("未找到地图质量检查脚本"), None

        report_path = yaml_path.with_name(f"{yaml_path.stem}_quality.json")
        command = [
            sys.executable,
            str(script),
            str(yaml_path),
            "--json",
            "--report",
            str(report_path),
        ]
        try:
            result = subprocess.run(
                command,
                cwd=str(self.workspace_root),
                env=self._env(),
                capture_output=True,
                text=True,
                timeout=30,
            )
        except Exception as exc:
            return self._quality_warning(f"地图质量检查失败：{exc}"), None

        if result.returncode != 0:
            message = (result.stderr or result.stdout or "地图质量检查命令失败").strip()
            return self._quality_warning(message), None

        try:
            output = result.stdout.strip()
            report = json.loads(output.splitlines()[-1]) if output else json.loads(report_path.read_text(encoding="utf-8"))
        except Exception as exc:
            return self._quality_warning(f"地图质量报告解析失败：{exc}"), None

        return self._compact_quality_report(report), report_path

    def _quality_script_path(self) -> Optional[Path]:
        candidates = [
            self.workspace_root / "src" / "wheelchair_mapping" / "scripts" / "map_quality_check.py",
            self.workspace_root
            / "install"
            / "wheelchair_mapping"
            / "share"
            / "wheelchair_mapping"
            / "scripts"
            / "map_quality_check.py",
        ]
        for candidate in candidates:
            if candidate.exists():
                return candidate
        return None

    @staticmethod
    def _compact_quality_report(report: Dict[str, Any]) -> Dict[str, Any]:
        return {
            "verdict": report.get("verdict", "WARNING"),
            "score": report.get("score", 0),
            "metrics": report.get("metrics", {}),
            "reasons": report.get("reasons", []),
        }

    @staticmethod
    def _quality_warning(message: str) -> Dict[str, Any]:
        return {
            "verdict": "WARNING",
            "score": 0,
            "metrics": {},
            "reasons": [{"severity": "warning", "message": message}],
        }

    @staticmethod
    def _quality_reason_text(quality_status: Optional[Dict[str, Any]]) -> str:
        if not quality_status:
            return ""
        verdict = quality_status.get("verdict", "WARNING")
        score = quality_status.get("score", 0)
        reasons = quality_status.get("reasons") or []
        message = reasons[0].get("message", "") if reasons else ""
        suffix = f"；地图质量：{verdict} ({score})"
        return f"{suffix}，{message}" if message else suffix

    def shutdown(self):
        with self.lock:
            self._stop_process_locked()

    def map_snapshot(self) -> Optional[Dict[str, Any]]:
        if not self.last_map_yaml:
            return None
        try:
            return load_map_yaml(self.last_map_yaml)
        except Exception:
            return None

    def activate_version(self, version_id: str) -> Dict[str, Any]:
        with self.lock:
            entry = self._find_map_version(version_id)
            if not entry:
                self.reason = f"未找到地图版本：{version_id}"
                return self.status({})

        try:
            version_yaml = Path(entry["version_yaml"])
            current_yaml = self._publish_current_map_alias(version_yaml, entry["map_name"])
            manifest = self._load_version_manifest()
            manifest.setdefault("current", {})[entry["map_name"]] = version_id
            manifest["latest"] = version_id
            manifest["updated_at"] = self._now_iso()
            self._write_version_manifest(manifest)
        except Exception as exc:
            with self.lock:
                self.state = "ERROR"
                self.reason = f"地图版本切换失败：{exc}"
                return self.status({})

        quality_report = entry.get("quality_report")
        quality_status = None
        quality_report_path = None
        if quality_report:
            quality_report_path = Path(quality_report)
            quality_status = self._load_quality_status(quality_report_path)

        with self.lock:
            updated = dict(entry)
            updated["current_yaml"] = str(current_yaml)
            self.last_map_yaml = current_yaml
            self.last_map_version = updated
            self.quality_report_path = quality_report_path
            self.quality_status = quality_status or entry.get("quality")
            self.state = "MAP_READY"
            self.reason = f"已切换到地图版本：{version_id}"
            return self.status({})

    def _save_map(self, map_name: str) -> MapSaveResult:
        self.map_dir.mkdir(parents=True, exist_ok=True)
        self.version_dir.mkdir(parents=True, exist_ok=True)
        version_id = self._new_version_id(map_name)
        output_prefix = self.version_dir / version_id
        version_yaml = Path(f"{output_prefix}.yaml")
        attempts = []
        for transient_local in (True, False):
            attempt = self._run_map_saver(output_prefix, transient_local)
            attempts.append(attempt)
            if attempt["returncode"] == 0 and version_yaml.exists():
                current_yaml = self._publish_current_map_alias(version_yaml, map_name)
                return MapSaveResult(
                    map_name=map_name,
                    version_id=version_id,
                    version_yaml=version_yaml,
                    current_yaml=current_yaml,
                )

        message = self._map_saver_failure_message(attempts, version_yaml)
        with self.lock:
            self.state = "ERROR"
            self.reason = message
        raise RuntimeError(message)

    def _run_map_saver(self, output_prefix: Path, transient_local: bool) -> Dict[str, Any]:
        command = [
            "ros2",
            "run",
            "nav2_map_server",
            "map_saver_cli",
            "-t",
            self.grid_topic,
            "-f",
            str(output_prefix),
            "--ros-args",
            "-p",
            f"map_subscribe_transient_local:={str(transient_local).lower()}",
            "-p",
            "save_map_timeout:=10.0",
        ]
        try:
            result = subprocess.run(
                command,
                cwd=str(self.workspace_root),
                env=self._env(),
                capture_output=True,
                text=True,
                timeout=18,
            )
            return {
                "qos": "transient_local" if transient_local else "volatile",
                "returncode": result.returncode,
                "stdout": result.stdout or "",
                "stderr": result.stderr or "",
                "timed_out": False,
            }
        except subprocess.TimeoutExpired as exc:
            return {
                "qos": "transient_local" if transient_local else "volatile",
                "returncode": None,
                "stdout": self._timeout_text(exc.output),
                "stderr": self._timeout_text(exc.stderr),
                "timed_out": True,
            }

    def _map_saver_failure_message(self, attempts: list[Dict[str, Any]], version_yaml: Path) -> str:
        if version_yaml.exists():
            return f"地图保存命令结束异常，但已发现输出文件：{version_yaml}"

        details = []
        for attempt in attempts:
            output = (attempt.get("stderr") or attempt.get("stdout") or "").strip()
            if not output:
                output = "无输出"
            if attempt.get("timed_out"):
                prefix = f"{attempt['qos']} 超时"
            else:
                prefix = f"{attempt['qos']} 返回码 {attempt.get('returncode')}"
            details.append(f"{prefix}: {output[-900:]}")
        return (
            "地图保存失败：map_saver_cli 没有从 /map 收到可保存地图。"
            "请确认建图日志中不再出现 Message Filter dropping message，并让雷达完成一段有效扫描后再保存。\n"
            + "\n".join(details)
        )

    def _save_map_3d(self, version_id: str) -> Optional[Path]:
        """Export the RTAB-Map 3D cloud (PLY) - the primary mapping product.

        Best-effort: a stationary single-keyframe DB has no odometry poses to
        export, so this returns None while the 2D nav projection still succeeds.
        """
        script = self.workspace_root / "scripts" / "save_rtabmap_3d_map.sh"
        if not script.exists():
            return None
        out_dir = self.version_dir / f"{version_id}_3d"
        try:
            subprocess.run(
                ["bash", str(script), "-o", str(out_dir)],
                cwd=str(self.workspace_root), env=self._env(),
                capture_output=True, text=True, timeout=90,
            )
        except Exception:
            return None
        ply = out_dir / "rtabmap_cloud.ply"
        return ply if ply.exists() else None

    @staticmethod
    def _timeout_text(value: Any) -> str:
        if value is None:
            return ""
        if isinstance(value, bytes):
            return value.decode("utf-8", errors="replace")
        return str(value)

    def _new_version_id(self, map_name: str) -> str:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = f"{map_name}_{stamp}"
        candidate = base
        counter = 2
        while Path(f"{self.version_dir / candidate}.yaml").exists():
            candidate = f"{base}_{counter}"
            counter += 1
        return candidate

    def _publish_current_map_alias(self, version_yaml: Path, map_name: str) -> Path:
        meta = yaml.safe_load(version_yaml.read_text(encoding="utf-8")) or {}
        image_path = self._map_image_path(version_yaml, meta)
        image_suffix = image_path.suffix or ".pgm"
        current_yaml = self.map_dir / f"{map_name}.yaml"
        current_image = self.map_dir / f"{map_name}{image_suffix}"

        tmp_image = current_image.with_name(f"{current_image.name}.tmp")
        shutil.copy2(image_path, tmp_image)
        tmp_image.replace(current_image)

        current_meta = dict(meta)
        current_meta["image"] = current_image.name
        tmp_yaml = current_yaml.with_name(f"{current_yaml.name}.tmp")
        tmp_yaml.write_text(
            yaml.safe_dump(current_meta, allow_unicode=True, sort_keys=False),
            encoding="utf-8",
        )
        tmp_yaml.replace(current_yaml)
        return current_yaml

    @staticmethod
    def _map_image_path(map_yaml: Path, meta: Dict[str, Any]) -> Path:
        image = meta.get("image")
        if not image:
            raise RuntimeError(f"地图 YAML 缺少 image 字段：{map_yaml}")
        image_path = Path(str(image))
        if not image_path.is_absolute():
            image_path = map_yaml.parent / image_path
        if not image_path.exists():
            raise RuntimeError(f"地图图像文件不存在：{image_path}")
        return image_path

    def _record_map_version(
        self,
        save_result: MapSaveResult,
        quality_status: Optional[Dict[str, Any]],
        quality_report_path: Optional[Path],
    ) -> Dict[str, Any]:
        manifest = self._load_version_manifest()
        entry = {
            "version_id": save_result.version_id,
            "map_name": save_result.map_name,
            "created_at": self._now_iso(),
            "current_yaml": str(save_result.current_yaml),
            "version_yaml": str(save_result.version_yaml),
            "quality_report": str(quality_report_path) if quality_report_path else None,
            "quality": self._quality_summary(quality_status),
            "log_path": str(self.log_path) if self.log_path else None,
            "source": "gui_online_mapping",
        }
        versions = manifest.setdefault("versions", [])
        versions[:] = [item for item in versions if item.get("version_id") != save_result.version_id]
        versions.append(entry)
        manifest.setdefault("current", {})[save_result.map_name] = save_result.version_id
        manifest["latest"] = save_result.version_id
        manifest["updated_at"] = self._now_iso()
        self._write_version_manifest(manifest)
        return entry

    def _load_last_saved_map(self):
        latest = self._latest_map_version()
        if not latest:
            return
        yaml_value = latest.get("current_yaml") or latest.get("version_yaml")
        if yaml_value:
            yaml_path = Path(yaml_value)
            if yaml_path.exists():
                self.last_map_yaml = yaml_path
        quality_report = latest.get("quality_report")
        if quality_report:
            report_path = Path(quality_report)
            if report_path.exists():
                self.quality_report_path = report_path
                self.quality_status = self._load_quality_status(report_path)
        if self.quality_status is None and latest.get("quality"):
            self.quality_status = latest["quality"]
        self.last_map_version = latest

    def _load_quality_status(self, report_path: Path) -> Optional[Dict[str, Any]]:
        try:
            report = json.loads(report_path.read_text(encoding="utf-8"))
        except Exception:
            return None
        return self._compact_quality_report(report)

    def _load_version_manifest(self) -> Dict[str, Any]:
        default = {"schema_version": 1, "current": {}, "latest": None, "versions": []}
        if not self.version_manifest_path.exists():
            return default
        try:
            manifest = json.loads(self.version_manifest_path.read_text(encoding="utf-8"))
        except Exception:
            return default
        if not isinstance(manifest, dict):
            return default
        manifest.setdefault("schema_version", 1)
        manifest.setdefault("current", {})
        manifest.setdefault("latest", None)
        manifest.setdefault("versions", [])
        return manifest

    def _write_version_manifest(self, manifest: Dict[str, Any]):
        self.map_dir.mkdir(parents=True, exist_ok=True)
        tmp_path = self.version_manifest_path.with_name(f"{self.version_manifest_path.name}.tmp")
        tmp_path.write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        tmp_path.replace(self.version_manifest_path)

    def _latest_map_version(self) -> Optional[Dict[str, Any]]:
        manifest = self._load_version_manifest()
        versions = manifest.get("versions") or []
        latest_id = manifest.get("latest")
        if latest_id:
            for item in reversed(versions):
                if item.get("version_id") == latest_id:
                    return dict(item)
        return dict(versions[-1]) if versions else None

    def _find_map_version(self, version_id: str) -> Optional[Dict[str, Any]]:
        manifest = self._load_version_manifest()
        for item in manifest.get("versions") or []:
            if item.get("version_id") == version_id:
                return dict(item)
        return None

    def _recent_map_versions(self, limit: int) -> list[Dict[str, Any]]:
        manifest = self._load_version_manifest()
        versions = manifest.get("versions") or []
        return [dict(item) for item in reversed(versions[-limit:])]

    @staticmethod
    def _quality_summary(quality_status: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        if not quality_status:
            return None
        return {
            "verdict": quality_status.get("verdict", "WARNING"),
            "score": quality_status.get("score", 0),
            "reasons": quality_status.get("reasons", [])[:3],
        }

    @staticmethod
    def _now_iso() -> str:
        return datetime.now().astimezone().isoformat(timespec="seconds")

    def _env(self) -> Dict[str, str]:
        env = os.environ.copy()
        env.setdefault("ROS_DOMAIN_ID", os.environ.get("SMARTWHEEL_ROS_DOMAIN_ID", "0"))
        env.setdefault("ROS_LOG_DIR", str(self.log_dir))
        return env

    def _ros_node_names(self) -> set[str]:
        try:
            result = subprocess.run(
                ["ros2", "node", "list", "--no-daemon"],
                cwd=str(self.workspace_root),
                env=self._env(),
                capture_output=True,
                text=True,
                timeout=3,
            )
        except Exception:
            return set()
        if result.returncode != 0:
            return set()
        return {line.strip() for line in result.stdout.splitlines() if line.strip()}

    def _smartwheel_service_active(self) -> bool:
        """Return True iff the user smartwheel.service is currently running.

        We check this so we can stop it before a mapping launch and restart
        it afterwards. Failure to talk to systemd is treated as "not running"
        to keep the mapping flow working in environments without the unit
        (developer machines, container builds, etc.).
        """
        try:
            result = subprocess.run(
                ["systemctl", "--user", "is-active", "smartwheel.service"],
                capture_output=True,
                text=True,
                timeout=5,
            )
        except Exception:
            return False
        return result.stdout.strip() == "active"

    def _stop_smartwheel_service(self):
        """Stop smartwheel.service so its ROS nodes do not collide with the
        mapping launch (same node names, same serial ports, same DDS topics).
        Waits for the unit to settle so subsequent mapping nodes start clean.
        """
        try:
            subprocess.run(
                ["systemctl", "--user", "stop", "smartwheel.service"],
                capture_output=True,
                text=True,
                timeout=20,
            )
        except Exception:
            return
        # Give the kernel a beat to release serial port FDs and DDS handles.
        time.sleep(3)

    def _restart_smartwheel_if_was_running(self):
        """If start() previously stopped smartwheel.service, restart it now."""
        if not self.smartwheel_service_was_running:
            return
        self.smartwheel_service_was_running = False
        try:
            subprocess.run(
                ["systemctl", "--user", "start", "smartwheel.service"],
                capture_output=True,
                text=True,
                timeout=20,
            )
        except Exception:
            pass

    def _refresh_process_locked(self):
        if self.process is None:
            return
        code = self.process.poll()
        if code is None:
            return
        self._close_log_locked()
        self.process = None
        if self.state in ("MAPPING", "SAVING"):
            self.state = "ERROR"
            self.reason = f"建图进程已退出，返回码 {code}"
            self.started_at = None

    def _stop_process_locked(self):
        if self.process is not None and self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                self.process.wait(timeout=6)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self.process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    self.process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(self.process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
        self.process = None
        self._close_log_locked()

    def _close_log_locked(self):
        if self.log_handle is not None:
            self.log_handle.close()
            self.log_handle = None

    def _progress_locked(self, elapsed: Optional[float]) -> int:
        if self.state == "IDLE":
            return 0
        if self.state == "PREFLIGHT_FAILED":
            return 8
        if self.state == "MAPPING":
            return 0
        if self.state == "SAVING":
            return 94
        if self.state == "MAP_READY":
            return 100
        if self.state == "ERROR":
            return 100
        return 0


def load_map_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        meta = yaml.safe_load(handle) or {}
    image_path = Path(meta.get("image", ""))
    if not image_path.is_absolute():
        image_path = path.parent / image_path
    width, height, pixels = read_pgm(image_path)
    negate = int(meta.get("negate", 0))
    occupied_thresh = float(meta.get("occupied_thresh", 0.65))
    free_thresh = float(meta.get("free_thresh", 0.196))
    data = []
    for value in pixels:
        normalized = value / 255.0
        occupancy = normalized if negate else 1.0 - normalized
        if occupancy > occupied_thresh:
            data.append(100)
        elif occupancy < free_thresh:
            data.append(0)
        else:
            data.append(-1)
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    return {
        "frame_id": "map",
        "width": width,
        "height": height,
        "resolution": float(meta.get("resolution", 0.05)),
        "origin": {"x": float(origin[0]), "y": float(origin[1])},
        "data": data,
        "source": str(path),
    }


def read_pgm(path: Path):
    raw = path.read_bytes()
    tokens = []
    index = 0
    while len(tokens) < 4:
        while index < len(raw) and raw[index] in b" \t\r\n":
            index += 1
        if index < len(raw) and raw[index] == ord("#"):
            while index < len(raw) and raw[index] not in b"\r\n":
                index += 1
            continue
        start = index
        while index < len(raw) and raw[index] not in b" \t\r\n":
            index += 1
        tokens.append(raw[start:index].decode("ascii"))
    magic, width_text, height_text, max_text = tokens
    width = int(width_text)
    height = int(height_text)
    max_value = int(max_text)
    while index < len(raw) and raw[index] in b" \t\r\n":
        index += 1
    if magic == "P5":
        pixels = list(raw[index : index + width * height])
    elif magic == "P2":
        pixels = [int(item) for item in raw[index:].decode("ascii").split()]
    else:
        raise ValueError(f"unsupported map image format {magic}")
    if max_value != 255:
        pixels = [int((value / max_value) * 255) for value in pixels]
    return width, height, pixels[: width * height]
