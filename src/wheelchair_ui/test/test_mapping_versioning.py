import json
import subprocess
from pathlib import Path

import yaml

from wheelchair_ui.mapping_manager import MapSaveResult, MappingManager


def test_map_version_manifest_alias_and_activation(tmp_path):
    workspace = tmp_path / "workspace"
    version_dir = workspace / "maps" / "versions"
    version_dir.mkdir(parents=True)

    version_id = "demo_20260527_120000"
    version_pgm = version_dir / f"{version_id}.pgm"
    version_yaml = version_dir / f"{version_id}.yaml"
    version_pgm.write_bytes(b"P5\n2 2\n255\n" + bytes([0, 255, 255, 0]))
    version_yaml.write_text(
        yaml.safe_dump(
            {
                "image": version_pgm.name,
                "resolution": 0.05,
                "origin": [0.0, 0.0, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    quality_report = version_dir / f"{version_id}_quality.json"
    quality_report.write_text(
        json.dumps(
            {
                "verdict": "GOOD",
                "score": 91,
                "metrics": {},
                "reasons": [{"severity": "ok", "message": "地图基础质量检查通过"}],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )

    manager = MappingManager(workspace)
    current_yaml = manager._publish_current_map_alias(version_yaml, "demo")
    quality = manager._load_quality_status(quality_report)
    entry = manager._record_map_version(
        MapSaveResult(
            map_name="demo",
            version_id=version_id,
            version_yaml=version_yaml,
            current_yaml=current_yaml,
        ),
        quality,
        quality_report,
    )

    manifest = json.loads((workspace / "maps" / "map_versions.json").read_text(encoding="utf-8"))
    current_meta = yaml.safe_load(current_yaml.read_text(encoding="utf-8"))

    assert entry["version_id"] == version_id
    assert manifest["latest"] == version_id
    assert manifest["current"]["demo"] == version_id
    assert manifest["versions"][0]["quality"]["verdict"] == "GOOD"
    assert current_meta["image"] == "demo.pgm"
    assert (workspace / "maps" / "demo.pgm").exists()

    status = manager.activate_version(version_id)

    assert status["state"] == "MAP_READY"
    assert status["map_version"]["version_id"] == version_id
    assert status["quality_status"]["verdict"] == "GOOD"


def test_save_map_retries_with_volatile_qos(tmp_path, monkeypatch):
    workspace = tmp_path / "workspace"
    manager = MappingManager(workspace)
    calls = []

    def fake_run(command, **_kwargs):
        calls.append(command)
        prefix = command[command.index("-f") + 1]
        if "map_subscribe_transient_local:=false" not in command:
            return subprocess.CompletedProcess(command, 1, stdout="no transient map", stderr="")

        pgm_path = Path(f"{prefix}.pgm")
        yaml_path = Path(f"{prefix}.yaml")
        pgm_path.parent.mkdir(parents=True, exist_ok=True)
        pgm_path.write_bytes(b"P5\n2 2\n255\n" + bytes([0, 255, 255, 0]))
        yaml_path.write_text(
            yaml.safe_dump(
                {
                    "image": pgm_path.name,
                    "resolution": 0.05,
                    "origin": [0.0, 0.0, 0.0],
                    "negate": 0,
                    "occupied_thresh": 0.65,
                    "free_thresh": 0.196,
                },
                sort_keys=False,
            ),
            encoding="utf-8",
        )
        return subprocess.CompletedProcess(command, 0, stdout="saved", stderr="")

    monkeypatch.setattr(subprocess, "run", fake_run)

    result = manager._save_map("demo")

    assert result.current_yaml.exists()
    assert len(calls) == 2
    assert "map_subscribe_transient_local:=true" in calls[0]
    assert "map_subscribe_transient_local:=false" in calls[1]
