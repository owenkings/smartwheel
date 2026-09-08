from pathlib import Path


REPO = Path(__file__).resolve().parents[3]


def _read(relative: str) -> str:
    return (REPO / relative).read_text(encoding="utf-8")


def test_save_mapping_result_is_a_strict_formal_session_wrapper():
    source = _read("scripts/save_mapping_result.sh")
    assert "set -Eeuo pipefail" in source
    assert "/map_session/stop" in source
    assert "/map_export/export" in source
    assert "success:[[:space:]]*True" in source
    assert "lio_save_cloud.py" not in source
    assert "map_saver_cli" not in source
    assert 'cp -f "$db_path"' not in source
    assert "accumulate_seconds arguments are no longer supported" in source


def test_lio_save_cloud_is_bounded_and_explicitly_diagnostic():
    source = _read("scripts/lio_save_cloud.py")
    assert "diagnostics only" in source
    assert "SessionCapture" in source
    assert "--max-points" in source
    assert "fixed-window diagnostic only" in source
    assert "os.replace" in source
    assert "self.buf" not in source
    assert "np.vstack" not in source
