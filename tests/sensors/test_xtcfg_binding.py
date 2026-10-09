"""Read-only provenance and filter bundle failure tests; never opens hardware."""
import hashlib
import importlib.util
import json
from pathlib import Path
import pytest
ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("xtcfg_vendor_checker", ROOT/"src/wc_xt_driver/cmake/verify_vendor.py")
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)

def fixture_bundle(tmp_path):
    root = tmp_path/"filter"
    manifest = {"schema_version": 1, "origin": "synthetic-inert-test",
                "commit": "3d3db067ae9bdc0528202c3087bc10fd3b706638", "files": {}}
    for arch in ("aarch64", "x86_64"):
        rel = "lib/linux/"+arch+"/libxtsdk_shared.so"
        path = root/rel
        path.parent.mkdir(parents=True)
        path.write_bytes(b"INERT TEST BYTES " + arch.encode())
        manifest["files"][rel] = hashlib.sha256(path.read_bytes()).hexdigest()
    file = tmp_path/"manifest.json"
    file.write_text(json.dumps(manifest))
    return root, file

def test_filter_bundle_checks_all_architectures(tmp_path):
    root, manifest = fixture_bundle(tmp_path)
    result = checker.verify_filter_runtime(root, manifest)
    assert len(result["files"]) == 2
    (root/"lib/linux/aarch64/libxtsdk_shared.so").write_bytes(b"wrong ABI")
    with pytest.raises(RuntimeError, match="filter binary hash mismatch"):
        checker.verify_filter_runtime(root, manifest)

def test_filter_bundle_refuses_wrong_origin_and_inventory(tmp_path):
    root, manifest = fixture_bundle(tmp_path)
    data = json.loads(manifest.read_text())
    data["commit"] = "unreviewed"
    manifest.write_text(json.dumps(data))
    with pytest.raises(RuntimeError, match="origin differs"):
        checker.verify_filter_runtime(root, manifest)

def test_project_keeps_complete_requested_fields():
    # Retain all 46 source fields, including the explicitly unsupported viewer switch.
    import configparser
    for side in ("left", "right"):
        path=ROOT/"src/wc_xt_driver/config"/(side+"-2026-09-11.xtcfg")
        c=configparser.ConfigParser();c.optionxform=str;c.read(path)
        assert sum(len(c[s]) for s in c.sections()) == 46
        assert c["Filters"]["spatialEnable"] == "1"
        assert c["Setting"]["pclFilterOn"] == "1"

def test_pinned_patch_replaces_five_frequency_stack_overrun():
    patch=(ROOT/"vendor_patches/xtsdk_ros_965d31a.patch").read_text(encoding="utf-8")
    assert "+        std::array<uint8_t, 5> freq{};" in patch
    assert "wc_before_host_filter" in patch
    assert "bypass_host_geometry" in patch
    assert "wc_xt_driver::allowed_sensor_command(cmdId, data)" in patch
