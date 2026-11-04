"""Pre-ROS rejection and process ownership checks for the planner sandbox."""
import os
from pathlib import Path
import subprocess
import sys

import pytest

from wc_navigation.nav2_planner import Nav2PlanningError, plan_nav2, _stop_owned


def invoke(tmp_path, **changes):
    options = {"map_yaml": tmp_path / "absent.yaml", "start_xy": [0.0, 0.0],
               "goal_xy": [2.0, 0.0], "radius_m": 0.55, "margin_m": 0.1,
               "output_dir": tmp_path / "result"}
    options.update(changes)
    return plan_nav2(**options)


@pytest.mark.parametrize("name,value", [
    ("start_xy", [float("nan"), 0.0]), ("goal_xy", [0.0, float("inf")]),
    ("start_xy", [0.0]), ("goal_xy", "12"), ("start_xy", {0: 0, 1: 1}),
    ("radius_m", float("nan")), ("radius_m", 0), ("radius_m", -0.1),
    ("margin_m", -0.1), ("margin_m", float("inf")), ("radius_m", None),
    ("timeout_s", 0), ("timeout_s", float("nan")), ("timeout_s", 301),
])
def test_invalid_request_is_rejected_before_ros_or_outputs(tmp_path, name, value):
    with pytest.raises(ValueError):
        invoke(tmp_path, **{name: value})
    assert not (tmp_path / "result").exists()


@pytest.mark.skipif(os.name != "posix", reason="Linux sandbox contract")
@pytest.mark.parametrize("domain,localhost", [(None, "1"), ("0", "1"), ("93", None), ("93", "0")])
def test_isolation_must_be_explicit(tmp_path, monkeypatch, domain, localhost):
    for key, value in [("ROS_DOMAIN_ID", domain), ("ROS_LOCALHOST_ONLY", localhost)]:
        if value is None:
            monkeypatch.delenv(key, raising=False)
        else:
            monkeypatch.setenv(key, value)
    with pytest.raises(Nav2PlanningError, match="ROS_DOMAIN_ID=93"):
        invoke(tmp_path)
    assert not (tmp_path / "result").exists()


@pytest.mark.skipif(os.name != "posix", reason="Linux sandbox contract")
@pytest.mark.parametrize("origin,resolution", [([0, 0, 0.2], 0.05), ([0, 0, 0], 0),
                                                      ([float("nan"), 0, 0], 0.05)])
def test_unsupported_map_is_not_silently_reinterpreted(tmp_path, monkeypatch, origin, resolution):
    import yaml
    monkeypatch.setenv("ROS_DOMAIN_ID", "93")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    source = tmp_path / "map.yaml"
    source.write_text(yaml.safe_dump({"origin": origin, "resolution": resolution}), encoding="utf-8")
    with pytest.raises(ValueError):
        invoke(tmp_path, map_yaml=source)
    assert not (tmp_path / "result").exists()


@pytest.mark.skipif(os.name != "posix", reason="POSIX child process groups")
def test_cleanup_terminates_its_owned_child_only():
    owned = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"], start_new_session=True)
    unrelated = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"], start_new_session=True)
    try:
        _stop_owned(owned)
        assert owned.poll() is not None
        assert unrelated.poll() is None
        _stop_owned(owned)  # Already reaped is harmless.
        assert unrelated.poll() is None
    finally:
        _stop_owned(owned)
        _stop_owned(unrelated)
