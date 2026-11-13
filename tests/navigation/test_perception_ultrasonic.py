"""Ultrasonic modules as collision-monitor range sources (perception.ultrasonic), off by default."""
import math

import numpy as np
import pytest

from wc_navigation.autonomy import nav2_config
from wc_navigation.perception import ultrasonic as us

MEASURED = [(0.81, 0.2425), (0.2, 0.35), (-0.255, 0.35), (-0.255, -0.35), (0.2, -0.35), (0.81, -0.2425)]


def params(profile):
    return nav2_config.navigation_parameters(profile, mode="navigate", bt_to_pose_path="a",
                                             bt_through_poses_path="b")["collision_monitor"]["ros__parameters"]


def test_arc_points_follow_the_installed_range_source():
    # range.cpp: one point per obstacles_angle from -fov/2 while below +fov/2, then the +fov/2 edge; the angle is a
    # float summed step by step, so at 60 deg the 61st step lands a hair below +30 deg and the edge comes on top
    arc = us.arc_points(1.0, math.radians(60), 0.2, 3.0)
    assert len(arc) == 62
    assert arc[0] == pytest.approx([math.cos(math.radians(30)), -0.5], abs=1e-6)
    assert arc[-1] == pytest.approx([math.cos(math.radians(30)), 0.5], abs=1e-6)
    assert arc[-2] == pytest.approx(arc[-1], abs=1e-6)
    assert np.hypot(arc[:, 0], arc[:, 1]) == pytest.approx(np.ones(62), abs=1e-6)
    # out of the module's span: ignored (no echo +inf, too close -inf, beyond max_range, inside the blind zone)
    for r in (math.inf, -math.inf, 3.5, 0.15):
        assert len(us.arc_points(r, math.radians(60), 0.2, 3.0)) == 0
    assert len(us.arc_points(3.0, math.radians(60), 0.2, 3.0)) == 62          # the span's ends are inside
    assert np.isnan(us.arc_points(math.nan, math.radians(60), 0.2, 3.0)).all()  # NaN passes, no zone holds it
    # a narrow field of view gives few points: a zone acts on more than 3 (MONITOR_MAX_POINTS)
    assert len(us.arc_points(0.3, 0.0, 0.2, 3.0)) == 1
    assert len(us.arc_points(0.3, math.radians(1.7), 0.2, 3.0)) == 3
    assert len(us.arc_points(0.3, math.radians(3.0), 0.2, 3.0)) == 5


def test_points_move_into_base_link_by_the_mount():
    centre = us.in_base(np.array([[0.5, 0.0]]), 0.273, 0.3095, math.pi / 2)
    assert centre[0] == pytest.approx([0.273, 0.8095])


def test_range_health_is_a_brake_never_a_release():
    reading = {"range": 1.2, "min_range": 0.2, "max_range": 3.0}
    assert us.range_health(reading, 0.3) == (True, "")
    assert us.range_health(dict(reading, range=math.inf), 0.3) == (True, "")        # no echo: nothing near
    assert us.range_health(dict(reading, range=4.0), 0.3) == (True, "")
    assert us.range_health(dict(reading, range=-math.inf), 0.3) == (False, "ULTRASONIC_TOO_CLOSE")
    assert us.range_health(dict(reading, range=0.1), 0.3) == (False, "ULTRASONIC_TOO_CLOSE")
    assert us.range_health(dict(reading, range=math.nan), 0.3) == (False, "ULTRASONIC_INVALID")
    assert us.range_health(dict(reading, min_range=0.0), 0.3) == (False, "ULTRASONIC_INVALID")
    assert us.range_health({"range": 1.0}, 0.3) == (False, "ULTRASONIC_INVALID")
    assert us.range_health(reading, us.RANGE_MAX_AGE_S + 0.1) == (False, "ULTRASONIC_OLD")
    assert us.range_health(reading, -0.5) == (False, "ULTRASONIC_OLD")
    assert us.range_health(None, None) == (False, "ULTRASONIC_NO_DATA")


def test_the_modules_are_checked_and_ordered():
    assert us.check_probes(("right_front", "left_front")) == (("left_front", "right_front"), "")
    assert us.check_probes(us.PROBES, "/wc_live/relayed")[1] == "/wc_live/relayed"
    for probes, prefix in ((("left_rear",), ""), (("left_front", "left_front"), ""), ((), "/x"),
                           (("left_front",), "x"), (("left_front",), "/x/")):
        with pytest.raises(ValueError):
            us.check_probes(probes, prefix)
    assert us.MODES["front"] == us.FRONT and us.MODES["off"] == ()
    assert us.topic("left_side", "/wc_live/relayed") == "/wc_live/relayed/wc_mapping/ultrasonic/left_side/range"


def test_off_by_default_and_the_profile_records_only_what_is_set():
    default = nav2_config.NavProfile(footprint=MEASURED)
    assert "monitor_ultrasonic" not in default.as_dict() and "monitor_ultrasonic_prefix" not in default.as_dict()
    monitor = params(default)
    assert monitor["observation_sources"] == ["scan"] and not any(k.startswith("ultrasonic_") for k in monitor)
    assert not any(k.startswith("ultrasonic_") for k in nav2_config.readback_expectations(default, "navigate")
                   ["collision_monitor"])
    front = nav2_config.NavProfile(footprint=MEASURED, monitor_ultrasonic=("right_front", "left_front"))
    assert front.as_dict()["monitor_ultrasonic"] == ["left_front", "right_front"]
    assert nav2_config.NavProfile(**front.as_dict()) == front
    with pytest.raises(ValueError):
        nav2_config.NavProfile(monitor_ultrasonic=("left_rear",))


def test_range_sources_join_the_monitor_and_its_readback():
    combo = dict(footprint=MEASURED, obstacle_input="scan", obstacle_clouds=("left", "right"),
                 obstacle_scan="filtered", obstacle_monitor="cloud")
    profile = nav2_config.NavProfile(**combo, monitor_ultrasonic=us.PROBES,
                                     monitor_ultrasonic_prefix="/wc_live/relayed")
    monitor = params(profile)
    names = ["ultrasonic_" + p for p in us.PROBES]
    assert monitor["observation_sources"] == ["cloud_left", "cloud_right"] + names
    for p in us.PROBES:
        source = monitor["ultrasonic_" + p]
        assert source == {"type": "range", "topic": "/wc_live/relayed/wc_mapping/ultrasonic/%s/range" % p,
                          "obstacles_angle": pytest.approx(math.pi / 180), "enabled": True}
    expected = nav2_config.readback_expectations(profile, "navigate")["collision_monitor"]
    assert expected["observation_sources"] == monitor["observation_sources"]
    assert expected["ultrasonic_left_side.topic"] == monitor["ultrasonic_left_side"]["topic"]
    assert expected["cloud_left.topic"] == monitor["cloud_left"]["topic"]
    # with /scan in the monitor the range sources follow it
    alone = nav2_config.NavProfile(footprint=MEASURED, monitor_ultrasonic=us.FRONT)
    assert params(alone)["observation_sources"] == ["scan", "ultrasonic_left_front", "ultrasonic_right_front"]
    assert nav2_config.readback_expectations(alone, "navigate")["collision_monitor"]["observation_sources"] == \
        ["scan", "ultrasonic_left_front", "ultrasonic_right_front"]
    # the zones do not change
    assert {k: v for k, v in monitor.items() if k in ("FootprintStop", "PolygonSlow", "FootprintApproach")} == \
        {k: v for k, v in params(nav2_config.NavProfile(**combo)).items()
         if k in ("FootprintStop", "PolygonSlow", "FootprintApproach")}


def test_the_live_command_option(tmp_path):
    from wc_navigation.live.cli import arguments
    from wc_navigation.perception.shadow_cli_options import live_profile_fields
    shadow = ["shadow", "--project-root", str(tmp_path), "--cell", "c", "--hardware-setup", "h", "--gyro-bias", "b",
              "--map", "m.yaml", "--initial-pose", "0", "0", "0", "--goal", "1", "2", "0"]
    bag = shadow + ["--source", "bag", "--bag", "x"]
    assert live_profile_fields(arguments(bag)) == {}
    assert live_profile_fields(arguments(bag + ["--monitor-ultrasonic", "off"])) == {}
    assert live_profile_fields(arguments(bag + ["--monitor-ultrasonic", "front"])) == {
        "monitor_ultrasonic": us.FRONT, "monitor_ultrasonic_prefix": ""}
    assert live_profile_fields(arguments(bag + ["--monitor-ultrasonic", "all", "--relay-test"])) == {
        "monitor_ultrasonic": us.PROBES, "monitor_ultrasonic_prefix": "/wc_live/relayed"}
    vehicle = shadow + ["--source", "vehicle", "--sensor-domain", "83", "--monitor-ultrasonic", "front"]
    assert live_profile_fields(arguments(vehicle))["monitor_ultrasonic_prefix"] == "/wc_live/relayed"
    with pytest.raises(SystemExit):
        arguments(bag + ["--monitor-ultrasonic", "left"])
