import math

import numpy as np
import pytest

from wc_navigation.live.bag_check import compare_odometry, compare_scans, read_live_odometry, read_live_scans
from wc_navigation.live.shadow_summary import gate_untrusted


def test_scan_comparison_matches_stamps_and_counts_differing_bins():
    stamps = np.array([1.0, 1.3, 1.6])
    frozen = np.full((3, 161), np.inf)
    frozen[:, 80] = 2.0
    live = frozen.copy()
    live[1, 80] = 2.00005            # within the 0.1 mm tolerance
    live[2, 10] = 3.0                # a return the frozen scan does not have
    result = compare_scans(stamps, frozen, np.array([1.6, 1.0000001, 1.3, 5.0]), live[[2, 0, 1, 0]])
    assert result["matched_frames"] == 3 and result["unmatched_frames"] == 0
    assert result["identical_frames"] == 2 and result["differing_bins_total"] == 1
    assert result["max_abs_difference_finite_m"] == pytest.approx(5e-5, abs=1e-9)
    shifted = compare_scans(stamps, frozen, stamps + 0.01, frozen)
    assert shifted["matched_frames"] == 0 and shifted["unmatched_frames"] == 3


def test_odometry_comparison_ignores_a_different_odometry_origin():
    t = np.linspace(0, 10, 101)
    axle = np.column_stack([0.5 * t, np.zeros_like(t), np.zeros_like(t)])
    # live odometry started elsewhere: rotated by 90 degrees and shifted, otherwise the same motion
    live = np.column_stack([t, 3.0 - 0.0 * t, 1.0 + 0.5 * t, np.full_like(t, math.pi / 2)])
    result = compare_odometry(t, axle, live)
    assert result["status"] == "COMPARED" and result["frames"] == 101
    assert result["position_difference_m"]["max"] < 1e-9 and result["heading_difference_deg"]["max"] < 1e-9
    live[:, 2] += 0.01 * t           # 0.1 m of lateral drift by the end
    drift = compare_odometry(t, axle, live)
    assert drift["position_difference_m"]["final"] == pytest.approx(0.1, abs=1e-6)
    assert compare_odometry(t + 100, axle, live)["status"] == "NO_OVERLAP"


def test_recorder_files_are_read_back(tmp_path):
    (tmp_path / "scans.csv").write_text("time_s,stamp_s,observed_bins,nearest_m," + ",".join("r%03d" % k for k in range(161))
                                        + "\n1.0,2.0,1,0.5," + ",".join(["inf"] * 80 + ["0.50000"] + ["inf"] * 80) + "\n")
    stamps, ranges = read_live_scans(tmp_path / "scans.csv")
    assert stamps.tolist() == [2.0] and ranges[0, 80] == 0.5 and np.isinf(ranges[0, 0])
    (tmp_path / "odom.csv").write_text("time_s,stamp_s,x,y,yaw,linear,angular\n0,1.0,0,0,0,0,0\n0,1.0,1,1,0,0,0\n"
                                       "0,1.1,0.1,0,0,0,0\n")
    odometry = read_live_odometry(tmp_path / "odom.csv")
    assert odometry[:, 0].tolist() == [1.0, 1.1]          # repeated stamps dropped


def test_gate_time_is_split_by_state_and_reason():
    events = [{"time_s": 10.0, "topic": "/wc_nav/gate_status", "value": {"state": "ACTIVE"}},
              {"time_s": 12.0, "topic": "/wc_live/inputs_status", "value": {"state": "RUNNING"}},
              {"time_s": 14.0, "topic": "/wc_nav/gate_status",
               "value": {"state": "LOCALIZATION_UNTRUSTED", "reasons": ["COVARIANCE_XY_TOO_LARGE"]}},
              {"time_s": 15.5, "topic": "/wc_nav/gate_status", "value": {"state": "COMMAND_STALE"}}]
    states, reasons = gate_untrusted(events, 11.0, 20.0)
    assert states == {"ACTIVE": 3.0, "LOCALIZATION_UNTRUSTED": 1.5, "COMMAND_STALE": 4.5}
    assert reasons == {"COVARIANCE_XY_TOO_LARGE": 1.5}
