"""Actual target robot_localization experiment branch; never substitute or skip."""
from pathlib import Path
import sys
import copy

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'operations'))
from test_live_experiment import evidence, live_settings, feed
from wc_runtime.mapping_prior import MotionPrior
from wc_runtime.offline_motion_adapter import OfflineCorrectedPrior
from wc_runtime.robot_localization_provider import worker_path


def test_actual_official_experiment_matches_offline_fixed_parameters_and_native_covariance(evidence):
    assert worker_path().is_file()
    c = live_settings(evidence, estimator='robot_localization', motion_correction=True)
    live = MotionPrior(c, 'new_official_experiment')
    off = copy.deepcopy(c)
    for key in ('live_scheme', 'live_experiment_motion', 'motion_correction'): off.pop(key)
    off['offline_experiment'] = True
    candidate = evidence[3]
    offline = OfflineCorrectedPrior(off, 'synthetic', candidate=candidate,
        candidate_sha256=evidence[4]['motion_candidate']['sha256'], clock=lambda: 1000.)
    stamp = feed(live, bias=candidate['bias_native_rad_s'], extra_queries=True)
    feed(offline, bias=candidate['bias_native_rad_s'])
    a, b = live._planar_state_at(stamp), offline._planar_state_at(stamp)
    ax, ap = a.state(); bx, bp = b.state()
    np.testing.assert_allclose(ax, bx, atol=1e-13, rtol=0)
    np.testing.assert_allclose(ap, bp, atol=1e-13, rtol=0)
    assert a.report()['last_update']['wheel']['measurement_covariance'][0][1] != 0
    assert live.config['confirmed_gyro_bias'] is None
    assert live.bias_report()['gyro_bias_independently_confirmed'] is False
