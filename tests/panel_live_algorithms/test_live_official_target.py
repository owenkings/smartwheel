"""Target-only actual robot_localization checks; no Python substitute or skip."""
import numpy as np
from test_live_motion_options import settings, declaration, feed, candidate
from wc_runtime.mapping_prior import MotionPrior
from wc_runtime.offline_motion_adapter import OfflineCorrectedPrior
from wc_runtime.robot_localization_provider import worker_path


def test_actual_official_live_correction_matches_offline_input_and_full_native_state():
    assert worker_path().is_file()
    c = settings(estimator='robot_localization', motion_correction=True)
    c['live_motion_calibration'] = declaration(c)
    live = MotionPrior(c, 'synthetic')
    off = settings(estimator='robot_localization', offline_experiment=True)
    offline = OfflineCorrectedPrior(off, 'synthetic',
        candidate=candidate(off, bias=(.0003, -.0002, .0008), a=.91, b=-.014),
        candidate_sha256='c'*64, clock=lambda: 1000.)
    stamp = feed(live, queries=True)
    feed(offline)
    a, b = live._planar_state_at(stamp), offline._planar_state_at(stamp)
    ax, ap = a.state()
    bx, bp = b.state()
    np.testing.assert_allclose(ax, bx, atol=1e-13, rtol=0)
    np.testing.assert_allclose(ap, bp, atol=1e-13, rtol=0)
    assert a.report()['last_update']['wheel']['measurement_covariance'][0][1] != 0
    assert live.config['geometry'] is False
