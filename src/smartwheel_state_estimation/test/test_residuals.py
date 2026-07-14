from smartwheel_state_estimation.residuals import OdomVelocity, ResidualMonitor


def test_residual_monitor_detects_degradation():
    monitor = ResidualMonitor(max_linear=0.2, max_angular=0.4)
    good = monitor.compare(OdomVelocity(0.3, 0.1), OdomVelocity(0.25, 0.05))
    assert not good["degraded"]
    assert good["score"] > 0.0
    bad = monitor.compare(OdomVelocity(0.6, 1.0), OdomVelocity(0.0, 0.0))
    assert bad["degraded"]
    assert bad["score"] == 0.0

