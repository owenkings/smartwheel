"""Runs only when the actual target library worker is installed; no ROS nodes."""
import numpy as np
import pytest

def native():
    from wc_runtime.robot_localization_provider import worker_path,RobotLocalizationEKF
    try: worker_path()
    except (ImportError,LookupError,RuntimeError): pytest.skip('actual target robot_localization worker not installed')
    return RobotLocalizationEKF({})

def test_actual_library_queries_cannot_change_later_authority_state():
    authority=native(); authority.wheel(0,.3,.05); authority.gyro(0,.053)
    control=authority.copy(); query=authority.copy()
    query.predict(.03); query.gyro(1,.07)
    authority.predict(.2); control.predict(.2)
    xa,pa=authority.state(); xc,pc=control.state()
    assert np.array_equal(xa,xc) and np.array_equal(pa,pc)
    assert not np.array_equal(query.state()[0],xa)

def test_actual_library_first_measurement_and_outlier_rejection_are_traced():
    model=native()
    assert model.wheel(0,.2,.03) is True
    assert model.last_update['wheel']['reason']=='INITIAL_MEASUREMENT_INITIALIZATION'
    assert model.P[3,3]==pytest.approx(model.config['wheel_v_variance'])
    before=model.state()
    assert model.gyro(0,5.) is False
    after=model.state()
    assert np.array_equal(before[0],after[0]) and np.array_equal(before[1],after[1])
    assert model.last_update['gyro']['nis']>25.
    assert model.gyro(0,0.) is None

def test_actual_library_unobserved_interval_freezes_pose_without_covariance_reset():
    model=native(); model.wheel(0,.3,.03); model.gyro(0,.035); model.predict(.2)
    before=model.state(); model.predict(1.,observed=False); after=model.state()
    assert np.array_equal(before[0],after[0])
    assert np.allclose(after[1][:6,6:],0.)
    assert np.all(np.diag(after[1])>=np.diag(before[1]))
    assert model.report()['gap_intervals']==1
