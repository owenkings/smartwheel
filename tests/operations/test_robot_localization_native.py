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
def test_posted_command_error_raises_at_next_exchange_and_later_answers_stay_aligned():
    model=native(); worker=model.worker
    worker.post('predict',10**9,.1,1)  # no such native state: its answer is an ERROR
    with pytest.raises(RuntimeError,match='ERROR'):
        worker.command('capabilities')
    assert worker.command('capabilities')==['CAPABILITIES','2','wheel_cov_v1']
    assert model.state()[0].shape==(15,)
def test_posted_updates_match_direct_updates_including_copies_made_before_answers():
    direct,posted=native(),native(); posted.posted_updates=True
    for model in (direct,posted):
        model.wheel(0,.2,.03); model.gyro(0,.05)
    early=posted.copy()  # copied before any posted answer was read
    for model in (direct,posted):
        model.predict(.1); model.gyro(1,5.)
    assert posted.gyro(1,0.) is None
    assert all(np.array_equal(a,b) for a,b in zip(direct.state(),posted.state()))
    assert direct.counts==posted.counts=={'wheel':1,'gyro':1} and direct.rejected_counts==posted.rejected_counts
    assert direct.last_update==posted.last_update and direct.last_sequence==posted.last_sequence
    assert early.counts=={'wheel':1,'gyro':1} and early.last_update['gyro']['sequence']==0
    assert posted.last_update['wheel']['reason']=='INITIAL_MEASUREMENT_INITIALIZATION'
def test_copy_keeps_its_own_bookkeeping_and_dropped_states_leave_the_worker():
    model=native(); model.wheel(0,.2,.03)
    fork=model.copy(); model.gyro(0,.05); model.predict(1.,observed=False)
    assert fork.counts=={'wheel':1,'gyro':0} and 'gyro' not in fork.last_update
    assert fork.last_sequence['imu']==-1 and fork.gap_intervals==0
    t=[.3,.2,0.]
    assert np.array_equal(model.pose(t),model.output(t)[0])
    handle=fork.handle; del fork
    with pytest.raises(RuntimeError,match='ERROR'):
        model.worker.command('get',handle)  # its drop was sent first
def test_native_planar_queries_and_history_retirement_do_not_change_results():
    from test_mapping_prior import config,imu,wheel
    from wc_runtime.mapping_prior import MotionPrior
    native()
    def prior():
        c=config(); c.update(continuous_mapping=True,motion_model='planar_ekf',estimator='robot_localization',
                             policy={'history_s':.3})
        return MotionPrior(c,'synthetic',clock=lambda:1000.)
    quiet,busy=prior(),prior()  # quiet never queries, so each retirement replays
    for i in range(121):
        t=1_000_000_000+i*10_000_000
        for p in (quiet,busy):
            p.add_wheel(wheel(i,t,-100,110)); imu(p,i,t,gyro=(0.,0.,.02))
        if i>3 and i%2:
            busy.record_forwarded(t-5_000_000,busy.pose_at(t-5_000_000))
    a,b=quiet._planar_state_at(t),busy._planar_state_at(t)
    assert all(np.array_equal(u,v) for u,v in zip(a.state(),b.state()))
    assert a.counts==b.counts and a.rejected_counts==b.rejected_counts and a.last_update==b.last_update
    assert np.array_equal(quiet.pose_at(t),busy.pose_at(t))
    assert busy.checkpoints[0]==(busy.planar_anchor[0],None) and len(busy.planar_cache)<40
    assert quiet.failure is None and busy.failure is None
