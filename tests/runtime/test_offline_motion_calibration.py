import copy
import importlib.util
import os
from pathlib import Path
import unittest

import numpy as np

try:
    from wc_runtime import offline_motion_calibration as m
except ImportError:
    path = Path(os.environ.get('WC_OFFLINE_CALIBRATION_MODULE', str(Path(__file__).with_name('offline_motion_calibration.py'))))
    spec = importlib.util.spec_from_file_location('offline_motion_calibration', path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)


def fixture(total=120., motion_start=22., motion_end=100.):
    rng = np.random.default_rng(524)
    def motion(t):
        active = (t >= motion_start) & (t < motion_end)
        return np.where(active, .45+.10*np.sin(.21*t), 0), np.where(active, .35*np.sin(.35*t), 0)
    imu = []
    bias = np.array([-.0005, -.0004, .0008])
    for t in np.arange(0, total+.001, .01):
        v, w = motion(t)
        g = bias+rng.normal(0, .0006, 3)
        g[2] += .91*w-.014*v
        imu.append({'t_s':float(t),'gyro_native_rad_s':g.tolist(),'accel_native_m_s2':[0.,0.,9.80665], 'device_id':'imu1','session_id':'record1'})
    wheel=[]
    for t in np.arange(0,total+.001,.05):
        v,w=motion(t)
        wheel.append({'t_s':float(t),'v_m_s':float(v),'w_rad_s':float(w),'device_id':'wheel1','session_id':'record1'})
    provenance={'session_id':'record1','imu_device_id':'imu1','wheel_device_id':'wheel1','sources':[{'path':'recorded_evidence','sha256':'a'*64}], 'R_reference_imu':np.eye(3).tolist()}
    windows={'warmup':[0.,10.],'bias_train':[10.,20.],'bias_validation':[105.,115.],'wheel_train':[23.,60.],'wheel_validation':[60.,99.]}
    return imu,wheel,provenance,windows,bias


class OfflineMotionCalibrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.imu,cls.wheel,cls.provenance,cls.windows,cls.bias=fixture()
        cls.candidate=m.build_candidate(cls.imu,cls.wheel,provenance=cls.provenance,windows=cls.windows)

    def test_known_scale_bias_and_heldout_motion_recovered(self):
        c=self.candidate
        self.assertEqual(c['status'],m.VALIDATED,c['validation'])
        np.testing.assert_allclose(c['bias_native_rad_s'],self.bias,atol=.00006)
        self.assertAlmostEqual(c['wheel_yaw_scale'],.91,delta=.001)
        self.assertAlmostEqual(c['wheel_yaw_speed_coefficient'],-.014,delta=.001)
        self.assertEqual(c['intercept_rad_s'],0.)
        h=c['holdout_metrics']['all']
        self.assertLess(h['after_rmse_rad_s'],h['before_rmse_rad_s']*.1)
        self.assertFalse(c['robust_fit']['holdout_residual_filtering'])

    def test_bias_subtracted_once_original_retained(self):
        raw=copy.deepcopy(self.imu[30]);before=copy.deepcopy(raw)
        candidate_before=copy.deepcopy(self.candidate)
        corrected=m.correct_measurement(raw,kind='imu',candidate=self.candidate)
        self.assertEqual(raw,before)
        self.assertEqual(corrected['gyro_native_rad_s'],raw['gyro_native_rad_s'])
        np.testing.assert_allclose(corrected['gyro_corrected_native_rad_s'],np.array(raw['gyro_native_rad_s'])-self.candidate['bias_native_rad_s'])
        with self.assertRaisesRegex(m.CalibrationError,'already applied'):
            m.correct_measurement(corrected,kind='imu',candidate=self.candidate)
        self.assertEqual(self.candidate,candidate_before)

    def test_wheel_correction_preserves_forward_velocity_and_prevents_reuse(self):
        raw=copy.deepcopy(self.wheel[900]);corrected=m.correct_measurement(raw,kind='wheel',candidate=self.candidate)
        self.assertEqual(corrected['v_m_s'],raw['v_m_s'])
        expected=self.candidate['wheel_yaw_scale']*raw['w_rad_s']+self.candidate['wheel_yaw_speed_coefficient']*raw['v_m_s']
        self.assertAlmostEqual(corrected['w_corrected_rad_s'],expected)
        with self.assertRaises(m.CalibrationError):m.correct_measurement(corrected,kind='wheel',candidate=self.candidate)

    def test_cross_device_or_session_rejected(self):
        for key,value in [('device_id','other-imu'),('session_id','other-session')]:
            raw=copy.deepcopy(self.imu[0]);raw[key]=value
            with self.assertRaises(m.CalibrationError):m.correct_measurement(raw,kind='imu',candidate=self.candidate)
        bad=list(self.imu);bad[100]=dict(bad[100],device_id='other-imu')
        with self.assertRaises(m.CalibrationError):m.build_candidate(bad,self.wheel,provenance=self.provenance,windows=self.windows)

    def test_nonfinite_sample_and_candidate_rejected(self):
        bad=list(self.imu);bad[100]=dict(bad[100],gyro_native_rad_s=[0.,float('nan'),0.])
        with self.assertRaises(m.CalibrationError):m.build_candidate(bad,self.wheel,provenance=self.provenance,windows=self.windows)
        bad=copy.deepcopy(self.candidate);bad['wheel_yaw_scale']=float('inf')
        with self.assertRaises(m.CalibrationError):m.validate_for_session(bad,session_id='record1',imu_device_id='imu1',wheel_device_id='wheel1')

    def test_windows_must_be_independent(self):
        for key,bounds in [('wheel_validation',[59.9,99.]),('bias_validation',[19.9,30.]),('warmup',[5.,15.])]:
            windows=dict(self.windows);windows[key]=bounds
            with self.assertRaises(m.CalibrationError):m.build_candidate(self.imu,self.wheel,provenance=self.provenance,windows=windows)

    def test_changed_tail_bias_rejects_instead_of_refitting_validation(self):
        imu=copy.deepcopy(self.imu)
        for r in imu:
            if 105<=r['t_s']<115:r['gyro_native_rad_s'][2]+=.0007
        c=m.build_candidate(imu,self.wheel,provenance=self.provenance,windows=self.windows)
        self.assertEqual(c['status'],'REJECTED')
        self.assertIn('INDEPENDENT_STATIC_BIAS_VALIDATION_FAILED',c['validation']['reasons'])
        np.testing.assert_allclose(c['bias_native_rad_s'],self.candidate['bias_native_rad_s'])

    def test_sparse_training_outliers_filtered_holdout_unchanged(self):
        imu=copy.deepcopy(self.imu)
        for r in imu:
            if any(a<=r['t_s']<a+.2 for a in (28.,35.,45.)):r['gyro_native_rad_s'][2]+=.3
        c=m.build_candidate(imu,self.wheel,provenance=self.provenance,windows=self.windows)
        self.assertEqual(c['status'],m.VALIDATED,c['validation'])
        self.assertGreater(c['robust_fit']['training_rejected_bins'],0)
        self.assertAlmostEqual(c['wheel_yaw_scale'],.91,delta=.002)
        self.assertFalse(c['robust_fit']['holdout_residual_filtering'])
        self.assertEqual(c['holdout_metrics']['all']['samples'],self.candidate['holdout_metrics']['all']['samples'])

    def test_holdout_failure_is_not_removed_as_outlier(self):
        imu=copy.deepcopy(self.imu)
        for r in imu:
            if 60<=r['t_s']<99:r['gyro_native_rad_s'][2]+=.15
        c=m.build_candidate(imu,self.wheel,provenance=self.provenance,windows=self.windows)
        self.assertEqual(c['status'],'REJECTED')
        self.assertIn('HOLDOUT_ABSOLUTE_RMSE_EXCEEDED',c['validation']['reasons'])

    def test_weak_excitation_rejected(self):
        wheel=copy.deepcopy(self.wheel)
        for r in wheel:r['w_rad_s']=0.
        c=m.build_candidate(self.imu,wheel,provenance=self.provenance,windows=self.windows)
        self.assertEqual(c['status'],'REJECTED')
        self.assertIn('TRAIN_WEAK_OR_COLLINEAR_EXCITATION',c['validation']['reasons'])

    def test_source_hash_and_candidate_diagnostics_tampering_rejected(self):
        p=copy.deepcopy(self.provenance);p['sources'][0]['sha256']='not-a-hash'
        with self.assertRaises(m.CalibrationError):m.build_candidate(self.imu,self.wheel,provenance=p,windows=self.windows)
        for mutate in (lambda c:c['covariance_candidates'].__setitem__('wheel_w_variance',.05),lambda c:c['provenance']['sources'][0].__setitem__('sha256','b'*64)):
            c=copy.deepcopy(self.candidate);mutate(c)
            with self.assertRaisesRegex(m.CalibrationError,'hash mismatch'):
                m.validate_for_session(c,session_id='record1',imu_device_id='imu1',wheel_device_id='wheel1')

    def test_explicit_twenty_window_policy_keeps_other_checks_and_default(self):
        imu, wheel, provenance, windows, _ = fixture()
        for row in wheel:
            t = row['t_s']
            if 22 <= t < 60:
                row['w_rad_s'] = -.25 if 25 <= t < 27.8 else abs(row['w_rad_s'])
        for row in imu:
            t = row['t_s']
            if 22 <= t < 60:
                old = .35*np.sin(.35*t)
                new = -.25 if 25 <= t < 27.8 else abs(old)
                row['gyro_native_rad_s'][2] += .91*(new-old)
        default = m.build_candidate(imu, wheel, provenance=provenance, windows=windows)
        self.assertIn('TRAIN_INSUFFICIENT_BOTH_TURN_DIRECTIONS', default['validation']['reasons'])
        candidate = m.build_candidate(imu, wheel, provenance=provenance, windows=windows,
                                      policy={'min_bins_each_turn_direction': 20})
        self.assertEqual(candidate['status'], m.VALIDATED, candidate['validation'])
        self.assertEqual({k:v for k,v in candidate['policy'].items() if k != 'min_bins_each_turn_direction'},
                         {k:v for k,v in m.DEFAULT_POLICY.items() if k != 'min_bins_each_turn_direction'})
        self.assertEqual(m.DEFAULT_POLICY['min_bins_each_turn_direction'], 30)
        self.assertTrue(m.validate_for_session(candidate, session_id='record1', imu_device_id='imu1', wheel_device_id='wheel1'))
        for row in imu:
            if 105 <= row['t_s'] < 115:
                row['gyro_native_rad_s'][2] += .0007
        rejected = m.build_candidate(imu, wheel, provenance=provenance, windows=windows,
                                     policy={'min_bins_each_turn_direction': 20})
        self.assertIn('INDEPENDENT_STATIC_BIAS_VALIDATION_FAILED', rejected['validation']['reasons'])

    def test_auto_windows_do_not_use_regression_quality(self):
        imu,wheel,p,_,_=fixture(total=150,motion_start=35,motion_end=125)
        win=m.select_windows(imu,wheel,provenance=p)
        self.assertLessEqual(win['bias_train'][1],35)
        self.assertGreaterEqual(win['bias_validation'][0],125)
        self.assertEqual(win['wheel_train'][1],win['wheel_validation'][0])
        imu2=copy.deepcopy(imu)
        for r in imu2:
            if 36<r['t_s']<124:r['gyro_native_rad_s'][2]*=1.2
        self.assertEqual(m.select_windows(imu2,wheel,provenance=p),win)

    def test_auto_selection_refuses_missing_stable_end(self):
        imu,wheel,p,_,_=fixture(total=120,motion_start=35,motion_end=121)
        with self.assertRaisesRegex(m.CalibrationError,'stationary segments unavailable'):
            m.select_windows(imu,wheel,provenance=p)


if __name__=='__main__':unittest.main()
