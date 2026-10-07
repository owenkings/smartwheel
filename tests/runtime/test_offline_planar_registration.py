import json,unittest,math
from pathlib import Path
import numpy as np
from wc_runtime.offline_planar_registration import align,se2,rigid,transform_xy,rotation_difference_deg,TrajectoryRefiner

rng=np.random.default_rng(417);REPORT=[]
def scene():
 a=np.linspace(.3,4.8,1800);b=np.linspace(-2.7,2.3,1800);c=np.linspace(-.6,.9,800)
 return np.concatenate([np.column_stack([a,a*0+2.3]),np.column_stack([a,a*0-2.7]),np.column_stack([b*0+4.8,b]),np.column_stack([c*0+1.8,c])])

def xyz(p):
 levels=np.linspace(-.42,.42,10);return np.column_stack([np.repeat(p,len(levels),axis=0),np.tile(levels,len(p))])

class GeometryTest(unittest.TestCase):
 def record(self,name,r):
  REPORT.append({'case':name,'accepted':r['accepted'],'rejection_reasons':r['rejection_reasons'],
   'correction_translation_m':r['correction_translation_m'],'correction_yaw_deg':r['correction_yaw_deg'],
   'final_quality':r.get('final_quality')})
 def test_known_se2(self):
  target=scene();truth=se2(.14,-.09,math.radians(4.));source=transform_xy(np.linalg.inv(truth),target)+rng.normal(0,.002,target.shape)
  initial=se2(.21,-.15,math.radians(1.5));r=align(xyz(source),xyz(target),initial);self.record('known_se2',r)
  self.assertTrue(r['accepted'],r['rejection_reasons']);T=np.array(r['T_target_source'])
  self.assertLess(np.linalg.norm(T[:2,3]-truth[:2,3]),.025);self.assertLess(rotation_difference_deg(T,truth),.5)
  self.assertLess(np.max(abs(T[:3,:3].T@T[:3,:3]-np.eye(3))),1e-12);self.assertAlmostEqual(np.linalg.det(T[:3,:3]),1,12)
 def test_corridor_degeneracy(self):
  x=np.linspace(-4,4,3000);target=np.vstack([np.column_stack([x,x*0+1]),np.column_stack([x,x*0-1])])
  r=align(xyz(target),xyz(target),np.eye(4));self.record('parallel_wall_corridor',r)
  self.assertFalse(r['accepted']);self.assertIn('SE2_GEOMETRIC_DEGENERACY',r['rejection_reasons']);self.assertTrue(np.array_equal(r['T_target_source'],np.eye(4)))
 def test_bad_initial(self):
  target=scene();initial=se2(2.5,2.5,math.radians(20));r=align(xyz(target),xyz(target),initial);self.record('wrong_initial',r)
  self.assertFalse(r['accepted']);self.assertTrue(np.array_equal(r['T_target_source'],initial))
 def test_dynamic_outliers(self):
  target=scene();truth=se2(.09,-.12,math.radians(3.));source=transform_xy(np.linalg.inv(truth),target)+rng.normal(0,.002,target.shape)
  source=np.vstack([source,rng.uniform([.4,-2.5],[4.5,2.],(300,2))]);initial=se2(.15,-.15,math.radians(1.))
  r=align(xyz(source),xyz(target),initial);self.record('dynamic_uniform_outliers',r)
  self.assertTrue(r['accepted'],r['rejection_reasons']);T=np.array(r['T_target_source']);self.assertLess(np.linalg.norm(T[:2,3]-truth[:2,3]),.04);self.assertLess(rotation_difference_deg(T,truth),.7)
 def test_nonplanar_and_reflection_rejected(self):
  bad=np.eye(4);bad[1,1]=-1
  with self.assertRaises(ValueError):rigid(bad)
  bad=np.eye(4);bad[2,3]=.1
  with self.assertRaises(ValueError):rigid(bad)
 def test_prior_fallback_and_segment(self):
  x=np.linspace(.4,5,3000);target=np.vstack([np.column_stack([x,x*0+1]),np.column_stack([x,x*0-1])])
  engine=TrajectoryRefiner();first=engine.update(1000000000,xyz(target),np.eye(4));self.assertEqual(first['status'],'INITIAL_REFERENCE')
  prior=se2(.1,0,0);r=engine.update(1300000000,xyz(target),prior)
  self.assertFalse(r['accepted']);self.assertTrue(np.allclose(r['T_world_scan'],prior));self.assertEqual(r['status'],'PRIOR_FALLBACK')
  next_prior=se2(.2,0,0);r=engine.update(3000000000,xyz(target),next_prior)
  self.assertEqual(r['status'],'INPUT_GAP_NEW_SEGMENT');self.assertTrue(np.allclose(r['T_world_scan'],next_prior));self.assertEqual(r['segment_id'],1)
 def test_nan_and_input_budget(self):
  target=scene();bad=np.full((100,3),np.nan);r=align(bad,xyz(target),np.eye(4));self.assertFalse(r['accepted']);self.assertTrue(np.array_equal(r['T_target_source'],np.eye(4)))
  with self.assertRaises(ValueError):align(np.zeros((200001,3)),xyz(target),np.eye(4))

 def test_world_gauge_equivariance(self):
  target=scene();A=TrajectoryRefiner();B=TrajectoryRefiner();G=se2(12.,-7.,.7)
  for i in range(8):
   true=se2(i*.04,i*.01,i*.008);prior=se2(i*.047,i*.009,i*.01)
   points=xyz(transform_xy(np.linalg.inv(true),target))
   ra=A.update(1000000000+i*250000000,points,prior)
   rb=B.update(1000000000+i*250000000,points,G@prior)
   self.assertEqual(ra['status'],rb['status']);self.assertTrue(np.allclose(np.array(rb['T_world_scan']),G@np.array(ra['T_world_scan']),atol=1e-8))
  self.assertGreater(A.accepted,0)
 def test_rejection_does_not_enter_submap_and_segment_retains_offset(self):
  x=np.linspace(.4,5,1500);points=xyz(np.vstack([np.column_stack([x,x*0+1]),np.column_stack([x,x*0-1])]))
  e=TrajectoryRefiner();G=se2(8.,-4.,.4);e.update(1000000000,points,G)
  original=e.history[0][0]
  for i in range(1,5):
   P=G@se2(i*.03,0,0);r=e.update(1000000000+i*300000000,points,P)
   self.assertEqual(len(e.history),1);self.assertEqual(e.history[0][0],original);self.assertTrue(np.allclose(r['T_world_scan'],P))
  for i in range(5,9):
   P=G@se2(i*.03,0,0);r=e.update(1000000000+i*300000000,points,P)
  self.assertEqual(e.segment,1);self.assertTrue(np.allclose(r['T_world_scan'],P));self.assertGreater(e.history[0][0],original)
 def test_validation_points_excluded(self):
  target=scene();r=align(xyz(target),xyz(target),np.eye(4))
  self.assertTrue(r['accepted']);self.assertTrue(r['heldout_validation']['not_used_in_optimizer'])
  self.assertEqual(r['source_points']+r['validation_source_points'],r['source_points_before_holdout'])


 def test_noisy_corridor_false_observability_rejected(self):
  x,z=np.meshgrid(np.arange(-5,5.01,.07),np.arange(-.42,.421,.07));face=np.column_stack([x.ravel(),np.zeros(x.size),z.ravel()])
  base=np.concatenate([face+[0,1.5,0],face+[0,-1.5,0]]);truth=se2(.13,.02,.015)
  for seed in (781,902,1407):
   for noise in (.0,.005,.01,.02,.03,.05,.07):
    random=np.random.default_rng(seed);target=base+random.normal(0,noise,base.shape)
    source=base@np.linalg.inv(truth)[:3,:3].T+np.linalg.inv(truth)[:3,3]+random.normal(0,noise,base.shape)
    r=align(source,target,np.eye(4));self.record('noisy_corridor_seed%d_noise%g'%(seed,noise),r)
    self.assertFalse(r['accepted'],(seed,noise,r['final_quality']));self.assertTrue(np.array_equal(r['T_target_source'],np.eye(4)))
