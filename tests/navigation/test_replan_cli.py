import numpy as np

from wc_navigation.geometry import Grid
from wc_navigation.replan_cli import ObservedMapPlanner
from wc_navigation.nav2_planner import Nav2PlanningError


def test_nav2_failure_with_connected_independent_grid_is_planner_error(monkeypatch, tmp_path):
    import wc_navigation.replan_cli as cli
    def failure(*args, **kwargs):
        raise Nav2PlanningError('Nav2 did not find a path (action status 6)')
    monkeypatch.setattr(cli, '_plan_nav2_isolated', failure)
    grid = Grid(np.zeros((100, 160), dtype=np.int8), .05)
    planner = ObservedMapPlanner(grid, .2, .05, tmp_path/'planner')
    response = planner({'elapsed_s':10., 'observation_stamp_s':9.9,
                        'pose_x_y_yaw':[1.,2.,0.], 'goal_xy':[7.,2.], 'tracks':[]})
    assert response['status'] == 'PLANNER_ERROR'
    assert response['metadata']['discrete_connectivity']['status'] == 'PATH'
    assert 'action status 6' in response['metadata']['nav2_error']


def test_no_route_requires_independent_no_path_and_base_map_stays_unchanged(monkeypatch, tmp_path):
    import wc_navigation.replan_cli as cli
    def failure(*args, **kwargs):
        raise Nav2PlanningError('Nav2 did not find a path (action status 6)')
    monkeypatch.setattr(cli, '_plan_nav2_isolated', failure)
    data = np.zeros((100,160), dtype=np.int8);data[:,80] = 100;data[0,:]=-1
    grid = Grid(data,.05);before=grid.data.copy()
    planner = ObservedMapPlanner(grid,.2,.05,tmp_path/'planner')
    response = planner({'elapsed_s':10., 'observation_stamp_s':9.9,
                        'pose_x_y_yaw':[1.,2.,0.], 'goal_xy':[7.,2.],
                        'tracks':[{'id':'block','center_xy':[4.,2.], 'velocity_xy':[0.,0.],
                                   'radius_m':.2,'stamp_s':9.9}]})
    assert response['status']=='NO_PATH'
    assert response['metadata']['discrete_connectivity']['status']=='NO_PATH'
    np.testing.assert_array_equal(grid.data,before)


def test_parent_cancellation_terminates_owned_worker(monkeypatch, tmp_path):
    import pytest
    import wc_navigation.replan_cli as cli
    class Worker:
        returncode = None
        terminated = False
        calls = 0
        def wait(self, timeout):
            self.calls += 1
            if self.calls == 1:
                raise KeyboardInterrupt()
            self.returncode = -15
            return -15
        def poll(self): return self.returncode
        def terminate(self): self.terminated = True
        def kill(self): raise AssertionError('graceful cleanup should finish')
    worker = Worker()
    monkeypatch.setattr(cli.subprocess, 'Popen', lambda *a, **k: worker)
    with pytest.raises(KeyboardInterrupt):
        cli._plan_nav2_isolated('map.yaml',[1,2],[3,4],.55,.1,tmp_path/'nav2')
    assert worker.terminated and worker.returncode == -15


def test_unsafe_nav2_interpolation_uses_explicit_certified_grid_fallback(monkeypatch,tmp_path):
    import wc_navigation.replan_cli as cli
    from wc_navigation.geometry import inflate_grid
    def unsafe_path(map_yaml,start,goal,*args,**kwargs):
        return {'path':[start,goal]}
    monkeypatch.setattr(cli,'_plan_nav2_isolated',unsafe_path)
    data=np.zeros((100,160),dtype=np.int8);data[35:65,75:85]=100
    grid=Grid(data,.05);planner=ObservedMapPlanner(grid,.2,.05,tmp_path/'planner')
    response=planner({'elapsed_s':10.,'observation_stamp_s':9.9,
                      'pose_x_y_yaw':[1.,2.5,0.],'goal_xy':[7.,2.5],'tracks':[]})
    assert response['status']=='PATH'
    assert response['metadata']['selected_planner']=='CONSERVATIVE_GRID_ASTAR_AFTER_NAV2_SWEEP_REJECTION'
    assert response['metadata']['rejected_nav2_segments']
    inflated=inflate_grid(grid,.2,.05)
    assert all(inflated.segment_is_free(a,b) for a,b in zip(response['path'],response['path'][1:]))
    assert response['path'][0]==[1.,2.5] and response['path'][-1]==[7.,2.5]
