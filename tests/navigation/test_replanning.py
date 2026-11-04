import math

import numpy as np
import pytest

from wc_navigation.geometry import Grid, inflate_grid
from wc_navigation.replanning import overlay_observed_discs, find_path


def room(width=120, height=80):
    cells = np.zeros((height, width), dtype=np.int8)
    cells[[0,-1],:] = 100
    cells[:,[0,-1]] = 100
    return Grid(cells, .1)


def assert_certified(inflated, result, start, goal):
    assert result['status'] == 'PATH'
    assert result['path'][0] == list(start)
    assert result['path'][-1] == list(goal)
    assert all(inflated.segment_is_free(a,b) for a,b in zip(result['path'][:-1],result['path'][1:]))
    assert result['metadata']['physical_navigation_validated'] is False
    assert result['metadata']['footprint_relaxed'] is False


def test_disc_covers_intersecting_squares_not_just_center_cell():
    grid = Grid(np.zeros((4,4), dtype=np.int8), 1.)
    before = grid.data.copy()
    over, report = overlay_observed_discs(grid, [{'center_xy':[1.05,1.05],'radius_m':.1}])
    assert set(map(tuple,np.argwhere(over.data==100))) == {(0,0),(0,1),(1,0),(1,1)}
    np.testing.assert_array_equal(grid.data,before)
    assert report['free_to_occupied_cells'] == 4 and report['new_free_cells'] == 0
    assert over is not grid


def test_tangent_cells_are_conservatively_included_but_nonintersecting_corners_are_not():
    grid = Grid(np.zeros((4,4), dtype=np.int8), 1.)
    over, _ = overlay_observed_discs(grid,[{'center_xy':[1.5,1.5],'radius_m':.5}])
    assert set(map(tuple,np.argwhere(over.data==100))) == {(1,0),(0,1),(1,1),(1,2),(2,1)}


def test_existing_unknown_and_occupied_are_preserved_without_free_promotion():
    data = np.zeros((4,4),dtype=np.int8);data[0,0]=-1;data[1,1]=100
    grid = Grid(data,1.)
    over, report = overlay_observed_discs(grid,[{'id':'seen','center_xy':[1.05,1.05],
                                               'radius_m':.1,'velocity_xy':[0,0],'stamp_s':3.}])
    assert over.data[0,0] == -1 and over.data[1,1] == 100
    assert report['covered_unknown_cells_preserved'] == 1
    assert report['covered_existing_occupied_cells'] == 1
    assert report['free_to_occupied_cells'] == 2
    assert np.all((grid.data==0) | (over.data==grid.data))


def test_rotated_translated_grid_preserves_disc_cell_intersection():
    yaw=.71;c,s=math.cos(yaw),math.sin(yaw)
    origin=(4.,-2.,yaw);local=np.array([1.5,1.5]);world=np.array([[c,-s],[s,c]])@local+origin[:2]
    rotated=Grid(np.zeros((4,4),dtype=np.int8),1.,origin)
    plain=Grid(np.zeros((4,4),dtype=np.int8),1.)
    a,_=overlay_observed_discs(rotated,[{'center_xy':world,'radius_m':.5}])
    b,_=overlay_observed_discs(plain,[{'center_xy':local,'radius_m':.5}])
    np.testing.assert_array_equal(a.data,b.data)


def test_partial_and_fully_outside_discs_are_bounded_and_reported():
    grid=Grid(np.zeros((20,20),dtype=np.int8),.1)
    over,report=overlay_observed_discs(grid,[{'center_xy':[-.05,1.],'radius_m':.1},
                                            {'center_xy':[-10.,1.],'radius_m':.1}])
    assert report['partially_clipped_tracks']==1 and report['fully_outside_tracks']==1
    assert np.count_nonzero(over.data==100)>0


def test_empty_observation_snapshot_returns_an_unchanged_fresh_grid():
    grid=room();over,report=overlay_observed_discs(grid,[])
    np.testing.assert_array_equal(grid.data,over.data)
    assert over is not grid and report['covered_cells']==0


def test_circle_outside_map_corner_is_outside_even_when_its_box_touches_map():
    grid=Grid(np.zeros((10,10),dtype=np.int8),.1)
    over,report=overlay_observed_discs(grid,[{'center_xy':[-.1,-.1],'radius_m':.1}])
    assert report['fully_outside_tracks']==1 and report['partially_clipped_tracks']==0
    assert report['covered_cells']==0
    np.testing.assert_array_equal(over.data,grid.data)


@pytest.mark.parametrize('track',[
    {'center_xy':[0,0],'radius_m':0},
    {'center_xy':[float('nan'),0],'radius_m':.1},
    {'center_xy':[0,0],'radius_m':True},
    {'center_xy':[0,0],'radius_m':.1,'active_until_s':10},
    {'center_xy':[0,0],'radius_m':.1,'velocity_xy':[float('inf'),0]},
    {'center_xy':[0,0],'radius_m':.1,'stamp_s':-1},
])
def test_invalid_or_future_world_fields_are_rejected(track):
    with pytest.raises(ValueError):overlay_observed_discs(room(),[track])


def test_wide_known_room_has_a_certified_detour_around_observed_disc():
    grid=room();before=grid.data.copy();start=(1.273,4.064);goal=(10.876,4.132)
    over,_=overlay_observed_discs(grid,[{'center_xy':[6.,4.],'radius_m':.6}])
    inflated=inflate_grid(over,.3,.1)
    assert not inflated.segment_is_free(start,goal)
    result=find_path(inflated,start,goal)
    assert_certified(inflated,result,start,goal)
    assert max(abs(p[1]-4.) for p in result['path'])>.8
    assert result['metadata']['path_length_m']>math.dist(start,goal)
    np.testing.assert_array_equal(grid.data,before)


def test_narrow_corridor_reports_no_route_without_shrinking_footprint():
    data=np.full((80,120),100,dtype=np.int8);data[30:50,1:-1]=0
    grid=Grid(data,.1);start=(1.25,4.05);goal=(10.75,4.05)
    assert find_path(inflate_grid(grid,.55,.1),start,goal)['status']=='PATH'
    over,_=overlay_observed_discs(grid,[{'center_xy':[6.,4.],'radius_m':.3}])
    result=find_path(inflate_grid(over,.55,.1),start,goal)
    assert result['status']=='NO_PATH' and result['path']==[]
    assert result['metadata']['radius_m']==.55 and result['metadata']['margin_m']==.1
    assert 'NOT_PHYSICAL_IMPOSSIBILITY' in result['metadata']['no_path_meaning']


def test_diagonal_corner_cutting_cannot_connect_two_free_pockets():
    data=np.full((8,8),100,dtype=np.int8);data[3,3]=data[4,4]=0
    grid=Grid(data,.1);inflated=inflate_grid(grid,.01)
    start=grid.cell_to_world((3,3));goal=grid.cell_to_world((4,4))
    assert inflated.is_free_world(start) and inflated.is_free_world(goal)
    assert find_path(inflated,start,goal)['status']=='NO_PATH'


def test_unknown_barrier_remains_blocked():
    grid=room(60,40);data=grid.data.copy();data[:,30]=-1;grid=Grid(data,.1)
    result=find_path(inflate_grid(grid,.1),(1.,2.),(5.,2.))
    assert result['status']=='NO_PATH'
    assert np.all(grid.data[:,30]==-1)


def test_free_arbitrary_start_is_not_silently_teleported_to_its_cell_center():
    grid=room(30,20);inflated=inflate_grid(grid,.1)
    start=(.225,.55);goal=(2.,.55)
    assert inflated.is_free_world(start)
    center=grid.cell_to_world(grid.world_to_cell(start))
    assert not inflated.segment_is_free(start,center)
    result=find_path(inflated,start,goal)
    assert result['status']=='NO_PATH'
    assert result['reason']=='NO_CERTIFIED_CONTINUOUS_ENDPOINT_CONNECTOR'


def test_rotated_grid_routes_are_in_world_coordinates_and_keep_exact_endpoints():
    plain=room(100,70);data=plain.data.copy();data[15:55,49:51]=100
    angle=.6;c,s=math.cos(angle),math.sin(angle);rotation=np.array([[c,-s],[s,c]])
    grid=Grid(data,.1,(8.,-3.,angle));start=tuple(rotation@np.array([1.23,3.31])+[8.,-3.])
    goal=tuple(rotation@np.array([8.65,3.22])+[8.,-3.]);inflated=inflate_grid(grid,.2,.05)
    result=find_path(inflated,start,goal)
    assert_certified(inflated,result,start,goal)


def test_same_point_and_blocked_or_outside_endpoints_have_explicit_results():
    grid=room();inflated=inflate_grid(grid,.2);point=(2.134,3.123)
    result=find_path(inflated,point,point);assert result['path']==[list(point)]
    assert find_path(inflated,(-1.,3.),point)['reason']=='START_NOT_CERTIFIED_FREE'
    assert find_path(inflated,point,(.01,.01))['reason']=='GOAL_NOT_CERTIFIED_FREE'


def test_search_budget_exhaustion_is_not_reported_as_disconnection():
    over,_=overlay_observed_discs(room(),[{'center_xy':[6.,4.],'radius_m':.6}])
    result=find_path(inflate_grid(over,.3,.1),(1.25,4.05),(10.75,4.05),max_expansions=1)
    assert result['status']=='RESOURCE_LIMIT' and result['path']==[]


@pytest.mark.parametrize('budget',[0,-1,1.5,True,500001])
def test_invalid_expansion_budgets_are_rejected(budget):
    with pytest.raises(ValueError):find_path(inflate_grid(room(),.2),(2.,3.),(3.,3.),max_expansions=budget)


def test_metric_and_raster_resource_limits_fail_without_mutating_grid():
    grid=room();before=grid.data.copy()
    with pytest.raises(ValueError):overlay_observed_discs(grid,[{'center_xy':[0,0],'radius_m':1}]*101)
    with pytest.raises(ValueError):find_path(inflate_grid(grid,.2),(float('nan'),2.),(3.,3.))
    np.testing.assert_array_equal(grid.data,before)
