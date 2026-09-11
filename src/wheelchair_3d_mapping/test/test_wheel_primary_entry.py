"""Offline wiring checks. Construct no device processes or motor commands."""
import ast
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
LAUNCH = ROOT / 'src/wheelchair_bringup/launch/right_lidar_diag_mapping.launch.py'
RVIZ = ROOT / 'src/wheelchair_bringup/rviz/wheel_imu_mapping.rviz'


def nodes():
    result = {}
    for node in ast.walk(ast.parse(LAUNCH.read_text())):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == 'Node':
            kw = {item.arg: item.value for item in node.keywords}
            if isinstance(kw.get('name'), ast.Constant):
                result[kw['name'].value] = kw
    return result


def test_fastlio_tf_and_aids_are_disabled_in_shadow_mode():
    kw = nodes()['fastlio_mapping']
    params = kw['parameters'].elts[1]
    entries = {ast.literal_eval(k): ast.unparse(v) for k, v in zip(params.keys, params.values)}
    assert entries['publish.tf_en'] == 'not wheel_primary'
    assert entries['wheel_update.enabled'] == 'False if wheel_primary else lio_wheel_aiding'
    assert entries['zupt.enabled'] == 'False if wheel_primary else lio_zupt'
    remaps = kw['remappings']
    assert isinstance(remaps, ast.IfExp) and ast.unparse(remaps.test) == 'wheel_primary'
    assert dict(ast.literal_eval(remaps.body))['/Odometry'] == '/lio/shadow/odometry'


def test_only_ekf_owns_vehicle_tf_in_wheel_mode():
    graph = nodes()
    for name in ('lio_map_to_camera_init', 'lio_body_to_base_link', 'cloud_to_occupancy_grid_node'):
        assert ast.unparse(graph[name]['condition']) == 'IfCondition(str(not wheel_primary).lower())'
    assert "'publish_tf': False" in ast.unparse(graph['wheel_imu_ekf']['parameters'])
    assert "'publish_tf': False" in ast.unparse(graph['zlac8030_driver_node']['parameters'])
    text = LAUNCH.read_text()
    assert 'wheel_imu pose owner requires use_wheel_imu_ekf=true' in text
    assert 'build_wheel_primary_nodes' in text


def test_rviz_does_not_display_shadow_as_vehicle_motion():
    rviz = yaml.safe_load(RVIZ.read_text())
    manager = rviz['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == 'odom'
    topics = {d['Topic']['Value'] for d in manager['Displays'] if isinstance(d.get('Topic'), dict)}
    assert {'/wheel_mapping/cloud_registered', '/wheel_mapping/path',
            '/wheel_mapping/odometry_gated', '/rtabmap/optimized_cloud', '/rtabmap/grid_map'} <= topics
    assert not any(topic.startswith('/lio/') or topic in ('/path', '/Odometry', '/cloud_registered')
                   for topic in topics)
    live = next(d for d in manager['Displays'] if d.get('Name') == 'Live Right XYZI (Wheel TF)')
    assert live['Color Transformer'] == 'Intensity'
    assert live['Channel Name'] == 'intensity'


def test_new_operator_entry_has_unique_database_and_never_stops_other_stacks():
    text = (ROOT / 'scripts/run_wheel_imu_mapping.sh').read_text()
    assert 'pose_owner:=wheel_imu' in text
    assert '${MOTION:-false}' in text
    assert 'mktemp -d' in text
    assert 'stop_mapping.sh' not in text
    assert 'pkill' not in text and 'killall' not in text
    assert 'Exit its own terminal first' in text
    assert 'unset AMENT_PREFIX_PATH' in text
