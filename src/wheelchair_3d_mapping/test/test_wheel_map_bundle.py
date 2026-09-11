"""No hardware/ROS processes: bundle validation and operator wiring checks."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location('wheel_map_bundle', ROOT / 'scripts/mapping/wheel_map_bundle.py')
bundle = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bundle)


def grid():
    return NS(header=NS(frame_id='map'), data=[-1, 0, 100, 20, 50, 70],
              info=NS(width=3, height=2, resolution=0.05,
                      origin=NS(position=NS(x=1., y=-2., z=0.),
                                orientation=NS(x=0., y=0., z=0., w=1.))))


def test_grid_axes_unknown_and_yaml(tmp_path):
    bundle.write_grid(tmp_path, grid())
    content = (tmp_path / 'map_2d.pgm').read_bytes()
    assert content == b'P5\n3 2\n255\n' + bytes([254, 205, 0, 205, 254, 0])
    cfg = yaml.safe_load((tmp_path / 'map_2d.yaml').read_text())
    assert cfg['origin'] == [1., -2., 0.]
    assert cfg['resolution'] == 0.05


@pytest.mark.parametrize('fault', ['frame', 'size', 'resolution', 'unknown', 'range', 'tilt', 'origin_z', 'nan'])
def test_invalid_grid_rejected(fault):
    item = grid()
    if fault == 'frame': item.header.frame_id = 'odom'
    if fault == 'size': item.info.width = 5
    if fault == 'resolution': item.info.resolution = float('nan')
    if fault == 'unknown': item.data = [-1] * 6
    if fault == 'range': item.data[0] = 101
    if fault == 'tilt': item.info.origin.orientation.x = 0.1
    if fault == 'origin_z': item.info.origin.position.z = 0.2
    if fault == 'nan': item.info.origin.position.x = float('nan')
    with pytest.raises(ValueError): bundle.validate_grid(item)


def make_bundle(path):
    names = ['map_data.cdr', 'grid.cdr', 'map_xyzi.pcd', 'map_xyzi.ply',
             'map_2d.pgm', 'map_2d.yaml', 'trajectory.csv', 'backend.db']
    for name in names: (path / name).write_bytes(name.encode())
    manifest = {'schema_version': 1, 'complete': True, 'files': {
        name: {'size': (path / name).stat().st_size, 'sha256': bundle.sha256(path / name)} for name in names}}
    (path / 'manifest.json').write_text(json.dumps(manifest))
    return manifest


def test_manifest_checksums_and_incomplete(tmp_path):
    make_bundle(tmp_path)
    bundle.verify_bundle(tmp_path)
    (tmp_path / '.incomplete').touch()
    with pytest.raises(ValueError, match='incomplete'): bundle.verify_bundle(tmp_path)
    (tmp_path / '.incomplete').unlink()
    (tmp_path / 'map_xyzi.pcd').write_bytes(b'bad')
    with pytest.raises(ValueError, match='checksum'): bundle.verify_bundle(tmp_path)


def test_manifest_missing_and_traversal(tmp_path):
    data = make_bundle(tmp_path)
    del data['files']['grid.cdr']
    (tmp_path / 'manifest.json').write_text(json.dumps(data))
    with pytest.raises(ValueError, match='missing'): bundle.verify_bundle(tmp_path)
    data = make_bundle(tmp_path)
    data['files']['../outside'] = {'size': 0, 'sha256': 'bad'}
    (tmp_path / 'manifest.json').write_text(json.dumps(data))
    with pytest.raises(ValueError, match='path'): bundle.verify_bundle(tmp_path)


def test_record_before_hardware_export_after_shutdown():
    text = (ROOT / 'scripts/run_wheel_imu_mapping.sh').read_text()
    assert text.index('setsid ros2 bag record') < text.index('setsid taskset')
    assert '/imu/data /wheel/odom /base/wheel_feedback_healthy' in text
    assert '/tf /tf_static' in text
    assert text.index('wait "$launch_pid"') < text.index('wheel_map_bundle.py" export')
    assert '${FASTLIO_SHADOW:-false}' in text
    assert '${MOTION:-false}' in text


def test_offline_isolation_and_no_source_overwrite():
    text = (ROOT / 'scripts/mapping/wheel_map_bundle.py').read_text()
    assert "os.environ['ROS_DOMAIN_ID'] = '93'" in text
    assert "'?mode=ro'" in text
    assert 'reader.backup(writer)' in text
    assert 'hardware_validated\': False' in text
    assert 'xtm60_adapter_node' not in text and 'zlac8030_driver_node' not in text
    assert "'output already exists;" in text
