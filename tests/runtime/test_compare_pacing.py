"""Offline wall pacing preserves source-time selection and per-frame ACK policy."""
import math
import sys
from types import SimpleNamespace

import pytest

from wc_runtime import compare_native, mapping_compare


@pytest.mark.parametrize('interval', [.05, .1, .2, 1., 10.])
def test_valid_wall_interval_is_independent_of_native_sampling_rate(interval):
    mapping_compare.validate_native_wall_interval(interval)
    # The accepted pacing value must not weaken the original native rate gate.
    with pytest.raises(ValueError, match='native replay rate'):
        mapping_compare.validate_comparison_rates([5.], 10.)


@pytest.mark.parametrize('interval', [0., -.1, .049999, 10.000001, math.inf, -math.inf, math.nan])
def test_unbounded_or_too_fast_wall_interval_is_rejected(interval):
    with pytest.raises(ValueError, match='native wall interval'):
        mapping_compare.validate_native_wall_interval(interval)


def test_cli_defaults_to_legacy_one_second_pacing_and_checks_override_before_dataset_read(tmp_path, monkeypatch):
    root = tmp_path/'project'
    source = root/'recorded'
    source.mkdir(parents=True)
    checked = []
    validate = mapping_compare.validate_native_wall_interval

    def observe(interval):
        validate(interval)
        checked.append(interval)

    class ReadReached(Exception):
        pass

    def stop_before_dataset_read(*args, **kwargs):
        raise ReadReached

    monkeypatch.setattr(mapping_compare, 'validate_native_wall_interval', observe)
    monkeypatch.setattr(mapping_compare, 'load_dataset', stop_before_dataset_read)
    base = ['--project-root', str(root), '--dataset', str(source),
            '--output', str(root/'new_output'), '--estimators', 'five_state']
    for override in ([], ['--native-wall-interval-s', '.2']):
        with pytest.raises(ReadReached):
            mapping_compare.main(base+override)
    assert checked == [1., .2]
    assert not (root/'new_output').exists()
    with pytest.raises(ValueError, match='native wall interval'):
        mapping_compare.main(base+['--native-wall-interval-s', 'nan'])
    assert checked == [1., .2]


@pytest.mark.parametrize('interval', [.05, .2, 1., 10.])
def test_native_dispatch_keeps_selected_stamps_and_ack_timeout(tmp_path, monkeypatch, interval):
    # Real recorded-time subsampling is retained; only ROS and the actual worker
    # are replaced, so this verifies the CLI-to-worker integration without ROS.
    root = tmp_path/'project'
    root.mkdir()
    output = tmp_path/'analysis'
    output.mkdir()
    stamps = [1_000_000_000, 1_050_000_000, 1_200_000_000,
              1_390_000_000, 1_400_000_000, 1_600_000_000]
    rows = {stamp: {'stamp_ns': stamp} for stamp in stamps}
    calls, lifecycle = [], []
    ros = SimpleNamespace(init=lambda **kwargs: lifecycle.append('init'),
                          shutdown=lambda: lifecycle.append('shutdown'))
    monkeypatch.setenv('ROS_DOMAIN_ID', '89')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setitem(sys.modules, 'rclpy', ros)
    monkeypatch.setattr(mapping_compare.time, 'sleep', lambda seconds: None)
    monkeypatch.setattr(compare_native, 'load_index', lambda directory: rows)

    def native_worker(root, frontend, native, name, received_rows, selected, args, rclpy, **kwargs):
        calls.append({'stamps': list(selected), 'wall_interval': args.wall_interval,
                      'input_hz': args.input_hz, 'frame_timeout': args.frame_timeout,
                      'same_rows': received_rows is rows})
        return {'status': 'FAKE_WORKER_ONLY'}

    monkeypatch.setattr(compare_native, 'run_cell', native_worker)
    args = SimpleNamespace(domain=89, native_rate_hz=5., native_limit=0,
                           native_wall_interval_s=interval)
    result = mapping_compare.run_native_maps(root, output,
        {'five_state_hz5_filter_on': {'directory': output/'cell'}}, stamps, args)
    assert calls == [{'stamps': [1_000_000_000, 1_200_000_000, 1_400_000_000, 1_600_000_000],
                      'wall_interval': interval, 'input_hz': 5., 'frame_timeout': 30., 'same_rows': True}]
    assert result['selected_original_stamp_ns'] == calls[0]['stamps']
    assert lifecycle == ['init', 'shutdown']
