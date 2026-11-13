"""Live ultrasonic ranges: configuration, REP 117 values, CAD geometry, status and the polling loop (no device)."""
import copy
import json
import math
from pathlib import Path

import pytest

from wc_runtime import ultrasonic_live as live

ROOT = Path(__file__).resolve().parents[2]


def config():
    return json.loads((ROOT / 'config/ultrasonic_live.json').read_text(encoding='utf-8'))


def setup():
    return json.loads((ROOT / 'config/hardware_setup.json').read_text(encoding='utf-8'))


def row(address, status='VALID_RESPONSE', raw=500, at=1_000_000_000):
    value = {'address': address, 'status': status, 'host_receive_ns': 1_794_000_000_000_000_000 + at,
             'host_monotonic_ns': at}
    if status == 'VALID_RESPONSE':
        value['response_value_u16'] = raw
    return value


def test_the_repository_configuration_is_valid_and_reports_everything_still_open():
    cfg = live.validate_live_config(config())
    assert sorted(p['name'] for p in cfg['probes']) == sorted(live.PROBES)
    assert live.not_verified(cfg) == ['address_mapping_status', 'raw_unit_status', 'no_echo_status',
                                      'range_limits_status', 'geometry_tape_check']
    done = dict(cfg, address_mapping_status='VERIFIED', raw_unit_status='VERIFIED', no_echo_status='MEASURED',
                range_limits_status='MEASURED', geometry_tape_check='DONE')
    assert live.not_verified(live.validate_live_config(done)) == []


@pytest.mark.parametrize('change', [
    lambda c: c['probes'].pop(),
    lambda c: c['probes'][0].update(address=2),
    lambda c: c['probes'][0].update(name='front'),
    lambda c: c['probes'][0].update(view='up'),
    lambda c: c['probes'][0].update(extra=1),
    lambda c: c.update(address_mapping_status='GUESSED'),
    lambda c: c.update(geometry_status='MEASURED'),
    lambda c: c.update(raw_unit_m=0),
    lambda c: c.update(min_range_m=3.0),
    lambda c: c.update(field_of_view_rad=4.0),
    lambda c: c.update(no_echo_raw=[70000]),
    lambda c: c.update(status_period_s=0.1),
    lambda c: c.update(schema_version=2)])
def test_the_live_configuration_is_strict(change):
    cfg = config()
    change(cfg)
    with pytest.raises(ValueError):
        live.validate_live_config(cfg)


def test_ranges_follow_rep_117():
    cfg = dict(config(), raw_unit_m=0.001, min_range_m=0.25, max_range_m=2.0, no_echo_raw=[0])
    assert live.range_value(row(1, raw=500), cfg) == pytest.approx(0.5)
    assert live.range_value(row(1, raw=2500), cfg) == math.inf           # beyond max_range: nothing in range
    assert live.range_value(row(1, raw=120), cfg) == -math.inf           # below min_range: too close
    assert live.range_value(row(1, raw=0), cfg) == math.inf              # the no-echo value
    for status in ('RESPONSE_TIMEOUT', 'CRC_ERROR', 'MODBUS_EXCEPTION'):
        assert live.range_value(row(1, status=status), cfg) is None      # no measurement: no Range


def test_cad_geometry_is_the_dual_hole_face_relative_to_the_same_side_lidar():
    transforms = live.probe_transforms(config(), setup())
    left_front, right_side = transforms['left_front'], transforms['right_side']
    assert left_front['status'] == 'CAD_NOMINAL' and left_front['frame_id'] == 'ultrasonic_left_front'
    assert left_front['lidar_frame'] == 'lidar_left'
    # mechanical_reference: lidar face (50, 228.75, 133.3) mm, module face (50, 259.5, 76.15) mm, view +x
    assert left_front['T_lidar_probe']['translation_m'] == pytest.approx([0.0, 0.03075, -0.05715])
    assert left_front['T_lidar_probe']['yaw_rad'] == 0.0
    # axle_from_mounting_M (0.273, 0, 0.4582) m, unit rotation
    assert left_front['T_base_probe']['translation_m'] == pytest.approx([0.323, 0.2595, 0.53435])
    assert right_side['T_lidar_probe']['translation_m'] == pytest.approx([-0.05, -0.08075, -0.05715])
    assert right_side['T_base_probe']['translation_m'] == pytest.approx([0.273, -0.3095, 0.53435])
    assert right_side['T_base_probe']['yaw_rad'] == pytest.approx(-math.pi / 2)
    assert right_side['T_base_probe']['rotation_xyzw'] == pytest.approx([0, 0, -math.sqrt(0.5), math.sqrt(0.5)])
    assert transforms['left_side']['T_base_probe']['yaw_rad'] == pytest.approx(math.pi / 2)
    wrong = config()
    wrong['probes'][0]['view'] = 'left'                     # the CAD says this module looks forward
    with pytest.raises(ValueError, match='view'):
        live.probe_transforms(wrong, setup())


def test_status_reports_counts_ages_cycles_and_what_is_not_verified():
    cfg = config()
    clock = iter([0.0, 0.0, 0.05, 0.05, 0.1, 0.1])
    book = live.StatusBook(cfg, {'geometry': 'stub'}, config_sha256='abc', cpu_clock=lambda: next(clock, 0.1))
    book.record('left_front', row(1, raw=400, at=1_000_000_000), 0.4)
    book.record('right_front', row(2, status='RESPONSE_TIMEOUT', at=1_200_000_000), None)
    book.record('left_side', row(3, raw=3000, at=1_400_000_000), math.inf)
    book.cycle(900_000_000, 1_800_000_000)
    message = book.message(now_ns=2_000_000_000)
    json.dumps(message, allow_nan=False)                     # +inf goes out as a string
    assert message['verified'] is False and 'address_mapping_status' in message['not_verified']
    assert message['state'] == 'DEGRADED' and set(message['failing_probes']) == {'right_front', 'right_side'}
    probes = message['probes']
    assert probes['left_front']['raw'] == 400 and probes['left_front']['range_m'] == 0.4
    assert probes['left_front']['age_s'] == pytest.approx(1.0)
    assert probes['left_side']['range_m'] == '+inf'
    assert probes['right_front']['counts'] == {'RESPONSE_TIMEOUT': 1}
    assert probes['right_front']['consecutive_failures'] == 1 and probes['right_side']['status'] is None
    assert message['cycle_s'] == {'last': 0.9, 'median': 0.9, 'max': 0.9}
    assert message['config_sha256'] == 'abc' and message['transactions'] == 3


class FakeLease:
    replies = {1: (b'\x01\x03\x02\x01\xf4', 'VALID_RESPONSE', 500), 2: (b'', 'RESPONSE_TIMEOUT', None),
               3: (b'', 'VALID_RESPONSE', 2600), 4: (b'', 'VALID_RESPONSE', 100)}
    opened = closed = 0

    def __init__(self, config, run_root):
        self.fd = 3

    def open(self):
        FakeLease.opened += 1
        return self

    def exchange(self, address):
        _, status, raw = self.replies[address]
        value = row(address, status=status, raw=raw or 0)
        value.update(request_hex='', response_hex='', response_sha256='')
        return value

    def close(self):
        FakeLease.closed += 1


def test_the_loop_journals_every_transaction_and_keeps_polling_through_failures(tmp_path, monkeypatch, capsys):
    from wc_runtime import ultrasonic_capture
    monkeypatch.setattr(ultrasonic_capture, 'DistanceLease', FakeLease)
    monkeypatch.setattr(ultrasonic_capture, 'observed_identity', lambda *a: {'source_id': 'ultrasonic'})
    capture = json.loads((ROOT / 'config/ultrasonic_capture.json').read_text(encoding='utf-8'))
    capture['inter_sensor_delay_s'] = 0.1
    (tmp_path / 'capture.json').write_text(json.dumps(capture), encoding='utf-8')
    out = tmp_path / 'ultrasonic'
    code = live.main(['--capture-config', str(tmp_path / 'capture.json'),
                      '--live-config', str(ROOT / 'config/ultrasonic_live.json'),
                      '--hardware-setup', str(ROOT / 'config/hardware_setup.json'),
                      '--output', str(out), '--run-root', str(tmp_path / 'run'), '--session', 's',
                      '--allow-read-queries', '--no-ros', '--cycles', '3'])
    assert code == 0 and FakeLease.opened == FakeLease.closed == 1
    rows = [json.loads(line) for line in (out / 'events.jsonl').read_text().splitlines()]
    transactions = [r for r in rows if r['event'] == 'transaction']
    assert len(transactions) == 12 and [r['sequence'] for r in transactions] == list(range(12))
    assert {r['probe'] for r in transactions if r['address'] == 2} == {'right_front'}
    final = rows[-1]
    assert final['event'] == 'source_finalization' and final['completed_transactions'] == 12
    assert final['counts']['right_front'] == {'RESPONSE_TIMEOUT': 3} and final['cycles'] == 3
    summary = json.loads((out / 'summary.json').read_text())
    assert summary['closed_normally'] is True and summary['event_counts'] == {'transaction': 12,
                                                                              'source_finalization': 1}
    lines = [json.loads(line) for line in capsys.readouterr().out.splitlines() if line.startswith('{')]
    assert len(lines) == 3 and lines[-1]['raw'] == {'left_front': 500, 'right_front': None, 'left_side': 2600,
                                                     'right_side': 100}


def test_reading_needs_the_explicit_selection(tmp_path):
    with pytest.raises(ValueError, match='explicit'):
        live.main(['--live-config', str(ROOT / 'config/ultrasonic_live.json'),
                   '--hardware-setup', str(ROOT / 'config/hardware_setup.json'),
                   '--output', str(tmp_path / 'o'), '--run-root', str(tmp_path)])


def test_the_field_binding_is_written_as_a_local_override_only(tmp_path):
    (tmp_path / 'config').mkdir()
    base = (ROOT / 'config/ultrasonic_capture.json').read_bytes()
    (tmp_path / 'config/ultrasonic_capture.json').write_bytes(base)
    report = {'ultrasonic': {'adapters_found': [{'node': '/dev/ttyUSB0',
                                                 'usb_path': 'platform-3610000.usb-usb-0:1:1.0'}],
                             'usb_path_now': 'platform-3610000.usb-usb-0:1:1.0', 'matches': False}}
    target = live.write_local_binding(tmp_path, report)
    assert target == tmp_path / 'config/local/project/ultrasonic_capture.json'
    written = json.loads(target.read_text(encoding='utf-8'))
    assert written['expected_usb_path'] == 'platform-3610000.usb-usb-0:1:1.0'
    assert (tmp_path / 'config/ultrasonic_capture.json').read_bytes() == base
    from wc_runtime.project_config import selected_config_path
    assert selected_config_path(tmp_path, 'ultrasonic_capture.json') == target
    with pytest.raises(RuntimeError, match='EXISTS'):
        live.write_local_binding(tmp_path, report)
    two = copy.deepcopy(report)
    two['ultrasonic']['adapters_found'].append({'node': '/dev/ttyUSB1', 'usb_path': 'x'})
    two['ultrasonic']['problem'] = 'two'
    with pytest.raises(RuntimeError, match='AMBIGUOUS'):
        live.write_local_binding(tmp_path, two, replace=True)
