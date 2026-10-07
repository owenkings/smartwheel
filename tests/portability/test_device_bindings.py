"""Machine-binding and archived identity regressions; no hardware I/O."""
import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from wc_runtime.device_bindings import load_device_bindings, validate_device_bindings
from wc_runtime.mapping_planar import validate_confirmed_bias
from wc_runtime.mapping_bias_confirm import save_holdout_calibration
from wc_motion import feedback_transport
from wc_motion.history_preview import HistoryPreview
from wc_motion.protocol import FeedbackError, crc16
from wc_cameras import config as cameras
from wc_cameras.capture import verify_properties
from wc_runtime import ultrasonic_capture

ROOT = Path(__file__).resolve().parents[2]


def fixture(name):
    return json.loads((ROOT/'config'/name).read_text(encoding='utf-8'))


def binding(serial='SYNTHETIC-IMU'):
    return dict(schema_version=1, imu=dict(device='/dev/my_imu',
        expected_by_id='/dev/serial/by-id/usb-1a86_USB_Single_Serial_'+serial+'-if00',
        hardware_serial=serial, sensor_id='H30-'+serial), network={'interface': None})


def write_binding(root, value, *, local=False):
    directory = root/'config'; directory.mkdir(parents=True, exist_ok=True)
    path = directory/('device_bindings.local.json' if local else 'device_bindings.json')
    path.write_text(json.dumps(value), encoding='utf-8')
    return path


def test_full_local_override_and_frozen_snapshot_with_original_hash(tmp_path):
    root = tmp_path/'another user'/'checkout'
    write_binding(root, binding('BASE'))
    path = write_binding(root, binding('OTHER'), local=True)
    snapshot = load_device_bindings(root)
    assert snapshot['imu']['sensor_id'] == 'H30-OTHER'
    assert snapshot['_provenance']['sha256'] == hashlib.sha256(path.read_bytes()).hexdigest()
    assert snapshot['_provenance']['original_text'].encode() == path.read_bytes()
    write_binding(root, binding('LATER'), local=True)
    assert snapshot['imu']['sensor_id'] == 'H30-OTHER'
    assert load_device_bindings(root)['imu']['sensor_id'] == 'H30-LATER'
    json.dumps(snapshot, allow_nan=False)


@pytest.mark.parametrize('bad', [{'network': {'interface': 'eth0'}}, {'schema_version': 2}])
def test_invalid_local_override_never_merges_or_falls_back(tmp_path, bad):
    write_binding(tmp_path, binding())
    write_binding(tmp_path, bad, local=True)
    with pytest.raises(ValueError): load_device_bindings(tmp_path)


def test_duplicate_binding_keys_rejected(tmp_path):
    path = write_binding(tmp_path, binding())
    path.write_text('{"imu":{},"imu":{}}', encoding='utf-8')
    with pytest.raises(ValueError, match='duplicate'): load_device_bindings(tmp_path)


@pytest.mark.parametrize('field,value', [('hardware_serial','OTHER'), ('sensor_id','H30-OTHER'),
    ('device','/dev/ttyACM0'), ('expected_by_id','/dev/ttyUSB0')])
def test_inconsistent_or_unstable_imu_binding_rejected(field, value):
    value_dict = binding(); value_dict['imu'][field] = value
    with pytest.raises(ValueError): validate_device_bindings(value_dict)


def test_another_wheel_identity_does_not_weaken_protocol_allowlist():
    config = fixture('wheel_feedback_current.json')
    config.update(device='/dev/other_wheel', hardware_serial='NEW-WHEEL', device_id='ZLAC8030D-NEW-WHEEL',
        expected_by_id='/dev/serial/by-id/usb-1a86_USB_Single_Serial_NEW-WHEEL-if00')
    assert feedback_transport.validate_config(config) == config
    with pytest.raises(FeedbackError):
        feedback_transport.validate_config({**config, 'device_id': 'ZLAC8030D-WRONG'})
    with pytest.raises(FeedbackError):
        feedback_transport.validate_config({**config, 'request_hex': '010600010000d80a'})


def explicit_cameras():
    value = fixture('cameras.json'); value['schema_version'] = 2
    for item in value['cameras']:
        path = 'pci-0000:00:14.0-usb-0:1.'+str(item['port'])+':1.0'
        item.update(device='/dev/v4l/by-path/'+path+'-video-index0', expected_id_path=path,
                    vid='1234', pid='5678', serial='EXPLICIT-CAMERA-'+str(item['port']))
    return value


def test_explicit_camera_paths_on_non_orin_and_actual_identity_mismatch():
    config = explicit_cameras(); cameras.validate_config(config)
    item = config['cameras'][0]
    actual = {'ID_VENDOR_ID':item['vid'], 'ID_MODEL_ID':item['pid'], 'ID_SERIAL_SHORT':item['serial'],
              'ID_PATH':item['expected_id_path'], 'ID_V4L_CAPABILITIES':':capture:'}
    verify_properties(item, actual)
    with pytest.raises(cameras.CameraError): verify_properties(item, {**actual, 'ID_PATH':'another-port'})
    assert cameras.camera_device(item).as_posix() == item['device']


@pytest.mark.parametrize('fault', ['half_pair', 'duplicate_path', 'duplicate_topology'])
def test_ambiguous_camera_bindings_rejected(fault):
    value = explicit_cameras()
    if fault == 'half_pair': del value['cameras'][0]['expected_id_path']
    elif fault == 'duplicate_path': value['cameras'][1]['device'] = value['cameras'][0]['device']
    else: value['cameras'][1]['expected_id_path'] = value['cameras'][0]['expected_id_path']
    with pytest.raises(cameras.CameraError): cameras.validate_config(value)


def test_legacy_camera_layout_keeps_its_recorded_meaning():
    value = fixture('cameras.json'); cameras.validate_config(value)
    for item in value['cameras']:
        assert cameras.camera_device(item) == cameras.port_path(item['port'])


def test_ultrasonic_binding_accepts_pc_topology_but_preserves_query_protocol():
    config = fixture('ultrasonic_capture.json')
    config.update(device='/dev/other_ultrasonic', expected_usb_path='pci-0000:00:14.0-usb-0:2:1.0')
    assert ultrasonic_capture.validate_config(config) == config
    with pytest.raises(ValueError): ultrasonic_capture.validate_config({**config,'function_code':6})


def bias(sensor_id):
    return dict(status='INDEPENDENTLY_CONFIRMED', sensor_id=sensor_id, bias_native_rad_s=[0,0,.0002],
                evidence_id='SYNTHETIC', evidence_sha256='a'*64, source='operator_visual_confirmation')


def test_bias_is_checked_against_input_identity_instead_of_current_machine():
    value = bias('H30-ARCHIVED-DEVICE')
    assert validate_confirmed_bias(value, expected_sensor_id='H30-ARCHIVED-DEVICE')['sensor_id'] == value['sensor_id']
    with pytest.raises(ValueError, match='mismatch'):
        validate_confirmed_bias(value, expected_sensor_id='H30-NEW-MACHINE')


def test_holdout_calibration_retains_original_sensor_and_rejects_mixed_rows(tmp_path):
    rows = [dict(sensor_id='H30-ARCHIVED', stamp_ns=1_000_000_000+i*20_000_000, sequence=i,
                 gyro_native=[0,0,.0002], acceleration_native=[0,0,9.80665],
                 wheel_velocity_m_s=0., wheel_age_s=0.) for i in range(1501)]
    path = tmp_path/'rows.jsonl'; evidence = tmp_path/'stationary.json'
    evidence.write_text(json.dumps(dict(physically_stationary=True, source='operator_visual_confirmation',
        evidence_id='SYNTHETIC', start_stamp_ns=1_000_000_000, end_stamp_ns=31_000_000_000)), encoding='utf-8')
    path.write_text('\n'.join(json.dumps(row) for row in rows), encoding='utf-8')
    output = tmp_path/'bias.json'; save_holdout_calibration(path, evidence, output)
    assert json.loads(output.read_text())['sensor_id'] == 'H30-ARCHIVED'
    rows[-1]['sensor_id'] = 'H30-OTHER'
    path.write_text('\n'.join(json.dumps(row) for row in rows), encoding='utf-8')
    with pytest.raises(ValueError, match='consistent input sensor_id'):
        save_holdout_calibration(path, evidence, tmp_path/'mixed.json')
    for row in rows: del row['sensor_id']
    path.write_text('\n'.join(json.dumps(row) for row in rows), encoding='utf-8')
    with pytest.raises(ValueError, match='explicit'):
        save_holdout_calibration(path, evidence, tmp_path/'unknown.json')


def wheel_record(identity, sequence=0):
    response = b'\x01\x03\x04\x00\x00\x00\x00'
    return dict(schema='wc_wheel_feedback_v1', device_id=identity, status='RESPONSE_VALID',
        request_hex=feedback_transport.QUERY.hex(), response_hex=(response+crc16(response)).hex(),
        stamp_ns=1_000_000_000+sequence*100_000_000, receive_monotonic_ns=1_000_000_000+sequence*100_000_000,
        sequence=sequence, stream_epoch='SYNTHETIC', time_valid=False, time_source='arrival_only')


def test_legacy_wheel_archive_binds_first_input_and_rejects_device_switch():
    conversion = fixture('wheel_history_calibration.json')
    decoder = HistoryPreview(conversion)
    decoder.update(wheel_record('ZLAC8030D-OLD'))
    with pytest.raises(FeedbackError, match='identity changed'):
        decoder.update(wheel_record('ZLAC8030D-OTHER',1))
    decoder = HistoryPreview(conversion, expected_device_id='ZLAC8030D-EXPECTED')
    with pytest.raises(FeedbackError, match='identity changed'):
        decoder.update(wheel_record('ZLAC8030D-OTHER'))


def test_manual_channel_rejects_evidence_for_another_open_controller():
    from wc_motion.manual_hardware import ManualRtuChannel
    with pytest.raises(FeedbackError, match='different controllers'):
        ManualRtuChannel(SimpleNamespace(config={'device_id':'ZLAC8030D-A'}),
                         SimpleNamespace(device_id='ZLAC8030D-B'), None)


def test_complete_imu_command_is_frozen_and_partial_conflict_rejected(monkeypatch):
    from wc_imu.ros_node import resolve_binding, ImuAcquisitionError
    from wc_runtime import device_bindings
    monkeypatch.setattr(device_bindings, 'load_device_bindings', lambda: binding('LIVE'))
    explicit = SimpleNamespace(**binding('ARCHIVE')['imu'])
    assert resolve_binding(explicit)['sensor_id'] == 'H30-ARCHIVE'
    incomplete = SimpleNamespace(device=None, expected_by_id=None, hardware_serial=None, sensor_id='H30-OTHER')
    with pytest.raises(ImuAcquisitionError): resolve_binding(incomplete)
    defaults = SimpleNamespace(device=None, expected_by_id=None, hardware_serial=None, sensor_id=None)
    assert resolve_binding(defaults)['sensor_id'] == 'H30-LIVE'
