"""Device declaration provenance, distinct from physical calibration accuracy."""
import copy
import hashlib
import json

import pytest

from wc_runtime.live_calibration_files import load_live_calibration_files


@pytest.fixture
def declaration(tmp_path):
    folder=tmp_path/'config';folder.mkdir()
    evidence=b'{"scope":"SYNTHETIC DEVICE DECLARATION"}\n'
    bias=dict(status='INDEPENDENTLY_CONFIRMED',sensor_id='H30-0000000015',
        bias_native_rad_s=[0.,0.,.001],source='operator_visual_confirmation',
        evidence_id='synthetic-bias',evidence_sha256='b'*64)
    value=dict(schema_version=1,status='EXPLICIT_DEVICE_CALIBRATION_DECLARATION',scope='device',
        imu_sensor_id=bias['sensor_id'],wheel_device_id='SYNTHETIC-WHEEL',
        R_reference_imu=[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]],
        wheel_yaw_scale=1.03,wheel_yaw_speed_coefficient=.01,confirmed_gyro_bias=bias,
        evidence_id='synthetic-device',evidence_sha256=hashlib.sha256(evidence).hexdigest())
    path=folder/'motion.json';path.write_text(json.dumps(value))
    path.with_name('motion.evidence.json').write_bytes(evidence)
    config=dict(motion_correction=True,live_motion_calibration_config='config/motion.json',
        imu_sensor_id=bias['sensor_id'],wheel_device_id='SYNTHETIC-WHEEL',
        confirmed_gyro_bias=copy.deepcopy(bias),gyro_bias_provenance={'sha256':'a'*64})
    return tmp_path,path,config,evidence


def test_live_declaration_freezes_exact_bytes_and_evidence(declaration):
    root,path,config,evidence=declaration
    value=load_live_calibration_files(root,config)
    assert value['_live_motion_raw_text'].encode()==path.read_bytes()
    assert value['_live_motion_evidence_raw_text'].encode()==evidence
    assert value['live_motion_provenance']['sha256']==hashlib.sha256(path.read_bytes()).hexdigest()
    assert value['live_motion_provenance']['physical_accuracy_validated'] is False


@pytest.mark.parametrize('damage',['evidence','identity','bias','missing_bias_proof','inline'])
def test_inconsistent_calibration_never_reaches_live_prior(declaration,damage):
    root,path,config,evidence=declaration
    if damage=='evidence':path.with_name('motion.evidence.json').write_bytes(b'{}')
    elif damage=='identity':config['imu_sensor_id']='H30-0000000099'
    elif damage=='bias':config['confirmed_gyro_bias']['bias_native_rad_s'][0]=.1
    elif damage=='missing_bias_proof':config['gyro_bias_provenance']=None
    else:config['live_motion_calibration']=json.loads(path.read_text())
    with pytest.raises(ValueError):load_live_calibration_files(root,config)
