import json
import hashlib

import pytest

from wc_runtime.mapping_bias_confirm import proposal, save_confirmation


def session(tmp_path, passed=True):
    p=tmp_path/'session'; (p/'prior').mkdir(parents=True)
    (p/'runtime_config.json').write_text(json.dumps({'source_mode':'real','imu_sensor_id':'H30-0000000015','session_id':'unit_static'}))
    candidate={'screening_passed':passed,'start_stamp_ns':1_000_000_000,'end_stamp_ns':6_000_000_000,
               'sample_count':500,'bias_native_rad_s':[0.,0.,.0003]}
    (p/'prior/status.json').write_text(json.dumps({'gyro_bias':{'latest_candidate':candidate}}))
    return p


def test_screened_window_alone_does_not_write_or_apply_bias(tmp_path):
    s=session(tmp_path); output=tmp_path/'bias.json'
    result=save_confirmation(s,output)
    assert result['status']=='CONFIRMATION_REQUIRED'
    assert not output.exists()
    assert not output.with_name('bias.evidence.json').exists()


def test_confirmation_writes_bound_evidence_and_preserves_session(tmp_path):
    s=session(tmp_path); original=(s/'prior/status.json').read_bytes()
    output=tmp_path/'bias.json'
    token=save_confirmation(s,output)['candidate_token']
    result=save_confirmation(s,output,confirmed=True,candidate_token=token)
    record=json.loads(output.read_text())
    evidence=output.with_name('bias.evidence.json').read_bytes()
    assert record['evidence_sha256']==hashlib.sha256(evidence).hexdigest()
    assert record['source']=='operator_visual_confirmation'
    assert json.loads(evidence)['physical_stationarity_confirmed_by_operator'] is True
    assert result['applied_to_running_session'] is False
    assert result['holdout_accuracy_validated'] is False
    assert (s/'prior/status.json').read_bytes()==original
    with pytest.raises(ValueError,match='already exists'):
        save_confirmation(s,output,confirmed=True,candidate_token=token)


def test_user_confirmation_cannot_override_failed_screening(tmp_path):
    s=session(tmp_path,passed=False)
    with pytest.raises(ValueError,match='没有完整合格零偏候选'):
        save_confirmation(s,tmp_path/'bad.json',confirmed=True)


def test_confirmation_cannot_silently_apply_a_new_live_window(tmp_path):
    s=session(tmp_path); output=tmp_path/'bias.json'
    token=save_confirmation(s,output)['candidate_token']
    p=s/'prior/status.json'; value=json.loads(p.read_text())
    value['gyro_bias']['latest_candidate']['start_stamp_ns']+=5_000_000_000
    value['gyro_bias']['latest_candidate']['end_stamp_ns']+=5_000_000_000
    p.write_text(json.dumps(value))
    with pytest.raises(ValueError,match='候选已变化'):
        save_confirmation(s,output,confirmed=True,candidate_token=token)
    assert not output.exists()
