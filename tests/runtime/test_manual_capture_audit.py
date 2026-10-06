"""Synthetic source-control accounting; never opens a device."""
import json
import pytest
from wc_runtime.capture_audit import audit_wheel_control
from wc_runtime.source_archive import atomic_json,digest
from wc_runtime.mapping_wheel import RELEASE_REQUEST

def fixture(root):
    path=root/'configuration/manual_runtime.json';path.parent.mkdir()
    atomic_json(path,{'session_id':'synthetic','manual_controls':{'arm_allowed':True,'interaction_policy':'hybrid_manual'},
        'manual_authorization':{'source':'EXPLICIT_CAPTURE_MANUAL_DRIVE_ARGUMENT'}})
    atomic_json(root/'capture_manifest.json',{'session_id':'synthetic','manual_drive':True,
        'input_hashes':{'configuration/manual_runtime.json':digest(path)}})
    event={'event':'wheel_io_complete','request_hex':RELEASE_REQUEST.hex(),'response_hex':RELEASE_REQUEST.hex(),
           'transmitted_bytes':len(RELEASE_REQUEST)}
    summary={'interaction_policy':'hybrid_manual','control_transmissions':1}
    return [event],summary

def test_authorized_manual_transactions_are_counted(tmp_path):
    events,summary=fixture(tmp_path)
    assert audit_wheel_control(tmp_path,events,summary)['control_transmissions']==1

def test_readonly_capture_cannot_hide_control_writes(tmp_path):
    events,summary=fixture(tmp_path)
    atomic_json(tmp_path/'capture_manifest.json',{'manual_drive':False})
    with pytest.raises(ValueError,match='read-only'):
        audit_wheel_control(tmp_path,events,summary)

def test_missing_tail_write_or_tampered_authorization_is_rejected(tmp_path):
    events,summary=fixture(tmp_path)
    with pytest.raises(ValueError,match='transmissions'):
        audit_wheel_control(tmp_path,[],summary)
    path=tmp_path/'configuration/manual_runtime.json'
    value=json.loads(path.read_text());value['manual_controls']['arm_allowed']=False
    atomic_json(path,value)
    with pytest.raises(ValueError,match='hash mismatch'):
        audit_wheel_control(tmp_path,events,summary)

def test_release_ack_must_match_exact_reviewed_request(tmp_path):
    events,summary=fixture(tmp_path)
    events[0]['response_hex']='01030400000000fa33'
    with pytest.raises(ValueError):
        audit_wheel_control(tmp_path,events,summary)
