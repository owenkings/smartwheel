"""Candidate freeze verifies real SQLite selected bytes and preserves source bytes."""
import json
import sys
import hashlib
from pathlib import Path
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'runtime'))
import test_offline_refinement_inputs as fixtures
from wc_runtime.mapping_compare import freeze_motion_candidate


@pytest.fixture
def case():
    case=fixtures.EvidenceTests();case.setUp()
    p=case.candidate['provenance']
    case.runtime={'session_id':case.source.name,'imu_sensor_id':p['imu_device_id'],
        'wheel_device_id':p['wheel_device_id'],'wheel_candidate':p['wheel_conversion'],
        'imu_mount':{'R_axle_imu':p['R_reference_imu']}}
    case.output=case.root/'output';case.output.mkdir()
    try:yield case
    finally:case.tearDown()


def test_freeze_preserves_exact_candidate_bytes_and_original_bag(case):
    case.path.write_text(json.dumps(case.candidate,indent=3)+'\n\n')
    before=case.path.read_bytes();bag=case.bag.read_bytes();hashes={}
    factory,proof=freeze_motion_candidate(case.path,case.output,case.source,hashes,case.runtime,None)
    assert callable(factory)
    assert (case.output/'motion_candidate.json').read_bytes()==before==case.path.read_bytes()
    assert case.bag.read_bytes()==bag
    assert proof['candidate_sha256']==hashes[str(case.path)]==hashlib.sha256(before).hexdigest()
    assert proof['process_noise']=='UNCHANGED_PER_ESTIMATOR' and proof['geometry_feedback'] is False
    assert json.loads((case.output/'candidate_verification.json').read_text())['status']=='VERIFIED'


def test_freeze_rejects_tampered_origin_evidence(case):
    case.evidence.write_text('changed evidence')
    with pytest.raises(ValueError,match='source SHA-256 mismatch'):
        freeze_motion_candidate(case.path,case.output,case.source,{},case.runtime,None)
    assert not (case.output/'candidate_verification.json').exists()


@pytest.mark.parametrize('bias_scope',['runtime','prior_template'])
def test_freeze_rejects_preexisting_bias_before_any_snapshot(case,bias_scope):
    target=case.runtime if bias_scope=='runtime' else case.runtime.setdefault('prior_template',{})
    target['confirmed_gyro_bias']={}
    with pytest.raises(ValueError,match='subtract bias twice'):
        freeze_motion_candidate(case.path,case.output,case.source,{},case.runtime,None)
    assert not (case.output/'motion_candidate.json').exists()
