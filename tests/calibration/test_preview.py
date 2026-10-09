import copy
import json

import numpy as np
import pytest

from wc_calibration.core import CalibrationError, _hash
from wc_calibration.preview import preview_payload, render_html, render_png


def fixture():
    points = np.arange(240, dtype=float).reshape(-1, 3)/100
    data = {'schema_version':1,'source_mode':'synthetic','units':'m',
            'sensor_ids':{'left':'SYN-L','right':'SYN-R'},'validation':[],
            'training':[{'id':'SYN-view','left':points.tolist(),'right':points.tolist()}]}
    t = np.eye(4);t[1, 3] = -.5
    result = {'kind':'single_scene_exploration','status':'EXPLORATION_ONLY','live_eligible':False,
              'input_hash':_hash(data),'candidates':[{'id':'SYN-candidate','candidate_T_left_right':t.tolist(),
                'score_m':.03,'geometry_usable':False,'rejection_reasons':['DEGENERATE'],'diagnostics':{}}]}
    return data, result


def test_source_binding_prevents_showing_transform_for_other_cloud():
    data, result = fixture();data['training'][0]['left'][0][0] += .1
    with pytest.raises(CalibrationError, match='exact input'):
        preview_payload(data, result)


def test_verified_or_live_result_cannot_be_displayed_as_exploration():
    data, result = fixture();result['live_eligible'] = True
    with pytest.raises(CalibrationError, match='unvalidated'):
        preview_payload(data, result)


def test_rejects_invalid_rotation():
    data, result = fixture();result['candidates'][0]['candidate_T_left_right'][0][0] = 2
    with pytest.raises(CalibrationError):
        preview_payload(data, result)


def test_decimation_preserves_source_and_candidate_direction():
    data, result = fixture();original = copy.deepcopy(data)
    payload = preview_payload(data, result, 20)
    assert len(payload['left']) == 20 and payload['original_counts']['left'] == 80
    assert payload['candidates'][0]['matrix'][1][3] == -.5
    assert data == original


def test_html_embeds_data_without_script_injection_or_network_dependencies():
    data, result = fixture();data['training'][0]['id'] = '</script><script>alert(1)</script>'
    result['input_hash'] = _hash(data)
    html = render_html(preview_payload(data, result))
    assert '</script><script>alert' not in html and 'https://' not in html
    assert '\\u003c/script\\u003e' in html


def test_static_png_is_created_from_synthetic_payload(tmp_path):
    data, result = fixture();path = tmp_path/'synthetic.png'
    render_png(preview_payload(data, result), path)
    assert path.read_bytes().startswith(b'\x89PNG\r\n\x1a\n')
