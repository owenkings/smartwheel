"""Pixel identity is independent of presentation flip, zoom and projection."""
import copy
import json

import pytest

from wc_calibration.core import CalibrationError
from wc_calibration.picker import PickerSession


def data():
    scene = {'id': 'pixel-scene', 'left': [[1, 2, 3], [None, 0, 0], [0, 0, 0], [4, 5, 6]],
        'right': [[7, 8, 9], [None, 0, 0], [0, 0, 0], [10, 11, 12]],
        'organized': {side: {'width': 2, 'height': 2, 'order': 'row_major',
            'amplitude': [10, 20, None, 40]} for side in ('left', 'right')},
        'selected_raw_frames': {side: {'units': 'm', 'coordinate_convention': 'FLU',
            'sensor_id': side} for side in ('left', 'right')}}
    return {'schema_version': 1, 'status': 'PREPARED_NOT_VALIDATED', 'source_mode': 'synthetic',
        'sensor_ids': {'left': 'left', 'right': 'right'}, 'training': [scene]}


def make(tmp_path, value):
    p = tmp_path/'prepared.json'
    p.write_text(json.dumps(value), encoding='utf-8')
    return PickerSession(p, tmp_path/'exports')


def test_original_pixel_indices_and_immutable_amp(tmp_path):
    original = data()
    session = make(tmp_path, original)
    scene = session.scene()
    assert [row['id'] for row in scene['clouds']['left']] == [0, 3]
    assert scene['organized']['left']['amplitude'] == [10, 20, None, 40]
    # Bottom-left display pixel under horizontal flip corresponds to raw row1 col1 id3.
    body = {'input_hash': session.input_hash, 'scene_id': scene['scene_id'],
        'pairs': [{'left_id': 3, 'right_id': 0}]}
    selected = session.selection(body)
    assert selected['left'] == [[4., 5., 6.]] and selected['right'] == [[7., 8., 9.]]
    scene['organized']['left']['amplitude'][0] = 999
    assert session.scene()['organized']['left']['amplitude'][0] == 10
    for invalid in (1, 2):
        body['pairs'][0]['left_id'] = invalid
        with pytest.raises(CalibrationError, match='nonfinite point index'):
            session.selection(body)


@pytest.mark.parametrize('fault', ['shape', 'amplitude_count', 'boolean', 'negative', 'order', 'budget', 'missing_side'])
def test_layout_rejects_ambiguous_correspondence(tmp_path, fault):
    value = data(); layout = value['training'][0]['organized']
    if fault == 'shape': layout['left']['width'] = 3
    elif fault == 'amplitude_count': layout['left']['amplitude'].pop()
    elif fault == 'boolean': layout['left']['amplitude'][0] = True
    elif fault == 'negative': layout['left']['amplitude'][0] = -1
    elif fault == 'order': layout['left']['order'] = 'column_major'
    elif fault == 'budget': layout['left']['width'] = 20001
    elif fault == 'missing_side': del layout['right']
    with pytest.raises(CalibrationError): make(tmp_path, value)


def test_no_organized_keeps_legacy_zero_origin_and_no_amplitude(tmp_path):
    value = data(); del value['training'][0]['organized']
    session = make(tmp_path, value)
    assert 'organized' not in session.scene()
    assert [row['id'] for row in session.scene()['clouds']['left']] == [0, 2, 3]
