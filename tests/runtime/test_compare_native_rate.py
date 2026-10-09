"""Offline native rate applies to a copied instance and must match its readback."""
import copy
import json
import math

import pytest

from wc_runtime.compare_native import with_native_detection_rate, verify_native_detection_rate_readback


@pytest.mark.parametrize('previous', [None, '1', '0', '5'])
def test_only_copied_offline_spec_receives_requested_rate(previous):
    # A nested object shared with a frozen runtime/profile must not be mutated.
    parameters = {'use_sim_time': True, 'RGBD/NeighborLinkRefining': 'false',
                  'Grid/CellSize': '0.05'}
    if previous is not None:
        parameters['Rtabmap/DetectionRate'] = previous
    runtime = {'profile': {'parameters': parameters}, 'input_rate_hz': 5.}
    spec = {'nodes': [{'package': 'rtabmap_slam', 'parameters': [parameters],
                       'remappings': [('scan_cloud', '/frozen/scan')]}]}
    prior_spec, prior_runtime = copy.deepcopy(spec), copy.deepcopy(runtime)
    actual, provenance = with_native_detection_rate(spec, 5.)
    expected = copy.deepcopy(spec)
    expected['nodes'][0]['parameters'][0]['Rtabmap/DetectionRate'] = '5.0'
    assert actual == expected
    assert spec == prior_spec and runtime == prior_runtime
    assert actual['nodes'][0]['parameters'][0] is not parameters
    assert provenance['previous_was_explicit'] is (previous is not None)
    assert provenance['previous_explicit_value'] == previous
    assert provenance['requested_hz'] == 5.
    assert provenance['applied_value'] == '5.0'
    assert provenance['scope'] == 'NEW_OFFLINE_NATIVE_INSTANCE_ONLY'
    assert provenance['effective_hz'] is None


@pytest.mark.parametrize('requested', [0., -1., 10.001, math.inf, -math.inf, math.nan])
def test_invalid_request_does_not_modify_instance_spec(requested):
    spec = {'nodes': [{'parameters': [{'Rtabmap/DetectionRate': '1'}]}]}
    previous = copy.deepcopy(spec)
    with pytest.raises(RuntimeError, match='finite and in'):
        with_native_detection_rate(spec, requested)
    assert spec == previous


def make_report(requested=5.):
    _, provenance = with_native_detection_rate({'nodes': [{'parameters': [{}]}]}, requested)
    return {'native_detection_rate': provenance}


@pytest.mark.parametrize('effective', [5., 5.+1e-10])
def test_matching_readback_is_recorded_before_replay(effective):
    report = make_report()
    verify_native_detection_rate_readback(report, effective)
    assert report['native_detection_rate']['readback_matches_request'] is True
    assert report['effective_native_detection_rate_hz'] == effective
    assert report['native_detection_rate']['effective_hz'] == effective
    json.dumps(report, allow_nan=False)


@pytest.mark.parametrize('effective', [0., -1., 1., 10., math.inf, -math.inf, math.nan])
def test_wrong_readback_fails_with_retained_json_safe_evidence(effective):
    report = make_report()
    with pytest.raises(RuntimeError, match='readback differs'):
        verify_native_detection_rate_readback(report, effective)
    provenance = report['native_detection_rate']
    assert provenance['readback_matches_request'] is False
    assert provenance['readback_value'] == str(effective)
    assert provenance['requested_hz'] == 5.
    expected = effective if math.isfinite(effective) else None
    assert report['effective_native_detection_rate_hz'] == expected
    assert provenance['effective_hz'] == expected
    json.dumps(report, allow_nan=False)
