import copy
import math

import pytest

from wc_navigation.localization_trust import coverage_category, evaluate_localization_evidence


STAMP = 1_000_000_000


def coverage(**changes):
    return {'stamp_ns': STAMP, 'received_time_ns': STAMP,
            'observed_endpoints': 100, 'known_fraction': .95,
            'supported_fraction': .90, 'along_heading_ambiguity_evidence': False,
            **changes}


def run(**changes):
    values = dict(stamp_ns=STAMP, now_ns=STAMP,
                  estimate={'stamp_ns': STAMP, 'received_time_ns': STAMP,
                            'pose_x_y_yaw': [0., 0., 0.]},
                  reference={'stamp_ns': STAMP, 'received_time_ns': STAMP,
                             'pose_x_y_yaw': [0., 0., 0.]},
                  reference_coverage=coverage(), estimated_coverage=coverage())
    values.update(changes)
    return evaluate_localization_evidence(**values)


def test_no_trigger_and_verified_inputs_still_never_accept():
    result = run(acquisition_time_verified=True, coordinate_reference_verified=True)
    assert result['diagnostic_status'] == 'NO_TRIGGER_IN_TESTED_EVIDENCE_UNVALIDATED'
    assert result['trust_status'] == 'NOT_TRUSTED'
    assert not result['physical_navigation_validated']
    assert result['acceptance_blockers'] == ['PHYSICAL_LOCALIZATION_ACCEPTANCE_NOT_ESTABLISHED']


def test_legacy_receipts_missing_are_explicit_offline_only():
    packet = {'stamp_ns': STAMP, 'pose_x_y_yaw': [1., 0., 0.]}
    result = run(estimate=packet)
    assert result['translation_disagreement_m'] == 1.
    assert 'ESTIMATE_RECEIPT_TIME_UNKNOWN' in result['acceptance_blockers']
    assert not result['packet_timing']['estimate']['available_at_query']


@pytest.mark.parametrize('receipt', [STAMP - 1, STAMP + 1])
def test_explicit_unavailable_estimate_not_consumed_even_low_covariance(receipt):
    result = run(estimate={'stamp_ns': STAMP, 'received_time_ns': receipt,
                          'pose_x_y_yaw': [8., 0., 0.],
                          'particle_std_x_y_yaw': [.001, .001, .001]})
    assert result['translation_disagreement_m'] is None
    assert result['particle_std_x_y_yaw'] is None
    assert 'ESTIMATE_NONCAUSAL_OR_UNAVAILABLE' in result['diagnostic_flags']


@pytest.mark.parametrize('name', ['reference_coverage', 'estimated_coverage'])
def test_explicit_future_coverage_not_consumed(name):
    result = run(**{name: coverage(received_time_ns=STAMP + 1)})
    key = 'reference_coverage' if name == 'reference_coverage' else 'estimated_pose_coverage'
    assert result[key] is None


def test_future_capture_cannot_produce_diagnostic_pose_or_coverage():
    result = run(now_ns=STAMP - 1)
    assert result['translation_disagreement_m'] is None
    assert result['reference_coverage'] is None
    assert 'FUTURE_EXPECTED_CAPTURE' in result['diagnostic_flags']


def test_stale_capture_never_unflagged():
    result = run(now_ns=STAMP + 400_000_001)
    assert 'STALE_EXPECTED_CAPTURE' in result['diagnostic_flags']
    assert result['trust_status'] == 'NOT_TRUSTED'


def test_exact_stamp_matching_no_nearest_covariance():
    result = run(estimate={'stamp_ns': STAMP - 1, 'received_time_ns': STAMP,
                          'pose_x_y_yaw': [7., 0., 0.],
                          'particle_std_x_y_yaw': [.01, .01, .01]})
    assert result['translation_disagreement_m'] is None
    assert result['particle_std_x_y_yaw'] is None
    assert 'ESTIMATE_STAMP_MISMATCH' in result['diagnostic_flags']


def test_small_covariance_and_good_fit_cannot_hide_large_conflict():
    result = run(estimate={'stamp_ns': STAMP, 'received_time_ns': STAMP,
                          'pose_x_y_yaw': [4., 0., 0.],
                          'particle_std_x_y_yaw': [.02, .03, math.radians(.5)]},
                 estimated_coverage=coverage(known_fraction=.98, supported_fraction=.97))
    assert 'SMALL_PARTICLE_COVARIANCE_WITH_REFERENCE_CONFLICT' in result['diagnostic_flags']
    assert 'GOOD_ENDPOINT_FIT_WITH_REFERENCE_CONFLICT' in result['diagnostic_flags']
    assert result['trust_status'] == 'NOT_TRUSTED'


def test_reference_coverage_selection_cannot_follow_fitted_pose():
    result = run(reference_coverage=coverage(known_fraction=.2, supported_fraction=.1),
                 estimated_coverage=coverage(known_fraction=.99, supported_fraction=.99))
    assert result['predeclared_reference_category'] == 'REFERENCE_COVERAGE_INSUFFICIENT'
    assert 'REFERENCE_COVERAGE_INSUFFICIENT' in result['diagnostic_flags']


def test_ambiguity_remains_distinct_from_missing_coverage():
    assert coverage_category(coverage(along_heading_ambiguity_evidence=True)) == 'REFERENCE_ALONG_HEADING_AMBIGUITY'
    result = run(reference_coverage=coverage(along_heading_ambiguity_evidence=True))
    assert 'REFERENCE_ALONG_HEADING_AMBIGUITY' in result['diagnostic_flags']


def test_yaw_wrap_and_cumulative_existing_review_threshold():
    result = run(estimate={'stamp_ns': STAMP, 'received_time_ns': STAMP,
                          'pose_x_y_yaw': [.351, 0., math.pi - .01]},
                 reference={'stamp_ns': STAMP, 'received_time_ns': STAMP,
                            'pose_x_y_yaw': [0., 0., -math.pi + .01]})
    assert result['yaw_disagreement_rad'] == pytest.approx(.02)
    assert 'ESTIMATE_REFERENCE_YAW_CONFLICT' not in result['diagnostic_flags']
    assert 'ESTIMATE_REFERENCE_TRANSLATION_CONFLICT' in result['diagnostic_flags']


def test_missing_frame_retained_as_missing_diagnostic():
    result = run(estimate=None)
    assert result['stamp_ns'] == STAMP
    assert result['translation_disagreement_m'] is None
    assert 'ESTIMATE_MISSING' in result['diagnostic_flags']


@pytest.mark.parametrize('packet', [coverage(supported_fraction=1.1),
                                    coverage(known_fraction=.1, supported_fraction=.2),
                                    coverage(observed_endpoints=True),
                                    coverage(known_fraction=float('nan'))])
def test_invalid_coverage_rejected(packet):
    with pytest.raises(ValueError):
        run(reference_coverage=packet)


@pytest.mark.parametrize('value', [True, -1, 1.5, 2**63])
def test_invalid_stamps_rejected(value):
    with pytest.raises(ValueError):
        run(stamp_ns=value)


def test_no_mutation():
    packet = coverage()
    original = copy.deepcopy(packet)
    run(reference_coverage=packet)
    assert packet == original


def test_empty_scan_cannot_be_coverage():
    result = run(reference_coverage=coverage(observed_endpoints=0, known_fraction=None, supported_fraction=None))
    assert result['predeclared_reference_category'] == 'REFERENCE_COVERAGE_INSUFFICIENT'
