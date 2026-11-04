"""Conservative offline localization evidence ledger; never an acceptance gate.

The reference/prior may share odometry with the map. Disagreement is a conflict
to investigate, not error against physical truth. Matching scan/map endpoints
and small particle covariance cannot certify localization. This module does not
publish, select controller poses, update maps, or reseed localization.
"""
from __future__ import annotations

import math
from numbers import Integral, Real


POLICY = {
    'minimum_observed_endpoints': 20,
    'minimum_reference_known_fraction': .80,
    'minimum_reference_supported_fraction': .70,
    'prior_translation_review_m': .35,
    'prior_yaw_review_rad': math.radians(8),
    'small_particle_xy_std_m': .10,
    'small_particle_yaw_std_rad': math.radians(2),
    'maximum_age_ns': 400_000_000,
    'meaning': 'FIXED_REVIEW_TRIGGERS_NOT_CALIBRATED_ACCURACY_LIMITS',
    'prior_review_source': 'EXISTING_PLANAR_POLICY_CUMULATIVE_REVIEW_ONLY',
}


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(label + ' must be finite numeric')
    return float(value)


def _stamp(value, label):
    if isinstance(value, bool) or not isinstance(value, Integral) or not 0 <= value < 2**63:
        raise ValueError(label + ' must be an integer nanosecond stamp')
    return int(value)


def _pose(value):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError('pose must contain x, y, yaw')
    result = [_number(v, 'pose') for v in value]
    if max(abs(result[0]), abs(result[1])) > 1_000_000:
        raise ValueError('pose exceeds metric resource bound')
    return result


def _coverage(value):
    if not isinstance(value, dict):
        raise ValueError('coverage packet must be a dictionary')
    n = value.get('observed_endpoints')
    if isinstance(n, bool) or not isinstance(n, Integral) or not 0 <= n <= 4096:
        raise ValueError('bounded observed endpoint count required')
    fractions = {}
    for key in ('known_fraction', 'supported_fraction'):
        v = value.get(key)
        if n == 0 and v is None:
            fractions[key] = None
        else:
            v = _number(v, key)
            if not 0 <= v <= 1:
                raise ValueError('coverage fractions must be in [0, 1]')
            fractions[key] = v
    if n and fractions['supported_fraction'] > fractions['known_fraction'] + 1e-12:
        raise ValueError('supported endpoints cannot exceed known endpoints')
    if not isinstance(value.get('along_heading_ambiguity_evidence'), bool):
        raise ValueError('explicit ambiguity boolean required')
    return {'observed_endpoints': int(n), **fractions,
            'along_heading_ambiguity_evidence': value['along_heading_ambiguity_evidence']}


def coverage_category(value):
    """Category fixed from the declared reference stream, never the fitted pose."""
    v = _coverage(value)
    if (v['observed_endpoints'] < POLICY['minimum_observed_endpoints']
            or v['known_fraction'] is None
            or v['known_fraction'] < POLICY['minimum_reference_known_fraction']
            or v['supported_fraction'] < POLICY['minimum_reference_supported_fraction']):
        return 'REFERENCE_COVERAGE_INSUFFICIENT'
    if v['along_heading_ambiguity_evidence']:
        return 'REFERENCE_ALONG_HEADING_AMBIGUITY'
    return 'NO_TRIGGER_IN_TESTED_REFERENCE_HYPOTHESES_UNVALIDATED'


def evaluate_localization_evidence(stamp_ns, now_ns, *, estimate, reference,
                                   reference_coverage, estimated_coverage=None,
                                   acquisition_time_verified=False,
                                   coordinate_reference_verified=False):
    """Evaluate one expected capture without nearest/future matching.

    Packets contain ``stamp_ns``, ``pose_x_y_yaw`` (the same declared metric
    point/frame for estimate and reference), and optional ``received_time_ns``.
    Optional ``particle_std_x_y_yaw`` belongs inside the estimate packet and is
    consumed only with that exact-stamp, available estimate. Missing receipt
    metadata remains unknown. A legacy offline report may still
    yield a geometric diagnostic, but cannot demonstrate causal availability.
    All output remains untrusted, including the no-trigger case.
    """
    stamp_ns, now_ns = _stamp(stamp_ns, 'capture'), _stamp(now_ns, 'now')
    if not isinstance(acquisition_time_verified, bool) or not isinstance(coordinate_reference_verified, bool):
        raise ValueError('verification fields require booleans')
    flags, blockers = [], ['PHYSICAL_LOCALIZATION_ACCEPTANCE_NOT_ESTABLISHED']
    if not acquisition_time_verified:
        blockers.append('ACQUISITION_TIME_UNVERIFIED')
    if not coordinate_reference_verified:
        blockers.append('COORDINATE_REFERENCE_UNVERIFIED')
    if stamp_ns > now_ns:
        flags.append('FUTURE_EXPECTED_CAPTURE')
    elif now_ns - stamp_ns > POLICY['maximum_age_ns']:
        flags.append('STALE_EXPECTED_CAPTURE')
    poses, timing = {}, {}
    for name, packet in [('estimate', estimate), ('reference', reference),
                         ('reference_coverage', reference_coverage),
                         ('estimated_coverage', estimated_coverage)]:
        if packet is None:
            if name != 'estimated_coverage':
                flags.append(name.upper() + '_MISSING')
            timing[name] = {'aligned': False, 'available_at_query': False}
            continue
        if not isinstance(packet, dict):
            raise ValueError(name + ' packet must be dictionary')
        p_stamp = _stamp(packet.get('stamp_ns'), name + ' capture')
        aligned = p_stamp == stamp_ns
        if not aligned:
            flags.append(name.upper() + '_STAMP_MISMATCH')
        receipt = packet.get('received_time_ns')
        if receipt is None:
            blockers.append(name.upper() + '_RECEIPT_TIME_UNKNOWN')
            available = False
        else:
            receipt = _stamp(receipt, name + ' receipt')
            available = receipt >= p_stamp and receipt <= now_ns and p_stamp <= now_ns
            if not available:
                flags.append(name.upper() + '_NONCAUSAL_OR_UNAVAILABLE')
        timing[name] = {'aligned': aligned, 'available_at_query': available,
                        'received_time_ns': receipt}
        # Exact-stamp legacy geometric evidence remains inspectable; future
        # captures are never consumed, even for geometric classification.
        if (aligned and p_stamp <= now_ns and (receipt is None or available)
                and name in ('estimate', 'reference')):
            poses[name] = _pose(packet.get('pose_x_y_yaw'))
    ref_coverage = None
    if (timing['reference_coverage']['aligned'] and stamp_ns <= now_ns
            and (timing['reference_coverage']['received_time_ns'] is None
                 or timing['reference_coverage']['available_at_query'])):
        ref_coverage = _coverage(reference_coverage)
    category = coverage_category(ref_coverage) if ref_coverage is not None else 'REFERENCE_COVERAGE_UNKNOWN'
    if category != 'NO_TRIGGER_IN_TESTED_REFERENCE_HYPOTHESES_UNVALIDATED':
        flags.append(category)
    translation, yaw = None, None
    if 'estimate' in poses and 'reference' in poses:
        a, b = poses['estimate'], poses['reference']
        translation = math.hypot(a[0] - b[0], a[1] - b[1])
        yaw = abs(math.atan2(math.sin(a[2] - b[2]), math.cos(a[2] - b[2])))
        if translation > POLICY['prior_translation_review_m']:
            flags.append('ESTIMATE_REFERENCE_TRANSLATION_CONFLICT')
        if yaw > POLICY['prior_yaw_review_rad']:
            flags.append('ESTIMATE_REFERENCE_YAW_CONFLICT')
    std = None
    particle_std_x_y_yaw = (estimate.get('particle_std_x_y_yaw')
                           if 'estimate' in poses else None)
    if particle_std_x_y_yaw is not None:
        std = _pose(particle_std_x_y_yaw)
        if min(std) < 0:
            raise ValueError('particle standard deviations must be nonnegative')
        if (translation is not None and
                (translation > POLICY['prior_translation_review_m'] or yaw > POLICY['prior_yaw_review_rad'])
                and max(std[:2]) <= POLICY['small_particle_xy_std_m']
                and std[2] <= POLICY['small_particle_yaw_std_rad']):
            flags.append('SMALL_PARTICLE_COVARIANCE_WITH_REFERENCE_CONFLICT')
    fit = None
    if (estimated_coverage is not None and timing['estimated_coverage']['aligned'] and stamp_ns <= now_ns
            and (timing['estimated_coverage']['received_time_ns'] is None
                 or timing['estimated_coverage']['available_at_query'])):
        fit = _coverage(estimated_coverage)
        if (translation is not None and translation > POLICY['prior_translation_review_m']
                and fit['supported_fraction'] is not None and fit['supported_fraction'] >= .90):
            flags.append('GOOD_ENDPOINT_FIT_WITH_REFERENCE_CONFLICT')
    return {'schema_version': 1, 'stamp_ns': stamp_ns, 'now_ns': now_ns,
            'trust_status': 'NOT_TRUSTED',
            'diagnostic_status': 'REVIEW_REQUIRED' if flags else 'NO_TRIGGER_IN_TESTED_EVIDENCE_UNVALIDATED',
            'diagnostic_flags': flags, 'acceptance_blockers': blockers,
            'predeclared_reference_category': category,
            'translation_disagreement_m': translation, 'yaw_disagreement_rad': yaw,
            'particle_std_x_y_yaw': std, 'reference_coverage': ref_coverage,
            'estimated_pose_coverage': fit, 'packet_timing': timing,
            'policy': dict(POLICY), 'physical_navigation_validated': False,
            'controller_pose_selected': False,
            'interpretation': 'CONFLICT_AND_COVERAGE_EVIDENCE_NOT_ERROR_AGAINST_PHYSICAL_TRUTH'}
