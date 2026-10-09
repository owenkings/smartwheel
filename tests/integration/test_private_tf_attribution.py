"""Synthetic native MessageInfo evidence fixtures, not actual DDS proof."""

import copy
import importlib.util
from pathlib import Path

import numpy as np
import pytest

spec = importlib.util.spec_from_file_location('wheel_guess_evidence_check', Path(__file__).with_name('check_wheel_guess.py'))
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


def fixture():
    fusion_gid, idle_icp_gid = '01' * 24, '02' * 24
    private, records = {}, []
    for index in range(60):
        stamp = 1_000_000_000 + index * 500_000_000
        pose = np.eye(4)
        pose[0, 3] = index * .001
        private[stamp] = pose.tolist()
        records.append({'stamp_ns': stamp, 'received_unix_ns': stamp + 1_000_000,
                        'publisher_gid': fusion_gid, 'transform_xyz_xyzw': [index * .001, 0., 0., 0., 0., 0., 1.]})
    evidence = {'schema_version': 1, 'test': 'native_tf_ownership_probe', 'status': 'PASS',
                'session_id': 'SYNTHETIC_FIXTURE', 'topic': '/wc_mapping/icp_guess_tf',
                'scope': 'Actual observed /wc_mapping/icp_guess_tf messages only; no claim about unobserved publishers or TF static topic.',
                'wheel_private_mode': True, 'interrupted': False, 'errors': [], 'rmw_gid_storage_size': 24,
                'edges': {'wheel_odom->rig_link': {'transform_count': 60, 'observations': records,
                                                 'publisher_gids': {fusion_gid: 60}}},
                'endpoint_history': [
                    {'publisher_gid': fusion_gid, 'node': '_NODE_NAMESPACE_UNKNOWN_/_NODE_NAME_UNKNOWN_'},
                    {'publisher_gid': fusion_gid, 'node': '/wc_dual_fusion'},
                    {'publisher_gid': idle_icp_gid, 'node': '/wc_icp_odometry'}]}
    return evidence, private


def check(evidence, private):
    return checker.check_private_tf_attribution(evidence, private, session_id='SYNTHETIC_FIXTURE', expected_count=60)


def test_idle_native_endpoint_is_not_an_actual_tf_sender():
    evidence, private = fixture()
    result = check(evidence, private)
    assert result['status'] == 'PASS'
    assert len(result['comparisons']) == 60
    assert result['actual_senders'] == [{'publisher_gid': '01' * 24, 'actual_transform_count': 60,
                                        'resolved_nodes': ['/wc_dual_fusion'], 'status': 'PASS'}]
    assert result['discovered_endpoints_without_observed_transforms'] == [
        {'publisher_gid': '02' * 24, 'node': '/wc_icp_odometry'}]


def test_second_real_sender_is_still_failure():
    evidence, private = fixture()
    edge = evidence['edges']['wheel_odom->rig_link']
    edge['observations'][0]['publisher_gid'] = '02' * 24
    edge['publisher_gids'] = {'01' * 24: 59, '02' * 24: 1}
    result = check(evidence, private)
    assert result['status'] == 'FAIL'
    assert 'NATIVE_PRIVATE_TF_SENDER_NOT_UNIQUE_OR_COUNT_MISMATCH' in result['errors']
    assert 'NATIVE_PRIVATE_TF_ACTUAL_SENDER_NOT_FUSION' in result['errors']


@pytest.mark.parametrize('field,value', [
    ('topic', '/tf'), ('wheel_private_mode', False), ('session_id', 'OTHER_SESSION'),
    ('status', 'BLOCKED'), ('interrupted', True), ('scope', 'Actual observed /tf messages only'),
    ('rmw_gid_storage_size', 16),
])
def test_wrong_probe_scope_and_session_fail(field, value):
    evidence, private = fixture()
    evidence[field] = value
    assert check(evidence, private)['status'] == 'FAIL'


def test_missing_or_duplicated_native_sample_is_not_complete():
    evidence, private = fixture()
    evidence['edges']['wheel_odom->rig_link']['observations'].pop()
    assert check(evidence, private)['status'] == 'FAIL'
    evidence, private = fixture()
    evidence['edges']['wheel_odom->rig_link']['observations'][-1] = copy.deepcopy(evidence['edges']['wheel_odom->rig_link']['observations'][0])
    result = check(evidence, private)
    assert result['status'] == 'FAIL'
    assert 'NATIVE_PRIVATE_TF_DUPLICATE_OR_INVALID_STAMP' in result['errors']


def test_native_transform_must_equal_python_observation():
    evidence, private = fixture()
    evidence['edges']['wheel_odom->rig_link']['observations'][20]['transform_xyz_xyzw'][0] += .0001
    result = check(evidence, private)
    assert result['status'] == 'FAIL'
    assert 'NATIVE_PRIVATE_TF_VALUE_MISMATCH' in result['errors']


def test_only_zero_missing_or_unresolved_gid_never_passes():
    evidence, private = fixture()
    for value in ('', '00' * 24, None):
        altered = copy.deepcopy(evidence)
        altered['edges']['wheel_odom->rig_link']['observations'][0]['publisher_gid'] = value
        assert check(altered, private)['status'] == 'FAIL'
    evidence['endpoint_history'] = [{'publisher_gid': '01' * 24, 'node': '_NODE_NAMESPACE_UNKNOWN_/_NODE_NAME_UNKNOWN_'}]
    assert check(evidence, private)['status'] == 'FAIL'


def test_discovery_alone_without_actual_records_fails():
    evidence, private = fixture()
    evidence['edges']['wheel_odom->rig_link']['observations'] = []
    assert check(evidence, private)['status'] == 'FAIL'
    assert check(None, private)['status'] == 'FAIL'
