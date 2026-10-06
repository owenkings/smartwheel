"""ROS-free authority message/TF contracts; no hardware or accuracy claims."""
import copy
import math
from types import SimpleNamespace as NS

import pytest

from wc_runtime.mapping_odometry import (adapt_prior_odometry,
    authority_identity_transform, validate_covariance)


def weights():
    return {'status': 'UNVALIDATED_MODEL_WEIGHTS',
            'pose_diagonal': [.25, .25, 1., .09, .09, .09],
            'twist_diagonal': [.04, .25, .25, .01, .01, .04]}


def original():
    covariance = [0.0]*36
    for index in (0, 7, 14, 21, 28, 35):
        covariance[index] = 1e6
    return NS(header=NS(stamp=NS(sec=1789370012, nanosec=345678901), frame_id='prior_odom'),
        child_frame_id='mapping_reference',
        pose=NS(pose=NS(position=NS(x=1.2, y=-.4, z=.05),
                       orientation=NS(x=0., y=0., z=math.sin(.3), w=math.cos(.3))),
                covariance=covariance.copy()),
        twist=NS(twist=NS(linear=NS(x=.1, y=.02, z=0.), angular=NS(x=.01, y=-.02, z=.2)),
                 covariance=covariance.copy()))


def transform_message():
    return NS(header=NS(stamp=NS(sec=0, nanosec=0), frame_id=''), child_frame_id='',
              transform=NS(translation=NS(x=9., y=9., z=9.), rotation=NS(x=9., y=9., z=9., w=9.)))


def test_adapter_preserves_original_stamp_pose_and_body_twist_without_mutation():
    source, model = original(), weights()
    before, model_before = copy.deepcopy(source), copy.deepcopy(model)
    output = adapt_prior_odometry(source, model)
    assert source == before and model == model_before
    assert output is not source and type(output) is type(source)
    assert output.header.frame_id == 'mapping_odom'
    assert output.header.stamp == source.header.stamp
    assert output.child_frame_id == source.child_frame_id == 'mapping_reference'
    assert output.pose.pose == source.pose.pose and output.twist.twist == source.twist.twist
    for field, key in ((output.pose, 'pose_diagonal'), (output.twist, 'twist_diagonal')):
        assert list(field.covariance[::7]) == model[key]
        assert all(v == 0. for i, v in enumerate(field.covariance) if i % 7)
    # Mutating the adapted message cannot change historical input or its stamp.
    output.header.stamp.nanosec = 1
    output.pose.pose.position.x = 500.
    output.twist.covariance[0] = .5
    assert source == before


def test_identity_bridge_has_single_parent_chain_and_same_original_stamp():
    source = original()
    before = copy.deepcopy(source)
    bridge = authority_identity_transform(source, transform_type=transform_message)
    output = adapt_prior_odometry(source, weights())
    assert bridge.header.frame_id == output.header.frame_id == 'mapping_odom'
    assert bridge.child_frame_id == source.header.frame_id == 'prior_odom'
    assert source.child_frame_id == output.child_frame_id == 'mapping_reference'
    assert bridge.header.stamp == output.header.stamp == source.header.stamp
    assert vars(bridge.transform.translation) == {'x': 0., 'y': 0., 'z': 0.}
    assert vars(bridge.transform.rotation) == {'x': 0., 'y': 0., 'z': 0., 'w': 1.}
    # The identity bridge composes to the exact recorded pose, never a second
    # direct TF parent for mapping_reference or a guessed sensor mounting.
    assert output.pose.pose == source.pose.pose and source == before


@pytest.mark.parametrize('value', [None, {}, {'status': 'CALIBRATED'},
    {**weights(), 'formal_accuracy': True}, {**weights(), 'status': 'VALIDATED'},
    {**weights(), 'pose_diagonal': [0.1]*5}, {**weights(), 'twist_diagonal': 'unknown'}])
def test_weight_schema_is_explicit_and_strict(value):
    with pytest.raises(ValueError):
        validate_covariance(value)


@pytest.mark.parametrize('value', [True, 0, -1, float('nan'), float('inf'), 9999, 1e6, 10**400, '0.1'])
@pytest.mark.parametrize('key', ['pose_diagonal', 'twist_diagonal'])
def test_nonpositive_unknown_and_reset_sentinel_weights_are_rejected(value, key):
    model = weights(); model[key][3] = value
    with pytest.raises(ValueError):
        adapt_prior_odometry(original(), model)


@pytest.mark.parametrize('key', ['pose_diagonal', 'twist_diagonal'])
def test_rtabmap_ignored_x_weight_is_rejected_but_non_x_one_is_valid(key):
    assert validate_covariance(weights())['pose_diagonal'][2] == 1.
    model = weights(); model[key][0] = 1.
    with pytest.raises(ValueError, match='ignored value'):
        validate_covariance(model)


def test_normalization_is_independent_and_does_not_invent_defaults():
    model = weights(); model['pose_diagonal'] = tuple(model['pose_diagonal'])
    result = validate_covariance(model)
    assert type(result['pose_diagonal']) is list
    result['twist_diagonal'][0] = 3.
    assert model['twist_diagonal'][0] == .04
    with pytest.raises(ValueError):
        validate_covariance(None)


@pytest.mark.parametrize('path,value', [
    ('header.frame_id', 'mapping_odom'), ('child_frame_id', 'lidar_right'),
    ('header.stamp.sec', -1), ('header.stamp.sec', 2**31), ('header.stamp.sec', True),
    ('header.stamp.nanosec', 1e9), ('header.stamp.nanosec', 1_000_000_000),
    ('pose.pose.position.x', float('nan')), ('twist.twist.angular.z', float('inf')),
    ('pose.pose.orientation.w', 2.), ('pose.covariance', [0.]*35),
    ('twist.covariance', [float('nan')]*36)])
def test_invalid_identity_time_or_geometry_never_becomes_authority(path, value):
    source = original()
    target = source
    names = path.split('.')
    for name in names[:-1]:
        target = getattr(target, name)
    setattr(target, names[-1], value)
    for adapter in (lambda: adapt_prior_odometry(source, weights()),
                    lambda: authority_identity_transform(source, transform_type=transform_message)):
        with pytest.raises(ValueError):
            adapter()


def test_zero_timestamp_and_quaternion_are_not_repaired():
    source = original(); source.header.stamp = NS(sec=0, nanosec=0)
    with pytest.raises(ValueError, match='timestamp'):
        adapt_prior_odometry(source, weights())
    source = original(); source.pose.pose.orientation = NS(x=0., y=0., z=0., w=0.)
    with pytest.raises(ValueError, match='unit length'):
        adapt_prior_odometry(source, weights())


def test_malformed_message_raises_value_error():
    with pytest.raises(ValueError, match='malformed'):
        adapt_prior_odometry(NS(), weights())
