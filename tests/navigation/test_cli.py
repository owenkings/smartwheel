import math
import pytest
from wc_navigation.cli import arguments


BASE = ['--project-root', '/tmp/project', '--map', '/tmp/map.yaml']


def test_footprint_assumption_must_be_explicit():
    with pytest.raises(SystemExit):
        arguments(BASE + ['--demo'])


@pytest.mark.parametrize('radius', ['0', '-1', 'nan', 'inf'])
def test_invalid_footprint_is_rejected(radius):
    with pytest.raises(SystemExit):
        arguments(BASE + ['--assumed-radius', radius, '--demo'])


def test_virtual_endpoints_cannot_be_confused_with_demo():
    with pytest.raises(SystemExit):
        arguments(BASE + ['--assumed-radius', '.55', '--demo', '--start', '0', '0'])
    with pytest.raises(SystemExit):
        arguments(BASE + ['--assumed-radius', '.55', '--start', '0', '0'])


def test_nonfinite_endpoint_rejected():
    with pytest.raises(SystemExit):
        arguments(BASE + ['--assumed-radius', '.55', '--start', 'nan', '0', '--goal', '2', '2'])


def test_valid_explicit_endpoints():
    args = arguments(BASE + ['--assumed-radius', '.55', '--start', '0', '0', '--goal', '2', '2'])
    assert args.start == [0, 0] and args.goal == [2, 2]
    assert args.assumed_radius == .55 and args.margin == .1
