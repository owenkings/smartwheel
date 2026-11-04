import pytest
from wc_navigation.replay_cli import arguments, select_rows, perturbed_initial_pose


BASE = ['--project-root', '/tmp/project', '--source-cell', '/tmp/cell']


@pytest.mark.parametrize('extra', [
    ['--training-window', '70', '120'],
    ['--training-window', '-1', '60'],
    ['--query-window', '105', '70'],
    ['--query-interval', '1.1'],
    ['--initial-offset', '1', '0', '0'],
    ['--max-gap', 'nan'],
])
def test_invalid_replay_split_or_capture_range_is_rejected(extra):
    with pytest.raises(SystemExit):
        arguments(BASE + extra)


def test_temporal_sampling_preserves_original_stamps():
    rows = [{'stamp_ns': n * 100000000} for n in range(30)]
    chosen = select_rows(rows, 0, [1., 2.], .45)
    assert [r['stamp_ns'] for r in chosen] == [1000000000, 1500000000, 2000000000]


def test_initial_perturbation_does_not_rotate_distant_map_translation():
    import math
    import numpy as np
    from wc_runtime.offline_planar_registration import se2
    original = se2(18., -3., 1.)
    changed = perturbed_initial_pose(original, [.15, .10, 3.])
    np.testing.assert_allclose(changed[:2, 3] - original[:2, 3], [.15, .10])
    assert math.atan2(changed[1, 0], changed[0, 0]) == pytest.approx(1. + math.radians(3))
