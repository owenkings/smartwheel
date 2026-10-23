"""Explicit experimental selection survives the interactive window's new maps."""
import copy
from types import SimpleNamespace

import pytest

from test_mapping_controller import tmp_path
from wc_runtime.mapping_app import parse_request
from wc_runtime.mapping_interactive import InteractiveCoordinator
from test_mapping_app import Backend, execute, request as lifecycle_request


@pytest.mark.parametrize('scheme', ['validated', 'experiment'])
def test_explicit_scheme_survives_repeated_maps(tmp_path, scheme):
    request = parse_request(['all', '--mapping', 'true', '--interactive', '--scheme', scheme,
        '--motion-correction', 'on', '--geometry', 'on', '--estimator', 'five_state'],
        project_root=tmp_path)
    original = copy.deepcopy(request)
    runner = InteractiveCoordinator(request, None, SimpleNamespace(owner={'window_id':'scheme_fixture'}))
    try:
        preview = runner.session_request(False)
        first = runner.session_request(True)
        second = runner.session_request(True)
        assert {r['live_scheme'] for r in (preview, first, second)} == {scheme}
        assert preview['motion_correction'] is preview['geometry'] is False
        assert first['motion_correction'] is second['geometry'] is True
        assert first['session_id'] != second['session_id']
        assert request == original
    finally:
        runner.executor.shutdown()


def test_default_does_not_opt_in_to_experiment(tmp_path):
    request = parse_request(['left', '--mapping', 'true'], project_root=tmp_path)
    assert request.get('live_scheme', 'validated') == 'validated'


def test_invalid_scheme_is_rejected(tmp_path):
    with pytest.raises(SystemExit):
        parse_request(['left', '--scheme', 'offline'], project_root=tmp_path)


@pytest.mark.parametrize('applied', [False, True])
def test_start_notice_distinguishes_fixed_candidate_from_confirmed_bias(tmp_path, applied):
    class ExperimentalBackend(Backend):
        def start(self, value):
            return {**super().start(value), 'live_scheme':'experiment',
                'experimental_motion_correction_applied':applied, 'confirmed_gyro_bias_applied':False}
    notices=[]
    execute(lifecycle_request(tmp_path), ExperimentalBackend(), emit=notices.append)
    notice=next(n for n in notices if n['status']=='MAPPING_STARTED_NOT_YET_VERIFIED')['message']
    assert ('本次显式应用未标定的离线运动候选' in notice) is applied
    assert '已加载用户确认的陀螺零偏文件' not in notice
