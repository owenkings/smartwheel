"""Live filter overrides survive CLI, configuration and repeated map starts."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from test_mapping_controller import setup, tmp_path
from wc_runtime import mapping_app, mapping_controller
from wc_runtime.mapping_interactive import InteractiveCoordinator


@pytest.mark.parametrize('enabled', [False, True])
def test_cli_filter_override_preserves_thresholds_and_source_file(setup, enabled):
    path = Path(setup['config_path'])
    config = json.loads(path.read_text())
    config['cloud_filter'] = dict(enabled=not enabled, range=dict(min_m=.7, max_m=8.))
    path.write_text(json.dumps(config))
    original = path.read_bytes()
    request = mapping_app.parse_request(['right', '--mapping', 'true', '--cloud-filter',
                                        'on' if enabled else 'off'], project_root=setup['project_root'])
    request['config_path'] = path
    resolved = mapping_controller.configuration(request)
    assert resolved['cloud_filter']['enabled'] is enabled
    assert resolved['cloud_filter']['range'] == config['cloud_filter']['range']
    assert path.read_bytes() == original
    assert mapping_controller.check_configuration(request)['cloud_filter_effective'] is enabled


@pytest.mark.parametrize('enabled', [False, True])
def test_filter_choice_survives_preview_and_multiple_new_maps(tmp_path, enabled):
    request = mapping_app.parse_request(['all', '--mapping', 'true', '--interactive',
        '--cloud-filter', 'on' if enabled else 'off'], project_root=tmp_path)
    original = copy.deepcopy(request)
    runner = InteractiveCoordinator(request, None, SimpleNamespace(owner={'window_id':'filter_fixture'}))
    try:
        preview = runner.session_request(False)
        first = runner.session_request(True)
        second = runner.session_request(True)
        assert preview['mapping_enabled'] is False
        assert first['cloud_filter_enabled'] is second['cloud_filter_enabled'] is enabled
        assert first['session_id'] != second['session_id']
        assert request == original
    finally:
        runner.executor.shutdown()


@pytest.mark.parametrize('invalid', [1, 'on', None])
def test_runtime_rejects_invalid_filter_switch(setup, invalid):
    with pytest.raises(ValueError, match='点云筛选'):
        mapping_controller.configuration(dict(setup, cloud_filter_enabled=invalid))
