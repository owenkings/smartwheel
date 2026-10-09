"""Cloud selection reaches real subscription, preview and immutable session config."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from wc_runtime import mapping_app as app, mapping_controller as controller, mapping_input as mapping_input
from wc_runtime.mapping_cloud import selected_cloud_source, source_frame_topic, preview_cloud_topic
from test_mapping_controller import setup, tmp_path
from test_mapping_input import config as input_config, source, finish_bootstrap, deliver, START


@pytest.mark.parametrize('choice', ['raw', 'filtered'])
@pytest.mark.parametrize('mode', ['left', 'right', 'all'])
@pytest.mark.parametrize('mapping_enabled', [True, False])
def test_cli_selection_controls_config_preview_and_recording(setup, choice, mode, mapping_enabled):
    request = app.parse_request([mode, '--cloud', choice, '--mapping', str(mapping_enabled).lower()],
                                project_root=setup['project_root'])
    config = controller.configuration(request)
    assert request['cloud_source'] == config['cloud_source'] == choice
    report = controller.check_configuration(request)
    assert report['cloud_source'] == choice
    assert report['hardware_started'] is False
    sides = ('left', 'right') if mode == 'all' else (mode,)
    assert report['source_topics'] == {side: source_frame_topic(side, choice) for side in sides}
    planned = controller.plan(request, config)
    if mapping_enabled:
        for side in sides:
            assert source_frame_topic(side, 'raw') in planned['bag_topics']
            assert source_frame_topic(side, 'filtered') in planned['bag_topics']
        assert any('wc_runtime.mapping_input' in argv for argv in planned['commands'])
    else:
        assert planned['bag_topics'] == []
        assert not any('wc_runtime.mapping_input' in argv for argv in planned['commands'])
    view = yaml.safe_load(controller.render_view(config))
    displays = view['Visualization Manager']['Displays']
    if not mapping_enabled:
        enabled = {d['Topic']['Value'] for d in displays if d.get('Class') == 'rviz_default_plugins/PointCloud2' and d['Enabled']}
        assert enabled == {preview_cloud_topic(side, choice) for side in sides}
    else:
        scan = next(d for d in displays if d.get('Topic', {}).get('Value') == '/wc_mapping/app/scan_cloud')
        assert ('主机滤波前' if choice == 'raw' else '主机滤波后') in scan['Name']
    if choice == 'raw':
        assert '不是原始光学深度' in report['cloud_description']
    # Resolving once freezes the selection even if the editable file changes.
    directory = Path(request['output_dir']); directory.mkdir(parents=True)
    frozen = controller.archive_configuration(directory, config)
    edited = json.loads(Path(request['config_path']).read_text())
    edited['cloud_source'] = 'filtered' if choice == 'raw' else 'raw'
    Path(request['config_path']).write_text(json.dumps(edited))
    assert json.loads((directory/'runtime_config.json').read_text())['cloud_source'] == choice
    assert selected_cloud_source(frozen) == choice


def test_legacy_default_and_explicit_cli_precedence(setup):
    path = Path(setup['config_path'])
    config = json.loads(path.read_text()); config.pop('cloud_source', None)
    path.write_text(json.dumps(config))
    assert controller.configuration(setup)['cloud_source'] == 'filtered'
    config['cloud_source'] = 'raw'; path.write_text(json.dumps(config))
    assert controller.configuration(setup)['cloud_source'] == 'raw'
    assert controller.configuration({**setup, 'cloud_source': 'filtered'})['cloud_source'] == 'filtered'
    request = app.parse_request(['right'], project_root=setup['project_root'])
    assert request['mapping_enabled'] is False and 'cloud_source' not in request
    assert controller.configuration(request)['cloud_source'] == 'raw'


@pytest.mark.parametrize('bad', ['auto', 'RAW', '', None, False, 1, []])
def test_unknown_source_fails_before_session_creation(setup, bad):
    with pytest.raises(ValueError, match='cloud_source'):
        controller.configuration({**setup, 'cloud_source': bad})
    config = input_config(); config['cloud_source'] = bad
    with pytest.raises(ValueError, match='cloud_source'):
        mapping_input.validate_config(config)
    assert not Path(setup['output_dir']).exists()


def test_cli_invalid_choice_rejected_without_output(setup):
    with pytest.raises(SystemExit):
        app.parse_request(['right', '--cloud', 'auto'], project_root=setup['project_root'])
    assert not Path(setup['output_dir']).exists()


@pytest.mark.parametrize('choice', ['raw', 'filtered'])
def test_actual_subscription_routes_each_side_and_no_other_representation(choice):
    captured, delivered = [], []
    def subscribe(cls, topic, callback, qos):
        captured.append((cls, topic, callback, qos))
        return topic
    node = SimpleNamespace(create_subscription=subscribe)
    config = {'mode': 'all', 'cloud_source': choice}
    subscriptions = mapping_input.subscribe_source_frames(node, config, 'SourceFrame',
        lambda msg, side: delivered.append((msg, side)), lambda _, callback: callback, 'SOURCE_QOS')
    assert subscriptions == [source_frame_topic(side, choice) for side in ('left', 'right')]
    for index, row in enumerate(captured):
        assert row[0] == 'SourceFrame' and row[3] == 'SOURCE_QOS'
        row[2](index)
    assert delivered == [(0, 'left'), (1, 'right')]


@pytest.mark.parametrize('choice', ['raw', 'filtered'])
def test_selected_source_preserves_single_frame_bytes_and_provenance(tmp_path, choice):
    config = input_config('right'); config['cloud_source'] = choice
    session = tmp_path/'session'; session.mkdir()
    owner = mapping_input.MappingInput(config, tmp_path, session/'input', started_ns=START)
    try:
        now = finish_bootstrap(owner)
        msg = source('right', sequence=20, host=now+100_000_000)
        representation = 'before_host_filter' if choice == 'raw' else 'host_filtered'
        msg.diagnostic_flags = ['representation='+representation]
        original = copy.deepcopy(msg.cloud)
        published = []
        assert deliver(owner, msg, published)
        assert len(published) == 1 and bytes(published[0].data) == bytes(original.data)
        assert published[0].header.frame_id == original.header.frame_id
        status = owner.status(msg.host_monotonic_ns)
        assert status['cloud_source'] == choice
        assert status['sources']['right']['input_topic'] == source_frame_topic('right', choice)
        assert ('selected '+choice+' SDK') in status['sources']['right']['archive_format']
        # Wrong-branch messages must not silently satisfy an explicit selection.
        wrong = source('right', sequence=21, host=now+400_000_000)
        wrong.diagnostic_flags = ['representation='+('host_filtered' if choice == 'raw' else 'before_host_filter')]
        with pytest.raises(mapping_input.InputFailure, match='CLOUD_SOURCE_REPRESENTATION_MISMATCH'):
            deliver(owner, wrong, published)
        assert len(published) == 1
    finally:
        owner.close()
