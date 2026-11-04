import json
from pathlib import Path

import pytest

from wc_navigation.dashboard import read_snapshot, render_html, reports_destination


def snapshot():
    return {'snapshot': {'mode': 'OFFLINE_SNAPSHOT', 'physical_navigation_validated': False,
                         'live_sensor_feed': False},
            **{key: {} for key in ['summary', 'mapping', 'near', 'localization', 'onsite']}}


def test_report_label_cannot_close_json_script_or_inject_markup():
    value = snapshot()
    value['summary']['label'] = '</script><script>malicious()</script>&\u2028'
    template = '<script id="dashboard-data" type="application/json">__WC_DATA__</script>'
    output = render_html(template, value)
    assert output.count('</script>') == 1
    encoded = output.split('>', 1)[1].rsplit('</script>', 1)[0]
    assert json.loads(encoded) == value
    assert 'malicious()' in encoded and '<' not in encoded


def test_missing_template_data_slot_rejected():
    with pytest.raises(ValueError, match='exactly one'):
        render_html('<main>No slot</main>', snapshot())


@pytest.mark.parametrize('edit', [{'snapshot': False}, {'snapshot': {'mode': 'LIVE'}},
                                {'near': None}])
def test_snapshot_cannot_imply_live_or_accepted_state(tmp_path, edit):
    value = snapshot();value.update(edit)
    p = tmp_path/'snapshot.json';p.write_text(json.dumps(value))
    with pytest.raises(ValueError):
        read_snapshot(p)


def test_nonfinite_input_is_rejected_before_render(tmp_path):
    p = tmp_path/'snapshot.json';p.write_text('{"value":NaN}')
    with pytest.raises(ValueError, match='nonfinite'):
        read_snapshot(p)


def test_unicode_snapshot_preserves_values_without_mutation(tmp_path):
    value = snapshot();value['summary']['label'] = '轮椅定位'
    p = tmp_path/'snapshot.json';p.write_text(json.dumps(value), encoding='utf-8')
    result, raw = read_snapshot(p)
    assert result == value and raw == p.read_bytes()


class Policy:
    def __init__(self, root):self.root = root
    def resolve(self, value):return self.root/value
    def logical(self, path):return path.relative_to(self.root).as_posix()


@pytest.mark.parametrize('value', ['src/result.html', 'reports/result.json'])
def test_destination_cannot_write_source_or_wrong_artifact(tmp_path, value):
    with pytest.raises(ValueError, match='configured reports'):
        reports_destination(Policy(tmp_path), value)


def test_existing_snapshot_is_not_overwritten(tmp_path):
    p = tmp_path/'reports/result.html';p.parent.mkdir();p.write_text('old')
    with pytest.raises(ValueError, match='overwrite'):
        reports_destination(Policy(tmp_path), 'reports/result.html')
    assert p.read_text() == 'old'
