"""Development records remain external, uniquely named and destination guarded."""
from datetime import datetime, timezone
import json
from pathlib import Path

import pytest

from wc_runtime.development_archive import CATEGORIES, create_task


def project(tmp_path):
    root = tmp_path / 'project'
    data = tmp_path / 'data'
    (root / 'config').mkdir(parents=True)
    data.mkdir()
    (root / 'config/storage.json').write_text(json.dumps({
        'schema_version': 2, 'backend': 'directory', 'archive_root': str(data),
        'fallback_allowed': False}), encoding='utf-8')
    return root, data


def test_task_records_use_configured_data_root_and_do_not_overwrite(tmp_path):
    root, data = project(tmp_path)
    now = datetime(2026, 10, 9, 0, 0, tzinfo=timezone.utc)
    first = create_task(root, 'fixture_review', now=now)
    marker = first / 'reports/preserved.txt'
    marker.write_text('original', encoding='utf-8')
    second = create_task(root, 'fixture_review', now=now)
    assert first != second and first.parent == second.parent == data / 'dev_archive'
    assert marker.read_text(encoding='utf-8') == 'original'
    assert not (root / 'dev_archive').exists()
    assert all((first / category / 'README.md').is_file() for category in CATEGORIES)
    rows = [json.loads(line) for line in (data / 'dev_archive/index.jsonl').read_text().splitlines()]
    assert [row['task'] for row in rows] == [first.name, second.name]


@pytest.mark.parametrize('name', ['../escape', '/absolute', 'a/b', '', '.', 'a' * 65])
def test_task_name_cannot_escape_or_replace_archive(tmp_path, name):
    root, data = project(tmp_path)
    with pytest.raises(ValueError):
        create_task(root, name)
    assert not (data / 'dev_archive').exists()


def test_invalid_local_storage_cannot_fall_back_to_base(tmp_path):
    root, data = project(tmp_path)
    (root / 'config/storage.local.json').write_text('{}', encoding='utf-8')
    with pytest.raises(ValueError):
        create_task(root, 'fixture_review')
    assert not (data / 'dev_archive').exists()
    assert not (root / 'dev_archive').exists()


def test_linked_archive_cannot_write_outside_data_root(tmp_path):
    root, data = project(tmp_path)
    other = tmp_path / 'other'
    other.mkdir()
    (data / 'dev_archive').symlink_to(other, target_is_directory=True)
    with pytest.raises(ValueError):
        create_task(root, 'fixture_review')
    assert not list(other.iterdir())


def test_bound_policy_cannot_be_reused_for_another_project(tmp_path):
    from wc_runtime.storage_policy import StoragePolicy
    root, data = project(tmp_path)
    other = tmp_path / 'other_project'
    other.mkdir()
    policy = StoragePolicy(root)
    with pytest.raises(ValueError, match='policy belongs to a different project'):
        create_task(other, 'fixture_review', storage_policy=policy)
    assert not (data / 'dev_archive').exists()
