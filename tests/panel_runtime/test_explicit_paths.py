"""Real POSIX filesystem paths; no device or recorded data modification."""
import json
from pathlib import Path

import pytest

from wc_runtime.user_data_paths import UserDataPaths


@pytest.fixture
def roots(tmp_path):
    project=tmp_path/'project'; (project/'config').mkdir(parents=True)
    archive=tmp_path/'configured'; archive.mkdir()
    (project/'config/storage.json').write_text(json.dumps(dict(schema_version=2,
        backend='directory', archive_root=str(archive), fallback_allowed=False)))
    data=tmp_path/'recorded data'; data.mkdir()
    return project,archive,data,tmp_path/'selected results'/'new run'


def test_explicit_roots_keep_configured_default_unchanged(roots):
    root,archive,data,output=roots
    before=(root/'config/storage.json').read_bytes()
    selected=UserDataPaths(root,data,output)
    assert selected.resolve(data/'source.json')==data/'source.json'
    assert selected.resolve(output/'cloud.ply')==output/'cloud.ply'
    assert selected.resolve('reports/old')==archive/'reports/old'
    assert (root/'config/storage.json').read_bytes()==before
    assert not output.exists()


def test_project_configuration_auxiliary_is_readable_without_becoming_output_root(roots):
    root,archive,data,output=roots
    config=root/'config/hardware_setup.json';config.write_text('{}')
    selected=UserDataPaths(root,data,output,['config/hardware_setup.json'])
    assert selected.resolve(config)==config
    with pytest.raises(ValueError,match='源代码'):
        UserDataPaths(root,data,root/'config'/'bad-output')


def test_project_relative_alias_is_resolved_against_project_not_working_directory(roots,monkeypatch,tmp_path):
    root,archive,data,output=roots
    source=archive/'data/experiments/run';source.mkdir(parents=True)
    monkeypatch.chdir(tmp_path)
    selected=UserDataPaths(root,'data/experiments/run',output)
    assert selected.source==source
    assert selected.resolve('data/experiments/run')==source


@pytest.mark.parametrize('kind',['equal','child','parent','linked'])
def test_input_overlap_and_symlink_are_rejected(roots,kind):
    root,archive,data,output=roots
    if kind=='equal':output=data
    elif kind=='child':output=data/'nested'
    elif kind=='parent':output=data.parent
    else:
        link=data.parent/'linked';link.symlink_to(data,target_is_directory=True)
        data=link
    with pytest.raises(ValueError):UserDataPaths(root,data,output)
