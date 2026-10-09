"""Map save to selected directories with the real configured storage policy."""
import json

import pytest

from wc_runtime import mapping_save
from wc_panel import storage as panel_storage


@pytest.fixture
def session(tmp_path):
    root=tmp_path/'project';(root/'config').mkdir(parents=True)
    archive=tmp_path/'archive';archive.mkdir()
    (root/'config/storage.json').write_text(json.dumps(dict(schema_version=2,backend='directory',
        archive_root=str(archive),fallback_allowed=False)))
    directory=archive/'reports/maps/SYNTHETIC';directory.mkdir(parents=True)
    runtime=root/'.phase1_runtime/sessions/SYNTHETIC/mapping_app';runtime.mkdir(parents=True)
    handle=dict(session_id='SYNTHETIC',directory=str(directory),runtime=str(runtime),
        mode='all',mapping_enabled=True,retention_profile='experiment')
    (directory/'session.json').write_text(json.dumps(dict(session_id='SYNTHETIC',
        output_dir=str(directory),mapping_enabled=True,data_retention='TEMPORARY_UNTIL_USER_SAVE_CHOICE')))
    (directory/'runtime_config.json').write_text('{"mapping_enabled":true}')
    for name in ('slam','bag','export'):(directory/name).mkdir()
    (directory/'slam/rtabmap.db').write_bytes(b'SYNTHETIC audited database')
    (directory/'bag/source.db3').write_bytes(b'SYNTHETIC original data')
    (directory/'export/map.ply').write_bytes(b'SYNTHETIC audited cloud')
    return root,directory,handle,tmp_path/'用户选择'/'地图 01'


def save(root,handle,destination):
    return mapping_save.save_session(root,handle,destination,
        inspect=lambda h:dict(state='STOPPED',exit_code=0),
        export=lambda h:dict(status='EXPORTED_EXPERIMENTAL_MAP'))


def test_selected_external_path_preserves_original_and_configured_default(session):
    root,directory,handle,destination=session
    config=(root/'config/storage.json').read_bytes()
    result=save(root,handle,destination)
    assert result['status']=='SAVED_EXPERIMENTAL_MAP'
    assert (destination/'export/map.ply').read_bytes()==(directory/'export/map.ply').read_bytes()
    assert (directory/'bag/source.db3').is_file()
    assert (root/'config/storage.json').read_bytes()==config


def test_destination_lost_during_save_keeps_source_and_never_commits(session,monkeypatch):
    root,directory,handle,destination=session
    original=panel_storage.resolve_user_destination
    class Guard:
        def __init__(self):self.count=0
        def check(self):
            self.count+=1
            if self.count>=3:raise RuntimeError('SYNTHETIC destination detached')
    guard=Guard()
    def selected(*args,**kwargs):
        path,_=original(*args,**kwargs)
        return path,guard
    monkeypatch.setattr(panel_storage,'resolve_user_destination',selected)
    with pytest.raises(RuntimeError,match='detached'):save(root,handle,destination)
    assert not destination.exists()
    assert (directory/'bag/source.db3').read_bytes()==b'SYNTHETIC original data'
    assert (directory/'slam/rtabmap.db').is_file()
