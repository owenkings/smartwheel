"""Reporting incomplete directories must not widen deletion authorization."""
import os
from pathlib import Path
from unittest.mock import patch

import pytest

from wc_panel.storage import DataStore


class Guard:
    def check(self):return {'status':'AVAILABLE'}


def resolver(project,path,**kwargs):return Path(path),Guard()


def test_scan_includes_root_files_and_hidden_entries_but_only_deletes_normal_dirs(tmp_path):
    root=tmp_path/'data';root.mkdir()
    (root/'loose.bin').write_bytes(b'abc')
    (root/'.hidden_file').write_bytes(b'abcd')
    (root/'.hidden_dir').mkdir();(root/'.hidden_dir/item').write_bytes(b'12345')
    (root/'normal').mkdir();(root/'normal/item').write_bytes(b'123456')
    store=DataStore(tmp_path,resolver=resolver)
    with patch.object(store,'active',return_value=False):scan=store.scan(root)
    assert scan['total_complete'] and scan['bytes']==18 and scan['known_bytes']==18
    assert scan['unknown_items']==0
    by_name={row['name']:row for row in scan['items']}
    assert set(by_name)=={'loose.bin','.hidden_file','.hidden_dir','normal'}
    assert [name for name,row in by_name.items() if row['deletable']]==['normal']
    for name in ('loose.bin','.hidden_file','.hidden_dir'):
        with pytest.raises(ValueError):store.prepare_delete(root,[root/name])


def test_unreadable_subtree_keeps_other_items_and_partial_accounting(tmp_path):
    root=tmp_path/'data';root.mkdir()
    (root/'good').mkdir();(root/'good/file').write_bytes(b'12345')
    bad=root/'bad';bad.mkdir();(bad/'visible').write_bytes(b'abc')
    locked=bad/'locked';locked.mkdir();(locked/'unread').write_bytes(b'not counted')
    original=Path.iterdir
    def list_directory(path):
        if path==locked:raise PermissionError('fixture permission denied')
        return original(path)
    store=DataStore(tmp_path,resolver=resolver)
    with patch.object(Path,'iterdir',list_directory),patch.object(store,'active',return_value=False):
        scan=store.scan(root)
    assert scan['bytes'] is None and scan['allocated_bytes'] is None
    assert scan['known_bytes']==8 and not scan['total_complete'] and scan['unknown_items']==1
    by_name={row['name']:row for row in scan['items']}
    assert by_name['good']['bytes']==5 and by_name['good']['deletable']
    assert by_name['bad']['bytes'] is None and by_name['bad']['known_bytes']==3
    assert not by_name['bad']['deletable']
    assert by_name['bad']['problems'][0]['path']==str(locked)


def test_nested_bind_mount_is_reported_and_cannot_be_deleted(tmp_path):
    root=tmp_path/'data';root.mkdir()
    mounted=root/'mounted';mounted.mkdir();(mounted/'foreign').write_bytes(b'do not traverse')
    normal=root/'normal';normal.mkdir();(normal/'file').write_bytes(b'abc')
    nested=normal/'nested';nested.mkdir()
    store=DataStore(tmp_path,resolver=resolver)
    with patch('wc_panel.storage._mount_points',return_value={mounted,nested}), \
            patch.object(store,'active',return_value=False):
        scan=store.scan(root)
        assert not scan['total_complete'] and scan['known_bytes']==3
        by_name={row['name']:row for row in scan['items']}
        assert by_name['mounted']['entry_type']=='mount'
        assert all(not row['deletable'] for row in scan['items'])
        with pytest.raises(ValueError,match='挂载'):store.prepare_delete(root,[mounted])
        with pytest.raises(ValueError,match='挂载'):store.prepare_delete(root,[normal])


def test_unknown_activity_blocks_selection_without_losing_byte_totals(tmp_path):
    root=tmp_path/'data';root.mkdir()
    (root/'normal').mkdir();(root/'normal/file').write_bytes(b'abc')
    store=DataStore(tmp_path,resolver=resolver)
    with patch.object(store,'active',side_effect=PermissionError('fixture process query denied')):
        scan=store.scan(root)
    assert scan['total_complete'] and scan['bytes']==3
    [item]=scan['items']
    assert item['active'] is None and not item['deletable']
    assert item['status']=='UNAVAILABLE' and '任务占用' in item['error']


@pytest.mark.skipif(os.name=='nt',reason='POSIX symlink fixture')
def test_symlink_reported_without_reading_target(tmp_path):
    root=tmp_path/'data';root.mkdir()
    outside=tmp_path/'outside';outside.mkdir();(outside/'private').write_bytes(b'not counted')
    link=root/'link';link.symlink_to(outside,target_is_directory=True)
    (root/'visible').write_bytes(b'abc')
    store=DataStore(tmp_path,resolver=resolver)
    with patch.object(store,'active',return_value=False):scan=store.scan(root)
    assert not scan['total_complete'] and scan['known_bytes']==3
    linked=next(row for row in scan['items'] if row['name']=='link')
    assert linked['entry_type']=='symlink' and not linked['deletable']
    with pytest.raises(ValueError):store.prepare_delete(root,[link])
