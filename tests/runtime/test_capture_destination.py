"""External archives must never silently resolve to the Orin root disk."""
from pathlib import Path
import stat
from types import SimpleNamespace

import pytest

from wc_runtime import capture, capture_destination as destination


@pytest.fixture
def mounted(tmp_path, monkeypatch):
    mount = tmp_path/'usb'
    mount.mkdir()
    output = mount/'wheelchair/experiments'
    source = tmp_path/'sda1'
    uuid_root = tmp_path/'by-uuid'
    required = uuid_root/'ABCD-1234'
    row = dict(mount_id='42',device='8:1',root='/',mount_point=str(mount),
        options=['rw','nosuid'],super_options=['rw'],filesystem='fuseblk',source=str(source))
    state = dict(rows=[row],source_device=2049,required_device=2049,root_device=1793)
    original_stat = Path.stat
    def metadata(path, *args, **kwargs):
        if path in (source, required):
            return SimpleNamespace(st_mode=stat.S_IFBLK,
                st_rdev=state['source_device' if path==source else 'required_device'])
        result = original_stat(path,*args,**kwargs)
        device = state['root_device'] if path==Path('/') else 2049
        fields = {name: getattr(result, name) for name in dir(result) if name.startswith('st_')}
        fields['st_dev'] = device
        return SimpleNamespace(**fields)
    monkeypatch.setattr(destination,'UUID_ROOT',uuid_root)
    monkeypatch.setattr(destination,'_mounts',lambda:state['rows'])
    monkeypatch.setattr(Path,'stat',metadata)
    monkeypatch.setattr(destination.os,'major',lambda dev:dev//256,raising=False)
    monkeypatch.setattr(destination.os,'minor',lambda dev:dev%256,raising=False)
    return output,state,row


def test_local_default_does_not_require_external_media(tmp_path):
    root,guard=destination.capture_destination(tmp_path)
    assert root==tmp_path/'data/experiments' and guard is None


@pytest.mark.parametrize('path,uuid', [('relative','ABCD-1234'),('/media/../tmp','ABCD-1234'),
    ('/media/usb','../other'),('/media/usb','')])
def test_external_destination_rejects_ambiguous_arguments(path,uuid):
    with pytest.raises(ValueError): destination.CaptureDestination(path,uuid)


def test_uuid_override_requires_an_explicit_output_root(tmp_path):
    with pytest.raises(ValueError,match='supplied together'):
        destination.capture_destination(tmp_path,required_uuid='ABCD-1234')


def test_selected_directory_without_uuid_must_already_exist(tmp_path):
    with pytest.raises(ValueError,match='目录不存在'):
        destination.capture_destination(tmp_path,output_root=tmp_path/'missing')
    assert not (tmp_path/'missing').exists()


def test_missing_future_archive_is_verified_without_creating_it(mounted):
    output,_,row=mounted
    guard=destination.CaptureDestination(output,'ABCD-1234')
    assert not output.exists()
    assert guard.mount_root==Path(row['mount_point'])
    assert guard.check()['required_uuid']=='ABCD-1234'
    assert guard.metadata['fallback_allowed'] is False


@pytest.mark.parametrize('fault', ['missing','wrong_uuid','read_only','super_read_only','root_disk','remounted'])
def test_each_dynamic_check_rejects_lost_or_replaced_mount(mounted,fault):
    output,state,row=mounted
    guard=destination.CaptureDestination(output,'ABCD-1234')
    if fault=='missing': state['rows']=[]
    elif fault=='wrong_uuid': state['required_device']=2050
    elif fault=='read_only': row['options']=['ro']
    elif fault=='super_read_only': row['super_options']=['ro']
    elif fault=='root_disk': state['root_device']=2049
    else: row['mount_id']='43'
    with pytest.raises(ValueError,match='CAPTURE_DESTINATION_UNAVAILABLE'): guard.check()
    assert not output.exists()


def test_linked_archive_ancestor_rejected_before_mount_lookup(tmp_path,monkeypatch):
    output=tmp_path/'linked'/'future'
    original=Path.lstat
    monkeypatch.setattr(Path,'lstat',lambda path:SimpleNamespace(st_mode=stat.S_IFLNK)
        if path==output.parent else original(path))
    monkeypatch.setattr(destination,'_mounts',lambda:pytest.fail('linked paths must be rejected first'))
    with pytest.raises(ValueError,match='symlinks'): destination.CaptureDestination(output,'ABCD-1234')


def test_stacked_mount_records_are_rejected(mounted):
    output,state,row=mounted
    state['rows'].append(dict(row,mount_id='43'))
    with pytest.raises(ValueError,match='stacked mounts'):
        destination.CaptureDestination(output,'ABCD-1234')


def test_mountinfo_decodes_spaces_and_retains_separate_rw_flags(tmp_path,monkeypatch):
    info=tmp_path/'mountinfo'
    info.write_text('42 1 8:1 / /media/USB\\040Disk rw,nosuid - fuseblk /dev/sda1 rw,user_id=1000\n')
    monkeypatch.setattr(destination,'MOUNTINFO',info)
    row=destination._mounts()[0]
    assert row['mount_point']=='/media/USB Disk' and row['device']=='8:1'
    assert row['options']==['rw','nosuid'] and row['super_options'][0]=='rw'


def test_invalid_mount_starts_nothing_and_creates_no_archive(mounted,tmp_path,monkeypatch):
    from wc_runtime import cli
    output,state,_=mounted
    state['rows']=[]
    monkeypatch.setattr(cli,'ROOT',tmp_path)
    monkeypatch.setattr(cli,'target',lambda:None)
    monkeypatch.setattr(capture,'preflight',lambda *a,**k:pytest.fail('mount must be checked first'))
    monkeypatch.setattr(capture.subprocess,'Popen',lambda *a,**k:pytest.fail('no process allowed'))
    args=SimpleNamespace(profile='mapping_core',session='sample',duration=1,staging='memory',
        output_root=output,required_output_uuid='ABCD-1234',dry_run=False,diagnostic=True)
    with pytest.raises(ValueError,match='CAPTURE_DESTINATION_UNAVAILABLE'): capture.run_capture(args)
    assert not output.exists() and not (tmp_path/'data').exists()


def test_external_dry_run_uses_usb_capacity_path_and_preserves_memory_default(mounted,tmp_path,monkeypatch,capsys):
    from wc_runtime import cli
    output,_,row=mounted
    output.mkdir(parents=True)
    monkeypatch.setattr(cli,'ROOT',tmp_path)
    monkeypatch.setattr(cli,'target',lambda:None)
    monkeypatch.setattr(capture,'MEMORY_STAGING_ROOT',tmp_path/'ram')
    seen=[]
    def preflight(*args,**kwargs):
        seen.append(kwargs['capacity_path'])
        return {'capacity':{'sufficient':True,'status':'AVAILABLE'}}
    monkeypatch.setattr(capture,'preflight',preflight)
    monkeypatch.setattr(capture,'memory_preflight',lambda *a,**k:{'sufficient':True,'status':'AVAILABLE'})
    args=SimpleNamespace(profile='mapping_core',session='sample',duration=1,staging='memory',
        output_root=output,required_output_uuid='ABCD-1234',dry_run=True,diagnostic=False)
    assert capture.run_capture(args)==0
    assert seen==[Path(row['mount_point'])] and not (output/'sample').exists()
    import json
    report=json.loads(capsys.readouterr().out)
    assert report['output']==str(output/'sample') and report['storage_staging']['mode']=='memory'


def test_missing_external_output_root_starts_nothing_and_creates_no_parents(mounted,tmp_path,monkeypatch):
    from wc_runtime import cli
    output,_,_=mounted
    monkeypatch.setattr(cli,'ROOT',tmp_path)
    monkeypatch.setattr(cli,'target',lambda:None)
    monkeypatch.setattr(capture,'preflight',lambda *a,**k:pytest.fail('archive must exist before preflight'))
    monkeypatch.setattr(capture.subprocess,'Popen',lambda *a,**k:pytest.fail('no process allowed'))
    args=SimpleNamespace(profile='mapping_core',session='sample',duration=1,staging='memory',
        output_root=output,required_output_uuid='ABCD-1234',dry_run=False,diagnostic=True)
    with pytest.raises(ValueError,match='external output-root must already exist'):
        capture.run_capture(args)
    assert not output.exists() and not output.parent.exists()


def test_preflight_measures_selected_filesystem_not_project_disk(tmp_path,monkeypatch):
    from wc_runtime import cli
    monkeypatch.setattr(cli,'device_preflight',lambda:None)
    monkeypatch.setattr(cli,'imu_preflight',lambda:None)
    selected=tmp_path/'usb'
    calls=[]
    monkeypatch.setattr(capture.shutil,'disk_usage',
        lambda path:(calls.append(path) or SimpleNamespace(free=9*1024**3)))
    checks=capture.preflight(tmp_path,'mapping_core',1,capacity_path=selected)
    assert calls==[selected] and checks['capacity']['free_bytes']==9*1024**3


def test_exfat_direct_manual_capture_rejected_before_preflight(mounted,tmp_path,monkeypatch):
    from wc_runtime import cli
    output,_,_=mounted
    monkeypatch.setattr(cli,'ROOT',tmp_path)
    monkeypatch.setattr(cli,'target',lambda:None)
    monkeypatch.setattr(capture,'preflight',lambda *a,**k:pytest.fail('manual socket unsupported'))
    args=SimpleNamespace(profile='mapping_core',session='sample',duration=1,staging='disk',
        output_root=output,required_output_uuid='ABCD-1234',manual_drive=True)
    with pytest.raises(ValueError,match='supports only --staging memory'): capture.run_capture(args)


def test_transfer_refuses_unverified_destination_before_writing(tmp_path):
    staged=tmp_path/'ram'; staged.mkdir()
    (staged/'raw').write_bytes(b'keep')
    final=tmp_path/'missing_usb'
    guard=SimpleNamespace(check=lambda:(_ for _ in ()).throw(ValueError('detached')))
    with pytest.raises(ValueError,match='detached'):
        capture.transfer_capture(staged,final,{},destination_guard=guard)
    assert (staged/'raw').read_bytes()==b'keep' and not final.exists()


def test_failure_reporting_keeps_ram_without_recreating_missing_usb(tmp_path):
    staged=tmp_path/'ram'; staged.mkdir()
    final=tmp_path/'missing_usb'
    guard=SimpleNamespace(check=lambda:(_ for _ in ()).throw(ValueError('detached')))
    assert capture.retain_staging_failure(staged,final,{},ValueError('detached'),destination_guard=guard)==2
    assert (staged/'capture_manifest.json').is_file() and not final.exists()
