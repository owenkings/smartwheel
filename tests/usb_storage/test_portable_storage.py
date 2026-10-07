import hashlib
import json
from pathlib import Path
import pytest
from wc_runtime import storage_policy as sp
from wc_runtime import capture_destination as cd
from wc_runtime import storage_directory as sd


def directory_config(tmp_path, monkeypatch):
    project=tmp_path/'code';(project/'config').mkdir(parents=True)
    archive=tmp_path/'user'/'wheelchair-data'
    (archive/'data/experiments').mkdir(parents=True)
    monkeypatch.setenv('HOME',str(archive.parent))
    config=dict(schema_version=2,enabled=True,backend='directory',archive_root='~/wheelchair-data',fallback_allowed=False)
    (project/'config/storage.json').write_text(json.dumps(config))
    monkeypatch.setattr(cd,'CaptureDestination',lambda *_:pytest.fail('directory must not probe USB UUID'))
    return project,archive,config


def test_directory_capture_and_paths_without_usb(tmp_path,monkeypatch):
    project,archive,_=directory_config(tmp_path,monkeypatch)
    policy=sp.StoragePolicy(project)
    assert policy.resolve('reports/a')==archive/'reports/a'
    assert policy.resolve('config/hardware_setup.json')==project/'config/hardware_setup.json'
    output,guard=cd.capture_destination(project)
    assert output==archive/'data/experiments'
    assert guard.check()['backend']=='directory'
    assert guard.metadata['filesystem']
    assert guard.storage_policy.backend=='directory'
    assert not (project/'reports').exists()


def test_missing_directory_never_created(tmp_path,monkeypatch):
    project,archive,config=directory_config(tmp_path,monkeypatch)
    config['archive_root']=str(tmp_path/'missing')
    (project/'config/storage.json').write_text(json.dumps(config))
    with pytest.raises(ValueError,match='must exist'):sp.StoragePolicy(project).check()
    assert not (tmp_path/'missing').exists()


def test_local_full_override_selected_and_frozen(tmp_path,monkeypatch):
    project,archive,config=directory_config(tmp_path,monkeypatch)
    alternate=tmp_path/'local';alternate.mkdir()
    config['archive_root']=str(alternate)
    local=project/'config/storage.local.json';local.write_text(json.dumps(config))
    policy=sp.StoragePolicy(project)
    assert policy.resolve('reports/a')==alternate/'reports/a'
    local.unlink()
    assert policy.resolve('reports/a')==alternate/'reports/a'
    assert sp.StoragePolicy(project).resolve('reports/a')==archive/'reports/a'
    assert not (project/'reports').exists()


def test_broken_local_refuses_fallback(tmp_path,monkeypatch):
    project,archive,_=directory_config(tmp_path,monkeypatch)
    (project/'config/storage.local.json').write_text('{broken')
    with pytest.raises(ValueError):sp.StoragePolicy(project)
    assert not (archive/'reports').exists()


def test_snapshot_is_selected_original_bytes_and_original_files(tmp_path,monkeypatch):
    project,archive,config=directory_config(tmp_path,monkeypatch)
    raw=json.dumps(config,indent=3).encode()+b'\n'
    (project/'config/storage.local.json').write_bytes(raw)
    policy=sp.StoragePolicy(project)
    (project/'config/storage.local.json').write_text('{changed after startup')
    output=tmp_path/'configuration';output.mkdir()
    hashes=policy.snapshot_configuration(output)
    assert (output/'storage.json').read_bytes()==raw
    assert (output/'storage_local.json').read_bytes()==raw
    selection=json.loads((output/'storage_selection.json').read_text())
    assert selection['selected_file']=='storage.local.json'
    assert selection['effective_sha256']==hashlib.sha256(raw).hexdigest()
    for name,digest in hashes.items():
        assert hashlib.sha256((output/Path(name).name).read_bytes()).hexdigest()==digest


def test_replacing_directory_breaks_active_guard(tmp_path,monkeypatch):
    project,archive,_=directory_config(tmp_path,monkeypatch)
    policy=sp.StoragePolicy(project);policy.check()
    archive.rename(archive.with_name('retained'));archive.mkdir()
    with pytest.raises(ValueError,match='changed'):policy.check()


def test_mount_change_breaks_guard(tmp_path,monkeypatch):
    project,archive,_=directory_config(tmp_path,monkeypatch)
    guard=sd.DirectoryDestination(archive)
    rows=[dict(v) for v in sd._mounts()]
    for row in rows:
        if row['mount_id']==guard.metadata['mount_id']:row['mount_id']='different'
    monkeypatch.setattr(sd,'_mounts',lambda:rows)
    with pytest.raises(ValueError,match='changed'):guard.check()


@pytest.mark.parametrize('raw',['../wheelchair-data','relative',''])
def test_invalid_config_path_not_runtime_traversal_escape(tmp_path,monkeypatch,raw):
    project,archive,config=directory_config(tmp_path,monkeypatch)
    config['archive_root']=raw;(project/'config/storage.json').write_text(json.dumps(config))
    with pytest.raises(ValueError):sp.StoragePolicy(project)


@pytest.mark.parametrize('relation',['inside','outside_parent'])
def test_archive_code_overlap_forbidden(tmp_path,monkeypatch,relation):
    project,archive,config=directory_config(tmp_path,monkeypatch)
    config['archive_root']=str(project/'outputs' if relation=='inside' else project.parent)
    (project/'config/storage.json').write_text(json.dumps(config))
    with pytest.raises(ValueError,match='separate'):sp.StoragePolicy(project)


def test_directory_local_symlink_forbidden(tmp_path,monkeypatch):
    project,archive,_=directory_config(tmp_path,monkeypatch)
    (project/'config/storage.local.json').symlink_to(project/'config/storage.json')
    with pytest.raises(ValueError,match='symlink'):sp.StoragePolicy(project)


def test_directory_runtime_traversal_forbidden(tmp_path,monkeypatch):
    project,archive,_=directory_config(tmp_path,monkeypatch)
    with pytest.raises(ValueError,match='traversal'):sp.StoragePolicy(project).resolve('data/../src')


def test_directory_compare_stops_at_dataset_loader(tmp_path,monkeypatch):
    from wc_runtime import mapping_compare
    project,archive,_=directory_config(tmp_path,monkeypatch)
    source=archive/'data/experiments/a';source.mkdir()
    seen=[]
    def load(path,**kwargs):
        seen.append(path);raise RuntimeError('STOP_AT_LOAD')
    monkeypatch.setattr(mapping_compare,'load_dataset',load)
    with pytest.raises(RuntimeError,match='STOP_AT_LOAD'):
        mapping_compare.main(['--project-root',str(project),'--dataset','data/experiments/a',
                              '--output','reports/replay','--estimators','five_state'])
    assert seen==[source]
    assert not (project/'reports').exists()


def test_directory_refine_stops_at_dataset_loader(tmp_path,monkeypatch):
    from wc_runtime import mapping_refine,mapping_compare
    project,archive,_=directory_config(tmp_path,monkeypatch)
    source=archive/'data/experiments/a';source.mkdir()
    seen=[]
    def load(path,**kwargs):
        seen.append(path);raise RuntimeError('STOP_AT_LOAD')
    monkeypatch.setattr(mapping_compare,'load_dataset',load)
    assert mapping_refine.main(['--project-root',str(project),'--dataset','data/experiments/a',
                              '--output','reports/refine']) == 2
    assert seen==[source]
    assert not (project/'reports').exists()


def test_replaced_archive_parent_breaks_capture_guard_even_if_child_retained(tmp_path,monkeypatch):
    project,archive,_=directory_config(tmp_path,monkeypatch)
    _,guard=cd.capture_destination(project)
    retained=archive.with_name('retained');archive.rename(retained);archive.mkdir()
    (retained/'data').rename(archive/'data')
    with pytest.raises(ValueError,match='changed'):guard.check()


@pytest.mark.parametrize('enabled',[False,0,1,'true',None])
def test_schema2_cannot_silently_disable_policy(tmp_path,monkeypatch,enabled):
    project,archive,config=directory_config(tmp_path,monkeypatch)
    config['enabled']=enabled;(project/'config/storage.json').write_text(json.dumps(config))
    with pytest.raises(ValueError,match='enabled=true'):sp.StoragePolicy(project)


def setup_cli_clone(tmp_path):
    import shutil
    work=Path(__file__).parents[2]
    clone=tmp_path/'clone';(clone/'scripts').mkdir(parents=True)
    (clone/'config').mkdir();(clone/'src/wc_runtime').mkdir(parents=True)
    shutil.copyfile(work/'scripts/configure_storage',clone/'scripts/configure_storage')
    for name in ('__init__.py','storage_policy.py','storage_directory.py','capture_destination.py'):
        shutil.copyfile(work/'src/wc_runtime'/name,clone/'src/wc_runtime'/name)
    return clone


def test_setup_cli_creates_directory_and_repeat_backup(tmp_path):
    import subprocess,sys
    clone=setup_cli_clone(tmp_path);archive=tmp_path/'ordinary-data'
    cmd=[sys.executable,str(clone/'scripts/configure_storage'),'--backend','directory','--archive-root',str(archive)]
    first=subprocess.run(cmd,text=True,capture_output=True)
    assert first.returncode==0,first.stderr
    selected=clone/'config/storage.local.json';raw=selected.read_bytes()
    for name in ('data/experiments','data/analysis','reports','maps'):assert (archive/name).is_dir()
    second=subprocess.run(cmd,text=True,capture_output=True)
    assert second.returncode==0,second.stderr
    assert Path(json.loads(second.stdout)['backup']).read_bytes()==raw


def test_setup_cli_rejects_code_destination_without_creating_config(tmp_path):
    import subprocess,sys
    clone=setup_cli_clone(tmp_path)
    result=subprocess.run([sys.executable,str(clone/'scripts/configure_storage'),'--backend','directory',
                           '--archive-root',str(clone/'data')],text=True,capture_output=True)
    assert result.returncode!=0
    assert not (clone/'config/storage.local.json').exists()
    assert not (clone/'data').exists()


def test_setup_cli_rejects_symlink_backup_directory(tmp_path):
    import subprocess,sys
    clone=setup_cli_clone(tmp_path);archive=tmp_path/'ordinary-data';outside=tmp_path/'outside';outside.mkdir()
    cmd=[sys.executable,str(clone/'scripts/configure_storage'),'--backend','directory','--archive-root',str(archive)]
    assert subprocess.run(cmd,capture_output=True).returncode==0
    selected=clone/'config/storage.local.json';raw=selected.read_bytes()
    (archive/'reports/storage_configuration').symlink_to(outside,target_is_directory=True)
    result=subprocess.run(cmd,text=True,capture_output=True)
    assert result.returncode!=0
    assert selected.read_bytes()==raw and not list(outside.iterdir())
