"""RAM acquisition is never durable COMPLETE; stopped archives transfer intact."""
import json
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from wc_runtime import capture
from wc_runtime.source_archive import atomic_json, digest


def ample_budget(**overrides):
    values=dict(estimated_recording_bytes=1_500_000_000,recorder_queue_size=256,
        camera_profile=dict(width=320,height=240),tmpfs_free_bytes=30*1024**3,
        available_memory_bytes=59*1024**3,total_memory_bytes=64*1024**3)
    values.update(overrides)
    return capture.memory_budget(**values)


@pytest.mark.parametrize('constraint',['tmpfs','memory'])
def test_ram_recording_forecast_shortfall_is_advisory(constraint):
    baseline=ample_budget()
    assert baseline['sufficient'] and baseline['camera_queue_estimated_bytes']>=4*256*320*240*3
    override={'tmpfs_free_bytes':baseline['required_tmpfs_bytes']-1} if constraint=='tmpfs' else {
        'available_memory_bytes':baseline['required_memory_bytes']-1}
    report=ample_budget(**override)
    assert not report['sufficient'] and report['status']=='WARNING' and report['startup_allowed']
    assert report['memory_reserve_bytes']>=4*1024**3


@pytest.mark.parametrize('constraint',['tmpfs','memory'])
def test_ram_actual_queue_or_reserve_shortfall_still_blocks(constraint):
    baseline=ample_budget()
    override={'tmpfs_free_bytes':baseline['tmpfs_reserve_bytes']} if constraint=='tmpfs' else {
        'available_memory_bytes':baseline['startup_required_memory_bytes']}
    report=ample_budget(**override)
    assert not report['startup_allowed'] and report['status']=='BLOCKED'


def test_optional_diagnostic_succeeds_with_long_forecast_shortfall_and_starts_nothing(tmp_path,monkeypatch,capsys):
    from wc_runtime import cli
    monkeypatch.setattr(cli,'ROOT',tmp_path);monkeypatch.setattr(cli,'target',lambda:None)
    monkeypatch.setattr(capture,'MEMORY_STAGING_ROOT',tmp_path/'ram')
    forecast=capture.capacity('mapping_core',300,6*1024**3,initialization_budget_s=30)
    assert not forecast['sufficient'] and forecast['startup_allowed']
    monkeypatch.setattr(capture,'preflight',lambda *a,**k:dict(capacity=forecast))
    monkeypatch.setattr(capture,'memory_preflight',lambda *a,**k:ample_budget(estimated_recording_bytes=100*1024**3))
    def forbidden(*args,**kwargs):raise AssertionError('diagnostic must not start acquisition')
    monkeypatch.setattr(capture.subprocess,'Popen',forbidden)
    args=SimpleNamespace(profile='mapping_core',session='session',duration=300,dry_run=True,diagnostic=False,staging='memory')
    assert capture.run_capture(args)==0
    result=json.loads(capsys.readouterr().out)
    assert result['preflight']['capacity']['status']=='WARNING'
    assert result['preflight']['memory_capacity']['status']=='WARNING'
    assert not (tmp_path/'data/experiments/session').exists()


def staged_archive(tmp_path):
    staged=tmp_path/'ram'/'session'; staged.mkdir(parents=True)
    final=tmp_path/'disk'/'session'; final.mkdir(parents=True)
    manifest=dict(session_id='session',profile='mapping_cameras',status='RECORDING',recording_complete=False)
    atomic_json(staged/'capture_manifest.json',manifest)
    for relative,payload in {'sources/cameras/left_front/frame.bgr':bytes(range(256))*4096,
        'bag/bag.db3':b'original serialized messages', 'configuration/hardware_setup.json':b'{"unknown":true}'}.items():
        path=staged/relative; path.parent.mkdir(parents=True,exist_ok=True); path.write_bytes(payload)
    return staged,final,manifest


def portable_sync(monkeypatch):
    if os.name=='nt': monkeypatch.setattr(capture,'sync_capture_directory',lambda path:None)


def test_transfer_copies_every_camera_byte_and_hash_without_promoting_complete(tmp_path,monkeypatch):
    staged,final,manifest=staged_archive(tmp_path)
    portable_sync(monkeypatch)
    monkeypatch.setattr(capture.shutil,'disk_usage',lambda path:SimpleNamespace(free=10**12))
    report=capture.transfer_capture(staged,final,manifest)
    assert report['status']=='PERSISTED_AWAITING_AUDIT' and staged.is_dir()
    assert (final/'sources/cameras/left_front/frame.bgr').read_bytes()==(staged/'sources/cameras/left_front/frame.bgr').read_bytes()
    for relative,row in report['files'].items(): assert digest(final/relative)==row['sha256']==digest(staged/relative)
    value=json.loads((final/'capture_manifest.json').read_text())
    assert value['status']=='PARTIAL' and not value['recording_complete']


def test_transfer_copy_failure_retains_ram_and_partial_final_with_recovery_paths(tmp_path,monkeypatch):
    staged,final,manifest=staged_archive(tmp_path)
    monkeypatch.setattr(capture.shutil,'disk_usage',lambda path:SimpleNamespace(free=10**12))
    failing_destination=final/'bag/bag.db3'
    original_open=Path.open
    attempted_writes=[]
    class FailingWriter:
        def __init__(self,stream):self.stream=stream
        def __enter__(self):self.stream.__enter__();return self
        def __exit__(self,*args):return self.stream.__exit__(*args)
        def write(self,payload):
            attempted_writes.append((failing_destination,len(payload)))
            raise OSError('injected slow-disk write failure')
        def __getattr__(self,name):return getattr(self.stream,name)
    def failing_open(path,mode='r',*args,**kwargs):
        stream=original_open(path,mode,*args,**kwargs)
        return FailingWriter(stream) if path==failing_destination and mode=='xb' else stream
    monkeypatch.setattr(Path,'open',failing_open)
    with pytest.raises(OSError) as caught: capture.transfer_capture(staged,final,manifest)
    assert attempted_writes==[(failing_destination,(staged/'bag/bag.db3').stat().st_size)]
    assert capture.retain_staging_failure(staged,final,manifest,caught.value)==2
    assert (staged/'sources/cameras/left_front/frame.bgr').stat().st_size>0
    for directory in (staged,final):
        value=json.loads((directory/'capture_manifest.json').read_text())
        assert value['status']=='PARTIAL' and not value['recording_complete']
        assert value['staging_transfer']['staging_directory']==str(staged)
        assert value['staging_transfer']['final_directory']==str(final)
        assert value['staging_transfer']['source_retained']


def test_cleanup_only_removes_exact_current_ram_session_and_rejects_outside(tmp_path,monkeypatch):
    root=tmp_path/'ram'; root.mkdir()
    monkeypatch.setattr(capture,'MEMORY_STAGING_ROOT',root)
    current=root/'current'; current.mkdir(); (current/'raw').write_bytes(b'durable elsewhere')
    other=root/'other'; other.mkdir(); (other/'raw').write_bytes(b'keep')
    with pytest.raises(ValueError): capture.remove_memory_staging(other,'current')
    with pytest.raises(ValueError): capture.checked_staging_path('../other')
    capture.remove_memory_staging(current,'current')
    assert not current.exists() and (other/'raw').read_bytes()==b'keep'


@pytest.mark.skipif(os.name=='nt',reason='target POSIX ownership and subprocess contract')
@pytest.mark.parametrize('audit_pass',[True,False])
def test_capture_audits_final_path_after_drain_then_conditionally_removes_ram(tmp_path,monkeypatch,audit_pass):
    from wc_runtime import cli,capture_contract
    root=tmp_path/'project'; root.mkdir(); run=root/'shared_run'; ram=tmp_path/'ram'; ram.mkdir()
    # Real POSIX locks remain exercised, but only in this test's root.
    from wc_runtime import runtime_locks
    lock_root=tmp_path/'resource_locks'; lock_root.mkdir(mode=0o700)
    monkeypatch.setattr(runtime_locks, 'shared_lock_root', lambda: lock_root)
    monkeypatch.setattr(cli,'ROOT',root);monkeypatch.setattr(cli,'RUN',run)
    monkeypatch.setattr(cli,'target',lambda:None);monkeypatch.setattr(cli,'environment',lambda:{})
    monkeypatch.setattr(cli,'ros_command',lambda arguments:list(map(str,arguments)))
    monkeypatch.setattr(capture,'MEMORY_STAGING_ROOT',ram)
    monkeypatch.setattr(capture,'preflight',lambda *a,**k:dict(capacity={'sufficient':True}))
    monkeypatch.setattr(capture,'memory_preflight',lambda *a,**k:ample_budget())
    monkeypatch.setattr(capture,'host_memory',lambda:dict(MemAvailable=59*1024**3,MemTotal=64*1024**3))
    monkeypatch.setattr(capture.shutil,'disk_usage',lambda path:SimpleNamespace(free=10**12))
    def snapshot(project,directory,**kwargs):
        config=directory/'configuration';config.mkdir();(config/'runtime_config.json').write_text('{}')
        project_source=Path(__file__).resolve().parents[2]
        wheel=json.loads((project_source/'config/wheel_feedback_current.json').read_text(encoding='utf-8'))
        atomic_json(config/'wheel_feedback.json',wheel)
        for side in ('left','right'):
            path=config/'lidar'/(side+'.yaml');path.parent.mkdir(exist_ok=True)
            path.write_text('side: '+side+'\nexpected_serial: synthetic_'+side+
                            '\ndevice_ip: synthetic_'+side+'\nreceive_ip: synthetic\n',encoding='utf-8')
        return {str(path.relative_to(directory)):digest(path) for path in config.rglob('*') if path.is_file()}
    monkeypatch.setattr(capture,'snapshot',snapshot)
    observed=[]
    def commands(project,shared_run,directory,session,profile,**kwargs):
        assert shared_run==run and directory==ram/session
        return {'lidar':['synthetic_source']}
    monkeypatch.setattr(capture,'source_commands',commands)
    class Process:
        def poll(self): return None
    def launch(command,**kwargs):
        if 'wc_runtime.source_recorder' in command:
            active=Path(command[command.index('--output')+1]);atomic_json(active/'recorder_ready.json',{'ready':True})
        return Process()
    monkeypatch.setattr(capture.subprocess,'Popen',launch)
    tick=[0.]
    def clock():tick[0]+=.25;return tick[0]
    monkeypatch.setattr(capture.time,'monotonic',clock);monkeypatch.setattr(capture.time,'sleep',lambda _:None)
    monkeypatch.setattr(capture.time,'monotonic_ns',lambda:int(tick[0]*1e9))
    def readiness(directory,contract):
        # Transfer supervision is isolated from the independently audited
        # source journals; simulate sink readiness/tail for every required ID.
        rows={logical:dict(first_valid_monotonic_ns=1,last_valid_monotonic_ns=int((tick[0]+2)*1e9))
              for logical in contract['sources']}
        return dict(all_ready=True,ready=True,sources=rows,missing=[],invalid={},observed_common_ready_ns=1)
    monkeypatch.setattr(capture_contract,'read_source_readiness',readiness)
    def stop(children,recorder,**kwargs):
        observed.append('drained');(ram/'session/sources/camera.bgr').write_bytes(b'all camera bytes')
        return {'lidar':0,'recorder':0}
    monkeypatch.setattr(capture,'stop_capture_children',stop)
    def finalize(directory,shared_run,manifest,hashes,*args):
        assert observed==['drained'] and directory==root/'data/experiments/session' and shared_run==run
        assert manifest['output']==str(directory) and manifest['storage_staging']['durable_archive']
        assert (directory/'sources/camera.bgr').read_bytes()==b'all camera bytes'
        assert digest(directory/'configuration/runtime_config.json')==hashes['configuration/runtime_config.json']
        observed.append('disk_audit')
        atomic_json(directory/'capture_manifest.json',dict(manifest,status='COMPLETE' if audit_pass else 'PARTIAL',recording_complete=audit_pass))
        return 0 if audit_pass else 2
    monkeypatch.setattr(capture,'finalize_capture',finalize)
    args=SimpleNamespace(profile='mapping_core',session='session',duration=1,dry_run=False,diagnostic=False,staging='memory')
    assert capture.run_capture(args)==(0 if audit_pass else 2)
    assert observed==['drained','disk_audit'] and (ram/'session').exists() is (not audit_pass)


def test_memory_capacity_failure_starts_no_children(tmp_path,monkeypatch):
    from wc_runtime import cli
    monkeypatch.setattr(cli,'ROOT',tmp_path);monkeypatch.setattr(cli,'target',lambda:None)
    monkeypatch.setattr(capture,'MEMORY_STAGING_ROOT',tmp_path/'ram')
    monkeypatch.setattr(capture,'preflight',lambda *a,**k:dict(capacity={'sufficient':True}))
    monkeypatch.setattr(capture,'memory_preflight',lambda *a,**k:dict(sufficient=False,status='BLOCKED'))
    def forbidden(*args,**kwargs):raise AssertionError('capacity failure must not start acquisition')
    monkeypatch.setattr(capture.subprocess,'Popen',forbidden)
    args=SimpleNamespace(profile='mapping_core',session='session',duration=1,dry_run=False,diagnostic=True,staging='memory')
    with pytest.raises(RuntimeError,match='CAPTURE_MEMORY_CAPACITY_INSUFFICIENT'):capture.run_capture(args)
    assert not (tmp_path/'data/experiments/session').exists()
