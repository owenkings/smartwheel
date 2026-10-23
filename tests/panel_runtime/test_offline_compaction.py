"""Synthetic post-export preservation, interrupted retention and viewer tests."""
import hashlib
import json
import lzma
import os
from pathlib import Path
import shutil
import struct
from types import SimpleNamespace
from unittest.mock import patch

import pytest
from wc_panel.storage import atomic_json
from wc_runtime import offline_compaction as compact


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.fixture
def saved_map(tmp_path):
    project=tmp_path/'project';(project/'config').mkdir(parents=True)
    atomic_json(project/'config/storage.json',{'schema_version':1,'enabled':False})
    dataset=project/'data/experiments/source';(dataset/'bag').mkdir(parents=True)
    (dataset/'bag/part_1.db3').write_bytes(b'synthetic original bag; never touched')
    atomic_json(dataset/'capture_manifest.json',{'status':'COMPLETE','recording_complete':True})
    batch=project/'data/analysis/batch';run=batch/'01_example';run.mkdir(parents=True)
    return SimpleNamespace(project=project,dataset=dataset,batch=batch,run=run)


def populate(case, mode='compare', tiny=False):
    cell='geometric_motion' if mode=='geometry' else 'calibrated_motion' if mode=='psd' else 'robot_localization_hz5_filter_on'
    case.cell=cell
    for directory in (case.run/cell/'input',case.run/cell/'frontend',case.run/'native'/cell/'export',
                      case.run/'native'/cell/'slam',case.run/'unrelated'):
        directory.mkdir(parents=True)
    (case.run/cell/'input/0001.cdr').write_bytes(b'input cached bytes')
    (case.run/cell/'frontend/0001.cdr').write_bytes(b'derived cached bytes')
    (case.run/cell/'trajectory.csv').write_text('stamp_ns,x,y,z\n1,1,2,3\n')
    (case.run/'unrelated/keep.cdr').write_bytes(b'unlisted category must be retained')
    export=case.run/'native'/cell/'export';case.export=export
    (export/'map_3d.pgm').write_bytes(b'P5\n2 2\n255\n\x00\xff\x7f\xff')
    (export/'map_3d.yaml').write_text('image: map_3d.pgm\nresolution: 0.05\norigin: [0,0,0]\n')
    (export/'native_export_copy.db').write_bytes(b'synthetic closed export database')
    (case.run/'native'/cell/'slam/rtabmap.db').write_bytes(b'synthetic closed original database')
    payload=b'ply\nformat binary_little_endian 1.0\nelement vertex 2000\nproperty float x\nproperty float y\nproperty float z\nend_header\n'
    payload+=b''.join(struct.pack('<fff',float(i%20),float(i//20),1.) for i in range(2000))
    (export/'map_3d_cloud.ply').write_bytes(b'ply\nend_header\n' if tiny else payload)
    report_name='result.json' if mode=='compare' else 'refinement_result.json'
    status=compact.COMPLETE[0 if mode=='compare' else 1]
    inputs={str(p):sha(p) for p in case.dataset.rglob('*') if p.is_file()}
    report=dict(status=status,dataset=str(case.dataset),recording_complete=True,input_hashes=inputs,
        source_selection={'mode':'all'},cells={cell:{}},native_map=dict(
        status='NATIVE_MAPS_CLOSED_AND_EXPORTED_REQUIRES_REVIEW',cells={cell:{'status':compact.NATIVE_COMPLETE}}))
    atomic_json(case.run/report_name,report)
    case.report=case.run/report_name
    case.manifest=case.run/('artifact_manifest.json' if mode=='compare' else 'refinement_artifacts.json')
    outputs={p.relative_to(case.run).as_posix():{'bytes':p.stat().st_size,'sha256':sha(p)}
             for p in case.run.rglob('*') if p.is_file()}
    atomic_json(case.manifest,dict(status=status,**({'outputs':outputs,'original_inputs':inputs} if mode=='compare'
                                                  else {'files':outputs,'input_hashes':inputs})))
    if mode!='compare':
        atomic_json(case.run/'refinement_complete.json',dict(status=status,result_sha256=sha(case.report),
                   artifact_manifest_sha256=sha(case.manifest)))
    return case


def run_compaction(case,verify=False):
    # Process inspection itself is covered separately; this fixture owns no device.
    with patch.object(compact,'assert_closed',return_value=[]):
        compact.execute(case.project,case.dataset,case.batch,case.run,verify_only=verify)


@pytest.mark.parametrize('mode',['compare','psd','geometry'])
def test_retains_full_outputs_originals_and_byte_exact_cloud(saved_map,mode):
    c=populate(saved_map,mode)
    originals={str(p):p.read_bytes() for p in c.dataset.rglob('*') if p.is_file()}
    protected={str(p):p.read_bytes() for p in c.run.rglob('*') if p.is_file()
               and p.suffix not in ('.ply','.cdr')}
    cloud=(c.export/'map_3d_cloud.ply').read_bytes()
    run_compaction(c)
    assert lzma.decompress((c.export/'map_3d_cloud.ply.xz').read_bytes())==cloud
    assert not (c.export/'map_3d_cloud.ply').exists()
    assert not list((c.run/c.cell/'input').glob('*.cdr'))
    assert not list((c.run/c.cell/'frontend').glob('*.cdr'))
    assert (c.run/'unrelated/keep.cdr').is_file()
    assert all(Path(p).read_bytes()==v for p,v in originals.items())
    assert all(Path(p).read_bytes()==v for p,v in protected.items())
    receipt=json.loads((c.run/compact.RECEIPT).read_text())
    assert receipt['status']=='COMPLETE' and receipt['direct_resume_supported'] is False
    run_compaction(c,verify=True)
    from wc_panel.catalog import results
    row,=results(c.batch)
    assert row['storage_status']=='COMPACTED' and row['direct_resume_supported'] is False
    from wc_panel.results_viewer import load_cloud
    points=load_cloud(c.export/'map_3d_cloud.ply.xz')
    assert points['total']==points['shown']==2000


def test_interrupted_xz_restarts_only_owned_partial_and_keeps_source(saved_map):
    c=populate(saved_map);real=compact.subprocess.run
    def interrupted(command,**kwargs):
        if command[0]=='xz':
            kwargs['stdout'].write(b'\xfd7zXZ truncated');raise KeyboardInterrupt()
        return real(command,**kwargs)
    with patch.object(compact.subprocess,'run',side_effect=interrupted):
        with pytest.raises(KeyboardInterrupt):run_compaction(c)
    assert (c.export/'map_3d_cloud.ply').is_file()
    assert (c.run/c.cell/'input/0001.cdr').is_file()
    run_compaction(c)
    assert json.loads((c.run/compact.RECEIPT).read_text())['status']=='COMPLETE'


def test_interrupted_pruning_and_metadata_temporary_can_resume(saved_map):
    c=populate(saved_map);real=Path.unlink;deleted=[]
    def interrupted(path,*args,**kwargs):
        if path.suffix=='.cdr' and path.is_relative_to(c.run/c.cell):
            if deleted:raise KeyboardInterrupt()
            deleted.append(path)
        return real(path,*args,**kwargs)
    with patch.object(Path,'unlink',interrupted):
        with pytest.raises(KeyboardInterrupt):run_compaction(c)
    assert deleted and not deleted[0].exists()
    (c.run/(compact.PACKED+'.partial')).write_bytes(b'truncated manifest')
    (c.run/(compact.RECEIPT+'.new')).write_bytes(b'truncated state')
    run_compaction(c)
    run_compaction(c)  # Idempotent replay of compaction, not of mapping.
    assert json.loads((c.run/compact.RECEIPT).read_text())['status']=='COMPLETE'


def test_missing_or_changed_artifact_and_changed_input_prevent_any_pruning(saved_map):
    c=populate(saved_map)
    (c.run/c.cell/'input/0001.cdr').write_bytes(b'tampered')
    with pytest.raises(ValueError,match='manifest mismatch'):run_compaction(c)
    assert (c.export/'map_3d_cloud.ply').exists()


def test_original_input_change_rejects_before_compression(saved_map):
    c=populate(saved_map);(c.dataset/'bag/part_1.db3').write_bytes(b'changed')
    with pytest.raises(ValueError,match='original input changed'):run_compaction(c)
    assert (c.export/'map_3d_cloud.ply').exists()
    assert (c.run/c.cell/'frontend/0001.cdr').exists()


def test_missing_manifest_entry_is_not_hidden_by_fresh_plan(saved_map):
    c=populate(saved_map);(c.run/c.cell/'input/0001.cdr').unlink()
    with pytest.raises(ValueError,match='missing artifacts'):run_compaction(c)
    assert (c.export/'map_3d_cloud.ply').exists()


def test_symlink_and_live_process_are_rejected(saved_map):
    c=populate(saved_map);(c.run/'linked').symlink_to(c.dataset,target_is_directory=True)
    with pytest.raises(ValueError,match='symlink|链接'):run_compaction(c)
    (c.run/'linked').unlink()
    with patch.object(compact,'assert_closed',side_effect=ValueError('active process references run')):
        with pytest.raises(ValueError,match='active process'):
            compact.execute(c.project,c.dataset,c.batch,c.run)
    assert (c.run/c.cell/'input/0001.cdr').is_file()


def test_bad_compressed_target_keeps_original_and_caches(saved_map):
    c=populate(saved_map);real=compact.compress_ply
    def corrupted(*args):
        result=real(*args);(c.export/'map_3d_cloud.ply.xz').write_bytes(b'corrupt');raise ValueError('bad archive')
    with patch.object(compact,'compress_ply',side_effect=corrupted):
        with pytest.raises(ValueError,match='bad archive'):run_compaction(c)
    assert (c.export/'map_3d_cloud.ply').is_file()
    assert (c.run/c.cell/'frontend/0001.cdr').is_file()
    with pytest.raises(lzma.LZMAError):run_compaction(c)


def test_nonbeneficial_compression_retains_original_cloud(saved_map):
    c=populate(saved_map,tiny=True)
    run_compaction(c)
    assert (c.export/'map_3d_cloud.ply').read_bytes()==b'ply\nend_header\n'
    assert not (c.export/'map_3d_cloud.ply.xz').exists()
    run_compaction(c)


def test_space_guard_prevents_compression_without_deletion(saved_map):
    c=populate(saved_map)
    with patch.object(compact.shutil,'disk_usage',return_value=SimpleNamespace(free=1)):
        with pytest.raises(ValueError,match='temporary space'):run_compaction(c)
    assert (c.export/'map_3d_cloud.ply').is_file() and (c.run/c.cell/'frontend/0001.cdr').is_file()


def test_incomplete_or_unclosed_maps_are_never_pruned(saved_map):
    c=populate(saved_map)
    report=json.loads(c.report.read_text());report['native_map']['status']='RUNNING';atomic_json(c.report,report)
    with pytest.raises(ValueError,match='native export incomplete'):run_compaction(c)
    assert (c.export/'map_3d_cloud.ply').is_file()



def test_capacity_preflight_checks_available_space_before_work(saved_map):
    c=populate(saved_map)
    with patch.object(compact.shutil,'disk_usage',return_value=SimpleNamespace(free=1)):
        with pytest.raises(ValueError,match='空间不足'):
            compact.preflight(c.project,c.dataset,c.batch,geometry=True)
    with patch.object(compact.shutil,'disk_usage',return_value=SimpleNamespace(free=100*1024**3)):
        compact.preflight(c.project,c.dataset,c.batch,geometry=True)
    assert (c.run/c.cell/'input/0001.cdr').exists()


def test_compacted_refine_resume_explains_regeneration(saved_map):
    from wc_runtime import mapping_refine
    c=populate(saved_map,'psd');run_compaction(c)
    args=SimpleNamespace(project_root=c.project,dataset=c.dataset,output=c.run,resume=True)
    with pytest.raises(ValueError,match='原录包创建新融合任务'):
        mapping_refine.run(args)


def test_real_cli_and_process_checks_can_compact_and_verify(saved_map):
    import subprocess,sys
    c=populate(saved_map)
    command=[sys.executable,'-s','-m','wc_runtime.offline_compaction','compact','--project-root',str(c.project),
             '--dataset',str(c.dataset),'--batch',str(c.batch),'--run',str(c.run)]
    done=subprocess.run(command,text=True,capture_output=True,timeout=30)
    assert done.returncode==0,done.stdout+done.stderr
    assert 'RUN_COMPACTION_COMPLETE' in done.stdout
    command[4]='verify'
    done=subprocess.run(command,text=True,capture_output=True,timeout=30)
    assert done.returncode==0,done.stdout+done.stderr
    assert 'COMPACTED_RUN_VERIFIED' in done.stdout
