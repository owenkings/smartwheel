import math
from pathlib import Path
import pytest
from wc_runtime.mapping_refine import stage_hashes,verify_stage,save,read

def test_stage_rejects_changed_pose_index_even_if_cdr_unchanged(tmp_path):
    (tmp_path/'index.jsonl').write_text('{"pose":0}')
    (tmp_path/'cloud.cdr').write_bytes(b'unchanged')
    hashes=stage_hashes(tmp_path);verify_stage(tmp_path,hashes)
    (tmp_path/'index.jsonl').write_text('{"pose":1}')
    with pytest.raises(ValueError):verify_stage(tmp_path,hashes)

def test_stage_rejects_missing_and_extra_artifacts(tmp_path):
    (tmp_path/'a').write_bytes(b'1');hashes=stage_hashes(tmp_path)
    (tmp_path/'b').write_bytes(b'2')
    with pytest.raises(ValueError):verify_stage(tmp_path,hashes)
    (tmp_path/'a').unlink()
    with pytest.raises(ValueError):verify_stage(tmp_path,hashes)

def test_no_unsealed_stage_reuse(tmp_path):
    with pytest.raises(ValueError):verify_stage(tmp_path,None)
    with pytest.raises(ValueError):verify_stage(tmp_path,{})

def test_atomic_evidence_replacement(tmp_path):
    path=tmp_path/'status.json';save(path,{'status':'RUNNING'});save(path,{'status':'FINALIZING'})
    assert read(path)['status']=='FINALIZING' and not path.with_name('status.json.tmp').exists()

def test_default_prefers_demonstrated_motion_correction(monkeypatch):
    from wc_runtime import mapping_refine
    seen={}
    def inspect(args):seen.update(vars(args));return 0
    monkeypatch.setattr(mapping_refine,'run',inspect)
    assert mapping_refine.main(['--dataset','example','--output','new','--native-map'])==0
    assert seen['geometry']=='off' and seen['native_cells']==['calibrated_motion']

def test_native_summary_keeps_hash_link_instead_of_repeating_graph_history(tmp_path):
    from wc_runtime.mapping_refine import compact_native_report
    directory=tmp_path/'native'/'calibrated_motion';directory.mkdir(parents=True)
    save(directory/'result.json',{'graphs':[{'poses':[1,2,3]}],'processed_info':[{'stamp_ns':1}]})
    cell={'status':'complete','published_pairs':1,'acknowledged_pairs':1,
          'graphs':[{'poses':[1,2,3]}],'processed_info':[{'stamp_ns':1}]}
    result=compact_native_report({'cells':{'calibrated_motion':cell}},tmp_path)
    summary=result['cells']['calibrated_motion']
    assert 'graphs' not in summary and 'processed_info' not in summary
    assert summary['published_pairs']==summary['acknowledged_pairs']==summary['processed_info_count']==1
    assert len(summary['detailed_result_sha256'])==64
    assert cell['graphs']==[{'poses':[1,2,3]}]
