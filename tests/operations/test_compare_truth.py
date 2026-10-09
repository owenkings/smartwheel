"""Common truth support, metric meaning, and retained failed-experiment artifacts."""
from collections import Counter
import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

import wc_runtime.mapping_compare as comparison


@pytest.fixture
def isolated_comparison_storage(monkeypatch):
    """Truth/report tests own tmp inputs and do not inspect host storage."""
    from wc_runtime import storage_policy
    class FixtureStoragePolicy:
        enabled = False
        def __init__(self, root):
            self.archive_root = Path(root)
        def check(self):
            return {'status': 'SYNTHETIC_TEST_STORAGE'}
    monkeypatch.setattr(storage_policy, 'StoragePolicy', FixtureStoragePolicy)


def pose(x=0.,*,rpy=(0.,0.,0.)):
    matrix=np.eye(4); matrix[0,3]=x
    matrix[:3,:3]=Rotation.from_euler('xyz',rpy,degrees=True).as_matrix()
    return matrix.tolist()


def truth_file(tmp_path,stamps=(1,2,3),*,poses=None):
    path=tmp_path/'truth.json'
    value={'independent':True,'time_correspondence_status':'CONFIRMED','frame_id':'mapping_odom',
           'child_frame_id':'mapping_reference','evidence_id':'external_test',
           'poses':[{'stamp_ns':stamp,'T_mapping_odom_reference':poses[i] if poses else pose()} for i,stamp in enumerate(stamps)]}
    path.write_text(json.dumps(value),encoding='utf-8')
    return path


def cell(stamps,*,poses=None):
    return {'frames':{stamp:{'T_prior_reference':poses[i] if poses else pose()} for i,stamp in enumerate(stamps)}}


def test_all_cells_have_identical_truth_support_with_every_exclusion_visible(tmp_path):
    report=comparison.truth_errors({'a':cell([1,2,3]),'b':cell([2,3,4])},truth_file(tmp_path))
    assert report['status']=='COMMON_EXACT_TRUTH_DIAGNOSTICS_COMPLETE'
    assert report['common_truth_stamp_ns']==[2,3]
    assert report['truth_stamps_excluded_from_all_cell_comparison']==[1]
    assert report['cells']['a']['excluded_due_to_other_cells_stamp_ns']==[1]
    assert report['cells']['b']['missing_truth_stamp_ns']==[1]
    assert report['cells']['b']['output_without_truth_count']==1
    assert [row['matched_truth_samples'] for row in report['cells'].values()]==[2,2]


def test_so3_and_horizontal_yaw_are_distinct_and_wrapped(tmp_path):
    report=comparison.truth_errors({'roll':cell([1,2],poses=[pose(rpy=(30.,0.,0.)),pose(rpy=(0.,0.,-179.))])},
        truth_file(tmp_path,[1,2],poses=[pose(),pose(rpy=(0.,0.,179.))]))
    assert report['cells']['roll']['so3_rotation_error_deg']['max']==pytest.approx(30.)
    assert report['cells']['roll']['yaw_error_deg']['max']==pytest.approx(2.)
    assert report['cells']['roll']['rotation_error_deg_alias'].startswith('SO(3)')


def test_undefined_yaw_is_one_shared_subset_not_per_cell_pruning(tmp_path):
    report=comparison.truth_errors({'a':cell([1,2]),'b':cell([1,2],poses=[pose(rpy=(0.,90.,0.)),pose()])},truth_file(tmp_path,[1,2]))
    assert report['common_yaw_stamp_ns']==[2]
    assert report['yaw_excluded_undefined_heading_stamp_ns']==[1]
    assert all(row['matched_yaw_samples']==1 and row['matched_truth_samples']==2 for row in report['cells'].values())


@pytest.mark.parametrize('minimum,stamps',[(1,[4]),(2,[1])])
def test_no_or_insufficient_common_truth_is_failure_without_empty_error_claim(tmp_path,minimum,stamps):
    report=comparison.truth_errors({'a':cell(stamps)},truth_file(tmp_path),min_common_samples=minimum)
    assert report['status']=='FAILED_INSUFFICIENT_COMMON_TRUTH'
    assert 'position_error_m' not in report['cells']['a']
    assert report['truth_exclusion_count']>=2


def test_invalid_noncommon_output_cannot_be_silently_pruned(tmp_path):
    broken=cell([1,4]); broken['frames'][4]['T_prior_reference'][0][0]=2.
    report=comparison.truth_errors({'a':broken,'b':cell([1])},truth_file(tmp_path))
    assert report['status']=='FAILED_INVALID_TRUTH_OR_ESTIMATE'
    assert report['invalid_estimates'][0]['stamp_ns']==4
    assert 'position_error_m' not in report['cells']['a']


def test_duplicate_truth_retains_failure_record(tmp_path):
    report=comparison.truth_errors({'a':cell([1])},truth_file(tmp_path,[1,1]))
    assert report['status']=='FAILED_INVALID_TRUTH_OR_ESTIMATE'
    assert report['invalid_truth_rows'][0]['row_index']==1


@pytest.mark.parametrize('disjoint',[False,True])
def test_cli_insufficient_truth_retains_completed_trajectories_and_manifest(tmp_path,monkeypatch,disjoint,isolated_comparison_storage):
    dataset=tmp_path/'dataset'; dataset.mkdir()
    original=dataset/'original.json'; original.write_text('{}')
    monkeypatch.setattr(comparison,'load_dataset',lambda *a,**kw:({},None,{str(original):comparison.digest(original)}))
    class Packets:
        def __init__(self,*args): self.events=[]; self.counts=Counter(); self.clock=self
        def report(self): return {'mapping':'TEST'}
        def close(self): pass
    monkeypatch.setattr(comparison,'RecordedPackets',Packets)
    def replay(root,output,runtime,candidate,packets,name,estimator,filtering,limit,rate):
        directory=output/name; directory.mkdir()
        selected_stamp=2 if disjoint and filtering else 1
        result=cell([selected_stamp]); result.update(directory=directory,pairs={selected_stamp:{'sources':[]}},
            summary={'estimator':estimator,'input_rate_hz':rate,'filter_enabled':filtering,
                     'output_frames':1,'pending_clouds_at_input_end':0})
        return result
    monkeypatch.setattr(comparison,'replay_cell',replay)
    output=tmp_path/'comparison'; path=truth_file(tmp_path,[1,2] if disjoint else [1])
    code=comparison.main(['--dataset',str(dataset),'--output',str(output),'--estimators','five_state',
        '--filter','off','on','--truth-json',str(path),'--truth-min-common-samples','2'])
    result=json.loads((output/'result.json').read_text())
    assert code!=0 and result['status']=='FAILED' and len(result['cells'])==2
    assert result['truth']['status']=='FAILED_INSUFFICIENT_COMMON_TRUTH'
    assert (output/'truth_evaluation.json').is_file() and (output/'truth_input.json').is_file()
    assert len(list(output.glob('*/trajectory.csv')))==2
    assert '真值评估失败' in (output/'summary_zh.md').read_text(encoding='utf-8')
    manifest=json.loads((output/'artifact_manifest.json').read_text())
    assert 'truth_evaluation.json' in manifest['outputs'] and str(path) in manifest['original_inputs']


@pytest.mark.parametrize('fail_first', [True, False])
def test_cli_failed_cell_records_supplied_truth_not_run_and_keeps_input_evidence(tmp_path, monkeypatch, fail_first, isolated_comparison_storage):
    dataset=tmp_path/'dataset'; dataset.mkdir()
    original=dataset/'original.json'; original.write_text('{}')
    monkeypatch.setattr(comparison,'load_dataset',lambda *a,**kw:({},None,{str(original):comparison.digest(original)}))
    class Packets:
        def __init__(self,*args): self.events=[]; self.counts=Counter(); self.clock=self
        def report(self): return {'mapping':'TEST'}
        def close(self): pass
    monkeypatch.setattr(comparison,'RecordedPackets',Packets)
    def replay(root,output,runtime,candidate,packets,name,estimator,filtering,limit,rate):
        directory=output/name; directory.mkdir()
        if fail_first or filtering:
            raise RuntimeError('injected replay cell failure')
        result=cell([1]); result.update(directory=directory,pairs={1:{'sources':[]}},
            summary={'estimator':estimator,'input_rate_hz':rate,'filter_enabled':filtering,
                     'output_frames':1,'pending_clouds_at_input_end':0})
        return result
    monkeypatch.setattr(comparison,'replay_cell',replay)
    def unexpected_evaluation(*args,**kwargs):
        raise AssertionError('truth metrics must not run after a failed experiment cell')
    monkeypatch.setattr(comparison,'truth_errors',unexpected_evaluation)
    output=tmp_path/'comparison'; path=truth_file(tmp_path,[1])
    code=comparison.main(['--dataset',str(dataset),'--output',str(output),'--estimators','five_state',
        '--filter','off','on','--truth-json',str(path)])
    result=json.loads((output/'result.json').read_text())
    assert code!=0 and result['status']=='FAILED'
    assert result['absolute_accuracy']=='TRUTH_EVALUATION_NOT_RUN'
    assert result['truth']['status']=='TRUTH_EVALUATION_NOT_RUN'
    assert result['truth']['reason']=='EXPERIMENT_FAILED_BEFORE_TRUTH_EVALUATION'
    assert result['truth']['error']=='RuntimeError: injected replay cell failure'
    assert result['truth']['failed_cell']==result['failed_cell']
    assert result['truth']['input_path']==str(path)
    assert result['truth']['input_sha256']==comparison.digest(path)
    assert 'common_truth_samples' not in result['truth'] and 'truth_exclusion_count' not in result['truth']
    assert json.loads((output/'truth_evaluation.json').read_text())==result['truth']
    assert (output/'truth_input.json').read_bytes()==path.read_bytes()
    assert len(list(output.glob('*/trajectory.csv')))==(0 if fail_first else 1)
    summary=(output/'summary_zh.md').read_text(encoding='utf-8')
    assert '已提供真值，但实验在评估前失败' in summary
    assert '共同样本、剔除和误差统计均未计算' in summary
    assert '未提供独立真值' not in summary
    manifest=json.loads((output/'artifact_manifest.json').read_text())
    assert manifest['absolute_accuracy']=='TRUTH_EVALUATION_NOT_RUN'
    assert manifest['original_inputs'][str(path)]==comparison.digest(path)
    assert 'truth_evaluation.json' in manifest['outputs'] and 'truth_input.json' in manifest['outputs']
