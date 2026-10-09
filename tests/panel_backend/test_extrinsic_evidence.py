"""Explicit evidence import must not silently turn unknown geometry into calibration."""
import copy
import hashlib
import json
from pathlib import Path
import shutil

import pytest

from wc_panel.parameters import ParameterStore
from wc_panel.storage import atomic_json


PROJECT=Path(__file__).resolve().parents[2]


@pytest.fixture
def store(tmp_path):
    project=tmp_path/'project'
    (project/'config').mkdir(parents=True)
    # Synthetic projects use one coherent public template, never host bindings.
    for name in ('hardware_setup.json','mapping_live.json','cameras.json','device_bindings.json',
                 'wheel_feedback_current.json'):
        shutil.copyfile(PROJECT/'config'/name,project/'config'/name)
    atomic_json(project/'config/storage.json',{'schema_version':1,'enabled':False})
    setup=json.loads((project/'config/hardware_setup.json').read_text(encoding='utf-8'))
    for source in setup['source_documents']:
        target=project/source['snapshot_path'];target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(PROJECT/source['snapshot_path'],target)
    return ParameterStore(project,tmp_path/'backups')


def edit(store):
    document=store.read()
    group=next(row for row in document['groups'] if row['id']=='extrinsics')
    return document,group,store.request_edit('extrinsics')


def import_evidence(store,tmp_path,document,group,kind='GEOMETRY_MEASUREMENT'):
    path=tmp_path/'synthetic_measurement.txt'
    path.write_text('SYNTHETIC TEST ONLY: selected lidar data frame measured for fixture.\n',encoding='utf-8')
    return store.prepare_extrinsic_evidence(str(path),kind,group['assembly_revision'],
        'Synthetic test only; fixture translation and rotation measurement, not physical calibration.',document['revision'])


def bind(group,prepared,*,filled=False,status='UNKNOWN',sides=('left',)):
    value=copy.deepcopy(group['value'])
    for transform in value:
        if transform['id'] not in {'mounting_M_from_lidar_'+side for side in sides}:continue
        for component in ('translation','rotation'):
            transform[component]['evidence']=[{'source_id':prepared['source']['id']}]
            transform[component]['status']=status
        if filled:
            transform['translation']['value_m']=[.3,.2 if transform['id'].endswith('left') else -.2,.5]
            transform['rotation']['value_matrix']=[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]
    return value


def test_import_is_pending_until_save_and_never_promotes_unknown(store,tmp_path):
    document,group,proof=edit(store)
    original=(store.project/'config/hardware_setup.json').read_bytes()
    prepared=import_evidence(store,tmp_path,document,group)
    assert (store.project/'config/hardware_setup.json').read_bytes()==original
    assert not (store.project/prepared['source']['snapshot_path']).exists()
    assert prepared['physical_accuracy_validated'] is False
    result=store.save('extrinsics',bind(group,prepared),proof['token'],document['revision'],[prepared['token']])
    setup=json.loads((store.project/'config/hardware_setup.json').read_text(encoding='utf-8'))
    left=next(row for row in setup['data_transforms'] if row['id']=='mounting_M_from_lidar_left')
    assert left['translation']['status']=='UNKNOWN' and left['translation']['value_m'] is None
    assert left['rotation']['status']=='UNKNOWN' and left['rotation']['value_matrix'] is None
    source=setup['source_documents'][-1]
    assert hashlib.sha256((store.project/source['snapshot_path']).read_bytes()).hexdigest()==source['sha256']
    assert (Path(result['backup'])/'hardware_setup.json').read_bytes()==original
    from wc_panel.live_calibration import capabilities
    assert not capabilities(store.project)['live_mapping']['by_sides']['left']['enabled']


@pytest.mark.parametrize('kind,status',[('GEOMETRY_MEASUREMENT','USER_MEASURED_EXPERIMENT'),
                                      ('EXTRINSIC_CALIBRATION','CALIBRATED')])
def test_explicit_measured_values_and_bound_evidence_enable_only_selected_side(store,tmp_path,kind,status):
    document,group,proof=edit(store)
    prepared=import_evidence(store,tmp_path,document,group,kind)
    value=bind(group,prepared,filled=True,status=status)
    store.save('extrinsics',value,proof['token'],document['revision'],[prepared['token']])
    from wc_panel.live_calibration import capabilities
    actual=capabilities(store.project)['live_mapping']['by_sides']
    assert actual['left']['enabled']
    assert not actual['right']['enabled'] and not actual['all']['enabled']
    assert not capabilities(store.project)['live_motion_correction']['enabled']


@pytest.mark.parametrize('status',['UNKNOWN','CALIBRATED'])
def test_filled_unknown_or_calibrated_with_measurement_only_is_rejected(store,tmp_path,status):
    document,group,proof=edit(store)
    prepared=import_evidence(store,tmp_path,document,group)
    with pytest.raises(ValueError,match='UNKNOWN|EXTRINSIC_CALIBRATION'):
        store.save('extrinsics',bind(group,prepared,filled=True,status=status),proof['token'],document['revision'],[prepared['token']])
    assert store.read()['revision']==document['revision']
    assert not (store.project/prepared['source']['snapshot_path']).exists()


def test_import_requires_current_assembly_explicit_type_and_nonempty_note(store,tmp_path):
    document,group,proof=edit(store)
    path=tmp_path/'measurement.txt';path.write_text('fixture')
    for kind,assembly,note in [('GEOMETRY_MEASUREMENT','OLD_ASSEMBLY','measured'),
                               ('CAD_AND_MANUAL_MEASUREMENTS',group['assembly_revision'],'nominal'),
                               ('GEOMETRY_MEASUREMENT',group['assembly_revision'],'')]:
        with pytest.raises(ValueError):store.prepare_extrinsic_evidence(path,kind,assembly,note,document['revision'])
    assert store.read()['revision']==document['revision']


def test_changed_import_original_and_unbound_token_are_rejected(store,tmp_path):
    document,group,proof=edit(store)
    prepared=import_evidence(store,tmp_path,document,group)
    with pytest.raises(ValueError,match='绑定'):
        store.save('extrinsics',group['value'],proof['token'],document['revision'],[prepared['token']])
    Path(prepared['source']['original_path']).write_text('changed after selection')
    with pytest.raises(ValueError,match='已改变'):
        store.save('extrinsics',bind(group,prepared),proof['token'],document['revision'],[prepared['token']])
    assert store.read()['revision']==document['revision']


def test_existing_source_hash_checked_before_config_save(store,tmp_path):
    document,group,proof=edit(store)
    snapshot=store.project/group['source_documents'][-1]['snapshot_path']
    snapshot.write_text('{}',encoding='utf-8')
    # A fresh revision/token cannot make tampered evidence acceptable.
    document,group,proof=edit(store)
    original=(store.project/'config/hardware_setup.json').read_bytes()
    with pytest.raises(ValueError,match='SHA-256'):
        store.save('extrinsics',group['value'],proof['token'],document['revision'])
    assert (store.project/'config/hardware_setup.json').read_bytes()==original


def test_old_revision_expires_if_evidence_snapshot_changes(store):
    document,group,proof=edit(store)
    snapshot=store.project/group['source_documents'][-1]['snapshot_path']
    snapshot.write_bytes(snapshot.read_bytes()+b'\n')
    with pytest.raises(ValueError,match='配置已改变'):
        store.save('extrinsics',group['value'],proof['token'],document['revision'])
