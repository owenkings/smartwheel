"""Archived eight-plus-four experiment layout, with no large sensor fixtures."""
import json
from pathlib import Path

from wc_panel.catalog import results


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding='utf-8')


def cell_files(batch, run, cell, compressed=False):
    front = batch/run/cell
    export = batch/run/'native'/cell/'export'
    front.mkdir(parents=True, exist_ok=True)
    export.mkdir(parents=True, exist_ok=True)
    (front/'trajectory.csv').write_text('stamp_ns,x_m,y_m\n1,0,0\n')
    (export/'map_3d.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
    cloud = export/('map_3d_cloud.ply.xz' if compressed else 'map_3d_cloud.ply')
    cloud.write_bytes(b'compressed fixture' if compressed else b'ply\nend_header\n')
    return front, cloud


def image_files(gallery, identifier, roles):
    gallery.mkdir(parents=True, exist_ok=True)
    images = {role:identifier+'_'+role+'.png' for role in roles}
    for name in images.values(): (gallery/name).write_bytes(b'fixture image')
    return images


def test_real_archived_eight_plus_four_layout_pairs_every_group(tmp_path):
    batch = tmp_path/'fusion_20261007'
    supplement = batch/'supplement_20261007'
    summary, gallery, retained = [], [], {}
    identities = [
        ('baseline_raw', 'five_state_hz5_filter_on'),
        ('baseline_raw', 'robot_localization_hz5_filter_on'),
        ('baseline_filtered', 'five_state_hz5_filter_on'),
        ('baseline_filtered', 'robot_localization_hz5_filter_on'),
        ('refine_raw', 'calibrated_motion'), ('refine_raw', 'geometric_motion'),
        ('refine_filtered', 'calibrated_motion'), ('refine_filtered', 'geometric_motion'),
        ('psd_raw', 'calibrated_motion'), ('psd_filtered', 'calibrated_motion'),
        ('official_corrected_raw', 'robot_localization_hz5_filter_on'),
        ('official_corrected_filtered', 'robot_localization_hz5_filter_on')]
    for index, (run, cell) in enumerate(identities, 1):
        identifier = f'{index:02d}'
        owner = batch if index <= 8 else supplement
        front, cloud = cell_files(owner, run, cell, compressed=index > 8)
        retained[identifier] = (front, cloud)
        if index <= 8:
            summary.append(dict(id=identifier, run=run, cell=cell, label='baseline',
                                cloud='filtered' if 'filtered' in run else 'raw',
                                export_directory=str(cloud.parent),
                                visualization_images=image_files(batch/'visualization', identifier, ('top', 'grid', '3d'))))
        gallery.append(dict(id=identifier, run=run, cell=cell, title='方案 '+identifier,
                            retained_trajectory=str(front/'trajectory.csv'),
                            trajectory_file='/dev/shm/deleted/'+run+'/'+cell+'/trajectory.csv',
                            ply='/dev/shm/deleted/'+run+'/native/'+cell+'/export/map_3d_cloud.ply',
                            images=image_files(supplement/'visualization', identifier,
                                               ('trajectory',) if index <= 8 else ('top', 'grid', '3d', 'trajectory'))))
        # Repeated lower-level indexes must not duplicate the named groups.
        write_json(owner/run/'refinement_result.json', {'status':'COMPLETE', 'cells':{
            name: {} for candidate_run, name in identities if candidate_run == run}})
    write_json(batch/'RESULT_SUMMARY.json', {'status':'REVIEWED', 'maps':summary})
    write_json(supplement/'visualization/visualization_manifest.json', {'status':'REVIEWED', 'cells':gallery})
    rows = results(tmp_path)
    assert [row['group_id'] for row in rows] == [f'{n:02d}' for n in range(1, 13)]
    assert len({row['id'] for row in rows}) == 12
    for row in rows:
        front, cloud = retained[row['group_id']]
        assert row['path'] == str(front)
        assert row['clouds'] == [str(cloud)]
        assert row['trajectories'] == [str(front/'trajectory.csv')]
        assert len(row['images']) == 5  # One native grid and four indexed figures.
        assert set(row['image_roles']) == {'top', 'grid', '3d', 'trajectory'}
        assert all(Path(path).name.startswith(row['group_id']+'_') for path in row['image_roles'].values())
    # Choosing a batch directory itself also yields its cells, not run aggregates.
    assert len(results(batch)) == 12


def test_nested_legacy_comparison_does_not_merge_same_basenames(tmp_path):
    run = tmp_path/'old_batch'/'compare_raw'
    cells = ['five_state_hz5_filter_on', 'robot_localization_hz5_filter_on']
    for cell in cells: cell_files(run.parent, run.name, cell)
    write_json(run/'result.json', {'status':'COMPLETE', 'cells':dict.fromkeys(cells, {})})
    rows = results(tmp_path)
    assert len(rows) == 2
    for row in rows:
        assert len(row['clouds']) == len(row['trajectories']) == 1
        assert Path(row['clouds'][0]).parent.parent.name == Path(row['trajectories'][0]).parent.name


def test_index_cannot_import_sibling_cell_images_or_external_data(tmp_path):
    batch = tmp_path/'batch'
    front, cloud = cell_files(batch, 'raw', 'official')
    outside = tmp_path/'unrelated_secret.png'
    outside.write_bytes(b'external')
    images = image_files(batch/'visualization', '04', ('top',))
    write_json(batch/'RESULT_SUMMARY.json', {'maps':[
        dict(id='02', run='raw', cell='official', export_directory=str(tmp_path),
             visualization_images={'top':images['top'], 'grid':str(outside)}),
        dict(id='99', run='../escape', cell='official', visualization_images={})]})
    rows = results(tmp_path)
    assert len(rows) == 1
    assert rows[0]['clouds'] == [str(cloud)]
    assert rows[0]['image_roles'] == {}
    assert rows[0]['images'] == [str(cloud.parent/'map_3d.pgm')]


def test_invalid_structured_index_never_aggregates_all_media(tmp_path):
    batch = tmp_path/'bad_index'
    cell_files(batch, 'first', 'cell')
    cell_files(batch, 'second', 'cell')
    write_json(batch/'RESULT_SUMMARY.json', {'maps':[{'run':'../outside', 'cell':'cell'}]})
    assert results(tmp_path) == []
