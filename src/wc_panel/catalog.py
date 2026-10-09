"""Recording/result discovery and the actual supported offline matrix."""
import json
import os
from pathlib import Path
from .storage import ordinary, tree_size


def read_json(path):
    path = Path(path)
    if path.stat().st_size > 4*1024*1024:
        raise ValueError('配置文件过大: ' + str(path))
    return json.loads(path.read_text(encoding='utf-8'))


def recording(path):
    path = ordinary(path)
    manifest_path = path/'capture_manifest.json'
    manifest = read_json(manifest_path) if manifest_path.is_file() else {}
    bag_name = manifest.get('bag_path', 'bag')
    runtime_name = manifest.get('runtime_config_path', 'runtime_config.json')
    for value in (bag_name, runtime_name):
        if Path(value).is_absolute() or '..' in Path(value).parts:
            raise ValueError('录包清单包含越界路径')
    bag, runtime = ordinary(path/bag_name), ordinary(path/runtime_name)
    if not bag.is_dir() or not any(bag.glob('*.db3')) or not runtime.is_file():
        raise ValueError('缺少 SQLite 原始录包或运行配置')
    config = read_json(runtime)
    complete = manifest.get('status') == 'COMPLETE' and manifest.get('recording_complete') is True
    from wc_runtime.recording_sources import source_inventory
    try:
        available = source_inventory(bag)
        inventory_error = ''
    except (ValueError, OSError) as error:
        available = {'left': [], 'right': []}
        inventory_error = str(error)
    return dict(id=path.name, name=path.name, path=str(path), status=manifest.get('status', 'LEGACY'),
                complete=complete, bytes=tree_size(path), sides=manifest.get('mode', config.get('mode', 'all')),
                available_sides=[side for side in ('left', 'right') if available[side]],
                available_clouds_by_side=available, source_inventory_error=inventory_error,
                reason='' if complete else '录制不完整或未提供完整性清单；离线处理需显式诊断许可')


def recordings(root):
    result = []
    for child in sorted(Path(root).iterdir(), reverse=True):
        if child.is_dir() and not child.name.startswith('.') and not child.is_symlink():
            try:
                result.append(recording(child))
            except (ValueError, OSError, KeyError):
                continue
    return result


def results(root):
    """Return individual experiment cells, joining galleries by run/cell identity.

    Summary/gallery IDs label a cell; they do not identify an arbitrary file
    with the same basename. A retained trajectory relocates historical /dev/shm
    references to the archived cell without reading outside the selected batch.
    """
    root = ordinary(Path(root).absolute())
    rows = []
    marker_names = {'result.json', 'refinement_result.json', 'panel_result.json',
                    'RESULT_SUMMARY.json', 'visualization_manifest.json'}

    def linked(path):
        return path.is_symlink() or getattr(path, 'is_junction', lambda: False)()

    def component(value):
        return (isinstance(value, str) and value not in ('', '.', '..')
                and '/' not in value and '\\' not in value and ':' not in value)

    def bounded(value, base, batch):
        if not isinstance(value, str) or not value:
            return None
        candidate = Path(value)
        if not candidate.is_absolute(): candidate = base/candidate
        try:
            candidate = ordinary(candidate)
            return candidate if candidate.is_relative_to(batch) else None
        except (ValueError, OSError):
            return None

    def files_under(directory):
        if not directory.is_dir() or linked(directory): return
        for current, directories, files in os.walk(directory, followlinks=False):
            current = Path(current)
            directories[:] = [name for name in directories if not linked(current/name)]
            for name in files:
                path = current/name
                if not linked(path) and path.is_file(): yield path

    def discover(batch):
        if linked(batch): return
        entries = []
        # Metadata is shallow. Do not enumerate millions of archived frame files
        # to find experiment indexes, nor descend into native result.json files.
        for current, directories, files in os.walk(batch, followlinks=False):
            current = Path(current)
            depth = len(current.relative_to(batch).parts)
            directories[:] = [name for name in directories
                              if depth < 4 and not name.startswith('.')
                              and name not in ('native', 'input', 'execution', 'configuration', '_configuration')
                              and not linked(current/name)]
            for name in sorted(marker_names.intersection(files)):
                path = current/name
                if linked(path): continue
                try:
                    metadata = read_json(path)
                    if isinstance(metadata, dict): entries.append((path, metadata))
                except (OSError, ValueError): continue

        grouped = {}

        def add_cell(run_root, cell_name, metadata, group=None, gallery=None):
            if not component(cell_name): return
            run_root = bounded(str(run_root), batch, batch)
            if run_root is None: return
            front, native = run_root/cell_name, run_root/'native'/cell_name
            if any(linked(p) for p in (front, native, native.parent)): return
            if not front.is_dir() and not native.is_dir(): return
            key = str(front)
            if key not in grouped:
                grouped[key] = dict(path=front if front.is_dir() else native,
                                    media_roots=(front, native), status=metadata.get('status', 'AVAILABLE'),
                                    dataset=metadata.get('dataset') or metadata.get('session'),
                                    source_selection=metadata.get('source_selection'),
                                    group_id=None, run=run_root.name, cell=cell_name,
                                    title=None, image_roles={})
            row = grouped[key]
            if metadata.get('source_selection'):
                row['source_selection'] = metadata['source_selection']
            if not group: return
            identifier = str(group.get('id', ''))
            if identifier and row['group_id'] not in (None, identifier): return
            if identifier: row['group_id'] = identifier
            title = group.get('title') or group.get('label')
            if title and (not row['title'] or group.get('title')):
                if not group.get('title'):
                    title += ' / '+str(group.get('cloud', run_root.name))
                    if group.get('geometry_enabled'): title += ' / 几何开启'
                row['title'] = title
            images = group.get('visualization_images', group.get('images', {}))
            if isinstance(images, dict) and gallery is not None:
                for role, value in images.items():
                    path = bounded(value, gallery, batch)
                    if path is None or not path.is_file(): continue
                    # Numbered gallery assets must belong to this numbered cell.
                    prefix = path.name.split('_', 1)[0]
                    if prefix.isdigit() and identifier and prefix != identifier: continue
                    row['image_roles'].setdefault(str(role), str(path))

        # Explicit summary labels take precedence, then galleries enrich them
        # (including the 12-cell supplement with relocated retained_trajectory).
        entries.sort(key=lambda item: (0 if item[0].name == 'RESULT_SUMMARY.json' else
                                      1 if item[0].name == 'visualization_manifest.json' else 2,
                                      len(item[0].parts), str(item[0])))
        for path, metadata in entries:
            maps = metadata.get('maps')
            if not isinstance(maps, list) and path.name == 'visualization_manifest.json':
                maps = metadata.get('cells')
            if isinstance(maps, list):
                owner = path.parent.parent if path.name == 'visualization_manifest.json' else path.parent
                gallery = path.parent if path.name == 'visualization_manifest.json' else owner/'visualization'
                for group in maps:
                    if not isinstance(group, dict): continue
                    run, cell = group.get('run'), group.get('cell')
                    if not component(run) or not component(cell): continue
                    run_root = owner/run
                    # Kept trajectories are authoritative relocation evidence.
                    # Historical export/ply paths are intentionally not followed.
                    for name in ('retained_trajectory', 'trajectory_file'):
                        source = bounded(group.get(name), owner, batch)
                        if (source is not None and source.is_file()
                                and source.parent.name == cell and source.parent.parent.name == run):
                            run_root = source.parent.parent
                            break
                    add_cell(run_root, cell, metadata, group, gallery)
            cells = metadata.get('cells')
            if isinstance(cells, dict):
                for cell in cells: add_cell(path.parent, cell, metadata)

        def emit(path, media_roots, metadata):
            files = set()
            for directory in media_roots: files.update(files_under(directory))
            roles = metadata.get('image_roles', {})
            files.update(Path(value) for value in roles.values())
            images, clouds, trajectories = [], [], []
            for file in sorted(files):
                suffix, name = file.suffix.lower(), file.name.lower()
                if suffix in ('.png', '.jpg', '.jpeg', '.pgm'): images.append(str(file))
                elif suffix in ('.ply', '.pcd') or name.endswith(('.ply.xz', '.pcd.xz')): clouds.append(str(file))
                elif suffix in ('.csv', '.jsonl') and any(n in name for n in ('trajectory', 'pose', 'route')):
                    trajectories.append(str(file))
            if not (images or clouds or trajectories or metadata.get('status')): return
            relative = str(path.relative_to(root))
            group_id = metadata.get('group_id')
            title = metadata.get('title')
            label = relative if not group_id else '{} / {} · {}'.format(
                str(batch.relative_to(root)) if batch != root else batch.name, group_id, title or metadata['cell'])
            source_selection = metadata.get('source_selection') or {}
            selection_label = {'all': '双雷达', 'left': '仅左雷达', 'right': '仅右雷达'}.get(source_selection.get('mode'))
            if selection_label: label += ' · ' + selection_label
            rows.append(dict(id=relative, name=label, path=str(path),
                             status=metadata.get('status', 'AVAILABLE'), dataset=metadata.get('dataset'),
                             bytes=sum(file.stat().st_size for file in files),
                             images=images, clouds=clouds, trajectories=trajectories,
                             group_id=group_id, run=metadata.get('run'), cell=metadata.get('cell'), image_roles=roles,
                             source_selection=source_selection))

        if grouped:
            for row in sorted(grouped.values(), key=lambda row: (row['group_id'] or '~~~~', str(row['path']))):
                emit(row['path'], row['media_roots'], row)
        else:
            # Unindexed, single saved maps remain readable. A structured but
            # invalid index must not collapse unrelated cells into one result.
            structured = any(isinstance(meta.get('maps'), list) or isinstance(meta.get('cells'), (dict, list))
                             for _, meta in entries)
            if not structured:
                metadata = next((meta for _, meta in entries), {})
                emit(batch, [batch], metadata)

    if any((root/name).is_file() for name in ('RESULT_SUMMARY.json', 'result.json', 'refinement_result.json', 'panel_result.json')):
        discover(root)
    else:
        for child in sorted(root.iterdir(), reverse=True):
            if child.is_dir() and not child.name.startswith('.'):
                try: discover(child)
                except (OSError, ValueError, KeyError): continue
    return rows


def variants():
    rows = []
    def add(estimator, cloud, filtering, motion, noise='legacy', geometry=False, enabled=True, reason=''):
        identifier = '_'.join([estimator, cloud, 'filter' if filtering else 'unfiltered',
                               'motion' if motion else 'original', noise, 'geometry' if geometry else 'plain'])
        label = ' / '.join(['官方 EKF' if estimator == 'robot_localization' else '五状态 EKF', cloud,
                  '点云筛选开' if filtering else '点云筛选关', '运动修正' if motion else '原始运动',
                  '连续噪声 PSD' if noise == 'white_acceleration' else '原噪声', '几何纠偏' if geometry else '无几何纠偏'])
        rows.append(dict(id=identifier, label=label, estimator=estimator, cloud=cloud,
                    filtering=filtering, motion_correction=motion, process_noise=noise,
                    geometry=geometry, enabled=enabled, reason=reason))
    for cloud in ('raw', 'filtered'):
        for estimator in ('robot_localization', 'five_state'):
            for filtering in (False, True):
                for motion in (False, True):
                    add(estimator, cloud, filtering, motion)
        add('five_state', cloud, True, True, 'white_acceleration')
        for noise in ('legacy', 'white_acceleration'):
            add('five_state', cloud, True, True, noise, True)
        add('robot_localization', cloud, True, True, 'legacy', True, False, '官方 EKF 的几何反馈尚未实现')
    return rows


def choose_variants(options):
    available = variants()
    by_id = {v['id']: v for v in available}
    if options.get('all'):
        return [v for v in available if v['enabled']]
    ids = options.get('variants')
    if ids is not None:
        if not ids or len(ids) != len(set(ids)):
            raise ValueError('请选择不重复的有效融合方案')
        selected = []
        for identifier in ids:
            if identifier not in by_id:
                raise ValueError('未知融合方案: ' + str(identifier))
            if not by_id[identifier]['enabled']:
                raise ValueError(by_id[identifier]['reason'])
            selected.append(by_id[identifier])
        return selected
    estimator = options.get('estimator', 'robot_localization')
    if estimator == 'official': estimator = 'robot_localization'
    noise = {'discrete':'legacy','continuous_psd':'white_acceleration'}.get(options.get('process_noise'), options.get('process_noise', 'legacy'))
    requested = dict(estimator=estimator, cloud=options.get('cloud', 'raw'), filtering=options.get('filtering', True),
                     motion_correction=options.get('motion_correction', False), process_noise=noise, geometry=options.get('geometry', False))
    for row in available:
        if all(row[key] == value for key, value in requested.items()):
            if not row['enabled']: raise ValueError(row['reason'])
            return [row]
    raise ValueError('该组合尚不支持：连续噪声和几何纠偏需要五状态 EKF、运动修正与点云筛选同时启用')
