#!/usr/bin/env python3
"""Post-export offline retention: full lossless clouds, listed cache omission.

Entry points run in a separate process with explicit project/dataset/batch/run.
The original generation manifests remain immutable. A durable per-run plan and
receipt permit interrupted compaction to resume without rebuilding the map.
Only listed derived CDR and verified original PLY counterparts may be removed.
"""
import argparse
import hashlib
import json
import lzma
import os
from pathlib import Path
import shutil
import stat
import subprocess
import sys
import time

# Bound once by the CLI; each queue step runs in its own process.
PROJECT = DATASET = BATCH = None
RUNS = {}
COMPLETE = ('OFFLINE_COMPARISON_COMPLETE', 'OFFLINE_REFINEMENT_COMPLETE_REQUIRES_MAP_REVIEW')
NATIVE_COMPLETE = 'NATIVE_REPLAY_AND_EXPORT_COMPLETE_REQUIRES_VISUAL_REVIEW'
RESERVE = 2 * 1024**3
CHUNK = 4 * 1024**2
META = 'archive_pre_compaction'
RECEIPT = 'compaction_receipt.json'
PACKED = 'compaction_manifest.json.xz'
NOTE = 'COMPACTION.md'
SCHEMA = 'wc-offline-compaction-v1'
_GUARDS = None


def require(value, message):
    if not value:
        raise ValueError(message)


def emit(stage, **values):
    print(json.dumps(dict(stage=stage, **values), ensure_ascii=False), flush=True)


def ordinary(path, root=None, must_exist=True):
    path = Path(path)
    require(path.is_absolute() and '..' not in path.parts, 'absolute ordinary path required: '+str(path))
    for item in (path, *path.parents):
        require(not item.is_symlink(), 'symlink forbidden: '+str(item))
    require(path.resolve(strict=must_exist) == path, 'path normalization mismatch: '+str(path))
    if root is not None:
        require(path.is_relative_to(root) and path != root, 'path escapes allowed root: '+str(path))
    return path


def under(root, relative, must_exist=True):
    relative = Path(relative)
    require(not relative.is_absolute() and '..' not in relative.parts, 'invalid relative path')
    return ordinary(root / relative, root, must_exist)


def read(path):
    ordinary(path)
    with Path(path).open(encoding='utf-8') as stream:
        return json.load(stream)


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(CHUNK), b''):
            h.update(block)
    return h.hexdigest()


def stat_row(path, with_hash=False):
    ordinary(path)
    info = Path(path).stat()
    require(stat.S_ISREG(info.st_mode), 'regular file required: '+str(path))
    row = dict(bytes=info.st_size, allocated_bytes=info.st_blocks*512,
               mtime_ns=info.st_mtime_ns, inode=info.st_ino, device=info.st_dev)
    if with_hash:
        row['sha256'] = digest(path)
    return row


def verify_file(path, row):
    current = stat_row(path)
    require(current['bytes'] == row['bytes'], 'file size changed: '+str(path))
    require(digest(path) == row['sha256'], 'file hash changed: '+str(path))


def sync_dir(path):
    fd = os.open(str(path), os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def atomic_json(path, value):
    ordinary(path, must_exist=False)
    require(path.parent.is_dir(), 'output parent must already exist: '+str(path.parent))
    temporary = path.with_name(path.name+'.new')
    ordinary(temporary, must_exist=False)
    if temporary.exists():
        # Exact owned metadata temporary, serialized by the per-run lock.
        # Fresh planning rejects any preexisting compaction metadata.
        guard();temporary.unlink();sync_dir(path.parent)
    with temporary.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    os.replace(temporary, path)
    sync_dir(path.parent)


def guard():
    global _GUARDS
    from wc_panel.storage import resolve_user_destination
    if _GUARDS is None:
        _GUARDS = [resolve_user_destination(PROJECT, p)[1] for p in (DATASET, BATCH)]
    for bound in _GUARDS:
        bound.check()
    ordinary(BATCH); ordinary(DATASET)
    return _GUARDS


def assert_closed(run):
    """Check owned processes; a surviving unknown permission gap blocks cleanup."""
    gaps = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            if proc.stat().st_uid != os.getuid():
                continue
            name = (proc/'comm').read_text().strip()
            cmd = (proc/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            require(str(run) not in cmd, 'active process references run: '+proc.name+' '+name)
            for link in [proc/'cwd', *list((proc/'fd').iterdir())]:
                try:
                    target = link.resolve(strict=True)
                except FileNotFoundError:
                    continue
                require(not target.is_relative_to(run), 'open run file: '+str(link)+' -> '+str(target))
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError:
            # These login helpers cannot be mapping workers. Report exact gaps;
            # do not disable process checks or waive an unidentified user job.
            try:
                name = (proc/'comm').read_text().strip()
            except FileNotFoundError:
                continue
            require(name in ('(sd-pam)', 'sshd'), 'uninspected user process: '+proc.name+' '+name)
            gaps.append(dict(pid=int(proc.name), comm=name))
    return gaps


def metadata(run):
    report_name = 'refinement_result.json' if (run/'refinement_result.json').is_file() else 'result.json'
    manifest_name = 'refinement_artifacts.json' if report_name.startswith('refinement') else 'artifact_manifest.json'
    report = read(run/report_name)
    require(report['status'] in COMPLETE, 'run is not successfully complete: '+str(run))
    require(report.get('recording_complete') is True, 'original recording incomplete')
    require(report.get('dataset') == str(DATASET), 'wrong recording')
    native = report.get('native_map', {})
    require(native.get('status') == 'NATIVE_MAPS_CLOSED_AND_EXPORTED_REQUIRES_REVIEW', 'native export incomplete')
    require(native.get('cells') and all(c.get('status') == NATIVE_COMPLETE for c in native['cells'].values()),
            'one or more native cells incomplete')
    original = run/META/manifest_name
    manifest = read(original if original.exists() else run/manifest_name)
    require(manifest.get('status') == report['status'], 'manifest/report status mismatch')
    listed = manifest.get('outputs', manifest.get('files'))
    require(isinstance(listed, dict) and listed, 'missing artifact manifest')
    for relative in listed:
        under(run, relative, must_exist=False)
    hashes = report['input_hashes']
    require(hashes == manifest.get('original_inputs', manifest.get('input_hashes')),
            'manifest/report original input hashes differ')
    if report_name.startswith('refinement'):
        marker = read(run/'refinement_complete.json')
        require(marker['status'] == report['status'], 'refine completion marker missing')
        require(marker['result_sha256'] == digest(run/report_name), 'refine result marker differs')
        require(marker['artifact_manifest_sha256'] == digest(run/manifest_name), 'refine artifact marker differs')
    return report, listed, manifest_name, hashes


def excluded(relative):
    p = Path(relative)
    return (p.parts[0] == META or p.name in (RECEIPT, RECEIPT+'.new', PACKED, PACKED+'.partial', NOTE, NOTE+'.new')
            or p.name.endswith(('.ply.xz', '.ply.xz.partial')))


def build_run_plan(number):
    run = ordinary(BATCH/RUNS[number], BATCH)
    require(not (run/RECEIPT).exists(), 'run already has compaction receipt; reuse original plan or verify')
    gaps = assert_closed(run)
    report, listed, manifest_name, hashes = metadata(run)
    cells, native = set(report['cells']), set(report['native_map']['cells'])
    selected, protected = {}, {}
    for path in sorted(run.rglob('*')):
        ordinary(path, run)
        if path.is_dir():
            continue
        relative = path.relative_to(run).as_posix()
        require(not excluded(relative), 'unexpected preexisting compaction file: '+relative)
        parts = Path(relative).parts
        action = None
        if path.suffix == '.cdr' and len(parts) >= 3 and parts[0] in cells and parts[1] in ('input', 'frontend'):
            action = 'OMIT_REGENERATABLE_CDR'
        if path.suffix == '.ply' and len(parts) == 4 and parts[0] == 'native' and parts[1] in native and parts[2] == 'export':
            action = 'LOSSLESS_XZ_PLY'
        row = stat_row(path, with_hash=True)
        if relative in listed:
            require(row['bytes'] == listed[relative]['bytes'] and row['sha256'] == listed[relative]['sha256'],
                    'preexisting artifact manifest mismatch: '+relative)
        elif action:
            raise ValueError('destructive target absent from original manifest: '+relative)
        if action:
            row['action'] = action
            selected[relative] = row
        else:
            protected[relative] = row
    require(not (set(listed)-set(selected)-set(protected)), 'generation manifest references missing artifacts')
    require(any(v['action'] == 'LOSSLESS_XZ_PLY' for v in selected.values()), 'no complete PLY to archive')
    require(any(v['action'] == 'OMIT_REGENERATABLE_CDR' for v in selected.values()), 'no CDR caches to prune')
    for cell in native:
        prefix = 'native/'+cell
        require(prefix+'/slam/rtabmap.db' in protected, 'original DB not protected')
        require(prefix+'/export/native_export_copy.db' in protected, 'export DB not protected')
        require(prefix+'/export/map_3d.pgm' in protected and prefix+'/export/map_3d.yaml' in protected,
                '2D map not protected')
    require(any(p.endswith('/trajectory.csv') for p in protected), 'trajectory not protected')
    return dict(number=number, name=run.name, status=report['status'], manifest_name=manifest_name,
                cells=sorted(cells), native_cells=sorted(native),
                original_inputs=hashes, selected=selected, protected=protected,
                known_process_inspection_gaps=gaps,
                omitted_cdr_count=sum(v['action']=='OMIT_REGENERATABLE_CDR' for v in selected.values()),
                guaranteed_released_allocated_bytes=sum(v['allocated_bytes'] for v in selected.values()
                                                         if v['action']=='OMIT_REGENERATABLE_CDR'))


def plan_command(args):
    guard()
    if args.plan_file.exists():
        previous,sha=load_plan(args)
        require([r['number'] for r in previous['runs']]==args.runs,'existing plan has different run scope')
        emit('EXISTING_PLAN_REUSED',path=str(args.plan_file),sha256=sha)
        return
    runs = []
    for number in args.runs:
        runs.append(build_run_plan(number))
        emit('RUN_PLAN_VERIFIED', run=RUNS[number], cdr_count=runs[-1]['omitted_cdr_count'],
             released_bytes=runs[-1]['guaranteed_released_allocated_bytes'])
    largest = max(v['bytes'] for run in runs for v in run['selected'].values() if v['action']=='LOSSLESS_XZ_PLY')
    free = shutil.disk_usage(BATCH).free
    plan = dict(schema=SCHEMA, batch=str(BATCH), dataset=str(DATASET), created_ns=time.time_ns(), runs=runs,
                free_before_bytes=free, reserve_bytes=RESERVE, max_compression_temporary_estimate_bytes=largest+16*1024**2,
                conservative_cdr_release_bytes=sum(r['guaranteed_released_allocated_bytes'] for r in runs),
                policy='Keep all DB, complete reports, metadata, configurations, logs, maps and trajectories; '
                       'lossless standard XZ for full PLY, omit only listed derived CDR; original recording untouched')
    require(free > RESERVE+largest+16*1024**2, 'insufficient compression temporary space')
    atomic_json(args.plan_file, plan)
    emit('PLAN_SAVED', path=str(args.plan_file), sha256=digest(args.plan_file),
         conservative_cdr_release_bytes=plan['conservative_cdr_release_bytes'], free_before_bytes=free)


def load_plan(args):
    plan = read(args.plan_file)
    require(plan['schema'] == SCHEMA and plan['batch'] == str(BATCH) and plan['dataset'] == str(DATASET), 'wrong plan')
    require(plan['runs'] and len({r['number'] for r in plan['runs']}) == len(plan['runs']), 'duplicate/missing runs')
    for run in plan['runs']:
        require(RUNS.get(run['number']) == run['name'], 'run outside exact allowlist')
        for relative in {**run['selected'], **run['protected']}:
            under(BATCH/run['name'], relative, must_exist=False)
        for relative, row in run['selected'].items():
            parts = Path(relative).parts
            require((row['action']=='OMIT_REGENERATABLE_CDR' and Path(relative).suffix=='.cdr'
                     and len(parts)>=3 and parts[1] in ('input','frontend') and parts[0] in run['cells'])
                    or (row['action']=='LOSSLESS_XZ_PLY' and len(parts)==4 and parts[0]=='native'
                        and parts[1] in run['native_cells'] and parts[2]=='export'
                        and Path(relative).suffix=='.ply'), 'invalid destructive target')
    return plan, digest(args.plan_file)


def verify_inputs(args, plan, plan_hash):
    guard()
    if args.inputs_receipt.exists() and time.time_ns()-read(args.inputs_receipt)['verified_ns'] < 24*3600*10**9:
        check_inputs_receipt(args,plan)
        emit('UNCHANGED_VERIFIED_INPUTS_REUSED',receipt=str(args.inputs_receipt))
        return
    expected = {}
    for run in plan['runs']:
        for name, sha in run['original_inputs'].items():
            require(name not in expected or expected[name]==sha, 'conflicting original input hashes')
            expected[name] = sha
    rows = {}
    for name, sha in expected.items():
        path = ordinary(name)
        # Input hashes also legitimately include installed software/library identities.
        row = stat_row(path, with_hash=True)
        require(row['sha256'] == sha, 'original input changed: '+name)
        rows[name] = row
    require(any(Path(name).is_relative_to(DATASET) and Path(name).suffix=='.db3' for name in rows),
            'raw bag missing from verified inputs')
    receipt = dict(schema=SCHEMA, status='ALL_ORIGINAL_INPUT_HASHES_VERIFIED', plan_sha256=plan_hash,
                   verified_ns=time.time_ns(), original_input_count=len(rows), files=rows)
    atomic_json(args.inputs_receipt, receipt)
    emit('INPUTS_VERIFIED', count=len(rows), receipt=str(args.inputs_receipt))


def check_inputs_receipt(args, plan):
    receipt = read(args.inputs_receipt)
    require(receipt['schema']==SCHEMA and receipt['status']=='ALL_ORIGINAL_INPUT_HASHES_VERIFIED',
            'invalid original input verification receipt')
    # Each experiment group has its own plan and input receipt. In particular, a raw
    # first-group receipt must never mask the later filtered candidate inputs.
    require(receipt['plan_sha256'] == digest(args.plan_file), 'input receipt belongs to another run plan')
    for item in plan['runs']:
        for name,sha in item['original_inputs'].items():
            require(name in receipt['files'] and receipt['files'][name]['sha256']==sha,
                    'original input not covered by reusable receipt: '+name)
    require(0 <= time.time_ns()-receipt['verified_ns'] < 24*3600*10**9, 'input verification receipt too old')
    for name, old in receipt['files'].items():
        now = stat_row(name)
        require(all(now[k]==old[k] for k in ('bytes','mtime_ns','inode','device')), 'input state changed since hash check: '+name)


def decoded_hash(path):
    h, count = hashlib.sha256(), 0
    with lzma.open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(CHUNK), b''):
            count += len(block); h.update(block)
    return h.hexdigest(), count


def compress_ply(run, relative, row):
    source = under(run, relative, must_exist=False)
    target = under(run, relative+'.xz', must_exist=False)
    temporary = under(run, relative+'.xz.partial', must_exist=False)
    if not target.exists():
        require(source.exists(), 'both original and compressed PLY missing')
        verify_file(source, row)
        if temporary.exists():
            try:
                complete = decoded_hash(temporary)==(row['sha256'],row['bytes'])
            except (lzma.LZMAError, EOFError):
                complete = False
            if not complete:
                # Plan and COMPRESSING receipt were persisted before this exact
                # temporary name was created. The verified original still exists.
                state=read(run/RECEIPT)
                require(state['status']=='COMPRESSING', 'partial XZ has no owned compression state')
                guard(); temporary.unlink(); sync_dir(temporary.parent)
        if not temporary.exists():
            require(shutil.disk_usage(run).free > RESERVE+row['bytes']+16*1024**2, 'compression temporary space insufficient')
            with temporary.open('xb') as stream:
                # Standard XZ delta filter, lossless for the complete PLY byte stream.
                subprocess.run(['xz','-T2','--delta=dist=32','--lzma2=preset=3','-c','--',str(source)],
                               stdout=stream, check=True)
                stream.flush(); os.fsync(stream.fileno())
        require(decoded_hash(temporary)==(row['sha256'],row['bytes']), 'partial XZ round-trip failure; original retained')
        if temporary.stat().st_size >= row['bytes']:
            guard(); temporary.unlink(); sync_dir(temporary.parent)
            return dict(original_path=relative,archive_path=relative,original_sha256=row['sha256'],
                        original_bytes=row['bytes'],archive_sha256=row['sha256'],archive_bytes=row['bytes'],
                        codec='identity',full_cloud_lossless=True)
        guard(); os.replace(temporary,target); sync_dir(target.parent)
    require(decoded_hash(target)==(row['sha256'],row['bytes']), 'XZ round-trip failure; original retained')
    compressed = stat_row(target, with_hash=True)
    return dict(original_path=relative, archive_path=relative+'.xz', original_sha256=row['sha256'],
                original_bytes=row['bytes'], archive_sha256=compressed['sha256'], archive_bytes=compressed['bytes'],
                codec='xz_delta32_lzma2_preset3', full_cloud_lossless=True)


def backup_manifest(run, name, row):
    directory = ordinary(run/META, run, must_exist=False)
    if not directory.exists():
        guard(); directory.mkdir(); sync_dir(run)
    path = under(directory, name, must_exist=False)
    if not path.exists():
        temporary=under(directory,name+'.new',must_exist=False)
        if temporary.exists():guard();temporary.unlink()
        with (run/name).open('rb') as source, temporary.open('xb') as output:
            shutil.copyfileobj(source, output, CHUNK); output.flush(); os.fsync(output.fileno())
        verify_file(temporary,row);guard();os.replace(temporary,path)
        sync_dir(directory)
    verify_file(path,row)


def compare_inventory(run, item, pruning_started):
    planned = set(item['protected']) | set(item['selected'])
    current = set()
    for path in run.rglob('*'):
        ordinary(path,run)
        if path.is_file():
            relative=path.relative_to(run).as_posix()
            if relative.endswith(('.ply.xz','.ply.xz.partial')):
                original=relative[:-len('.xz.partial')] if relative.endswith('.partial') else relative[:-3]
                require(original in item['selected'],'unplanned compressed cloud appeared: '+relative)
            if not excluded(relative):
                current.add(relative)
    require(not (current-planned), 'unplanned run files appeared: '+str(sorted(current-planned)[:10]))
    for relative,row in item['protected'].items():
        verify_file(under(run,relative),row)
    for relative,row in item['selected'].items():
        path=under(run,relative,must_exist=False)
        if path.exists():
            verify_file(path,row)
        else:
            require(pruning_started, 'target missing before durable pre-pruning verification: '+relative)
            if row['action']=='LOSSLESS_XZ_PLY':
                require(decoded_hash(under(run,relative+'.xz'))==(row['sha256'],row['bytes']), 'archived PLY differs')


def compact_run(item, plan_hash):
    run = ordinary(BATCH/item['name'], BATCH)
    guard(); gaps = assert_closed(run); metadata(run)
    receipt_path = run/RECEIPT
    state = read(receipt_path) if receipt_path.exists() else None
    if state:
        require(state['plan_sha256']==plan_hash, 'existing run receipt belongs to another plan')
        require(state['status'] in ('COMPRESSING','VERIFIED_READY_TO_PRUNE','PRUNING','COMPLETE'), 'unknown recovery state')
    compare_inventory(run,item,bool(state and state['status']!='COMPRESSING'))
    if state and state['status']=='COMPLETE':
        verify_compacted_run(item,plan_hash); emit('ALREADY_COMPLETE',run=item['name']); return
    backup_manifest(run,item['manifest_name'],item['protected'][item['manifest_name']])
    if not state or state['status']=='COMPRESSING':
        atomic_json(receipt_path,dict(schema=SCHEMA,status='COMPRESSING',plan_sha256=plan_hash,
                    run=item['name'],original_recording_untouched=True,verified_ns=time.time_ns()))
    compressed=[]
    for relative,row in item['selected'].items():
        if row['action']=='LOSSLESS_XZ_PLY':
            compressed.append(compress_ply(run,relative,row))
    guard(); assert_closed(run)
    # All original protected outputs and selected targets were verified before any
    # removal. Publish that fact durably before pruning for interrupted-run recovery.
    state=dict(schema=SCHEMA,status='VERIFIED_READY_TO_PRUNE',plan_sha256=plan_hash,
               run=item['name'],verified_ns=time.time_ns(),compressed_maps=compressed,
               original_manifest_backup=META+'/'+item['manifest_name'],
               known_process_inspection_gaps=gaps,original_recording_untouched=True,
               omitted_cdr_losslessly_archived=False)
    atomic_json(receipt_path,state)
    omitted=[]
    kept_ply={r['original_path'] for r in compressed if r['codec']=='identity'}
    for relative,row in item['selected'].items():
        if relative in kept_ply:continue
        path=under(run,relative,must_exist=False)
        if path.exists():
            # Check every individual destructive target again immediately before
            # unlink; parent containment, ordinary-file checks and mount guard hold.
            current=stat_row(path)
            require(all(current[k]==row[k] for k in ('bytes','mtime_ns','inode','device')),
                    'target changed after full SHA verification: '+relative)
            guard(); path.unlink()
        if row['action']=='OMIT_REGENERATABLE_CDR':
            omitted.append(dict(path=relative,bytes=row['bytes'],sha256=row['sha256'],
                                allocated_bytes=row['allocated_bytes'],reason='REGENERATABLE_DERIVED_CDR_OMITTED'))
    for directory in {under(run,p,must_exist=False).parent for p in item['selected']}:
        sync_dir(directory)
    for relative,row in item['protected'].items():
        verify_file(under(run,relative),row)
    payload=dict(schema=SCHEMA,status='COMPLETE',run=item['name'],plan_sha256=plan_hash,
                 compressed_maps=compressed,omitted_cdr=omitted,protected_files=item['protected'],
                 original_inputs=item['original_inputs'],original_recording_untouched=True,
                 original_manifest_role='GENERATION_TIME_SNAPSHOT_UNCHANGED',
                 regeneration='Re-run from retained original bag, frozen software/configuration and validated candidate. '
                              'Omitted derived CDR bytes are not promised reproducible byte for byte.',
                 completed_ns=time.time_ns())
    manifest_path=run/PACKED
    if not manifest_path.exists():
        temporary=run/(PACKED+'.partial')
        ordinary(temporary,run,must_exist=False)
        if temporary.exists():
            # This exact manifest temporary is owned by the persisted pruning state.
            guard(); temporary.unlink(); sync_dir(run)
        with lzma.open(temporary,'xb',preset=3) as stream:
            stream.write(json.dumps(payload,ensure_ascii=False,indent=2,allow_nan=False).encode('utf-8'))
        with temporary.open('rb') as stream:os.fsync(stream.fileno())
        with lzma.open(temporary,'rt',encoding='utf-8') as stream:
            require(json.load(stream)==payload,'retention manifest round trip differs')
        guard();os.replace(temporary,manifest_path);sync_dir(run)
    else:
        with lzma.open(manifest_path,'rt',encoding='utf-8') as stream:
            previous=json.load(stream)
        require(previous['plan_sha256']==plan_hash and previous['omitted_cdr']==omitted
                and previous['compressed_maps']==compressed,'existing retention manifest differs')
    note=run/NOTE
    if not note.exists():
        text=('# 已完成地图的存储精简\n\n完整 PLY 已无损压缩为 `.ply.xz`，结果对比可直接读取。'
              '二维地图、轨迹、两个数据库、配置、日志和完整报告保持原字节。\n\n'
              '仅省略清单列出的派生 CDR 缓存；这部分不是无损归档，直接重放/续跑这些缓存前需从原录包重算。'
              '原录包和运动修正候选均保留。\n\n'
              '原 artifact_manifest / refinement_artifacts 及阶段完成标记是精简前的生成时证据，保持不改；'
              '当前存储状态以 compaction_manifest.json.xz 和 compaction_receipt.json 为准。\n\n'
              '恢复点云：`xz -dc map_3d_cloud.ply.xz > /有足够空间的新目录/map_3d_cloud.ply`，'
              '解压后应核对压缩清单中的 original_sha256。不要覆盖已有文件。\n')
        temporary=run/(NOTE+'.new')
        ordinary(temporary,run,must_exist=False)
        if temporary.exists():
            guard();temporary.unlink();sync_dir(run)
        with temporary.open('x',encoding='utf-8') as stream:
            stream.write(text);stream.flush();os.fsync(stream.fileno())
        guard();os.replace(temporary,note);sync_dir(run)
    state.update(status='COMPLETE',completed_ns=time.time_ns(),omitted_cdr_count=len(omitted),
                 omitted_cdr_allocated_bytes=sum(v['allocated_bytes'] for v in omitted),
                 manifest=PACKED,manifest_sha256=digest(manifest_path),
                 free_after_bytes=shutil.disk_usage(run).free)
    state['direct_resume_supported']=False
    state['direct_resume_note']='Rebuild omitted derived CDR from the retained original recording before replay/resume.'
    atomic_json(receipt_path,state);guard()
    verify_compacted_run(item,plan_hash)
    emit('RUN_COMPACTION_COMPLETE',run=item['name'],cdr_count=len(omitted),
         omitted_cdr_allocated_bytes=state['omitted_cdr_allocated_bytes'],free_bytes=state['free_after_bytes'])


def verify_compacted_run(item,plan_hash):
    run=ordinary(BATCH/item['name'],BATCH)
    state=read(run/RECEIPT)
    require(state['status']=='COMPLETE' and state['plan_sha256']==plan_hash,'run not compacted with this plan')
    require(digest(run/PACKED)==state['manifest_sha256'],'retention manifest changed')
    with lzma.open(run/PACKED,'rt',encoding='utf-8') as stream:manifest=json.load(stream)
    require(manifest['plan_sha256']==plan_hash and manifest['protected_files']==item['protected'], 'retention plan binding mismatch')
    for relative,row in item['protected'].items():verify_file(under(run,relative),row)
    for relative,row in item['selected'].items():
        saved=next((r for r in manifest['compressed_maps'] if r['original_path']==relative),None)
        if saved and saved['codec']=='identity':
            verify_file(under(run,relative),row);continue
        require(not under(run,relative,must_exist=False).exists(),'uncompacted selected file remains: '+relative)
        if row['action']=='LOSSLESS_XZ_PLY':
            require(decoded_hash(under(run,relative+'.xz'))==(row['sha256'],row['bytes']),'compressed map differs')
    for row in manifest['compressed_maps']:
        require(digest(under(run,row['archive_path']))==row['archive_sha256'],'compressed file hash differs')
    emit('COMPACTED_RUN_VERIFIED',run=item['name'])


def bind(project, dataset, batch, run):
    """Bind explicit CLI scope; never infer a dataset or use a machine UUID."""
    global PROJECT, DATASET, BATCH, RUNS, _GUARDS
    PROJECT,DATASET,BATCH=map(ordinary,(project,dataset,batch))
    run=ordinary(run,BATCH)
    require(run.parent==BATCH,'run must be a direct child of this batch')
    require(not BATCH.is_relative_to(DATASET) and not DATASET.is_relative_to(BATCH),
            'compaction output overlaps original recording')
    RUNS={run.name:run.name};_GUARDS=None
    guard()
    # tree_entries rejects nested mounts, special files and links without traversal.
    from wc_panel.storage import tree_entries
    tree_entries(run)
    return run


def preflight(project, dataset, output_root, *, rate=5., geometry=False):
    from wc_panel.storage import resolve_user_destination, tree_size
    source,sg=resolve_user_destination(project,dataset)
    destination,dg=resolve_user_destination(project,output_root)
    require(source!=destination and not source.is_relative_to(destination)
            and not destination.is_relative_to(source),'output overlaps original recording')
    require(0<float(rate)<=10,'invalid input rate')
    bag=source/'bag'
    raw_bytes=tree_size(bag)
    # A disclosed conservative estimate, not a promise of peak memory/disk usage.
    working=max(2*1024**3,int(raw_bytes*(2. if geometry else 1.5)*max(1.,float(rate)/5.)))
    required=RESERVE+working;free=shutil.disk_usage(destination).free
    sg.check();dg.check()
    require(free>=required,'融合前可用空间不足：估计需 {:.2f} GiB（含 2 GiB 余量），现有 {:.2f} GiB'.format(
            required/1024**3,free/1024**3))
    emit('CAPACITY_PREFLIGHT_PASS',free_bytes=free,estimated_required_bytes=required,
         estimate_only=True,reserve_bytes=RESERVE)


def refresh_index():
    """Publish small map metadata even when full numerical reports exceed 4 MiB."""
    from wc_panel.storage import atomic_json
    maps=[];selection=None
    panel_metadata=read(BATCH/'panel_result.json') if (BATCH/'panel_result.json').exists() else {}
    groups={Path(t['output']).name:t for t in panel_metadata.get('plan',{}).get('tasks',[])
            if t.get('variant') and t.get('output')}
    old=read(BATCH/'RESULT_SUMMARY.json') if (BATCH/'RESULT_SUMMARY.json').exists() else {}
    for number,name in RUNS.items():
        run=BATCH/name;report,_,_,_=metadata(run)
        selection=report.get('source_selection',selection)
        group=groups.get(name,{'id':name,'label':name})
        for cell in report['native_map']['cells']:
            export=run/'native'/cell/'export'
            paths=[export/'map_3d.pgm',export/'map_3d.yaml',export/'native_export_copy.db',
                   run/'native'/cell/'slam/rtabmap.db',run/cell/'trajectory.csv']
            require(all(p.is_file() and p.stat().st_size>0 for p in paths),'result index missing final files')
            require(any(p.is_file() for p in (export/'map_3d_cloud.ply',export/'map_3d_cloud.ply.xz')),
                    'result index missing full cloud')
            maps.append(dict(id=group['id'] if len(report['native_map']['cells'])==1 else group['id']+'_'+cell,
                             run=name,cell=cell,title=group['label'],status='AVAILABLE',storage_status='COMPACTED'))
    replaced=set(RUNS.values())
    maps=[m for m in old.get('maps',[]) if m.get('run') not in replaced]+maps
    maps.sort(key=lambda item:(item['run'],item['cell']))
    atomic_json(BATCH/'RESULT_SUMMARY.json',dict(schema_version=1,status='AVAILABLE',dataset=str(DATASET),
                maps=maps,source_selection=selection or old.get('source_selection'),completed_map_count=len(maps),
                planned_map_count=panel_metadata.get('plan',{}).get('comparison_count',len(maps)),updated_at_ns=time.time_ns()))
    from wc_panel.catalog import results
    rows=results(BATCH)
    for entry in maps:
        row=next((r for r in rows if r['run']==entry['run'] and r['cell']==entry['cell']),None)
        require(row and row['clouds'] and row['images'] and row['trajectories'],'viewer cannot discover compacted map')
    emit('RESULT_INDEX_VERIFIED',map_count=len(maps))


def execute(project,dataset,batch,run,*,verify_only=False):
    from types import SimpleNamespace
    from wc_panel.storage import atomic_json
    run=bind(project,dataset,batch,run)
    import fcntl
    from wc_runtime.project_paths import shared_lock_root
    lock=shared_lock_root()/('compaction-'+hashlib.sha256(str(run).encode()).hexdigest()[:24]+'.lock')
    # Separate per-run lock: the queue already owns its project offline lock.
    with lock.open('a') as stream:
        fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
        directory=ordinary(BATCH/'_compaction',BATCH,must_exist=False)
        directory.mkdir(exist_ok=True)
        directory=ordinary(directory/run.name,BATCH,must_exist=False);directory.mkdir(exist_ok=True)
        args=SimpleNamespace(plan_file=directory/'plan.json',inputs_receipt=directory/'inputs.json',runs=[run.name])
        if verify_only:
            plan,plan_hash=load_plan(args)
            for item in plan['runs']:verify_compacted_run(item,plan_hash)
            refresh_index();return
        try:
            plan_command(args);plan,plan_hash=load_plan(args)
            verify_inputs(args,plan,plan_hash);check_inputs_receipt(args,plan)
            for item in plan['runs']:compact_run(item,plan_hash)
            check_inputs_receipt(args,plan);refresh_index()
            atomic_json(directory/'status.json',dict(status='COMPLETE',run=str(run),updated_ns=time.time_ns()))
        except BaseException as error:
            guard()
            atomic_json(directory/'status.json',dict(status='INTERRUPTED' if isinstance(error,KeyboardInterrupt) else 'FAILED',
                        run=str(run),error=str(error),updated_ns=time.time_ns(),maps_retained=True))
            raise


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=('compact','verify','preflight'))
    parser.add_argument('--project-root',type=Path,required=True)
    parser.add_argument('--dataset',type=Path,required=True)
    parser.add_argument('--batch',type=Path)
    parser.add_argument('--run',type=Path)
    parser.add_argument('--output-root',type=Path)
    parser.add_argument('--input-rate-hz',type=float,default=5.)
    parser.add_argument('--geometry',action='store_true')
    args=parser.parse_args(argv)
    if args.command=='preflight':
        require(args.output_root is not None,'preflight requires output root')
        preflight(args.project_root,args.dataset,args.output_root,rate=args.input_rate_hz,geometry=args.geometry)
    else:
        require(args.batch is not None and args.run is not None,'explicit batch and run required')
        execute(args.project_root,args.dataset,args.batch,args.run,verify_only=args.command=='verify')


if __name__=='__main__':
    try:main()
    except Exception as error:
        emit('COMPACTION_STOPPED',error=type(error).__name__+': '+str(error))
        raise


