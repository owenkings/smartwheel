"""Controlled 50 cm board, multiple static poses, offline relative calibration."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import secrets

import numpy as np

from .environment_calibration import render_report
from .environment_registration import solve_environment
from .small_board_diagnostic import diagnose_small_board
from .small_board_planes import controlled_board_planes
from .board_interior_patch import controlled_interior_patch


def _software_sources():
    here = Path(__file__).resolve().parent
    return {name: hashlib.sha256((here / name).read_bytes()).hexdigest() for name in (
        'small_board_calibration.py', 'small_board_planes.py', 'small_board_diagnostic.py',
        'environment_planes.py', 'environment_registration.py', 'environment_calibration.py',
        'plane_capture.py', 'board_interior_patch.py')}


def analyse_board_captures(captures, *, same_board_face_confirmed,
                           selection_x_band_m=(0.6, 1.6), selection_provenance=None,
                           interior_rois=None, interior_regions_confirmed=False):
    """Pure computation on trusted inputs; CLI binds real bags via load/assert_sources.

    In-memory source_mode is a caller declaration, not a file authenticity check.
    """
    if same_board_face_confirmed is not True:
        raise ValueError('Operator must confirm the same single physical board face')
    if not 1 <= len(captures) <= 8:
        raise ValueError('Expected 1..8 complete, distinct captures')
    if interior_regions_confirmed is True and interior_rois is None:
        raise ValueError('Confirmed interior regions require explicit interior_rois')
    if interior_rois is not None and (not isinstance(interior_rois, dict) or
            set(interior_rois) != {c.get('capture_id') for c in captures}):
        raise ValueError('Interior-region configuration must name every and only supplied capture')
    software = _software_sources()
    records, seen, modes = [], set(), set()
    identities = None
    for source in captures:
        cid = source.get('capture_id')
        if not isinstance(cid, str) or not cid or cid in seen:
            raise ValueError('Missing or repeated capture identity')
        seen.add(cid)
        if source.get('role') not in ('train', 'holdout'):
            raise ValueError('Each capture must have an explicit train/holdout role')
        modes.add(source.get('source_mode'))
        if len(modes) != 1 or not modes <= {'real', 'synthetic'} or source.get('units') != 'm' or source.get(
                'coordinate_conventions') != {'left': 'FLU', 'right': 'FLU'}:
            raise ValueError('Expected consistently labelled real or synthetic FLU points in metres')
        ids = source.get('sensor_ids')
        if not isinstance(ids, dict) or set(ids) != {'left', 'right'} or not all(
                isinstance(v, str) and v for v in ids.values()) or ids['left'] == ids['right']:
            raise ValueError('Missing or invalid distinct sensor identities')
        if identities is not None and ids != identities:
            raise ValueError('Sensor identities changed between captures')
        identities = dict(ids)
        width, height = source['width'], source['height']
        record = {'capture_id': cid, 'role': source['role'], 'source_mode': source['source_mode'],
                  'sensor_ids': ids, 'width': width, 'height': height,
                  'provenance': source['provenance'], 'clouds': source['clouds'],
                  'controlled_board_diagnostics': {}}
        for side in ('left', 'right'):
            cloud = np.asarray(source['clouds'][side], dtype=float)
            frames = np.asarray(source['frames'][side], dtype=float)
            if interior_rois is not None:
                regions = interior_rois[cid]
                if not isinstance(regions, dict) or set(regions) != {'left', 'right'}:
                    raise ValueError('Each capture needs explicit left and right interior regions')
                patch = controlled_interior_patch(cloud, frames=frames, width=width, height=height,
                    roi_xyxy=regions[side], operator_confirmed=interior_regions_confirmed)
                record[side] = patch
                record['controlled_board_diagnostics'][side] = patch['diagnostic']
                record['selection_method'] = 'OPERATOR_CONFIRMED_BOARD_INTERIOR_PATCH'
                continue
            diag = diagnose_small_board(cloud, frames=frames, width=width, height=height,
                selection_x_band_m=selection_x_band_m,
                selection_provenance=selection_provenance)
            # The adapter receives actual full-point support rather than an
            # inferred area or inferred outlier count from summary metrics.
            interior = np.asarray(diag['interior_indices'], dtype=int)
            raw = np.asarray(diag['raw_candidate_indices'], dtype=int)
            adapter_input = dict(diag)
            adapter_input.update(interior_points=cloud[interior], raw_candidate_points=cloud[raw],
                fixed_interior_frames=frames[:, interior, :],
                identity_assumption={'basis': 'OPERATOR_SAME_SINGLE_WOOD_BOARD_FACE',
                                     'automatic_identity_verified': False})
            record[side] = controlled_board_planes(adapter_input)
            record['controlled_board_diagnostics'][side] = diag
        records.append(record)
    result = solve_environment(records)
    if software != _software_sources():
        raise ValueError('Calibration sources changed during computation')
    level = 'REAL_BAG' if modes == {'real'} else 'SYNTHETIC'
    result['verification_level'] = level
    return {'schema_version': 1, 'kind': 'offline_controlled_small_board_plane_calibration',
        'created_utc': datetime.now(timezone.utc).isoformat(), 'verification_level': level,
        'source_mode': next(iter(modes)), 'software_sha256': software, 'units': 'm',
        'coordinate_convention': 'FLU', 'transform_direction': 'p_left = R @ p_right + t',
        'captures': records, 'result': result, 'live_eligible': False,
        'limitations': [
            '同一块木板同一面由操作者确认；自动选区不证明物理板身份。',
            '每个姿态单独静止录制；不同姿态的板面法向共同提供独立约束。',
            '同一录制内的重复帧只用于稳定性检查；验证录制不得参与拟合。',
            '小板门限是针对有限目标支持的试验规则，不能宣称毫米级绝对精度。',
            '环境大面筛选门限保持原样；整板自动分支拒绝裁切；内部选区分支需要操作者另行确认板面身份。',
            '内部选区仅证明局部观测可用，不能证明整块木板平直；箱子、外环和全图残差仍保留诊断。',
            '相对外参不确定SDK点云原点在外壳内或轮椅基准中的位置。',
        ]}


def export_board_report(report, destination, *, guard):
    destination = Path(destination)
    if any(p.is_symlink() or getattr(p, 'is_junction', lambda: False)()
           for p in (destination, *destination.parents)):
        raise ValueError('Output path contains a link')
    raw = json.dumps(report, ensure_ascii=False, allow_nan=False, separators=(',', ':')).encode('utf-8')
    html = render_report(report).replace('双雷达环境平面标定', '双雷达单板多姿态标定').replace(
        '双雷达 · 自动环境平面标定', '双雷达 · 50厘米木板多姿态标定').replace(
        '墙面与地面自动分割、内部区域拟合、平面配对、约束检查。仅生成离线候选。',
        '同一木板自动选区、固定内部点检查、多姿态平面联合求解。仅生成离线候选。')
    if any(c.get('selection_method') == 'OPERATOR_CONFIRMED_BOARD_INTERIOR_PATCH'
           for c in report['captures']):
        html = html.replace('同一木板自动选区、固定内部点检查、多姿态平面联合求解。仅生成离线候选。',
            '操作者确认板内粗区域，程序拟合和检查固定内部点，再联合不同姿态的平面。仅生成离线候选。').replace(
            '自动平面分区', '操作者确认的板内区域：合格局部平面').replace(
            '自动提取：左 ', '通过全部检查的局部平面：左 ')
    guard.check()
    destination.mkdir(parents=True, exist_ok=False)
    with (destination / 'board_calibration.json').open('xb') as stream:
        stream.write(raw)
    guard.check()
    with (destination / 'report.html').open('x', encoding='utf-8') as stream:
        stream.write(html)
    receipt = {'status': 'OFFLINE_BOARD_EXPORT_COMPLETE', 'report_sha256': hashlib.sha256(raw).hexdigest(),
        'candidate_status': report['result']['status'], 'verification_level': report['verification_level'],
        'live_eligible': False, 'output_directory': str(destination),
        'report_html': str(destination / 'report.html'), 'reasons': report['result']['reasons']}
    guard.check()
    with (destination / 'complete.json').open('x', encoding='utf-8') as stream:
        json.dump(receipt, stream, ensure_ascii=False, indent=2)
    return receipt


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', required=True)
    parser.add_argument('--train', action='append', default=[])
    parser.add_argument('--holdout', action='append', default=[])
    parser.add_argument('--same-board-face-confirmed', action='store_true')
    parser.add_argument('--forward-x-min', type=float, default=0.6)
    parser.add_argument('--forward-x-max', type=float, default=1.6)
    parser.add_argument('--selection-note', default='Original diagnostic foreground range band')
    parser.add_argument('--interior-rois', help='JSON capture_id -> left/right inclusive pixel rectangles')
    parser.add_argument('--interior-regions-confirmed', action='store_true')
    args = parser.parse_args(argv)
    if not args.train or len(args.train) + len(args.holdout) > 8:
        parser.error('At least one train capture; at most eight captures in total')
    if not args.same_board_face_confirmed:
        parser.error('The operator must confirm the same board face')
    from .plane_capture import load_capture, assert_sources
    from wc_runtime.storage_policy import StoragePolicy
    root = Path(args.project_root).absolute()
    policy = StoragePolicy(root)
    policy.check()
    config_before = dict(policy.configuration_files)
    roi_path = Path(args.interior_rois).absolute() if args.interior_rois else None
    if args.interior_regions_confirmed and roi_path is None:
        parser.error('--interior-regions-confirmed requires --interior-rois')
    roi_before = roi_path.read_bytes() if roi_path else None
    regions = json.loads(roi_before.decode('utf-8')) if roi_before is not None else None
    captures = []
    for role, paths in (('train', args.train), ('holdout', args.holdout)):
        for path in paths:
            capture = load_capture(root, path)
            capture['role'] = role
            captures.append(capture)
    report = analyse_board_captures(captures, same_board_face_confirmed=args.same_board_face_confirmed,
        selection_x_band_m=(args.forward_x_min, args.forward_x_max),
        selection_provenance={'basis': 'EXPLICIT_CLI_FOREGROUND_RANGE_BAND',
                              'note': args.selection_note},
        interior_rois=regions, interior_regions_confirmed=args.interior_regions_confirmed)
    if roi_path:
        if roi_path.read_bytes() != roi_before:
            raise ValueError('Interior-region configuration changed during analysis')
        report['interior_region_configuration'] = {'path': str(roi_path),
            'sha256': hashlib.sha256(roi_before).hexdigest(), 'regions': regions,
            'operator_confirmed': args.interior_regions_confirmed,
            'interpretation': 'Coarse face identity only; no manual point correspondences'}
    for capture in captures:
        assert_sources(capture)
    current = StoragePolicy(root)
    if current.configuration_files != config_before:
        raise ValueError('Storage configuration changed during analysis')
    current.check()
    session = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_') + secrets.token_hex(6)
    output = current.resolve('data/calibration/small_board_planes') / session
    receipt = export_board_report(report, output, guard=current)
    print(json.dumps(receipt, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
