"""Offline environment-plane calibration; source-bound inputs, diagnostic exports only."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import secrets

import numpy as np

from .environment_planes import extract_planes
from .environment_registration import solve_environment


def _software_sources():
    directory = Path(__file__).resolve().parent
    return {name: hashlib.sha256((directory / name).read_bytes()).hexdigest()
            for name in ('environment_planes.py', 'environment_registration.py', 'environment_calibration.py')}


def analyse_captures(captures):
    """Analyse complete captures already decoded by plane_capture.load_capture.

    A scene can supply several independent *directions*, but remains one capture
    for train/holdout independence. No per-frame pose or TF is estimated here.
    """
    software = _software_sources()
    if not 1 <= len(captures) <= 8:
        raise ValueError('需要 1–8 份完整录制')
    identities, modes, seen = [], set(), set()
    records = []
    for index, source in enumerate(captures):
        identity = source.get('capture_id')
        if not isinstance(identity, str) or not identity or identity in seen:
            raise ValueError('录制身份缺失或重复；同包不能同时作为训练与验证')
        seen.add(identity)
        if source.get('role') not in ('train', 'holdout'):
            raise ValueError('每份录制必须明确用于训练或独立验证')
        if source.get('units') != 'm' or source.get('coordinate_conventions') != {'left': 'FLU', 'right': 'FLU'}:
            raise ValueError('只接受明确标记为米、FLU 的左右原始点云')
        sensor_ids = source.get('sensor_ids')
        if not isinstance(sensor_ids, dict) or set(sensor_ids) != {'left', 'right'} or not all(
                isinstance(v, str) and v for v in sensor_ids.values()) or sensor_ids['left'] == sensor_ids['right']:
            raise ValueError('左右雷达身份无效')
        identities.append(sensor_ids)
        modes.add(source.get('source_mode'))
        if any(ids != identities[0] for ids in identities) or len(modes) != 1 or not modes <= {'real', 'synthetic'}:
            raise ValueError('不同录制的设备身份或实机/合成来源不一致')
        width, height = source['width'], source['height']
        if type(width) is not int or type(height) is not int or not 0 < width * height <= 76800:
            raise ValueError('有序点云尺寸无效或超过预算')
        record = {'capture_id': identity, 'role': source['role'], 'sensor_ids': sensor_ids,
                  'source_mode': source['source_mode'], 'width': width, 'height': height,
                  'provenance': source.get('provenance', {}), 'clouds': source['clouds']}
        for side in ('left', 'right'):
            record[side] = extract_planes(source['clouds'][side], width=width, height=height,
                                         frames=source.get('frames', {}).get(side), seed=index * 2 + (side == 'right'))
        records.append(record)
    result = solve_environment(records)
    if software != _software_sources():
        raise ValueError('计算期间算法源码改变，拒绝导出混合版本结果')
    if modes == {'synthetic'}:
        result['verification_level'] = 'SYNTHETIC'
    else:
        result['verification_level'] = 'REAL_BAG'
    return {'schema_version': 1, 'kind': 'offline_environment_plane_calibration',
            'created_utc': datetime.now(timezone.utc).isoformat(),
            'verification_level': result['verification_level'], 'source_mode': next(iter(modes)),
            'software_sha256': software,
            'transform_direction': 'p_left = R_left_right @ p_right + t_left_right',
            'units': 'm', 'coordinate_convention': 'FLU',
            'captures': records, 'result': result, 'live_eligible': False,
            'limitations': ['环境物理静止由操作者确认；到达时间配对不证明曝光同步。',
                           '同一录制的多帧只检查重复性，不作为独立验证。',
                           '默认机械初值只限制对应搜索，结果仍需独立验证。',
                           '相对外参不确定点云原点相对外壳或轮轴的位置。',
                           '筛选门限为实验起点，不是测量精度证明。']}


def _json(report):
    return json.dumps(report, ensure_ascii=False, allow_nan=False, separators=(',', ':'))


def render_report(report):
    """Self-contained, no-network HTML; canvas overlays use raw SDK pixel indices."""
    data = _json(report).replace('<', '\\u003c').replace('>', '\\u003e').replace('&', '\\u0026')
    return '''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta http-equiv="Content-Security-Policy" content="default-src 'none'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; img-src data:; connect-src 'none'; base-uri 'none'">
<title>双雷达环境平面标定</title>
<style>body{margin:28px auto;max-width:1200px;padding:0 20px;background:#101923;color:#edf4fa;font:16px/1.6 system-ui}h1{font-size:27px;margin-bottom:4px}.muted{color:#a9bbc9}.panel{background:#192735;border:1px solid #34475a;border-radius:12px;padding:18px;margin:18px 0}.grid{display:grid;grid-template-columns:1fr 1fr;gap:16px}.views{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}canvas{width:100%;background:#0b131c;border:1px solid #42566a;border-radius:8px}select{padding:8px;background:#192735;color:#fff;border:1px solid #65798b}table{width:100%;border-collapse:collapse;font-size:14px}td,th{text-align:left;padding:8px;border-bottom:1px solid #34475a}pre{overflow:auto;font-size:13px;max-height:400px;white-space:pre-wrap;overflow-wrap:anywhere}.tag{color:#ffd07c}details{margin-top:14px}label{display:block;margin-bottom:12px}@media(max-width:800px){.grid,.views{grid-template-columns:1fr}body{margin:16px auto}}</style>
<h1>双雷达 · 自动环境平面标定</h1><p class="muted">墙面与地面自动分割、内部区域拟合、平面配对、约束检查。仅生成离线候选。</p>
<div class="panel"><strong id="status"></strong><p id="summary"></p><p class="tag" id="scope"></p><ul id="limitations"></ul></div>
<div class="panel"><label>查看录制 <select id="capture"></select></label><p id="counts"></p><div class="grid"><div><strong>左雷达：自动平面分区</strong><canvas id="left" width="800" height="300"></canvas></div><div><strong>右雷达：自动平面分区</strong><canvas id="right" width="800" height="300"></canvas></div></div><p class="muted">各侧颜色按各自平面编号分配；对应关系见表格。灰色为未采用的有效点，黑色为无效点。</p><table><thead><tr><th>雷达 / 平面</th><th>有效点</th><th>二维跨度 m</th><th>残差 P95 mm</th><th>逐帧检查</th></tr></thead><tbody id="planes"></tbody></table></div>
<div class="panel"><strong>点云叠加：左侧青色，右侧紫色</strong><p id="overlay"></p><div class="views"><div>俯视 XY<canvas id="xy" width="500" height="360"></canvas></div><div>侧视 XZ<canvas id="xz" width="500" height="360"></canvas></div><div>前视 YZ<canvas id="yz" width="500" height="360"></canvas></div></div></div>
<div class="panel"><strong>外参、匹配与拒绝原因</strong><pre id="result"></pre><details><summary>录制来源与平面诊断</summary><pre id="diagnostics"></pre></details></div>
<script id="report" type="application/json">''' + data + '''</script><script>
const report=JSON.parse(document.getElementById('report').textContent), result=report.result;
const $=id=>document.getElementById(id), palette=['#f7bf68','#71cba2','#65adf4','#c995e8','#f18c9c','#a9d670','#81d8db','#ed9d70'];
$('status').textContent='结果：'+result.status;
$('summary').textContent='验证等级：'+report.verification_level+'；完整录制 '+report.captures.length+' 份。';
$('scope').textContent=report.source_mode==='synthetic'?'当前为合成演示，不能作为实机标定通过。':'当前为原始录包离线检查；候选未写入运行外参。';
for(const line of report.limitations){const li=document.createElement('li');li.textContent=line;$('limitations').append(li)}
$('result').textContent=JSON.stringify(result,null,2);
report.captures.forEach((c,i)=>{const o=document.createElement('option');o.value=i;o.textContent=c.capture_id+' · '+(c.role==='train'?'参与求解':'独立验证');$('capture').append(o)});
function transform(){const v=result.transform_right_to_left||result.T_left_right||result.initial_T_left_right; if(Array.isArray(v)&&v.length===4&&v.every(r=>Array.isArray(r)&&r.length===4))return v; return null;}
function paintMap(c,side){const canvas=$(side),ctx=canvas.getContext('2d');ctx.fillStyle='#05090d';ctx.fillRect(0,0,canvas.width,canvas.height);const cloud=c.clouds[side], dx=canvas.width/c.width,dy=canvas.height/c.height;for(let i=0;i<cloud.length;i++){if(cloud[i].every(Number.isFinite)){ctx.fillStyle='#344454';ctx.fillRect((i%c.width)*dx,Math.floor(i/c.width)*dy,dx+.2,dy+.2)}}(c[side].planes||[]).forEach((p,k)=>{ctx.fillStyle=palette[k%palette.length];for(const i of p.indices||[])ctx.fillRect((i%c.width)*dx,Math.floor(i/c.width)*dy,dx+.2,dy+.2)});}
function scatter(c,id,axes,T){const canvas=$(id),ctx=canvas.getContext('2d'),points=[];ctx.clearRect(0,0,canvas.width,canvas.height);for(const side of ['left','right']){for(let i=0;i<c.clouds[side].length;i+=2){let p=c.clouds[side][i];if(!p.every(Number.isFinite))continue;if(side==='right'&&T)p=T.slice(0,3).map(row=>row[0]*p[0]+row[1]*p[1]+row[2]*p[2]+row[3]);points.push([p[axes[0]],p[axes[1]],side])}}if(!points.length)return;let xmin=Infinity,xmax=-Infinity,ymin=Infinity,ymax=-Infinity;for(const p of points){xmin=Math.min(xmin,p[0]);xmax=Math.max(xmax,p[0]);ymin=Math.min(ymin,p[1]);ymax=Math.max(ymax,p[1])}const s=Math.min((canvas.width-36)/Math.max(xmax-xmin,.1),(canvas.height-36)/Math.max(ymax-ymin,.1)), ox=(canvas.width-s*(xmax-xmin))/2,oy=(canvas.height-s*(ymax-ymin))/2;ctx.globalAlpha=.55;for(const [x,y,side]of points){ctx.fillStyle=side==='left'?'#50d8e7':'#eb81d1';ctx.fillRect(ox+(x-xmin)*s,canvas.height-oy-(y-ymin)*s,2,2)}ctx.globalAlpha=1;ctx.fillStyle='#b8cbd9';ctx.font='12px sans-serif';ctx.fillText('同一米制比例 · '+Math.max(xmax-xmin,ymax-ymin).toFixed(1)+' m 范围',12,18);}
function draw(){const c=report.captures[Number($('capture').value)],T=transform();$('counts').textContent='自动提取：左 '+(c.left.planes||[]).length+' 面，右 '+(c.right.planes||[]).length+' 面。';$('planes').replaceChildren();for(const side of ['left','right']){paintMap(c,side);(c[side].planes||[]).forEach(p=>{const tr=document.createElement('tr'), values=[(side==='left'?'左':'右')+' / '+p.id,p.point_count,(p.span_m||[]).map(x=>x.toFixed(2)).join(' × '),p.p95_m==null?'—':(1000*p.p95_m).toFixed(1),p.temporal?.status||'NOT_EVALUATED'];for(const v of values){const td=document.createElement('td');td.textContent=v;tr.append(td)}$('planes').append(tr)})}$('overlay').textContent=T?'右点云使用本次估计变换；请结合结果状态和独立验证判断。':'尚无可用变换；图中左右均为各自原生坐标，仅用于诊断。';scatter(c,'xy',[0,1],T);scatter(c,'xz',[0,2],T);scatter(c,'yz',[1,2],T);$('diagnostics').textContent=JSON.stringify({...c,clouds:undefined},null,2);}
$('capture').addEventListener('change',draw);draw();
</script></html>'''


def export_report(report, destination, *, guard=None):
    destination = Path(destination)
    if any(p.is_symlink() or getattr(p, 'is_junction', lambda: False)()
           for p in (destination, *destination.parents)):
        raise ValueError('输出路径不可包含链接')
    raw = _json(report).encode('utf-8')
    document = render_report(report)
    if guard is not None:
        guard.check()
    destination.mkdir(parents=True, exist_ok=False)
    (destination / 'environment_calibration.json').write_bytes(raw)
    if guard is not None:
        guard.check()
    (destination / 'report.html').write_text(document, encoding='utf-8')
    receipt = {'status': 'OFFLINE_EXPORT_COMPLETE', 'report_sha256': hashlib.sha256(raw).hexdigest(),
               'candidate_status': report['result']['status'], 'live_eligible': False,
               'verification_level': report['verification_level']}
    if guard is not None:
        guard.check()
    (destination / 'complete.json').write_text(json.dumps(receipt, indent=2), encoding='utf-8')
    return receipt


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', required=True)
    parser.add_argument('--train', action='append', default=[], help='已完成训练录制的目录；可重复')
    parser.add_argument('--holdout', action='append', default=[], help='另外录制的验证目录；可重复')
    args = parser.parse_args(argv)
    if not args.train or len(args.train) + len(args.holdout) > 8:
        parser.error('至少一份训练录制，训练与验证合计不超过八份')
    from .plane_capture import load_capture, assert_sources
    from wc_runtime.storage_policy import StoragePolicy
    policy = StoragePolicy(Path(args.project_root).absolute())
    policy.check()
    config_before = dict(policy.configuration_files)
    captures = []
    for role, paths in [('train', args.train), ('holdout', args.holdout)]:
        for path in paths:
            capture = load_capture(args.project_root, path)
            capture['role'] = role
            captures.append(capture)
    report = analyse_captures(captures)
    for capture in captures:
        assert_sources(capture)
    current = StoragePolicy(Path(args.project_root).absolute())
    if current.configuration_files != config_before:
        raise ValueError('计算期间存储配置改变，拒绝导出')
    current.check()
    session = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_') + secrets.token_hex(6)
    output = current.resolve('data/calibration/environment_planes') / session
    receipt = export_report(report, output, guard=current)
    current.check()
    print(json.dumps({**receipt, 'output_directory': str(output), 'report_html': str(output / 'report.html'),
                      'reasons': report['result'].get('reasons', [])}, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
