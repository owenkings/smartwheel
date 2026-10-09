"""Offline, source-bound visualization of unvalidated single-scene candidates.

Never publishes TF, edits production configuration, or claims a geometric fix.
HTML has no network dependencies; its decimated points remain a private artifact.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from .core import CalibrationError, _hash, validate_transform


def preview_payload(data, result, max_points=3500):
    if (result.get('kind') != 'single_scene_exploration' or
            result.get('status') != 'EXPLORATION_ONLY' or result.get('live_eligible') is not False):
        raise CalibrationError('only explicitly unvalidated exploration results may be previewed')
    if result.get('input_hash') != _hash(data):
        raise CalibrationError('candidate result does not belong to these exact input observations')
    if type(max_points) is not int or not 20 <= max_points <= 10000:
        raise CalibrationError('bounded preview point count 20..10000 required')
    if len(data.get('training', [])) != 1 or data.get('validation') != [] or data.get('units') != 'm':
        raise CalibrationError('one metre-unit scene with no claimed holdout required')
    scene = data['training'][0]
    points = {}
    counts = {}
    for side in ('left', 'right'):
        cloud = np.asarray(scene[side], dtype=float)
        if cloud.ndim != 2 or cloud.shape[1] != 3 or not len(cloud) or not np.isfinite(cloud).all():
            raise CalibrationError('finite nonempty Nx3 cloud required')
        counts[side] = len(cloud)
        indices = np.linspace(0, len(cloud)-1, min(len(cloud), max_points), dtype=int)
        points[side] = cloud[indices].tolist()
    candidates = []
    for row in result.get('candidates', []):
        transform = validate_transform(row['candidate_T_left_right'])
        candidates.append({'id': str(row['id']), 'matrix': transform.tolist(),
                           'score_m': row['score_m'], 'geometry_usable': row['geometry_usable'],
                           'rejection_reasons': row['rejection_reasons'],
                           'diagnostics': row['diagnostics']})
    if len(candidates) > 12:
        raise CalibrationError('preview candidate budget exceeded')
    return {'status': 'EXPLORATION_ONLY', 'source_mode': data['source_mode'],
            'scene_id': scene['id'], 'sensor_ids': data['sensor_ids'], 'units': 'm',
            'left': points['left'], 'right': points['right'], 'original_counts': counts,
            'candidates': candidates, 'ambiguity': result.get('ambiguity'),
            'time_quality': scene.get('time_quality', 'UNVALIDATED'),
            'input_hash': result['input_hash'], 'live_eligible': False}


_HTML = r'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>双雷达候选对齐 · 未验证</title>
<style>
body{margin:0;padding:24px;background:#101821;color:#e9f0f7;font:16px system-ui,sans-serif}
h1{font-size:26px;margin:0 0 8px}.note{color:#ffc66a;line-height:1.6}
button,select{font:inherit;padding:8px;margin:6px 8px 6px 0;background:#263645;color:white;border:1px solid #617080;border-radius:5px}
canvas{display:block;width:100%;height:62vh;min-height:350px;background:#0b1118;border:1px solid #435367;touch-action:none}
pre{white-space:pre-wrap;word-break:break-word;background:#1b2835;padding:16px;border-radius:6px;font-size:13px}
.red{color:#ff8070}.blue{color:#66c5ff}.help{color:#b9c7d5}#details{max-height:340px;overflow:auto}
</style><h1>双雷达单场景候选对齐</h1>
<div class="note">EXPLORATION_ONLY · 所有候选均未验证，不是地图，也未写入 TF 或正式外参。</div>
<p id="identity"></p><label>查看 <select id="choice"><option value="raw">原坐标直接叠放（未标定）</option></select></label>
<button data-view="oblique">斜视</button><button data-view="top">俯视 XY</button><button data-view="front">前视 YZ</button><button data-view="side">侧视 XZ</button>
<p><span class="red">● 左雷达</span>　<span class="blue">● 右雷达</span>　<span id="summary"></span></p>
<canvas id="cloud"></canvas><p class="help">拖动旋转，滚轮缩放，Shift + 拖动平移。候选模式将右点转换到左雷达坐标系；原坐标模式只作比较，不表示两传感器共址。</p>
<pre id="matrix"></pre><details><summary>候选诊断、歧义与来源</summary><pre id="details"></pre></details>
<script id="payload" type="application/json">__PAYLOAD__</script>
<script>
'use strict';
const d=JSON.parse(document.getElementById('payload').textContent),c=document.getElementById('cloud'),ctx=c.getContext('2d'),sel=document.getElementById('choice');
const identity=document.getElementById('identity'); identity.textContent=d.scene_id+' · '+d.source_mode+' · '+d.time_quality+' · 单位：米';
d.candidates.forEach((v,i)=>{const o=document.createElement('option');o.value=String(i);o.textContent='候选 '+v.id+(v.geometry_usable?' · 局部几何检查通过':' · 存在几何问题');sel.appendChild(o)});
let points=[],center=[0,0,0],extent=1,yaw=-.75,pitch=.6,zoom=1,pan=[0,0],drag=null;
function transformed(p,t){return [0,1,2].map(i=>t[i][0]*p[0]+t[i][1]*p[1]+t[i][2]*p[2]+t[i][3]);}
function update(){const row=sel.value==='raw'?null:d.candidates[Number(sel.value)];
const right=row?d.right.map(p=>transformed(p,row.matrix)):d.right;
points=d.left.map(p=>[p,'#ff8070']).concat(right.map(p=>[p,'#66c5ff']));
const low=[0,1,2].map(i=>Math.min(...points.map(p=>p[0][i]))),high=[0,1,2].map(i=>Math.max(...points.map(p=>p[0][i])));
center=low.map((v,i)=>(v+high[i])/2);extent=Math.max(...low.map((v,i)=>high[i]-v),.1);zoom=1;pan=[0,0];
document.getElementById('summary').textContent=row?'双向匹配评分 '+Number(row.score_m).toFixed(4)+' m（不是测距精度）':'未应用安装变换';
document.getElementById('matrix').textContent=row?'候选 T_left_right：p_left = R × p_right + t\n'+row.matrix.map(r=>r.map(v=>v.toFixed(6)).join('  ')).join('\n'):'原坐标比较；未声明任何安装变换。';
document.getElementById('details').textContent=JSON.stringify({candidate:row,ambiguity:d.ambiguity,source_mode:d.source_mode,input_hash:d.input_hash,original_counts:d.original_counts,displayed_counts:{left:d.left.length,right:d.right.length},live_eligible:false},null,2);draw();}
function draw(){const w=c.clientWidth,h=c.clientHeight,ratio=window.devicePixelRatio||1;c.width=w*ratio;c.height=h*ratio;ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,w,h);const scale=Math.min(w,h)*.78/extent*zoom;
const co=Math.cos(yaw),si=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch);
function project(p){const [x,y,z]=p.map((v,i)=>v-center[i]),a=co*x-si*y,b=si*x+co*y;return [w/2+pan[0]+a*scale,h/2+pan[1]-(cp*z-sp*b)*scale,sp*z+cp*b];}
const rendered=points.map(p=>[project(p[0]),p[1]]).sort((a,b)=>a[0][2]-b[0][2]);ctx.globalAlpha=.8;
for(const [p,color] of rendered){ctx.fillStyle=color;ctx.fillRect(p[0],p[1],2,2);}ctx.globalAlpha=1;
ctx.strokeStyle='#8697aa';ctx.lineWidth=2;const x0=25,y0=h-30,len=scale;ctx.beginPath();ctx.moveTo(x0,y0);ctx.lineTo(x0+len,y0);ctx.stroke();ctx.fillStyle='#d5dfeb';ctx.font='13px sans-serif';ctx.fillText('1 m（当前投影比例）',x0,y0-8);}
sel.addEventListener('change',update);window.addEventListener('resize',draw);
c.addEventListener('pointerdown',e=>{drag=[e.clientX,e.clientY];c.setPointerCapture(e.pointerId)});
c.addEventListener('pointerup',()=>drag=null);c.addEventListener('pointercancel',()=>drag=null);
c.addEventListener('pointermove',e=>{if(!drag)return;const dx=e.clientX-drag[0],dy=e.clientY-drag[1];drag=[e.clientX,e.clientY];if(e.shiftKey){pan[0]+=dx;pan[1]+=dy;}else{yaw+=dx*.006;pitch=Math.max(-Math.PI/2,Math.min(Math.PI/2,pitch+dy*.006));}draw();});
c.addEventListener('wheel',e=>{e.preventDefault();zoom=Math.max(.1,Math.min(20,zoom*Math.exp(-e.deltaY*.001)));draw();},{passive:false});
for(const b of document.querySelectorAll('[data-view]'))b.addEventListener('click',()=>{const v=b.dataset.view;[yaw,pitch]=v==='top'?[0,Math.PI/2]:v==='front'?[-Math.PI/2,0]:v==='side'?[0,0]:[-.75,.6];draw();});
if(d.candidates.length)sel.value='0';update();
</script></html>'''


def render_html(payload):
    encoded = json.dumps(payload, ensure_ascii=False, separators=(',', ':'), allow_nan=False)
    return _HTML.replace('__PAYLOAD__', encoded.replace('<', '\\u003c').replace('>', '\\u003e').replace('&', '\\u0026'))


def render_png(payload, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    left, right = np.asarray(payload['left']), np.asarray(payload['right'])
    row = payload['candidates'][0] if payload['candidates'] else None
    transformed = right
    if row:
        t = np.asarray(row['matrix'])
        transformed = right @ t[:3, :3].T+t[:3, 3]
    fig, axes = plt.subplots(2, 3, figsize=(14, 8), facecolor='white')
    all_points = np.vstack((left, right, transformed))
    for column, (i, j, name) in enumerate(((0, 1, 'XY (top)'), (0, 2, 'XZ (side)'), (1, 2, 'YZ (front)'))):
        for line, cloud in enumerate((right, transformed)):
            ax = axes[line, column]
            ax.scatter(left[:, i], left[:, j], s=1, c='#d44836', alpha=.65, label='Left')
            ax.scatter(cloud[:, i], cloud[:, j], s=1, c='#007acc', alpha=.65, label='Right')
            ax.set_title(('Unaligned native coordinates' if line == 0 else 'Best numerical candidate - UNVALIDATED')+'\n'+name, fontsize=10)
            for index, setter in ((i, ax.set_xlim), (j, ax.set_ylim)):
                low, high = all_points[:, index].min(), all_points[:, index].max()
                margin = max(.05, .03*(high-low));setter(low-margin, high+margin)
            ax.set_aspect('equal', adjustable='box');ax.grid(alpha=.2)
            ax.set_xlabel('XYZ'[i]+' (m)');ax.set_ylabel('XYZ'[j]+' (m)')
    axes[0, 0].legend(markerscale=5)
    fig.suptitle('EXPLORATION ONLY | '+payload['source_mode']+' | '+str(payload['scene_id'])+'\nNo independent scene validation; not a map or installed extrinsics', fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, .94]);fig.savefig(output, dpi=130);plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True)
    parser.add_argument('--result', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--max-points', type=int, default=3500)
    args = parser.parse_args(argv)
    data = json.loads(Path(args.input).read_text())
    result = json.loads(Path(args.result).read_text())
    payload = preview_payload(data, result, args.max_points)
    target = Path(args.output_dir).absolute()
    if target.is_symlink() or any(p.is_symlink() for p in target.parents):
        raise CalibrationError('output path must not traverse symlinks')
    target.mkdir(parents=True, exist_ok=False)
    (target/'preview.html').write_text(render_html(payload), encoding='utf-8')
    render_png(payload, target/'preview.png')
    (target/'manifest.json').write_text(json.dumps({'status':'EXPLORATION_ONLY','input_hash':payload['input_hash'],
        'result_hash':_hash(result),'displayed_points':{s:len(payload[s]) for s in ('left','right')},
        'candidate_count':len(payload['candidates']),'live_eligible':False},indent=2))
    print(json.dumps({'output_dir':str(target),'status':'EXPLORATION_ONLY'}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
