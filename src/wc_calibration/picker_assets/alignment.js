/* Independent offline alignment page. Pure geometry is also exported for Node tests. */
(function(root,factory){
  "use strict";
  const api=factory();
  if(typeof module==="object"&&module.exports)module.exports=api;
  if(root&&root.document){root.AlignmentUI=api;const start=()=>{if(root.document.getElementById("alignment-app"))api.start(root.PickerUI);};if(root.document.readyState==="loading")root.document.addEventListener("DOMContentLoaded",start);else start();}
})(typeof window!=="undefined"?window:null,function(){
  "use strict";
  const RAD=Math.PI/180, finite=v=>typeof v==="number"&&Number.isFinite(v);
  const clone=m=>m.map(row=>row.slice());
  const need=(ok,message)=>{if(!ok)throw new Error(message);};
  function validateMatrix(m){
    need(Array.isArray(m)&&m.length===4&&m.every(r=>Array.isArray(r)&&r.length===4&&r.every(finite)),"变换必须是有限的 4×4 矩阵。");
    need(m.slice(0,3).every(row=>Math.abs(row[3])<=1e6),"平移超出本页面数值显示范围（±1000000 米），未应用。");
    need(m[3].every((v,i)=>Math.abs(v-(i===3?1:0))<1e-8),"变换矩阵末行无效。");
    for(let i=0;i<3;i++)for(let j=0;j<3;j++)need(Math.abs(m[i].slice(0,3).reduce((s,v,k)=>s+v*m[j][k],0)-(i===j?1:0))<1e-5,"旋转不是正交刚体旋转。");
    const d=m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])-m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])+m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
    need(Math.abs(d-1)<1e-5,"变换不能含反射或缩放。");return m;
  }
  function poseToMatrix(p){
    need(Array.isArray(p)&&p.length===6&&p.every(finite),"六个参数都必须为有限数值；平移单位米，角度单位度。");
    const [r,b,y]=p.slice(3).map(v=>(v%360)*RAD),cr=Math.cos(r),sr=Math.sin(r),cp=Math.cos(b),sp=Math.sin(b),cy=Math.cos(y),sy=Math.sin(y);
    return [[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,p[0]],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,p[1]],[-sp,cp*sr,cp*cr,p[2]],[0,0,0,1]];
  }
  function matrixToPose(m){
    validateMatrix(m);
    const cp=Math.hypot(m[0][0],m[1][0]),p=Math.atan2(-m[2][0],cp);
    // At pitch +/-90 deg, choose roll=0 and preserve the equivalent rigid rotation.
    const r=cp>1e-8?Math.atan2(m[2][1],m[2][2]):0;
    const y=cp>1e-8?Math.atan2(m[1][0],m[0][0]):Math.atan2(-m[0][1],m[1][1]);
    return [m[0][3],m[1][3],m[2][3],r/RAD,p/RAD,y/RAD];
  }
  function dragTranslation(m,dx,dy,view){
    validateMatrix(m);need([dx,dy,view.yaw,view.pitch,view.scale].every(finite)&&view.scale>0,"拖动视图参数无效。");
    const cy=Math.cos(view.yaw),sy=Math.sin(view.yaw),cp=Math.cos(view.pitch),sp=Math.sin(view.pitch);
    const right=[-sy,-cy,0],down=[sp*cy,-sp*sy,-cp],out=clone(m);
    for(let i=0;i<3;i++)out[i][3]+=(dx*right[i]+dy*down[i])/view.scale;
    return validateMatrix(out);
  }
  function sameMatrix(a,b){return !!a&&!!b&&a.every((r,i)=>r.every((v,j)=>Math.abs(v-b[i][j])<1e-10));}
  // Axes for changing one displayed Euler parameter, expressed in the left frame.
  // They pass through t (the transformed right sensor origin), not the cloud centre.
  function rotationGuide(m,index,sign=1){
    need(Number.isInteger(index)&&index>=3&&index<=5&&(sign===1||sign===-1),"旋转方向参数无效。");
    const p=matrixToPose(m),pitch=p[4]*RAD,yaw=p[5]*RAD;
    const axis=index===3?[Math.cos(yaw)*Math.cos(pitch),Math.sin(yaw)*Math.cos(pitch),-Math.sin(pitch)]:index===4?[-Math.sin(yaw),Math.cos(yaw),0]:[0,0,1];
    const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
    const seed=axis.map((_,i)=>i===axis.reduce((best,v,j)=>Math.abs(v)<Math.abs(axis[best])?j:best,0)?1:0);
    const raw=cross(axis,seed),length=Math.hypot(...raw),u=raw.map(x=>x/length),v=cross(axis,u);
    const arc=Array.from({length:49},(_,i)=>{const angle=sign*(-35+i*270/48)*RAD;return u.map((x,j)=>x*Math.cos(angle)+v[j]*Math.sin(angle));});
    return {axis,arc,pivot:p.slice(0,3)};
  }
  function checkedResult(value,operation){
    need(value&&value.live_eligible===false&&typeof value.status==="string","服务端必须明确返回未获正式使用资格的候选。");
    validateMatrix(value.initial_T_left_right);
    if(operation==="refine")validateMatrix(value.refined_T_left_right);
    return value;
  }
  function pixelColor(left,right){
    const bg=[8,17,29],l=[71,217,229],r=[255,181,107];
    if(left&&right)return bg.map((v,i)=>Math.round(.2*v+.4*l[i]+.4*r[i]));
    return bg.map((v,i)=>Math.round(left ? .2*v+.8*l[i] : right ? .2*v+.8*r[i] : v));
  }
  function metricText(m){
    if(!m||typeof m!=="object")return "未提供几何指标。";
    const rows=[["source_points","右云参与点数","count"],["target_points","左云参与点数","count"],["max_correspondence_distance_m","最近邻门限","m"],["forward_inlier_count","右→左门限内点数","count"],["reverse_inlier_count","左→右门限内点数","count"],["forward_overlap_fraction","右→左重叠比例","fraction"],["reverse_overlap_fraction","左→右重叠比例","fraction"],["nn_rmse_m","右→左最近邻 RMSE","distance"],["nn_median_m","右→左最近邻中位数","distance"],["nn_p95_m","右→左最近邻 P95","distance"]];
    return rows.map(([key,label,unit])=>{const value=m[key];let text="未提供";if(value===null)text=unit==="distance"?"无内点":"未提供";else if(finite(value))text=unit==="fraction"?(value*100).toFixed(2)+" %":unit==="distance"?(value*1000).toFixed(2)+" mm":unit==="m"?value.toFixed(3)+" m":String(value);return label+"："+text;}).join("\n");
  }
  class PoseState{
    constructor(initial){this.initial=clone(validateMatrix(initial));this.current=clone(initial);this.before=null;this.refined=null;}
    set(m){this.current=clone(validateMatrix(m));}
    reset(){this.set(this.initial);}
    acceptRefinement(result,submitted){checkedResult(result,"refine");this.before=clone(validateMatrix(submitted));this.refined=clone(result.refined_T_left_right);this.set(this.refined);}
  }
  function start(ui){
    const $=id=>document.getElementById(id),canvas=$("alignment-canvas"),ctx=canvas.getContext("2d",{alpha:false});
    const ids=["x","y","z","roll","pitch","yaw"],inputs=ids.map(id=>$("pose-"+id));
    let scene=null,model=null,view=null,busy=false,mode="view",queued=false,gesture=null,lastEvaluated=null,shownInputs=[],guideIndex=3,guideSign=1,level=new ui.LevelDisplay();
    const guideCanvas=$("rotation-guide"),guideContext=guideCanvas.getContext("2d");
    const error=message=>{$("alignment-error").textContent=String(message);$("alignment-error").hidden=false;};
    const clear=()=>{$("alignment-error").hidden=true;};
    const json=v=>JSON.stringify(v===undefined?null:v,null,2).slice(0,50000);
    const notice=message=>{$("pose-state").textContent=message;};
    function controls(){
      $("pose-controls").disabled=!model||busy;
      for(const id of ["save-manual","refine","reset-initial"])$(id).disabled=!model||busy;
      $("show-before").disabled=!model||!model.before||busy;$("show-refined").disabled=!model||!model.refined||busy;
      $("adjust-mode").disabled=!model||busy;
      $("display-frame").disabled=!model||busy||!level.available;
      if(lastEvaluated&&model)$("result-dirty").hidden=sameMatrix(lastEvaluated,model.current);
    }
    function sync(){
      const p=matrixToPose(level.toDisplay(model.current));inputs.forEach((input,i)=>{input.value=Number(p[i].toFixed(i<3?9:7));input.setCustomValidity("");});
      shownInputs=inputs.map(input=>input.value);
      $("current-matrix").textContent=json(model.current);
      $("display-frame").value=level.enabled?"level":"raw";
      $("transform-title").textContent=level.enabled?"右云在水平参考中的姿态":"右 → 左变换";
      $("pose-formula").textContent=(level.enabled?"p_level = R · p_right + t":"p_left = R · p_right + t")+"\nR = Rz(yaw) · Ry(pitch) · Rx(roll)";
      $("pose-axis-note").textContent=(level.enabled?"平移沿水平参考轴：X 参考前向、Y 参考左向、Z 参考上向。":"平移沿左雷达 FLU 轴：X 前、Y 左、Z 上。")+"旋转围绕右雷达原点；不拟合缩放。";
      $("level-status").textContent=level.enabled?"已应用水平参考（候选）。左云和右云共同调平；方向图与参数使用同一参考。":level.available?"当前显示原始左雷达坐标；可切换水平参考。":"当前场景未提供水平参考，显示原始左雷达坐标。";
      $("frame-note").textContent=(level.enabled?"当前显示使用水平参考候选；IMU 与安装关系仍需独立精度验证。":"当前参考是左雷达原始坐标，青色左云保留原姿态。")+"切换显示参考保留配准结果；保存、ICP 与下方矩阵始终使用原始右 → 左外参。";
      controls();schedule();
    }
    function applyInputs(){
      // Merely saving or dragging must not round a full-precision bootstrap/ICP matrix.
      if(inputs.every((input,i)=>input.value===shownInputs[i]))return true;
      try{const p=inputs.map(input=>{need(input.value.trim()!==""&&finite(input.valueAsNumber),"请完整填写六个有限参数。");return input.valueAsNumber;});model.set(level.toRaw(poseToMatrix(p)));sync();clear();notice("当前为手动调整姿态，尚未对当前姿态计算几何指标。");return true;}
      catch(e){error(e.message);return false;}
    }
    function clouds(){return {left:level.cloud(scene.clouds.left),right:level.cloud(scene.clouds.right,model.current)};}
    function fit(preset){if(!model)return;if(preset==="oblique"){view.yaw=.25;view.pitch=-.4;}if(preset==="front"){view.yaw=0;view.pitch=0;}if(preset==="top"){view.yaw=0;view.pitch=-Math.PI/2;}if(preset==="side"){view.yaw=Math.PI/2;view.pitch=0;}const c=clouds();view=ui.fitView(c.left.concat(c.right),canvas.width,canvas.height,view);schedule();}
    function resize(){const rect=canvas.getBoundingClientRect(),w=Math.max(1,Math.min(2048,Math.round(rect.width))),h=Math.max(1,Math.min(1200,Math.round(rect.height)));if(canvas.width!==w||canvas.height!==h){canvas.width=w;canvas.height=h;if(model)fit("fit");}}
    function schedule(){if(queued||!model)return;queued=true;requestAnimationFrame(()=>{queued=false;try{draw();}catch(e){error(e.message);}});}
    function draw(){
      const w=canvas.width,h=canvas.height,c=clouds(),radius=Number($("point-radius").value);
      const layers={};for(const side of ["left","right"])if($("show-"+side).checked)layers[side]=ui.rasterize(c[side].map(p=>ui.projectPoint(p,view,w,h)),w,h,radius);
      const colors=[pixelColor(false,false),pixelColor(true,false),pixelColor(false,true),pixelColor(true,true)],frame=ctx.createImageData(w,h);
      for(let i=0;i<w*h;i++){const mask=(layers.left&&layers.left.indices[i]>=0?1:0)+(layers.right&&layers.right.indices[i]>=0?2:0),color=colors[mask],offset=i*4;frame.data[offset]=color[0];frame.data[offset+1]=color[1];frame.data[offset+2]=color[2];frame.data[offset+3]=255;}
      ctx.putImageData(frame,0,0);ctx.fillStyle="#aac1d3";ctx.font="12px sans-serif";ctx.fillText((level.enabled?"水平参考（候选）":"左雷达原始坐标")+" · 视图宽度约 "+(w/view.scale).toFixed(2)+" m",12,h-12);
      drawGuide();
    }
    function selectGuide(index,sign=1){guideIndex=index;guideSign=sign;document.querySelectorAll("[data-guide-axis]").forEach(b=>b.setAttribute("aria-pressed",String(Number(b.dataset.guideAxis)===index)));schedule();}
    function drawGuide(){
      if(!guideContext)return;
      const g=rotationGuide(level.toDisplay(model.current),guideIndex,guideSign),c=guideContext,w=guideCanvas.width,h=guideCanvas.height;
      c.clearRect(0,0,w,h);const localView={...view,center:[0,0,0],pan:[0,4],scale:52};
      const project=xyz=>ui.projectPoint({id:0,xyz},localView,w,h),origin=project([0,0,0]);
      function line(points,color,width=2,arrow=false){c.beginPath();c.strokeStyle=color;c.lineWidth=width;points.forEach((p,i)=>{if(i)c.lineTo(p.x,p.y);else c.moveTo(p.x,p.y);});c.stroke();if(arrow){const b=points[points.length-1],a=points[Math.max(0,points.length-3)],angle=Math.atan2(b.y-a.y,b.x-a.x);if(Math.hypot(b.x-a.x,b.y-a.y)>1){c.beginPath();c.moveTo(b.x,b.y);c.lineTo(b.x-9*Math.cos(angle-.45),b.y-9*Math.sin(angle-.45));c.moveTo(b.x,b.y);c.lineTo(b.x-9*Math.cos(angle+.45),b.y-9*Math.sin(angle+.45));c.stroke();}}}
      const colors=["#ff8f96","#8bdaa0","#83baff"],labels=["X 前","Y 左","Z 上"];
      c.font="12px system-ui";c.textAlign="center";
      for(let i=0;i<3;i++){
        const end=project([0,0,0].map((_,j)=>j===i?1.22:0));line([origin,end],colors[i],1.5,true);c.fillStyle=colors[i];
        if(Math.hypot(end.x-origin.x,end.y-origin.y)<8){const nearer=end.depth<origin.depth;c.fillText((nearer?"⊙ ":"⊗ ")+labels[i],origin.x,origin.y+20);}
        else c.fillText(labels[i],end.x+(end.x-origin.x)*.14,end.y+(end.y-origin.y)*.14+4);
      }
      const ring=g.arc.map(p=>project(p.map(x=>x*.78)));line(ring,"#ffc77f",3,true);
      const axisEnd=project(g.axis.map(x=>x*1.02));line([origin,axisEnd],"#ffffff",3,true);
      c.fillStyle="#ffffff";c.beginPath();c.arc(origin.x,origin.y,3,0,Math.PI*2);c.fill();
      const name=ids[guideIndex][0].toUpperCase()+ids[guideIndex].slice(1),positive=guideSign>0;
      $("guide-title").textContent=name+" "+(positive?"＋ 增大":"− 减小")+"方向";
      const f=x=>Math.abs(x)<.0005?"0":x.toFixed(3);
      $("guide-axis-vector").textContent="当前轴（"+(level.enabled?"水平候选":"原始左系")+"）["+g.axis.map(f).join(", ")+"]";
      const tips=["零姿态参考：＋左侧抬起、右侧降低；− 相反。","零姿态参考：＋前端下俯、后端抬起；− 相反。","零姿态参考：＋朝左转；− 朝右转。"];
      $("guide-description").textContent=tips[guideIndex-3]+" 橙色箭头按当前角度与视角绘制。";
      const xs=ring.map(p=>p.x),ys=ring.map(p=>p.y);
      $("guide-edge-on").hidden=Math.min(Math.max(...xs)-Math.min(...xs),Math.max(...ys)-Math.min(...ys))>14;
    }
    function position(event){const r=canvas.getBoundingClientRect();return [(event.clientX-r.left)*canvas.width/r.width,(event.clientY-r.top)*canvas.height/r.height];}
    function setMode(next){mode=next;gesture=null;canvas.classList.toggle("adjust",next==="adjust");for(const name of ["view","adjust"]){$(name+"-mode").classList.toggle("active",name===next);$(name+"-mode").setAttribute("aria-pressed",String(name===next));}$("gesture-help").textContent=next==="adjust"?"左键拖动整片右云，在当前视图平面内平移；左云固定。旋转使用右侧 RPY 输入。滚轮只缩放视图。":"左键拖动旋转视角，Shift + 拖动平移视角，滚轮在光标处缩放。";}
    canvas.addEventListener("pointerdown",event=>{if(!model||event.button!==0||(mode==="adjust"&&busy))return;if(mode==="adjust"&&!$("show-right").checked){error("右云已隐藏，请先勾选右云再调整。");return;}if(mode==="adjust"&&!applyInputs())return;const [x,y]=position(event);gesture={id:event.pointerId,x,y,lastX:x,lastY:y,movement:0,shift:event.shiftKey};canvas.setPointerCapture(event.pointerId);canvas.focus();});
    canvas.addEventListener("pointermove",event=>{const g=gesture;if(!g||g.id!==event.pointerId)return;const [x,y]=position(event),dx=x-g.lastX,dy=y-g.lastY;g.movement=Math.max(g.movement,Math.hypot(x-g.x,y-g.y));if(g.movement>=3){if(mode==="adjust"&&!busy){model.set(level.toRaw(dragTranslation(level.toDisplay(model.current),dx,dy,view)));sync();notice("已在视图平面内平移右云；当前姿态尚未重新计算。");}else if(mode==="view"){if(g.shift||event.shiftKey){view.pan[0]+=dx;view.pan[1]+=dy;}else{view.yaw=(view.yaw+dx*.007)%(2*Math.PI);view.pitch=Math.max(-Math.PI/2,Math.min(Math.PI/2,view.pitch+dy*.007));}schedule();}}g.lastX=x;g.lastY=y;});
    canvas.addEventListener("pointerup",event=>{if(gesture&&gesture.id===event.pointerId){gesture=null;if(canvas.hasPointerCapture(event.pointerId))canvas.releasePointerCapture(event.pointerId);}});
    for(const type of ["pointercancel","lostpointercapture"])canvas.addEventListener(type,()=>{gesture=null;});
    canvas.addEventListener("wheel",event=>{if(!model)return;event.preventDefault();const [x,y]=position(event),old=view.scale;view.scale=Math.max(1e-6,Math.min(1e7,old*Math.exp(-Math.max(-200,Math.min(200,event.deltaY))*.0015)));const ratio=view.scale/old;view.pan[0]=(view.pan[0]-(x-canvas.width/2))*ratio+x-canvas.width/2;view.pan[1]=(view.pan[1]-(y-canvas.height/2))*ratio+y-canvas.height/2;schedule();},{passive:false});
    function showResult(result,exportDir,evaluated){
      lastEvaluated=clone(evaluated);$("alignment-result").hidden=false;
      $("result-status").textContent=result.status+(/REJECT|FAIL|UNVALIDATED|DEGENERATE|NOT_CONVERGED|INSUFFICIENT/.test(result.status)?" · 未通过，请继续调整并复核":" · 仅探索，未正式验证");
      const reasons=result.reasons===undefined?result.rejection_reasons:result.reasons;
      $("result-reasons").textContent=reasons&&(!Array.isArray(reasons)||reasons.length)?"服务端原因／限制："+json(reasons):"未报告拒绝原因也不等于标定通过；需检查重叠、退化与独立数据。";
      $("initial-metrics").textContent=metricText(result.initial_metrics);$("final-metrics").textContent=metricText(result.final_metrics);$("transform-change").textContent=json(result.transform_change);
      $("result-policy").textContent=json({policy:result.policy,history:result.history,live_eligible:result.live_eligible});$("saved-path").textContent="已保存到服务端："+exportDir;controls();
    }
    async function request(operation){
      if(!model||busy||!applyInputs())return;const submitted=clone(model.current);busy=true;gesture=null;clear();controls();$("alignment-request").textContent=operation==="refine"?"正在以当前右→左姿态运行 ICP，请等待…":"正在保存手动候选…";
      try{const response=await fetch("/api/alignment",{method:"POST",credentials:"same-origin",headers:{"Content-Type":"application/json","X-Picker-Token":scene.token},body:JSON.stringify({input_hash:scene.input_hash,scene_id:scene.scene_id,initial_T_left_right:submitted,operation})});let payload;try{payload=await response.json();}catch(_){throw new Error("服务返回非 JSON，当前手动姿态已保留。");}need(response.ok&&!payload.error,typeof payload.error==="string"?payload.error:"服务请求失败，HTTP "+response.status);need(typeof payload.export_dir==="string"&&payload.export_dir.length>0,"服务未返回保存目录。");const result=checkedResult(payload.result,operation);if(operation==="refine"){model.acceptRefinement(result,submitted);sync();notice("显示最近一次 ICP 结果；可切换前后比较。这仍是探索候选。");}else notice("手动姿态已保存；尚未因保存而通过标定。");showResult(result,payload.export_dir,model.current);$("alignment-request").textContent="已保存新版本；正式配置和 TF 未改变。";}
      catch(e){error(e.message);$("alignment-request").textContent="请求未确认完成，当前右云变换已保留；请查看错误后重试。";}
      finally{busy=false;controls();}
    }
    $("view-mode").addEventListener("click",()=>setMode("view"));$("adjust-mode").addEventListener("click",()=>setMode("adjust"));
    $("display-frame").addEventListener("change",()=>{
      const selection=$("display-frame");
      if(!model||busy){selection.value=level.enabled?"level":"raw";return;}
      if(inputs.some((input,i)=>input.value!==shownInputs[i])){selection.value=level.enabled?"level":"raw";error("请先应用当前输入值，再切换显示参考。");return;}
      try{level.setEnabled(selection.value==="level");gesture=null;sync();fit("fit");clear();notice("显示参考已切换；观察角度与当前右 → 左外参保持不变。");}catch(e){selection.value=level.enabled?"level":"raw";error(e.message);}
    });
    document.querySelectorAll("[data-guide-axis]").forEach(b=>b.addEventListener("click",()=>selectGuide(Number(b.dataset.guideAxis))));
    document.querySelectorAll("[data-nudge]").forEach(b=>{if(Number(b.dataset.nudge)>=3)for(const event of ["pointerenter","focus"])b.addEventListener(event,()=>selectGuide(Number(b.dataset.nudge),Number(b.dataset.sign)));});
    inputs.forEach((input,index)=>{if(index>=3)input.addEventListener("focus",()=>selectGuide(index));});
    $("guide-face-axis").addEventListener("click",()=>{if(!model)return;const a=rotationGuide(level.toDisplay(model.current),guideIndex).axis,camera=[Math.cos(view.yaw)*Math.cos(view.pitch),-Math.sin(view.yaw)*Math.cos(view.pitch),Math.sin(view.pitch)],sign=a.reduce((s,x,i)=>s+x*camera[i],0)>=0?1:-1;view.yaw=Math.atan2(-sign*a[1],sign*a[0]);view.pitch=Math.asin(Math.max(-1,Math.min(1,sign*a[2])));fit("fit");});
    for(const id of ["show-left","show-right","point-radius"])$(id).addEventListener("change",schedule);
    document.querySelectorAll("[data-view]").forEach(button=>button.addEventListener("click",()=>fit(button.dataset.view)));
    $("apply-pose").addEventListener("click",applyInputs);
    document.querySelectorAll("[data-nudge]").forEach(button=>button.addEventListener("click",()=>{if(!model||busy||!applyInputs())return;try{const axis=Number(button.dataset.nudge),step=$(axis<3?"translation-step":"rotation-step");need(step.value.trim()!==""&&finite(step.valueAsNumber)&&step.valueAsNumber>0&&step.checkValidity(),"微调步长必须为输入范围内的正数。");const p=matrixToPose(level.toDisplay(model.current));p[axis]+=Number(button.dataset.sign)*step.valueAsNumber;model.set(level.toRaw(poseToMatrix(p)));sync();notice("当前为手动微调姿态，尚未重新计算。");}catch(e){error(e.message);}}));
    $("reset-initial").addEventListener("click",()=>{if(!model||busy)return;model.reset();sync();notice("已恢复页面加载时的初值；不代表该初值正确。");});
    $("show-before").addEventListener("click",()=>{if(model&&model.before&&!busy){model.set(model.before);sync();notice("显示最近一次 ICP 的输入姿态。");}});
    $("show-refined").addEventListener("click",()=>{if(model&&model.refined&&!busy){model.set(model.refined);sync();notice("显示最近一次 ICP 输出，仍未正式验证。");}});
    $("save-manual").addEventListener("click",()=>request("save_manual"));$("refine").addEventListener("click",()=>request("refine"));
    try{need(ui&&ctx,"点云绘图库或 Canvas 不可用。");view=ui.newView();resize();if(typeof ResizeObserver!=="undefined")new ResizeObserver(resize).observe(canvas);else window.addEventListener("resize",resize);}
    catch(e){error(e.message);return;}
    fetch("/api/alignment-bootstrap",{credentials:"same-origin",cache:"no-store"}).then(async response=>{const payload=await response.json();need(response.ok&&!payload.error,typeof payload.error==="string"?payload.error:"无法读取初值场景。");return payload;}).then(payload=>{
      scene=ui.validateScene(payload.scene);level=new ui.LevelDisplay(scene.level_reference);model=new PoseState(payload.initial_T_left_right);
      $("alignment-source").textContent=(scene.source_mode==="real"?"真实冻结点云":"合成点云，仅软件测试")+" · "+scene.scene_id+" · 左 "+scene.clouds.left.length+" 点 / 右 "+scene.clouds.right.length+" 点 · 米 / FLU";
      $("alignment-provenance").textContent=json({scene_id:scene.scene_id,input_hash:scene.input_hash,sensor_ids:scene.sensor_ids,units:scene.units,coordinate_conventions:scene.coordinate_conventions,time_quality:scene.time_quality,initial_source:payload.initial_source,level_reference:scene.level_reference});
      $("alignment-time").hidden=false;$("alignment-time").textContent=scene.time_quality==="NON_SIMULTANEOUS_STATIC_SCENE_ASSUMPTION"?"左右点云分时采集。只有两次采集之间轮椅、安装和场景均未移动时，才能探索安装外参；这不是时间同步验证。":"时间质量："+scene.time_quality+"。此工具不验证同步、测距尺度或安装稳定性。";
      let loadedSaved=false;
      if(payload.saved_candidate&&payload.saved_candidate.result){try{const r=payload.saved_candidate.result;checkedResult(r,r.refined_T_left_right?"refine":"save_manual");if(r.refined_T_left_right){model.before=clone(r.initial_T_left_right);model.refined=clone(r.refined_T_left_right);}model.set(r.refined_T_left_right||r.initial_T_left_right);showResult(r,String(payload.saved_candidate.export_dir||"未提供"),model.current);loadedSaved=true;}catch(e){error("旧候选不能显示："+e.message);}}
      sync();fit(level.enabled?"oblique":"front");notice(loadedSaved?"正在显示已保存候选；可一键恢复页面原始初值或比较 ICP 前后。候选尚未正式验证。":"已载入来源绑定的初值。可先调整视角，再切换拖动右云；初值未获正式验证。");
    }).catch(e=>{error(e.message);$("alignment-source").textContent="场景读取失败，未启用调整。";});
  }
  return {validateMatrix,poseToMatrix,matrixToPose,dragTranslation,rotationGuide,sameMatrix,checkedResult,pixelColor,metricText,PoseState,start};
});
