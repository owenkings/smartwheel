"use strict";
(() => {
  const $=id=>document.getElementById(id),canvas=$("map"),ctx=canvas.getContext("2d");
  let points=[],revision=-1,mapData=null,busy=false,gesture=null,queued=false;
  const view={yaw:0,pitch:-.3,center:[0,0,0],pan:[0,0],scale:100};
  function project(q,center=view.center,scale=view.scale,pan=view.pan){
    const [x,y,z]=q.map((v,i)=>v-center[i]),cy=Math.cos(view.yaw),sy=Math.sin(view.yaw),cp=Math.cos(view.pitch),sp=Math.sin(view.pitch),rx=x*cy-y*sy,ry=x*sy+y*cy;
    return [canvas.width/2-ry*scale+pan[0],canvas.height/2-(z*cp-rx*sp)*scale+pan[1],rx*cp+z*sp];
  }
  function fit(){if(!points.length)return;const lo=[Infinity,Infinity,Infinity],hi=[-Infinity,-Infinity,-Infinity];points.forEach(p=>p.forEach((v,i)=>{lo[i]=Math.min(lo[i],v);hi[i]=Math.max(hi[i],v);}));view.center=lo.map((v,i)=>(v+hi[i])/2);view.pan=[0,0];let ex=0,ey=0;points.forEach(p=>{const q=project(p,view.center,1,[0,0]);ex=Math.max(ex,Math.abs(q[0]-canvas.width/2));ey=Math.max(ey,Math.abs(q[1]-canvas.height/2));});view.scale=Math.min(canvas.width*.43/Math.max(ex,.05),canvas.height*.43/Math.max(ey,.05));schedule();}
  function resize(){const r=canvas.getBoundingClientRect();canvas.width=Math.max(1,Math.min(2000,Math.round(r.width)));canvas.height=Math.max(1,Math.min(1200,Math.round(r.height)));fit();schedule();}
  function schedule(){if(queued)return;queued=true;requestAnimationFrame(()=>{queued=false;draw();});}
  function draw(){const w=canvas.width,h=canvas.height;ctx.fillStyle="#07101d";ctx.fillRect(0,0,w,h);const depth=new Float32Array(w*h);depth.fill(Infinity);ctx.fillStyle="#68c6dc";for(const point of points){const q=project(point),x=Math.round(q[0]),y=Math.round(q[1]);if(x<0||y<0||x>=w||y>=h)continue;const k=y*w+x;if(q[2]<depth[k]){depth[k]=q[2];ctx.fillRect(x,y,1.7,1.7);}}
    ctx.font="12px system-ui";ctx.fillStyle="#9db4c9";ctx.fillText(points.length?`视图宽约 ${(w/view.scale).toFixed(2)} m` : "等待三维地图",12,h-14);
    const origin=[64,67],colors=["#ff9393","#85d9a0","#8ab9ff"],labels=["X","Y","Z"],o=project([0,0,0],[0,0,0],36,[0,0]);for(let i=0;i<3;i++){const q=project([0,0,0].map((_,j)=>i===j?1:0),[0,0,0],36,[0,0]),end=[origin[0]+q[0]-o[0],origin[1]+q[1]-o[1]];ctx.strokeStyle=colors[i];ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(...origin);ctx.lineTo(...end);ctx.stroke();ctx.fillStyle=colors[i];ctx.fillText(labels[i],end[0]+4,end[1]-4);}
  }
  function metric(rows){const dl=$("metrics");dl.replaceChildren();rows.forEach(([a,b])=>{const dt=document.createElement("dt"),dd=document.createElement("dd");dt.textContent=a;dd.textContent=b;dl.append(dt,dd);});}
  const age=x=>x==null?"未知":`${x.toFixed(1)} s`,n=x=>x==null?"未知":String(x);
  async function request(path){const r=await fetch(path,{cache:"no-store"});if(!r.ok)throw Error(`读取失败 HTTP ${r.status}`);return r.json();}
  async function poll(){if(busy)return;busy=true;try{const s=await request("/api/status");$("session").textContent=`${s.session_id} · ${s.sensor_mode} · ${s.offline?"已保存快照，非实时":"实验几何地图"}`;$("state").textContent=(s.offline?"离线 · ":"")+s.status;$("state").classList.toggle("failed",s.status==="FAILED"||s.status==="TRACKING_LOST");$("error").textContent=s.failure?`${s.failure.code}: ${s.failure.detail}`:"";
      metric([["运行时长",`${s.elapsed_s.toFixed(1)} s`],["里程计消息",n(s.counts.odom)],["跟踪状态消息",n(s.counts.odom_info)],["地图消息",n(s.counts.cloud_map)],["里程计年龄",age(s.age_s.odom)],["跟踪状态年龄",age(s.age_s.odom_info)],["地图年龄",age(s.age_s.cloud_map)],["连续跟踪丢失",n(s.consecutive_lost)],["累计丢失报告",n(s.total_lost)]]);
      $("pose").textContent=s.odometry?(s.odometry.pose_valid===false?"该帧跟踪丢失，没有有效位姿":`${s.odometry.frame_id}\nX ${s.odometry.position[0].toFixed(3)} m\nY ${s.odometry.position[1].toFixed(3)} m\nZ ${s.odometry.position[2].toFixed(3)} m`):"未知";
      $("trajectory").textContent=`当前轨迹段 ${s.trajectory_segment} · 缓存 ${s.trajectory.length} 个可靠位姿`;
      if(s.map.revision!==revision){mapData=await request("/api/map");if(!Array.isArray(mapData.points)||mapData.points.length>50000||!mapData.points.every(p=>Array.isArray(p)&&p.length===3&&p.every(Number.isFinite)))throw Error("地图响应格式无效");points=mapData.points;revision=mapData.revision;if($("follow").checked)fit();schedule();}
      if(mapData&&mapData.available){$("map-detail").textContent=`${mapData.frame_id} · 原始 ${mapData.raw_point_count.toLocaleString()} 点 · 有限 ${mapData.finite_point_count.toLocaleString()} 点 · 显示 ${mapData.display_point_count.toLocaleString()} 点`;
        $("bounds").textContent=mapData.bounds?`最小 [${mapData.bounds.min.map(x=>x.toFixed(2)).join(", ")}] m\n最大 [${mapData.bounds.max.map(x=>x.toFixed(2)).join(", ")}] m`:"已收到地图，但暂无有限点";}
    }catch(e){$("error").textContent=`连接中断：${e.message}。当前画面是最近一次快照。`;$("state").textContent="连接未知";$("state").classList.add("failed");}finally{busy=false;setTimeout(poll,1000);}}
  function position(e){const r=canvas.getBoundingClientRect();return[(e.clientX-r.left)*canvas.width/r.width,(e.clientY-r.top)*canvas.height/r.height];}
  canvas.addEventListener("pointerdown",e=>{if(e.button!==0)return;const[x,y]=position(e);gesture={id:e.pointerId,x,y,shift:e.shiftKey};canvas.setPointerCapture(e.pointerId);});
  canvas.addEventListener("pointermove",e=>{if(!gesture||gesture.id!==e.pointerId)return;const[x,y]=position(e),dx=x-gesture.x,dy=y-gesture.y;$("follow").checked=false;if(e.shiftKey||gesture.shift){view.pan[0]+=dx;view.pan[1]+=dy;}else{view.yaw+=dx*.007;view.pitch=Math.max(-Math.PI/2,Math.min(Math.PI/2,view.pitch+dy*.007));}gesture.x=x;gesture.y=y;schedule();});
  for(const event of ["pointerup","pointercancel","lostpointercapture"])canvas.addEventListener(event,()=>{gesture=null;});
  canvas.addEventListener("wheel",e=>{e.preventDefault();$("follow").checked=false;const[x,y]=position(e),old=view.scale;view.scale=Math.max(1e-6,Math.min(1e7,old*Math.exp(-Math.max(-200,Math.min(200,e.deltaY))*.0015)));const ratio=view.scale/old;view.pan[0]=(view.pan[0]-(x-canvas.width/2))*ratio+x-canvas.width/2;view.pan[1]=(view.pan[1]-(y-canvas.height/2))*ratio+y-canvas.height/2;schedule();},{passive:false});
  document.querySelectorAll("[data-view]").forEach(button=>button.addEventListener("click",()=>{const preset=button.dataset.view;view.yaw=preset==="side"?Math.PI/2:0;view.pitch=preset==="top"?-Math.PI/2:0;fit();}));$("fit").addEventListener("click",fit);new ResizeObserver(resize).observe(canvas);resize();poll();
})();
