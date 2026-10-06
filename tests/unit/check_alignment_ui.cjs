"use strict";
// Offline only. No DOM, files written, network, devices or third-party packages.
const assert=require("node:assert/strict");
const a=require("../../src/wc_calibration/picker_assets/alignment.js");
const p=require("../../src/wc_calibration/picker_assets/picker.js");
let count=0;
function test(name,fn){fn();count++;process.stdout.write("PASS "+name+"\n");}
const near=(x,y,t=1e-9)=>assert.ok(Math.abs(x-y)<t,`${x} != ${y}`);
function matrixNear(x,y,t=1e-9){x.forEach((r,i)=>r.forEach((v,j)=>near(v,y[i][j],t)));}
const pose=[.18,-.41,.08,13,-24,37];
const initial=a.poseToMatrix(pose);
const result={status:"EXPLORATION_ONLY",live_eligible:false,initial_T_left_right:initial,refined_T_left_right:a.poseToMatrix([.2,-.4,.09,12,-23,36])};

test("RPY is Rz Ry Rx with column-vector right-to-left translation",()=>{
  const m=a.poseToMatrix([.6,-.1,.02,90,0,90]);
  const out=p.transformPoint([1,2,3],m);[3.6,.9,2.02].forEach((v,i)=>near(out[i],v));
  a.validateMatrix(m);
});
test("RPY roundtrip handles combined rotations and nonzero translation",()=>{
  for(const r of [-170,-20,0,74,179])for(const b of [-89,-30,0,60,89])for(const y of [-179,0,131]){
    const m=a.poseToMatrix([.12,-.6,1.3,r,b,y]);matrixNear(a.poseToMatrix(a.matrixToPose(m)),m);
  }
});
test("gimbal singularities preserve rotation without inventing a unique roll",()=>{
  for(const pitch of [-90,90])for(const roll of [-127,0,43])for(const yaw of [-175,0,69]){
    const m=a.poseToMatrix([1,2,3,roll,pitch,yaw]);matrixNear(a.poseToMatrix(a.matrixToPose(m)),m);
    near(a.matrixToPose(m)[3],0);
  }
});
test("angle wrap changes representation but not the rigid transform",()=>{
  matrixNear(a.poseToMatrix([0,0,0,30+720,-40-360,90+360]),a.poseToMatrix([0,0,0,30,-40,90]));
});
test("invalid poses, reflections, scales and projective matrices are rejected",()=>{
  for(const bad of [NaN,Infinity,"1",null])assert.throws(()=>a.poseToMatrix([0,0,0,0,0,bad]));
  assert.throws(()=>a.poseToMatrix([0,0,0]));
  for(const mutation of [m=>m[0][0]=2,m=>m[0][0]=-1,m=>m[3][2]=1,m=>m[0][3]=Infinity,m=>m[0][3]=1e308]){
    const m=a.poseToMatrix([0,0,0,0,0,0]);mutation(m);assert.throws(()=>a.validateMatrix(m));
  }
});
test("front-view drag moves right cloud along -Y and -Z, not depth",()=>{
  const m=a.dragTranslation(initial,20,-10,{yaw:0,pitch:0,scale:100});
  near(m[0][3],initial[0][3]);near(m[1][3],initial[1][3]-.2);near(m[2][3],initial[2][3]+.1);
});
test("screen-plane translation matches projected displacement at arbitrary views",()=>{
  for(const yaw of [0,.3,Math.PI/2,-1.6])for(const pitch of [0,.7,-Math.PI/2,Math.PI/2]){
    const view={...p.newView(),yaw,pitch,scale:80},xyz=[2,.1,-.7];
    const before=p.projectPoint({id:1,xyz:p.transformPoint(xyz,initial)},view,800,600);
    const moved=a.dragTranslation(initial,17,-23,view);
    const after=p.projectPoint({id:1,xyz:p.transformPoint(xyz,moved)},view,800,600);
    near(after.x-before.x,17);near(after.y-before.y,-23);near(after.depth,before.depth);
    for(let i=0;i<3;i++)for(let j=0;j<3;j++)near(moved[i][j],initial[i][j]);
  }
});
test("drag leaves input matrix untouched and rejects invalid views",()=>{
  const original=JSON.stringify(initial);a.dragTranslation(initial,1,2,{yaw:.2,pitch:.3,scale:100});assert.equal(JSON.stringify(initial),original);
  for(const scale of [0,-1,NaN])assert.throws(()=>a.dragTranslation(initial,1,2,{yaw:0,pitch:0,scale}));
});
test("left XYZ remains unchanged while right uses full precision matrix once",()=>{
  const left={id:7,xyz:[3.12,.09,-.46]},before=JSON.stringify(left),right={id:7,xyz:[1,2,3]};
  const transformed={id:right.id,xyz:p.transformPoint(right.xyz,initial)};
  assert.equal(JSON.stringify(left),before);assert.deepEqual(right.xyz,[1,2,3]);assert.equal(transformed.id,7);
  for(let i=0;i<3;i++)near(transformed.xyz[i],initial[i][0]+2*initial[i][1]+3*initial[i][2]+initial[i][3]);
});
test("two-color mix is distinct from either side and does not hide the farther side",()=>{
  const both=a.pixelColor(true,true);assert.notDeepEqual(both,a.pixelColor(true,false));assert.notDeepEqual(both,a.pixelColor(false,true));
  assert.notDeepEqual(both,a.pixelColor(false,false));assert.ok(both.every(v=>Number.isInteger(v)&&v>=0&&v<=255));
});
test("successful ICP retains submitted and refined matrices for comparison",()=>{
  const s=new a.PoseState(initial);s.acceptRefinement(result,initial);matrixNear(s.current,result.refined_T_left_right);matrixNear(s.before,initial);s.set(s.before);matrixNear(s.current,initial);s.set(s.refined);matrixNear(s.current,result.refined_T_left_right);
  result.refined_T_left_right[0][3]+=1;assert.notEqual(s.current[0][3],result.refined_T_left_right[0][3]);result.refined_T_left_right[0][3]-=1;
});
test("failed or missing ICP transform never replaces the current manual pose",()=>{
  const s=new a.PoseState(initial);s.set(a.poseToMatrix([.9,.8,.7,9,8,7]));const before=JSON.stringify(s.current);
  assert.throws(()=>s.acceptRefinement({...result,refined_T_left_right:null},initial));assert.equal(JSON.stringify(s.current),before);
  assert.throws(()=>s.acceptRefinement({...result,live_eligible:true},initial));assert.equal(JSON.stringify(s.current),before);
  assert.equal(s.before,null);assert.equal(s.refined,null);
});
test("manual saves and rejected exploratory results never imply validation",()=>{
  a.checkedResult({status:"EXPLORATION_ONLY",live_eligible:false,initial_T_left_right:initial},"save_manual");
  a.checkedResult({...result,status:"UNVALIDATED"},"refine");
  assert.throws(()=>a.checkedResult({...result,live_eligible:undefined},"refine"));
});
test("nearest-neighbor metrics distinguish no inliers from measured zero residual",()=>{
  const text=a.metricText({source_points:9000,target_points:8990,max_correspondence_distance_m:.15,forward_overlap_fraction:.4,reverse_overlap_fraction:.5,nn_rmse_m:null,nn_median_m:0,nn_p95_m:.003});
  assert.match(text,/RMSE：无内点/);assert.match(text,/中位数：0\.00 mm/);assert.match(text,/P95：3\.00 mm/);assert.match(text,/40\.00 %/);assert.match(text,/门限：0\.150 m/);
  assert.doesNotMatch(text,/精度/);assert.match(a.metricText(null),/未提供/);
});
test("reset restores page bootstrap, not an identity or guessed baseline",()=>{
  const s=new a.PoseState(initial);s.set(a.poseToMatrix([1,2,3,4,5,6]));s.reset();matrixNear(s.current,initial);
  s.current[0][3]=999;s.reset();matrixNear(s.current,initial);
});
process.stdout.write(`\n${count} alignment UI checks passed (synthetic / no DOM / no hardware).\n`);
