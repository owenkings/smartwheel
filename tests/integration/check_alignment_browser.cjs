/* Owned Playwright browser only. Real mode is GET-only; writes require --synthetic. */
"use strict";
const fs=require("node:fs"),path=require("node:path"),crypto=require("node:crypto"),assert=require("node:assert/strict");
const geometry=require("../../src/wc_calibration/picker_assets/alignment.js");
const argv=process.argv.slice(2),options={};
for(let i=0;i<argv.length;i++){
  const key=argv[i];
  if(key==="--synthetic"){assert.equal(options.synthetic,undefined,"Duplicate --synthetic");options.synthetic=true;}
  else{assert.ok(["--url","--output"].includes(key),"Unknown argument: "+key);assert.equal(options[key],undefined,"Duplicate option");assert.ok(argv[i+1]&&!argv[i+1].startsWith("--"),"Missing option value");options[key]=argv[++i];}
}
assert.ok(options["--url"]&&options["--output"],"Usage: --url http://127.0.0.1:PORT/alignment --output NEW_DIR [--synthetic]");
const target=new URL(options["--url"]),synthetic=options.synthetic===true;
assert.ok(target.protocol==="http:"&&target.hostname==="127.0.0.1"&&target.port&&target.pathname==="/alignment"&&!target.search&&!target.hash&&!target.username&&!target.password,"Only an explicit owned loopback /alignment URL is allowed");
const output=path.resolve(options["--output"]);
fs.mkdirSync(output,{recursive:false}); // New output only: preserve all earlier evidence.
const result={status:"FAIL",source:synthetic?"synthetic":"real_read_only",url:target.href,
  starts_devices:false,starts_motion:false,actions:[],screenshots:[],post_requests:[],page_errors:[],network_policy_violations:[]};
let browser,context,page;
function matrixNear(actual,expected,tolerance=1e-9){geometry.validateMatrix(actual);geometry.validateMatrix(expected);for(let i=0;i<4;i++)for(let j=0;j<4;j++)assert.ok(Math.abs(actual[i][j]-expected[i][j])<=tolerance,`Matrix [${i},${j}] ${actual[i][j]} != ${expected[i][j]}`);}
const copy=value=>JSON.parse(JSON.stringify(value));
const sha=filename=>crypto.createHash("sha256").update(fs.readFileSync(filename)).digest("hex");
async function settle(){await page.evaluate(()=>new Promise(resolve=>requestAnimationFrame(()=>requestAnimationFrame(resolve))));}
async function matrix(){return JSON.parse(await page.locator("#current-matrix").textContent());}
async function ready(){await page.waitForFunction(()=>{const e=document.querySelector("#current-matrix");return e&&e.textContent.trim().startsWith("[");});await page.locator("#refine").waitFor({state:"visible"});await settle();}
async function bootstrapNavigate(reload=false){
  const response=page.waitForResponse(r=>new URL(r.url()).pathname==="/api/alignment-bootstrap"&&r.request().method()==="GET");
  if(reload)await page.reload({waitUntil:"domcontentloaded"});else await page.goto(target.href,{waitUntil:"domcontentloaded"});
  const received=await response;assert.equal(received.status(),200);const payload=await received.json();await ready();return payload;
}
async function screenshot(name){
  const filename=path.join(output,name+".png");await page.screenshot({path:filename,fullPage:true});
  const canvasFile=path.join(output,name+"_canvas.png");await page.locator("#alignment-canvas").screenshot({path:canvasFile});
  const colors=await page.locator("#alignment-canvas").evaluate((canvas,expected)=>{
    const data=canvas.getContext("2d").getImageData(0,0,canvas.width,canvas.height).data;
    const counts={left:0,right:0,mixed:0};
    for(let i=0;i<data.length;i+=4)for(const key of Object.keys(counts)){const c=expected[key];if(data[i]===c[0]&&data[i+1]===c[1]&&data[i+2]===c[2])counts[key]++;}
    return {width:canvas.width,height:canvas.height,pixel_counts:counts};
  },{left:geometry.pixelColor(true,false),right:geometry.pixelColor(false,true),mixed:geometry.pixelColor(true,true)});
  assert.ok(colors.pixel_counts.left+colors.pixel_counts.mixed>25,"Left cloud is not visibly represented");
  assert.ok(colors.pixel_counts.right+colors.pixel_counts.mixed>25,"Right cloud is not visibly represented");
  result.screenshots.push({name,full_page_sha256:sha(filename),canvas_sha256:sha(canvasFile),...colors});
}
async function drag(dx,dy){
  const canvas=page.locator("#alignment-canvas");await canvas.scrollIntoViewIfNeeded();const box=await canvas.boundingBox();assert.ok(box&&box.width>100&&box.height>100);
  const x=box.x+box.width*.46,y=box.y+box.height*.44;
  await page.mouse.move(x,y);await page.mouse.down();await page.mouse.move(x+dx,y+dy,{steps:5});await page.mouse.up();await settle();
}
async function applyFields(changes){
  for(const [field,delta]of Object.entries(changes)){const input=page.locator("#pose-"+field);await input.fill(String(Number(await input.inputValue())+delta));}
  const pose=[];for(const key of ["x","y","z","roll","pitch","yaw"])pose.push(Number(await page.locator("#pose-"+key).inputValue()));
  await page.locator("#apply-pose").click();await settle();matrixNear(await matrix(),geometry.poseToMatrix(pose));
}
async function compareControls(bootstrap){
  const original=await matrix();await page.locator("#view-mode").click();await drag(30,14);matrixNear(await matrix(),original,0);
  result.actions.push("view_drag_does_not_change_right_to_left_transform");
  await page.locator("#adjust-mode").click();const before=await matrix();await drag(25,-12);const moved=await matrix();
  assert.ok(Math.hypot(...moved.slice(0,3).map((r,i)=>r[3]-before[i][3]))>1e-4);
  for(let i=0;i<3;i++)for(let j=0;j<3;j++)assert.equal(moved[i][j],before[i][j]);
  result.actions.push("right_cloud_drag_changes_only_translation");
  await applyFields({x:.003,roll:.5,pitch:-.2,yaw:.4});
  await page.locator("#translation-step").fill("0.005");const prior=await matrix();await page.locator('[data-nudge="1"][data-sign="1"]').click();await settle();const nudged=copy(prior);nudged[1][3]+=.005;matrixNear(await matrix(),nudged);
  result.actions.push("six_pose_inputs_and_translation_nudge_match_displayed_matrix");
  const stable=await matrix();
  for(const preset of ["top","side","front","fit"]){await page.locator(`[data-view="${preset}"]`).click();await settle();matrixNear(await matrix(),stable,0);}
  for(const side of ["left","right"]){await page.locator("#show-"+side).uncheck();await settle();matrixNear(await matrix(),stable,0);await page.locator("#show-"+side).check();}
  await page.locator("#point-radius").selectOption("2.6");await settle();await page.locator("#point-radius").selectOption("1.8");
  await page.locator("#reset-initial").click();await settle();matrixNear(await matrix(),bootstrap.initial_T_left_right);
  result.actions.push("view_presets_layer_visibility_and_initial_reset");
}
async function post(operation,button){
  const expected=await matrix();
  const pending=page.waitForResponse(r=>new URL(r.url()).pathname==="/api/alignment"&&r.request().method()==="POST",{timeout:120000});
  await page.locator(button).click();const response=await pending;assert.equal(response.status(),200);
  const sent=response.request().postDataJSON();assert.deepEqual(Object.keys(sent).sort(),["initial_T_left_right","input_hash","operation","scene_id"]);
  assert.equal(sent.operation,operation);assert.equal(sent.input_hash,result.scene.input_hash);assert.equal(sent.scene_id,result.scene.scene_id);matrixNear(sent.initial_T_left_right,expected);
  const payload=await response.json();geometry.checkedResult(payload.result,operation);assert.ok(typeof payload.export_dir==="string"&&payload.export_dir.length>0);
  await page.waitForFunction(()=>!document.querySelector("#refine").disabled);await settle();assert.equal(await page.locator("#alignment-error").isVisible(),false);
  result.actions.push("actual_backend_"+operation);return {payload,submitted:expected};
}
(async()=>{
  try{
    const {chromium}=require(process.env.WC_TEST_PLAYWRIGHT||"playwright");
    browser=await chromium.launch({headless:true,executablePath:process.env.WC_TEST_CHROME});
    context=await browser.newContext({viewport:{width:1500,height:1080},deviceScaleFactor:1});
    await context.route("**/*",async route=>{
      const request=route.request(),u=new URL(request.url()),method=request.method();
      const allowed=u.origin===target.origin&&(["GET","HEAD"].includes(method)||(synthetic&&method==="POST"&&u.pathname==="/api/alignment"));
      if(!allowed){result.network_policy_violations.push({method,url:u.href});await route.abort("blockedbyclient");return;}await route.continue();
    });
    page=await context.newPage();page.setDefaultTimeout(20000);page.setDefaultNavigationTimeout(30000);
    page.on("pageerror",error=>result.page_errors.push(String(error)));
    page.on("request",request=>{if(request.method()==="POST"){let operation=null;try{operation=request.postDataJSON().operation;}catch(_){}result.post_requests.push({path:new URL(request.url()).pathname,operation});}});
    const bootstrap=await bootstrapNavigate();const scene=bootstrap.scene;
    assert.equal(scene.source_mode,synthetic?"synthetic":"real");assert.equal(scene.live_eligible,false);
    assert.ok(scene.clouds.left.length>=1000&&scene.clouds.right.length>=1000,"Expected actual dense test clouds");
    result.scene={scene_id:scene.scene_id,input_hash:scene.input_hash,source_mode:scene.source_mode,time_quality:scene.time_quality,counts:{left:scene.clouds.left.length,right:scene.clouds.right.length},initial_source:bootstrap.initial_source};
    const saved=bootstrap.saved_candidate;
    if(!synthetic)assert.ok(saved&&saved.result,"Real scene must provide its source-bound saved candidate");
    const savedT=saved?(saved.result.refined_T_left_right||saved.result.initial_T_left_right):bootstrap.initial_T_left_right;
    matrixNear(await matrix(),savedT);result.bootstrap_status=saved?saved.result.status:bootstrap.initial_source.status;
    if(!synthetic){
      assert.match(scene.scene_id,/mybox/i);assert.equal(saved.result.status,"NOT_CONVERGED");assert.equal(saved.result.live_eligible,false);
      assert.match(await page.locator("#result-status").textContent(),/NOT_CONVERGED/);assert.match(await page.locator("#result-status").textContent(),/未通过/);
      assert.match(await page.locator("#alignment-time").textContent(),/分时采集/);
    }
    assert.equal(await page.locator('#pose-controls input').evaluateAll(inputs=>inputs.every(input=>input.checkValidity())),true,'Default full-precision pose and nudge inputs must be valid');
    await screenshot("loaded_candidate");await compareControls(bootstrap);
    if(synthetic){
      // Explicit synthetic perturbation after reset: no real source is ever posted.
      await applyFields({x:.03,yaw:.6});const perturbed=await matrix();
      const manual=await post("save_manual","#save-manual");matrixNear(manual.payload.result.initial_T_left_right,perturbed);matrixNear(await matrix(),perturbed);
      result.manual_save={export_dir:manual.payload.export_dir,result:manual.payload.result};
      const refined=await post("refine","#refine");const r=refined.payload.result;matrixNear(r.initial_T_left_right,refined.submitted);matrixNear(await matrix(),r.refined_T_left_right);
      assert.notEqual(refined.payload.export_dir,manual.payload.export_dir);result.refinement={export_dir:refined.payload.export_dir,result:r};
      const known=geometry.poseToMatrix([.12,-.6,.03,2,-3,4]);
      const translationError=Math.hypot(...r.refined_T_left_right.slice(0,3).map((row,i)=>row[3]-known[i][3]));
      const trace=known.slice(0,3).reduce((sum,row,i)=>sum+row.slice(0,3).reduce((s,v,j)=>s+v*r.refined_T_left_right[i][j],0),0);
      const rotationError=Math.acos(Math.max(-1,Math.min(1,(trace-1)/2)))*180/Math.PI;
      result.synthetic_truth_check={translation_error_m:translationError,rotation_error_deg:rotationError,translation_limit_m:.02,rotation_limit_deg:1};
      assert.ok(translationError<.02&&rotationError<1,"Synthetic refinement did not recover the known rigid transform within the declared test bounds");
      assert.ok(Number.isFinite(r.initial_metrics.nn_rmse_m)&&Number.isFinite(r.final_metrics.nn_rmse_m));
      assert.ok(r.final_metrics.nn_rmse_m<r.initial_metrics.nn_rmse_m,"Synthetic nearest-neighbor residual did not improve");
      await page.locator("#show-before").click();await settle();matrixNear(await matrix(),r.initial_T_left_right);
      await page.locator("#show-refined").click();await settle();matrixNear(await matrix(),r.refined_T_left_right);
      result.actions.push("ICP_before_after_match_actual_response_and_synthetic_known_geometry");await screenshot("synthetic_refined");
      // Fail one RPC locally; backend files and latest candidate are not touched by this request.
      await applyFields({z:.007});const failurePose=await matrix();
      const failRoute=route=>route.fulfill({status:422,contentType:"application/json",body:JSON.stringify({error:"SYNTHETIC_ALIGNMENT_RPC_FAILURE"})});
      await page.route(target.origin+"/api/alignment",failRoute);await page.locator("#refine").click();
      await page.locator("#alignment-error").filter({hasText:"SYNTHETIC_ALIGNMENT_RPC_FAILURE"}).waitFor();await page.waitForFunction(()=>!document.querySelector("#refine").disabled);matrixNear(await matrix(),failurePose,0);
      await page.unroute(target.origin+"/api/alignment",failRoute);result.actions.push("failed_RPC_preserves_current_manual_transform");
      const refreshed=await bootstrapNavigate(true);assert.equal(refreshed.scene.input_hash,scene.input_hash);assert.equal(refreshed.saved_candidate.export_dir,refined.payload.export_dir);
      matrixNear(refreshed.saved_candidate.result.refined_T_left_right,r.refined_T_left_right);matrixNear(await matrix(),r.refined_T_left_right);
      assert.equal(await page.locator("#result-dirty").isVisible(),false);result.actions.push("refresh_loads_latest_matching_saved_candidate");await screenshot("synthetic_reloaded");
      assert.deepEqual(result.post_requests.map(r=>r.operation),["save_manual","refine","refine"]);
    }else{
      await page.locator("#show-before").click();await settle();matrixNear(await matrix(),saved.result.initial_T_left_right);
      await page.locator("#show-refined").click();await settle();matrixNear(await matrix(),saved.result.refined_T_left_right);
      result.actions.push("real_candidate_before_after_controls_only_no_POST");await screenshot("real_restored_candidate");
      assert.equal(result.post_requests.length,0,"Real mode must never make a POST request");
    }
    assert.deepEqual(result.network_policy_violations,[]);assert.deepEqual(result.page_errors,[]);result.status="PASS";
  }catch(error){result.error=String(error.stack||error);process.exitCode=1;}
  finally{
    if(page&&result.status!=="PASS")try{await page.screenshot({path:path.join(output,"failure.png"),fullPage:true});result.failure_screenshot_sha256=sha(path.join(output,"failure.png"));}catch(error){result.failure_screenshot_error=String(error);}
    if(context)try{await context.close();result.context_closed=true;}catch(error){result.status="FAIL";result.context_cleanup_error=String(error);process.exitCode=1;}
    if(browser)try{await browser.close();result.browser_closed=true;}catch(error){result.status="FAIL";result.browser_closed=false;result.browser_cleanup_error=String(error);process.exitCode=1;}
    fs.writeFileSync(path.join(output,"result.json"),JSON.stringify(result,null,2)+"\n");
    console.log(JSON.stringify({status:result.status,source:result.source,actions:result.actions,error:result.error,output}));
  }
})();
