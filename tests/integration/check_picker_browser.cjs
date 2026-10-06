/* Real browser interaction against the actual picker. No sensor/motor access. */
const fs = require('node:fs');
const path = require('node:path');
const assert = require('node:assert/strict');
const {chromium} = require(process.env.WC_TEST_PLAYWRIGHT || 'playwright');
const math = require('../../src/wc_calibration/picker_assets/picker.js');
const argv = process.argv.slice(2);
function option(name) { return argv[argv.indexOf(name)+1]; }
const url = option('--url'), output = path.resolve(option('--output'));
const synthetic = argv.includes('--synthetic');
if (!/^http:\/\/127\.0\.0\.1:\d+\/$/.test(url)) throw new Error('Only own loopback test server allowed');
fs.mkdirSync(output, {recursive: false});
(async () => {
  const result = {status:'FAIL', source: synthetic?'synthetic':'real_read_only', starts_devices:false, actions:[]};
  let browser;
  try {
    browser = await chromium.launch({headless:true, executablePath:process.env.WC_TEST_CHROME});
    const page = await browser.newPage({viewport:{width:1440,height:1100},deviceScaleFactor:1});
    const errors=[]; page.on('pageerror',e=>errors.push(String(e)));
    await page.goto(url);
    await page.locator('#left-count').filter({hasText:'个有效原始点'}).waitFor();
    const scene = await (await page.request.get(url+'api/scene')).json();
    result.scene={scene_id:scene.scene_id,source_mode:scene.source_mode,input_hash:scene.input_hash,
      counts:Object.fromEntries(['left','right'].map(s=>[s,scene.clouds[s].length]))};
    assert.equal(scene.source_mode,synthetic?'synthetic':'real');
    assert.equal(await page.locator('#solve').isDisabled(),true);
    const count = async n => page.waitForFunction(n=>document.querySelector('#pair-count').textContent.startsWith(n+' 对'),n);
    async function point(side,id) {
      const canvas=page.locator('#'+side+'-canvas');await canvas.scrollIntoViewIfNeeded();
      const box=await canvas.boundingBox();
      const size=await canvas.evaluate(c=>({width:c.width,height:c.height}));
      const view=math.fitView(scene.clouds[side],size.width,size.height,math.newView());
      const p=math.projectPoint(scene.clouds[side].find(p=>p.id===id),view,size.width,size.height);
      await page.mouse.click(box.x+p.x*box.width/size.width,box.y+p.y*box.height/size.height);
    }
    await page.screenshot({path:path.join(output,'loaded.png'),fullPage:true});
    if (synthetic) {
      await point('left',0);await count(0);result.actions.push('browse_click_does_not_select');
      await page.locator('#mode-pick').click();
      await point('left',0);await point('left',1);await point('right',1);await count(1);
      assert.match(await page.locator('#pair-rows').innerText(),/#1/);
      result.actions.push('replace_pending_side_and_complete_pair');
      await page.locator('#pair-rows button').first().click();await count(0);
      await page.locator('#undo').click();await count(1);
      await page.locator('#clear').click();await count(0);
      result.actions.push('delete_undo_clear');
      for(let id=0;id<6;id++){await point('left',id);await point('right',id);await count(id+1);}
      result.actions.push('six_actual_canvas_correspondence_clicks');
      let response=page.waitForResponse(r=>r.url().endsWith('/api/export')&&r.request().method()==='POST');
      await page.locator('#export').click();const exported=await(await response).json();
      assert.equal(exported.selection.left.length,6);assert.equal(exported.result,undefined);
      result.export_path=exported.export_dir;
      response=page.waitForResponse(r=>r.url().endsWith('/api/solve')&&r.request().method()==='POST');
      await page.locator('#solve').click();const solved=await(await response).json();
      assert.equal(solved.result.status,'CANDIDATE');assert.equal(solved.result.live_eligible,false);
      const expected=[.2,-.3,.1];
      for(let i=0;i<3;i++)assert.ok(Math.abs(solved.result.initial_T_left_right[i][3]-expected[i])<1e-10);
      assert.ok(solved.result.rmse_m<1e-10);result.solve=solved.result;result.solve_path=solved.export_dir;
      await page.locator('#result-section').waitFor({state:'visible'});
      await page.locator('#result-status').filter({hasText:'初值候选'}).waitFor();
      await page.screenshot({path:path.join(output,'solved.png'),fullPage:true});
      // A failed request must preserve the user's selections and expose the error.
      await page.route('**/api/solve',route=>route.fulfill({status:422,contentType:'application/json',body:JSON.stringify({error:'SYNTHETIC_REQUEST_FAILURE'})}));
      await page.locator('#solve').click();
      await page.locator('#error').filter({hasText:'SYNTHETIC_REQUEST_FAILURE'}).waitFor();await count(6);
      result.actions.push('export_solve_known_transform_error_preserves_selection');
    } else {
      for(const side of ['left','right']){
        await page.locator(`.view-tools[data-view="${side}"] [data-preset="top"]`).click();
        await page.locator(`.view-tools[data-view="${side}"] [data-preset="front"]`).click();
      }
      await count(0);result.actions.push('real_frozen_clouds_visible_views_switch_no_selection_or_export');
    }
    result.page_errors=errors;assert.deepEqual(errors,[]);result.status='PASS';
  }catch(error){result.error=String(error.stack||error);throw error;}
  finally{
    if(browser)await browser.close();
    fs.writeFileSync(path.join(output,'result.json'),JSON.stringify(result,null,2));
    console.log(JSON.stringify({status:result.status,source:result.source,actions:result.actions,error:result.error}));
  }
})();
