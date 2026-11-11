const assert=require('node:assert/strict');
const ControlSession=require('./control-session.js');
function setup(saved={}){
  let value=JSON.stringify(saved),now=1000,manager;
  const calls=[],preparations=[],state={connected:false,my_slot:null,input:0};
  const storage={getItem:()=>value,setItem:(key,v)=>{value=v;}};
  const api=async(action,data={})=>{
    calls.push([action,data]);
    if(action==='connect')state.connected=true;
    if(action==='claim')state.my_slot=data.role;
    if(action==='release-role'){state.my_slot=null;state.input=0;}
    if(action==='leave'){state.connected=false;state.my_slot=null;state.input=0;}
    manager.observe(state);return {...state};
  };
  manager=new ControlSession({api,storage,now:()=>now,beforeConnect:async connected=>{preparations.push(connected);}});
  return {manager,state,calls,preparations,storage,advance:n=>{now+=n;},saved:()=>JSON.parse(value)};
}
async function main(){
  const first=setup();
  assert.equal(first.manager.shouldReconnect(),false);
  await first.manager.connect('LINEAR');
  assert.deepEqual(first.calls.map(x=>x[0]),['connect','claim']);
  assert.deepEqual(first.saved(),{role:'LINEAR',resume:true});
  const resumed=setup(first.saved());
  assert.equal(resumed.manager.shouldReconnect(),true);
  await resumed.manager.connect();
  assert.equal(resumed.state.my_slot,'LINEAR');

  const brief=setup({role:'FULL',resume:true});
  brief.state.connected=true;brief.state.my_slot='FULL';brief.state.input=.5;
  brief.manager.observe(brief.state);
  await brief.manager.connect();
  assert.deepEqual(brief.calls.map(x=>x[0]),['connect','release-role','claim']);
  assert.equal(brief.state.input,0,'Even the same role must discard pre-disconnection input');

  await resumed.manager.disconnect();
  assert.equal(resumed.manager.shouldReconnect(),false);
  assert.equal(new ControlSession({api:async()=>{},storage:resumed.storage}).enabled,false);
  await first.manager.release();
  assert.equal(first.saved().role,null,'Explicit role release must not be undone on reconnect');

  const failing=setup({role:'FULL',resume:true});
  failing.manager.api=async()=>{throw new Error('BUSY');};
  await assert.rejects(failing.manager.connect(),/BUSY/);
  assert.equal(failing.manager.shouldReconnect(),false);
  failing.advance(1501);assert.equal(failing.manager.shouldReconnect(),true);

  const cancel=setup();let release;
  const originalApi=cancel.manager.api;
  cancel.manager.api=async(action,data)=>{
    if(action==='connect')await new Promise(resolve=>{release=resolve;});
    return originalApi(action,data);
  };
  const pending=cancel.manager.connect('ANGULAR');
  await Promise.resolve();
  const leaving=cancel.manager.disconnect();
  release();await Promise.all([pending,leaving]);
  assert.deepEqual(cancel.calls.map(x=>x[0]),['connect','leave'],'Disconnect cancels pending role restoration');

  const conflict=setup({role:'FULL',resume:true});
  const conflictApi=conflict.manager.api;
  conflict.manager.api=async(action,data)=>{
    if(action==='claim')throw new Error('角色冲突');
    return conflictApi(action,data);
  };
  await assert.rejects(conflict.manager.connect(),/角色冲突/);
  assert.equal(conflict.manager.connected,true);
  assert.equal(conflict.manager.shouldReconnect(),false,'A conflict waits for explicit role selection');

  const privateMode=new ControlSession({api:async()=>({connected:true,my_slot:null}),
    storage:{getItem(){throw new Error('denied');},setItem(){throw new Error('denied');}}});
  await privateMode.connect('FULL');
  await assert.rejects(privateMode.connect('INVALID'),/无效/);
  for(const item of [first,resumed,brief,failing,cancel,conflict]){
    assert.ok(item.calls.every(x=>['connect','claim','release-role','leave'].includes(x[0])),
      'Session restoration never sends motion, reset or microphone requests');
  }
  console.log('PASS one-tap role connection, remembered roles, zero-input restoration, disconnect cancellation, backoff, conflicts and private mode');
  await testPage();
}
async function testPage(){
  const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
  const html=fs.readFileSync(path.join(__dirname,'index.html'),'utf8');
  const source=html.match(/<script>([\s\S]*?)<\/script>/)[1];
  async function page(saved={},conflict=false,hidden=false){
    const nodes=new Map(),trace=[],intervals=[];
    const node=id=>{
      if(!nodes.has(id))nodes.set(id,{textContent:'',value:'0',disabled:false,
        classList:{add(){},remove(){},toggle(){}},querySelector:()=>({textContent:''})});
      return nodes.get(id);
    };
    const roles=['FULL','LINEAR','ANGULAR','LEFT','RIGHT'].map(role=>Object.assign(node('role-'+role),{dataset:{role}}));
    const directions=['forward','backward','left','right'].map(direction=>Object.assign(node('direction-'+direction),{dataset:{direction}}));
    let stored=JSON.stringify(saved);
    const state={connected:false,my_slot:null,my_epoch:null,slots:{FULL:null,LINEAR:null,ANGULAR:null,LEFT:null,RIGHT:null},
      my_input:{},audio:{},feedback:{},voice_settings:{linear:.35,angular:.4},
      runtime:{mode:'ROS2',cmd_topic:'cmd_vel_bl',domain_id:'20',publish_hz:50,publisher_state:'RUNNING'},
      safety_state:'NORMAL',estop:true,last_cmd:{linear_x:0,angular_z:0},fast_partial:true,
      collision:{enabled:true,status:'CLEAR',action:'IDLE',fresh:true,scan_age_ms:1,front_degrees:0,message:'正常',
        distances:{front:1,rear:1,left:1,right:1},thresholds:{warning:.45,slowdown:.3,stop:.25},last_event:null}};
    const context=vm.createContext({ControlSession,CollisionNotifier:require('./collision-alerts.js'),
      document:{hidden,activeElement:null,getElementById:node,addEventListener(){},querySelectorAll:s=>s==='[data-role]'?roles:s==='[data-direction]'?directions:[]},
      window:{isSecureContext:true,localStorage:{getItem:()=>stored,setItem:(key,v)=>{stored=v;}},addEventListener(){}},
      navigator:{sendBeacon:(url,body)=>{trace.push(url+'-release');state.manual_takeover={active:false};return true;}},Blob,
      crypto:require('node:crypto').webcrypto,AbortController,setTimeout,clearTimeout,clearInterval(){},
      setInterval:fn=>{intervals.push(fn);return intervals.length;},
      fetch:async(url,options={})=>{
        const action=url.split('?')[0],body=options.body?JSON.parse(options.body):{};
        trace.push(action);
        if(action==='/connect')state.connected=true;
        if(action==='/claim'){
          if(conflict)return {json:async()=>({ok:false,error:'角色冲突'})};
          state.my_slot=body.role;state.slots[body.role]=body.client_id;state.my_epoch=1;
        }
        if(action==='/escape-mode'){state.driving_mode={escape:body.enabled};state.my_epoch++;}
        if(action==='/cmd')state.my_input={linear:body.linear||0,angular:body.angular||0};
        return {json:async()=>JSON.parse(JSON.stringify({ok:true,...state}))};
      }});
    vm.runInContext(source,context);
    await new Promise(resolve=>setImmediate(resolve));
    return {node,roles,directions,state,trace,intervals,context};
  }
  const first=await page();
  await first.roles[0].onclick();
  assert.deepEqual(first.trace,['/state','/connect','/claim']);
  assert.equal(first.node('slot').textContent,'FULL');
  assert.equal(first.state.estop,true);
  assert.equal(first.node('startVoice').disabled,true);
  const resumed=await page({role:'ANGULAR',resume:true});
  assert.deepEqual(resumed.trace,['/connect','/claim']);
  assert.equal(resumed.node('slot').textContent,'ANGULAR');
  const occupied=await page({role:'FULL',resume:true},true);
  assert.match(occupied.node('message').textContent,/请选择其他角色/);
  await occupied.intervals[0]();
  assert.equal(occupied.trace.filter(x=>x==='/claim').length,1);
  const background=await page({role:'FULL',resume:true},false,true);
  assert.deepEqual(background.trace,[],'Background pages do not reconnect');
  first.state.estop=false;
  vm.runInContext('poll()',first.context);await new Promise(resolve=>setImmediate(resolve));
  assert.equal(first.directions[0].disabled,false);
  first.node('escapeMode').checked=true;
  await first.node('escapeMode').onchange();
  assert.equal(first.trace.at(-1),'/escape-mode');
  assert.equal(first.node('linearSlider').disabled,false,'Same controls remain available in escape mode');
  assert.match(first.node('takeoverStatus').textContent,/人工脱困/);
  assert.equal(first.node('startVoice').disabled,false,'Speech remains usable with radar bypass');
  first.directions[0].onpointerdown({preventDefault(){}});
  await new Promise(resolve=>setImmediate(resolve));
  assert.equal(first.trace.at(-1),'/cmd');
  assert.equal(first.state.my_input.linear,.35);
  first.directions[2].onkeydown({key:'Enter',preventDefault(){}});
  await new Promise(resolve=>setImmediate(resolve));
  assert.equal(first.state.my_input.angular,.5,'Keyboard and pointer use the same command path');
  first.node('escapeMode').checked=false;
  await first.node('escapeMode').onchange();
  assert.equal(first.state.driving_mode.escape,false);
  first.directions[3].onpointerdown({preventDefault(){}});
  await new Promise(resolve=>setImmediate(resolve));
  assert.equal(first.trace.at(-1),'/cmd','Protected mode uses the same pad and endpoint');
  assert.equal(first.state.my_input.angular,-.5);
  assert.equal(first.trace.some(x=>x==='/escape'),false,'No hidden held lease or extra direction buttons');
  console.log('PASS actual page startup, one-tap role click, reload restoration, conflict feedback and foreground-only polling');
}
main().catch(error=>{console.error(error);process.exitCode=1;});
