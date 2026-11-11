const assert=require('node:assert/strict');
const Notifier=require('./collision-alerts.js');
const a=new Notifier(),b=new Notifier();
const state=(id,status='STOP',instance_id='server-a')=>({instance_id,status,last_event:id?{id,status,message:'防撞提醒'}:null});
for(const phone of [a,b]){
  assert.equal(phone.observe(state(0,'CLEAR')),null);
  assert.equal(phone.observe(state(1,'WARNING')).id,1);
  assert.equal(phone.observe(state(1,'WARNING')),null);
  assert.equal(phone.observe(state(2)).id,2);
  assert.equal(phone.observe(state(2,'CLEAR')),null);
}
assert.equal(a.observe(state(3,'CLEAR')).id,3,'A short alarm between polls still delivers its persistent event');
assert.equal(a.observe(state(1,'NO_DATA','server-b')),null,'A restart establishes a new event baseline');
assert.equal(a.observe(state(2,'STOP','server-b')).id,2);
const late=new Notifier();
assert.equal(late.observe(state(50)),null,'A newly opened page does not replay historical sounds');
assert.equal(late.observe(state(51)).id,51);
console.log('PASS both phone alarms, deduplication, missed-poll events, restart and late join');

const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const html=fs.readFileSync(path.join(__dirname,'index.html'),'utf8');
const renderSource=html.slice(html.indexOf('function renderCollision(c){'),html.indexOf("\n$('enableAlerts').onclick"));
const elements=new Map();
const element=id=>{
  if(!elements.has(id))elements.set(id,{textContent:'',classList:{toggle(){}}});
  return elements.get(id);
};
const context=vm.createContext({$:element,collisionNotifier:{observe(){return null;}},connected:false});
vm.runInContext(renderSource,context);
const collision={status:'STOP',action:'PASS',awaiting_new_command:false,enabled:true,
  message:'左侧附近有障碍，仅允许向安全方向操作',fresh:true,scan_age_ms:10,
  distances:{front:1,rear:1,left:.08,right:1},front_degrees:0,last_event:null,
  thresholds:{warning:.45,slowdown:.3,stop:.25},
  thresholds_by_direction:{front:{warning:.45,slowdown:.3,stop:.25},left:{warning:.225,slowdown:.15,stop:.1}}};
context.renderCollision(collision);
assert.match(element('collisionThresholds').textContent,/前后 45cm \/ 30cm \/ 25cm/);
assert.match(element('collisionThresholds').textContent,/左右 22\.5cm \/ 15cm \/ 10cm/);
assert.equal(element('collisionStatus').textContent,'近距离障碍','A side obstacle must not label permitted straight driving as stopped');
collision.action='STOP';collision.awaiting_new_command=true;
context.renderCollision(collision);
assert.equal(element('collisionStatus').textContent,'防撞停车');
collision.fresh=false;
context.renderCollision(collision);
assert.equal(element('radar-left').textContent,'8 cm','Last reading remains visible for observation');
assert.match(element('age-left').textContent,/旧读数/,'Old distances are explicitly marked');
delete collision.thresholds_by_direction;
assert.doesNotThrow(()=>context.renderCollision(collision),'Supports a server that still uses uniform thresholds');
console.log('PASS direction thresholds, 22.5cm display, actual stop labels, stale readings and old-server fallback');
