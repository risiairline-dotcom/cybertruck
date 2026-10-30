import assert from 'node:assert/strict';
import {initArrival} from '../pi/robot/web/static/workbench-arrival.js';
class Field {
    constructor(value=''){this.value=value;this.checked=false;this.textContent='';this.disabled=false;this.children=[];}
    replaceChildren(...children){this.children=children;if(!this.value&&children[0])this.value=String(children[0].value);}
    get options(){return this.children;}
    checkValidity(){return Number(this.value)>=1&&Number(this.value)<=5;}
}
const fields=new Map(['version','order','confirm','status','start','step','finish','preview','reference','vision','image',
    'auto-start','auto','pause','reload','calibration','records','key-prompt'].map(k=>[k,new Field()]));
fields.get('order').value='1';fields.get('step').value='5';
const buttons=['x','y'].flatMap(axis=>[1,-1].map(sign=>({dataset:{axis,sign:String(sign)},disabled:false})));
const panel={innerHTML:'',className:'',querySelector:s=>fields.get(s.match(/"(.*?)"/)[1]),
    querySelectorAll:()=>buttons,addEventListener(){},getClientRects:()=>[]};
globalThis.document={createElement:()=>panel,hidden:false};
globalThis.Option=class {constructor(text,value){this.text=text;this.value=String(value);}};
globalThis.setInterval=()=>0;
let state={connected:true,site_ready:true,manual_debug:true,firmware:'GC-BASE-0.3.44',
    mechanism:{zero_valid:true,fault:0},base:{fault:0},sequences:[{id:'observe',revision:1,sha256:'a'.repeat(64)}],
    arrival:{phase:'IDLE'}},owned=true,busy=false,calls=[];
const ui=initArrival({append(){}},()=>state,()=>owned,()=>busy,async(action,args)=>calls.push({action,args}),()=>{});
ui.draw();assert.equal(calls.length,0);assert.ok(fields.get('start').disabled);assert.ok(buttons.every(b=>b.disabled));
state.firmware='GC-BASE-0.3.45';ui.draw();assert.equal(fields.get('start').disabled,false);
state.firmware='GC-BASE-0.3.46';ui.draw();assert.equal(fields.get('start').disabled,false);
state.firmware='GC-BASE-0.3.47';ui.draw();assert.equal(fields.get('start').disabled,false);
state.arrival.phase='MOVING';ui.draw();assert.match(fields.get('status').textContent,/停车取样/);assert.ok(buttons.every(b=>b.disabled));
state.arrival.phase='IDLE';state.firmware='GC-BASE-0.3.48';ui.draw();assert.equal(fields.get('start').disabled,false);
state.firmware='GC-BASE-0.3.49';ui.draw();assert.equal(fields.get('start').disabled,false);
state.firmware='GC-BASE-0.3.50';ui.draw();assert.equal(fields.get('start').disabled,false);
state.firmware='GC-BASE-0.3.51';ui.draw();assert.equal(fields.get('start').disabled,false);
state.arrival.phase='MOVING';ui.draw();assert.match(fields.get('status').textContent,/停车取样/);assert.ok(buttons.every(b=>b.disabled));
state.arrival.phase='IDLE';state.firmware='GC-BASE-0.3.52';ui.draw();assert.equal(fields.get('start').disabled,true);
state.firmware='GC-BASE-0.3.45';ui.draw();
assert.ok(fields.get('auto-start').disabled);
fields.get('confirm').checked=true;fields.get('start').onclick();await Promise.resolve();
assert.equal(calls[0].action,'alignment_start');assert.equal(calls[0].args.sha256,'a'.repeat(64));
state.arrival.phase='WAIT_PC0';ui.draw();assert.ok(buttons.every(b=>b.disabled));
assert.match(fields.get('key-prompt').textContent,/先等待/);
state.physical_start={phase:2,age_ms:20,remaining_ms:59100};ui.draw();
assert.match(fields.get('key-prompt').textContent,/现在松开后按一次PC0.*59秒/);
assert.match(fields.get('key-prompt').textContent,/不会自行移动/);
state.physical_start.age_ms=251;ui.draw();assert.match(fields.get('key-prompt').textContent,/先等待/);
state.arrival.phase='READY';ui.draw();assert.ok(buttons.every(b=>!b.disabled));
buttons[0].onclick();await Promise.resolve();assert.deepEqual(calls[1],{action:'alignment_step',args:{axis:'x',mm:5}});
state.centering={mode:'auto',calibration_available:true};ui.draw();assert.ok(buttons.every(b=>b.disabled));
assert.equal(fields.get('pause').disabled,false);assert.ok(fields.get('auto').disabled);
fields.get('pause').onclick();await Promise.resolve();assert.equal(calls.at(-1).action,'alignment_pause');
state.arrival.phase='DONE';state.centering.mode='manual';ui.draw();assert.equal(fields.get('auto-start').disabled,false);
fields.get('confirm').checked=true;fields.get('auto-start').onclick();await Promise.resolve();
assert.equal(calls.at(-1).args.mode,'auto');
owned=false;ui.draw();assert.ok(buttons.every(b=>b.disabled));
owned=true;busy=true;ui.draw();assert.ok(buttons.every(b=>b.disabled));
busy=false;state.connected=false;ui.draw();assert.ok(fields.get('finish').disabled);
state.arrival={phase:'FAILED',reason:'PC0准备取消：等待PC0满60秒超时，未启动'};ui.draw();
assert.match(fields.get('key-prompt').textContent,/满60秒超时/);assert.ok(buttons.every(b=>b.disabled));
console.log('微调旧固件禁用、PC0阶段、所有者/断线/忙状态和显式方向请求 PASS；无网络/设备调用');

