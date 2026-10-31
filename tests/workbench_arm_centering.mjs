import assert from 'node:assert/strict';
import {initArmCentering} from '../pi/robot/web/static/workbench-arm-centering.js';
class Field {
    constructor(value=''){this.value=value;this.checked=false;this.textContent='';this.disabled=false;}
    checkValidity(){return Number.isFinite(Number(this.value));}
}
const defaults={'probe-reach':'2','probe-arm':'1','win-reach':'6','win-arm':'3','step-reach':'3','step-arm':'1.5','deadband':'4'};
const fields=new Map(['confirm','probe','auto','status','probe-result','accept','window-checked','calibration',...Object.keys(defaults)]
    .map(k=>[k,new Field(defaults[k]||'')]));
const panel={innerHTML:'',className:'',querySelector:s=>fields.get(s.match(/"(.*?)"/)[1]),addEventListener(){}};
globalThis.document={createElement:()=>panel};
let state={connected:true,site_ready:true,manual_debug:true,mechanism:{fault:0},base:{fault:0,stopped:true},
    sequence:{status:'RUNNING'},arrival:{phase:'IDLE'},arm_centering:{status:'IDLE',calibration_available:false}};
const calls=[];
const ui=initArmCentering({append(){}},()=>state,()=>true,()=>false,(action,args)=>calls.push({action,args}));
ui.draw();
assert.ok(fields.get('probe').disabled,'观察联调表未完成时不能探测');
state.sequence.status='DONE';ui.draw();
assert.equal(fields.get('probe').disabled,false);
assert.ok(fields.get('auto').disabled,'无正式标定不能自动对中');
fields.get('probe').onclick();assert.equal(calls.length,0,'未勾选确认不发送');
fields.get('confirm').checked=true;fields.get('probe').onclick();
assert.deepEqual(calls[0],{action:'arm_centering_start',args:{mode:'probe',confirmed:true,probe_reach_mm:2,probe_arm_deg:1}});
assert.equal(fields.get('confirm').checked,false,'确认只用一次');
state.arrival.phase='READY';ui.draw();assert.ok(fields.get('probe').disabled,'底盘微调占用时禁用');
state.arrival.phase='IDLE';state.arm_centering={status:'DONE',probe:{jacobian_px:[[6,0],[0,9]],residual_px:.2,hysteresis_px:.4,condition:1.5}};ui.draw();
assert.equal(fields.get('accept').disabled,false);
fields.get('accept').onclick();assert.equal(calls.length,1,'未确认窗口无碰撞不导入');
fields.get('window-checked').checked=true;fields.get('accept').onclick();
assert.equal(calls[1].action,'arm_centering_accept');
assert.deepEqual(calls[1].args.window,{reach_mm:6,arm_deg:3});
assert.equal(calls[1].args.physical_window_checked,true);
