import {firmwareSince} from './workbench-firmware.js?v=2';
import {setText, setSteps} from './workbench-dom.js?v=1';
import {actuators,moveArgs,allowed,gripMaximum} from './workbench-remote-state.js?v=49.1';

/* 参数表只编排正式远程请求；不计时模拟到位，不保存设备令牌。 */
export function sequenceRows(rows,mode='ordered',options={}) {
    if(!Array.isArray(rows)||!rows.length||rows.length>40)throw Error('请添加1～40行动作');
    if(!['ordered','together'].includes(mode))throw Error('执行方式无效');
    const groups=new Map();
    return rows.map(row=>{
        if(row.accel!==undefined&&String(row.accel).trim()==='')throw Error('请填写明确的加速度档位');
        const order=Number(row.order);
        if(!Number.isInteger(order)||order<1||order>999)throw Error('顺序号需为1～999整数');
        const args=moveArgs(row.axis,row.target,Number(row.speed),false,
            {manualTuning:true,armMax:359,gripMax:gripMaximum(options),accel:row.accel===undefined?1:Number(row.accel),speedDegS:Number(row.speed)});
        const key=mode==='together'?1:order;
        if(!groups.has(key))groups.set(key,new Set());
        if(groups.get(key).has(row.axis))throw Error('同一同时组不能重复设置同一执行器，请使用不同顺序号');
        groups.get(key).add(row.axis);
        return {order,...args};
    });
}
export function sameRows(left,right) {
    return Array.isArray(left)&&Array.isArray(right)&&left.length===right.length&&left.every((a,i)=>{
        const b=right[i];return a.order===b.order&&a.axis===b.axis&&(a.accel??1)===(b.accel??1)&&
            ['target_m','target_rad','rpm','speed_rad_s'].every(k=>a[k]===b[k]||
            (Number.isFinite(a[k])&&Number.isFinite(b[k])&&Math.abs(a[k]-b[k])<1e-10));
    });
}

export function canDeleteSequence(state,record,owned,pending) {
    const m=state.mechanism||{};
    return !!record && state.sequence_management?.schema===1 && state.connected===true && !pending &&
        (!state.owner || owned) && state.sequence?.status!=='RUNNING' && state.group?.state!==1 &&
        !['CAPTURE','REQUEST','REBOOT','VERIFY','ATTACH'].includes(state.recovery?.status) &&
        !['axis_state','servo_state','pose_state'].some(k=>m[k]>=1&&m[k]<=5);
}

export function initSequence(root,read,act,notify,owned,pending,erase,metadata=read) {
    const card=document.createElement('article');card.className='card remote-sequence';
    card.innerHTML=`<div class="card-head"><h2>机械联调</h2><span id="seq-state" class="badge muted">尚未运行</span></div>
      <p class="hint">同号同时、不同号依次。整组全部完成后进入下一号；两轴以反馈到位，舵机仅确认命令完成。PE14不参与。</p>
      <div class="remote-toolbar"><label>联调表名称<input id="seq-name" maxlength="48" value="mechanism_1"></label>
      <label>保存版本<select id="seq-versions"><option value="">暂无保存版本</option></select></label><button id="seq-load">读取版本</button><button id="seq-delete" class="danger">删除选中版本</button></div>
      <div id="seq-delete-review" class="seq-delete-review" hidden><span id="seq-delete-name"></span><button id="seq-delete-confirm" class="danger">确认删除这个版本</button><button id="seq-delete-cancel">取消</button></div>
      <p class="hint">两轴可逐行填写加速度档位0～255；旧版本未填写时沿用1。舵机不使用此档位，PE11继续使用原S曲线。</p>
      <div class="seq-table"><div class="seq-row seq-heading"><span>顺序</span><span>执行器</span><span>目标 mm / °</span><span>速度 RPM / °每秒</span><span>两轴加速度档</span><span></span></div><div id="seq-rows"></div></div>
      <div class="remote-toolbar"><button id="seq-add">添加动作</button><button id="seq-save">保存新版本</button>
      <button id="seq-run" class="primary">按顺序执行</button><button id="seq-together">全部同时执行</button></div>
      <p id="seq-message" class="hint">修改参数不会触发动作；执行使用选中且与当前参数一致的保存版本。</p>`;
    root.insertBefore(card,root.querySelector('.remote-poses'));
    const el=id=>card.querySelector(`#seq-${id}`);
    let rows=[],versionKey='',deleteTarget=null;
    try {const value=JSON.parse(localStorage.getItem('gc_remote_sequence_v1')||'null');if(value&&Array.isArray(value.rows)){rows=value.rows;el('name').value=value.name||'mechanism_1';}}catch{}
    function persist(){try{localStorage.setItem('gc_remote_sequence_v1',JSON.stringify({name:el('name').value,rows}));}catch{notify('本机参数草稿保存失败');}}
    function collect(){return rows.map(row=>({...row}));}
    function renderRows(){
        el('rows').replaceChildren(...rows.map((row,i)=>{
            const div=document.createElement('div');div.className='seq-row';
            div.innerHTML=`<input data-field="order" type="number" min="1" max="999" step="1" aria-label="顺序号">
              <select data-field="axis" aria-label="执行器">${actuators.map(a=>`<option value="${a.id}">${a.name} · ${a.pin}</option>`).join('')}</select>
              <input data-field="target" type="number" step="0.1" aria-label="目标">
              <input data-field="speed" type="number" step="1" min="1" aria-label="速度">
              <input data-field="accel" type="number" step="1" min="0" max="255" aria-label="两轴加速度档"><button data-remove aria-label="删除动作">删除</button>`;
            const linear=['lift','reach'].includes(row.axis);
            div.dataset.linear=linear?'1':'0';
            div.querySelector('[data-field="accel"]').disabled=!linear;
            for(const input of div.querySelectorAll('[data-field]')){
                input.value=input.dataset.field==='accel'?(linear?(row.accel??1):''):row[input.dataset.field];input.onchange=()=>{
                    if(input.dataset.field==='accel'&&!input.value.trim()){input.value=row.accel??1;notify('加速度不能为空，已保留原值');return;}
                    row[input.dataset.field]=input.value;
                    if(input.dataset.field==='axis'){row.speed=['lift','reach'].includes(row.axis)?5:45;renderRows();}
                    persist();draw();
                };
            }
            div.querySelector('[data-remove]').onclick=()=>{rows.splice(i,1);persist();renderRows();draw();};
            return div;
        }));
    }
    const selected=()=>(read().sequences||[]).find(r=>`${r.id}:${r.revision}`===el('versions').value);
    function values(){return sequenceRows(collect(),'ordered',read());}
    function matches(record){try{return record&&record.id===el('name').value.trim()&&sameRows(values(),record.rows);}catch{return false;}}
    function draw(){
        const state=read(),records=state.sequences||[],nextKey=JSON.stringify(records.map(r=>[r.id,r.revision,r.sha256]));
        if(nextKey!==versionKey){const old=el('versions').value;versionKey=nextKey;el('versions').replaceChildren(...records.map(r=>new Option(`${r.id} · v${r.revision}`,`${r.id}:${r.revision}`)));if(records.some(r=>`${r.id}:${r.revision}`===old))el('versions').value=old;else if(records.length)el('versions').value=`${records.at(-1).id}:${records.at(-1).revision}`;else el('versions').append(new Option('暂无保存版本',''));}
        const run=state.sequence?.status==='RUNNING'||state.group?.state===1,record=selected(),match=matches(record);
        const accelUnsupported=!firmwareSince(state.firmware, 42)&&rows.some(r=>['lift','reach'].includes(r.axis)&&Number(r.accel??1)!==1);
        const names={IDLE:'尚未运行',RUNNING:'执行中',DONE:'命令流程完成',FAILED:'已失败',CANCELLED:'已停止'};
        setText(el('state'),`${names[state.sequence?.status]||'尚未运行'}${state.sequence?.count?` · ${state.sequence.index}/${state.sequence.count}组`:''}`);
        for(const button of card.querySelectorAll('input,select,[data-remove],#seq-add,#seq-load'))button.disabled=run||
            (button.dataset.field==='accel'&&button.parentElement.dataset.linear!=='1');
        el('save').disabled=run||!allowed(state,owned(),pending(),'program_save');
        el('delete').disabled=!canDeleteSequence(metadata(),record,owned(),pending());
        el('delete').title=state.sequence_management?.schema!==1?'设备核心待更新，现有版本未删除':'仅删除选中的版本；参数草稿保留，历史归档可追溯';
        el('delete-confirm').disabled=!canDeleteSequence(metadata(),deleteTarget,owned(),pending());
        el('run').disabled=el('together').disabled=run||!match||accelUnsupported||!allowed(state,owned(),pending(),'program_run');
        setText(el('message'),state.sequence?.reason||(accelUnsupported?'可调轴加速度需0.3.42；当前编辑草稿已在本机保留。':!match?'参数未保存或与选中版本不同；保存/读取后再执行。':!firmwareSince(state.firmware, 32)?'联调执行需要0.3.32；当前仅编辑保存参数。':'已保存；同组并行不保证严格同步启动或同时到位，故障/停止后不自动续跑。'));
    }
    el('add').onclick=()=>{if(rows.length>=40)return;rows.push({order:rows.length?Math.max(...rows.map(r=>Number(r.order)||1))+1:1,axis:'lift',target:0,speed:5});persist();renderRows();draw();};
    el('name').onchange=()=>{persist();draw();};el('versions').onchange=()=>{deleteTarget=null;el('delete-review').hidden=true;draw();};
    el('load').onclick=()=>{const record=selected();if(!record)return;el('name').value=record.id;rows=record.rows.map(r=>({order:r.order,axis:r.axis,target:['lift','reach'].includes(r.axis)?r.target_m*1000:r.target_rad*180/Math.PI,speed:['lift','reach'].includes(r.axis)?r.rpm:r.speed_rad_s*180/Math.PI,accel:r.accel??1}));persist();renderRows();draw();};
    el('save').onclick=()=>{try{act('program_save',{id:el('name').value.trim(),rows:values()});}catch(error){notify(error.message);}};
    el('delete').onclick=()=>{
        const record=selected();if(!canDeleteSequence(metadata(),record,owned(),pending()))return;
        deleteTarget=record;setText(el('delete-name'),`删除 ${record.id} · v${record.revision}？历史归档，编辑内容保留。`);
        el('delete-review').hidden=false;draw();
    };
    el('delete-cancel').onclick=()=>{deleteTarget=null;el('delete-review').hidden=true;draw();};
    el('delete-confirm').onclick=async()=>{
        const record=deleteTarget;if(!canDeleteSequence(metadata(),record,owned(),pending()))return;
        try{await erase(record);deleteTarget=null;el('delete-review').hidden=true;}catch(error){notify(error.message);}finally{draw();}
    };
    const run=mode=>{try{sequenceRows(collect(),mode,read());const r=selected();if(!matches(r))throw Error('请先保存当前参数');act('program_run',{id:r.id,revision:r.revision,mode});}catch(error){notify(error.message);}};
    el('run').onclick=()=>run('ordered');el('together').onclick=()=>run('together');
    renderRows();draw();
    return {draw};
}
