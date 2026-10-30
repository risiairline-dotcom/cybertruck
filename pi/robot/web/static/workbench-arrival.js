import {firmwareSince} from './workbench-firmware.js?v=1';
import {setText} from './workbench-dom.js?v=1';

/* 沿用机械页控制权及保活；自动对中使用Pi本地订阅，不采信浏览器像素报文。 */
export function initArrival(root, current, owned, busy, command, message) {
    const panel=document.createElement('article');panel.className='card';
    panel.innerHTML=`<h2>粗加工2号工位对中</h2><p>目标：2号圆环中心对准画面中心。固定观察姿态、空爪；先验收四向小位移，再使用自动对中。</p>
      <label>观察姿态保存版本 <select data-field="version"></select></label>
      <label>观察检查点顺序号 <input data-field="order" type="number" min="1" step="1" value="1"></label>
      <label><input data-field="confirm" type="checkbox">相机安装未变，实物已处于该观察姿态，空爪且小位移范围无干涉</label>
      <p data-field="status"></p><p data-field="key-prompt" role="status" aria-live="polite" style="padding:12px;border:2px solid #d8a14d;border-radius:8px;font-weight:700"></p>
      <p>准备前请到PC0按键旁；出现下方倒计时后直接按键，无需等待聊天通知。60秒未按会取消，不自动重新准备。</p>
      <button data-field="start">准备手动标定 / 等待PC0</button>
      <button data-field="auto-start">准备自动对中 / 等待PC0</button>
      <button data-field="reload">读取已验收标定</button><p data-field="calibration"></p>
      <label>单步 / mm <input data-field="step" type="number" min="1" max="5" step="1" value="5"></label>
      <button data-axis="x" data-sign="1">车头＋X</button><button data-axis="x" data-sign="-1">车尾−X</button>
      <button data-axis="y" data-sign="1">左侧＋Y</button><button data-axis="y" data-sign="-1">右侧−Y</button>
      <button data-field="finish">结束微调 / 返回机械控制</button>
      <button data-field="auto">开始自动对中</button><button data-field="pause">暂停自动对中</button>
      <p data-field="records"></p>
      <p><button data-field="preview">开始圆环观察</button><button data-field="reference">保存当前候选参考图</button></p>
      <canvas data-field="image" width="1280" height="720" style="width:100%;max-width:960px"></canvas>
      <p data-field="vision">圆环观察尚未开始；参考图按当前观察姿态保存。</p>
      <p>手动模式按PC0后由方向按钮逐步执行；自动模式按PC0后会自行小步对中，稳定后交回机构，不执行抓放。相机姿态或配置改变需重新验收标定。停止实车沿用上方红色按钮。</p>`;
    root.append(panel);const el=name=>panel.querySelector(`[data-field="${name}"]`);let versionKey='';
    let preview=false,reading=false,observation=null,history=[],lastPhase='';
    const call=async(action,args)=>{try{await command(action,args);}catch(e){message(e.message);}};
    const start=mode=>{
        const s=current(),record=(s.sequences||[])[Number(el('version').value)];
        if(!record||!el('confirm').checked||!el('order').checkValidity())return;
        setText(el('key-prompt'),'正在核对并准备；看到“现在按PC0”和倒计时后再按。');
        call('alignment_start',{id:record.id,revision:record.revision,sha256:record.sha256,
            order:Number(el('order').value),mode,empty_path_confirmed:true,actual_pose_confirmed:true});
        el('confirm').checked=false;
    };
    el('start').onclick=()=>start('manual');
    el('auto-start').onclick=()=>start('auto');
    el('auto').onclick=()=>call('alignment_auto',{confirmed:true});
    el('pause').onclick=()=>call('alignment_pause',{});
    el('reload').onclick=()=>call('alignment_reload',{confirmed:true});
    for(const b of panel.querySelectorAll('[data-axis]'))b.onclick=()=>{
        if(el('step').value.trim()&&el('step').checkValidity())call('alignment_step',{
            axis:b.dataset.axis,mm:Number(el('step').value)*Number(b.dataset.sign)});
    };
    el('finish').onclick=()=>call('alignment_finish',{});
    el('preview').onclick=()=>{preview=!preview;history=[];observation=null;
        setText(el('preview'),preview?'暂停圆环观察':'开始圆环观察');};
    el('reference').onclick=async()=>{
        try{
            const s=current(),o=observation,record=(s.sequences||[])[Number(el('version').value)];
            if(!['IDLE','DONE','READY'].includes(s.arrival?.phase||'IDLE')||!s.base?.stopped||
                !record||!s.connected||!o||performance.now()-o.received>250-o.age_ms||history.length<5||
                history.at(-1).captured-history[0].captured<.5)throw Error('需要保存观察版本及至少5张新鲜、稳定的2号圆环图');
            const points=history.map(h=>h.center),center=[0,1].map(k=>points.reduce((a,p)=>a+p[k],0)/points.length);
            if(points.some(p=>Math.hypot(p[0]-center[0],p[1]-center[1])>3))throw Error('圆环还在移动，保持静止后再保存');
            const r=await fetch('/api/vision/samples',{method:'POST',headers:{'Content-Type':'application/json','X-Workbench':'1'},
                body:JSON.stringify({observation_id:o.observation_id,name:`粗加工2号候选_${Date.now()}`,stage:'rough',label:'empty',
                    binding:{kind:'sequence',id:record.id,revision:record.revision,sha256:record.sha256},
                    observation_order:Number(el('order').value),note:'固定观察姿态的候选停靠参考；尚未通过夹持中心或放置实物验证，不授权运动。'})});
            const value=await r.json();if(!r.ok)throw Error(value.detail||'参考保存失败');
            setText(el('vision'),`候选参考已保存：${value.id}；由助手读取并核对，尚未开放自动修正。`);
        }catch(e){message(e.message);}
    };
    setInterval(async()=>{
        if(!preview||reading||document.hidden||!panel.getClientRects().length)return;
        reading=true;
        try{
            const r=await fetch('/api/vision/live/observation?role=material&mode=ring'),o=await r.json();
            if(!r.ok)throw Error(o.detail||'圆环画面未就绪');
            o.received=performance.now();observation=o;
            const ring=o.items?.find(i=>i.position_id===2&&!i.frame_clipped&&!i.roi_clipped);
            if(!ring||history.at(-1)?.sequence===o.frame_id)history=[];
            else{
                if(history.length&&(history.at(-1).stream!==o.stream_id||history.at(-1).profile!==o.profile_sha256||
                    o.captured_s<=history.at(-1).captured||o.captured_s-history.at(-1).captured>.5))history=[];
                history.push({sequence:o.frame_id,captured:o.captured_s,center:ring.center_px,stream:o.stream_id,profile:o.profile_sha256});
                history=history.slice(-5);
            }
            const image=new Image();image.src=o.image;await image.decode();
            if(performance.now()-o.received+o.age_ms>250)throw Error('圆环图已过期，等待新帧');
            const [w,h]=o.image_size,canvas=el('image');
            if(canvas.width!==w||canvas.height!==h){canvas.width=w;canvas.height=h;}
            const ctx=canvas.getContext('2d');ctx.drawImage(image,0,0);ctx.strokeStyle='#e27b16';ctx.lineWidth=2;
            ctx.beginPath();ctx.moveTo(w/2-14,h/2);ctx.lineTo(w/2+14,h/2);
            ctx.moveTo(w/2,h/2-14);ctx.lineTo(w/2,h/2+14);ctx.stroke();
            if(ring){ctx.strokeStyle='#008b8b';ctx.lineWidth=3;ctx.beginPath();ctx.arc(...ring.center_px,10,0,Math.PI*2);ctx.stroke();}
            setText(el('vision'),ring?`2号圆环 (${ring.center_px.map(x=>x.toFixed(1)).join(', ')}) px；用于观察与采样`:'当前没有完整、可用的2号圆环');
        }catch(e){history=[];observation=null;setText(el('vision'),e.message);}
        finally{reading=false;}
    },200);
    function draw(){
        const s=current(),a=s.arrival||{},phase=a.phase||'IDLE',c=s.centering||{};
        if(phase!==lastPhase){lastPhase=phase;history=[];observation=null;}
        const key=JSON.stringify((s.sequences||[]).map(r=>[r.id,r.revision,r.sha256]));
        if(key!==versionKey){const old=el('version').value;versionKey=key;
            el('version').replaceChildren(...(s.sequences||[]).map((r,i)=>new Option(`${r.id} · v${r.revision}`,i)));
            if([...el('version').options].some(o=>o.value===old))el('version').value=old;
        }
        const compatible=firmwareSince(s.firmware, 45);
        const ready=s.connected&&owned()&&!busy()&&s.site_ready&&compatible&&
            !s.action_locked&&!s.mechanism?.fault&&!s.base?.fault;
        const keyState=s.physical_start||{},keyFresh=s.connected&&keyState.age_ms>=0&&keyState.age_ms<250;
        const countdown=Number.isFinite(keyState.remaining_ms)?`，剩余约${Math.floor(keyState.remaining_ms/1000)}秒`:'';
        setText(el('key-prompt'),phase==='WAIT_PC0'&&keyFresh&&[1,2,3].includes(keyState.phase)?
            `现在松开后按一次PC0${countdown}。${c.mode==='auto'?'短鸣后将自动小步对中。':'短鸣后等待方向按钮，不会自行移动。'}`:
            phase==='WAIT_PC0'?'正在刷新PC0状态，请先等待':phase==='READY'?
            c.mode==='auto'?'PC0已确认，自动对中准备中。':'PC0已确认；等待你按方向按钮执行单步。':
            phase==='FAILED'?`本轮已取消，未自动重试：${a.reason||'请检查停止记录'}`:
            ['PREPARE','COMMIT','CONFIG','KEY_PREPARE','KEY_REQUEST'].includes(phase)?'正在准备，先不要按PC0。':
            '尚未等待PC0；确认观察姿态后准备。');
        setText(el('status'),!compatible?'当前固件未接入微调，需要配套固件验收':
            `${phase} ${a.reason||''} ${c.status||''} ${c.reason||''}${phase==='MOVING'&&firmwareSince(s.firmware, 47)?'：小步移动与停车取样中，请等待本步完成':''}${phase==='WAIT_PC0'?c.mode==='auto'?'：现在按PC0后将自动小步对中':'：现在可按一次PC0，随后由方向按钮执行小位移':''}`);
        el('start').disabled=!ready||!s.manual_debug||!s.mechanism?.zero_valid||
            !['IDLE','DONE','FAILED'].includes(phase);
        el('auto-start').disabled=el('start').disabled||!c.calibration_available;
        el('auto').disabled=!ready||phase!=='READY'||!c.calibration_available||c.mode==='auto';
        el('pause').disabled=!ready||phase!=='READY'||c.mode!=='auto';
        el('reload').disabled=!ready||!['IDLE','DONE','FAILED'].includes(phase);
        for(const b of panel.querySelectorAll('[data-axis]'))b.disabled=!ready||phase!=='READY'||c.mode==='auto';
        el('finish').disabled=!ready||phase!=='READY';
        setText(el('calibration'),c.calibration_available?`已验收标定 ${c.calibration_id}；只适用于绑定的观察姿态及相机配置`:
            c.calibration_error||'本车方向/比例尚未实测；自动按钮保持关闭');
        const records=c.records||[];
        setText(el('records'),`已记录 ${c.record_count||0} 组停车前后观测；${records.slice(-3).map(r=>
            `#${r.id} ${r.axis.toUpperCase()} ${r.command_mm}mm，图像变化 ${r.delta_px.map(v=>v.toFixed(1)).join('/')}px`).join('；')}`+
            (s.centering_io?.[1]?`；${s.centering_io[1]}`:'')+
            (s.centering_records_writable===false?'；已有记录校验失败，原文件保留，新记录暂未写入':''));
        el('reference').disabled=!s.connected||!s.base?.stopped||!['IDLE','DONE','READY'].includes(phase);
    }
    panel.addEventListener('input',draw);return{draw};
}
