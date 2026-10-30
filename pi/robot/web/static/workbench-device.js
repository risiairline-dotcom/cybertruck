import {firmwareSince} from './workbench-firmware.js?v=1';
import {setText, setSteps} from './workbench-dom.js?v=1';
import {axisRateText} from './workbench-axis-profile.js?v=1';
import {initSequence} from './workbench-sequence.js?v=48.1';
import {stateName, samePose, rejectionText} from './workbench-mech-guide.js?v=3';
import {actuators, freshDevice, blockedReasons, allowed, moveArgs, manualTuning, armMaximum, gripMaximum, gripHold, canRecover} from './workbench-remote-state.js?v=48.1';
import {mechanismNotice} from './workbench-mech-status.js?v=13';
import {initArrival} from './workbench-arrival.js?v=6';

/* 仅访问实车网关；页面初始化只读取，不自动取得控制权或恢复动作。 */
export function initDevice(root) {
    const card = document.createElement('div');
    card.className = 'remote-mechanism';
    card.innerHTML = `
      <div class="section-intro remote-intro"><div><span class="eyebrow">LIVE / MECHANISM</span>
        <h2>机械远程控制</h2><p>两轴 · 三舵机 · 实车姿态。执行请求统一交给设备核心。</p></div>
        <span id="real-status" class="badge muted">读取设备状态</span></div>
      <article class="card remote-status"><div id="real-steps" class="remote-steps" aria-label="调试准备状态"></div><div class="remote-metrics">
        <div><span>固件</span><strong id="real-firmware">—</strong></div>
        <div><span>故障 · 底盘 / 机构</span><strong id="real-fault">未知</strong></div>
        <div><span>停车状态</span><strong id="real-stopped">未知</strong></div>
        <div><span>人工零点</span><strong id="real-zero-state">无有效反馈</strong></div>
        <div><span>PWM通道</span><strong id="real-pwm">未知</strong></div></div>
        <div class="remote-toolbar"><button id="real-prepare">正常停止后重新准备</button><button id="real-claim">获取控制权</button><button id="real-attach">接入机构反馈</button>
          <span id="real-owner">未持有控制权</span><button id="real-stop" class="danger">停止实车 · 保持夹爪</button><button id="real-recover">清除故障并重新连接</button></div>
        <details class="remote-recovery-help"><summary>停止与恢复说明</summary><p id="real-recovery-note" class="hint">恢复先保存故障记录，再关闭PWM并重启STM32。请先修复线路，确认机构静止、空爪且已有支撑；恢复后重新取得控制权、建零和记录参考，不自动续跑。</p></details>
        <div class="remote-toolbar"><span id="real-mode-state">姿态互锁模式</span><button id="real-mode">进入手动调试</button></div>
        <p class="hint">手动模式支持单轴、舵机和顺序 / 同时联调。输入只编辑目标，点击执行才动作。</p>
        <p id="real-progress" class="remote-progress" role="status" aria-live="polite"></p>
        <p id="real-blockers" class="remote-blockers"></p>
        <p id="real-message" role="status" aria-live="polite">输入只修改目标；点击执行才发送实车请求。</p>
        <label class="remote-confirm"><input id="real-confirm" type="checkbox">现场已核对，允许切换模式或下一项准备操作</label>
      </article>
      <div class="remote-control-grid"><article class="card"><div class="card-head"><h2>直线轴</h2><span class="badge muted">绝对位置 · mm</span></div>
        ${actuators.slice(0,2).map(a=>`<div class="remote-actuator"><div class="remote-actuator-title"><b>${a.name}</b><small>${a.pin}</small><span id="real-value-${a.id}">位置未知</span></div>
          <div class="remote-inputs"><label>目标 / mm<input id="real-target-${a.id}" type="number" min="${a.min}" max="${a.max}" step="0.5" value="0"></label>
            <label>速度上限 / RPM<input id="real-rpm-${a.id}" type="number" min="1" max="30" step="1" value="5"></label>
            <label>加速度档位<input id="real-accel-${a.id}" type="number" min="0" max="255" step="1" value="1"></label>
            <button id="real-enable-${a.id}">使能${a.name}</button><button id="real-move-${a.id}" class="primary">执行${a.name}</button></div>
            <p id="real-rate-${a.id}" class="hint"></p></div>`).join('')}
        <p class="hint">加速度档位1～255，越大起停越快；0表示直接启动。旧版固定1；可调档位需0.3.42。修改输入不会动作。</p>
        <div class="remote-foot"><button id="real-zero">确认最高 / 最近端并建零</button><span id="real-range-note">下降、伸出为正；手动行程配置待核对。</span></div>
      </article><article class="card"><div class="card-head"><h2>舵机</h2><span class="badge muted">命令角度 · °</span></div>
        <p id="real-angle-note" class="hint">转臂命令角不加零点偏置；手动扩展范围需0.3.31。</p>
        ${actuators.slice(2).map(a=>`<div class="remote-actuator"><div class="remote-actuator-title"><b>${a.name}</b><small>${a.pin}</small><span id="real-value-${a.id}">实际角度未知</span></div>
          <div class="remote-servo-row"><label>目标 / °<input id="real-target-${a.id}" type="number" min="${a.min}" max="${a.max}" step="1" placeholder="核对后填写"></label>
            <label>指令速度 / °每秒<input id="real-speed-${a.id}" type="number" min="1" max="360" step="1" value="45" disabled></label>
            <button id="real-move-${a.id}" class="primary">执行${a.name}</button></div>
          ${a.id==='tray'?`<div class="remote-presets">${[0,30,150,270].map(n=>`<button data-tray="${n}">${n}°${n===0?' 基准':''}</button>`).join('')}</div>`:''}
          ${a.id==='gripper'?'<p id="real-grip-note" class="hint">默认夹持98°；安全上限由当前固件决定。输入目标不动作。</p>':''}</div>`).join('')}
        <div id="real-confirmation" class="remote-confirmation" hidden><p id="real-confirmation-text"></p><button id="real-evidence">人工确认当前舵机姿态</button></div>
        <p class="hint">速度控制PWM指令变化，实际转速仍需实测。手动模式完成后可继续下一角度；点击执行夹爪即执行所填开合目标，无额外释放勾选。PE14暂不开放。</p>
      </article></div>
      <article class="card remote-setup"><div class="card-head"><h2>舵机回零初始化</h2><span id="real-home-state" class="badge muted">未初始化</span></div>
        <p class="hint">明确点击后，PE9、PE11、PE13同时输出命令0°。首次位置未知，不能保证按网页速度慢速回零。请确保三路0°及途中均无干涉、线缆有余量，夹爪空。</p>
        <div class="remote-toolbar"><button id="real-home">三个舵机同时回0°</button><button id="real-home-confirm">确认三路已回0°并建立参考</button></div>
        <p id="real-home-note" class="hint">先完成两轴人工建零并进入手动调试。实物回零并确认后，直接输入后续角度，无需再填写三个参考角。</p></article>
      <details class="card remote-setup"><summary>高级 · 手动记录当前舵机参考（可选）</summary>
        <p class="hint">必须与当前实物相符；默认值不能当作实际角度。记录参考不输出PWM，首次执行从该参考开始。</p>
        <div class="remote-reference-fields">${actuators.slice(2).map(a=>`<label>${a.name}当前参考 / °<input id="real-seed-${a.id}" type="number" min="${a.min}" max="${a.max}" step="1" placeholder="按实物填写"></label>`).join('')}</div>
        <button id="real-anchor">记录已核对的参考角</button>
      </details>
      <details class="card remote-poses"><summary>实车姿态 · 保存与调用</summary>
        <p class="hint">保存只记录目标版本，不运动。调用由板端执行安全过渡，逐步放行。</p>
        <div class="remote-pose-heading"><label>姿态编号<input id="real-pose-id" value="physical_pose_1" maxlength="48"></label>
          <label>已保存版本<select id="real-poses"><option value="">暂无保存版本</option></select></label><button id="real-pose-load">读取选中版本</button></div>
        <div class="remote-pose-fields">${actuators.map(a=>`<label>${a.name} / ${a.unit}<input id="real-pose-${a.id}" type="number" min="${a.min}" max="${a.max}" step="0.1" value="${a.initial}"></label>`).join('')}</div>
        <div class="remote-toolbar"><button id="real-teach">保存为新版本</button><button id="real-pose" class="primary">执行选中姿态</button><button id="real-next">放行当前步骤</button></div>
        <p id="real-version-note" class="hint"></p>
      </details>
      <details class="card remote-diagnostics"><summary>实时反馈与本页请求记录</summary><pre id="real-feedback">等待设备</pre><div id="real-history"></div></details>`;
    root.append(card);
    const el=id=>card.querySelector(`#real-${id}`);
    const client=`physical_${crypto.randomUUID().replaceAll('-','')}`;
    let token='', state={}, receivedAt=0, transportMs=0, pending='', pendingAction='', pendingLabel='', pendingAxis='';
    let polling=false, sending=false, poseKey='', identity='', heartbeatBusy=false, renewalUncertain=false;
    const acceptedEnables=new Set(), entries=[];
    // 仅保存输入草稿；不保存控制权、零点、许可或设备状态。
    const motionFields=['target-lift','rpm-lift','accel-lift','target-reach','rpm-reach','accel-reach'];
    try {
        const draft=JSON.parse(localStorage.getItem('gc_axis_motion_v1')||'{}');
        for(const key of motionFields)if(typeof draft[key]==='string'&&draft[key].trim()&&Number.isFinite(Number(draft[key])))el(key).value=draft[key];
    }catch{}
    function saveMotionDraft(){
        try{localStorage.setItem('gc_axis_motion_v1',JSON.stringify(Object.fromEntries(motionFields.map(key=>[key,el(key).value]))));}catch{}
    }
    const message=text=>{setText(el('message'),text);};
    function log(text) {
        entries.unshift(`${new Date().toLocaleTimeString('zh-CN',{hour12:false})} · ${text}`);
        entries.length=Math.min(entries.length,30);
        el('history').replaceChildren(...entries.map(text=>{const p=document.createElement('p');p.textContent=text;return p;}));
    }
    const current=()=>{
        const connected=freshDevice(state,receivedAt,performance.now(),transportMs);
        return {...state,connected,reason:state.connected&&!connected?
            '反馈新鲜度不足（含网络传输时间）；等待新反馈后再操作，不重发已接受的动作':state.reason};
    };
    const targets=()=>Object.fromEntries(actuators.map(a=>[a.id,number(`pose-${a.id}`)*(a.unit==='mm'?.001:Math.PI/180)]));
    const selected=()=>(state.poses||[]).find(p=>`${p.id}:${p.revision}`===el('poses').value);
    function number(id) {
        const input=el(id),value=input.valueAsNumber;
        if(!input.value.trim() || !Number.isFinite(value) || !input.checkValidity())throw Error('请填写范围内的明确目标');
        return value;
    }
    function label(action,args) {
        const name=actuators.find(a=>a.id===args.axis)?.name||'';
        return ({enable:`使能${name}`,move:`${name}目标`,zero:'人工建零',anchor:'记录舵机参考',attach:'接入反馈',
            home:'三舵机回零',home_confirm:'确认0度参考',recover:'清故障并重连',program_save:'保存联调表',program_run:'执行联调表',prepare:'重新准备',mode:'切换调试模式',confirm:'人工确认舵机姿态',teach:'保存姿态',pose:'调用姿态',next:'放行步骤'})[action]||action;
    }
    async function send(op,args={}) {
        const requestId=crypto.randomUUID().replaceAll('-',''),modifying=['mechanism','prepare','recover'].includes(op);
        if(op==='prepare')args.action='prepare';
        if(op==='recover')args.action='recover';
        if(modifying){pending=requestId;pendingAction=args.action;pendingLabel=label(args.action,args);pendingAxis=args.axis||'';}
        let response;
        try {
            response=await fetch('/api/device/command',{method:'POST',headers:{'Content-Type':'application/json','X-Workbench':'1'},
                body:JSON.stringify({op,args,client,token,request_id:requestId})});
            const value=await response.json();
            if(!response.ok){
                if(modifying && response.status<500)pending='';
                const error=Error(value.detail||'设备核心拒绝请求');
                error.status=response.status;
                throw error;
            }
            if(value.token)token=value.token;
            if(modifying && value.status!=='QUEUED'){pending='';log(`${pendingLabel}：${value.status==='SAVED_DRAFT'?'已保存新版本，未执行':value.status}`);}
            return value;
        }catch(error){
            if(!response && modifying)message(`${pendingLabel}结果未知，禁止重复发送；请核对反馈或停止`);
            throw error;
        }
    }
    async function act(action,args={}) {
        if(sending)return;
        sending=true;
        try {
            if(!allowed(current(),!!token,pending||renewalUncertain,action))throw Error('实时反馈、控制权、故障或互锁条件未满足');
            if(!['attach','teach'].includes(action) && !(state.manual_debug && ['move','program_save','program_run'].includes(action)) && !el('confirm').checked)throw Error('请勾选本次现场及目标确认');
            const value=await send('mechanism',{action,zero_version:state.mechanism?.zero_version||0,...args});
            message(value.status==='QUEUED'?`${label(action,args)}请求已排队，等待板端结果`:'姿态已保存，未发送动作');
            el('confirm').checked=false;
        }catch(error){message(error.message);log(error.message);}
        finally{sending=false;draw();}
    }
    async function stop(source='panel-stop-button') {
        token=''; // 停止请求等待期间不再续约；请求丢失仍由租约保护停车。
        renewalUncertain=false;
        try{await send('stop',{source});message('停止请求已发送；动作锁定，保留实时反馈，不自动恢复或张爪');log(`停止来源：${source}`);}
        catch(error){message(`停止未确认：${error.message}；请核对实物状态`);}
        finally{draw();}
    }
    const release=(source='page-release')=>{
        if(!token)return;
        const body={op:'release',client,token,request_id:crypto.randomUUID().replaceAll('-',''),args:{source}};token='';renewalUncertain=false;
        fetch('/api/device/command',{method:'POST',keepalive:true,headers:{'Content-Type':'application/json','X-Workbench':'1'},body:JSON.stringify(body)})
            .catch(()=>message('退出时停止未确认，请核对车辆；主控保护继续生效'));
        draw();
    };
    el('prepare').onclick=async()=>{
        if(sending)return;sending=true;
        try{
            if(!allowed(current(),!!token,pending,'prepare')||!el('confirm').checked)
                throw Error('请核对现场；仅正常停止、PWM关闭且无真实故障时允许重新准备');
            acceptedEnables.clear();
            await send('prepare',{confirmed:true,boot_id:state.boot_id,session_id:state.session_id,
                zero_version:state.mechanism.zero_version});
            el('confirm').checked=false;
            message('重新准备已排队；不会使能、建零或输出PWM，接受后请重新取得控制权');
        }catch(error){message(error.message);}
        finally{sending=false;draw();}
    };
    el('recover').onclick=async()=>{
        if(sending)return;sending=true;
        try{
            if(!canRecover(current(),!!token,pending)||!el('confirm').checked)
                throw Error('请先停止，修复线路并确认机构静止、空爪且已有支撑，再勾选现场确认');
            await send('recover',{confirmed:true,wiring_repaired:true,outputs_supported:true,
                boot_id:state.boot_id,session_id:state.session_id,zero_version:state.mechanism.zero_version});
            acceptedEnables.clear();el('confirm').checked=false;
            for(const a of actuators.slice(2))for(const kind of ['target','seed'])el(`${kind}-${a.id}`).value='';
            message('正在归档并恢复，等待新的启动身份；不自动使能、建零或执行目标');
        }catch(error){message(error.message);}
        finally{sending=false;draw();}
    };
    el('claim').onclick=async()=>{
        if(sending)return;sending=true;
        try{await send('claim');message('已取得控制权；尚未使能或动作');}catch(error){message(error.message);}
        finally{sending=false;draw();}
    };
    el('mode').onclick=()=>act('mode',{manual_debug:!state.manual_debug,confirmed:true});
    el('home').onclick=()=>act('home',{confirmed:true,safe_zero:true});
    el('home-confirm').onclick=()=>act('home_confirm',{confirmed:true,home_id:state.home?.id});
    el('attach').onclick=()=>act('attach');el('stop').onclick=()=>stop('panel-stop-button');
    for(const a of actuators){
        el(`move-${a.id}`).onclick=()=>{try{act('move',moveArgs(a.id,el(`target-${a.id}`).value,a.unit==='mm'?number(`rpm-${a.id}`):5,false,
            {manualTuning:manualTuning(current()),armMax:armMaximum(current()),gripMax:gripMaximum(current()),accel:a.unit==='mm'?number(`accel-${a.id}`):undefined,speedDegS:a.unit==='mm'?undefined:number(`speed-${a.id}`)}));}catch(error){message(error.message);}};
        if(a.unit==='mm')el(`enable-${a.id}`).onclick=()=>act('enable',{axis:a.id,confirmed:true});
    }
    for(const button of card.querySelectorAll('[data-tray]'))button.onclick=()=>{el('target-tray').value=button.dataset.tray;message(`料盘目标已填为${button.dataset.tray}°，尚未执行`);};
    el('zero').onclick=()=>act('zero',{confirmed:true});
    el('anchor').onclick=()=>{try{act('anchor',{confirmed:true,targets:Object.fromEntries(actuators.slice(2).map(a=>[a.id,number(`seed-${a.id}`)*Math.PI/180]))});}catch(error){message(error.message);}};
    el('evidence').onclick=()=>act('confirm',{confirmed:true});
    el('teach').onclick=()=>{try{act('teach',{pose_id:el('pose-id').value,targets:targets()});}catch(error){message(error.message);}};
    el('pose-load').onclick=()=>{
        const record=selected();if(!record)return;el('pose-id').value=record.id;
        for(const a of actuators)el(`pose-${a.id}`).value=+(record.targets[a.id]*(a.unit==='mm'?1000:180/Math.PI)).toFixed(5);
        message('已读取姿态目标，未发送动作');draw();
    };
    el('pose').onclick=()=>{
        const record=selected();if(!record)return;
        try{if(!samePose(targets(),record))throw Error('当前目标与选中版本不同，请先读取或保存新版本');
            act('pose',{pose_id:record.id,revision:record.revision,physical_taught:true,single_step:true,release_confirmed:record.targets.gripper<gripHold*Math.PI/180});}
        catch(error){message(error.message);}
    };
    el('next').onclick=()=>act('next',{action_id:state.mechanism?.pose_id,revision:state.mechanism?.revision,step_index:state.mechanism?.step_index});
    const sequenceUi=initSequence(card,current,act,message,()=>!!token,()=>pending||renewalUncertain||sending,async record=>{
        if(sending)return;sending=true;
        try{
            await send('sequence_delete',{id:record.id,revision:record.revision,sha256:record.sha256,confirmed:true});
            const text=`已删除 ${record.id} · v${record.revision}；历史已归档，当前编辑内容保留，未执行动作`;
            message(text);log(text);
        }finally{sending=false;draw();}
    },()=>({...state,connected:state.connected===true&&performance.now()-receivedAt<1500}));
    const arrivalUi=initArrival(card,current,()=>!!token,()=>sending||pending||renewalUncertain,
        async(action,args)=>{
            if(sending)return;sending=true;
            try{const result=await send('mechanism',{action,...args});message(result.status);}
            finally{sending=false;draw();}
        },message);
    card.addEventListener('input',()=>{saveMotionDraft();draw();});el('poses').onchange=draw;
    function draw(){
        sequenceUi.draw();
        arrivalUi.draw();
        const value=current(),m=value.mechanism;
        setText(el('status'),!value.connected?(state.connected?'反馈过期 · 动作暂停':'状态未知 / 连接中断'):value.action_locked?'动作已锁定 · 反馈在线':renewalUncertain?'控制权核对中 · 动作暂停':value.base?.fault||m?.fault?'故障锁定':!value.site_ready?'实车只读':'实车在线');
        setText(el('firmware'),value.firmware||'—');
        setText(el('fault'),value.connected?`${value.base?.fault??'—'} / ${m?.fault??'—'}`:'未知');
        setText(el('stopped'),value.connected?(value.manual_debug&&value.park_failure?.manual_speed_ignored?
            '停车事务已确认 · 手动免轮速判定':value.base?.stopped?'已确认':'未确认'):'未知');
        setText(el('zero-state'),!value.connected?'未知':m?.zero_valid?`有效 · v${m.zero_version}`:'无效 / 未建立');
        setText(el('pwm'),value.connected&&m?actuators.slice(2).filter((a,i)=>m.pwm_mask&(1<<i)).map(a=>a.pin).join('、')||'全部关闭':'未知');
        setSteps(el('steps'),[
            ['连接',value.connected],['控制权',value.connected&&!!token],
            ['机构反馈',value.connected&&m?.attached&&m.config_mask===3],
            ['人工建零',value.connected&&m?.zero_valid],['手动模式',value.connected&&value.manual_debug]
        ]);
        setText(el('owner'),token?'本页持有控制权':'未持有控制权');
        const notice=mechanismNotice(value,pending?pendingLabel:'');
        setText(el('progress'),notice.text);el('progress').dataset.kind=notice.kind;
        setText(el('blockers'),blockedReasons(value,!!token,pending||renewalUncertain).join('；')||'接口就绪；每项动作仍由板端复核');
        const permit=action=>!sending&&allowed(value,!!token,pending||renewalUncertain,action);
        el('prepare').disabled=!permit('prepare');
        el('home').disabled=!permit('home');el('home-confirm').disabled=!permit('home_confirm');
        setText(el('home-state'),({0:'未初始化',1:'等待输出',2:'已输出 · 等待稳定',3:'等待实物确认',4:'0°参考已确认',5:'已中止'})[value.home?.state]||'待核对');
        setText(el('home-note'),!firmwareSince(value.firmware, 34)?'一键回零需要GC-BASE-0.3.34；当前固件仍可使用高级手动参考。':value.home?.state===3?'请目视确认三个舵机均已到命令0°且停止，再勾选现场确认并建立参考；软件等待时间不代表真实到位。':value.home?.state===4?'本次0°参考已由人工确认，后续可直接输入目标角和指令速度。':'先完成两轴人工建零并进入手动调试；确认三路可安全回0度后才点击，不会自动回零。');
        el('recover').disabled=!canRecover(value,!!token,pending||renewalUncertain);
        setText(el('recovery-note'),value.recovery?.status && value.recovery.status!=='IDLE'
            ? `恢复 ${value.recovery.status}：${value.recovery.reason}`
            : '恢复先保存故障记录，再关闭PWM并重启STM32。请先修复线路，确认机构静止、空爪且已有支撑；恢复后重新取得控制权、建零和记录参考，不自动续跑。');
        el('prepare').title='仅恢复正常STOP后的准备能力；CAN故障、PWM保持或旧固件均不可使用';
        el('claim').disabled=!!token||!permit('claim');el('attach').disabled=!permit('attach');
        el('mode').disabled=!permit('mode');
        setText(el('mode'),value.manual_debug?'返回互锁模式':'进入手动调试');
        setText(el('mode-state'),value.manual_debug?'手动调试 · 人工核对避让':'姿态互锁模式');
        const tuning=manualTuning(value),armMax=armMaximum(value),gripMax=gripMaximum(value);
        setText(el('grip-note'),`默认夹持目标${gripHold}°，当前安全上限${gripMax}°；0°观察、60°松开。目标在范围内单独填写，完成后持续PWM保持，不是力矩设定。`);
        setText(el('angle-note'),tuning?`转臂手动0～${armMax}°，0°直接使用原PWM换算，不重定义当前姿态；实际角度仍需核对。${firmwareSince(value.firmware, 37)?' PE11转臂使用S曲线；PE9料盘、PE13夹爪按所填速度匀速步进。':['GC-BASE-0.3.35','GC-BASE-0.3.36'].includes(value.firmware)?' 当前三路采用S曲线平滑起停，所填速度为上限。':''}`:'转臂当前按83～180°校验；进入0.3.32手动模式后开放0～359°。');
        setText(el('range-note'),tuning?'手动取消49/50mm上限；0为最高/最近端，下降和伸出为正。'+
            (value.manual_policy?`到位容差${(value.manual_policy.endpoint_tolerance_um??value.manual_policy.tolerance_um)/1000}mm；按所填速度执行及驱动器原加减速，不额外拆段停车。`:
             value.axis_zero_policy?`目标≤${value.axis_zero_policy.target_max_um/1000}mm时，执行速度最高${value.axis_zero_policy.max_rpm}RPM；保存原值不变。`:''):
            '0.3.31手动模式支持扩展行程和速度；自动姿态沿用已验证范围。');
        el('zero').disabled=!permit('zero')||!!m?.zero_valid;el('anchor').disabled=!permit('anchor');
        for(const a of actuators){
            el(`move-${a.id}`).disabled=!permit('move');
            if(a.unit==='mm'){
                const editingManual=tuning||!value.connected;
                if(editingManual)el(`target-${a.id}`).removeAttribute('max');else el(`target-${a.id}`).max=a.max;
                el(`rpm-${a.id}`).max=editingManual?3000:30;
                const accel=el(`accel-${a.id}`).valueAsNumber;
                const accelBlocked=accel!==1 && !firmwareSince(value.firmware, 42);
                el(`move-${a.id}`).disabled=!permit('move')||accelBlocked;
                el(`move-${a.id}`).title=accelBlocked?'当前固件固定acc=1；可编辑草稿，烧录0.3.42后执行':'';
                const accepted=acceptedEnables.has(a.id);el(`enable-${a.id}`).disabled=!permit('enable')||accepted;
                setText(el(`enable-${a.id}`),accepted?'使能请求已接受':`使能${a.name}`);
                const position=m?.[a.id==='lift'?'lift_um':'reach_um'];
                const target=el(`target-${a.id}`).valueAsNumber;
                setText(el(`rate-${a.id}`),axisRateText(a.id,el(`rpm-${a.id}`).valueAsNumber,accel,
                    value.connected&&m?.zero_valid&&Number.isFinite(target)&&Number.isFinite(position)?Math.abs(target-position/1000):null));
                setText(el(`value-${a.id}`),value.connected&&m?.zero_valid?`${(position/1000).toFixed(2)} mm`:'位置未知');
            }else{
                if(a.id==='arm')for(const kind of ['target','seed']){
                    el(`${kind}-${a.id}`).min=tuning?0:a.min;
                    el(`${kind}-${a.id}`).max=tuning?armMax:a.max;
                }
                if(a.id==='gripper')for(const kind of ['target','seed','pose'])el(`${kind}-${a.id}`).max=gripMax;
                el(`speed-${a.id}`).disabled=!tuning;
                setText(el(`value-${a.id}`),value.connected&&m?.channel===actuators.slice(2).indexOf(a)+1?`命令 ${(m.target_mdeg/1000).toFixed(1)}° · 实际未知`:'实际角度未知');
            }
        }
        el('confirmation').hidden=!(value.connected&&m?.channel>=1&&m.channel<=3&&[5,9].includes(m?.servo_state));
        setText(el('confirmation-text'),m?`${actuators.slice(2)[m.channel-1]?.name||'舵机'}命令已完成。${m.servo_state===9?'可继续手动调试，实际姿态仍未确认；自动姿态前需核对实物。':'核对实物后勾选本次确认并点击下方按钮，板端超时保护仍生效。'}`:'');
        el('evidence').disabled=!permit('confirm');el('teach').disabled=!permit('teach');el('next').disabled=!permit('next');
        const record=selected();let matches=false;try{matches=!!record&&samePose(targets(),record);}catch{}
        el('pose-load').disabled=!record;el('pose').disabled=!permit('pose')||!matches;
        setText(el('version-note'),record?`${record.id} · v${record.revision} · ${record.sha256.slice(0,12)} · ${matches?'目标与版本一致':'当前目标不同，禁止调用'}`:'暂无实车姿态；保存不运动。');
        const safety=(value.safety_events||[]).slice(-5).map(e=>`${new Date(e.wall_s*1000).toLocaleTimeString('zh-CN',{hour12:false})} · ${e.source} · ${e.reason}`).join('\n');
        setText(el('feedback'),(value.connected&&m?`轴：${stateName('axis',m.axis_state)} / ${m.axis_reason}\n舵机：${stateName('servo',m.servo_state)} / ${m.servo_reason}\n姿态：${stateName('pose',m.pose_state)} · #${m.pose_id} v${m.revision} · ${m.step_index}/${m.step_count}\n反馈龄 ${value.state_age_ms}ms · 配置 ${m.config_mask}/3 · PWM位图 ${m.pwm_mask}\n启动 ${value.boot_id} · 会话 ${value.session_id}`:value.reason||'没有新鲜设备状态，不使用旧反馈授权动作')+(safety?`\n最近停止来源：\n${safety}`:''));
    }
    async function poll(){
        if(polling||document.hidden)return;polling=true;
        const requestedAt=performance.now();
        try{
            const response=await fetch('/api/device/state',{cache:'no-store',signal:AbortSignal.timeout(1200)});if(!response.ok)throw Error('设备状态读取失败');
            state=await response.json();receivedAt=performance.now();transportMs=receivedAt-requestedAt;
            const nextIdentity=`${state.boot_id}:${state.session_id}`;
            if(identity&&identity!==nextIdentity){token='';pending='';renewalUncertain=false;acceptedEnables.clear();el('confirm').checked=false;message('设备身份已变化，旧控制权和参考无效；不自动恢复');}
            identity=nextIdentity;const result=(state.results||[]).find(r=>r.request_id===pending);
            if(result){
                const text=result.result?`${pendingLabel}：${rejectionText(pendingAction,result.result,state)}`:`${pendingLabel}：板端已接受，完成以反馈为准`;
                message(text);log(text);if(pendingAction==='enable'&&!result.result)acceptedEnables.add(pendingAxis);
                if(!result.result && ['home','home_confirm'].includes(pendingAction)){
                    for(const a of actuators.slice(2))for(const kind of ['target','seed'])
                        el(`${kind}-${a.id}`).value=pendingAction==='home_confirm'?'0':'';
                }
                pending='';pendingAction='';pendingAxis='';
            }
            const key=JSON.stringify((state.poses||[]).map(p=>[p.id,p.revision]));
            if(key!==poseKey){
                poseKey=key;const old=el('poses').value;
                el('poses').replaceChildren(...(state.poses||[]).map(p=>new Option(`${p.id} · v${p.revision}`,`${p.id}:${p.revision}`)));
                if(!(state.poses||[]).length)el('poses').append(new Option('暂无保存版本',''));
                if((state.poses||[]).some(p=>`${p.id}:${p.revision}`===old))el('poses').value=old;
            }
        }catch(error){state={...state,connected:false,reason:error.message};} // 保留版本和输入；失联照常禁用动作。
        finally{
            polling=false;draw();
            window.dispatchEvent(new CustomEvent('gc-device-state',{detail:{
                connected:state.connected,firmware:state.firmware,domain:state.domain,
                boot_id:state.boot_id,session_id:state.session_id,state_age_ms:state.state_age_ms,
                base:state.base,mechanism:state.mechanism,action_locked:state.action_locked,reason:state.reason
            }}));
        }
    }
    setInterval(poll,100); // 每次轮询结束统一绘制，不重复运行另一套100ms绘制。
    setInterval(async()=>{
        if(!token||document.hidden||heartbeatBusy)return;heartbeatBusy=true;
        try{await send('renew');renewalUncertain=false;}
        catch(error){
            renewalUncertain=true;
            if(error.status===409 || error.status===403){token='';renewalUncertain=false;message('控制权已失效，动作不自动恢复；反馈继续显示');}
            else message('续约回复未确认，暂停新动作；在原租约期限内核对，不重发动作');
        }
        finally{heartbeatBusy=false;draw();}
    },400);
    window.addEventListener('pagehide',()=>release('page-exit'));
    window.addEventListener('offline',()=>release('browser-offline'));
    document.addEventListener('visibilitychange',()=>{if(document.hidden)release('page-hidden');});
    draw();poll();return{release,stop};
}
